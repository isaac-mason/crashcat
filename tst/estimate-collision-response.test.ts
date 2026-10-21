import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    createCollisionEstimationResult,
    createContactSettings,
    estimateCollisionResponse,
    type Listener,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
} from '../src';
import { createContactManifold } from '../src/manifold/manifold';
import { createTestWorld } from './helpers';

describe('estimateCollisionResponse', () => {
    test('should predict zero impulse for non-colliding bodies', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

        // create two bodies far apart
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(10, 0, 0),
        });

        // no contact manifold, so we create an empty one
        const manifold = createContactManifold();
        manifold.numContactPoints = 0;

        const result = createCollisionEstimationResult();
        const settings = createContactSettings();
        settings.combinedFriction = 0.5;
        settings.combinedRestitution = 0.5;
        estimateCollisionResponse(result, body1, body2, manifold, settings);

        expect(result.numImpulses).toBe(0);
    });

    test('should match actual solver across 729 parameterized scenarios', () => {
        // following JoltPhysics/UnitTests/Physics/EstimateCollisionResponseTest.cpp exactly
        const box1HalfExtents = vec3.fromValues(0.1, 1, 2);
        const box2HalfExtents = vec3.fromValues(0.2, 3, 4);
        const baseOffset = vec3.fromValues(1, 2, 3);
        const epsilon = 0.0001;

        const motionTypes = [MotionType.STATIC, MotionType.KINEMATIC, MotionType.DYNAMIC];
        const restitutionValues = [0.0, 0.3, 1.0];
        const frictionValues = [0.0, 0.3, 1.0];
        const yPositions = [0.0, 0.5, box2HalfExtents[1]];
        const zPositions = [0.0, 0.5, box2HalfExtents[2]];
        const angularVelocities = [0.0, -1.0, 1.0];

        let testCount = 0;
        let passCount = 0;

        for (const mt of motionTypes) {
            for (const restitution of restitutionValues) {
                for (const friction of frictionValues) {
                    for (const y of yPositions) {
                        for (const z of zPositions) {
                            for (const w of angularVelocities) {
                                testCount++;

                                const { world, layers } = createTestWorld();

                                const result = createCollisionEstimationResult();

                                const listener: Listener = {
                                    onContactAdded: (bodyA, bodyB, manifold, settings) => {
                                        estimateCollisionResponse(result, bodyA, bodyB, manifold, settings);
                                    },
                                };

                                const box1Shape = box.create({ halfExtents: box1HalfExtents });
                                const box2Shape = box.create({ halfExtents: box2HalfExtents });

                                const body1 = rigidBody.create(world, {
                                    shape: box1Shape,
                                    objectLayer: layers.OBJECT_LAYER_MOVING,
                                    motionType: MotionType.DYNAMIC,
                                    position: vec3.clone(baseOffset),
                                    friction,
                                    restitution,
                                });
                                rigidBody.setLinearVelocity(world, body1, vec3.fromValues(1, 1, 0));
                                rigidBody.setAngularVelocity(world, body1, vec3.fromValues(0, w, 0));

                                const box2Position = vec3.create();
                                vec3.copy(box2Position, baseOffset);
                                box2Position[0] += box1HalfExtents[0] + box2HalfExtents[0] - epsilon;
                                box2Position[1] += y;
                                box2Position[2] += z;

                                const body2 = rigidBody.create(world, {
                                    shape: box2Shape,
                                    objectLayer:
                                        mt === MotionType.STATIC ? layers.OBJECT_LAYER_NOT_MOVING : layers.OBJECT_LAYER_MOVING,
                                    motionType: mt,
                                    position: box2Position,
                                    friction,
                                    restitution,
                                });

                                if (mt !== MotionType.STATIC) {
                                    rigidBody.setLinearVelocity(world, body2, vec3.fromValues(-1, 0, 0));
                                }

                                // step the simulation
                                updateWorld(world, listener, 1 / 60);

                                // check that the predicted velocities match the actual velocities
                                if (result.numImpulses > 0) {
                                    const linearVel1Diff = vec3.distance(
                                        result.linearVelocity1,
                                        body1.motionProperties!.linearVelocity,
                                    );
                                    const angularVel1Diff = vec3.distance(
                                        result.angularVelocity1,
                                        body1.motionProperties!.angularVelocity,
                                    );
                                    const linearVel2Diff = vec3.distance(
                                        result.linearVelocity2,
                                        body2.motionProperties!.linearVelocity,
                                    );
                                    const angularVel2Diff = vec3.distance(
                                        result.angularVelocity2,
                                        body2.motionProperties!.angularVelocity,
                                    );

                                    // velocities should match closely (within 0.1 m/s or rad/s)
                                    const threshold = 0.1;
                                    const passed =
                                        linearVel1Diff < threshold &&
                                        angularVel1Diff < threshold &&
                                        linearVel2Diff < threshold &&
                                        angularVel2Diff < threshold;

                                    if (passed) {
                                        passCount++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // should pass the vast majority of cases
        expect(passCount).toBeGreaterThan(testCount * 0.95);
    });

    test('follows the contact settings: sensors, mass and inertia scales, surface velocity', () => {
        const { world, layers } = createTestWorld();
        const radius = 0.5;
        const v0 = 4;
        // a ball resting its lowest point on a static box's top face at y = 0, the one contact at the origin
        const ball = rigidBody.create(world, {
            shape: sphere.create({ radius, density: 1000 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, radius, 0),
        });
        const ground = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(5, 1, 5) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -1, 0),
        });
        vec3.set(ball.motionProperties.linearVelocity, 0, -v0, 0);
        const mass = 1 / ball.motionProperties.invMass;
        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        vec3.set(manifold.worldSpaceNormal, 0, -1, 0);
        const result = createCollisionEstimationResult();
        const materials = (friction: number, restitution: number) => {
            const settings = createContactSettings();
            settings.combinedFriction = friction;
            settings.combinedRestitution = restitution;
            return settings;
        };

        // a sensor reports the contact but has no response
        const sensor = materials(0.5, 0.5);
        sensor.isSensor = true;
        estimateCollisionResponse(result, ball, ground, manifold, sensor);
        expect(result.numImpulses).toBe(1);
        expect(result.contactImpulse[0]).toBe(0);
        expect(result.linearVelocity1[1]).toBe(-v0);

        // the ball made immovable for this contact, against a static ground: nothing can respond
        const immovable = materials(0.5, 0);
        immovable.invMassScale1 = 0;
        immovable.invInertiaScale1 = 0;
        estimateCollisionResponse(result, ball, ground, manifold, immovable);
        expect(result.contactImpulse[0]).toBe(0);
        expect(result.linearVelocity1[1]).toBe(-v0);

        // twice the inverse mass: half the impulse stops it
        const lighter = materials(0, 0);
        lighter.invMassScale1 = 2;
        estimateCollisionResponse(result, ball, ground, manifold, lighter);
        expect(result.contactImpulse[0]).toBeCloseTo(mass * v0 * 0.5, 3);
        expect(result.linearVelocity1[1]).toBeCloseTo(0, 5);

        // a conveyor: the ground's surface moving along -x, relative to the ball, drags the ball along with it
        const conveyor = materials(1, 0);
        vec3.set(conveyor.relativeLinearSurfaceVelocity, -2, 0, 0);
        estimateCollisionResponse(result, ball, ground, manifold, conveyor);
        expect(result.linearVelocity1[0]).toBeLessThan(-0.5);
    });
});
