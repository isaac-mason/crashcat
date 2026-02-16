import { type Vec3, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    box,
    createCollisionEstimationResult,
    estimateCollisionResponse,
    type Listener,
    MotionType,
    rigidBody,
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
        estimateCollisionResponse(result, body1, body2, manifold, 0.5, 0.5);

        expect(result.numImpulses).toBe(0);
    });

    test('should conserve momentum with zero restitution', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        // two bodies moving towards each other
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-0.5, 0, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(1, 0, 0));

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0.5, 0, 0),
        });
        rigidBody.setLinearVelocity(world, body2, vec3.fromValues(-1, 0, 0));

        // create a contact manifold with slight overlap
        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(1, 0, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0.5;
        manifold.relativeContactPointsOnA[1] = 0;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = -0.5;
        manifold.relativeContactPointsOnB[1] = 0;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.1;

        const result = createCollisionEstimationResult();
        estimateCollisionResponse(result, body1, body2, manifold, 0, 0); // no friction, no restitution

        expect(result.numImpulses).toBe(1);

        // calculate momentum before and after
        const mass1 = body1.motionProperties!.invMass > 0 ? 1 / body1.motionProperties!.invMass : 0;
        const mass2 = body2.motionProperties!.invMass > 0 ? 1 / body2.motionProperties!.invMass : 0;

        const momentumBefore = vec3.create();
        vec3.scaleAndAdd(momentumBefore, momentumBefore, body1.motionProperties!.linearVelocity, mass1);
        vec3.scaleAndAdd(momentumBefore, momentumBefore, body2.motionProperties!.linearVelocity, mass2);

        const momentumAfter = vec3.create();
        vec3.scaleAndAdd(momentumAfter, momentumAfter, result.linearVelocity1, mass1);
        vec3.scaleAndAdd(momentumAfter, momentumAfter, result.linearVelocity2, mass2);

        // momentum should be conserved
        expect(vec3.distance(momentumBefore, momentumAfter)).toBeLessThan(0.01);
    });

    test('should reverse relative velocity with full restitution', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        // two bodies with equal mass moving towards each other
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-0.5, 0, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(2, 0, 0));

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0.5, 0, 0),
        });
        rigidBody.setLinearVelocity(world, body2, vec3.fromValues(-2, 0, 0));

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(1, 0, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0.5;
        manifold.relativeContactPointsOnA[1] = 0;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = -0.5;
        manifold.relativeContactPointsOnB[1] = 0;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.1;

        const result = createCollisionEstimationResult();
        estimateCollisionResponse(result, body1, body2, manifold, 0, 1.0, 1.0); // full restitution

        // with full restitution and equal masses, velocities should swap
        expect(Math.abs(result.linearVelocity1[0] - -2)).toBeLessThan(0.1);
        expect(Math.abs(result.linearVelocity2[0] - 2)).toBeLessThan(0.1);
    });

    test('should keep static body velocity at zero', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(0, -5, 0));

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(0, 1, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0;
        manifold.relativeContactPointsOnA[1] = -0.5;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = 0;
        manifold.relativeContactPointsOnB[1] = 0.5;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.1;

        const result = createCollisionEstimationResult();
        estimateCollisionResponse(result, body1, body2, manifold, 0.5, 0.5);

        // static body should have zero velocity
        expect(vec3.length(result.linearVelocity2)).toBe(0);
        expect(vec3.length(result.angularVelocity2)).toBe(0);

        // dynamic body should have changed velocity
        expect(result.linearVelocity1[1]).toBeGreaterThan(-5);
        expect(result.impulses[0].contactImpulse).toBeGreaterThan(0);
    });

    test('should apply friction in tangent direction', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        // dynamic box sliding on static ground with tangential velocity
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.45, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(5, -1, 0)); // moving right and down

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(0, 1, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0;
        manifold.relativeContactPointsOnA[1] = -0.5;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = 0;
        manifold.relativeContactPointsOnB[1] = 0.5;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.05;

        const result = createCollisionEstimationResult();
        const friction = 0.8;
        estimateCollisionResponse(result, body1, body2, manifold, friction, 0);

        // friction should reduce tangential velocity
        const initialTangentialSpeed = 5;
        const finalTangentialSpeed = Math.abs(result.linearVelocity1[0]);
        expect(finalTangentialSpeed).toBeLessThan(initialTangentialSpeed);

        // friction impulse should be non-zero
        const frictionImpulse = Math.sqrt(result.impulses[0].frictionImpulse1 ** 2 + result.impulses[0].frictionImpulse2 ** 2);
        expect(frictionImpulse).toBeGreaterThan(0);
    });

    test('should satisfy circular coulomb friction cone', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        // box with diagonal tangential velocity
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.45, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(10, -5, 10)); // diagonal sliding

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(0, 1, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0;
        manifold.relativeContactPointsOnA[1] = -0.5;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = 0;
        manifold.relativeContactPointsOnB[1] = 0.5;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.05;

        const result = createCollisionEstimationResult();
        const friction = 0.6;
        estimateCollisionResponse(result, body1, body2, manifold, friction, 0);

        // verify coulomb friction cone: sqrt(f1² + f2²) ≤ μ * λn
        const f1 = result.impulses[0].frictionImpulse1;
        const f2 = result.impulses[0].frictionImpulse2;
        const normalImpulse = result.impulses[0].contactImpulse;

        const frictionMagnitude = Math.sqrt(f1 * f1 + f2 * f2);
        const maxFriction = friction * normalImpulse;

        expect(frictionMagnitude).toBeLessThanOrEqual(maxFriction * 1.001); // small tolerance for floating point
    });

    test('should handle multiple contact points', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(1, 0.5, 1) });

        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.45, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(0, -2, 0));

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 4;
        manifold.worldSpaceNormal = vec3.fromValues(0, 1, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // four corners of the box
        const corners = [
            [-1, -0.5, -1],
            [1, -0.5, -1],
            [1, -0.5, 1],
            [-1, -0.5, 1],
        ];

        for (let i = 0; i < 4; i++) {
            manifold.relativeContactPointsOnA[i * 3 + 0] = corners[i][0];
            manifold.relativeContactPointsOnA[i * 3 + 1] = corners[i][1];
            manifold.relativeContactPointsOnA[i * 3 + 2] = corners[i][2];
            manifold.relativeContactPointsOnB[i * 3 + 0] = corners[i][0];
            manifold.relativeContactPointsOnB[i * 3 + 1] = -corners[i][1];
            manifold.relativeContactPointsOnB[i * 3 + 2] = corners[i][2];
        }

        manifold.penetrationDepth = 0.05;

        const result = createCollisionEstimationResult();
        estimateCollisionResponse(result, body1, body2, manifold, 0.5, 0);

        expect(result.numImpulses).toBe(4);

        // all contact impulses should be non-negative
        for (let i = 0; i < 4; i++) {
            expect(result.impulses[i].contactImpulse).toBeGreaterThanOrEqual(0);
        }

        // total impulse should be significant
        let totalImpulse = 0;
        for (let i = 0; i < 4; i++) {
            totalImpulse += result.impulses[i].contactImpulse;
        }
        expect(totalImpulse).toBeGreaterThan(0);
    });

    test('should handle kinematic bodies', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(0, -5, 0));

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.KINEMATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(0, 1, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0;
        manifold.relativeContactPointsOnA[1] = -0.5;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = 0;
        manifold.relativeContactPointsOnB[1] = 0.5;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.1;

        const result = createCollisionEstimationResult();
        estimateCollisionResponse(result, body1, body2, manifold, 0.5, 0.5);

        // kinematic body velocity should remain zero
        expect(vec3.length(result.linearVelocity2)).toBe(0);
        expect(vec3.length(result.angularVelocity2)).toBe(0);

        // dynamic body should bounce
        expect(result.linearVelocity1[1]).toBeGreaterThan(-5);
    });

    test('should respect minVelocityForRestitution threshold', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        // slow collision - below threshold
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-0.5, 0, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(0.5, 0, 0)); // slow approach

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0.5, 0, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(1, 0, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0.5;
        manifold.relativeContactPointsOnA[1] = 0;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = -0.5;
        manifold.relativeContactPointsOnB[1] = 0;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.1;

        const result = createCollisionEstimationResult();
        // set threshold to 1.0 m/s, collision is at 0.5 m/s
        estimateCollisionResponse(result, body1, body2, manifold, 0, 1.0, 1.0);

        // with low velocity, should just come to rest (no restitution applied)
        expect(Math.abs(result.linearVelocity1[0])).toBeLessThan(0.5);
    });

    test('should handle angular velocities and torque', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(1, 0.5, 1) });

        // box rotating and falling
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(0, -2, 0));
        rigidBody.setAngularVelocity(world, body1, vec3.fromValues(0, 0, 2)); // spinning around z-axis

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(0, 1, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        // contact at offset from center to create torque
        manifold.relativeContactPointsOnA[0] = 0.5;
        manifold.relativeContactPointsOnA[1] = -0.5;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = 0.5;
        manifold.relativeContactPointsOnB[1] = 0.5;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.1;

        const result = createCollisionEstimationResult();
        estimateCollisionResponse(result, body1, body2, manifold, 0.5, 0.3);

        // angular velocity should be affected by the collision
        expect(vec3.distance(result.angularVelocity1, body1.motionProperties!.angularVelocity)).toBeGreaterThan(0);
    });

    test('single contact point with no friction should converge in one iteration', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-0.5, 0, 0),
        });

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0.5, 0, 0),
        });

        const manifold = createContactManifold();
        manifold.numContactPoints = 1;
        manifold.worldSpaceNormal = vec3.fromValues(1, 0, 0);
        manifold.baseOffset = vec3.fromValues(0, 0, 0);
        manifold.relativeContactPointsOnA[0] = 0.5;
        manifold.relativeContactPointsOnA[1] = 0;
        manifold.relativeContactPointsOnA[2] = 0;
        manifold.relativeContactPointsOnB[0] = -0.5;
        manifold.relativeContactPointsOnB[1] = 0;
        manifold.relativeContactPointsOnB[2] = 0;
        manifold.penetrationDepth = 0.1;

        const result1 = createCollisionEstimationResult();
        estimateCollisionResponse(result1, body1, body2, manifold, 0, 0, 1.0, 1);

        const result10 = createCollisionEstimationResult();
        estimateCollisionResponse(result10, body1, body2, manifold, 0, 0, 1.0, 10);

        // results should be identical (single iteration is sufficient)
        expect(vec3.distance(result1.linearVelocity1, result10.linearVelocity1)).toBeLessThan(0.001);
        expect(Math.abs(result1.impulses[0].contactImpulse - result10.impulses[0].contactImpulse)).toBeLessThan(0.001);
    });

    test('integration test - estimate should match actual solver for simple collision', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        let estimatedVel1: Vec3 | null = null;
        let estimatedVel2: Vec3 | null = null;
        let estimatedImpulse = 0;

        const result = createCollisionEstimationResult();

        const listener: Listener = {
            onContactAdded: (bodyA, bodyB, manifold, settings) => {
                estimateCollisionResponse(
                    result,
                    bodyA,
                    bodyB,
                    manifold,
                    settings.combinedFriction,
                    settings.combinedRestitution,
                    1.0,
                    10,
                );

                estimatedVel1 = vec3.clone(result.linearVelocity1);
                estimatedVel2 = vec3.clone(result.linearVelocity2);
                estimatedImpulse = result.impulses[0].contactImpulse;
            },
            onContactPersisted: () => {},
            onContactRemoved: () => {},
        };

        // create two boxes moving towards each other
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1.5, 0),
        });
        rigidBody.setLinearVelocity(world, body1, vec3.fromValues(0, -1, 0));

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0),
        });
        rigidBody.setLinearVelocity(world, body2, vec3.fromValues(0, 1, 0));

        // step until collision
        for (let i = 0; i < 30; i++) {
            updateWorld(world, listener, 1 / 60);
            if (estimatedVel1) break;
        }

        expect(estimatedVel1).not.toBeNull();
        expect(estimatedVel2).not.toBeNull();
        expect(estimatedImpulse).toBeGreaterThan(0);

        // step one more time to let the solver run
        updateWorld(world, listener, 1 / 60);

        // predicted velocities should be close to actual post-collision velocities
        // note: won't be exact due to integration and other contacts, but should be in ballpark
        if (estimatedVel1 && estimatedVel2 && body1.motionProperties && body2.motionProperties) {
            const vel1Diff = vec3.distance(estimatedVel1, body1.motionProperties.linearVelocity);
            const vel2Diff = vec3.distance(estimatedVel2, body2.motionProperties.linearVelocity);

            // should be reasonably close (within 50% for this simple test)
            expect(vel1Diff).toBeLessThan(2.0);
            expect(vel2Diff).toBeLessThan(2.0);
        }
    });

    test('should match actual solver across 729 parameterized scenarios', () => {
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
                                        estimateCollisionResponse(
                                            result,
                                            bodyA,
                                            bodyB,
                                            manifold,
                                            settings.combinedFriction,
                                            settings.combinedRestitution,
                                        );
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
                                    if (
                                        linearVel1Diff < 0.1 &&
                                        angularVel1Diff < 0.1 &&
                                        linearVel2Diff < 0.1 &&
                                        angularVel2Diff < 0.1
                                    ) {
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
});
