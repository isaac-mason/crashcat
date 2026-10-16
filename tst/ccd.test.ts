import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    type ContactManifold,
    type Listener,
    MotionQuality,
    MotionType,
    type RigidBody,
    rigidBody,
    sphere,
    updateWorld,
} from '../src';
import { createTestWorld } from './helpers';

describe('CCD (Continuous Collision Detection) / MotionQuality.LINEAR_CAST', () => {
    test('should prevent fast bullet from tunneling through thin wall with LINEAR_CAST', () => {
        const { world, layers } = createTestWorld();

        // thin wall facing the travel axis (X): spans x in [-0.1, 0.1]
        const wallShape = box.create({ halfExtents: vec3.fromValues(0.1, 5, 5), density: 1000 });
        rigidBody.create(world, {
            shape: wallShape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // fast bullet, clearly separated from the wall (no overlap at frame start)
        const bulletShape = sphere.create({ radius: 0.2, density: 1000 });
        const bullet = rigidBody.create(world, {
            shape: bulletShape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-2, 0, 0),
        });
        bullet.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;

        // 300 m/s -> 5 units this frame, would reach x=3 (well past the wall) without CCD
        vec3.set(bullet.motionProperties.linearVelocity, 300, 0, 0);

        updateWorld(world, undefined, 1 / 60);

        // bullet advanced toward the wall but was stopped at its near face (~ -0.1 - radius), not tunneled through
        expect(bullet.position[0]).toBeGreaterThan(-2);
        expect(bullet.position[0]).toBeLessThan(0);
        // velocity was killed by the contact
        expect(bullet.motionProperties.linearVelocity[0]).toBeLessThan(300);
    });

    // A CCD contact is reported through the contact listener on the step the bodies actually
    // collide, independently of the discrete narrowphase. The bullet starts fully separated from
    // the wall, so no discrete contact can exist at the start of the step: the only way the listener
    // fires this step is via the CCD sweep that detects the mid-step crossing.
    test('should fire onContactAdded on the impact frame for a CCD contact', () => {
        const { world, layers } = createTestWorld();

        const wallShape = box.create({ halfExtents: vec3.fromValues(0.1, 5, 5), density: 1000 });
        const wall = rigidBody.create(world, {
            shape: wallShape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const bulletShape = sphere.create({ radius: 0.2, density: 1000 });
        const bullet = rigidBody.create(world, {
            shape: bulletShape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            // clearly separated: ~1.6 gap to the wall face, so no discrete contact can exist at frame start
            position: vec3.fromValues(-2, 0, 0),
        });
        bullet.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(bullet.motionProperties.linearVelocity, 300, 0, 0);

        let added = 0;
        const bodyIds: number[] = [];
        let normalX = 0;
        const listener: Listener = {
            onContactAdded: (bodyA: RigidBody, bodyB: RigidBody, manifold: ContactManifold) => {
                added++;
                bodyIds.push(bodyA.id, bodyB.id);
                normalX = manifold.worldSpaceNormal[0];
            },
        };

        // one step: the bullet crosses the wall mid-step, so the CCD sweep is the only thing that
        // can detect and report this contact
        updateWorld(world, listener, 1 / 60);

        expect(added).toBe(1);
        expect(bodyIds).toContain(bullet.id);
        expect(bodyIds).toContain(wall.id);
        // contact normal is along the travel axis (the wall face the bullet hit)
        expect(Math.abs(normalX)).toBeGreaterThan(0.9);
        // and it actually stopped at the wall rather than tunneling
        expect(bullet.position[0]).toBeLessThan(0);
    });

    // The CCD sweep reads the target's derived position, world aabb and broadphase leaf. Those are
    // derived from the centre of mass during velocity integration, which runs before the sweep, so a
    // target that moves this step has to be swept against its new pose. Deriving them only at the
    // end of the step instead lets the bullet tunnel through a plate that dropped into its path.
    test('should sweep against a target that moved into the path during the same step', () => {
        const { world, layers } = createTestWorld();

        // kinematic plate: bottom face starts at y = 0.245, above the bullet, and ends at y = 0.185,
        // just inside it. a 0.06 drop, far below any broadphase fattening.
        const plate = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.1, 0.5, 5), density: 1000 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.KINEMATIC,
            position: vec3.fromValues(0, 0.745, 0),
        });
        vec3.set(plate.motionProperties.linearVelocity, 0, -3.6, 0); // 0.06 per 1/60 step

        const bullet = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.2, density: 1000 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-2, 0, 0),
        });
        bullet.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(bullet.motionProperties.linearVelocity, 300, 0, 0); // 5 units this step

        updateWorld(world, undefined, 1 / 60);

        // the plate ended in the path
        expect(plate.position[1] - 0.5).toBeLessThan(0.2);
        // and the bullet stopped at it instead of reaching x = 3
        expect(bullet.position[0]).toBeLessThan(0);
    });
});

describe('CCD contact manifold', () => {
    test('a box cast flat onto a floor makes a manifold of its four bottom corners', () => {
        const { world, layers } = createTestWorld();

        const floor = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(10, 0.5, 10) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        // 3 units above the floor and 5 units of travel this step: the discrete narrowphase cannot see the floor
        const crate = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 3.5, 0),
        });
        crate.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(crate.motionProperties.linearVelocity, 0, -300, 0);

        const manifolds: { numContactPoints: number; points: number[] }[] = [];
        const listener: Listener = {
            onContactAdded: (bodyA, _bodyB, manifold) => {
                const points: number[] = [];
                for (let i = 0; i < manifold.numContactPoints; i++) {
                    // world points on the floor's side
                    const onFloor = bodyA === floor ? manifold.relativeContactPointsOnA : manifold.relativeContactPointsOnB;
                    points.push(
                        manifold.baseOffset[0] + onFloor[i * 3],
                        manifold.baseOffset[1] + onFloor[i * 3 + 1],
                        manifold.baseOffset[2] + onFloor[i * 3 + 2],
                    );
                }
                manifolds.push({ numContactPoints: manifold.numContactPoints, points });
            },
        };

        updateWorld(world, listener, 1 / 60);

        expect(manifolds.length).toBe(1);
        expect(manifolds[0].numContactPoints).toBe(4);
        for (let i = 0; i < 4; i++) {
            expect(Math.abs(manifolds[0].points[i * 3])).toBeCloseTo(0.5, 4);
            expect(manifolds[0].points[i * 3 + 1]).toBeCloseTo(0, 4);
            expect(Math.abs(manifolds[0].points[i * 3 + 2])).toBeCloseTo(0.5, 4);
        }

        // the impulse goes through the centre of the four corners, so the crate lands without spinning
        expect(crate.position[1]).toBeGreaterThan(0.4);
        expect(crate.position[1]).toBeLessThan(0.6);
        expect(vec3.length(crate.motionProperties.angularVelocity)).toBeLessThan(1e-6);
    });
});
