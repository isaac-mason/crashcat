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

describe('CCD resolution', () => {
    const addBullet = (
        world: ReturnType<typeof createTestWorld>['world'],
        layer: number,
        radius: number,
        x: number,
        y: number,
        velocityX: number,
        materialId = -1,
    ) => {
        const bullet = rigidBody.create(world, {
            shape: sphere.create({ radius, materialId }),
            objectLayer: layer,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(x, y, 0),
        });
        bullet.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(bullet.motionProperties.linearVelocity, velocityX, 0, 0);
        return bullet;
    };

    // a sweeps b (the lower index does the pair) and b finds nothing, so a's hit is what shortens b's travel
    test('a ccd body another ccd body hits earlier only travels to the impact', () => {
        const { world, layers } = createTestWorld();
        const a = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.2, -3, 0, 300);
        const b = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.2, 3, 0, -300);

        updateWorld(world, undefined, 1 / 60);

        // they meet at the origin, 5.6 of the 10 units closing: b stops at its side, not 5 units on through a
        expect(b.position[0]).toBeCloseTo(0.2, 2);
        expect(a.position[0]).toBeLessThan(b.position[0]);
        // head on with equal masses and no restitution, the impulse stops both
        expect(Math.abs(a.motionProperties.linearVelocity[0])).toBeLessThan(1);
        expect(Math.abs(b.motionProperties.linearVelocity[0])).toBeLessThan(1);
    });

    test('a ccd body whose target stopped earlier against something else takes no response from it', () => {
        const { world, layers } = createTestWorld();
        // a chases b; b reaches the wall at 0.28 of the step, a reaches b's swept path at 0.35 and the wall at 0.43
        const a = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.5, -4.05, 0, 480);
        const b = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.5, -2, 0, 300);
        const wall = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.1, 5, 5) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        updateWorld(world, undefined, 1 / 60);

        expect(world.ccd.ccdBodies[b.ccdBodyIndex].hitBodyIndex).toBe(wall.index);
        expect(world.ccd.ccdBodies[a.ccdBodyIndex].hitBodyIndex).toBe(b.index);
        // b resolved first and stopped at the wall; a's hit on b's swept path never happened, so b is not pushed on
        expect(b.position[0]).toBeCloseTo(-0.6, 1);
        expect(Math.abs(b.motionProperties.linearVelocity[0])).toBeLessThan(1);
        expect(a.motionProperties.linearVelocity[0]).toBeGreaterThan(470);
    });

    test('a kinematic linear cast target has already moved, so the contact is not shifted by its motion', () => {
        const { world, layers } = createTestWorld();
        // the wall slides 1 unit along +y this step; it is kinematic, so it moves during integration like any other
        const wall = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.1, 5, 5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.KINEMATIC,
            position: vec3.fromValues(0, 0, 0),
        });
        wall.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(wall.motionProperties.linearVelocity, 0, 60, 0);
        addBullet(world, layers.OBJECT_LAYER_MOVING, 0.2, -2, 0, 300);

        const pointsOnWallY: number[] = [];
        const listener: Listener = {
            onContactAdded: (bodyA, _bodyB, manifold) => {
                const onWall = bodyA === wall ? manifold.relativeContactPointsOnA : manifold.relativeContactPointsOnB;
                for (let i = 0; i < manifold.numContactPoints; i++) {
                    pointsOnWallY.push(manifold.baseOffset[1] + onWall[i * 3 + 1]);
                }
            },
        };

        updateWorld(world, listener, 1 / 60);

        // the sphere meets the wall's face on its own line of travel, y = 0, not 0.34 units up the wall
        expect(pointsOnWallY.length).toBe(1);
        expect(pointsOnWallY[0]).toBeCloseTo(0, 2);
    });

    test('a ccd contact reported with the bodies swapped has its normal negated and its materials swapped', () => {
        const { world, layers } = createTestWorld();
        // the wall is created first, so it has the lower id and comes first in the report
        const wall = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.1, 5, 5), materialId: 7 }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });
        const bullet = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.2, -2, 0, 300, 3);

        const reports: { bodyA: RigidBody; normalX: number; materialIdA: number; materialIdB: number }[] = [];
        const listener: Listener = {
            onContactAdded: (bodyA, _bodyB, manifold) => {
                reports.push({
                    bodyA,
                    normalX: manifold.worldSpaceNormal[0],
                    materialIdA: manifold.materialIdA,
                    materialIdB: manifold.materialIdB,
                });
            },
        };

        updateWorld(world, listener, 1 / 60);

        expect(reports.length).toBe(1);
        expect(reports[0].bodyA).toBe(wall);
        // from a (the wall) to b (the bullet): back along -x
        expect(reports[0].normalX).toBeLessThan(-0.9);
        expect(reports[0].materialIdA).toBe(7);
        expect(reports[0].materialIdB).toBe(3);
        expect(bullet.position[0]).toBeLessThan(0);
    });

    // a grazes b, which is discrete and has already moved this step, up and away from a's path (0.045, inside its
    // broadphase leaf). at the time of impact the normal is nearly along +y, where b moves away faster than a follows,
    // so the bodies are separating. jolt solves the ccd contact anyway, with an unbounded normal impulse, so they
    // leave with no relative normal velocity
    test('a ccd contact whose bodies are separating is still solved', () => {
        const { world, layers } = createTestWorld();
        // a small a keeps its allowed penetration (a quarter of its radius) small, so the grazing hit is kept
        const a = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.02, 0, 0, 15);
        const b = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.5 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0.28, 0.47, 0),
        });
        vec3.set(b.motionProperties.linearVelocity, 0, 2.7, 0);

        updateWorld(world, undefined, 1 / 60);

        const ccdBodyA = world.ccd.ccdBodies[a.ccdBodyIndex];
        expect(ccdBodyA.hitBodyIndex).toBe(b.index);
        expect(ccdBodyA.contactNormal[1]).toBeGreaterThan(0.95);
        const relativeVelocity = vec3.subtract(
            vec3.create(),
            b.motionProperties.linearVelocity,
            a.motionProperties.linearVelocity,
        );
        expect(vec3.dot(relativeVelocity, ccdBodyA.contactNormal)).toBeCloseTo(0, 3);
    });

    // b is linear cast but moves 0.04 this step, under its 0.05 threshold, so it has already moved. jolt still counts
    // its motion against a's sweep: a sweeps 1 - 0.04 relative to b and stops short of b's face, which the full 1
    // would have reached (at 0.97, inside the step even with a's 0.02 allowed penetration)
    test('a linear cast target below its threshold still has its motion taken off the sweep', () => {
        const { world, layers } = createTestWorld();
        const a = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.1, 0, 0, 60);
        const b = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(2.03, 0, 0),
        });
        b.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(b.motionProperties.linearVelocity, 2.4, 0, 0);

        updateWorld(world, undefined, 1 / 60);

        // b's face ends at 1.07, a's leading point at 1.1 after a full step and 1.06 after the relative one
        expect(b.ccdBodyIndex).toBe(-1);
        expect(world.ccd.ccdBodies[a.ccdBodyIndex].hitBodyIndex).toBe(-1);
        expect(a.position[0]).toBeCloseTo(1, 2);
        expect(a.motionProperties.linearVelocity[0]).toBeGreaterThan(59);
    });

    test('ccd bodies resolved out of their creation order still find the ccd records of the bodies they hit', () => {
        const { world, layers } = createTestWorld();
        // a and b converge as in the test above; c, created last, hits its own wall first and sorts ahead of both
        const a = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.2, -3, 0, 300);
        const b = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.2, 3, 0, -300);
        const c = addBullet(world, layers.OBJECT_LAYER_MOVING, 0.2, -2, 20, 300);
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.1, 1, 1) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 20, 0),
        });

        updateWorld(world, undefined, 1 / 60);

        // every body still points at its own record
        expect(world.ccd.ccdBodies[a.ccdBodyIndex].bodyIndex).toBe(a.index);
        expect(world.ccd.ccdBodies[b.ccdBodyIndex].bodyIndex).toBe(b.index);
        expect(world.ccd.ccdBodies[c.ccdBodyIndex].bodyIndex).toBe(c.index);
        expect(c.position[0]).toBeCloseTo(-0.3, 1);
        expect(b.position[0]).toBeCloseTo(0.2, 2);
        expect(a.position[0]).toBeLessThan(b.position[0]);
    });
});
