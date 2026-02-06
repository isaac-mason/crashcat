import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { box, capsule, MotionType, type RigidBody, rigidBody, sphere, updateWorld } from '../../src';
import { createTestWorld } from '../helpers';

describe('Body Sleeping', () => {
    test('sleeping body should wake when active body collides with it', () => {
        const { world, layers } = createTestWorld();

        // create sleeping dynamic body
        const sleepingBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0),
        });
        sleepingBody.sleeping = true;

        // create active dynamic body that will collide
        const activeBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1.5, 0), // directly above, touching/overlapping
        });
        vec3.set(activeBody.motionProperties.linearVelocity, 0, -1, 0);

        expect(sleepingBody.sleeping).toBe(true);
        expect(activeBody.sleeping).toBe(false);

        // step simulation
        updateWorld(world, undefined, 1 / 60);

        // sleeping body should have been woken by the collision
        expect(sleepingBody.sleeping).toBe(false);
    });

    test('sleeping body on static floor should wake when active body falls on it', () => {
        const { world, layers } = createTestWorld();

        // static floor
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [10, 0.5, 10] }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        // sleeping dynamic body on floor
        const sleepingBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0),
        });
        sleepingBody.sleeping = true;

        // falling dynamic body
        const fallingBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 2, 0),
        });
        vec3.set(fallingBody.motionProperties.linearVelocity, 0, -5, 0);

        expect(sleepingBody.sleeping).toBe(true);

        // simulate for a few frames until falling body hits sleeping body
        for (let i = 0; i < 30; i++) {
            updateWorld(world, undefined, 1 / 60);
            if (!sleepingBody.sleeping) break;
        }

        // sleeping body should have been woken
        expect(sleepingBody.sleeping).toBe(false);
    });

    test('falling body should not pass through sleeping body', () => {
        const { world, layers } = createTestWorld();

        // static floor to prevent bodies from falling forever
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [10, 0.5, 10] }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        // sleeping dynamic body on floor
        const sleepingBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [1, 1, 1] }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1, 0),
        });
        sleepingBody.sleeping = true;

        // falling dynamic body starting just above sleeping body
        const fallingBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 3, 0),
        });

        // simulate for many frames
        for (let i = 0; i < 120; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        // falling body should be resting on or above the sleeping body
        // sleeping body top is at y=2, falling body center should be above y=2
        // (falling body has half extent 0.5, so center at 2.5 means bottom at 2.0)
        expect(fallingBody.position[1]).toBeGreaterThan(2.0);
        
        // sleeping body should still be at approximately y=1 (resting on floor)
        expect(sleepingBody.position[1]).toBeCloseTo(1.0, 0);
    });

    test('capsule falling onto sleeping capsule should collide (many bodies scenario)', () => {
        const { world, layers } = createTestWorld();

        // static floor
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [20, 0.5, 20] }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        // create many dynamic bodies to simulate the voxel example scenario
        const bodies: RigidBody[] = [];
        for (let i = 0; i < 50; i++) {
            const body = rigidBody.create(world, {
                shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(
                    (i % 10) * 2 - 9, // spread out on x
                    0.8, // just above floor
                    Math.floor(i / 10) * 2 - 4, // spread out on z
                ),
            });
            bodies.push(body);
        }

        // let bodies settle and go to sleep
        for (let i = 0; i < 300; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        // verify most bodies are sleeping
        const sleepingCount = bodies.filter(b => b.sleeping).length;
        expect(sleepingCount).toBeGreaterThan(40);

        // pick a sleeping body in the middle
        const targetBody = bodies.find(b => b.sleeping && Math.abs(b.position[0]) < 2 && Math.abs(b.position[2]) < 2);
        expect(targetBody).toBeDefined();
        
        const targetY = targetBody!.position[1];
        const targetX = targetBody!.position[0];
        const targetZ = targetBody!.position[2];

        // create falling capsule directly above the sleeping one
        const fallingCapsule = rigidBody.create(world, {
            shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(targetX, targetY + 3, targetZ),
        });

        // simulate
        for (let i = 0; i < 120; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        // falling capsule should be above the target body (not passed through)
        expect(fallingCapsule.position[1]).toBeGreaterThan(1.0);
    });

    test('capsule falling onto sleeping capsule with body respawning', () => {
        const { world, layers } = createTestWorld();

        // static floor
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [20, 0.5, 20] }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        // create many dynamic bodies
        const bodies: RigidBody[] = [];
        for (let i = 0; i < 50; i++) {
            const body = rigidBody.create(world, {
                shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(
                    (i % 10) * 2 - 9,
                    0.8,
                    Math.floor(i / 10) * 2 - 4,
                ),
            });
            bodies.push(body);
        }

        // let bodies settle and go to sleep
        for (let i = 0; i < 300; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        // simulate respawning: destroy and recreate several bodies (like voxel example does)
        for (let r = 0; r < 10; r++) {
            const idx = r * 5; // every 5th body
            if (idx < bodies.length) {
                rigidBody.remove(world, bodies[idx]);
                bodies[idx] = rigidBody.create(world, {
                    shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
                    objectLayer: layers.OBJECT_LAYER_MOVING,
                    motionType: MotionType.DYNAMIC,
                    position: vec3.fromValues(
                        (idx % 10) * 2 - 9,
                        5, // spawn high
                        Math.floor(idx / 10) * 2 - 4,
                    ),
                });
            }
        }

        // let newly spawned bodies fall and settle
        for (let i = 0; i < 300; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        // now pick a sleeping body and drop another on it
        const sleepingBodies = bodies.filter(b => b.sleeping && !b._pooled);
        expect(sleepingBodies.length).toBeGreaterThan(0);

        const targetBody = sleepingBodies[0];
        const targetY = targetBody.position[1];
        const targetX = targetBody.position[0];
        const targetZ = targetBody.position[2];

        console.log('target index:', targetBody.index, 'sleeping:', targetBody.sleeping);

        // create falling capsule
        const fallingCapsule = rigidBody.create(world, {
            shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(targetX, targetY + 3, targetZ),
        });

        console.log('falling capsule index:', fallingCapsule.index);

        // simulate
        for (let i = 0; i < 120; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        console.log('final falling capsule Y:', fallingCapsule.position[1].toFixed(3));
        console.log('target body Y:', targetBody.position[1].toFixed(3));

        // falling capsule should be above the target body
        expect(fallingCapsule.position[1]).toBeGreaterThan(1.0);
    });

    test('voxel example capsule size - falling onto sleeping capsules', () => {
        // exact capsule size from example-voxel-custom-shape.ts
        const CAPSULE_HALF_HEIGHT = 1.0;
        const CAPSULE_RADIUS = 1.0;
        // total height is (2 * halfHeight) + (2 * radius) = 4 units

        const { world, layers } = createTestWorld();

        // static floor (like voxel terrain)
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [50, 0.5, 50] }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        // create many capsules spread out and let them settle
        const bodies: RigidBody[] = [];
        for (let i = 0; i < 50; i++) {
            const body = rigidBody.create(world, {
                shape: capsule.create({ halfHeightOfCylinder: CAPSULE_HALF_HEIGHT, radius: CAPSULE_RADIUS }),
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(
                    (i % 10) * 5 - 22.5, // spread out more due to larger radius
                    3, // higher initial position to account for size
                    Math.floor(i / 10) * 5 - 10,
                ),
                restitution: 0,
                friction: 0.5,
            });
            bodies.push(body);
        }

        // let bodies settle and go to sleep
        for (let i = 0; i < 400; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        const sleepingCount = bodies.filter(b => b.sleeping).length;
        console.log('sleeping bodies:', sleepingCount, '/', bodies.length);
        expect(sleepingCount).toBeGreaterThan(40);

        // pick a sleeping body
        const targetBody = bodies.find(b => b.sleeping);
        expect(targetBody).toBeDefined();

        const targetX = targetBody!.position[0];
        const targetY = targetBody!.position[1];
        const targetZ = targetBody!.position[2];
        console.log('target at:', targetX.toFixed(2), targetY.toFixed(2), targetZ.toFixed(2));

        // drop a new capsule from high above (like SPAWN_HEIGHT = 25 in example)
        const fallingCapsule = rigidBody.create(world, {
            shape: capsule.create({ halfHeightOfCylinder: CAPSULE_HALF_HEIGHT, radius: CAPSULE_RADIUS }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(targetX, 25, targetZ),
            restitution: 0,
            friction: 0.5,
        });

        // simulate for longer to let it fall from high
        for (let i = 0; i < 180; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        console.log('final falling capsule Y:', fallingCapsule.position[1].toFixed(3));
        console.log('target body Y:', targetBody!.position[1].toFixed(3));
        console.log('target sleeping:', targetBody!.sleeping);

        // falling capsule should have collided, not passed through
        // capsule centers when stacked should be at least 2 + 2 = 4 units apart (radius + radius)
        expect(fallingCapsule.position[1]).toBeGreaterThan(3.0);
    });

    test('should put bodies to sleep when at rest', () => {
        const { world, layers } = createTestWorld();

        // Disable gravity so body doesn't fall
        world.settings.gravityEnabled = false;

        // Create a dynamic sphere with zero velocity
        const body = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // Ensure velocity is zero
        vec3.zero(body.motionProperties.linearVelocity);
        vec3.zero(body.motionProperties.angularVelocity);

        // Initially not sleeping
        expect(body.sleeping).toBe(false);

        // Simulate for timeBeforeSleep (0.5s) + margin
        const timeBeforeSleep = world.settings.sleeping.timeBeforeSleep;
        const steps = Math.ceil((timeBeforeSleep + 0.1) * 60); // 60 fps for 0.6s
        
        for (let i = 0; i < steps; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        // Body should now be sleeping
        expect(body.sleeping).toBe(true);

        // Wake the body by applying an impulse
        rigidBody.addImpulse(world, body, vec3.fromValues(0, 5, 0));

        // Step once
        updateWorld(world, undefined, 1 / 60);

        // Body should not be sleeping (has velocity)
        expect(body.sleeping).toBe(false);
    });
});
