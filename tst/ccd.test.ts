import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { type RigidBody, box, type Listener, MotionQuality, MotionType, rigidBody, sphere, updateWorld } from '../src';
import { createTestWorld } from './helpers';

describe('CCD (Continuous Collision Detection) / MotionQuality.LINEAR_CAST', () => {
    test('should prevent fast bullet from tunneling through thin wall with LINEAR_CAST', () => {
        const { world, layers } = createTestWorld();

        // Create a thin static wall
        const wallShape = box.create({ halfExtents: vec3.fromValues(5, 5, 0.1), density: 1000 });
        rigidBody.create(world, {
            shape: wallShape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // Create a fast-moving bullet with LINEAR_CAST (CCD enabled)
        const bulletShape = sphere.create({ radius: 0.2, density: 1000 });
        const bullet = rigidBody.create(world, {
            shape: bulletShape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-10, 0, 0),
        });

        // Set bullet motion quality to LINEAR_CAST for CCD
        bullet.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;

        // Give bullet very high velocity toward the wall (enough to tunnel without CCD)
        const highVelocity = vec3.fromValues(200, 0, 0); // 200 m/s
        vec3.copy(bullet.motionProperties.linearVelocity, highVelocity);

        // Store initial position
        const initialX = bullet.position[0];

        // Run physics for one frame (1/60 second)
        const timeStep = 1 / 60;
        updateWorld(world, undefined, timeStep);

        // Without CCD, bullet would travel ~3.33 units (200 * 1/60) and tunnel through the wall
        // With CCD enabled, bullet should stop at or very close to the wall surface (near x=0)

        // Bullet should have moved forward from start
        expect(bullet.position[0]).toBeGreaterThan(initialX);

        // Most importantly: bullet should stop before passing through the wall
        // (x should be < ~0.3 accounting for radius and wall thickness)
        expect(bullet.position[0]).toBeLessThan(0.5);

        // CCD should have detected and handled the collision (velocity changed)
        const finalVelocityX = bullet.motionProperties.linearVelocity[0];
        expect(finalVelocityX).not.toBe(highVelocity[0]); // Velocity should have changed
    });

    test('should trigger contact listener callbacks for CCD contacts', () => {
        const { world, layers } = createTestWorld();

        // Create a static wall
        const wallShape = box.create({ halfExtents: vec3.fromValues(5, 5, 0.5), density: 1000 });
        rigidBody.create(world, {
            shape: wallShape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // Create a fast-moving bullet with CCD enabled
        const bulletShape = sphere.create({ radius: 0.3, density: 1000 });
        const bullet = rigidBody.create(world, {
            shape: bulletShape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-5, 0, 0),
        });

        bullet.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(bullet.motionProperties.linearVelocity, 100, 0, 0);

        // Track contact callbacks
        let contactAddedCalled = false;
        const bodiesFromCallback: RigidBody[] = [];

        const listener: Listener = {
            onContactAdded: (bodyA: RigidBody, bodyB: RigidBody) => {
                contactAddedCalled = true;
                bodiesFromCallback.push(bodyA, bodyB);
            },
        };

        // Run physics update
        updateWorld(world, listener, 1 / 60);

        // Verify contact listener was called for CCD collision
        expect(contactAddedCalled).toBe(true);

        // Verify the callback was called with our bullet (don't assume ordering)
        const bodyIdsFromCallback = bodiesFromCallback.map((b) => b.id);
        expect(bodyIdsFromCallback).toContain(bullet.id);

        // Verify bullet was stopped by the wall (main goal of CCD - no tunneling)
        // Bullet should be close to wall surface, not past it
        expect(bullet.position[0]).toBeLessThan(0.5);

        // Bullet shouldn't have tunneled through (absolute position check)
        expect(Math.abs(bullet.position[0])).toBeLessThan(10); // sanity check
    });
});
