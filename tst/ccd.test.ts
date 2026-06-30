import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    type ContactManifold,
    type RigidBody,
    box,
    type Listener,
    MotionQuality,
    MotionType,
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
});
