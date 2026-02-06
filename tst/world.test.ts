import { quat, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { MotionType, rigidBody, sphere, updateWorld } from '../src';
import { getBodyIdIndex, getBodyIdSequence, INVALID_BODY_ID } from '../src/body/body-id';
import { createTestWorld } from './helpers';

describe('World', () => {
    test('should add and remove body', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const testBody = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });
        // Body should have valid ID now
        expect(testBody.id).not.toBe(INVALID_BODY_ID);

        // World should contain the body at its index
        const index = getBodyIdIndex(testBody.id);
        expect(world.bodies.pool[index]).toBe(testBody);
        expect(testBody._pooled).toBe(false);

        // Remove body from world
        const removed = rigidBody.remove(world, testBody);
        expect(removed).toBe(true);

        // Body should be marked as pooled (stays in array)
        expect(testBody._pooled).toBe(true);
        expect(world.bodies.pool[index]).toBe(testBody); // Still in array
    });

    test('should reuse index but increment sequence when adding after removal', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });

        // Add first body
        const body1 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });
        const id1 = body1.id;
        const index1 = getBodyIdIndex(id1);
        const sequence1 = getBodyIdSequence(id1);

        // Remove first body
        rigidBody.remove(world, body1);

        // Add second body - should reuse index but have different sequence
        const body2 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });
        const id2 = body2.id;
        const index2 = getBodyIdIndex(id2);
        const sequence2 = getBodyIdSequence(id2);

        // Index should be reused
        expect(index2).toBe(index1);

        // Sequence should be different (incremented)
        expect(sequence2).toBe(sequence1 + 1);

        // IDs should be different overall
        expect(id2).not.toBe(id1);
    });

    test('should allocate new indices when pool is empty', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });

        // Add first body - gets index 0
        const body1 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });
        const id1 = body1.id;
        expect(getBodyIdIndex(id1)).toBe(0);

        // Add second body - gets index 1 (pool is empty)
        const body2 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });
        const id2 = body2.id;
        expect(getBodyIdIndex(id2)).toBe(1);

        // Remove first body - index 0 goes into pool
        rigidBody.remove(world, body1);

        // Add third body - reuses index 0 from pool
        const body3 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });
        const id3 = body3.id;
        expect(getBodyIdIndex(id3)).toBe(0);

        // Add fourth body - pool is empty again, gets index 2
        const body4 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });
        const id4 = body4.id;
        expect(getBodyIdIndex(id4)).toBe(2);
    });

    test('should maintain correct storage when removing non-last body', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });

        // Add three bodies
        const body1 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC, position: vec3.fromValues(1, 0, 0) });
        const id1 = body1.id;
        const index1 = getBodyIdIndex(id1);

        const body2 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC, position: vec3.fromValues(2, 0, 0) });
        const id2 = body2.id;
        const index2 = getBodyIdIndex(id2);

        const body3 = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC, position: vec3.fromValues(3, 0, 0) });
        const id3 = body3.id;
        const index3 = getBodyIdIndex(id3);

        // Remove middle body (body2)
        const removed = rigidBody.remove(world, body2);
        expect(removed).toBe(true);

        // Bodies stay at their indices (no swap-and-pop with pooling)
        // body2 is now pooled but still in the array
        expect(world.bodies.pool[index2]).toBe(body2);
        expect(body2._pooled).toBe(true);

        // body1 and body3 stay at their original indices
        expect(world.bodies.pool[index1]).toBe(body1);
        expect(body1._pooled).toBe(false);
        expect(world.bodies.pool[index3]).toBe(body3);
        expect(body3._pooled).toBe(false);
    });

    test('should return false when removing non-existent body', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const testBody = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        // Remove body (should succeed)
        const removed = rigidBody.remove(world, testBody);
        expect(removed).toBe(true);

        // Try to remove again - should return false
        const removedAgain = rigidBody.remove(world, testBody);
        expect(removedAgain).toBe(false);
    });

    test('should clean up stale contacts when bodies move apart and trigger contact callbacks', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0, density: 1000 });

        // Create two bodies that are initially colliding (overlapping)
        const bodyA = rigidBody.create(world, { 
            shape, 
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING, 
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const bodyB = rigidBody.create(world, { 
            shape, 
            objectLayer: layers.OBJECT_LAYER_MOVING, 
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0.5, 0, 0), // overlapping with bodyA
        });

        // Track contact callbacks
        const contactAdded: Array<{ bodyIdA: number; bodyIdB: number }> = [];
        const contactPersisted: Array<{ bodyIdA: number; bodyIdB: number }> = [];
        const contactRemoved: Array<{ bodyIdA: number; bodyIdB: number; subShapeIdA: number; subShapeIdB: number }> = [];

        const listener = {
            onContactAdded: (bodyA: any, bodyB: any) => {
                contactAdded.push({ bodyIdA: bodyA.id, bodyIdB: bodyB.id });
            },
            onContactPersisted: (bodyA: any, bodyB: any) => {
                contactPersisted.push({ bodyIdA: bodyA.id, bodyIdB: bodyB.id });
            },
            onContactRemoved: (bodyIdA: number, bodyIdB: number, subShapeIdA: number, subShapeIdB: number) => {
                contactRemoved.push({ bodyIdA, bodyIdB, subShapeIdA, subShapeIdB });
            },
        };

        // Run world update - should detect collision and create contacts
        updateWorld(world, listener, 1 / 60);

        // Verify onContactAdded was called
        expect(contactAdded.length).toBeGreaterThan(0);
        expect(contactAdded[0].bodyIdA).toBe(bodyA.id);
        expect(contactAdded[0].bodyIdB).toBe(bodyB.id);

        // Verify contacts exist
        expect(bodyA.contactCount).toBeGreaterThan(0);
        expect(bodyB.contactCount).toBeGreaterThan(0);

        // Reset callback tracking
        contactAdded.length = 0;
        contactPersisted.length = 0;
        contactRemoved.length = 0;

        // Run another update - contacts should persist (bodies still touching)
        updateWorld(world, listener, 1 / 60);

        // Verify onContactPersisted was called
        expect(contactPersisted.length).toBeGreaterThan(0);
        expect(contactPersisted[0].bodyIdA).toBe(bodyA.id);
        expect(contactPersisted[0].bodyIdB).toBe(bodyB.id);

        // Reset callback tracking
        contactAdded.length = 0;
        contactPersisted.length = 0;
        contactRemoved.length = 0;

        // Move bodies far apart (beyond broadphase detection range)
        rigidBody.setTransform(world, bodyA, vec3.fromValues(0, 0, 0), quat.create(), false);
        rigidBody.setTransform(world, bodyB, vec3.fromValues(100, 0, 0), quat.create(), false);

        // Run world update - broadphase won't detect them, so contacts should be removed
        updateWorld(world, listener, 1 / 60);

        // Verify onContactRemoved was called
        expect(contactRemoved.length).toBeGreaterThan(0);
        expect(contactRemoved[0].bodyIdA).toBe(bodyA.id);
        expect(contactRemoved[0].bodyIdB).toBe(bodyB.id);

        // Verify contacts were destroyed
        expect(bodyA.contactCount).toBe(0);
        expect(bodyB.contactCount).toBe(0);
    });

    test('should clean up stale contacts when sub-shapes stop colliding within same body pair', () => {
        const { world, layers } = createTestWorld();

        // Create two simple spheres for the compound
        const sphere1 = sphere.create({ radius: 0.5, density: 1000 });
        const sphere2 = sphere.create({ radius: 0.5, density: 1000 });

        // Create two bodies that are initially colliding
        const bodyA = rigidBody.create(world, {
            shape: sphere1,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        rigidBody.create(world, {
            shape: sphere2,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0.5, 0, 0), // overlapping
        });

        // Track contact callbacks
        const contactRemoved: Array<{ bodyIdA: number; bodyIdB: number; subShapeIdA: number; subShapeIdB: number }> = [];

        const listener = {
            onContactRemoved: (bodyIdA: number, bodyIdB: number, subShapeIdA: number, subShapeIdB: number) => {
                contactRemoved.push({ bodyIdA, bodyIdB, subShapeIdA, subShapeIdB });
            },
        };

        // First update - spheres collide
        updateWorld(world, listener, 1 / 60);

        // Verify contact exists
        expect(bodyA.contactCount).toBeGreaterThan(0);

        // Move sphere to a different position (still in broadphase range but not colliding)
        rigidBody.setTransform(world, bodyA, vec3.fromValues(5, 0, 0), quat.create(), false);

        // Reset tracking
        contactRemoved.length = 0;

        // Second update - spheres no longer collide
        updateWorld(world, listener, 1 / 60);

        // Verify onContactRemoved was called
        expect(contactRemoved.length).toBeGreaterThan(0);

        // Verify contacts were destroyed
        expect(bodyA.contactCount).toBe(0);
    });
});
