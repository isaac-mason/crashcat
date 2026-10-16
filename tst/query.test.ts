import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    castRay,
    castShape,
    collidePoint,
    collideShape,
    createAllCastRayCollector,
    createAllCastShapeCollector,
    createAllCollidePointCollector,
    createAllCollideShapeCollector,
    createClosestCastShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    filter,
    MotionType,
    rigidBody,
    sphere,
} from '../src';
import { createTestWorld } from './helpers';

describe('Query API - castRay', () => {
    test('should hit box when ray intersects', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        const shape = box.create({ halfExtents: [1, 1, 1] });
        rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Cast ray from behind the box, through it
        const collector = createAllCastRayCollector();
        const settings = createDefaultCastRaySettings();
        const origin = vec3.fromValues(-5, 0, 0);
        const direction = vec3.fromValues(1, 0, 0); // Ray pointing in +X direction
        const length = 10;

        castRay(world, collector, settings, origin, direction, length, queryFilter);

        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].fraction).toBeGreaterThan(0);
        expect(collector.hits[0].fraction).toBeLessThan(1);
    });

    test('should not hit box when ray misses', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        const shape = box.create({ halfExtents: [1, 1, 1] });
        rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Cast ray that misses the box (above it)
        const collector = createAllCastRayCollector();
        const settings = createDefaultCastRaySettings();
        const origin = vec3.fromValues(-5, 5, 0);
        const direction = vec3.fromValues(1, 0, 0); // Ray pointing in +X direction
        const length = 10;

        castRay(world, collector, settings, origin, direction, length, queryFilter);

        expect(collector.hits.length).toBe(0);
    });
});

describe('Query API - castShape', () => {
    test('should hit box when sphere sweep intersects', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        const shape = box.create({ halfExtents: [1, 1, 1] });
        rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Sweep a sphere from left to right through the box
        const collector = createAllCastShapeCollector();
        const settings = createDefaultCastShapeSettings();
        const sweepShape = sphere.create({ radius: 0.5 });
        const position = vec3.fromValues(-3, 0, 0);
        const rotation = quat.create();
        const scale = vec3.fromValues(1, 1, 1);
        const displacement = vec3.fromValues(6, 0, 0); // Sweep in +X direction

        castShape(world, collector, settings, sweepShape, position, rotation, scale, displacement, queryFilter);

        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].fraction).toBeGreaterThan(0);
        expect(collector.hits[0].fraction).toBeLessThan(1);
    });

    test('should not hit box when sphere sweep misses', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        const shape = box.create({ halfExtents: [1, 1, 1] });
        rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Sweep a sphere that misses the box (above it)
        const collector = createAllCastShapeCollector();
        const settings = createDefaultCastShapeSettings();
        const sweepShape = sphere.create({ radius: 0.5 });
        const position = vec3.fromValues(-5, 5, 0);
        const rotation = quat.create();
        const scale = vec3.fromValues(1, 1, 1);
        const displacement = vec3.fromValues(10, 0, 0); // Sweep in +X direction

        castShape(world, collector, settings, sweepShape, position, rotation, scale, displacement, queryFilter);

        expect(collector.hits.length).toBe(0);
    });

    // two slab bodies side by side: the left one's top at y = 0 over x in [-2, 0], the right one's at y = 0.3 over
    // x in [0, 2]. a radius 0.5 sphere centred at (0, 0.4) starts 0.1 into the left body and 0.4 into the right one.
    for (const order of ['left first', 'right first']) {
        test(`closest collector reports the deeper of two bodies the cast starts inside (${order})`, () => {
            const { world, layers } = createTestWorld();
            const queryFilter = filter.create(world.settings.layers);

            const slabs = [vec3.fromValues(-1, -0.5, 0), vec3.fromValues(1, -0.2, 0)];
            if (order === 'right first') slabs.reverse();
            for (const position of slabs) {
                rigidBody.create(world, {
                    shape: box.create({ halfExtents: [1, 0.5, 1], convexRadius: 0 }),
                    objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
                    motionType: MotionType.STATIC,
                    position,
                    quaternion: quat.create(),
                });
            }

            const collector = createClosestCastShapeCollector();
            const settings = createDefaultCastShapeSettings();
            settings.returnDeepestPoint = true;
            const sweepShape = sphere.create({ radius: 0.5 });

            castShape(
                world,
                collector,
                settings,
                sweepShape,
                vec3.fromValues(0, 0.4, 0),
                quat.create(),
                vec3.fromValues(1, 1, 1),
                vec3.fromValues(0, -1, 0),
                queryFilter,
            );

            expect(collector.hit.fraction).toBe(0);
            expect(collector.hit.penetrationDepth).toBeCloseTo(0.4, 4);
            expect(collector.hit.pointB[1]).toBeCloseTo(0.3, 4);
        });
    }
});

describe('Query API - collidePoint', () => {
    test('should hit box when point is inside', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        const shape = box.create({ halfExtents: [1, 1, 1] });
        rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Check point inside the box
        const collector = createAllCollidePointCollector();
        const settings = createDefaultCollidePointSettings();
        const point = vec3.fromValues(0.5, 0.5, 0.5);

        collidePoint(world, collector, settings, point, queryFilter);

        expect(collector.hits.length).toBeGreaterThan(0);
    });

    test('should not hit box when point is outside', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        const shape = box.create({ halfExtents: [1, 1, 1] });
        rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Check point outside the box
        const collector = createAllCollidePointCollector();
        const settings = createDefaultCollidePointSettings();
        const point = vec3.fromValues(5, 5, 5);

        collidePoint(world, collector, settings, point, queryFilter);

        expect(collector.hits.length).toBe(0);
    });
});

describe('Query API - collideShape', () => {
    test('should hit box when sphere overlaps', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [1, 1, 1] }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Check sphere that overlaps with the box
        const collector = createAllCollideShapeCollector();
        const settings = createDefaultCollideShapeSettings();
        const queryShape = sphere.create({ radius: 1 });
        const position = vec3.fromValues(0.5, 0.5, 0.5); // Sphere overlapping box
        const rotation = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        collideShape(world, collector, settings, queryShape, position, rotation, scale, queryFilter);

        expect(collector.hits.length).toBeGreaterThan(0);
    });

    test('should not hit box when sphere does not overlap', () => {
        const { world, layers } = createTestWorld();
        const queryFilter = filter.create(world.settings.layers);

        // Create a box at origin
        const shape = box.create({ halfExtents: [1, 1, 1] });
        rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });

        // Check sphere that doesn't overlap with the box
        const collector = createAllCollideShapeCollector();
        const settings = createDefaultCollideShapeSettings();
        const queryShape = sphere.create({ radius: 1 });
        const position = vec3.fromValues(5, 5, 5); // Sphere far from box
        const rotation = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        collideShape(world, collector, settings, queryShape, position, rotation, scale, queryFilter);

        expect(collector.hits.length).toBe(0);
    });
});
