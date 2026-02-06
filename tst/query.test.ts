import { quat, vec3 } from 'mathcat';
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
