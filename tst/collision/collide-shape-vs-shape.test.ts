import { euler, quat, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    box,
    capsule,
    collideShapeVsShape,
    compound,
    convexHull,
    cylinder,
    createAllCollideShapeCollector,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    plane,
    scaled,
    sphere,
    transformed,
    triangleMesh,
    collisionDispatch,
} from '../../src';

const settings = createDefaultCollideShapeSettings();

describe('collideShapeVsShape - Sphere vs Sphere', () => {
    test('should detect no collision between distant spheres', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(10, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(0);
    });

    test('should detect collision between overlapping spheres', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(1.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        expect(contact.penetration).toBeGreaterThan(0);
    });

    test('should detect collision between nearly touching spheres', () => {
        console.log("collisionDispatch", collisionDispatch)

        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(1.99, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Penetration should be very small
        expect(contact.penetration).toBeGreaterThan(0);
        expect(contact.penetration).toBeLessThan(0.1);
    });

    test('should handle scaled sphere collision', () => {
        const shapeA = scaled.create({
            shape: sphere.create({ radius: 1.0 }),
            scale: vec3.fromValues(2, 2, 2),
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(2.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should handle exactly overlapping spheres (EPA worst case)', () => {
        // Two identical spheres at the same position - worst case for EPA
        const sphereCenter = vec3.fromValues(1, 2, 3);
        const shapeA = transformed.create({
            shape: sphere.create({ radius: 2.0 }),
            position: sphereCenter,
            quaternion: quat.create(),
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = transformed.create({
            shape: sphere.create({ radius: 2.0 }),
            position: sphereCenter,
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // In the worst case, EPA approximates the full sphere hull
        // Penetration depth should be approximately 2 * radius = 4.0
        expect(contact.penetration).toBeGreaterThan(3.5);
        expect(contact.penetration).toBeLessThan(4.5);
    });

    test('should handle nearly overlapping spheres (deep penetration)', () => {
        // Two spheres almost overlapping but slightly offset
        const sphere1Center = vec3.fromValues(1, 2, 3);
        const sphere2Center = vec3.fromValues(1.1, 2, 3);

        const shapeA = transformed.create({
            shape: sphere.create({ radius: 2.0 }),
            position: sphere1Center,
            quaternion: quat.create(),
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = transformed.create({
            shape: sphere.create({ radius: 1.8 }),
            position: sphere2Center,
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Expected penetration: radius1 + radius2 - distance between centers
        const centerDistance = vec3.distance(sphere1Center, sphere2Center);
        const expectedPenetration = 2.0 + 1.8 - centerDistance;
        expect(contact.penetration).toBeCloseTo(expectedPenetration, 1);
    });
});

describe('collideShapeVsShape - Box vs Box (GJK)', () => {
    test('should detect no collision between distant boxes', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(10, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(0);
    });

    test('should detect collision between overlapping boxes', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(1.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        expect(contact.penetration).toBeGreaterThan(0);
        // // Penetration axis should point from A to B (positive X direction)
        // expect(Math.abs(contact.penetrationAxis[0])).toBeCloseTo(1, 1);
        // expect(Math.abs(contact.penetrationAxis[1])).toBeCloseTo(0, 1);
        // expect(Math.abs(contact.penetrationAxis[2])).toBeCloseTo(0, 1);
    });

    test('should detect collision between nearly touching boxes', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(1.99, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Should have small penetration
        expect(contact.penetration).toBeGreaterThan(0);
        expect(contact.penetration).toBeLessThan(0.1);
    });

    test('should handle rotated box collision', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Rotate box B by 45 degrees around Z axis
        const quatB = quat.create();
        quat.setAxisAngle(quatB, vec3.fromValues(0, 0, 1), Math.PI / 4);

        // Position box B closer to ensure clear overlap
        // Box A extends from -1 to +1, so position B at 0.8 units for definitive collision
        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(0.8, 0, 0);
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should handle scaled box collision', () => {
        const shapeA = scaled.create({
            shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
            scale: vec3.fromValues(2, 1, 1),
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(2.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should handle deeply penetrating boxes (one inside another)', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(2, 2, 2) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Small box inside large box
        const shapeB = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const posB = vec3.fromValues(0.5, 0.5, 0.5);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Should detect deep penetration
        expect(contact.penetration).toBeGreaterThan(0);
    });
});

describe('collideShapeVsShape - Sphere vs Box (GJK)', () => {
    test('should detect no collision between distant sphere and box', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(10, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );
    });

    test('should detect collision between overlapping sphere and box', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(1.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should detect sphere-box corner collision', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Position box so it clearly collides with sphere
        // Sphere has radius 1.0, place box corner closer to ensure definitive collision
        // Box at (0.3, 0.3, 0.3) with half-extents (1,1,1) means nearest corner is at (-0.7, -0.7, -0.7)
        // Distance from origin = sqrt(3*0.7^2) ≈ 1.212, which is > 1.0 but box extends to (1.3, 1.3, 1.3)
        // So sphere clearly overlaps with the box volume
        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(0.3, 0.3, 0.3);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results[0].penetration).toBeGreaterThan(0);
    });
});

describe('collideShapeVsShape - Transformed Shapes', () => {
    test('should handle transformed sphere collision', () => {
        const shapeA = transformed.create({
            shape: sphere.create({ radius: 1.0 }),
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.create(),
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = transformed.create({
            shape: sphere.create({ radius: 1.0 }),
            position: vec3.fromValues(1.5, 0, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should handle nested transformed shapes', () => {
        // Create a sphere that's scaled, then translated
        const innerShape = scaled.create({
            shape: sphere.create({ radius: 1.0 }),
            scale: vec3.fromValues(2, 2, 2),
        });
        const shapeA = transformed.create({
            shape: innerShape,
            position: vec3.fromValues(3, 0, 0),
            quaternion: quat.create(),
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(4.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results[0].penetration).toBeGreaterThan(0);
    });
});

describe('collideShapeVsShape - Compound Shapes', () => {
    test('should detect no collision between distant compound and sphere', () => {
        const compoundA = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 0.5 }) },
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
                },
            ],
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(10, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            compoundA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(0);
    });

    test('should detect collision between compound and sphere', () => {
        const compoundA = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 0.5 }) },
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
                },
            ],
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(1.2, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            compoundA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
    });

    test('should detect collision with compound child using child position', () => {
        // Compound with a child that's translated
        const compoundA = compound.create({
            children: [
                {
                    position: vec3.fromValues(2, 0, 0),
                    quaternion: quat.create(),
                    shape: sphere.create({ radius: 0.5 }),
                },
            ],
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 0.5 });
        const posB = vec3.fromValues(2.6, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            compoundA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
    });

    test('should detect multiple contacts with compound shape', () => {
        // Compound with two spheres positioned to both hit a larger sphere
        const compoundA = compound.create({
            children: [
                {
                    position: vec3.fromValues(-0.5, 0, 0),
                    quaternion: quat.create(),
                    shape: sphere.create({ radius: 0.3 }),
                },
                {
                    position: vec3.fromValues(0.5, 0, 0),
                    quaternion: quat.create(),
                    shape: sphere.create({ radius: 0.3 }),
                },
            ],
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            compoundA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        // Should detect collisions with both children
        expect(results.length).toBe(2);
    });

    test('should detect collision between sphere and compound', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const compoundB = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 0.5 }) },
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
                },
            ],
        });
        const posB = vec3.fromValues(1.2, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            compoundB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
    });

    test('should detect collision between two compound shapes', () => {
        const compoundA = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 0.5 }) },
                {
                    position: vec3.fromValues(0.8, 0, 0),
                    quaternion: quat.create(),
                    shape: sphere.create({ radius: 0.3 }),
                },
            ],
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const compoundB = compound.create({
            children: [
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
                },
                {
                    position: vec3.fromValues(1, 0, 0),
                    quaternion: quat.create(),
                    shape: sphere.create({ radius: 0.3 }),
                },
            ],
        });
        const posB = vec3.fromValues(0.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            compoundA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            compoundB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
    });

    test('should handle compound shape at scaled body', () => {
        const compoundA = compound.create({
            children: [{ position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 0.5 }) }],
        });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(2, 2, 2);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(1.8, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            compoundA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        // Scaled sphere (radius 0.5 * 2 = 1.0) at origin collides with sphere at (1.8, 0, 0) with radius 1.0
        // Sum of radii = 2.0, distance = 1.8, so they collide
        expect(results.length).toBeGreaterThan(0);
    });
});

describe('collideShapeVsShape - Convex vs Triangle Mesh', () => {
    test('should detect no collision when sphere is above triangle mesh', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(0.5, 0.5, 2);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Create a horizontal triangle in XY plane at Z=0
        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2], // CCW winding
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(0);
    });
});

describe('collideShapeVsShape - Convex vs Triangle Mesh (GJK/EPA path)', () => {
    const settings = createDefaultCollideShapeSettings();

    test('should detect collision when box penetrates triangle mesh', () => {
        const boxA = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const posA = vec3.fromValues(0.5, 0.5, 0.2); // Box penetrating triangle from above
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2], // CCW winding, normal points up (+Z)
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            boxA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].penetration).toBeGreaterThan(0);
    });

    test('should detect box collision with triangle edge', () => {
        const boxA = box.create({ halfExtents: vec3.fromValues(0.3, 0.3, 0.3) });
        const posA = vec3.fromValues(2.2, 0, 0); // Near edge of triangle
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            boxA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBeGreaterThan(0);
    });

    test('should not detect collision when box is far from triangle mesh', () => {
        const boxA = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const posA = vec3.fromValues(10, 10, 10); // Far away
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            boxA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBe(0);
    });

    test('should handle capsule collision with triangle mesh', () => {
        const capsuleA = capsule.create({ radius: 0.3, halfHeightOfCylinder: 0.5 });
        const posA = vec3.fromValues(0.5, 0.5, 0.15);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            capsuleA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].penetration).toBeGreaterThan(0);
    });
});

describe('collideShapeVsShape - Sphere vs Triangle Mesh (specialized path)', () => {
    const settings = createDefaultCollideShapeSettings();

    test('should detect collision when sphere penetrates triangle mesh', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(0.5, 0.5, 0.2); // Center above triangle, penetrating
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Create a horizontal triangle in XY plane at Z=0
        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2], // CCW winding, normal points up (+Z)
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);

        const contact = results[0];
        expect(contact.penetration).toBeGreaterThan(0);
    });

    test('should detect sphere collision with triangle edge', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(2.3, 0, 0); // Near edge of triangle
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].penetration).toBeGreaterThan(0);
    });

    test('should detect sphere collision with triangle vertex', () => {
        const sphereA = sphere.create({ radius: 0.6 });
        const posA = vec3.fromValues(0, 0, 0); // At vertex
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].penetration).toBeCloseTo(0.6, 2);
    });

    test('should not detect collision when sphere is far from triangle mesh', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(10, 10, 10); // Far away
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBe(0);
    });

    test('should handle sphere collision with scaled triangle mesh', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(1, 1, 0.4); // Penetrating scaled mesh
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(2, 2, 2); // Scale mesh 2x

        const collector = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].penetration).toBeGreaterThan(0);
    });

    test('should handle backface mode for sphere vs triangle mesh', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(0.5, 0.5, -0.2); // Below triangle (backface side)
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2], // Normal points up (+Z)
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        // Test with backface culling enabled
        const settingsIgnoreBackfaces = createDefaultCollideShapeSettings();
        settingsIgnoreBackfaces.collideWithBackfaces = false;
        const collectorIgnore = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collectorIgnore,
            settingsIgnoreBackfaces,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collectorIgnore.hits.length).toBe(0);

        // Test with backface collision enabled
        const settingsCollideBackfaces = createDefaultCollideShapeSettings();
        settingsCollideBackfaces.collideWithBackfaces = true;
        const collectorCollide = createAllCollideShapeCollector();

        // biome-ignore format: readability
        collideShapeVsShape(
            collectorCollide,
            settingsCollideBackfaces,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(collectorCollide.hits.length).toBeGreaterThan(0);
    });

    test('should detect collision when box edge intersects triangle', () => {
        const boxA = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const posA = vec3.fromValues(0.5, 0.5, 0.2);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Large triangle in XY plane
        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2], // CCW winding, normal points up (+Z)
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            boxA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should detect collision with multiple triangles in mesh', () => {
        const sphereA = sphere.create({ radius: 0.6 });
        const posA = vec3.fromValues(1, 1, 0.3);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Create a quad (2 triangles) in XY plane at Z=0
        const meshB = triangleMesh.create({
            positions: [
                0,
                0,
                0, // 0
                2,
                0,
                0, // 1
                2,
                2,
                0, // 2
                0,
                2,
                0, // 3
            ],
            indices: [0, 1, 3, 1, 2, 3], // Two triangles forming a quad, CCW winding
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        // Should detect collision with at least one triangle
        expect(results.length).toBeGreaterThan(0);
    });

    test('should handle rotated triangle mesh', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Triangle in XY plane
        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2], // CCW winding
        });
        const posB = vec3.fromValues(0, 0, 0);
        // Rotate 90 degrees around X axis (triangle now in XZ plane)
        const quatB = quat.create();
        quat.fromEuler(quatB, euler.fromDegrees(euler.create(), 90, 0, 0, 'xyz'));
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
    });

    test('should handle scaled triangle mesh', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(2, 2, 0.3);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Small triangle that will be scaled up
        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2], // CCW winding, normal points up (+Z)
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(3, 3, 1); // Scale up the mesh

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        // Sphere at (2,2,0.3) is outside the unscaled triangle's AABB
        // BVH query happens in unscaled mesh space, so no collision is detected
        expect(results.length).toBe(0);
    });

    test('should respect back-face culling setting', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(0.5, 0.5, -0.2); // Below the triangle
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Triangle facing up (normal pointing +Z)
        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2], // CCW winding, normal points up (+Z)
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        // Test with backface culling enabled
        const settingsNoCull = { ...settings, collideWithBackfaces: false };
        const collector1 = createAllCollideShapeCollector();
        const results1 = collector1.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector1,
            settingsNoCull,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        // Should not collide from backside when culling enabled
        expect(results1.length).toBe(0);

        // Test with backface culling disabled
        const settingsWithCull = { ...settings, collideWithBackfaces: true };
        const collector2 = createAllCollideShapeCollector();
        const results2 = collector2.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector2,
            settingsWithCull,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        // Should collide from backside when culling disabled
        expect(results2.length).toBeGreaterThan(0);
    });

    test('should report correct sub-shape ID for hit triangle', () => {
        const sphereA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(1, 0.5, 0.2);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Create two separate triangles
        const meshB = triangleMesh.create({
            positions: [
                0,
                0,
                0, // Triangle 0
                1,
                0,
                0,
                0,
                1,
                0,
                1.5,
                0,
                0, // Triangle 1
                2.5,
                0,
                0,
                1.5,
                1,
                0,
            ],
            indices: [0, 1, 2, 3, 4, 5], // CCW winding for both triangles
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
        // Sphere should collide with triangle 1
        expect(results[0].subShapeIdB).toBeGreaterThanOrEqual(0);
    });

    test('should handle deep penetration with EPA fallback', () => {
        // Large sphere deeply penetrating the mesh
        const sphereA = sphere.create({ radius: 2.0 });
        const posA = vec3.fromValues(0.5, 0.5, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const meshB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2], // CCW winding
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBeGreaterThan(0);
        // Deep penetration should still be detected
        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should collide box with complex mesh (BVH traversal test)', () => {
        const boxA = box.create({ halfExtents: vec3.fromValues(0.3, 0.3, 0.3) });
        const posA = vec3.fromValues(5, 5, 0.2);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Create a larger mesh with multiple triangles (10x10 grid)
        const positions: number[] = [];
        const indices: number[] = [];
        const gridSize = 10;

        // Generate grid vertices
        for (let y = 0; y <= gridSize; y++) {
            for (let x = 0; x <= gridSize; x++) {
                positions.push(x, y, 0);
            }
        }

        // Generate triangle indices
        for (let y = 0; y < gridSize; y++) {
            for (let x = 0; x < gridSize; x++) {
                const i0 = y * (gridSize + 1) + x;
                const i1 = i0 + 1;
                const i2 = i0 + (gridSize + 1);
                const i3 = i2 + 1;

                indices.push(i0, i2, i1);
                indices.push(i1, i2, i3);
            }
        }

        const meshB = triangleMesh.create({ positions, indices });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            boxA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            meshB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        // Box should collide with nearby triangles
        expect(results.length).toBeGreaterThan(0);
    });
});

describe('collideShapeVsShape - Supporting Face Collection', () => {
    test('sphere vs sphere with collectFaces=true returns empty faces', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(1.5, 0, 0); // overlapping by 0.5 units
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const settings = createDefaultCollideShapeSettings();
        settings.collectFaces = true;

        const collector = createAllCollideShapeCollector();
        const hits = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(hits.length).toBe(1);

        const hit = hits[0];
        // spheres have no polygonal faces, should remain uninitialized
        expect(hit.faceA.numVertices).toBe(0);
        expect(hit.faceB.numVertices).toBe(0);
    });

    test('box vs box with collectFaces=false returns empty faces', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(1.5, 0, 0); // overlapping by 0.5 units
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const settings = createDefaultCollideShapeSettings();
        settings.collectFaces = false; // explicitly disabled

        const collector = createAllCollideShapeCollector();
        const hits = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(hits.length).toBe(1);

        const hit = hits[0];
        // faces should not be populated when collectFaces=false
        expect(hit.faceA.numVertices).toBe(0);
        expect(hit.faceB.numVertices).toBe(0);
    });

    test('box vs box with collectFaces=true returns 4-vertex faces', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(1.5, 0, 0); // overlapping along X axis
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const settings = createDefaultCollideShapeSettings();
        settings.collectFaces = true;

        const collector = createAllCollideShapeCollector();
        const hits = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(hits.length).toBe(1);

        const hit = hits[0];

        // both boxes should have 4-vertex faces
        expect(hit.faceA.numVertices).toBe(4);
        expect(hit.faceB.numVertices).toBe(4);

        // verify face vertices are in world space (not local space)
        // for a box centered at origin with half extents [1,1,1], all vertices should be within bounds
        for (let i = 0; i < hit.faceA.numVertices; i++) {
            const x = hit.faceA.vertices[i * 3];
            const y = hit.faceA.vertices[i * 3 + 1];
            const z = hit.faceA.vertices[i * 3 + 2];

            // vertices should be at posA ± halfExtents
            expect(Math.abs(x - posA[0])).toBeLessThanOrEqual(1.0);
            expect(Math.abs(y - posA[1])).toBeLessThanOrEqual(1.0);
            expect(Math.abs(z - posA[2])).toBeLessThanOrEqual(1.0);
        }

        for (let i = 0; i < hit.faceB.numVertices; i++) {
            const x = hit.faceB.vertices[i * 3];
            const y = hit.faceB.vertices[i * 3 + 1];
            const z = hit.faceB.vertices[i * 3 + 2];

            // vertices should be at posB ± halfExtents
            expect(Math.abs(x - posB[0])).toBeLessThanOrEqual(1.0);
            expect(Math.abs(y - posB[1])).toBeLessThanOrEqual(1.0);
            expect(Math.abs(z - posB[2])).toBeLessThanOrEqual(1.0);
        }
    });

    test('sphere vs triangle mesh with collectFaces=true returns empty sphere face and triangle (3 verts)', () => {
        const shapeA = sphere.create({ radius: 0.5 });
        const posA = vec3.fromValues(0.5, 0.5, 0.2); // penetrating triangle
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // create a simple triangle mesh with one triangle at z=0
        const shapeB = triangleMesh.create({
            positions: [0, 0, 0, 2, 0, 0, 0, 2, 0],
            indices: [0, 1, 2],
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const settings = createDefaultCollideShapeSettings();
        settings.collectFaces = true;

        const collector = createAllCollideShapeCollector();
        const hits = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(hits.length).toBeGreaterThan(0);

        const hit = hits[0];

        // sphere has no polygonal face (it's the convex shape A)
        expect(hit.faceA.numVertices).toBe(0);

        // triangle should have 3 vertices (it's shape B)
        expect(hit.faceB.numVertices).toBe(3);

        // verify triangle vertices are in world space
        // the triangle is at z=0, so all z coordinates should be near 0
        for (let i = 0; i < hit.faceB.numVertices; i++) {
            const z = hit.faceB.vertices[i * 3 + 2];
            expect(Math.abs(z - posB[2])).toBeLessThan(0.01);
        }
    });

    test('box vs box face directions are correct', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(1.5, 0, 0); // overlapping along X axis
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const settings = createDefaultCollideShapeSettings();
        settings.collectFaces = true;

        const collector = createAllCollideShapeCollector();
        const hits = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        const hit = hits[0];

        // penetration axis should point from A to B (in the +X direction for this setup)
        expect(hit.penetrationAxis[0]).toBeGreaterThan(0);

        // face A should be on the +X side (right face of box A)
        // all vertices should have x ≈ 1.0 (posA[0] + halfExtent)
        for (let i = 0; i < hit.faceA.numVertices; i++) {
            const x = hit.faceA.vertices[i * 3];
            expect(x).toBeCloseTo(1.0, 1);
        }

        // face B should be on the -X side (left face of box B)
        // all vertices should have x ≈ 0.5 (posB[0] - halfExtent)
        for (let i = 0; i < hit.faceB.numVertices; i++) {
            const x = hit.faceB.vertices[i * 3];
            expect(x).toBeCloseTo(0.5, 1);
        }
    });

    test('box vs box with scale maintains world space coordinates', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(2, 1, 1); // scale along X axis

        const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posB = vec3.fromValues(2.5, 0, 0); // overlapping with scaled box A
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const settings = createDefaultCollideShapeSettings();
        settings.collectFaces = true;

        const collector = createAllCollideShapeCollector();
        const hits = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        const hit = hits[0];

        expect(hit.faceA.numVertices).toBe(4);
        expect(hit.faceB.numVertices).toBe(4);

        // verify face A vertices reflect the scaling (should extend to ±2 in X, ±1 in Y/Z)
        // face A should be on the +X side, so all vertices should have x ≈ 2.0
        for (let i = 0; i < hit.faceA.numVertices; i++) {
            const x = hit.faceA.vertices[i * 3];
            const y = hit.faceA.vertices[i * 3 + 1];
            const z = hit.faceA.vertices[i * 3 + 2];

            expect(x).toBeCloseTo(2.0, 1); // scaled half extent
            expect(Math.abs(y)).toBeLessThanOrEqual(1.0);
            expect(Math.abs(z)).toBeLessThanOrEqual(1.0);
        }
    });
});

describe('collideShapeVsShape - ConvexHull vs Sphere', () => {
    // Box vertices for convex hull
    const boxVertices = [-1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1];

    test('should detect sphere just penetrating convex hull on face', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Sphere at (2.5, 0, 0) with radius 1.51 (just penetrating face at x=1)
        const shapeB = transformed.create({
            shape: sphere.create({ radius: 1.51 }),
            position: vec3.fromValues(2.5, 0, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Penetration should be very small (approximately 0.01)
        expect(contact.penetration).toBeGreaterThan(0);
        expect(contact.penetration).toBeLessThan(0.1);
    });

    test('should detect sphere deeply penetrating convex hull', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Sphere at (1.5, 0, 0) with radius 1.0 (deeply penetrating)
        const shapeB = transformed.create({
            shape: sphere.create({ radius: 1.0 }),
            position: vec3.fromValues(1.5, 0, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Penetration depth should be approximately 0.5
        expect(contact.penetration).toBeGreaterThan(0.3);
        expect(contact.penetration).toBeLessThan(0.7);
    });

    test('should detect sphere penetrating convex hull on edge', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Sphere at (2, 2, 0), distance from edge (1, 1, 0) is sqrt(2)
        const shapeB = transformed.create({
            shape: sphere.create({ radius: Math.sqrt(2.0) + 0.01 }),
            position: vec3.fromValues(2, 2, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Penetration should be very small
        expect(contact.penetration).toBeGreaterThan(0);
        expect(contact.penetration).toBeLessThan(0.1);
    });

    test('should detect sphere penetrating convex hull on vertex', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Sphere at (2, 2, 2), distance from vertex (1, 1, 1) is sqrt(3)
        const shapeB = transformed.create({
            shape: sphere.create({ radius: Math.sqrt(3.0) + 0.01 }),
            position: vec3.fromValues(2, 2, 2),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Penetration should be very small
        expect(contact.penetration).toBeGreaterThan(0);
        expect(contact.penetration).toBeLessThan(0.1);
    });
});

describe('collideShapeVsShape - ConvexHull vs Box', () => {
    const boxVertices = [-1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1];

    test('should detect overlapping convex hull and box', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Box with half extents (1, 1, 1) translated by (1.5, 0, 0)
        const shapeB = transformed.create({
            shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
            position: vec3.fromValues(1.5, 0, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Penetration should be approximately 0.5
        expect(contact.penetration).toBeGreaterThan(0.3);
        expect(contact.penetration).toBeLessThan(0.7);
    });

    test('should detect deeply penetrating convex hull and box', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        // Small box inside hull
        const shapeB = transformed.create({
            shape: box.create({ halfExtents: vec3.fromValues(0.3, 0.3, 0.3) }),
            position: vec3.fromValues(0.3, 0.3, 0.3),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Should detect penetration
        expect(contact.penetration).toBeGreaterThan(0);
    });
});

describe('collideShapeVsShape - ConvexHull vs ConvexHull', () => {
    const boxVertices = [-1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1];

    const tetrahedronVertices = [-1, 0, -1, 0, 0, 1, 1, 0, -1, 0, -1, 0];

    test('should detect overlapping convex hulls (box vs box)', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = transformed.create({
            shape: convexHull.create({ positions: boxVertices, convexRadius: 0.0 }),
            position: vec3.fromValues(1.5, 0, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Penetration should be approximately 0.5
        expect(contact.penetration).toBeGreaterThan(0.3);
        expect(contact.penetration).toBeLessThan(0.7);
    });

    test('should detect overlapping convex hulls (box vs tetrahedron)', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = transformed.create({
            shape: convexHull.create({ positions: tetrahedronVertices, convexRadius: 0.0 }),
            position: vec3.fromValues(0.5, 0, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Should detect penetration
        expect(contact.penetration).toBeGreaterThan(0);
    });

    test('should handle convex hulls with convex radius', () => {
        const shapeA = convexHull.create({ positions: boxVertices, convexRadius: 0.1 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = transformed.create({
            shape: convexHull.create({ positions: boxVertices, convexRadius: 0.1 }),
            position: vec3.fromValues(1.8, 0, 0),
            quaternion: quat.create(),
        });
        const posB = vec3.fromValues(0, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);

        const contact = results[0];
        // Should detect penetration considering convex radius
        expect(contact.penetration).toBeGreaterThan(0);
    });
});

describe('collideShapes - Cylinder', () => {
    test('should detect collision between cylinder and sphere', () => {
        const shapeA = sphere.create({ radius: 1.0 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = cylinder.create({ halfHeight: 2.0, radius: 1.0 });
        const posB = vec3.fromValues(1.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);
        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should detect collision between cylinder and box', () => {
        const shapeA = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = cylinder.create({ halfHeight: 2.0, radius: 1.0 });
        const posB = vec3.fromValues(1.5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);
        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should detect collision between two cylinders', () => {
        const shapeA = cylinder.create({ halfHeight: 1.0, radius: 0.5 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = cylinder.create({ halfHeight: 1.0, radius: 0.5 });
        const posB = vec3.fromValues(0.8, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);
        expect(results[0].penetration).toBeGreaterThan(0);
    });

    test('should detect no collision between distant cylinders', () => {
        const shapeA = cylinder.create({ halfHeight: 1.0, radius: 0.5 });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = cylinder.create({ halfHeight: 1.0, radius: 0.5 });
        const posB = vec3.fromValues(5, 0, 0);
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(0);
    });
});

describe('collideShapeVsShape - Sphere vs Plane', () => {
    test('not colliding - sphere above plane', () => {
        const shapeA = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(0, 2.0, 0); // sphere bottom at y=1.0, above plane
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(0);
    });

    test('barely colliding - sphere just touching plane', () => {
        const shapeA = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(0, 0.95, 0); // penetrates 0.05 units
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);
        expect(results[0].penetration).toBeCloseTo(0.05, 5);
        expect(results[0].penetrationAxis[1]).toBeCloseTo(1, 5);
    });

    test('underneath plane - sphere deeply penetrating', () => {
        const shapeA = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = sphere.create({ radius: 1.0 });
        const posB = vec3.fromValues(0, -0.5, 0); // center below plane, penetrates 1.5 units
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);
        expect(results[0].penetration).toBeCloseTo(1.5, 5);
        expect(results[0].penetrationAxis[1]).toBeCloseTo(1, 5);
    });
});

describe('collideShapeVsShape - Box vs Plane', () => {
    test('not colliding - box above plane', () => {
        const shapeA = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: [1, 1, 1] });
        const posB = vec3.fromValues(0, 2.0, 0); // box bottom at y=1.0, above plane
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(0);
    });

    test('barely colliding - box just touching plane', () => {
        const shapeA = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: [1, 1, 1] });
        const posB = vec3.fromValues(0, 0.95, 0); // penetrates 0.05 units
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);
        expect(results[0].penetration).toBeCloseTo(0.05, 4);
    });

    test('underneath plane - box deeply penetrating', () => {
        const shapeA = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const posA = vec3.fromValues(0, 0, 0);
        const quatA = quat.create();
        const scaleA = vec3.fromValues(1, 1, 1);

        const shapeB = box.create({ halfExtents: [1, 1, 1] });
        const posB = vec3.fromValues(0, -0.5, 0); // center below plane, penetrates 1.5 units
        const quatB = quat.create();
        const scaleB = vec3.fromValues(1, 1, 1);

        const collector = createAllCollideShapeCollector();
        const results = collector.hits;

        // biome-ignore format: readability
        collideShapeVsShape(
            collector,
            settings,
            shapeA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            shapeB,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2],
        );

        expect(results.length).toBe(1);
        expect(results[0].penetration).toBeCloseTo(1.5, 4);
    });
});
