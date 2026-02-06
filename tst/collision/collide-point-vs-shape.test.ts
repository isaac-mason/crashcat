import { euler, quat, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    box,
    collidePointVsShape,
    compound,
    createAllCollidePointCollector,
    createAnyCollidePointCollector,
    createDefaultCollidePointSettings,
    EMPTY_SUB_SHAPE_ID,
    plane,
    scaled,
    sphere,
    transformed,
    triangleMesh,
} from '../../src';

const settings = createDefaultCollidePointSettings();

describe('collidePointVsShape - Sphere', () => {
    test('should detect point inside sphere at origin', () => {
        const shape = sphere.create({ radius: 1.0 });
        const point = vec3.fromValues(0, 0, 0);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 42;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
        expect(collector.hit?.bodyIdB).toBe(42);
        expect(collector.hit?.subShapeIdB).toBe(EMPTY_SUB_SHAPE_ID);
    });

    test('should detect point inside sphere offset from origin', () => {
        const shape = sphere.create({ radius: 2.0 });
        const point = vec3.fromValues(5, 3, -2);
        const pos = vec3.fromValues(5, 3, -2);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should detect point on sphere surface', () => {
        const shape = sphere.create({ radius: 1.0 });
        const point = vec3.fromValues(1, 0, 0); // exactly on surface
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should not detect point outside sphere', () => {
        const shape = sphere.create({ radius: 1.0 });
        const point = vec3.fromValues(2, 0, 0); // outside sphere
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).toBeNull();
    });

    test('should handle scaled sphere', () => {
        const shape = sphere.create({ radius: 1.0 });
        const point = vec3.fromValues(1.5, 0, 0); // inside scaled sphere
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(2, 2, 2); // radius becomes 2.0

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should not detect point outside scaled sphere', () => {
        const shape = sphere.create({ radius: 1.0 });
        const point = vec3.fromValues(1.5, 0, 0); // outside unscaled sphere
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).toBeNull();
    });

    test('should handle translated sphere', () => {
        const shape = sphere.create({ radius: 1.0 });
        const point = vec3.fromValues(5, 3, -2);
        const pos = vec3.fromValues(5, 3, -2); // sphere center at point
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should use AllCollidePointCollector correctly', () => {
        const shape = sphere.create({ radius: 1.0 });
        const point = vec3.fromValues(0, 0, 0);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAllCollidePointCollector();
        collector.bodyIdB = 99;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hits.length).toBe(1);
        expect(collector.hits[0].bodyIdB).toBe(99);
        expect(collector.hits[0].subShapeIdB).toBe(EMPTY_SUB_SHAPE_ID);
    });
});

describe('collidePointVsShape - Box', () => {
    test('should detect point inside box at origin', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(0, 0, 0);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
        expect(collector.hit?.bodyIdB).toBe(1);
    });

    test('should detect point inside box (off-center)', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(2, 2, 2) });
        const point = vec3.fromValues(1, 1, 1);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should detect point on box surface', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(1, 0, 0); // exactly on +X face
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should detect point on box corner', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(1, 1, 1); // corner
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should not detect point outside box', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(2, 0, 0); // outside box
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).toBeNull();
    });

    test('should handle rotated box (45 degrees around Y)', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(1.4, 0, 0); // would be outside unrotated box
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        quat.fromEuler(rot, euler.fromDegrees(euler.create(), 0, 45, 0, 'xyz'));
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should handle rotated box - point still inside after rotation', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(0.5, 0, 0); // inside box
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        quat.fromEuler(rot, euler.fromDegrees(euler.create(), 0, 90, 0, 'xyz'));
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        // After 90 degree rotation, point at (0.5, 0, 0) should still be inside
        expect(collector.hit).not.toBeNull();
    });

    test('should handle scaled box', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(1.5, 0, 0); // outside unscaled box
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(2, 2, 2); // half-extents become (2, 2, 2)

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should handle non-uniform scale', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(0, 2, 0); // outside unscaled box
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 3, 1); // stretch Y axis

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should handle translated box', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(5, 3, -2);
        const pos = vec3.fromValues(5, 3, -2); // box center at point
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should handle translated and rotated box', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(11, 5, 3); // inside translated+rotated box
        const pos = vec3.fromValues(10, 5, 3);
        const rot = quat.create();
        quat.fromEuler(rot, euler.fromDegrees(euler.create(), 0, 45, 0, 'xyz'));
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hit).not.toBeNull();
    });

    test('should handle negative scale', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(0.5, 0, 0);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(-1, -1, -1); // negative scale

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        // Negative scale should still work (absolute value used)
        expect(collector.hit).not.toBeNull();
    });

    test('should use AllCollidePointCollector correctly', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const point = vec3.fromValues(0, 0, 0);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAllCollidePointCollector();
        collector.bodyIdB = 99;

        // biome-ignore format: readability
        collidePointVsShape(
			collector,
			settings,
			point[0], point[1], point[2],
			shape,
			EMPTY_SUB_SHAPE_ID, 0,
			pos[0], pos[1], pos[2],
			rot[0], rot[1], rot[2], rot[3],
			scale[0], scale[1], scale[2],
		);

        expect(collector.hits.length).toBe(1);
        expect(collector.hits[0].bodyIdB).toBe(99);
        expect(collector.hits[0].subShapeIdB).toBe(EMPTY_SUB_SHAPE_ID);
    });
});

describe('collidePointVsShape - TriangleMesh', () => {
    const createRandomizedCubeVertices = () => {
        // biome-ignore format: readability
        const baseCubeVertices = [
            [-1.0, -1.0, -1.0],
            [ 1.0, -1.0, -1.0],
            [-1.0, -1.0,  1.0],
            [ 1.0, -1.0,  1.0],
            [-1.0,  1.0, -1.0],
            [ 1.0,  1.0, -1.0],
            [-1.0,  1.0,  1.0],
            [ 1.0,  1.0,  1.0]
        ];

        const cubeVertices: number[] = [];
        for (const [x, y, z] of baseCubeVertices) {
            // Range 0.1 to 0.3, like uniform_real_distribution<float> range(0.1f, 0.3f)
            const rx = 0.1 + Math.random() * 0.2;
            const ry = 0.1 + Math.random() * 0.2;
            const rz = 0.1 + Math.random() * 0.2;
            cubeVertices.push(x * rx, y * ry, z * rz);
        }
        return cubeVertices;
    };

    const cubeIndices = [
        0, 1, 3, 0, 3, 2, 4, 7, 5, 4, 6, 7, 2, 3, 6, 3, 7, 6, 1, 0, 4, 1, 4, 5, 1, 7, 3, 1, 5, 7, 0, 2, 6, 0, 6, 4,
    ];

    test('should detect point inside simple cube mesh', () => {
        const mesh = triangleMesh.create({
            positions: createRandomizedCubeVertices(),
            indices: cubeIndices,
        });

        const point = vec3.fromValues(0, 0, 0); // center of cube
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
        expect(collector.hit?.bodyIdB).toBe(1);
    });

    test('should not detect point outside cube mesh', () => {
        const mesh = triangleMesh.create({
            positions: [
                -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1,
                1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1,
                1, -1, 1, 1, -1, 1, -1,
            ],
            indices: [
                0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11, 12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21,
                22, 20, 22, 23,
            ],
        });

        const point = vec3.fromValues(5, 0, 0); // far outside
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
collector,
settings,
point[0], point[1], point[2],
mesh,
EMPTY_SUB_SHAPE_ID, 0,
pos[0], pos[1], pos[2],
rot[0], rot[1], rot[2], rot[3],
scale[0], scale[1], scale[2],
);

        expect(collector.hit).toBeNull();
    });

    test('should detect point inside single triangle mesh (open)', () => {
        // Single triangle in XZ plane at Y=0
        const mesh = triangleMesh.create({
            positions: [
                -1,
                0,
                -1, // vertex 0
                1,
                0,
                -1, // vertex 1
                0,
                0,
                1, // vertex 2
            ],
            indices: [0, 1, 2],
        });

        const point = vec3.fromValues(0, 0, 0); // center of triangle
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        // Single open triangle - point should not be detected as "inside"
        // (requires closed manifold for odd-even rule to work correctly)
    });

    test('should handle translated triangle mesh', () => {
        const mesh = triangleMesh.create({
            positions: createRandomizedCubeVertices(),
            indices: cubeIndices,
        });

        const point = vec3.fromValues(10, 5, 3); // translated position
        const pos = vec3.fromValues(10, 5, 3); // mesh center at same point
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should handle scaled triangle mesh', () => {
        const mesh = triangleMesh.create({
            positions: [
                -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1,
                1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1,
                1, -1, 1, 1, -1, 1, -1,
            ],
            indices: [
                0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11, 12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21,
                22, 20, 22, 23,
            ],
        });

        const point = vec3.fromValues(1.5, 0, 0); // outside unit cube, inside scaled
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(2, 2, 2); // scale up to [-2,2] cube

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should handle rotated triangle mesh', () => {
        const mesh = triangleMesh.create({
            positions: createRandomizedCubeVertices(),
            indices: cubeIndices,
        });

        const point = vec3.fromValues(0, 0, 0); // center
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        quat.fromEuler(rot, euler.fromDegrees(euler.create(), 0, 45, 0, 'xyz'));
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        // Center should still be inside rotated cube
        expect(collector.hit).not.toBeNull();
    });

    test('should use AllCollidePointCollector correctly', () => {
        const mesh = triangleMesh.create({
            positions: createRandomizedCubeVertices(),
            indices: cubeIndices,
        });

        const point = vec3.fromValues(0, 0, 0);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAllCollidePointCollector();
        collector.bodyIdB = 99;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hits.length).toBe(1);
        expect(collector.hits[0].bodyIdB).toBe(99);
    });

    test('should handle point near mesh surface', () => {
        const mesh = triangleMesh.create({
            positions: [
                -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1,
                1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1,
                1, -1, 1, 1, -1, 1, -1,
            ],
            indices: [
                0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11, 12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21,
                22, 20, 22, 23,
            ],
        });

        const point = vec3.fromValues(0.9, 0, 0); // near +X face but inside
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });
});

describe('collidePointVsShape - CompoundShape', () => {
    test('should detect point inside compound shape with multiple spheres', () => {
        const sphere1 = sphere.create({ radius: 1.0 });
        const sphere2 = sphere.create({ radius: 0.5 });

        const compoundShape = compound.create({
            children: [
                {
                    shape: sphere1,
                    position: vec3.fromValues(0, 0, 0),
                    quaternion: quat.create(),
                },
                {
                    shape: sphere2,
                    position: vec3.fromValues(2, 0, 0),
                    quaternion: quat.create(),
                },
            ],
        });

        const point = vec3.fromValues(0.5, 0, 0); // inside first sphere
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            compoundShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should detect point inside second sub-shape of compound', () => {
        const sphere1 = sphere.create({ radius: 0.5 });
        const sphere2 = sphere.create({ radius: 1.0 });

        const compoundShape = compound.create({
            children: [
                {
                    shape: sphere1,
                    position: vec3.fromValues(-2, 0, 0),
                    quaternion: quat.create(),
                },
                {
                    shape: sphere2,
                    position: vec3.fromValues(2, 0, 0),
                    quaternion: quat.create(),
                },
            ],
        });

        const point = vec3.fromValues(2.5, 0, 0); // inside second sphere
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            compoundShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should not detect point outside all sub-shapes', () => {
        const sphere1 = sphere.create({ radius: 0.5 });
        const sphere2 = sphere.create({ radius: 0.5 });

        const compoundShape = compound.create({
            children: [
                {
                    shape: sphere1,
                    position: vec3.fromValues(-2, 0, 0),
                    quaternion: quat.create(),
                },
                {
                    shape: sphere2,
                    position: vec3.fromValues(2, 0, 0),
                    quaternion: quat.create(),
                },
            ],
        });

        const point = vec3.fromValues(0, 5, 0); // outside all spheres
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            compoundShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).toBeNull();
    });

    test('should detect point with compound shape using AllCollector', () => {
        const box1 = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const box2 = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        const compoundShape = compound.create({
            children: [
                {
                    shape: box1,
                    position: vec3.fromValues(0, 0, 0),
                    quaternion: quat.create(),
                },
                {
                    shape: box2,
                    position: vec3.fromValues(0.3, 0, 0),
                    quaternion: quat.create(),
                },
            ],
        });

        const point = vec3.fromValues(0.2, 0, 0); // inside overlapping region
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAllCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            compoundShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hits.length).toBeGreaterThan(0);
    });
});

describe('collidePointVsShape - TransformedShape', () => {
    test('should detect point inside transformed sphere', () => {
        const innerSphere = sphere.create({ radius: 1.0 });
        const transformedShape = transformed.create({
            shape: innerSphere,
            position: vec3.fromValues(5, 3, -2),
            quaternion: quat.create(),
        });

        const point = vec3.fromValues(5.5, 3, -2); // inside transformed sphere
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            transformedShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should detect point inside transformed and rotated box', () => {
        const innerBox = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const rotation = quat.fromEuler(quat.create(), euler.fromValues(0, Math.PI / 4, 0, 'xyz'));

        const transformedShape = transformed.create({
            shape: innerBox,
            position: vec3.fromValues(0, 0, 0),
            quaternion: rotation,
        });

        // Point that's well inside the box regardless of rotation
        const point = vec3.fromValues(0.2, 0.2, 0.2);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            transformedShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should not detect point outside transformed shape', () => {
        const innerSphere = sphere.create({ radius: 1.0 });
        const transformedShape = transformed.create({
            shape: innerSphere,
            position: vec3.fromValues(10, 0, 0),
            quaternion: quat.create(),
        });

        const point = vec3.fromValues(0, 0, 0); // far from transformed sphere
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            transformedShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).toBeNull();
    });

    test('should handle transformed shape with body transformation', () => {
        const innerSphere = sphere.create({ radius: 1.0 });
        const transformedShape = transformed.create({
            shape: innerSphere,
            position: vec3.fromValues(2, 0, 0),
            quaternion: quat.create(),
        });

        const point = vec3.fromValues(7.5, 0, 0);
        const pos = vec3.fromValues(5, 0, 0); // body offset
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            transformedShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });
});

describe('collidePointVsShape - ScaledShape', () => {
    test('should detect point inside scaled up sphere', () => {
        const innerSphere = sphere.create({ radius: 1.0 });
        const scaledShape = scaled.create({
            shape: innerSphere,
            scale: vec3.fromValues(2, 2, 2),
        });

        const point = vec3.fromValues(1.5, 0, 0); // inside scaled sphere (radius 2)
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            scaledShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should detect point inside non-uniformly scaled box', () => {
        const innerBox = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const scaledShape = scaled.create({
            shape: innerBox,
            scale: vec3.fromValues(2, 0.5, 1),
        });

        // Point inside the non-uniformly scaled box (2x1x2 half extents)
        const point = vec3.fromValues(1.5, 0.3, 0.5);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            scaledShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should not detect point outside scaled sphere', () => {
        const innerSphere = sphere.create({ radius: 1.0 });
        const scaledShape = scaled.create({
            shape: innerSphere,
            scale: vec3.fromValues(0.5, 0.5, 0.5),
        });

        const point = vec3.fromValues(0.8, 0, 0); // outside scaled sphere (radius 0.5)
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            scaledShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).toBeNull();
    });

    test('should handle scaled shape with body scale', () => {
        const innerBox = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const scaledShape = scaled.create({
            shape: innerBox,
            scale: vec3.fromValues(2, 2, 2),
        });

        // Scaled box has half extents [2, 2, 2]
        // With body scale of 2, effective half extents are [4, 4, 4]
        // Point at (3, 0, 0) should be inside
        const point = vec3.fromValues(3, 0, 0);
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const bodyScale = vec3.fromValues(2, 2, 2);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            scaledShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            bodyScale[0], bodyScale[1], bodyScale[2],
        );

        expect(collector.hit).not.toBeNull();
    });

    test('should detect point on surface of scaled shape', () => {
        const innerSphere = sphere.create({ radius: 1.0 });
        const scaledShape = scaled.create({
            shape: innerSphere,
            scale: vec3.fromValues(3, 3, 3),
        });

        const point = vec3.fromValues(3, 0, 0); // exactly on surface
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            scaledShape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
    });
});

describe('collidePointVsShape - Plane', () => {
    test('should detect point in solid half-space (below plane)', () => {
        const shape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const point = vec3.fromValues(0, -1, 0); // below plane (in solid half-space)
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            shape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).not.toBeNull();
        expect(collector.hit?.bodyIdB).toBe(1);
    });

    test('should not detect point in empty half-space (above plane)', () => {
        const shape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
        const point = vec3.fromValues(0, 1, 0); // above plane (in empty half-space)
        const pos = vec3.fromValues(0, 0, 0);
        const rot = quat.create();
        const scale = vec3.fromValues(1, 1, 1);

        const collector = createAnyCollidePointCollector();
        collector.bodyIdB = 1;

        // biome-ignore format: readability
        collidePointVsShape(
            collector,
            settings,
            point[0], point[1], point[2],
            shape,
            EMPTY_SUB_SHAPE_ID, 0,
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2], rot[3],
            scale[0], scale[1], scale[2],
        );

        expect(collector.hit).toBeNull();
    });
});
