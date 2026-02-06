import { describe, expect, test } from 'vitest';
import { computeMassProperties, massProperties, ShapeType, triangleMesh } from '../../src';

describe('Triangle mesh AABB caching', () => {
    test('triangle mesh should have correct AABB', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });
        expect(mesh.aabb).toBeDefined();
        // Triangle vertices: (0,0,0), (1,0,0), (0,1,0)
        // AABB should be [0,0,0] to [1,1,0]
        expect(mesh.aabb[0][0]).toBe(0);
        expect(mesh.aabb[0][1]).toBe(0);
        expect(mesh.aabb[0][2]).toBe(0);
        expect(mesh.aabb[1][0]).toBe(1);
        expect(mesh.aabb[1][1]).toBe(1);
        expect(mesh.aabb[1][2]).toBe(0);
    });
});

describe('Triangle mesh creation and mass properties', () => {
    test('should create a triangle mesh', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        expect(mesh.type).toBe(ShapeType.TRIANGLE_MESH);
        expect(mesh.data.triangleCount).toBe(1);
        expect(mesh.bvh).toBeDefined();
    });

    test('should compute triangle mesh as static (zero mass)', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        const props = massProperties.create();
        computeMassProperties(props, mesh);

        // Triangle meshes are static geometry by default
        expect(props.mass).toBe(0);
    });

    test('should handle triangle mesh with multiple triangles', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1],
            indices: [0, 1, 2, 1, 2, 3],
        });

        expect(mesh.data.triangleCount).toBe(2);
        expect(mesh.bvh).toBeDefined();
    });
});

describe('Triangle mesh center of mass', () => {
    test('triangle mesh should have center of mass at origin', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        // currently returns zero
        expect(mesh.centerOfMass[0]).toBe(0);
        expect(mesh.centerOfMass[1]).toBe(0);
        expect(mesh.centerOfMass[2]).toBe(0);
    });
});
