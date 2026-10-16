import { mat4, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { createFace, cylinder, getShapeSupportingFace, ShapeType } from '../../src';
import { DEFAULT_CONVEX_RADIUS } from '../../src/collision/support';

describe('Cylinder creation', () => {
    test('should create a cylinder with default density and convex radius', () => {
        const c = cylinder.create({ halfHeight: 1, radius: 0.5 });
        expect(c.type).toBe(ShapeType.CYLINDER);
        expect(c.halfHeight).toBe(1);
        expect(c.radius).toBe(0.5);
        expect(c.convexRadius).toBe(DEFAULT_CONVEX_RADIUS);
        expect(c.density).toBe(1000);
    });

    test('should compute correct volume', () => {
        // V = pi * r^2 * 2h
        const c = cylinder.create({ halfHeight: 2, radius: 1 });
        expect(c.volume).toBeCloseTo(Math.PI * 1 * 1 * 4, 10);
    });

    test('should compute correct AABB', () => {
        const c = cylinder.create({ halfHeight: 2, radius: 1.5 });
        // aabb = [-radius, -halfHeight, -radius, radius, halfHeight, radius]
        expect(c.aabb[0]).toBe(-1.5);
        expect(c.aabb[1]).toBe(-2);
        expect(c.aabb[2]).toBe(-1.5);
        expect(c.aabb[3]).toBe(1.5);
        expect(c.aabb[4]).toBe(2);
        expect(c.aabb[5]).toBe(1.5);
    });

    test('center of mass should be at origin', () => {
        const c = cylinder.create({ halfHeight: 3, radius: 1 });
        expect(c.centerOfMass[0]).toBe(0);
        expect(c.centerOfMass[1]).toBe(0);
        expect(c.centerOfMass[2]).toBe(0);
    });
});

describe('Cylinder convex radius auto-clamping', () => {
    test('convex radius that fits should not be clamped', () => {
        // min(halfHeight, radius) = min(1, 0.5) = 0.5, convexRadius 0.1 fits
        const c = cylinder.create({ halfHeight: 1, radius: 0.5, convexRadius: 0.1 });
        expect(c.convexRadius).toBe(0.1);
    });

    test('default convex radius should be used when not specified', () => {
        const c = cylinder.create({ halfHeight: 1, radius: 0.5 });
        expect(c.convexRadius).toBe(DEFAULT_CONVEX_RADIUS);
    });

    test('oversized convex radius should be clamped to min(halfHeight, radius)', () => {
        // min(0.03, 0.5) = 0.03, convexRadius 1.0 is too large
        const c = cylinder.create({ halfHeight: 0.03, radius: 0.5, convexRadius: 1.0 });
        expect(c.convexRadius).toBe(0.03);
    });

    test('oversized convex radius should clamp to radius when radius is smallest', () => {
        // min(2, 0.02) = 0.02
        const c = cylinder.create({ halfHeight: 2, radius: 0.02, convexRadius: 1.0 });
        expect(c.convexRadius).toBe(0.02);
    });

    test('default convex radius should be clamped when larger than min dimension', () => {
        // min(0.01, 0.01) = 0.01 < DEFAULT_CONVEX_RADIUS (0.05)
        const c = cylinder.create({ halfHeight: 0.01, radius: 0.01 });
        expect(c.convexRadius).toBe(0.01);
    });

    test('zero-sized cylinder should clamp convex radius to zero', () => {
        const c = cylinder.create({ halfHeight: 0, radius: 0, convexRadius: 1.0 });
        expect(c.convexRadius).toBe(0);
    });

    test('convex radius exactly equal to min dimension should not be clamped', () => {
        const c = cylinder.create({ halfHeight: 0.3, radius: 0.5, convexRadius: 0.3 });
        expect(c.convexRadius).toBe(0.3);
    });
});

describe('Cylinder validation', () => {
    test('negative halfHeight should throw', () => {
        expect(() => cylinder.create({ halfHeight: -1, radius: 1 })).toThrow('cylinder halfHeight must be >= 0');
    });

    test('negative radius should throw', () => {
        expect(() => cylinder.create({ halfHeight: 1, radius: -1 })).toThrow('cylinder radius must be >= 0');
    });

    test('negative convex radius should throw', () => {
        expect(() => cylinder.create({ halfHeight: 1, radius: 1, convexRadius: -0.1 })).toThrow(
            'cylinder convexRadius must be >= 0',
        );
    });
});

describe('Cylinder supporting face', () => {
    test('a mirrored scale returns the same side line as the positive scale', () => {
        const shape = cylinder.create({ halfHeight: 1, radius: 0.5 });
        const direction = vec3.fromValues(-1, 0, 0);

        const positive = createFace();
        getShapeSupportingFace(positive, shape, 0, direction, mat4.create(), vec3.fromValues(2, 3, 2));
        const mirrored = createFace();
        getShapeSupportingFace(mirrored, shape, 0, direction, mat4.create(), vec3.fromValues(-2, -3, -2));

        // the line on the +x side, at x = 2 * radius, from y = +3h to y = -3h
        expect(mirrored.numVertices).toBe(2);
        expect(mirrored.vertices[0]).toBeCloseTo(1, 6);
        expect(mirrored.vertices[1]).toBeCloseTo(3, 6);
        expect(mirrored.vertices[3]).toBeCloseTo(1, 6);
        expect(mirrored.vertices[4]).toBeCloseTo(-3, 6);
        expect(mirrored.vertices.slice(0, 6)).toEqual(positive.vertices.slice(0, 6));
    });

    test('a mirrored scale returns the same tilted cap with the same winding as the positive scale', () => {
        const shape = cylinder.create({ halfHeight: 1, radius: 0.5 });
        const direction = vec3.fromValues(-0.3, -1, 0);

        const positive = createFace();
        getShapeSupportingFace(positive, shape, 0, direction, mat4.create(), vec3.fromValues(2, 3, 2));
        const mirrored = createFace();
        getShapeSupportingFace(mirrored, shape, 0, direction, mat4.create(), vec3.fromValues(-2, 3, -2));

        // the top cap at y = 3h, one vertex rotated towards +x (against the direction)
        expect(mirrored.numVertices).toBe(8);
        let maxX = Number.NEGATIVE_INFINITY;
        for (let i = 0; i < 8; i++) {
            expect(mirrored.vertices[i * 3 + 1]).toBeCloseTo(3, 6);
            maxX = Math.max(maxX, mirrored.vertices[i * 3]);
        }
        expect(maxX).toBeCloseTo(1, 6);
        for (let i = 0; i < 24; i++) {
            expect(mirrored.vertices[i]).toBeCloseTo(positive.vertices[i], 10);
        }
    });
});
