import { describe, expect, test } from 'vitest';
import { cylinder, ShapeType } from '../../src';
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
