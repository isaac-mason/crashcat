import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, computeMassProperties, ShapeType } from '../../src';
import * as massProperties from '../../src/body/mass-properties';
import { DEFAULT_CONVEX_RADIUS } from '../../src/collision/support';

describe('Box AABB caching', () => {
    test('box should have correct AABB', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        expect(b.aabb).toBeDefined();
        // AABB should be [-he[i], -he[i], -he[i]] to [he[i], he[i], he[i]]
        expect(b.aabb[0]).toBe(-1);
        expect(b.aabb[1]).toBe(-2);
        expect(b.aabb[2]).toBe(-3);
        expect(b.aabb[3]).toBe(1);
        expect(b.aabb[4]).toBe(2);
        expect(b.aabb[5]).toBe(3);
    });
});

describe('Box creation and mass properties', () => {
    test('should create a box with default density', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        expect(b.type).toBe(ShapeType.BOX);
        expect(b.halfExtents[0]).toBe(1);
        expect(b.halfExtents[1]).toBe(2);
        expect(b.halfExtents[2]).toBe(3);
        expect(b.density).toBe(1000);
    });

    test('should compute box mass properties', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1), density: 1000 });
        const props = massProperties.create();
        computeMassProperties(props, b);

        // For a box with halfExtents [1,1,1]: volume = 8, mass = 8*density
        expect(props.mass).toBeCloseTo(8000, 0);
    });
});

describe('Box center of mass', () => {
    test('box should have center of mass at origin', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        expect(b.centerOfMass[0]).toBe(0);
        expect(b.centerOfMass[1]).toBe(0);
        expect(b.centerOfMass[2]).toBe(0);
    });
});

describe('Box convex radius auto-clamping', () => {
    test('convex radius that fits should not be clamped', () => {
        // halfExtents min is 0.5, convexRadius 0.1 fits fine
        const b = box.create({ halfExtents: vec3.fromValues(0.5, 1, 2), convexRadius: 0.1 });
        expect(b.convexRadius).toBe(0.1);
    });

    test('default convex radius should be used when not specified', () => {
        // halfExtents min is 1.0, DEFAULT_CONVEX_RADIUS (0.05) fits
        const b = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        expect(b.convexRadius).toBe(DEFAULT_CONVEX_RADIUS);
    });

    test('oversized convex radius should be clamped to min half extent', () => {
        // halfExtents min is 0.02, convexRadius 1.0 is way too large
        const b = box.create({ halfExtents: vec3.fromValues(0.02, 1, 2), convexRadius: 1.0 });
        expect(b.convexRadius).toBe(0.02);
    });

    test('default convex radius should be clamped when larger than min half extent', () => {
        // halfExtents min is 0.01 < DEFAULT_CONVEX_RADIUS (0.05)
        const b = box.create({ halfExtents: vec3.fromValues(0.01, 1, 2) });
        expect(b.convexRadius).toBe(0.01);
    });

    test('zero-sized box should clamp convex radius to zero', () => {
        const b = box.create({ halfExtents: vec3.fromValues(0, 0, 0), convexRadius: 1.0 });
        expect(b.convexRadius).toBe(0);
    });

    test('convex radius exactly equal to min half extent should not be clamped', () => {
        const b = box.create({ halfExtents: vec3.fromValues(0.3, 1, 2), convexRadius: 0.3 });
        expect(b.convexRadius).toBe(0.3);
    });
});

describe('Box validation', () => {
    test('negative half extent x should throw', () => {
        expect(() => box.create({ halfExtents: vec3.fromValues(-1, 1, 1) })).toThrow('box halfExtents must be >= 0');
    });

    test('negative half extent y should throw', () => {
        expect(() => box.create({ halfExtents: vec3.fromValues(1, -1, 1) })).toThrow('box halfExtents must be >= 0');
    });

    test('negative half extent z should throw', () => {
        expect(() => box.create({ halfExtents: vec3.fromValues(1, 1, -1) })).toThrow('box halfExtents must be >= 0');
    });

    test('negative convex radius should throw', () => {
        expect(() => box.create({ halfExtents: vec3.fromValues(1, 1, 1), convexRadius: -0.1 })).toThrow(
            'box convexRadius must be >= 0',
        );
    });
});
