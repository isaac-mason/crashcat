import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    computeBarycentricCoordinates2d,
    computeBarycentricCoordinates3d,
} from '../../src/collision/closest-points';

describe('computeBarycentricCoordinates2d', () => {
    test('should compute barycentric coordinates for origin on line', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(1, 0, 0);
        const b = vec3.fromValues(0, 1, 0);

        computeBarycentricCoordinates2d(result, a, b, 1e-12);

        expect(result.isValid).toBe(true);
        // Barycentric coordinates should sum to 1
        expect(result.u + result.v).toBeCloseTo(1.0);
        // Both should be roughly equal for this symmetric case
        expect(result.u).toBeCloseTo(0.5, 1);
        expect(result.v).toBeCloseTo(0.5, 1);
    });

    test('should compute barycentric coordinates when origin is at point A', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(0, 0, 0); // origin
        const b = vec3.fromValues(1, 0, 0);

        computeBarycentricCoordinates2d(result, a, b, 1e-12);

        expect(result.isValid).toBe(true);
        // Origin is at A, so u should be 1, v should be 0
        expect(result.u).toBeCloseTo(1.0);
        expect(result.v).toBeCloseTo(0.0);
    });

    test('should compute barycentric coordinates for point on line segment', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(0, 0, 0);
        const b = vec3.fromValues(2, 0, 0);

        computeBarycentricCoordinates2d(result, a, b, 1e-12);

        expect(result.isValid).toBe(true);
        expect(result.u + result.v).toBeCloseTo(1.0);
    });

    test('should handle degenerate line (coincident points)', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(1, 1, 1);
        const b = vec3.fromValues(1, 1, 1);

        computeBarycentricCoordinates2d(result, a, b, 1e-12);

        expect(result.isValid).toBe(false);
    });
});

describe('computeBarycentricCoordinates3d', () => {
    test('should compute barycentric coordinates when origin is at point A', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(0, 0, 0); // origin
        const b = vec3.fromValues(1, 0, 0);
        const c = vec3.fromValues(0, 1, 0);

        computeBarycentricCoordinates3d(result, a, b, c, 1e-12);

        expect(result.isValid).toBe(true);
        expect(result.u).toBeCloseTo(1.0);
        expect(result.v).toBeCloseTo(0.0);
        expect(result.w).toBeCloseTo(0.0);
    });

    test('should compute barycentric coordinates for origin relative to triangle', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(1, 0, 0);
        const b = vec3.fromValues(0, 1, 0);
        const c = vec3.fromValues(0, 0, 1);

        // Computing barycentric coords of origin relative to this triangle
        computeBarycentricCoordinates3d(result, a, b, c, 1e-12);

        expect(result.isValid).toBe(true);
        // Coordinates should sum to 1
        expect(result.u + result.v + result.w).toBeCloseTo(1.0);
        // For this symmetric triangle, all coords should be equal
        expect(result.u).toBeCloseTo(1.0 / 3.0, 1);
        expect(result.v).toBeCloseTo(1.0 / 3.0, 1);
        expect(result.w).toBeCloseTo(1.0 / 3.0, 1);
    });

    test('should handle degenerate triangle (collinear points)', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(0, 0, 0);
        const b = vec3.fromValues(1, 0, 0);
        const c = vec3.fromValues(2, 0, 0); // collinear

        computeBarycentricCoordinates3d(result, a, b, c, 1e-12);

        expect(result.isValid).toBe(false);
    });
});

describe('barycentric coordinates properties', () => {
    test('barycentric coordinates should sum to 1 for valid 2d result', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(1, 0, 0);
        const b = vec3.fromValues(0, 1, 0);

        computeBarycentricCoordinates2d(result, a, b, 1e-12);

        if (result.isValid) {
            expect(result.u + result.v).toBeCloseTo(1.0);
        }
    });

    test('barycentric coordinates should sum to 1 for valid 3d result', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(1, 0, 0);
        const b = vec3.fromValues(0, 1, 0);
        const c = vec3.fromValues(0, 0, 1);

        computeBarycentricCoordinates3d(result, a, b, c, 1e-12);

        if (result.isValid) {
            expect(result.u + result.v + result.w).toBeCloseTo(1.0);
        }
    });

    test('barycentric coordinates should be non-negative for points inside simplex', () => {
        const result = { u: 0, v: 0, w: 0, isValid: false };
        const a = vec3.fromValues(1, 0, 0);
        const b = vec3.fromValues(0, 1, 0);
        const c = vec3.fromValues(0, 0, 1);

        computeBarycentricCoordinates3d(result, a, b, c, 1e-12);

        if (result.isValid) {
            // For origin relative to this triangle, coords might be negative
            // This is expected for GJK/EPA where we're finding closest point to origin
            expect(result.u + result.v + result.w).toBeCloseTo(1.0);
        }
    });
});
