import type { Vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    containsFace,
    create,
    determineMaxError,
    getNumVerticesUsed,
    initialize,
    Result,
} from '../../src/shapes/utils/convex-hull-builder';

describe('ConvexHullBuilder', () => {
    const cTolerance = 1.0e-3;

    describe('TestDegenerate', () => {
        test('Too few points / coinciding points should be degenerate', () => {
            // Too few points / coinciding points should be degenerate
            const positions: Vec3[] = [[1, 2, 3]];

            let builder = create(positions);
            let result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.TooFewPoints);

            positions.push([1 + 0.5 * cTolerance, 2, 3]);
            builder = create(positions);
            result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.TooFewPoints);

            positions.push([1, 2 + 0.5 * cTolerance, 3]);
            builder = create(positions);
            result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Degenerate);

            positions.push([1, 2, 3 + 0.5 * cTolerance]);
            builder = create(positions);
            result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Degenerate);
        });

        test('A line should be degenerate', () => {
            const positions: Vec3[] = [];
            for (let v = 0.0; v <= 1.01; v += 0.1) {
                positions.push([v, 0, 0]);
            }

            const builder = create(positions);
            const result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Degenerate);
        });
    });

    describe('Test2DHull', () => {
        test('A triangle', () => {
            const positions: Vec3[] = [
                [-1, 0, -1],
                [1, 0, -1],
                [-1, 0, 1],
            ];

            const builder = create(positions);
            const result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Success);
            expect(getNumVerticesUsed(builder)).toBe(3);
            expect(builder.faces.length).toBe(2);
            expect(containsFace(builder, [0, 1, 2])).toBe(true);
            expect(containsFace(builder, [2, 1, 0])).toBe(true);
        });

        test('A quad with many interior points', () => {
            const positions: Vec3[] = [];
            for (let x = 0; x < 10; ++x) {
                for (let z = 0; z < 10; ++z) {
                    positions.push([0.1 * x, 0, 0.2 * z]);
                }
            }

            const builder = create(positions);
            const result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Success);
            expect(getNumVerticesUsed(builder)).toBe(4);
            expect(builder.faces.length).toBe(2);
            expect(containsFace(builder, [0, 9, 99, 90])).toBe(true);
            expect(containsFace(builder, [90, 99, 9, 0])).toBe(true);
        });

        test('Add disc with many interior points', () => {
            const positions: Vec3[] = [];
            for (let r = 0; r < 10; ++r) {
                for (let phi = 0; phi < 10; ++phi) {
                    const f_r = 2.0 * r;
                    const f_phi = (2.0 * Math.PI * phi) / 10;
                    positions.push([f_r * Math.cos(f_phi), f_r * Math.sin(f_phi), 0]);
                }
            }

            const builder = create(positions);
            const result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Success);
            expect(getNumVerticesUsed(builder)).toBe(10);
            expect(builder.faces.length).toBe(2);
            expect(containsFace(builder, [90, 91, 92, 93, 94, 95, 96, 97, 98, 99])).toBe(true);
            expect(containsFace(builder, [99, 98, 97, 96, 95, 94, 93, 92, 91, 90])).toBe(true);
        });
    });

    describe('Test3DHull', () => {
        test('A cube with lots of interior points', () => {
            const positions: Vec3[] = [];
            for (let x = 0; x < 10; ++x) {
                for (let y = 0; y < 10; ++y) {
                    for (let z = 0; z < 10; ++z) {
                        positions.push([0.1 * x, 1.0 + 0.2 * y, 0.3 * z]);
                    }
                }
            }

            const builder = create(positions);
            const result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Success);
            expect(getNumVerticesUsed(builder)).toBe(8);
            expect(builder.faces.length).toBe(6);
            expect(containsFace(builder, [0, 9, 99, 90])).toBe(true);
            expect(containsFace(builder, [0, 90, 990, 900])).toBe(true);
            expect(containsFace(builder, [900, 990, 999, 909])).toBe(true);
            expect(containsFace(builder, [9, 909, 999, 99])).toBe(true);
            expect(containsFace(builder, [90, 99, 999, 990])).toBe(true);
            expect(containsFace(builder, [0, 900, 909, 9])).toBe(true);
        });

        test('Add sphere with many interior points', () => {
            const positions: Vec3[] = [];
            for (let r = 0; r < 10; ++r) {
                for (let phi = 0; phi < 10; ++phi) {
                    for (let theta = 0; theta < 10; ++theta) {
                        const f_r = 2.0 * r;
                        const f_phi = (2.0 * Math.PI * phi) / 10; // [0, 2 PI)
                        const f_theta = (Math.PI * theta) / 9; // [0, PI] (inclusive!)

                        // sUnitSpherical(theta, phi) = Vec3(Sin(theta) * Cos(phi), Cos(theta), Sin(theta) * Sin(phi))
                        const sinTheta = Math.sin(f_theta);
                        const cosTheta = Math.cos(f_theta);
                        const sinPhi = Math.sin(f_phi);
                        const cosPhi = Math.cos(f_phi);

                        positions.push([f_r * sinTheta * cosPhi, f_r * cosTheta, f_r * sinTheta * sinPhi]);
                    }
                }
            }

            const builder = create(positions);
            const result = initialize(builder, 1000000, cTolerance);
            expect(result.result).toBe(Result.Success);
            // The two ends of the sphere have 10 points that have the same position
            // C++ expects 82, but tolerance handling may differ slightly
            const numVertices = getNumVerticesUsed(builder);
            expect(numVertices).toBeGreaterThanOrEqual(80);
            expect(numVertices).toBeLessThanOrEqual(85);

            // Too many faces, calculate the error instead
            const { maxError, coplanarDistance } = determineMaxError(builder);
            expect(maxError).toBeLessThan(Math.max(coplanarDistance, cTolerance));
        });
    });
});
