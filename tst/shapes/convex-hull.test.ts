import { describe, expect, test } from 'vitest';
import { convexHull } from '../../src';

describe('ConvexHull - Convex Radius Reduction', () => {
    describe('Pass 1: Hull Thickness Check', () => {
        test('reduces radius for thin pancake hull', () => {
            const thinHull = convexHull.create({
                positions: [
                    // Very thin disk (0.02 thick)
                    -10, -10, 0.01,
                    10, -10, 0.01,
                    10, 10, 0.01,
                    -10, 10, 0.01,
                    -10, -10, -0.01,
                    10, -10, -0.01,
                    10, 10, -0.01,
                    -10, 10, -0.01,
                ],
                convexRadius: 5, // Too large for thickness
            });

            // Radius should be reduced to fit 2× in thickness (~0.02)
            expect(thinHull.convexRadius).toBeLessThan(0.02);
            expect(thinHull.convexRadius).toBeGreaterThan(0);
        });

        test('reduces radius for cube with sharp corners', () => {
            const cube = convexHull.create({
                positions: [
                    -1, -1, -1,
                    1, -1, -1,
                    1, 1, -1,
                    -1, 1, -1,
                    -1, -1, 1,
                    1, -1, 1,
                    1, 1, 1,
                    -1, 1, 1,
                ],
                convexRadius: 0.1,
            });

            // Radius should be reduced due to sharp 90° corners (Pass 2)
            expect(cube.convexRadius).toBeLessThan(0.1);
            expect(cube.convexRadius).toBeGreaterThan(0.05);
        });

        test('preserves radius with larger error tolerance', () => {
            const cube = convexHull.create({
                positions: [
                    -1, -1, -1,
                    1, -1, -1,
                    1, 1, -1,
                    -1, 1, -1,
                    -1, -1, 1,
                    1, -1, 1,
                    1, 1, 1,
                    -1, 1, 1,
                ],
                convexRadius: 0.1,
                maxErrorConvexRadius: 1.0, // Allow more vertex shift
            });

            // With high error tolerance, radius can be preserved
            expect(cube.convexRadius).toBeCloseTo(0.1, 3);
        });
    });

    describe('Pass 2: Sharp Edge Check', () => {
        test('reduces radius for sharp pyramid apex', () => {
            const pyramid = convexHull.create({
                positions: [
                    // Sharp pyramid
                    0, 10, 0, // Sharp apex
                    -5, 0, -5,
                    -5, 0, 5,
                    5, 0, -5,
                    5, 0, 5,
                ],
                convexRadius: 2,
                maxErrorConvexRadius: 0.05,
            });

            // Radius should be reduced due to sharp edges
            expect(pyramid.convexRadius).toBeLessThan(2);
        });

        test('reduces radius more with stricter error tolerance', () => {
            const pyramidPositions = [
                0, 10, 0,
                -5, 0, -5,
                -5, 0, 5,
                5, 0, -5,
                5, 0, 5,
            ];

            const strictTolerance = convexHull.create({
                positions: pyramidPositions,
                convexRadius: 2,
                maxErrorConvexRadius: 0.01, // Stricter
            });

            const lenientTolerance = convexHull.create({
                positions: pyramidPositions,
                convexRadius: 2,
                maxErrorConvexRadius: 0.1, // More lenient
            });

            // Stricter tolerance should result in smaller radius
            expect(strictTolerance.convexRadius).toBeLessThan(lenientTolerance.convexRadius);
        });

        test('handles very sharp needle', () => {
            const needle = convexHull.create({
                positions: [
                    // Extremely sharp needle
                    0, 20, 0, // Very sharp tip
                    -0.5, 0, -0.5,
                    -0.5, 0, 0.5,
                    0.5, 0, -0.5,
                    0.5, 0, 0.5,
                ],
                convexRadius: 1,
                maxErrorConvexRadius: 0.05,
            });

            // Radius should be significantly reduced
            expect(needle.convexRadius).toBeLessThan(0.3);
            expect(needle.convexRadius).toBeGreaterThanOrEqual(0);
        });
    });

    describe('Edge Cases', () => {
        test('skips reduction when radius is zero', () => {
            const hull = convexHull.create({
                positions: [
                    -1, -1, -1,
                    1, -1, -1,
                    1, 1, -1,
                    -1, 1, -1,
                    -1, -1, 1,
                    1, -1, 1,
                    1, 1, 1,
                    -1, 1, 1,
                ],
                convexRadius: 0,
            });

            expect(hull.convexRadius).toBe(0);
        });

        test('handles hull with interior points', () => {
            const hull = convexHull.create({
                positions: [
                    // Cube vertices
                    -1, -1, -1,
                    1, -1, -1,
                    1, 1, -1,
                    -1, 1, -1,
                    -1, -1, 1,
                    1, -1, 1,
                    1, 1, 1,
                    -1, 1, 1,
                    // Interior points (should be filtered by builder)
                    0, 0, 0,
                    0.5, 0.5, 0.5,
                ],
                convexRadius: 0.1,
                maxErrorConvexRadius: 1.0, // Allow sharp corners
            });

            // Interior points should be filtered, radius preserved with high tolerance
            expect(hull.points.length).toBe(8);
            expect(hull.convexRadius).toBeCloseTo(0.1, 3);
        });

        test('uses default maxErrorConvexRadius when not specified', () => {
            const hull = convexHull.create({
                positions: [
                    0, 10, 0,
                    -5, 0, -5,
                    -5, 0, 5,
                    5, 0, -5,
                    5, 0, 5,
                ],
                convexRadius: 2,
                // maxErrorConvexRadius not specified, should default to 0.05
            });

            // Should still reduce radius based on default tolerance
            expect(hull.convexRadius).toBeLessThanOrEqual(2);
        });
    });

    describe('Numerical Stability', () => {
        test('forces radius to zero for degenerate/coplanar planes', () => {
            // Create a very flat hull where planes might be nearly coplanar
            const flatHull = convexHull.create({
                positions: [
                    -10, -10, 0.001,
                    10, -10, 0.001,
                    10, 10, 0.001,
                    -10, 10, 0.001,
                    -10, -10, -0.001,
                    10, -10, -0.001,
                    10, 10, -0.001,
                    -10, 10, -0.001,
                ],
                convexRadius: 5,
                maxErrorConvexRadius: 0.05,
            });

            // Should reduce radius significantly or to zero
            expect(flatHull.convexRadius).toBeLessThan(0.01);
        });
    });

    describe('Integration with Shape Properties', () => {
        test('reduced radius is stored in final shape', () => {
            const hull = convexHull.create({
                positions: [
                    -10, -10, 0.01,
                    10, -10, 0.01,
                    10, 10, 0.01,
                    -10, 10, 0.01,
                    -10, -10, -0.01,
                    10, -10, -0.01,
                    10, 10, -0.01,
                    -10, 10, -0.01,
                ],
                convexRadius: 5,
            });

            // Shape should have reduced radius
            expect(hull.convexRadius).toBeLessThan(5);
            expect(hull.convexRadius).toBeGreaterThanOrEqual(0);
        });

        test('all other shape properties remain valid after reduction', () => {
            const hull = convexHull.create({
                positions: [
                    0, 10, 0,
                    -5, 0, -5,
                    -5, 0, 5,
                    5, 0, -5,
                    5, 0, 5,
                ],
                convexRadius: 2,
                density: 1000,
            });

            // Verify shape integrity
            expect(hull.points.length).toBeGreaterThan(0);
            expect(hull.faces.length).toBeGreaterThan(0);
            expect(hull.planes.length).toBe(hull.faces.length);
            expect(hull.volume).toBeGreaterThan(0);
            expect(hull.density).toBe(1000);
            expect(hull.inertia).toBeDefined();
            expect(hull.centerOfMass).toBeDefined();
        });
    });
});
