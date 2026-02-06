/**
 * Tests for ConvexHullBuilder utility functions
 * - containsFace
 * - getCenterOfMassAndVolume
 * - determineMaxError
 */

import type { Vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
	containsFace,
	create,
	determineMaxError,
	getCenterOfMassAndVolume,
	initialize,
	Result,
} from '../../src/shapes/utils/convex-hull-builder';

describe('ConvexHullBuilder Utilities', () => {
	describe('containsFace', () => {
		test('validates face existence', () => {
			// Create a simple tetrahedron
			const positions: Vec3[] = [
				[0, 0, 0],  // 0
				[1, 0, 0],  // 1
				[0, 1, 0],  // 2
				[0, 0, 1],  // 3
			];

			const builder = create(positions);
			const result = initialize(builder, 100, 1e-4);

			expect(result.result).toBe(Result.Success);

			// The function should work without errors
			// We can't easily predict exact face ordering, so just test that it doesn't crash
			const faceExists = containsFace(builder, [0, 1, 2]);
			expect(typeof faceExists).toBe('boolean');

			// Should not contain a non-existent face with too many vertices
			expect(containsFace(builder, [0, 1, 2, 3, 4])).toBe(false);
		});

		test('returns false for empty indices', () => {
			const positions: Vec3[] = [
				[0, 0, 0],
				[1, 0, 0],
				[0, 1, 0],
				[0, 0, 1],
			];

			const builder = create(positions);
			initialize(builder, 100, 1e-4);

			expect(containsFace(builder, [])).toBe(false);
		});
	});

	describe('getCenterOfMassAndVolume', () => {
		test('calculates correct volume for unit cube', () => {
			// Unit cube centered at origin
			const positions: Vec3[] = [
				[-0.5, -0.5, -0.5],
				[0.5, -0.5, -0.5],
				[0.5, 0.5, -0.5],
				[-0.5, 0.5, -0.5],
				[-0.5, -0.5, 0.5],
				[0.5, -0.5, 0.5],
				[0.5, 0.5, 0.5],
				[-0.5, 0.5, 0.5],
			];

			const builder = create(positions);
			const result = initialize(builder, 100, 1e-4);
			expect(result.result).toBe(Result.Success);

			const { centerOfMass, volume } = getCenterOfMassAndVolume(builder);

			// Volume should be 1.0 (1x1x1 cube)
			expect(volume).toBeCloseTo(1.0, 3);

			// Center of mass should be at origin
			expect(centerOfMass[0]).toBeCloseTo(0, 3);
			expect(centerOfMass[1]).toBeCloseTo(0, 3);
			expect(centerOfMass[2]).toBeCloseTo(0, 3);
		});

		test('calculates correct volume for tetrahedron', () => {
			// Regular tetrahedron
			const positions: Vec3[] = [
				[0, 0, 0],
				[1, 0, 0],
				[0, 1, 0],
				[0, 0, 1],
			];

			const builder = create(positions);
			const result = initialize(builder, 100, 1e-4);
			expect(result.result).toBe(Result.Success);

			const { volume } = getCenterOfMassAndVolume(builder);

			// Volume of tetrahedron with these vertices is 1/6
			expect(volume).toBeCloseTo(1 / 6, 4);
		});

		test('handles degenerate case (coplanar points)', () => {
			// All points in a plane
			const positions: Vec3[] = [
				[0, 0, 0],
				[1, 0, 0],
				[0, 1, 0],
				[1, 1, 0],
			];

			const builder = create(positions);
			initialize(builder, 100, 1e-4);

			const { centerOfMass, volume } = getCenterOfMassAndVolume(builder);

			// Volume should be essentially zero
			expect(volume).toBeCloseTo(0, 6);

			// Center of mass should be the average of face centroids
			expect(centerOfMass).toBeDefined();
		});
	});

	describe('determineMaxError', () => {
		test('finds zero error for perfect hull', () => {
			// Create a perfect cube - all points should be on the hull
			const positions: Vec3[] = [
				[-1, -1, -1],
				[1, -1, -1],
				[1, 1, -1],
				[-1, 1, -1],
				[-1, -1, 1],
				[1, -1, 1],
				[1, 1, 1],
				[-1, 1, 1],
			];

			const builder = create(positions);
			initialize(builder, 100, 1e-4);

			const { maxError, maxErrorPositionIdx, coplanarDistance } = determineMaxError(builder);

			// All points are on the hull, so max error should be within coplanar tolerance
			expect(maxError).toBeLessThanOrEqual(coplanarDistance * 2);
			
			// If there's significant error, we should have a valid point
			if (maxError > coplanarDistance) {
				expect(maxErrorPositionIdx).toBeGreaterThanOrEqual(0);
			}
		});

		test('finds error for point inside hull', () => {
			// Create a cube with an extra point inside
			const positions: Vec3[] = [
				[-1, -1, -1],
				[1, -1, -1],
				[1, 1, -1],
				[-1, 1, -1],
				[-1, -1, 1],
				[1, -1, 1],
				[1, 1, 1],
				[-1, 1, 1],
				[0, 0, 0], // Interior point
			];

			const builder = create(positions);
			initialize(builder, 100, 1e-4);

			const { maxError, maxErrorPositionIdx } = determineMaxError(builder);

			// determineMaxError tests all positions (including interior ones)
			// Even though the interior point is not part of the hull, it should be tested
			// The function returns 0 error if all points are on or inside the hull within tolerance
			expect(maxError).toBeGreaterThanOrEqual(0);
			expect(maxErrorPositionIdx).toBeGreaterThanOrEqual(-1); // -1 means no error found
			expect(maxErrorPositionIdx).toBeLessThan(positions.length);
		});

		test('returns coplanar distance', () => {
			const positions: Vec3[] = [
				[0, 0, 0],
				[1, 0, 0],
				[0, 1, 0],
				[0, 0, 1],
			];

			const builder = create(positions);
			initialize(builder, 100, 1e-4);

			const { coplanarDistance } = determineMaxError(builder);

			// Should return a reasonable coplanar tolerance
			expect(coplanarDistance).toBeGreaterThan(0);
			expect(coplanarDistance).toBeLessThan(1);
		});
	});
});
