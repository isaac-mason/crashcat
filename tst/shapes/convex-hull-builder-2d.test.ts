import { type Vec3, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { createConvexHullBuilder2D, EResult, initialize } from '../../src/shapes/utils/convex-hull-builder-2d';

describe('ConvexHullBuilder2D', () => {
    test('should build a simple square hull', () => {
        // Create 4 points forming a square
        const points: Vec3[] = [
            vec3.fromValues(0, 0, 0),
            vec3.fromValues(1, 0, 0),
            vec3.fromValues(1, 1, 0),
            vec3.fromValues(0, 1, 0),
        ];

        const builder = createConvexHullBuilder2D(points);
        const outEdges: number[] = [];

        const result = initialize(
            builder,
            0, // inIdx1
            1, // inIdx2
            2, // inIdx3
            Number.MAX_SAFE_INTEGER, // inMaxVertices
            0.001, // inTolerance
            outEdges,
        );

        expect(result).toBe(EResult.Success);
        expect(outEdges.length).toBe(4); // Square has 4 edges
        expect(outEdges).toContain(0);
        expect(outEdges).toContain(1);
        expect(outEdges).toContain(2);
        expect(outEdges).toContain(3);
    });
});
