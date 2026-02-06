import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { activeEdges } from '../../src';

describe('Active Edges - isEdgeActive', () => {
    test('coplanar triangles should have inactive edge', () => {
        // Two triangles with same normal (coplanar)
        const normal1 = vec3.fromValues(0, 1, 0);
        const normal2 = vec3.fromValues(0, 1, 0);
        const edgeDirection = vec3.fromValues(1, 0, 0);
        const cosThreshold = Math.cos(10 * Math.PI / 180); // 10 degrees

        const result = activeEdges.isEdgeActive(normal1, normal2, edgeDirection, cosThreshold);

        expect(result).toBe(false);
    });

    test('sharp convex edge (90 degrees) should be active', () => {
        // Two triangles at 90 degree angle (convex edge)
        const normal1 = vec3.fromValues(0, 1, 0);  // Up
        const normal2 = vec3.fromValues(1, 0, 0);  // Right
        // For convex edge, cross product n1 x n2 = (0,0,-1), so edge direction should align
        const edgeDirection = vec3.fromValues(0, 0, -1); // Edge points along -Z
        const cosThreshold = Math.cos(10 * Math.PI / 180); // 10 degrees

        const result = activeEdges.isEdgeActive(normal1, normal2, edgeDirection, cosThreshold);

        expect(result).toBe(true);
    });

    test('sharp convex edge (45 degrees) should be active', () => {
        // Two triangles at 45 degree angle
        const normal1 = vec3.fromValues(0, 1, 0);  // Up
        const normal2 = vec3.normalize(vec3.create(), vec3.fromValues(1, 1, 0)); // 45 degrees
        // Cross product will point in -Z direction for this convex configuration
        const edgeDirection = vec3.fromValues(0, 0, -1);
        const cosThreshold = Math.cos(10 * Math.PI / 180); // 10 degrees

        const result = activeEdges.isEdgeActive(normal1, normal2, edgeDirection, cosThreshold);

        expect(result).toBe(true);
    });

    test('small angle convex edge (5 degrees) should be inactive with 10 degree threshold', () => {
        // Two triangles at 5 degree angle - below threshold
        const angle = 5 * Math.PI / 180;
        const normal1 = vec3.fromValues(0, 1, 0);
        const normal2 = vec3.normalize(vec3.create(), vec3.fromValues(Math.sin(angle), Math.cos(angle), 0));
        const edgeDirection = vec3.fromValues(0, 0, 1);
        const cosThreshold = Math.cos(10 * Math.PI / 180); // 10 degrees

        const result = activeEdges.isEdgeActive(normal1, normal2, edgeDirection, cosThreshold);

        expect(result).toBe(false);
    });

    test('concave edge should always be inactive', () => {
        // Two triangles forming a valley (concave edge)
        // Normal1 points up, Normal2 points left - forms a valley
        const normal1 = vec3.fromValues(0, 1, 0);   // Up
        const normal2 = vec3.fromValues(-1, 0, 0);  // Left (forms valley when edge points forward)
        const edgeDirection = vec3.fromValues(0, 0, 1);
        const cosThreshold = 0.0; // Even with very permissive threshold

        const result = activeEdges.isEdgeActive(normal1, normal2, edgeDirection, cosThreshold);

        expect(result).toBe(false);
    });

    test('back-to-back triangles should always be active', () => {
        // Triangles with opposite normals (back to back)
        const normal1 = vec3.fromValues(0, 1, 0);
        const normal2 = vec3.fromValues(0, -1, 0);  // Opposite direction
        const edgeDirection = vec3.fromValues(1, 0, 0);
        const cosThreshold = Math.cos(10 * Math.PI / 180);

        const result = activeEdges.isEdgeActive(normal1, normal2, edgeDirection, cosThreshold);

        expect(result).toBe(true);
    });

    test('nearly back-to-back triangles (179 degrees) should be active', () => {
        // Triangles at 179 degrees apart (nearly opposite)
        const angle = 179 * Math.PI / 180;
        const normal1 = vec3.fromValues(0, 1, 0);
        const normal2 = vec3.fromValues(0, Math.cos(angle), 0); // Almost pointing down
        const edgeDirection = vec3.fromValues(0, 0, 1);
        const cosThreshold = Math.cos(10 * Math.PI / 180);

        const result = activeEdges.isEdgeActive(normal1, normal2, edgeDirection, cosThreshold);

        expect(result).toBe(true);
    });
});

describe('Active Edges - fixNormal', () => {
    test('all edges active should return original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b111; // All edges active
        const point = vec3.fromValues(0.5, 0, 0.5);
        const normal = vec3.fromValues(0.1, 0.9, 0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        expect(result).toBe(normal); // Should return original normal reference
    });

    test('no active edges should return triangle normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b000; // No edges active
        const point = vec3.fromValues(0.5, 0, 0.5);
        const normal = vec3.fromValues(0.1, 0.9, 0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        expect(result).toBe(triangleNormal); // Should return triangle normal reference
    });

    test('interior hit should return triangle normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b101; // Mixed edges
        const point = vec3.fromValues(0.4, 0, 0.4); // Interior point (barycentric coords all > epsilon)
        const normal = vec3.fromValues(0.1, 0.9, 0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        expect(result).toBe(triangleNormal);
    });

    test('edge v0-v1 hit with active edge should return original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b001; // Only edge 0 (v0-v1) active
        const point = vec3.fromValues(0.5, 0, 0); // On edge v0-v1
        const normal = vec3.fromValues(0, 0.9, -0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        expect(result).toBe(normal); // Active edge hit, use calculated normal
    });

    test('edge v0-v1 hit with inactive edge should return triangle normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b110; // Edges 1 and 2 active, edge 0 (v0-v1) inactive
        const point = vec3.fromValues(0.5, 0, 0); // On edge v0-v1 (inactive)
        const normal = vec3.fromValues(0, 0.9, -0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        expect(result).toBe(triangleNormal); // Inactive edge hit, use triangle normal
    });

    test('edge v1-v2 hit with active edge should return original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b010; // Only edge 1 (v1-v2) active
        const point = vec3.fromValues(0.5, 0, 0.5); // On edge v1-v2
        const normal = vec3.fromValues(0.1, 0.9, 0);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        expect(result).toBe(normal);
    });

    test('edge v2-v0 hit with active edge should return original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b100; // Only edge 2 (v2-v0) active
        const point = vec3.fromValues(0, 0, 0.5); // On edge v2-v0
        const normal = vec3.fromValues(-0.1, 0.9, 0);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        expect(result).toBe(normal);
    });

    test('vertex v0 with one adjacent edge active should use original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b001; // Only edge 0 (v0-v1) active, adjacent to v0
        const point = vec3.fromValues(0.00001, 0, 0.00001); // Very close to v0
        const normal = vec3.fromValues(0, 0.9, 0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        // Vertex v0 requires edge 0 OR edge 2 (collidingEdge = 0b101)
        // Edge 0 is active, so should return original normal
        expect(result).toBe(normal);
    });

    test('vertex v0 with no adjacent edges active should use triangle normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b010; // Only edge 1 (v1-v2) active, not adjacent to v0
        const point = vec3.fromValues(0.00001, 0, 0.00001); // Very close to v0
        const normal = vec3.fromValues(0, 0.9, 0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        // Vertex v0 requires edge 0 OR edge 2 (collidingEdge = 0b101)
        // Neither edge 0 nor edge 2 is active, so should return triangle normal
        expect(result).toBe(triangleNormal);
    });

    test('vertex v1 with one adjacent edge active should use original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b010; // Only edge 1 (v1-v2) active, adjacent to v1
        const point = vec3.fromValues(0.99999, 0, 0.00001); // Very close to v1
        const normal = vec3.fromValues(0.1, 0.9, 0);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        // Vertex v1 requires edge 0 OR edge 1 (collidingEdge = 0b011)
        // Edge 1 is active, so should return original normal
        expect(result).toBe(normal);
    });

    test('vertex v2 with one adjacent edge active should use original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b100; // Only edge 2 (v2-v0) active, adjacent to v2
        const point = vec3.fromValues(0.00001, 0, 0.99999); // Very close to v2
        const normal = vec3.fromValues(-0.1, 0.9, 0.1);
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        // Vertex v2 requires edge 1 OR edge 2 (collidingEdge = 0b110)
        // Edge 2 is active, so should return original normal
        expect(result).toBe(normal);
    });

    test('movement direction heuristic - prefer calculated normal when it opposes movement less', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0); // Straight up
        const activeEdgesValue = 0b000; // All inactive (would normally return triangle normal)
        const point = vec3.fromValues(0.5, 0, 0); // On edge
        const normal = vec3.fromValues(0.707, 0.707, 0); // 45 degree angle
        const movement = vec3.fromValues(-1, 0, 0); // Moving in -X direction

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        // movement dot normal = -0.707 (opposes movement)
        // movement dot triangleNormal = 0 (perpendicular)
        // Check: movementDotNormal * triangleNormalLength < movementDotTriangleNormal * normalLength
        // -0.707 * 1 < 0 * ~1  =>  -0.707 < 0  =>  TRUE
        // Since the condition is true, should return normal (early exit from heuristic)
        expect(result).toBe(normal);
    });

    test('parallel normals should skip edge detection and return original normal', () => {
        const v0 = vec3.fromValues(0, 0, 0);
        const v1 = vec3.fromValues(1, 0, 0);
        const v2 = vec3.fromValues(0, 0, 1);
        const triangleNormal = vec3.fromValues(0, 1, 0);
        const activeEdgesValue = 0b001; // Only edge 0 active
        const point = vec3.fromValues(0.5, 0, 0.5); // Interior
        // Normal is nearly parallel to triangle normal
        const normal = vec3.normalize(vec3.create(), vec3.fromValues(0.001, 1, 0.001));
        const movement = vec3.fromValues(0, 0, 0);

        const result = activeEdges.fixNormal(v0, v1, v2, triangleNormal, activeEdgesValue, point, normal, movement);

        // Normals are nearly parallel (within 1 degree), skip barycentric check
        expect(result).toBe(normal);
    });
});
