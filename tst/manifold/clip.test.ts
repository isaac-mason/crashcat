import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { createFace, type Face } from '../../src/utils/face';
import { clipPolyVsEdge, clipPolyVsPlane, clipPolyVsPoly } from '../../src/manifold/clip';

function createFaceWithVertices(vertices: number[]): Face {
    const face = createFace();
    face.vertices = [...vertices];
    face.numVertices = vertices.length / 3;
    return face;
}

function expectVerticesClose(actual: Face, expected: number[], tolerance = 1e-6): void {
    expect(actual.numVertices).toBe(expected.length / 3);
    for (let i = 0; i < expected.length; i++) {
        expect(actual.vertices[i]).toBeCloseTo(expected[i], tolerance);
    }
}

describe('clipPolyVsPlane', () => {
    test('clips triangle through center creating trapezoid', () => {
        // Triangle: [(0,0,0), (2,0,0), (1,2,0)]
        const triangle = createFaceWithVertices([0, 0, 0, 2, 0, 0, 1, 2, 0]);

        // Plane at y=1, normal pointing DOWN (keep y < 1, which is positive halfspace when normal points down)
        const planeOrigin = vec3.fromValues(0, 1, 0);
        const planeNormal = vec3.fromValues(0, -1, 0);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPlane(out, triangle, planeOrigin, planeNormal);

        // Should create a trapezoid with 4 vertices:
        // Original bottom edge: (0,0,0), (2,0,0)
        // Clipped edges: (0.5,1,0), (1.5,1,0)
        expect(out.numVertices).toBe(4);

        // Vertices order: (0.5,1,0), (0,0,0), (2,0,0), (1.5,1,0)
        expectVerticesClose(out, [0.5, 1, 0, 0, 0, 0, 2, 0, 0, 1.5, 1, 0]);
    });

    test('keeps triangle fully inside plane', () => {
        // Triangle: [(0,0,0), (1,0,0), (0.5,1,0)]
        const triangle = createFaceWithVertices([0, 0, 0, 1, 0, 0, 0.5, 1, 0]);

        // Plane at y=2, normal pointing DOWN (keep y < 2)
        const planeOrigin = vec3.fromValues(0, 2, 0);
        const planeNormal = vec3.fromValues(0, -1, 0);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPlane(out, triangle, planeOrigin, planeNormal);

        // Triangle completely inside, should be unchanged
        expect(out.numVertices).toBe(3);
        expectVerticesClose(out, [0, 0, 0, 1, 0, 0, 0.5, 1, 0]);
    });

    test('removes triangle fully outside plane', () => {
        // Triangle: [(0,0,0), (1,0,0), (0.5,1,0)]
        const triangle = createFaceWithVertices([0, 0, 0, 1, 0, 0, 0.5, 1, 0]);

        // Plane at y=-1, normal pointing DOWN (keep y < -1)
        const planeOrigin = vec3.fromValues(0, -1, 0);
        const planeNormal = vec3.fromValues(0, -1, 0);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPlane(out, triangle, planeOrigin, planeNormal);

        // Triangle completely outside
        expect(out.numVertices).toBe(0);
    });

    test('handles edge parallel to plane', () => {
        // Square with bottom edge slightly below the plane
        const square = createFaceWithVertices([0, -0.1, 0, 1, -0.1, 0, 1, 1, 0, 0, 1, 0]);

        // Plane through y=0, normal pointing DOWN (keep y < 0)
        const planeOrigin = vec3.fromValues(0, 0, 0);
        const planeNormal = vec3.fromValues(0, -1, 0);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPlane(out, square, planeOrigin, planeNormal);

        // Bottom edge is below the plane, top vertices clipped
        expect(out.numVertices).toBe(4); // Gets intersection points at y=0 plus bottom edge
    });

    test('handles unnormalized plane normal', () => {
        // Triangle
        const triangle = createFaceWithVertices([0, 0, 0, 2, 0, 0, 1, 2, 0]);

        // Plane at y=1 with unnormalized normal pointing DOWN
        const planeOrigin = vec3.fromValues(0, 1, 0);
        const planeNormal = vec3.fromValues(0, -5, 0); // Unnormalized

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPlane(out, triangle, planeOrigin, planeNormal);

        // Should work the same as normalized normal
        expect(out.numVertices).toBe(4);
        expectVerticesClose(out, [0.5, 1, 0, 0, 0, 0, 2, 0, 0, 1.5, 1, 0]);
    });
});

describe('clipPolyVsPoly', () => {
    test('clips square against rotated square', () => {
        // Square 1: axis-aligned, 1x1 centered at origin
        const square1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // Square 2: rotated 45 degrees, slightly larger
        const s = Math.sqrt(2) / 2;
        const square2 = createFaceWithVertices([-s, 0, 0, 0, -s, 0, s, 0, 0, 0, s, 0]);

        // Normal for square 2 (pointing out of XY plane)
        const normal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPoly(out, square1, square2, normal);

        // Result should be an octagon (8 vertices)
        expect(out.numVertices).toBe(8);
    });

    test('keeps square when clipped by larger square', () => {
        // Small square
        const smallSquare = createFaceWithVertices([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0]);

        // Large square containing the small one
        const largeSquare = createFaceWithVertices([-1, -1, 0, 2, -1, 0, 2, 2, 0, -1, 2, 0]);

        const normal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPoly(out, smallSquare, largeSquare, normal);

        // Small square should be unchanged (4 vertices)
        expect(out.numVertices).toBe(4);
    });

    test('returns empty when polygons do not overlap', () => {
        // Square 1
        const square1 = createFaceWithVertices([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0]);

        // Square 2, completely separated
        const square2 = createFaceWithVertices([5, 5, 0, 6, 5, 0, 6, 6, 0, 5, 6, 0]);

        const normal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPoly(out, square1, square2, normal);

        // No overlap
        expect(out.numVertices).toBe(0);
    });

    test('handles partial overlap creating smaller polygon', () => {
        // Square 1: [0,0] to [2,2]
        const square1 = createFaceWithVertices([0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0]);

        // Square 2: [1,1] to [3,3] (overlaps bottom-right quadrant)
        const square2 = createFaceWithVertices([1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0]);

        const normal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPoly(out, square1, square2, normal);

        // Should create a smaller polygon where they overlap
        expect(out.numVertices).toBeGreaterThanOrEqual(3);
        expect(out.numVertices).toBeLessThanOrEqual(8);
    });

    test('handles degenerate case with too few vertices in clipping polygon', () => {
        const square = createFaceWithVertices([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0]);

        // Degenerate polygon with only 2 vertices
        const line = createFaceWithVertices([0.5, 0, 0, 0.5, 1, 0]);

        const normal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsPoly(out, square, line, normal);

        // Should return empty (clipping polygon must have >= 3 vertices)
        expect(out.numVertices).toBe(0);
    });
});

describe('clipPolyVsEdge', () => {
    test('clips square with edge through center', () => {
        // Square in XY plane: [0,0] to [2,2]
        const square = createFaceWithVertices([0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0]);

        // Edge through center, will be projected onto square plane
        const edgeV1 = vec3.fromValues(1, 0, 5); // Z=5, will project to z=0
        const edgeV2 = vec3.fromValues(1, 2, 5);

        // Clipping normal perpendicular to polygon (pointing out of plane)
        // Edge normal will be: clippingNormal × edge
        const clippingNormal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsEdge(out, square, edgeV1, edgeV2, clippingNormal);

        // Should produce 2 intersection points where square edges cross the edge projection
        expect(out.numVertices).toBe(2);
    });

    test('clips square with edge at corner', () => {
        // Square in XY plane: [0,0] to [2,2]
        const square = createFaceWithVertices([0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0]);

        // Edge slightly offset from corner, above the plane
        const edgeV1 = vec3.fromValues(0.5, 0, 5);
        const edgeV2 = vec3.fromValues(0.5, 2, 5);

        // Clipping normal perpendicular to polygon
        const clippingNormal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsEdge(out, square, edgeV1, edgeV2, clippingNormal);

        // Should produce 2 intersection points
        expect(out.numVertices).toBe(2);
    });

    test('projects edge endpoints when intersection falls outside edge', () => {
        // Large square in XY plane
        const square = createFaceWithVertices([0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0]);

        // Short edge in the middle, above the plane
        const edgeV1 = vec3.fromValues(2, 1.5, 5);
        const edgeV2 = vec3.fromValues(2, 2.5, 5);

        // Clipping normal perpendicular to polygon
        const clippingNormal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsEdge(out, square, edgeV1, edgeV2, clippingNormal);

        // Should clamp intersection points to edge endpoints
        expect(out.numVertices).toBeGreaterThanOrEqual(1);

        // Verify that output points are clamped to the edge segment
        for (let i = 0; i < out.numVertices; i++) {
            const y = out.vertices[i * 3 + 1];
            // Y should be within [1.5, 2.5] (edge endpoints)
            expect(y).toBeGreaterThanOrEqual(1.5 - 1e-6);
            expect(y).toBeLessThanOrEqual(2.5 + 1e-6);
        }
    });

    test('handles degenerate case with too few polygon vertices', () => {
        // Line (2 vertices, need at least 3)
        const line = createFaceWithVertices([0, 0, 0, 1, 0, 0]);

        const edgeV1 = vec3.fromValues(0.5, -1, 0);
        const edgeV2 = vec3.fromValues(0.5, 1, 0);
        const clippingNormal = vec3.fromValues(-1, 0, 0);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsEdge(out, line, edgeV1, edgeV2, clippingNormal);

        // Should return empty (polygon must have >= 3 vertices)
        expect(out.numVertices).toBe(0);
    });

    test('edge projected onto polygon plane produces correct intersections', () => {
        // Triangle in XY plane
        const triangle = createFaceWithVertices([0, 0, 0, 2, 0, 0, 1, 2, 0]);

        // Edge NOT in XY plane (will be projected)
        const edgeV1 = vec3.fromValues(1, 0, 5);
        const edgeV2 = vec3.fromValues(1, 2, 5);

        // Clipping normal perpendicular to polygon
        const clippingNormal = vec3.fromValues(0, 0, 1);

        const out = createFace();
        out.vertices = new Array(64 * 3);

        clipPolyVsEdge(out, triangle, edgeV1, edgeV2, clippingNormal);

        // Should project edge to z=0 plane and clip
        expect(out.numVertices).toBeGreaterThanOrEqual(1);

        // All output vertices should be in z=0 plane (polygon's plane)
        for (let i = 0; i < out.numVertices; i++) {
            const z = out.vertices[i * 3 + 2];
            expect(z).toBeCloseTo(0, 6);
        }
    });
});
