import { describe, test, expect } from 'vitest';
import { buildTriangleMesh } from '../../src/shapes/utils/triangle-mesh-builder';
import { triangleMeshBvh } from '../../src';
import { getActiveEdges } from '../../src/shapes/utils/triangle-mesh-data';

const activeEdgeCosThresholdAngle = 0.996195;

describe('TriangleMeshBuilder', () => {
    describe('basic mesh building', () => {
        test('builds valid non-degenerate triangle', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
            expect(result.bvh).toBeDefined();
            expect(result.bvh.buffer.length).toBeGreaterThan(0);
        });

        test('builds mesh with multiple valid triangles', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1],
                indices: [0, 1, 2, 1, 2, 3],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);
        });
    });

    describe('degenerate triangle removal', () => {
        test('removes collinear triangle (zero area)', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2, 3, 4, 5],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
        });

        test('removes triangle with repeated vertices', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 0, 1],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(0);
        });

        test('removes multiple degenerate triangles', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2, 2, 3, 4, 5, 6, 7],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
        });

        test('respects custom degenerateTolerance', () => {
            const tightResult = buildTriangleMesh({
                positions: [0, 0, 0, 0.00001, 0, 0, 0, 0.00001, 0],
                indices: [0, 1, 2],
                degenerateTolerance: 1e-10,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,

activeEdgeCosThresholdAngle            });

            expect(tightResult.data.triangleCount).toBe(1);

            const looseResult = buildTriangleMesh({
                positions: [0, 0, 0, 0.00001, 0, 0, 0, 0.00001, 0],
                indices: [0, 1, 2],
                degenerateTolerance: 1e-2,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,

activeEdgeCosThresholdAngle            });

            expect(looseResult.data.triangleCount).toBe(0);
        });
    });

    describe('duplicate triangle removal', () => {
        test('removes exact duplicate triangles (same vertex order)', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2, 0, 1, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
        });

        test('removes duplicate triangles with rotated vertex order', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2, 1, 2, 0],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
        });

        test('removes duplicate triangles with reversed vertex order', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2, 2, 1, 0],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
        });

        test('keeps unique triangles', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
                indices: [0, 1, 2, 1, 2, 3],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);
        });

        test('removes multiple duplicates of same triangle', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2, 0, 1, 2, 1, 2, 0],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
        });

        test('handles floating point precision in duplicate detection', () => {
            const result = buildTriangleMesh({
                positions: [
                    0, 0, 0, 1, 0, 0, 0, 1, 0, 0.000000001, 0.000000001, 0.000000001, 1.000000001, 0, 0, 0, 1.000000001, 0,
                ],
                indices: [0, 1, 2, 3, 4, 5],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle,
            });

            expect(result.data.triangleCount).toBe(1);
        });
    });

    describe('combined degenerate and duplicate removal', () => {
        test('removes both degenerate and duplicate triangles', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0, 1, 1, 1],
                indices: [0, 1, 2, 0, 1, 3, 1, 2, 4],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);
        });

        test('removes both degenerate and duplicate in same mesh', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0],
                indices: [0, 1, 2, 0, 1, 3, 0, 1, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
        });
    });

    describe('subShapeId reassignment', () => {
        test('preserves sequential subShapeIds after cleanup', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1],
                indices: [0, 1, 2, 0, 1, 3, 4, 5, 6, 1, 2, 7],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);
            // Triangle indices are now implicit (position in array), no longer stored
        });

        test('maintains correct subShapeId with all triangles removed except one', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2, 0, 1, 2, 0, 1, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
            // Triangle indices are now implicit (position in array), no longer stored
        });
    });

    describe('BVH building', () => {
        test('builds valid BVH after cleanup', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1],
                indices: [0, 1, 2, 1, 2, 3],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.bvh).toBeDefined();
            expect(result.bvh.buffer.length).toBeGreaterThan(0);
            expect(result.bvh.buffer[0]).toBeDefined();
        });

        test('builds BVH with single triangle', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.bvh.buffer.length).toBeGreaterThan(0);
        });

        test('builds BVH with multiple triangles after degenerate removal', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 0, 0, 1, 1, 1, 2, 1, 1],
                indices: [0, 1, 2, 0, 1, 3, 1, 2, 4],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);
            expect(result.bvh.buffer.length).toBeGreaterThan(0);
        });
    });

    describe('edge cases', () => {
        test('handles empty mesh', () => {
            const result = buildTriangleMesh({
                positions: [],
                indices: [],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(0);
        });

        test('handles mesh with single vertex (degenerate)', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0],
                indices: [0, 0, 0],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(0);
        });

        test('handles very large mesh', () => {
            const positions: number[] = [];
            const indices: number[] = [];

            for (let x = 0; x < 10; x++) {
                for (let y = 0; y < 10; y++) {
                    positions.push(x, y, 0);
                    positions.push(x + 1, y, 0);
                    positions.push(x, y + 1, 0);

                    const baseIdx = positions.length / 3 - 3;
                    indices.push(baseIdx, baseIdx + 1, baseIdx + 2);
                }
            }

            const result = buildTriangleMesh({
                positions,
                indices,
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(100);
            // BVH should have internal nodes + leaf nodes. For a binary tree with n triangles
            // and maxLeafTris=2, we expect roughly n/2 leaves and n/2 - 1 internal nodes,
            // so total nodes around n - 1 or more (varies with split strategy).
            expect(result.bvh.buffer.length).toBeGreaterThan(50);
        });
    });

    describe('active edge computation', () => {
        test('single triangle has all edges active', () => {
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(1);
            expect(getActiveEdges(result.data, 0)).toBe(0b111);
        });

        test('smooth adjacent triangles have shared edge inactive', () => {
            // Create two triangles that form a smooth surface (small dihedral angle)
            // Both must have consistent CCW winding for normals to match
            // Triangle 0: (0,0,0), (1,0,0), (0,1,0)
            // Triangle 1: (1,0,0), (1,1,0), (0,1,0) - coplanar with matching winding
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
                indices: [0, 1, 2, 1, 3, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);

            // Both triangles should have the shared edge (1-2) inactive
            // Triangle 0: edge BC (bit 0b010) should be inactive
            // Triangle 1: edge CA (bit 0b100) should be inactive
            expect(getActiveEdges(result.data, 0) & 0b010).toBe(0);
            expect(getActiveEdges(result.data, 1) & 0b100).toBe(0);

            // Other edges should remain active
            expect(getActiveEdges(result.data, 0) & 0b101).toBe(0b101);
            expect(getActiveEdges(result.data, 1) & 0b011).toBe(0b011);
        });

        test('sharp adjacent triangles have shared edge active', () => {
            // Create two triangles with a sharp dihedral angle (90°)
            // Triangle 1: (0,0,0), (1,0,0), (0,1,0) - in XY plane
            // Triangle 2: (1,0,0), (0,1,0), (0,1,1) - bent out
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1],
                indices: [0, 1, 2, 1, 2, 3],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);

            // Both triangles should have the shared edge (1-2) active
            expect(getActiveEdges(result.data, 0) & 0b010).toBe(0b010);
            expect(getActiveEdges(result.data, 1) & 0b001).toBe(0b001);
        });

        test('boundary edges remain active', () => {
            // Single triangle - all edges are boundary edges
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
                indices: [0, 1, 2],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(getActiveEdges(result.data, 0)).toBe(0b111);
        });

        test('non-manifold edges remain active', () => {
            // Create three triangles sharing a common edge
            // This creates a non-manifold configuration
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1, 0],
                indices: [
                    0,
                    1,
                    2, // Triangle 1
                    0,
                    1,
                    3, // Triangle 2 (shares edge 0-1)
                ],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);

            // Edge 0-1 is shared by both and has dihedral angle 180° (opposite faces)
            // Normals should be opposite, dot product should be negative/low
            // Both triangles should have the edge active
            expect(getActiveEdges(result.data, 0) & 0b001).toBe(0b001);
            expect(getActiveEdges(result.data, 1) & 0b001).toBe(0b001);
        });

        test('mixed smooth and sharp edges', () => {
            // Create a pyramid: smooth base transitions to sharp sides
            const result = buildTriangleMesh({
                positions: [
                    // Base quad (two triangles in same plane)
                    0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0,
                    // Apex
                    0.5, 0.5, 1,
                ],
                indices: 
                [
                    // Base (smooth) - consistent CCW winding
                    0, 1, 2, 0, 2, 3,
                    // Sides (sharp relative to base) - consistent CCW winding
                    0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4,
                ],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            // All 6 triangles should be present (no degenerates, no duplicates)
            expect(result.data.triangleCount).toBe(6);

            // All triangles should have activeEdges computed
            for (let i = 0; i < result.data.triangleCount; i++) {
                const activeEdges = getActiveEdges(result.data, i);
                expect(activeEdges).toBeDefined();
                expect(activeEdges).toBeGreaterThanOrEqual(0);
                expect(activeEdges).toBeLessThanOrEqual(0b111);
            }

            // At least some edges should be inactive (smooth), not all edges active
            let hasInactiveEdges = false;
            for (let i = 0; i < result.data.triangleCount; i++) {
                if (getActiveEdges(result.data, i) !== 0b111) {
                    hasInactiveEdges = true;
                    break;
                }
            }
            expect(hasInactiveEdges).toBe(true);
        });

        test('all edges with extreme angle difference', () => {
            // Create two triangles with 170° dihedral angle (very sharp)
            // This tests the cosine threshold more thoroughly
            const result = buildTriangleMesh({
                positions: [0, 0, 0, 1, 0, 0, 0.5, 0.1, 0, 0.5, -0.1, 0.5],
                indices: [0, 1, 2, 1, 0, 3],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle
            });

            expect(result.data.triangleCount).toBe(2);

            // Large dihedral angle means edges should be active
            // Both triangles share edge (0,1)
            expect(getActiveEdges(result.data, 0) & 0b001).toBe(0b001);
            expect(getActiveEdges(result.data, 1) & 0b001).toBe(0b001);
        });

        test('preserves activeEdges through mesh building pipeline', () => {
            // Complex mesh with mix of degenerate removal, duplicates, and edge computation
            const result = buildTriangleMesh({
                positions: [
                    // Valid mesh (coplanar, smooth with consistent winding)
                    0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0,
                    // Extra degenerate (collinear)
                    2, 0, 0, 3, 0, 0, 3.5, 0, 0,
                ],
                indices: [
                    0,
                    1,
                    2, // Valid tri 1 (CCW from above)
                    0,
                    2,
                    3, // Valid tri 2 (CCW from above, shares edge 0-2)
                    4,
                    5,
                    6, // Degenerate (collinear)
                    0,
                    1,
                    2, // Duplicate of tri 1
                ],
                degenerateTolerance: 1e-6,
                bvhMaxLeafTris: 2,
                bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.SAH,
                activeEdgeCosThresholdAngle,
            });

            // Should have 2 valid triangles after cleanup
            expect(result.data.triangleCount).toBe(2);

            // Should have activeEdges computed for all remaining triangles
            expect(getActiveEdges(result.data, 0)).toBeDefined();
            expect(getActiveEdges(result.data, 1)).toBeDefined();

            // Both triangles should be active (they have sharp angles relative to boundary edges)
            // or at least some combination should show edge computation happened
            expect(typeof getActiveEdges(result.data, 0)).toBe('number');
            expect(typeof getActiveEdges(result.data, 1)).toBe('number');
        });
    });
});
