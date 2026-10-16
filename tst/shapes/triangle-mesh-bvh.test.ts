import { describe, expect, test } from 'vitest';
import { triangleMeshBvh } from '../../src';
import {
    NODE_AXIS_OR_COUNT,
    NODE_MAX_X,
    NODE_MAX_Y,
    NODE_MAX_Z,
    NODE_MIN_X,
    NODE_MIN_Y,
    NODE_MIN_Z,
    NODE_STRIDE,
    nodeIsLeaf,
    nodeLeft,
    nodeRight,
} from '../../src/shapes/utils/bvh';
import { buildTriangleMesh, type TriangleMeshBuildResult } from '../../src/shapes/utils/triangle-mesh-builder';

const activeEdgeCosThresholdAngle = 0.996195;

const strategies = [
    triangleMeshBvh.BvhSplitStrategy.CENTER,
    triangleMeshBvh.BvhSplitStrategy.AVERAGE,
    triangleMeshBvh.BvhSplitStrategy.SAH,
];

function build(
    positions: number[],
    indices: number[],
    strategy: triangleMeshBvh.BvhSplitStrategy,
    maxLeafTris: number,
): TriangleMeshBuildResult {
    return buildTriangleMesh({
        positions,
        indices,
        degenerateTolerance: 1e-12,
        bvhMaxLeafTris: maxLeafTris,
        bvhSplitStrategy: strategy,
        activeEdgeCosThresholdAngle,
    });
}

/** unit right triangles (x, 0, 0), (x + 1, 0, 0), (x, 1, 0) in the xy plane, one per x */
function strip(xs: number[]): { positions: number[]; indices: number[] } {
    const positions: number[] = [];
    const indices: number[] = [];
    for (let i = 0; i < xs.length; i++) {
        const x = xs[i];
        positions.push(x, 0, 0, x + 1, 0, 0, x, 1, 0);
        indices.push(i * 3, i * 3 + 1, i * 3 + 2);
    }
    return { positions, indices };
}

/** whether outer contains inner, with no tolerance */
function containsExactly(outer: number[], outerOffset: number, inner: number[], innerOffset: number): boolean {
    return (
        outer[outerOffset + NODE_MIN_X] <= inner[innerOffset] &&
        outer[outerOffset + NODE_MIN_Y] <= inner[innerOffset + 1] &&
        outer[outerOffset + NODE_MIN_Z] <= inner[innerOffset + 2] &&
        outer[outerOffset + NODE_MAX_X] >= inner[innerOffset + 3] &&
        outer[outerOffset + NODE_MAX_Y] >= inner[innerOffset + 4] &&
        outer[outerOffset + NODE_MAX_Z] >= inner[innerOffset + 5]
    );
}

describe('triangle mesh bvh', () => {
    test('sah small mode picks the split its own candidates give, whatever earlier nodes left in the bins', () => {
        // an earlier build leaves candidates below every center of the mesh under test in the shared bins
        const seed = strip([-200, -199, -198, -197, -196, -195, -194, -193, -192, -191, -190, -189, -188, -187, -186, -185]);
        build(seed.positions, seed.indices, triangleMeshBvh.BvhSplitStrategy.SAH, 2);

        // five unit triangles at x = 0, 10, 20, 30, 40: aabb centers x = 0.5, 10.5, 20.5, 30.5, 40.5, y = 0.5,
        // z = 0. the center extent is zero on y and z, so only x is tried. surface areas of flat boxes are
        // 2 * dx * dy, all boxes have dy = 1; the leaf cost is 1.25 * n.
        //
        // root: bounds x [0, 41], area 82, leaf cost 6.25. candidates (left | right, cost = 1 + 1.25 * sum / 82):
        //   0.5:  {} | {0..4}, all right, cost 1 + 1.25 * 5 = 7.25
        //   10.5: {0} [0,1] area 2 | {1..4} [10,41] area 62, sum 2 * 1 + 62 * 4 = 250, cost 4.81
        //   20.5: {0,1} [0,11] area 22 | {2,3,4} [20,41] area 42, sum 44 + 126 = 170, cost 3.59
        //   30.5: {0,1,2} [0,21] area 42 | {3,4} [30,41] area 22, sum 126 + 44 = 170, cost 3.59
        //   40.5: {0..3} [0,31] area 62 | {4} area 2, sum 250, cost 4.81
        // 20.5 and 30.5 tie; the first strictly lower cost wins, so the root splits at 20.5:
        // {0,1} is a leaf (2 <= maxLeafTris), {2,3,4} splits again.
        //
        // {2,3,4}: bounds x [20, 41], area 42, leaf cost 3.75:
        //   20.5: all right, cost 4.75
        //   30.5: {2} area 2 | {3,4} area 22, sum 2 + 44 = 46, cost 1 + 1.25 * 46 / 42 = 2.37
        //   40.5: {2,3} area 22 | {4} area 2, sum 46, cost 2.37
        // it splits at 30.5 into leaves {2} and {3,4}.
        //
        // pre-order: root, leaf {0,1}, node, leaf {2}, leaf {3,4}
        const mesh = strip([0, 10, 20, 30, 40]);
        const { bvh } = build(mesh.positions, mesh.indices, triangleMeshBvh.BvhSplitStrategy.SAH, 2);
        const buffer = bvh.buffer;

        expect(buffer.length).toBe(5 * NODE_STRIDE);

        const leafCountOf = (node: number) => triangleMeshBvh.nodeTriCount(buffer, node * NODE_STRIDE);
        const xRangeOf = (node: number) => [buffer[node * NODE_STRIDE + NODE_MIN_X], buffer[node * NODE_STRIDE + NODE_MAX_X]];

        expect(nodeIsLeaf(buffer, 0)).toBe(false);
        expect(buffer[NODE_AXIS_OR_COUNT]).toBe(0);
        expect(nodeRight(buffer, 0)).toBe(2 * NODE_STRIDE);
        expect(xRangeOf(0)).toEqual([0, 41]);

        expect(nodeIsLeaf(buffer, 1 * NODE_STRIDE)).toBe(true);
        expect(leafCountOf(1)).toBe(2);
        expect(xRangeOf(1)).toEqual([0, 11]);

        expect(nodeIsLeaf(buffer, 2 * NODE_STRIDE)).toBe(false);
        expect(buffer[2 * NODE_STRIDE + NODE_AXIS_OR_COUNT]).toBe(0);
        expect(nodeRight(buffer, 2 * NODE_STRIDE)).toBe(4 * NODE_STRIDE);
        expect(xRangeOf(2)).toEqual([20, 41]);

        expect(nodeIsLeaf(buffer, 3 * NODE_STRIDE)).toBe(true);
        expect(leafCountOf(3)).toBe(1);
        expect(xRangeOf(3)).toEqual([20, 21]);

        expect(nodeIsLeaf(buffer, 4 * NODE_STRIDE)).toBe(true);
        expect(leafCountOf(4)).toBe(2);
        expect(xRangeOf(4)).toEqual([30, 41]);
    });

    test('every node contains its triangles and children exactly', () => {
        // a 9 x 9 vertex grid whose coordinates are sums and products of values with no exact binary form
        const side = 9;
        const positions: number[] = [];
        for (let j = 0; j < side; j++) {
            for (let i = 0; i < side; i++) {
                const x = i * 0.1 * 3 + j * 0.7e-3 * 7;
                const y = 0.1 * Math.sin(0.7 * i + 0.3) * Math.cos(1.1 * j) + 1e-3 * 13 * (i % 3);
                const z = j * 0.7 * 0.3 - i * 1e-3 * 11;
                positions.push(x, y, z);
            }
        }
        const indices: number[] = [];
        for (let j = 0; j < side - 1; j++) {
            for (let i = 0; i < side - 1; i++) {
                const v00 = j * side + i;
                const v10 = v00 + 1;
                const v01 = v00 + side;
                const v11 = v01 + 1;
                indices.push(v00, v01, v11, v00, v11, v10);
            }
        }

        for (const strategy of strategies) {
            for (const maxLeafTris of [1, 2, 3, 4, 8]) {
                const { data, bvh } = build(positions, indices, strategy, maxLeafTris);
                const buffer = bvh.buffer;
                const aabbs = data.triangleAABBs;
                expect(data.triangleCount).toBe((side - 1) * (side - 1) * 2);

                let leavesContain = true;
                let nodesContain = true;
                let covered = 0;
                const stack = [0];
                while (stack.length > 0) {
                    const offset = stack.pop()!;
                    if (nodeIsLeaf(buffer, offset)) {
                        const start = triangleMeshBvh.nodeTriStart(buffer, offset);
                        const count = triangleMeshBvh.nodeTriCount(buffer, offset);
                        covered += count;
                        for (let t = start; t < start + count; t++) {
                            leavesContain = leavesContain && containsExactly(buffer, offset, aabbs, t * 6);
                        }
                    } else {
                        const left = nodeLeft(offset);
                        const right = nodeRight(buffer, offset);
                        nodesContain =
                            nodesContain &&
                            containsExactly(buffer, offset, buffer, left) &&
                            containsExactly(buffer, offset, buffer, right);
                        stack.push(right, left);
                    }
                }

                expect(covered).toBe(data.triangleCount);
                expect(leavesContain).toBe(true);
                expect(nodesContain).toBe(true);
            }
        }
    });
});
