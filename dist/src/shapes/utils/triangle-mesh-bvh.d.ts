import type { TriangleMeshData } from './triangle-mesh-data.js';
/** bvh split strategies */
export declare enum BvhSplitStrategy {
    /**
     * CENTER: Split at midpoint of center bounds.
     * - Fast: O(n) with simple min/max computation
     * - Good balance for most geometries
     */
    CENTER = 0,
    /**
     * AVERAGE: Split at arithmetic mean of centers.
     * - Fast: O(n) iteration to compute mean
     * - Can create unbalanced splits on skewed distributions
     */
    AVERAGE = 1,
    /**
     * SAH: Surface Area Heuristic with binning.
     * - Slow: O(n) with high constant factor (binning + cost evaluation)
     * - Best quality: optimal expected query cost
     * - Use for static meshes queried many times
     */
    SAH = 2
}
/** settings for BVH construction */
export type BvhBuildSettings = {
    /** split strategy to use */
    strategy: BvhSplitStrategy;
    /** maximum triangles per leaf node */
    maxLeafTris: number;
};
/** triangle mesh BVH stored as a flat number array */
export type TriangleMeshBVH = {
    /** packed node data */
    buffer: number[];
};
/** get triangle start index (leaf only) */
export declare function nodeTriStart(buffer: number[], offset: number): number;
/** get triangle count (leaf only). Decodes from negative flag. */
export declare function nodeTriCount(buffer: number[], offset: number): number;
export declare function build(data: TriangleMeshData, settings: BvhBuildSettings): TriangleMeshBVH;
export type TriangleMeshBvhStats = {
    nodeCount: number;
    leafCount: number;
    minDepth: number;
    maxDepth: number;
    avgDepth: number;
};
/** compute depth statistics of the BVH tree, useful for performance analysis */
export declare function stats(bvh: TriangleMeshBVH): TriangleMeshBvhStats;
