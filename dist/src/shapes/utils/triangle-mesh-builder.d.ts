import type { BvhSplitStrategy } from './triangle-mesh-bvh.js';
import * as triangleMeshBvh from './triangle-mesh-bvh.js';
import type { TriangleMeshData } from './triangle-mesh-data.js';
/** settings for building a triangle mesh */
export type TriangleMeshBuilderSettings = {
    positions: number[];
    indices: number[];
    /**
     * Optional per-triangle material indices.
     * Length should match number of triangles (indices.length / 3).
     * Default: -1 for all triangles (no material).
     */
    materialIds?: number[];
    /**
     * Threshold for detecting degenerate triangles.
     * A triangle is considered degenerate if |cross product| < this value.
     */
    degenerateTolerance: number;
    /**
     * BVH split strategy.
     */
    bvhSplitStrategy: BvhSplitStrategy;
    /**
     * Maximum triangles per leaf node in the BVH.
     */
    bvhMaxLeafTris: number;
    /**
     * Cosine threshold for active edge determination.
     * Edges with cos(dihedral_angle) >= this value are considered smooth/inactive.
     */
    activeEdgeCosThresholdAngle: number;
};
/** result of building a triangle mesh */
export type TriangleMeshBuildResult = {
    data: TriangleMeshData;
    bvh: triangleMeshBvh.TriangleMeshBVH;
};
/** build a triangle mesh from positions and indices */
export declare function buildTriangleMesh(settings: TriangleMeshBuilderSettings): TriangleMeshBuildResult;
