import type { Vec3 } from 'mathcat';
import type { Box3 } from 'mathcat/shapes';
export declare const TRIANGLE_STRIDE = 8;
export declare const OFFSET_INDEX_A = 0;
export declare const OFFSET_INDEX_B = 1;
export declare const OFFSET_INDEX_C = 2;
export declare const OFFSET_NORMAL_X = 3;
export declare const OFFSET_NORMAL_Y = 4;
export declare const OFFSET_NORMAL_Z = 5;
export declare const OFFSET_ACTIVE_EDGES = 6;
export declare const OFFSET_MATERIAL_ID = 7;
/** triangle mesh data in interleaved format */
export type TriangleMeshData = {
    /** deduplicated vertex positions [x0, y0, z0, x1, y1, z1, ...] */
    positions: number[];
    /** interleaved triangle data (stride = TRIANGLE_STRIDE) */
    triangleBuffer: number[];
    /** total number of triangles */
    triangleCount: number;
    /**
     * per-triangle mesh-local aabbs [minX, minY, minZ, maxX, maxY, maxZ, ...], 6 per triangle.
     * derived at build time (after the bvh build finalizes triangle order) so cast queries
     * read them instead of recomputing from the vertex indices per query.
     */
    triangleAABBs: number[];
};
/** get triangle vertex indices */
export declare function getTriangleIndices(out: [number, number, number], data: TriangleMeshData, triIdx: number): void;
/** get triangle vertex positions */
export declare function getTriangleVertices(outA: Vec3, outB: Vec3, outC: Vec3, data: TriangleMeshData, triIdx: number): void;
/** get triangle normal */
export declare function getTriangleNormal(out: Vec3, data: TriangleMeshData, triIdx: number): void;
/** set triangle normal */
export declare function setTriangleNormal(data: TriangleMeshData, triIdx: number, normal: Vec3): void;
/** get active edges bitmask (bit 0 = edge AB, bit 1 = edge BC, bit 2 = edge CA) */
export declare function getActiveEdges(data: TriangleMeshData, triIdx: number): number;
/** set active edges */
export declare function setActiveEdges(data: TriangleMeshData, triIdx: number, activeEdges: number): void;
/** get material id */
export declare function getMaterialId(data: TriangleMeshData, triIdx: number): number;
/** fill data.triangleAABBs from the (final, post-bvh-build) triangle buffer */
export declare function precomputeTriangleAABBs(data: TriangleMeshData): void;
/** calculate triangle aabb from vertices */
export declare function calculateTriangleAABB(out: Box3, data: TriangleMeshData, triIdx: number): void;
/** calculate triangle centroid from vertices */
export declare function calculateTriangleCentroid(out: Vec3, data: TriangleMeshData, triIdx: number): void;
/** swap two triangles */
export declare function swapTriangles(data: TriangleMeshData, indexA: number, indexB: number): void;
