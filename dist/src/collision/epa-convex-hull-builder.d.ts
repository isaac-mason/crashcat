/**
 * triangle pool, flat double storage with stride TRI_FLOAT_STRIDE (9), indexed by triangle index:
 * [normalX, normalY, normalZ, centroidX, centroidY, centroidZ, closestLengthSq, lambda0, lambda1].
 * a triangle "is" its index into these arrays; -1 means null.
 */
export declare const TRI_FLOAT_STRIDE = 9;
/** offset of the triangle normal (3 slots) within a float record */
export declare const TRI_NORMAL = 0;
/** offset of the triangle centroid (3 slots) within a float record */
export declare const TRI_CENTROID = 3;
/** offset of the signed squared distance from origin to triangle plane within a float record */
export declare const TRI_CLOSEST_LENGTH_SQ = 6;
/** offset of barycentric coordinate lambda0 within a float record */
export declare const TRI_LAMBDA0 = 7;
/** offset of barycentric coordinate lambda1 within a float record */
export declare const TRI_LAMBDA1 = 8;
/**
 * triangle pool, flat integer storage with stride TRI_INT_STRIDE (11), indexed by triangle index.
 * the three edges live at slot `edge * 3`, each edge holding
 * [neighbourTriangle (-1 for null), neighbourEdge, startIndex] (slots 0-8); flags (9); nextFree (10).
 */
export declare const TRI_INT_STRIDE = 11;
/** per-edge offset (relative to `edge * 3`) of the neighbouring triangle index (-1 for null) */
export declare const EDGE_NEIGHBOUR_TRIANGLE = 0;
/** per-edge offset (relative to `edge * 3`) of the neighbouring edge index */
export declare const EDGE_NEIGHBOUR_EDGE = 1;
/** per-edge offset (relative to `edge * 3`) of the edge start vertex index */
export declare const EDGE_START_INDEX = 2;
/** offset of the packed flags word within an int record */
export declare const TRI_FLAGS = 9;
/** offset of the free-list link (next free triangle index, -1 if none) within an int record */
export declare const TRI_NEXT_FREE = 10;
/** flags bit: triangle has been removed from the hull */
export declare const TRI_FLAG_REMOVED = 1;
/** flags bit: triangle is currently in the priority queue */
export declare const TRI_FLAG_IN_QUEUE = 2;
/** flags bit: barycentric coordinates are relative to vertex 0 (else vertex 1) */
export declare const TRI_FLAG_LAMBDA_RELATIVE_TO_0 = 4;
/** flags bit: closest point to origin lies within the triangle interior */
export declare const TRI_FLAG_CLOSEST_POINT_INTERIOR = 8;
/**
 * silhouette edge list output by findEdge, flat with stride EDGE_STRIDE (3):
 * [neighbourTriangle, neighbourEdge, startIndex].
 */
export declare const EDGE_STRIDE = 3;
export declare function triangleIsFacing(state: EpaConvexHullBuilderState, tri: number, points: number[], offset: number): boolean;
/** flat stride-3 point store: `values` packs [x, y, z] per point, `size` counts stored points */
export type Points = {
    values: number[];
    size: number;
};
export declare function createPoints(capacity: number): Points;
/** new triangle indices produced by addPoint */
export type NewTriangles = number[];
export type EpaConvexHullBuilderState = {
    /** triangle pool double storage, stride TRI_FLOAT_STRIDE */
    triFloat: number[];
    /** triangle pool integer storage, stride TRI_INT_STRIDE */
    triInt: number[];
    triangleHighWatermark: number;
    triangleFreeHead: number;
    /** priority queue of triangle indices (binary heap keyed by closestLengthSq) */
    queue: number[];
    /** flat stride-3 vertex positions (aliases the support-point y store) */
    positions: number[];
    /** flat DFS stack, stride STACK_STRIDE */
    stack: number[];
    /** flat silhouette edge list, stride EDGE_STRIDE */
    edges: number[];
    /** number of edges currently stored in `edges` */
    edgesSize: number;
};
export declare function init(): EpaConvexHullBuilderState;
export declare function linkTriangle(state: EpaConvexHullBuilderState, t1: number, edge1: number, t2: number, edge2: number): void;
export declare function createTriangle(state: EpaConvexHullBuilderState, idx1: number, idx2: number, idx3: number): number;
export declare function initialize(state: EpaConvexHullBuilderState, idx1: number, idx2: number, idx3: number): void;
export declare function hasNextTriangle(state: EpaConvexHullBuilderState): boolean;
export declare function peekClosestTriangleInQueue(state: EpaConvexHullBuilderState): number;
export declare function popClosestTriangleFromQueue(state: EpaConvexHullBuilderState): number;
export declare function findFacingTriangle(state: EpaConvexHullBuilderState, points: number[], offset: number, outBestDistSq: {
    value: number;
}): number;
export declare function freeTriangle(state: EpaConvexHullBuilderState, tri: number): void;
export declare function unlinkTriangle(state: EpaConvexHullBuilderState, tri: number): void;
export declare function findEdge(state: EpaConvexHullBuilderState, facingTriangle: number, pointOffset: number): boolean;
export declare function addPoint(state: EpaConvexHullBuilderState, facingTriangle: number, idx: number, closestDistSq: number, outTriangles: NewTriangles): boolean;
