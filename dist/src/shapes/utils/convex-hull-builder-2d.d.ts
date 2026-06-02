import type { Vec3 } from 'mathcat';
export declare enum EResult {
    Success = 0,
    MaxVerticesReached = 1
}
export type Positions = readonly Vec3[];
export type Edges = number[];
export type ConvexHullBuilder2D = {
    positions: Positions;
    _firstEdge: Edge | null;
    _numEdges: number;
};
export declare function createConvexHullBuilder2D(inPositions: Positions): ConvexHullBuilder2D;
export declare function initialize(builder: ConvexHullBuilder2D, inIdx1: number, inIdx2: number, inIdx3: number, inMaxVertices: number, inTolerance: number, outEdges: Edges): EResult;
type Edge = {
    mNormal: Vec3;
    mCenter: Vec3;
    mConflictList: number[];
    mPrevEdge: Edge | null;
    mNextEdge: Edge | null;
    mStartIdx: number;
    mFurthestPointDistanceSq: number;
};
/**
 * Validates that the edge structure is intact:
 * - All edges properly linked (next->prev and prev->next)
 * - Edge count matches mNumEdges
 */
export declare function validateEdges(builder: ConvexHullBuilder2D): void;
export {};
