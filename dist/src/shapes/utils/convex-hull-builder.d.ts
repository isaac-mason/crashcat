import type { Vec3 } from 'mathcat';
export declare enum Result {
    Success = 0,
    MaxVerticesReached = 1,
    TooFewPoints = 2,
    TooFewFaces = 3,
    Degenerate = 4
}
export type Positions = Vec3[];
export type Faces = Face[];
export type Face = {
    normal: Vec3;
    centroid: Vec3;
    conflictList: number[];
    firstEdge: Edge | null;
    furthestPointDistanceSq: number;
    removed: boolean;
};
export type Edge = {
    mFace: Face;
    mNextEdge: Edge | null;
    mNeighbourEdge: Edge | null;
    mStartIdx: number;
};
export type ConvexHullBuilder = {
    positions: Positions;
    faces: Faces;
    coplanarList: Coplanar[];
};
type Coplanar = {
    positionIdx: number;
    distanceSq: number;
};
export declare const create: (inPositions: Positions) => ConvexHullBuilder;
/**
 * Build the 3D convex hull
 * Main QuickHull algorithm implementation. Builds a 3D convex hull from the input points.
 *
 * @param builder the builder instance
 * @param inMaxVertices maximum vertices allowed in hull
 * @param inTolerance distance tolerance for point inclusion
 * @returns object with result code and error message (if any)
 */
export declare function initialize(builder: ConvexHullBuilder, inMaxVertices: number, inTolerance: number): {
    result: Result;
    error: string;
};
/**
 * Returns amount of vertices currently used by hull
 * Counts unique vertex indices used by all faces in the hull.
 */
export declare const getNumVerticesUsed: (builder: ConvexHullBuilder) => number;
/**
 * ContainsFace - Check if hull contains polygon with given indices
 *
 * Checks if the hull contains a face with vertices matching the given indices in order.
 * Used for validation and debugging.
 */
export declare const containsFace: (builder: ConvexHullBuilder, inIndices: readonly number[]) => boolean;
/**
 * Calculate center of mass and volume.
 * Calculates the center of mass and volume of the convex hull by decomposing it
 * into tetrahedra. Each face is triangulated into a fan from its centroid.
 */
export declare const getCenterOfMassAndVolume: (builder: ConvexHullBuilder) => {
    centerOfMass: Vec3;
    volume: number;
};
/**
 * Find the point furthest outside the hull
 * Finds the point that is furthest outside the convex hull and returns information
 * about the face it's furthest from. Used for error analysis and tolerance testing.
 */
export declare function determineMaxError(builder: ConvexHullBuilder): {
    faceWithMaxError: Face | null;
    maxError: number;
    maxErrorPositionIdx: number;
    coplanarDistance: number;
};
export {};
