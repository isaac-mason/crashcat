import type { Vec3 } from 'mathcat';
import { vec3 } from 'mathcat';

export enum EResult {
    Success = 0,
    MaxVerticesReached = 1,
}

export type Positions = readonly Vec3[];
export type Edges = number[];

export type ConvexHullBuilder2D = {
    positions: Positions;
    _firstEdge: Edge | null;
    _numEdges: number;
};

export function createConvexHullBuilder2D(inPositions: Positions): ConvexHullBuilder2D {
    return {
        positions: inPositions,
        _firstEdge: null,
        _numEdges: 0,
    };
}

export function initialize(
    builder: ConvexHullBuilder2D,
    inIdx1: number,
    inIdx2: number,
    inIdx3: number,
    inMaxVertices: number,
    inTolerance: number,
    outEdges: Edges,
): EResult {
    // clear any leftovers
    freeEdges(builder);
    outEdges.length = 0;

    // reset flag
    let result = EResult.Success;

    // determine a suitable tolerance for detecting that points are colinear
    // formula as per: "Implementing Quickhull - Dirk Gregorius"
    const vmax: Vec3 = [0, 0, 0];
    for (const v of builder.positions) {
        vmax[0] = Math.max(vmax[0], Math.abs(v[0]));
        vmax[1] = Math.max(vmax[1], Math.abs(v[1]));
        vmax[2] = Math.max(vmax[2], Math.abs(v[2]));
    }
    const FLT_EPSILON = 1.1920929e-7;
    const colinearToleranceSq = (2.0 * FLT_EPSILON * (vmax[0] + vmax[1])) ** 2;

    // increase desired tolerance if accuracy doesn't allow it
    const toleranceSq = Math.max(colinearToleranceSq, inTolerance * inTolerance);

    // start with the initial indices in counter clockwise order
    // z = (mPositions[inIdx2] - mPositions[inIdx1]).Cross(mPositions[inIdx3] - mPositions[inIdx1]).GetZ()
    const p1 = builder.positions[inIdx1];
    const p2 = builder.positions[inIdx2];
    const p3 = builder.positions[inIdx3];
    const edge1x = p2[0] - p1[0];
    const edge1y = p2[1] - p1[1];
    const edge2x = p3[0] - p1[0];
    const edge2y = p3[1] - p1[1];
    const z = edge1x * edge2y - edge1y * edge2x; // cross product Z component

    if (z < 0.0) {
        // swap to make counter-clockwise
        const temp = inIdx1;
        inIdx1 = inIdx2;
        inIdx2 = temp;
    }

    // create and link edges
    const e1: Edge = {
        mNormal: [0, 0, 0],
        mCenter: [0, 0, 0],
        mConflictList: [],
        mPrevEdge: null,
        mNextEdge: null,
        mStartIdx: inIdx1,
        mFurthestPointDistanceSq: 0,
    };
    const e2: Edge = {
        mNormal: [0, 0, 0],
        mCenter: [0, 0, 0],
        mConflictList: [],
        mPrevEdge: null,
        mNextEdge: null,
        mStartIdx: inIdx2,
        mFurthestPointDistanceSq: 0,
    };
    const e3: Edge = {
        mNormal: [0, 0, 0],
        mCenter: [0, 0, 0],
        mConflictList: [],
        mPrevEdge: null,
        mNextEdge: null,
        mStartIdx: inIdx3,
        mFurthestPointDistanceSq: 0,
    };

    e1.mNextEdge = e2;
    e1.mPrevEdge = e3;
    e2.mNextEdge = e3;
    e2.mPrevEdge = e1;
    e3.mNextEdge = e1;
    e3.mPrevEdge = e2;
    builder._firstEdge = e1;
    builder._numEdges = 3;

    // build the initial conflict lists
    const edges = [e1, e2, e3];
    for (const edge of edges) {
        calculateNormalAndCenter(edge, builder.positions);
    }
    for (let idx = 0; idx < builder.positions.length; idx++) {
        if (idx !== inIdx1 && idx !== inIdx2 && idx !== inIdx3) {
            assignPointToEdge(builder, idx, edges);
        }
    }

    // add the remaining points to the hull
    for (; ;) {
        // check if we've reached the max amount of vertices that are allowed
        if (builder._numEdges >= inMaxVertices) {
            result = EResult.MaxVerticesReached;
            break;
        }

        // find the edge with the furthest point on it
        let edgeWithFurthestPoint: Edge | null = null;
        let furthestDistSq = 0.0;
        let edge = builder._firstEdge!;
        do {
            if (edge.mFurthestPointDistanceSq > furthestDistSq) {
                furthestDistSq = edge.mFurthestPointDistanceSq;
                edgeWithFurthestPoint = edge;
            }
            edge = edge.mNextEdge!;
        } while (edge !== builder._firstEdge);

        // if there is none closer than our tolerance value, we're done
        if (edgeWithFurthestPoint === null || furthestDistSq < toleranceSq) {
            break;
        }

        // take the furthest point
        const furthestPointIdx = edgeWithFurthestPoint.mConflictList.pop()!;
        const furthestPoint = builder.positions[furthestPointIdx];

        // find the horizon of edges that need to be removed
        // walk backwards from edge_with_furthest_point
        let firstEdge = edgeWithFurthestPoint;
        do {
            const prev = firstEdge.mPrevEdge!;
            if (!isFacing(prev, furthestPoint)) {
                break;
            }
            firstEdge = prev;
        } while (firstEdge !== edgeWithFurthestPoint);

        // walk forwards from edge_with_furthest_point
        let lastEdge = edgeWithFurthestPoint;
        do {
            const next = lastEdge.mNextEdge!;
            if (!isFacing(next, furthestPoint)) {
                break;
            }
            lastEdge = next;
        } while (lastEdge !== edgeWithFurthestPoint);

        // create new edges
        const newE1: Edge = {
            mNormal: [0, 0, 0],
            mCenter: [0, 0, 0],
            mConflictList: [],
            mPrevEdge: firstEdge.mPrevEdge,
            mNextEdge: null,
            mStartIdx: firstEdge.mStartIdx,
            mFurthestPointDistanceSq: 0,
        };
        const newE2: Edge = {
            mNormal: [0, 0, 0],
            mCenter: [0, 0, 0],
            mConflictList: [],
            mPrevEdge: null,
            mNextEdge: lastEdge.mNextEdge,
            mStartIdx: furthestPointIdx,
            mFurthestPointDistanceSq: 0,
        };
        newE1.mNextEdge = newE2;
        newE2.mPrevEdge = newE1;
        newE1.mPrevEdge!.mNextEdge = newE1;
        newE2.mNextEdge!.mPrevEdge = newE2;
        builder._firstEdge = newE1; // we could delete mFirstEdge so just update it to the newly created edge
        builder._numEdges += 2;

        // calculate normals
        const newEdges = [newE1, newE2];
        for (const newEdge of newEdges) {
            calculateNormalAndCenter(newEdge, builder.positions);
        }

        // delete the old edges
        for (; ;) {
            const next = firstEdge.mNextEdge!;

            // redistribute points in conflict list
            for (const idx of firstEdge.mConflictList) {
                assignPointToEdge(builder, idx, newEdges);
            }

            // clear the edge
            firstEdge.mConflictList = [];
            firstEdge.mNextEdge = null;
            firstEdge.mPrevEdge = null;
            builder._numEdges--;

            if (firstEdge === lastEdge) {
                break;
            }
            firstEdge = next;
        }
    }

    // convert the edge list to a list of indices
    let outputEdge = builder._firstEdge!;
    do {
        outEdges.push(outputEdge.mStartIdx);
        outputEdge = outputEdge.mNextEdge!;
    } while (outputEdge !== builder._firstEdge);

    return result;
}

type Edge = {
    mNormal: Vec3;
    mCenter: Vec3;
    mConflictList: number[];
    mPrevEdge: Edge | null;
    mNextEdge: Edge | null;
    mStartIdx: number;
    mFurthestPointDistanceSq: number;
};

/** free all edges, walks the circular edge list and removes all edges */
function freeEdges(builder: ConvexHullBuilder2D): void {
    if (builder._firstEdge === null) {
        return;
    }

    // walk circular list and clear all edges
    let edge = builder._firstEdge;
    do {
        const next = edge.mNextEdge;
        // clear references to help GC
        edge.mNextEdge = null;
        edge.mPrevEdge = null;
        edge.mConflictList = [];
        edge = next!;
    } while (edge !== null && edge !== builder._firstEdge);

    builder._firstEdge = null;
    builder._numEdges = 0;
}

/**
 * Assign a position to closest edge
 *
 * Finds the edge for which the point is furthest in front and adds it to that edge's conflict list.
 * Same logic as 3D assignPointToFace but for edges instead of faces.
 *
 * Maintains conflict list order:
 * - Last element is the furthest point
 * - Other elements are unsorted
 */
function assignPointToEdge(builder: ConvexHullBuilder2D, inPositionIdx: number, inEdges: readonly Edge[]): void {
    const point = builder.positions[inPositionIdx];

    let bestEdge: Edge | null = null;
    let bestDistSq = 0.0;

    // test against all edges
    for (const edge of inEdges) {
        // determine distance to edge
        const dx = point[0] - edge.mCenter[0];
        const dy = point[1] - edge.mCenter[1];
        const dz = point[2] - edge.mCenter[2];
        const dot = edge.mNormal[0] * dx + edge.mNormal[1] * dy + edge.mNormal[2] * dz;

        if (dot > 0.0) {
            const normalLengthSq =
                edge.mNormal[0] * edge.mNormal[0] + edge.mNormal[1] * edge.mNormal[1] + edge.mNormal[2] * edge.mNormal[2];
            const distSq = (dot * dot) / normalLengthSq;

            if (distSq > bestDistSq) {
                bestEdge = edge;
                bestDistSq = distSq;
            }
        }
    }

    // if this point is in front of the edge, add it to the conflict list
    if (bestEdge !== null) {
        if (bestDistSq > bestEdge.mFurthestPointDistanceSq) {
            // this point is further away than any others, update the distance and add point as last point
            bestEdge.mFurthestPointDistanceSq = bestDistSq;
            bestEdge.mConflictList.push(inPositionIdx);
        } else {
            // not the furthest point, add it as the before last point
            const insertIdx = bestEdge.mConflictList.length - 1;
            bestEdge.mConflictList.splice(insertIdx, 0, inPositionIdx);
        }
    }
}

/**
 * Validates that the edge structure is intact:
 * - All edges properly linked (next->prev and prev->next)
 * - Edge count matches mNumEdges
 */
export function validateEdges(builder: ConvexHullBuilder2D): void {
    if (builder._firstEdge === null) {
        if (builder._numEdges !== 0) {
            throw new Error('validateEdges: _firstEdge is null but _numEdges is not 0');
        }
        return;
    }

    let count = 0;
    let edge = builder._firstEdge;

    do {
        // Validate connectivity
        if (edge.mNextEdge === null) {
            throw new Error('validateEdges: edge.mNextEdge is null');
        }
        if (edge.mPrevEdge === null) {
            throw new Error('validateEdges: edge.mPrevEdge is null');
        }
        if (edge.mNextEdge.mPrevEdge !== edge) {
            throw new Error('validateEdges: edge.mNextEdge.mPrevEdge !== edge');
        }
        if (edge.mPrevEdge.mNextEdge !== edge) {
            throw new Error('validateEdges: edge.mPrevEdge.mNextEdge !== edge');
        }

        count++;
        edge = edge.mNextEdge;
    } while (edge !== builder._firstEdge);

    // validate that count matches
    if (count !== builder._numEdges) {
        throw new Error(`validateEdges: count (${count}) !== _numEdges (${builder._numEdges})`);
    }
}

/**
 * Calculates the center and outward-pointing normal for a 2D edge.
 *
 * The normal is calculated as:
 * - For edge from p1 to p2, edge vector = p2 - p1
 * - We want outward pointing normal (perpendicular to edge)
 * - Two choices: (-edge.y, edge.x, 0) or (edge.y, -edge.x, 0)
 * - We use (edge.y, -edge.x, 0) so that (normal × edge).z > 0 (points outward)
 */
function calculateNormalAndCenter(edge: Edge, positions: readonly Vec3[]): void {
    const p1 = positions[edge.mStartIdx];
    const p2 = positions[edge.mNextEdge!.mStartIdx];

    // center of edge
    vec3.add(edge.mCenter, p1, p2);
    vec3.scale(edge.mCenter, edge.mCenter, 0.5);

    // create outward pointing normal.
    const edgeX = p2[0] - p1[0];
    const edgeY = p2[1] - p1[1];
    vec3.set(edge.mNormal, edgeY, -edgeX, 0);
}

/** check if this edge is facing inPosition, returns true if the point is in front of the edge (outside the hull) */
function isFacing(edge: Edge, inPosition: Vec3): boolean {
    const dx = inPosition[0] - edge.mCenter[0];
    const dy = inPosition[1] - edge.mCenter[1];
    const dz = inPosition[2] - edge.mCenter[2];

    return edge.mNormal[0] * dx + edge.mNormal[1] * dy + edge.mNormal[2] * dz > 0.0;
}
