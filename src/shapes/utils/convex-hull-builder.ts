import type { Vec3 } from 'mathcat';
import { vec3 } from 'mathcat';
import { createConvexHullBuilder2D, EResult as EResult2D, initialize as initialize2D } from './convex-hull-builder-2d';
import { assert } from '../../utils/assert';

// minimum triangle area squared
const MIN_TRIANGLE_AREA_SQ = 1e-12;

export enum Result {
    Success = 0,
    MaxVerticesReached = 1,
    TooFewPoints = 2,
    TooFewFaces = 3,
    Degenerate = 4,
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

export const create = (inPositions: Positions): ConvexHullBuilder => {
    return {
        positions: inPositions,
        faces: [],
        coplanarList: [],
    };
};

/**
 * Build the 3D convex hull
 * Main QuickHull algorithm implementation. Builds a 3D convex hull from the input points.
 *
 * @param builder the builder instance
 * @param inMaxVertices maximum vertices allowed in hull
 * @param inTolerance distance tolerance for point inclusion
 * @returns object with result code and error message (if any)
 */
export function initialize(
    builder: ConvexHullBuilder,
    inMaxVertices: number,
    inTolerance: number,
): { result: Result; error: string } {
    // free the faces possibly left over from an earlier hull
    freeFaces(builder);

    // test that we have at least 3 points
    if (builder.positions.length < 3) {
        return { result: Result.TooFewPoints, error: 'Need at least 3 points to make a hull' };
    }

    // determine a suitable tolerance for detecting that points are coplanar
    const coplanarToleranceSq = determineCoplanarDistance(builder) ** 2;

    // increase desired tolerance if accuracy doesn't allow it
    const toleranceSq = Math.max(coplanarToleranceSq, inTolerance * inTolerance);

    // find point furthest from the origin
    let idx1 = -1;
    let maxDistSq = -1.0;
    for (let i = 0; i < builder.positions.length; i++) {
        const distSq = vec3.squaredLength(builder.positions[i]);
        if (distSq > maxDistSq) {
            maxDistSq = distSq;
            idx1 = i;
        }
    }

    // find point that is furthest away from this point
    let idx2 = -1;
    maxDistSq = -1.0;
    for (let i = 0; i < builder.positions.length; i++) {
        if (i !== idx1) {
            const distSq = vec3.squaredDistance(builder.positions[i], builder.positions[idx1]);
            if (distSq > maxDistSq) {
                maxDistSq = distSq;
                idx2 = i;
            }
        }
    }

    // find point that forms the biggest triangle
    let idx3 = -1;
    let bestTriangleAreaSq = -1.0;
    const _temp1: Vec3 = [0, 0, 0];
    const _temp2: Vec3 = [0, 0, 0];
    const _cross: Vec3 = [0, 0, 0];

    for (let i = 0; i < builder.positions.length; i++) {
        if (i !== idx1 && i !== idx2) {
            vec3.subtract(_temp1, builder.positions[idx1], builder.positions[i]);
            vec3.subtract(_temp2, builder.positions[idx2], builder.positions[i]);
            vec3.cross(_cross, _temp1, _temp2);
            const triangleAreaSq = vec3.squaredLength(_cross);

            if (triangleAreaSq > bestTriangleAreaSq) {
                bestTriangleAreaSq = triangleAreaSq;
                idx3 = i;
            }
        }
    }

    if (bestTriangleAreaSq < MIN_TRIANGLE_AREA_SQ) {
        return {
            result: Result.Degenerate,
            error: 'Could not find a suitable initial triangle because its area was too small',
        };
    }

    // check if we have only 3 vertices
    if (builder.positions.length === 3) {
        // create two triangles (back to back)
        const t1 = createTriangle(builder, idx1, idx2, idx3);
        const t2 = createTriangle(builder, idx1, idx3, idx2);

        // link face edges
        linkFace(t1.firstEdge!, t2.firstEdge!.mNextEdge!.mNextEdge!);
        linkFace(t1.firstEdge!.mNextEdge!, t2.firstEdge!.mNextEdge!);
        linkFace(t1.firstEdge!.mNextEdge!.mNextEdge!, t2.firstEdge!);

        return { result: Result.Success, error: '' };
    }

    // find point that forms the biggest tetrahedron
    const _initialPlaneNormal: Vec3 = [0, 0, 0];
    const _initialPlaneCentroid: Vec3 = [0, 0, 0];
    const _edge1: Vec3 = [0, 0, 0];
    const _edge2: Vec3 = [0, 0, 0];

    vec3.subtract(_edge1, builder.positions[idx2], builder.positions[idx1]);
    vec3.subtract(_edge2, builder.positions[idx3], builder.positions[idx1]);
    vec3.cross(_initialPlaneNormal, _edge1, _edge2);
    vec3.normalize(_initialPlaneNormal, _initialPlaneNormal);

    vec3.add(_initialPlaneCentroid, builder.positions[idx1], builder.positions[idx2]);
    vec3.add(_initialPlaneCentroid, _initialPlaneCentroid, builder.positions[idx3]);
    vec3.scale(_initialPlaneCentroid, _initialPlaneCentroid, 1.0 / 3.0);

    let idx4 = -1;
    let maxDist = 0.0;
    const _diff: Vec3 = [0, 0, 0];

    for (let i = 0; i < builder.positions.length; i++) {
        if (i !== idx1 && i !== idx2 && i !== idx3) {
            vec3.subtract(_diff, builder.positions[i], _initialPlaneCentroid);
            const dist = vec3.dot(_diff, _initialPlaneNormal);
            if (Math.abs(dist) > Math.abs(maxDist)) {
                maxDist = dist;
                idx4 = i;
            }
        }
    }

    // check if the hull is coplanar
    if (maxDist * maxDist <= 25.0 * coplanarToleranceSq) {
        // first project all points in 2D space
        const _base1: Vec3 = [0, 0, 0];
        const _base2: Vec3 = [0, 0, 0];

        // find perpendicular by setting smallest component to 0 and swapping/negating others
        const absX = Math.abs(_initialPlaneNormal[0]);
        const absY = Math.abs(_initialPlaneNormal[1]);
        const absZ = Math.abs(_initialPlaneNormal[2]);

        if (absX <= absY && absX <= absZ) {
            // x is smallest
            vec3.set(_base1, 0, -_initialPlaneNormal[2], _initialPlaneNormal[1]);
        } else if (absY <= absX && absY <= absZ) {
            // y is smallest
            vec3.set(_base1, -_initialPlaneNormal[2], 0, _initialPlaneNormal[0]);
        } else {
            // z is smallest
            vec3.set(_base1, -_initialPlaneNormal[1], _initialPlaneNormal[0], 0);
        }
        vec3.normalize(_base1, _base1);

        vec3.cross(_base2, _initialPlaneNormal, _base1);

        // project all points to 2D
        const positions2d: Vec3[] = [];
        for (const v of builder.positions) {
            positions2d.push([vec3.dot(_base1, v), vec3.dot(_base2, v), 0.0]);
        }

        // build hull
        const edges2d: number[] = [];
        const builder2d = createConvexHullBuilder2D(positions2d);
        const result2d = initialize2D(builder2d, idx1, idx2, idx3, inMaxVertices, inTolerance, edges2d);

        // create faces (back to back)
        const f1 = createFace(builder);
        const f2 = createFace(builder);

        // create edges for face 1
        const edgesF1: Edge[] = [];
        for (const startIdx of edges2d) {
            const edge: Edge = {
                mFace: f1,
                mStartIdx: startIdx,
                mNextEdge: null,
                mNeighbourEdge: null,
            };

            if (edgesF1.length === 0) {
                f1.firstEdge = edge;
            } else {
                edgesF1[edgesF1.length - 1].mNextEdge = edge;
            }
            edgesF1.push(edge);
        }
        edgesF1[edgesF1.length - 1].mNextEdge = f1.firstEdge;

        // create edges for face 2 (reversed order)
        const edgesF2: Edge[] = [];
        for (let i = edges2d.length - 1; i >= 0; i--) {
            const edge: Edge = {
                mFace: f2,
                mStartIdx: edges2d[i],
                mNextEdge: null,
                mNeighbourEdge: null,
            };

            if (edgesF2.length === 0) {
                f2.firstEdge = edge;
            } else {
                edgesF2[edgesF2.length - 1].mNextEdge = edge;
            }
            edgesF2.push(edge);
        }
        edgesF2[edgesF2.length - 1].mNextEdge = f2.firstEdge;

        // link edges
        for (let i = 0; i < edges2d.length; i++) {
            linkFace(edgesF1[i], edgesF2[(2 * edges2d.length - 2 - i) % edges2d.length]);
        }

        // calculate the plane for both faces
        calculateNormalAndCentroid(f1, builder.positions);
        vec3.negate(f2.normal, f1.normal);
        vec3.copy(f2.centroid, f1.centroid);

        return {
            result: result2d === EResult2D.MaxVerticesReached ? Result.MaxVerticesReached : Result.Success,
            error: '',
        };
    }

    // ensure the planes are facing outwards
    if (maxDist < 0.0) {
        const temp = idx2;
        idx2 = idx3;
        idx3 = temp;
    }

    // create tetrahedron
    const t1 = createTriangle(builder, idx1, idx2, idx4);
    const t2 = createTriangle(builder, idx2, idx3, idx4);
    const t3 = createTriangle(builder, idx3, idx1, idx4);
    const t4 = createTriangle(builder, idx1, idx3, idx2);

    // link face edges
    linkFace(t1.firstEdge!, t4.firstEdge!.mNextEdge!.mNextEdge!);
    linkFace(t1.firstEdge!.mNextEdge!, t2.firstEdge!.mNextEdge!.mNextEdge!);
    linkFace(t1.firstEdge!.mNextEdge!.mNextEdge!, t3.firstEdge!.mNextEdge!);
    linkFace(t2.firstEdge!, t4.firstEdge!.mNextEdge!);
    linkFace(t2.firstEdge!.mNextEdge!, t3.firstEdge!.mNextEdge!.mNextEdge!);
    linkFace(t3.firstEdge!, t4.firstEdge!);

    // build the initial conflict lists
    const faces: Faces = [t1, t2, t3, t4];
    for (let idx = 0; idx < builder.positions.length; idx++) {
        if (idx !== idx1 && idx !== idx2 && idx !== idx3 && idx !== idx4) {
            assignPointToFace(builder, idx, faces, toleranceSq);
        }
    }

    // overestimate of the actual amount of vertices we use, for limiting the amount of vertices in the hull
    let numVerticesUsed = 4;

    // loop through the remainder of the points and add them
    for (;;) {
        // find the face with the furthest point on it
        let faceWithFurthestPoint: Face | null = null;
        let furthestDistSq = 0.0;
        for (const f of builder.faces) {
            if (f.furthestPointDistanceSq > furthestDistSq) {
                furthestDistSq = f.furthestPointDistanceSq;
                faceWithFurthestPoint = f;
            }
        }

        let furthestPointIdx: number;
        if (faceWithFurthestPoint !== null) {
            // take the furthest point
            furthestPointIdx = faceWithFurthestPoint.conflictList.pop()!;
        } else if (builder.coplanarList.length > 0) {
            // try to assign points to faces (this also recalculates the distance to the hull for the coplanar vertices)
            const coplanar = builder.coplanarList;
            builder.coplanarList = [];
            let added = false;

            for (const c of coplanar) {
                added = assignPointToFace(builder, c.positionIdx, builder.faces, toleranceSq) || added;
            }

            // if we were able to assign a point, loop again to pick it up
            if (added) {
                continue;
            }

            // if the coplanar list is empty, there are no points left and we're done
            if (builder.coplanarList.length === 0) {
                break;
            }

            do {
                // find the vertex that is furthest from the hull
                let bestIdx = 0;
                let bestDistSq = builder.coplanarList[0].distanceSq;
                for (let idx = 1; idx < builder.coplanarList.length; idx++) {
                    const c = builder.coplanarList[idx];
                    if (c.distanceSq > bestDistSq) {
                        bestIdx = idx;
                        bestDistSq = c.distanceSq;
                    }
                }

                // swap it to the end
                const temp = builder.coplanarList[bestIdx];
                builder.coplanarList[bestIdx] = builder.coplanarList[builder.coplanarList.length - 1];
                builder.coplanarList[builder.coplanarList.length - 1] = temp;

                // remove it
                furthestPointIdx = builder.coplanarList.pop()!.positionIdx;

                // find the face for which the point is furthest away
                const result = { face: null as Face | null, distSq: 0 };
                getFaceForPoint(builder.positions[furthestPointIdx], builder.faces, result);
                faceWithFurthestPoint = result.face;
            } while (builder.coplanarList.length > 0 && faceWithFurthestPoint === null);

            if (faceWithFurthestPoint === null) {
                break;
            }
        } else {
            // if there are no more vertices, we're done
            break;
        }

        // check if we have a limit on the max vertices that we should produce
        if (numVerticesUsed >= inMaxVertices) {
            // count the actual amount of used vertices (we did not take the removal of any vertices into account)
            numVerticesUsed = getNumVerticesUsed(builder);

            // check if there are too many
            if (numVerticesUsed >= inMaxVertices) {
                return { result: Result.MaxVerticesReached, error: '' };
            }
        }

        // we're about to add another vertex
        ++numVerticesUsed;

        // add the point to the hull
        const newFaces: Faces = [];
        addPoint(builder, faceWithFurthestPoint, furthestPointIdx, coplanarToleranceSq, newFaces);

        // redistribute points on conflict lists belonging to removed faces
        for (const face of builder.faces) {
            if (face.removed) {
                for (const idx of face.conflictList) {
                    assignPointToFace(builder, idx, newFaces, toleranceSq);
                }
            }
        }

        // permanently delete faces that we removed in addPoint()
        garbageCollectFaces(builder);
    }

    // check if we are left with a hull. It is possible that hull building fails if the points are nearly coplanar.
    if (builder.faces.length < 2) {
        return { result: Result.TooFewFaces, error: 'Too few faces in hull' };
    }

    return { result: Result.Success, error: '' };
}

/**
 * Returns amount of vertices currently used by hull
 * Counts unique vertex indices used by all faces in the hull.
 */
export const getNumVerticesUsed = (builder: ConvexHullBuilder): number => {
    const usedVerts = new Set<number>();

    for (const face of builder.faces) {
        let edge = face.firstEdge;
        if (edge === null) continue;

        do {
            usedVerts.add(edge.mStartIdx);
            edge = edge.mNextEdge!;
        } while (edge !== face.firstEdge);
    }

    return usedVerts.size;
};

/**
 * ContainsFace - Check if hull contains polygon with given indices
 *
 * Checks if the hull contains a face with vertices matching the given indices in order.
 * Used for validation and debugging.
 */
export const containsFace = (builder: ConvexHullBuilder, inIndices: readonly number[]): boolean => {
    // loop through all faces
    for (const f of builder.faces) {
        const e = f.firstEdge;
        if (e === null) continue;

        // find if any index from inIndices appears in this face's first edge
        const firstIndex = inIndices.indexOf(e.mStartIdx);
        if (firstIndex !== -1) {
            let matches = 0;
            let edge: Edge | null = e;
            let indexPos = firstIndex;

            do {
                // check if index matches
                if (inIndices[indexPos] !== edge.mStartIdx) {
                    break;
                }

                // increment number of matches
                matches++;

                // next index in list of inIndices (circular)
                indexPos++;
                if (indexPos === inIndices.length) {
                    indexPos = 0;
                }

                // next edge
                edge = edge.mNextEdge;
            } while (edge !== null && edge !== f.firstEdge);

            if (matches === inIndices.length) {
                return true;
            }
        }
    }

    return false;
};

const _getCOM_v1MinusV4: Vec3 = [0, 0, 0];
const _getCOM_v2MinusV4: Vec3 = [0, 0, 0];
const _getCOM_v3MinusV4: Vec3 = [0, 0, 0];
const _getCOM_cross: Vec3 = [0, 0, 0];

/**
 * Calculate center of mass and volume.
 * Calculates the center of mass and volume of the convex hull by decomposing it
 * into tetrahedra. Each face is triangulated into a fan from its centroid.
 */
export const getCenterOfMassAndVolume = (builder: ConvexHullBuilder): { centerOfMass: Vec3; volume: number } => {
    // fourth point is the average of all face centroids
    const v4: Vec3 = [0, 0, 0];
    for (const f of builder.faces) {
        vec3.add(v4, v4, f.centroid);
    }
    vec3.scale(v4, v4, 1.0 / builder.faces.length);

    // calculate mass and center of mass of this convex hull by summing all tetrahedrons
    let volume = 0.0;
    const centerOfMass: Vec3 = [0, 0, 0];

    for (const f of builder.faces) {
        // get the first vertex that we'll use to create a triangle fan
        let e = f.firstEdge;
        if (e === null) continue;

        const v1 = builder.positions[e.mStartIdx];

        // get the second vertex
        e = e.mNextEdge;
        if (e === null || e === f.firstEdge) continue;

        let v2 = builder.positions[e.mStartIdx];

        for (e = e.mNextEdge; e !== null && e !== f.firstEdge; e = e.mNextEdge) {
            // fetch the last point of the triangle
            const v3 = builder.positions[e.mStartIdx];

            // calculate center of mass and mass of this tetrahedron,
            // see: https://en.wikipedia.org/wiki/Tetrahedron#Volume
            // volume = (v1 - v4).Dot((v2 - v4).Cross(v3 - v4))
            vec3.subtract(_getCOM_v1MinusV4, v1, v4);
            vec3.subtract(_getCOM_v2MinusV4, v2, v4);
            vec3.subtract(_getCOM_v3MinusV4, v3, v4);
            vec3.cross(_getCOM_cross, _getCOM_v2MinusV4, _getCOM_v3MinusV4);
            const volumeTetrahedron = vec3.dot(_getCOM_v1MinusV4, _getCOM_cross); // Needs to be divided by 6, postpone this until the end

            // center_of_mass = v1 + v2 + v3 + v4 (needs to be divided by 4, postpone this until the end)
            // compute weighted sum directly without allocating intermediate vector
            const comX = v1[0] + v2[0] + v3[0] + v4[0];
            const comY = v1[1] + v2[1] + v3[1] + v4[1];
            const comZ = v1[2] + v2[2] + v3[2] + v4[2];

            // accumulate results
            volume += volumeTetrahedron;
            centerOfMass[0] += volumeTetrahedron * comX;
            centerOfMass[1] += volumeTetrahedron * comY;
            centerOfMass[2] += volumeTetrahedron * comZ;

            // update v2 for next triangle
            v2 = v3;
        }
    }

    // calculate center of mass, fall back to average point in case there is no volume (everything is on a plane in this case)
    const FLT_EPSILON = 1.1920929e-7;
    if (volume > FLT_EPSILON) {
        vec3.scale(centerOfMass, centerOfMass, 1.0 / (4.0 * volume));
    } else {
        vec3.copy(centerOfMass, v4);
    }

    volume /= 6.0;

    return { centerOfMass, volume };
};

/**
 * Find the point furthest outside the hull
 * Finds the point that is furthest outside the convex hull and returns information
 * about the face it's furthest from. Used for error analysis and tolerance testing.
 */
export function determineMaxError(builder: ConvexHullBuilder): {
    faceWithMaxError: Face | null;
    maxError: number;
    maxErrorPositionIdx: number;
    coplanarDistance: number;
} {
    const coplanarDistance = determineCoplanarDistance(builder);

    // this measures the distance from a polygon to the furthest point outside of the hull
    let maxError = 0.0;
    let maxErrorFace: Face | null = null;
    let maxErrorPoint = -1;

    for (let i = 0; i < builder.positions.length; i++) {
        const v = builder.positions[i];

        // this measures the closest edge from all faces to point v
        //
        // note that we take the min of all faces since there may be multiple near coplanar faces so if we were to test this per face
        // we may find that a point is outside of a polygon and mark it as an error, while it is actually inside a nearly coplanar polygon.
        let minEdgeDistSq = Number.MAX_VALUE;
        let minEdgeDistFace: Face | null = null;

        for (const f of builder.faces) {
            // check if point is on or in front of plane
            const normalLen = vec3.length(f.normal);
            // TODO: assert here? JPH_ASSERT(normalLen > 0.0f)

            const vMinusCentroid = vec3.subtract([0, 0, 0], v, f.centroid);
            const planeDist = vec3.dot(f.normal, vMinusCentroid) / normalLen;
            if (planeDist > -coplanarDistance) {
                // check distance to the edges of this face
                const edgeDistSq = getDistanceToEdgeSq(builder, v, f);
                if (edgeDistSq < minEdgeDistSq) {
                    minEdgeDistSq = edgeDistSq;
                    minEdgeDistFace = f;
                }

                // if the point is inside the polygon and the point is in front of the plane, measure the distance
                if (edgeDistSq === 0.0 && planeDist > maxError) {
                    maxError = planeDist;
                    maxErrorFace = f;
                    maxErrorPoint = i;
                }
            }
        }

        // if the minimum distance to an edge is further than our current max error, we use that as max error
        const minEdgeDist = Math.sqrt(minEdgeDistSq);
        if (minEdgeDistFace !== null && minEdgeDist > maxError) {
            maxError = minEdgeDist;
            maxErrorFace = minEdgeDistFace;
            maxErrorPoint = i;
        }
    }

    return {
        faceWithMaxError: maxErrorFace,
        maxError,
        maxErrorPositionIdx: maxErrorPoint,
        coplanarDistance,
    };
}

/** edge with start and end indices */
type FullEdge = {
    mNeighbourEdge: Edge | null;
    mStartIdx: number;
    mEndIdx: number;
};

const _calcNormal_edge0: Vec3 = [0, 0, 0];
const _calcNormal_edge1: Vec3 = [0, 0, 0];
const _calcNormal_edge2: Vec3 = [0, 0, 0];
const _calcNormal_normalE01: Vec3 = [0, 0, 0];
const _calcNormal_normalE02: Vec3 = [0, 0, 0];

/**
 * Calculates the normal and centroid for a face using a triangle fan approach.
 * The normal length is 2x the area of the face.
 */
const calculateNormalAndCentroid = (face: Face, positions: readonly Vec3[]): void => {
    // get point that we use to construct a triangle fan
    let e = face.firstEdge;
    if (e === null) return;

    const y0 = positions[e.mStartIdx];

    // get the 2nd point
    e = e.mNextEdge;
    if (e === null || e === face.firstEdge) return;

    let y1 = positions[e.mStartIdx];

    // start accumulating the centroid
    vec3.add(face.centroid, y0, y1);
    let n = 2;

    // start accumulating the normal
    vec3.set(face.normal, 0, 0, 0);

    // loop over remaining edges accumulating normals in a triangle fan fashion
    for (e = e.mNextEdge; e !== null && e !== face.firstEdge; e = e.mNextEdge) {
        // get the 3rd point
        const y2 = positions[e.mStartIdx];

        // calculate edges (counter clockwise)
        vec3.subtract(_calcNormal_edge0, y1, y0);
        vec3.subtract(_calcNormal_edge1, y2, y1);
        vec3.subtract(_calcNormal_edge2, y0, y2);

        // the best normal is calculated by using the two shortest edges
        // see: https://box2d.org/posts/2014/01/troublesome-troublesome-triangle/
        // we check which edge is shorter: e1 or e2
        const e1LengthSq = vec3.dot(_calcNormal_edge1, _calcNormal_edge1);
        const e2LengthSq = vec3.dot(_calcNormal_edge2, _calcNormal_edge2);
        const e1ShorterThanE2 = e1LengthSq < e2LengthSq;

        // calculate both normals
        vec3.cross(_calcNormal_normalE01, _calcNormal_edge0, _calcNormal_edge1);
        vec3.cross(_calcNormal_normalE02, _calcNormal_edge2, _calcNormal_edge0);

        // select the one with the shortest edge
        const selectedNormal = e1ShorterThanE2 ? _calcNormal_normalE01 : _calcNormal_normalE02;
        vec3.add(face.normal, face.normal, selectedNormal);

        // accumulate centroid
        vec3.add(face.centroid, face.centroid, y2);
        n++;

        // update y1 for next triangle
        y1 = y2;
    }

    // finalize centroid
    vec3.scale(face.centroid, face.centroid, 1.0 / n);
};

const _isFacing_diff: Vec3 = [0, 0, 0];

/** check if face is facing inPosition. Returns true if the point is in front of the face plane */
function isFacing(face: Face, inPosition: Vec3): boolean {
    // Calculate (inPosition - mCentroid)
    vec3.subtract(_isFacing_diff, inPosition, face.centroid);

    // Return mNormal.Dot(inPosition - mCentroid) > 0
    return vec3.dot(face.normal, _isFacing_diff) > 0.0;
}

/** walks the circular edge list to find the edge that points to this edge */
function getPreviousEdge(edge: Edge): Edge {
    let prevEdge = edge;
    while (prevEdge.mNextEdge !== null && prevEdge.mNextEdge !== edge) {
        prevEdge = prevEdge.mNextEdge;
    }
    return prevEdge;
}

/** deletes all faces in the hull */
function freeFaces(builder: ConvexHullBuilder): void {
    // clear references and let GC handle cleanup
    for (const face of builder.faces) {
        // free all edges
        if (face.firstEdge !== null) {
            let e = face.firstEdge;
            do {
                const next = e.mNextEdge;
                e.mNextEdge = null;
                e.mNeighbourEdge = null;
                if (next === null) break;
                e = next!;
            } while (e !== face.firstEdge);
            face.firstEdge = null;
        }
        face.conflictList = [];
    }

    builder.faces = [];
}

const _coplanarDist_vmax: Vec3 = [0, 0, 0];

/**
 * Formula as per: Implementing Quickhull - Dirk Gregorius.
 * Returns a suitable tolerance for detecting that points are coplanar.
 */
function determineCoplanarDistance(builder: ConvexHullBuilder): number {
    const positions = builder.positions;

    // find component-wise maximum absolute values
    vec3.set(_coplanarDist_vmax, 0, 0, 0);
    for (const v of positions) {
        _coplanarDist_vmax[0] = Math.max(_coplanarDist_vmax[0], Math.abs(v[0]));
        _coplanarDist_vmax[1] = Math.max(_coplanarDist_vmax[1], Math.abs(v[1]));
        _coplanarDist_vmax[2] = Math.max(_coplanarDist_vmax[2], Math.abs(v[2]));
    }

    // 3.0 * FLT_EPSILON * (vmax.x + vmax.y + vmax.z)
    const FLT_EPSILON = 1.1920929e-7; // Float32 machine epsilon
    return 3.0 * FLT_EPSILON * (_coplanarDist_vmax[0] + _coplanarDist_vmax[1] + _coplanarDist_vmax[2]);
}

const _getFaceForPoint_diff: Vec3 = [0, 0, 0];

/**
 * Find the face for which inPoint is furthest to the front.
 * Uses out params to avoid allocations.
 */
function getFaceForPoint(inPoint: Vec3, inFaces: readonly Face[], out: { face: Face | null; distSq: number }): void {
    out.face = null;
    out.distSq = 0.0;

    for (const f of inFaces) {
        if (f.removed) continue;

        // determine distance to face
        vec3.subtract(_getFaceForPoint_diff, inPoint, f.centroid);
        const dot = vec3.dot(f.normal, _getFaceForPoint_diff);

        if (dot > 0.0) {
            const distSq = (dot * dot) / vec3.squaredLength(f.normal);
            if (distSq > out.distSq) {
                out.face = f;
                out.distSq = distSq;
            }
        }
    }
}

const _getDistToEdge_p1MinusPoint: Vec3 = [0, 0, 0];
const _getDistToEdge_p2MinusPoint: Vec3 = [0, 0, 0];
const _getDistToEdge_p2MinusP1: Vec3 = [0, 0, 0];
const _getDistToEdge_pointMinusP1: Vec3 = [0, 0, 0];
const _getDistToEdge_cross: Vec3 = [0, 0, 0];
const _getDistToEdge_closestPoint: Vec3 = [0, 0, 0];

/**
 * Calculates the squared distance from inPoint to the edges of inFace.
 * Returns 0 if the point is inside all edges, otherwise returns the squared distance to the closest edge.
 */
function getDistanceToEdgeSq(builder: ConvexHullBuilder, inPoint: Vec3, inFace: Face): number {
    let allInside = true;
    let edgeDistSq = Number.MAX_VALUE;

    // test if it is inside the edges of the polygon
    let edge = inFace.firstEdge;
    if (edge === null) return 0;

    const prevEdge = getPreviousEdge(edge);
    let p1 = builder.positions[prevEdge.mStartIdx];

    do {
        const p2 = builder.positions[edge.mStartIdx];

        // check if point is outside this edge
        // (p2 - p1).Cross(inPoint - p1).Dot(inFace.mNormal) < 0
        vec3.subtract(_getDistToEdge_p2MinusP1, p2, p1);
        vec3.subtract(_getDistToEdge_pointMinusP1, inPoint, p1);
        vec3.cross(_getDistToEdge_cross, _getDistToEdge_p2MinusP1, _getDistToEdge_pointMinusP1);

        if (vec3.dot(_getDistToEdge_cross, inFace.normal) < 0.0) {
            // it is outside
            allInside = false;

            // calculate closest point on line segment
            vec3.subtract(_getDistToEdge_p1MinusPoint, p1, inPoint);
            vec3.subtract(_getDistToEdge_p2MinusPoint, p2, inPoint);

            // compute barycentric coordinate for closest point on line
            const p1p2 = vec3.dot(_getDistToEdge_p1MinusPoint, _getDistToEdge_p2MinusPoint);
            const p2p2 = vec3.dot(_getDistToEdge_p2MinusPoint, _getDistToEdge_p2MinusPoint);
            const p1p1 = vec3.dot(_getDistToEdge_p1MinusPoint, _getDistToEdge_p1MinusPoint);

            const denom = p1p1 + p2p2 - 2.0 * p1p2;
            let t = 0.0;

            if (Math.abs(denom) > 1e-10) {
                t = (p2p2 - p1p2) / denom;
                t = Math.max(0.0, Math.min(1.0, t)); // clamp to [0, 1]
            }

            // closest point = p1 * (1 - t) + p2 * t
            vec3.lerp(_getDistToEdge_closestPoint, _getDistToEdge_p1MinusPoint, _getDistToEdge_p2MinusPoint, t);
            const distSq = vec3.squaredLength(_getDistToEdge_closestPoint);

            edgeDistSq = Math.min(edgeDistSq, distSq);
        }

        p1 = p2;
        edge = edge.mNextEdge;
    } while (edge !== null && edge !== inFace.firstEdge);

    return allInside ? 0.0 : edgeDistSq;
}

const _getFaceForPoint_result = { face: null as Face | null, distSq: 0 };

/**
 * Assigns a position to one of the supplied faces based on which face is closest.
 * Returns true if the point was assigned to a face's conflict list.
 * Points within tolerance are either discarded or added to the coplanar list.
 */
function assignPointToFace(
    builder: ConvexHullBuilder,
    inPositionIdx: number,
    inFaces: readonly Face[],
    inToleranceSq: number,
): boolean {
    const point = builder.positions[inPositionIdx];

    // find the face for which the point is furthest away
    getFaceForPoint(point, inFaces, _getFaceForPoint_result);
    const bestFace = _getFaceForPoint_result.face;
    const bestDistSq = _getFaceForPoint_result.distSq;

    if (bestFace !== null) {
        // check if this point is within the tolerance margin to the plane
        if (bestDistSq <= inToleranceSq) {
            // check distance to edges
            const distToEdgeSq = getDistanceToEdgeSq(builder, point, bestFace);
            if (distToEdgeSq > inToleranceSq) {
                // point is outside of the face and too far away to discard
                builder.coplanarList.push({
                    positionIdx: inPositionIdx,
                    distanceSq: distToEdgeSq,
                });
            }
        } else {
            // this point is in front of the face, add it to the conflict list
            if (bestDistSq > bestFace.furthestPointDistanceSq) {
                // this point is further away than any others, update the distance and add point as last point
                bestFace.furthestPointDistanceSq = bestDistSq;
                bestFace.conflictList.push(inPositionIdx);
            } else {
                // not the furthest point, add it as the before last point
                const insertIdx = bestFace.conflictList.length - 1;
                bestFace.conflictList.splice(insertIdx, 0, inPositionIdx);
            }

            return true;
        }
    }

    return false;
}

/** creates a new empty face and adds it to the builder's face list */
function createFace(builder: ConvexHullBuilder): Face {
    const face: Face = {
        normal: [0, 0, 0],
        centroid: [0, 0, 0],
        conflictList: [],
        firstEdge: null,
        furthestPointDistanceSq: 0,
        removed: false,
    };

    builder.faces.push(face);

    return face;
}

/**
 * Creates a new triangle face from three vertex indices.
 * Initializes edges and calculates normal and centroid.
 */
function createTriangle(builder: ConvexHullBuilder, inIdx0: number, inIdx1: number, inIdx2: number): Face {
    const face = createFace(builder);

    // create 3 edges
    const e0: Edge = { mFace: face, mNextEdge: null, mNeighbourEdge: null, mStartIdx: inIdx0 };
    const e1: Edge = { mFace: face, mNextEdge: null, mNeighbourEdge: null, mStartIdx: inIdx1 };
    const e2: Edge = { mFace: face, mNextEdge: null, mNeighbourEdge: null, mStartIdx: inIdx2 };

    // link edges in circular list
    e0.mNextEdge = e1;
    e1.mNextEdge = e2;
    e2.mNextEdge = e0;
    face.firstEdge = e0;

    // calculate normal and centroid
    calculateNormalAndCentroid(face, builder.positions);

    return face;
}

/** marks a face as removed. The face must not have any neighbour edges */
function freeFace(face: Face): void {
    // TODO: debug build could validate face has no neighbour edges?

    // unlink from all neighbours (should already be done, but be safe)
    unlinkFace(face);

    // clear edges to help GC (edges form circular list)
    if (face.firstEdge !== null) {
        let e = face.firstEdge;
        do {
            const next = e.mNextEdge;
            e.mNextEdge = null;
            e.mNeighbourEdge = null;
            e = next!;
        } while (e !== null && e !== face.firstEdge);
        face.firstEdge = null;
    }

    // clear conflict list
    face.conflictList = [];

    // mark as removed
    face.removed = true;
}

/** links two edges as neighbours, validates that vertices match correctly */
function linkFace(inEdge1: Edge, inEdge2: Edge): void {
    // TODO: in debug build, could validate a) not already connected, b) not on same face, c) vertices match correctly

    // link up
    inEdge1.mNeighbourEdge = inEdge2;
    inEdge2.mNeighbourEdge = inEdge1;
}

/**
 * Unlinks a face from all of its neighbouring faces.
 */
function unlinkFace(inFace: Face): void {
    // unlink from neighbours
    let e = inFace.firstEdge;
    if (e === null) return;

    do {
        if (e.mNeighbourEdge !== null) {
            // unlink bidirectionally
            e.mNeighbourEdge.mNeighbourEdge = null;
            e.mNeighbourEdge = null;
        }
        e = e.mNextEdge;
    } while (e !== null && e !== inFace.firstEdge);
}

/** used by findEdge for non-recursive traversal */
type StackEntry = {
    mFirstEdge: Edge;
    mCurrentEdge: Edge;
    mVisited: boolean;
};

/**
 * Finds the horizon edges when adding a new vertex to the convex hull.
 * Uses a stack-based traversal to avoid recursion.
 * Marks all visible faces as removed and returns the edges forming the horizon.
 */
function findEdge(inFacingFace: Face, inVertex: Vec3, outEdges: FullEdge[]): void {
    assert(outEdges.length === 0, 'findEdge: outEdges must be empty');

    // should start with a facing face
    if (!isFacing(inFacingFace, inVertex)) {
        throw new Error('findEdge: inFacingFace must be facing inVertex');
    }

    // flag as removed
    inFacingFace.removed = true;

    // instead of recursing, we build our own stack with the information we need
    const MAX_EDGE_LENGTH = 128;
    const stack: StackEntry[] = [];
    let curStackPos = 0;

    // start with the face/edge provided
    const firstEdge = inFacingFace.firstEdge;
    if (firstEdge === null) {
        throw new Error('findEdge: inFacingFace has no edges');
    }

    stack[0] = {
        mFirstEdge: firstEdge,
        mCurrentEdge: firstEdge,
        mVisited: false,
    };

    for (;;) {
        const curEntry = stack[curStackPos];

        // get current edge and advance to next
        const e = curEntry.mCurrentEdge;

        if (e.mNextEdge === null) {
            throw new Error('findEdge: edge has no next edge');
        }
        curEntry.mCurrentEdge = e.mNextEdge;

        // if we're back at the first edge we've completed the face and we're done
        if (e === curEntry.mFirstEdge && curEntry.mVisited) {
            // this face needs to be removed, unlink it now, caller will free
            unlinkFace(e.mFace);

            // pop from stack
            curStackPos--;
            if (curStackPos < 0) {
                break;
            }
        } else {
            curEntry.mVisited = true;

            // visit neighbour face
            const ne = e.mNeighbourEdge;
            if (ne !== null) {
                const n = ne.mFace;
                if (!n.removed) {
                    // check if vertex is on the front side of this face
                    if (isFacing(n, inVertex)) {
                        // vertex on front, this face needs to be removed
                        n.removed = true;

                        // add element to the stack of elements to visit
                        curStackPos++;
                        if (curStackPos >= MAX_EDGE_LENGTH) {
                            throw new Error('findEdge: stack overflow (too many edges)');
                        }

                        if (ne.mNextEdge === null) {
                            throw new Error('findEdge: neighbour edge has no next edge');
                        }

                        stack[curStackPos] = {
                            mFirstEdge: ne,
                            mCurrentEdge: ne.mNextEdge, // we don't need to test this edge again since we came from it
                            mVisited: false,
                        };
                    } else {
                        // vertex behind, keep edge
                        const full: FullEdge = {
                            mNeighbourEdge: ne,
                            mStartIdx: e.mStartIdx,
                            mEndIdx: ne.mStartIdx,
                        };
                        outEdges.push(full);
                    }
                }
            }
        }
    }

    // validate that we have a fully connected loop (debug assertion, could be removed in release build)
    for (let i = 0; i < outEdges.length; i++) {
        const endIdx = outEdges[i].mEndIdx;
        const nextStartIdx = outEdges[(i + 1) % outEdges.length].mStartIdx;
        if (endIdx !== nextStartIdx) {
            throw new Error('findEdge: edges do not form a connected loop');
        }
    }
}

/**
 * Adds a new vertex to the convex hull.
 * - Finds horizon edges using findEdge
 * - Creates new faces connecting horizon to the vertex
 * - Links new faces together and to existing hull
 * - Calls merging functions to clean up (these are stubbed for now)
 */
function addPoint(
    builder: ConvexHullBuilder,
    inFacingFace: Face,
    inIdx: number,
    inCoplanarToleranceSq: number,
    outNewFaces: Face[],
): void {
    // get position
    const pos = builder.positions[inIdx];

    // find edge of convex hull of faces that are not facing the new vertex
    const edges: FullEdge[] = [];
    findEdge(inFacingFace, pos, edges);

    if (edges.length < 3) {
        throw new Error('addPoint: need at least 3 edges to form valid hull');
    }

    // create new faces
    outNewFaces.length = 0; // clear the output array
    for (const e of edges) {
        if (e.mStartIdx === e.mEndIdx) {
            throw new Error('addPoint: edge start and end indices must be different');
        }
        const f = createTriangle(builder, e.mStartIdx, e.mEndIdx, inIdx);
        outNewFaces.push(f);
    }

    // link edges
    for (let i = 0; i < outNewFaces.length; i++) {
        const face = outNewFaces[i];
        const edge = edges[i];

        if (face.firstEdge === null) {
            throw new Error('addPoint: face has no edges');
        }

        // link to the horizon edge's neighbour
        if (edge.mNeighbourEdge !== null) {
            linkFace(face.firstEdge, edge.mNeighbourEdge);
        }

        // link to next new face in the ring
        const nextFace = outNewFaces[(i + 1) % outNewFaces.length];
        if (nextFace.firstEdge === null || face.firstEdge.mNextEdge === null) {
            throw new Error('addPoint: face edges not properly initialized');
        }
        if (nextFace.firstEdge.mNextEdge === null || nextFace.firstEdge.mNextEdge.mNextEdge === null) {
            throw new Error('addPoint: next face edges not properly initialized');
        }

        linkFace(face.firstEdge.mNextEdge, nextFace.firstEdge.mNextEdge.mNextEdge);
    }

    // loop on faces that were modified until nothing needs to be checked anymore
    const affectedFaces: Face[] = [...outNewFaces];
    while (affectedFaces.length > 0) {
        // take the next face
        const face = affectedFaces.pop()!;

        if (!face.removed) {
            // merge with neighbour if this is a degenerate face
            mergeDegenerateFace(builder, face, affectedFaces);

            // merge with coplanar neighbours (or when the neighbour forms a concave edge)
            if (!face.removed) {
                mergeCoplanarOrConcaveFaces(builder, face, inCoplanarToleranceSq, affectedFaces);
            }
        }
    }
}

/** removes all faces marked with mRemoved from the builder's face list, iterates backwards for safe removal during iteration. */
function garbageCollectFaces(builder: ConvexHullBuilder): void {
    // iterate backwards for safe removal
    for (let i = builder.faces.length - 1; i >= 0; i--) {
        const f = builder.faces[i];
        if (f.removed) {
            freeFace(f);
            builder.faces.splice(i, 1);
        }
    }
}

/** merges two faces by removing a shared edge */
function mergeFaces(builder: ConvexHullBuilder, inEdge: Edge): void {
    // get the face
    const face = inEdge.mFace;

    // find the previous and next edge
    const nextEdge = inEdge.mNextEdge;
    const prevEdge = getPreviousEdge(inEdge);

    // get the other face
    const otherEdge = inEdge.mNeighbourEdge!;
    const otherFace = otherEdge.mFace;

    // check if attempting to merge with self
    if (face === otherFace) {
        throw new Error('Cannot merge face with itself');
    }

    // loop over the edges of the other face and make them belong to inFace
    let edge = otherEdge.mNextEdge!;
    prevEdge.mNextEdge = edge;
    for (;;) {
        edge.mFace = face;
        if (edge.mNextEdge === otherEdge) {
            // terminate when we are back at other_edge
            edge.mNextEdge = nextEdge;
            break;
        }
        edge = edge.mNextEdge!;
    }

    // if the first edge happens to be inEdge we need to fix it because this edge is no longer part of the face.
    //
    // note that we replace it with the first edge of the merged face so that if the MergeFace function is called
    // from a loop that loops around the face that it will still terminate after visiting all edges once.
    if (face.firstEdge === inEdge) {
        face.firstEdge = prevEdge.mNextEdge;
    }

    // free the edges (we just remove references - garbage collector will handle cleanup)
    // note: in a pool-based version, these would return to the pool

    // mark the other face as removed
    otherFace.firstEdge = null;
    otherFace.removed = true;

    // recalculate plane
    calculateNormalAndCentroid(face, builder.positions);

    // merge conflict lists
    if (face.furthestPointDistanceSq > otherFace.furthestPointDistanceSq) {
        // this face has a point that's further away, make sure it remains the last one as we add the other points to this faces list
        const lastPoint = face.conflictList.pop()!;
        face.conflictList.push(...otherFace.conflictList);
        face.conflictList.push(lastPoint);
    } else {
        // the other face has a point that's furthest away, add that list at the end.
        face.conflictList.push(...otherFace.conflictList);
        face.furthestPointDistanceSq = otherFace.furthestPointDistanceSq;
    }
    otherFace.conflictList.length = 0;
}

/** merges a degenerate (very small area) face with its longest edge neighbor */
function mergeDegenerateFace(builder: ConvexHullBuilder, inFace: Face, ioAffectedFaces: Face[]): void {
    // check area of face
    if (vec3.squaredLength(inFace.normal) < MIN_TRIANGLE_AREA_SQ) {
        // find longest edge, since this face is a sliver this should keep the face convex
        let maxLengthSq = 0.0;
        let longestEdge: Edge | null = null;
        let e = inFace.firstEdge!;
        let p1 = builder.positions[e.mStartIdx];
        do {
            const next = e.mNextEdge!;
            const p2 = builder.positions[next.mStartIdx];
            const lengthSq = vec3.squaredDistance(p2, p1);
            if (lengthSq >= maxLengthSq) {
                maxLengthSq = lengthSq;
                longestEdge = e;
            }
            p1 = p2;
            e = next;
        } while (e !== inFace.firstEdge);

        // merge with face on longest edge
        mergeFaces(builder, longestEdge!);

        // remove any invalid edges
        removeInvalidEdges(builder, inFace, ioAffectedFaces);
    }
}

const _mergeCoplanar_deltaCentroid: Vec3 = [0, 0, 0];

/** merges faces that are coplanar or concave with the current face */
function mergeCoplanarOrConcaveFaces(
    builder: ConvexHullBuilder,
    inFace: Face,
    inCoplanarToleranceSq: number,
    ioAffectedFaces: Face[],
): void {
    let merged = false;

    let edge = inFace.firstEdge!;
    do {
        // store next edge since this edge can be removed
        const nextEdge = edge.mNextEdge!;

        // test if centroid of one face is above plane of the other face by inCoplanarToleranceSq.
        // if so we need to merge other face into inFace.
        const otherFace = edge.mNeighbourEdge!.mFace;
        vec3.subtract(_mergeCoplanar_deltaCentroid, otherFace.centroid, inFace.centroid);
        const deltaCentroid = _mergeCoplanar_deltaCentroid;
        const distOtherFaceCentroid = vec3.dot(inFace.normal, deltaCentroid);
        const signedDistOtherFaceCentroidSq = Math.abs(distOtherFaceCentroid) * distOtherFaceCentroid;
        const distFaceCentroid = -vec3.dot(otherFace.normal, deltaCentroid);
        const signedDistFaceCentroidSq = Math.abs(distFaceCentroid) * distFaceCentroid;
        const faceNormalLenSq = vec3.squaredLength(inFace.normal);
        const otherFaceNormalLenSq = vec3.squaredLength(otherFace.normal);

        if (
            (signedDistOtherFaceCentroidSq > -inCoplanarToleranceSq * faceNormalLenSq ||
                signedDistFaceCentroidSq > -inCoplanarToleranceSq * otherFaceNormalLenSq) &&
            vec3.dot(inFace.normal, otherFace.normal) > 0.0 // Never merge faces that are back to back
        ) {
            mergeFaces(builder, edge);
            merged = true;
        }

        edge = nextEdge;
    } while (edge !== inFace.firstEdge);

    if (merged) {
        removeInvalidEdges(builder, inFace, ioAffectedFaces);
    }
}

/** adds a face to the affected faces list if not already present */
function markAffected(inFace: Face, ioAffectedFaces: Face[]): void {
    if (!ioAffectedFaces.includes(inFace)) {
        ioAffectedFaces.push(inFace);
    }
}

/** removes edges that are invalid after face merging (self-loops and duplicate edges) */
function removeInvalidEdges(builder: ConvexHullBuilder, inFace: Face, ioAffectedFaces: Face[]): void {
    // this marks that the plane needs to be recalculated (we delay this until the end of the
    // function since we don't use the plane and we want to avoid calculating it multiple times)
    let recalculatePlane = false;

    // we keep going through this loop until no more edges were removed
    let removed: boolean;
    do {
        removed = false;

        // loop over all edges in this face
        let edge = inFace.firstEdge!;
        let neighbourFace = edge.mNeighbourEdge!.mFace;
        do {
            const nextEdge = edge.mNextEdge!;
            const nextNeighbourFace = nextEdge.mNeighbourEdge!.mFace;

            if (neighbourFace === inFace) {
                // we only remove 1 edge at a time, check if this edge's next edge is our neighbour.
                // if this check fails, we will continue to scan along the edge until we find an edge where this is the case.
                if (edge.mNeighbourEdge === nextEdge) {
                    // this edge leads back to the starting point, this means the edge is interior and needs to be removed

                    // remove edge
                    const prevEdge = getPreviousEdge(edge);
                    prevEdge.mNextEdge = nextEdge.mNextEdge;
                    if (inFace.firstEdge === edge || inFace.firstEdge === nextEdge) {
                        inFace.firstEdge = prevEdge;
                    }
                    // delete edge and nextEdge (we just remove references)

                    // check if inFace now has only 2 edges left
                    if (removeTwoEdgeFace(inFace, ioAffectedFaces)) {
                        return; // Bail if face no longer exists
                    }

                    // restart the loop
                    recalculatePlane = true;
                    removed = true;
                    break;
                }
            } else if (neighbourFace === nextNeighbourFace) {
                // there are two edges that connect to the same face, we will remove the second one

                // first merge the neighbours edges
                const neighbourEdge = nextEdge.mNeighbourEdge!;
                const nextNeighbourEdge = neighbourEdge.mNextEdge!;
                if (neighbourFace.firstEdge === nextNeighbourEdge) {
                    neighbourFace.firstEdge = neighbourEdge;
                }
                neighbourEdge.mNextEdge = nextNeighbourEdge.mNextEdge;
                neighbourEdge.mNeighbourEdge = edge;
                // delete nextNeighbourEdge

                // then merge my own edges
                if (inFace.firstEdge === nextEdge) {
                    inFace.firstEdge = edge;
                }
                edge.mNextEdge = nextEdge.mNextEdge;
                edge.mNeighbourEdge = neighbourEdge;
                // delete nextEdge

                // check if neighbour has only 2 edges left
                if (!removeTwoEdgeFace(neighbourFace, ioAffectedFaces)) {
                    // no, we need to recalculate its plane
                    calculateNormalAndCentroid(neighbourFace, builder.positions);

                    // mark neighbour face as affected
                    markAffected(neighbourFace, ioAffectedFaces);
                }

                // check if inFace now has only 2 edges left
                if (removeTwoEdgeFace(inFace, ioAffectedFaces)) {
                    return; // bail if face no longer exists
                }
                // restart loop
                recalculatePlane = true;
                removed = true;
                break;
            }

            // This edge is ok, go to the next edge
            edge = nextEdge;
            neighbourFace = nextNeighbourFace;
        } while (edge !== inFace.firstEdge);
    } while (removed);

    // recalculate plane?
    if (recalculatePlane) {
        calculateNormalAndCentroid(inFace, builder.positions);
    }
}

/** removes a face that has only 2 edges left (degenerate) */
function removeTwoEdgeFace(inFace: Face, ioAffectedFaces: Face[]): boolean {
    // check if this face contains only 2 edges
    const edge = inFace.firstEdge!;
    const nextEdge = edge.mNextEdge!;
    if (edge === nextEdge) {
        throw new Error('1 edge faces should not exist');
    }
    if (nextEdge.mNextEdge === edge) {
        // schedule both neighbours for re-checking
        const neighbourEdge = edge.mNeighbourEdge!;
        const neighbourFace = neighbourEdge.mFace;
        const nextNeighbourEdge = nextEdge.mNeighbourEdge!;
        const nextNeighbourFace = nextNeighbourEdge.mFace;
        markAffected(neighbourFace, ioAffectedFaces);
        markAffected(nextNeighbourFace, ioAffectedFaces);

        // link my neighbours to each other
        neighbourEdge.mNeighbourEdge = nextNeighbourEdge;
        nextNeighbourEdge.mNeighbourEdge = neighbourEdge;

        // unlink my edges
        edge.mNeighbourEdge = null;
        nextEdge.mNeighbourEdge = null;

        // mark this face as removed
        inFace.removed = true;

        return true;
    }

    return false;
}
