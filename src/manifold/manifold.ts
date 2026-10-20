import { type Vec3, vec3 } from 'math';
import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import { assert } from '../utils/assert';
import type { Face } from '../utils/face';
import { createFace } from '../utils/face';
import { clipPolyVsEdge, clipPolyVsPoly } from './clip';

/** Maximum number of contact points in a manifold after reduction */
export const MAX_CONTACT_POINTS = 4;

/** Maximum number of contact points during polygon clipping (before reduction) */
export const MAX_CLIPPING_CONTACT_POINTS = 64;

/**
 * Contact manifold between two shapes.
 *
 * Contact points are stored relative to baseOffset.
 *
 * Pre-allocates space for 64 contact points
 * During manifold generation:
 * - ManifoldBetweenTwoFaces fills with up to 64 points
 * - PruneContactPoints reduces to max 4 points in-place
 * - numContactPoints tracks the current count
 */
export type ContactManifold = {
    /** number of active contact points (0-64 during clipping, 0-4 after reduction) */
    numContactPoints: number;

    /** base offset for all contact points */
    baseOffset: Vec3;

    /** contact point positions on shape A relative to baseOffset (flat array: [x1,y1,z1, x2,y2,z2, ...]) */
    relativeContactPointsOnA: number[];

    /** contact point positions on shape B relative to baseOffset (flat array: [x1,y1,z1, x2,y2,z2, ...]) */
    relativeContactPointsOnB: number[];

    /** contact normal (world space, normalized, points from A to B) */
    worldSpaceNormal: Vec3;

    /** penetration depth for the entire manifold (single value, not per-point) */
    penetrationDepth: number;

    /** sub-shape id of shape A */
    subShapeIdA: number;

    /** sub-shape id of shape B */
    subShapeIdB: number;

    /** material id of sub-shape A */
    materialIdA: number;

    /** material id of sub-shape B */
    materialIdB: number;
};

/** creates a new contact manifold with pre-allocated storage for 64 contact points */
export function createContactManifold(): ContactManifold {
    return {
        numContactPoints: 0,
        baseOffset: vec3.create(),
        relativeContactPointsOnA: new Array(MAX_CLIPPING_CONTACT_POINTS * 3).fill(0.0),
        relativeContactPointsOnB: new Array(MAX_CLIPPING_CONTACT_POINTS * 3).fill(0.0),
        worldSpaceNormal: vec3.create(),
        penetrationDepth: 0,
        subShapeIdA: EMPTY_SUB_SHAPE_ID,
        subShapeIdB: EMPTY_SUB_SHAPE_ID,
        materialIdA: -1,
        materialIdB: -1,
    };
}

export function resetContactManifold(manifold: ContactManifold): void {
    manifold.numContactPoints = 0;
    vec3.set(manifold.baseOffset, 0, 0, 0);
    vec3.set(manifold.worldSpaceNormal, 0, 0, 0);
    manifold.penetrationDepth = 0;
    manifold.subShapeIdA = EMPTY_SUB_SHAPE_ID;
    manifold.subShapeIdB = EMPTY_SUB_SHAPE_ID;
    manifold.materialIdA = -1;
    manifold.materialIdB = -1;
}

const _swapShapes_tempNormal = /* @__PURE__ */ vec3.create();

export function swapShapes(manifold: ContactManifold): void {
    // negate normal
    vec3.negate(_swapShapes_tempNormal, manifold.worldSpaceNormal);
    vec3.copy(manifold.worldSpaceNormal, _swapShapes_tempNormal);

    // swap sub-shape IDs
    const tempSubShapeId = manifold.subShapeIdA;
    manifold.subShapeIdA = manifold.subShapeIdB;
    manifold.subShapeIdB = tempSubShapeId;

    // swap material IDs
    const tempMaterialId = manifold.materialIdA;
    manifold.materialIdA = manifold.materialIdB;
    manifold.materialIdB = tempMaterialId;

    // swap relative contact point arrays (just swap references - zero allocation)
    const tempArray = manifold.relativeContactPointsOnA;
    manifold.relativeContactPointsOnA = manifold.relativeContactPointsOnB;
    manifold.relativeContactPointsOnB = tempArray;
}

/** gets relative contact point on shape A (relative to baseOffset) */
export function getRelativeContactPointOnA(out: Vec3, manifold: ContactManifold, index: number): Vec3 {
    const i = index * 3;
    out[0] = manifold.relativeContactPointsOnA[i];
    out[1] = manifold.relativeContactPointsOnA[i + 1];
    out[2] = manifold.relativeContactPointsOnA[i + 2];
    return out;
}

/** gets relative contact point on shape B (relative to baseOffset) */
export function getRelativeContactPointOnB(out: Vec3, manifold: ContactManifold, index: number): Vec3 {
    const i = index * 3;
    out[0] = manifold.relativeContactPointsOnB[i];
    out[1] = manifold.relativeContactPointsOnB[i + 1];
    out[2] = manifold.relativeContactPointsOnB[i + 2];
    return out;
}

/** gets world-space contact point on shape A (baseOffset + relativePoint) */
export function getWorldSpaceContactPointOnA(out: Vec3, manifold: ContactManifold, index: number): Vec3 {
    const i = index * 3;
    out[0] = manifold.baseOffset[0] + manifold.relativeContactPointsOnA[i];
    out[1] = manifold.baseOffset[1] + manifold.relativeContactPointsOnA[i + 1];
    out[2] = manifold.baseOffset[2] + manifold.relativeContactPointsOnA[i + 2];
    return out;
}

/** gets world-space contact point on shape B (baseOffset + relativePoint) */
export function getWorldSpaceContactPointOnB(out: Vec3, manifold: ContactManifold, index: number): Vec3 {
    const i = index * 3;
    out[0] = manifold.baseOffset[0] + manifold.relativeContactPointsOnB[i];
    out[1] = manifold.baseOffset[1] + manifold.relativeContactPointsOnB[i + 1];
    out[2] = manifold.baseOffset[2] + manifold.relativeContactPointsOnB[i + 2];
    return out;
}

// pre-allocated flat arrays for pruneContactPoints (max 64 projected points)
const _pruneProjectedPoints = new Float64Array(MAX_CLIPPING_CONTACT_POINTS * 3);
const _prunePenetrationDepthSq = new Float64Array(MAX_CLIPPING_CONTACT_POINTS);
const _pruneKeptOnA = new Float64Array(MAX_CONTACT_POINTS * 3);
const _pruneKeptOnB = new Float64Array(MAX_CONTACT_POINTS * 3);

const MIN_DISTANCE_SQ = 1.0e-6; // 1 mm²

/**
 * Reduces contact points from up to 64 down to max 4.
 *
 * Algorithm:
 * 1. Project all contact points onto plane perpendicular to penetrationAxis
 * 2. Calculate metric for each point: (distance_in_plane²) × (penetration_depth²)
 *    where penetration_depth = distance between contact points on shape1 vs shape2
 * 3. Greedily select 4 points:
 *    - Point 1: Maximum metric (furthest from COM in plane + deepest penetration)
 *    - Point 2: Maximum distance from Point 1 (maximize edge length)
 *    - Point 3 & 4: On opposite sides of P1-P2 edge (maximize area)
 * 4. Output in winding order: [P1, P3, P2, P4] to form a proper quad
 *
 * The selected points are gathered in scratch, then copied back to indices 0-3,
 * manifold.numContactPoints is updated to final count (2-4).
 *
 * @param manifold contactManifold with contact points to prune (modifies in-place)
 * @param penetrationAxis normalized penetration direction (from shape 1 to shape 2)
 */
export function pruneContactPoints(manifold: ContactManifold, penetrationAxis: Vec3): void {
    assert(manifold.numContactPoints > MAX_CONTACT_POINTS, 'pruneContactPoints should only be called with > 4 contact points');

    // early exit if already <= 4 points (safety fallback)
    if (manifold.numContactPoints <= MAX_CONTACT_POINTS) {
        return;
    }

    const numPoints = manifold.numContactPoints;
    const axisX = penetrationAxis[0];
    const axisY = penetrationAxis[1];
    const axisZ = penetrationAxis[2];
    assert(
        Math.abs(axisX * axisX + axisY * axisY + axisZ * axisZ - 1) <= 1.0e-6,
        'pruneContactPoints penetration axis must be normalized',
    );

    // step 1: project all contact points onto plane perpendicular to penetrationAxis
    for (let i = 0; i < numPoints; i++) {
        const i3 = i * 3;

        // get point on shape A
        const v1x = manifold.relativeContactPointsOnA[i3];
        const v1y = manifold.relativeContactPointsOnA[i3 + 1];
        const v1z = manifold.relativeContactPointsOnA[i3 + 2];

        // get point on shape B
        const v2x = manifold.relativeContactPointsOnB[i3];
        const v2y = manifold.relativeContactPointsOnB[i3 + 1];
        const v2z = manifold.relativeContactPointsOnB[i3 + 2];

        // project v1 onto plane with normal = penetrationAxis
        // projected = v1 - (v1 · axis) * axis
        // v1 · axis
        const dotProduct = v1x * axisX + v1y * axisY + v1z * axisZ;
        // v1 - axis * dotProduct
        const projX = v1x - axisX * dotProduct;
        const projY = v1y - axisY * dotProduct;
        const projZ = v1z - axisZ * dotProduct;

        // store projected point in flat array
        _pruneProjectedPoints[i3] = projX;
        _pruneProjectedPoints[i3 + 1] = projY;
        _pruneProjectedPoints[i3 + 2] = projZ;

        // penetration depth = distance between points on shapes
        // v2 - v1
        const dx = v2x - v1x;
        const dy = v2y - v1y;
        const dz = v2z - v1z;
        // squared length: dx² + dy² + dz²
        const depthSq = Math.max(MIN_DISTANCE_SQ, dx * dx + dy * dy + dz * dz);
        _prunePenetrationDepthSq[i] = depthSq;
    }

    // step 2: select point 1 (maximum metric in plane)
    let point1 = 0;
    // squared length for projected[0]
    const p0x = _pruneProjectedPoints[0];
    const p0y = _pruneProjectedPoints[1];
    const p0z = _pruneProjectedPoints[2];
    let maxMetric = Math.max(MIN_DISTANCE_SQ, p0x * p0x + p0y * p0y + p0z * p0z) * _prunePenetrationDepthSq[0];

    for (let i = 1; i < numPoints; i++) {
        const i3 = i * 3;
        // squared length for projected[i]
        const px = _pruneProjectedPoints[i3];
        const py = _pruneProjectedPoints[i3 + 1];
        const pz = _pruneProjectedPoints[i3 + 2];
        const metric = Math.max(MIN_DISTANCE_SQ, px * px + py * py + pz * pz) * _prunePenetrationDepthSq[i];
        if (metric > maxMetric) {
            maxMetric = metric;
            point1 = i;
        }
    }

    // step 3: select point 2 (furthest from point 1)
    let point2 = -1;
    maxMetric = -Infinity;
    const p1Idx = point1 * 3;
    const p1x = _pruneProjectedPoints[p1Idx];
    const p1y = _pruneProjectedPoints[p1Idx + 1];
    const p1z = _pruneProjectedPoints[p1Idx + 2];

    for (let i = 0; i < numPoints; i++) {
        if (i !== point1) {
            const i3 = i * 3;
            // projected[i] - point1Projected
            const dx = _pruneProjectedPoints[i3] - p1x;
            const dy = _pruneProjectedPoints[i3 + 1] - p1y;
            const dz = _pruneProjectedPoints[i3 + 2] - p1z;
            // squared length
            const metric = Math.max(MIN_DISTANCE_SQ, dx * dx + dy * dy + dz * dz) * _prunePenetrationDepthSq[i];
            if (metric > maxMetric) {
                maxMetric = metric;
                point2 = i;
            }
        }
    }

    assert(point2 !== -1, 'pruneContactPoints found no second point');

    // step 4: select points 3 & 4 (opposite sides of P1-P2 edge)
    let point3 = -1;
    let point4 = -1;
    let minDot = 0.0; // furthest in negative perpendicular direction
    let maxDot = 0.0; // furthest in positive perpendicular direction

    const p2Idx = point2 * 3;
    const p2x = _pruneProjectedPoints[p2Idx];
    const p2y = _pruneProjectedPoints[p2Idx + 1];
    const p2z = _pruneProjectedPoints[p2Idx + 2];

    // edgeVec = point2Projected - point1Projected
    const edgeX = p2x - p1x;
    const edgeY = p2y - p1y;
    const edgeZ = p2z - p1z;

    // calculate perpendicular direction: perp = edge × axis
    // edge × penetrationAxis
    const perpX = edgeY * axisZ - edgeZ * axisY;
    const perpY = edgeZ * axisX - edgeX * axisZ;
    const perpZ = edgeX * axisY - edgeY * axisX;

    for (let i = 0; i < numPoints; i++) {
        if (i !== point1 && i !== point2) {
            const i3 = i * 3;
            // toPoint = projected[i] - point1Projected
            const toX = _pruneProjectedPoints[i3] - p1x;
            const toY = _pruneProjectedPoints[i3 + 1] - p1y;
            const toZ = _pruneProjectedPoints[i3 + 2] - p1z;
            // dot product: toPoint · perpendicular
            const d = toX * perpX + toY * perpY + toZ * perpZ;

            if (d < minDot) {
                minDot = d;
                point3 = i;
            } else if (d > maxDot) {
                maxDot = d;
                point4 = i;
            }
        }
    }

    // step 5: gather the kept points in winding order [p1, p3, p2, p4] so they form a polygon, then copy them back
    let numKept = pruneKeepPoint(manifold, point1, 0);
    if (point3 !== -1) {
        numKept = pruneKeepPoint(manifold, point3, numKept);
    }
    numKept = pruneKeepPoint(manifold, point2, numKept);
    if (point4 !== -1) {
        assert(point3 !== point4, 'pruneContactPoints chose the same point on both sides of the p1-p2 edge');
        numKept = pruneKeepPoint(manifold, point4, numKept);
    }

    for (let i = 0; i < numKept * 3; i++) {
        manifold.relativeContactPointsOnA[i] = _pruneKeptOnA[i];
        manifold.relativeContactPointsOnB[i] = _pruneKeptOnB[i];
    }
    manifold.numContactPoints = numKept;
}

function pruneKeepPoint(manifold: ContactManifold, index: number, numKept: number): number {
    const srcI = index * 3;
    const dstI = numKept * 3;
    _pruneKeptOnA[dstI] = manifold.relativeContactPointsOnA[srcI];
    _pruneKeptOnA[dstI + 1] = manifold.relativeContactPointsOnA[srcI + 1];
    _pruneKeptOnA[dstI + 2] = manifold.relativeContactPointsOnA[srcI + 2];
    _pruneKeptOnB[dstI] = manifold.relativeContactPointsOnB[srcI];
    _pruneKeptOnB[dstI + 1] = manifold.relativeContactPointsOnB[srcI + 1];
    _pruneKeptOnB[dstI + 2] = manifold.relativeContactPointsOnB[srcI + 2];
    return numKept + 1;
}

const _manifoldBetweenTwoFaces_clippedFace = /* @__PURE__ */ createFace();
const _manifoldBetweenTwoFaces_penetrationAxis = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_edgeV1 = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_edgeV2 = /* @__PURE__ */ vec3.create();

/**
 * Generates contact manifold from two supporting faces using polygon clipping.
 *
 * Algorithm:
 * 1. Clip the face with 3+ vertices (incident) against the other face (reference), swapping the faces and
 *    negating the penetration axis when face 2 is the one with fewer than 3 vertices
 * 2. Project clipped points onto the reference face's plane
 * 3. Filter by max contact distance
 * 4. Fallback to original contact points if clipping produces nothing
 *
 * Contact points are stored RELATIVE to manifold.baseOffset for numerical precision.
 *
 * @param out output contact manifold (points/numContactPoints populated)
 * @param inContactPoint1 initial contact point on shape 1 (world space)
 * @param inContactPoint2 initial contact point on shape 2 (world space)
 * @param inPenetrationAxis penetration direction (unnormalized, from shape 1 to 2, world space)
 * @param inMaxContactDistance max distance for contact filtering, > 0
 * @param inShape1Face supporting face on shape 1 (world space)
 * @param inShape2Face supporting face on shape 2 (world space)
 */
export function manifoldBetweenTwoFaces(
    out: ContactManifold,
    inContactPoint1: Vec3,
    inContactPoint2: Vec3,
    inPenetrationAxis: Vec3,
    inMaxContactDistance: number,
    inShape1Face: Face,
    inShape2Face: Face,
): void {
    assert(inMaxContactDistance > 0, 'manifoldBetweenTwoFaces max contact distance must be positive');

    // reset output
    out.numContactPoints = 0;

    // both faces need at least 2 vertices or else there can never be more than 1 contact point.
    // at least one face needs at least 3 vertices (when both have 2, only exactly matching edges give 2 contact
    // points, and that situation is unstable anyhow)
    if (
        Math.min(inShape1Face.numVertices, inShape2Face.numVertices) >= 2 &&
        Math.max(inShape1Face.numVertices, inShape2Face.numVertices) >= 3
    ) {
        // swap the shapes if the 2nd face doesn't have enough vertices
        let shape1Face: Face;
        let shape2Face: Face;
        let contactPoints1: number[];
        let contactPoints2: number[];
        const penetrationAxis = _manifoldBetweenTwoFaces_penetrationAxis;
        if (inShape2Face.numVertices >= 3) {
            shape1Face = inShape1Face;
            shape2Face = inShape2Face;
            contactPoints1 = out.relativeContactPointsOnA;
            contactPoints2 = out.relativeContactPointsOnB;
            vec3.copy(penetrationAxis, inPenetrationAxis);
        } else {
            shape1Face = inShape2Face;
            shape2Face = inShape1Face;
            contactPoints1 = out.relativeContactPointsOnB;
            contactPoints2 = out.relativeContactPointsOnA;
            vec3.negate(penetrationAxis, inPenetrationAxis);
        }

        // clip face 2 against face 1
        const clippedFace = _manifoldBetweenTwoFaces_clippedFace;
        clippedFace.numVertices = 0;

        if (shape1Face.numVertices >= 3) {
            // polygon vs polygon
            clipPolyVsPoly(clippedFace, shape2Face, shape1Face, penetrationAxis);
        } else {
            // polygon vs edge of face 1
            const edgeV1 = vec3.set(
                _manifoldBetweenTwoFaces_edgeV1,
                shape1Face.vertices[0],
                shape1Face.vertices[1],
                shape1Face.vertices[2],
            );
            const edgeV2 = vec3.set(
                _manifoldBetweenTwoFaces_edgeV2,
                shape1Face.vertices[3],
                shape1Face.vertices[4],
                shape1Face.vertices[5],
            );
            clipPolyVsEdge(clippedFace, shape2Face, edgeV1, edgeV2, penetrationAxis);
        }

        const axisX = penetrationAxis[0];
        const axisY = penetrationAxis[1];
        const axisZ = penetrationAxis[2];

        // plane origin is the first vertex of face 1
        const planeOriginX = shape1Face.vertices[0];
        const planeOriginY = shape1Face.vertices[1];
        const planeOriginZ = shape1Face.vertices[2];

        // firstEdge = v1 - planeOrigin
        const firstEdgeX = shape1Face.vertices[3] - planeOriginX;
        const firstEdgeY = shape1Face.vertices[4] - planeOriginY;
        const firstEdgeZ = shape1Face.vertices[5] - planeOriginZ;

        // calculate plane normal
        let planeNormalX: number;
        let planeNormalY: number;
        let planeNormalZ: number;

        if (shape1Face.numVertices >= 3) {
            // three+ vertices: planeNormal = firstEdge × (v2 - planeOrigin)
            const secondEdgeX = shape1Face.vertices[6] - planeOriginX;
            const secondEdgeY = shape1Face.vertices[7] - planeOriginY;
            const secondEdgeZ = shape1Face.vertices[8] - planeOriginZ;

            planeNormalX = firstEdgeY * secondEdgeZ - firstEdgeZ * secondEdgeY;
            planeNormalY = firstEdgeZ * secondEdgeX - firstEdgeX * secondEdgeZ;
            planeNormalZ = firstEdgeX * secondEdgeY - firstEdgeY * secondEdgeX;
        } else {
            // two vertices (edge): normal = (firstEdge × penetrationAxis) × firstEdge
            // this gives a vector perpendicular to firstEdge that lies in the plane containing both the edge and penetration axis
            const perpX = firstEdgeY * axisZ - firstEdgeZ * axisY;
            const perpY = firstEdgeZ * axisX - firstEdgeX * axisZ;
            const perpZ = firstEdgeX * axisY - firstEdgeY * axisX;

            planeNormalX = perpY * firstEdgeZ - perpZ * firstEdgeY;
            planeNormalY = perpZ * firstEdgeX - perpX * firstEdgeZ;
            planeNormalZ = perpX * firstEdgeY - perpY * firstEdgeX;
        }

        // if penetration axis and plane normal are perpendicular, fall back to the contact points
        const penetrationAxisDotPlaneNormal = axisX * planeNormalX + axisY * planeNormalY + axisZ * planeNormalZ;

        if (penetrationAxisDotPlaneNormal !== 0.0) {
            // length of penetration axis (to convert distance to world units)
            const penetrationAxisLen = Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);

            for (let i = 0; i < clippedFace.numVertices; i++) {
                const vertexIndex = i * 3;
                const p2X = clippedFace.vertices[vertexIndex];
                const p2Y = clippedFace.vertices[vertexIndex + 1];
                const p2Z = clippedFace.vertices[vertexIndex + 2];

                // project p2 onto face 1 plane:
                // solve: p1 = p2 + distance * penetrationAxis / |penetrationAxis|
                //        (p1 - planeOrigin) · planeNormal = 0
                // result: distance = (p2 - planeOrigin) · planeNormal / (penetrationAxis · planeNormal)
                // (the -|penetrationAxis| term is left out)
                const distance =
                    ((p2X - planeOriginX) * planeNormalX +
                        (p2Y - planeOriginY) * planeNormalY +
                        (p2Z - planeOriginZ) * planeNormalZ) /
                    penetrationAxisDotPlaneNormal;

                // keep the point if it is less than inMaxContactDistance in front of the plane of face 1
                if (distance * penetrationAxisLen < inMaxContactDistance) {
                    // p1 = p2 - distance * penetrationAxis, both relative to baseOffset
                    const outIndex = out.numContactPoints * 3;
                    contactPoints1[outIndex] = p2X - distance * axisX - out.baseOffset[0];
                    contactPoints1[outIndex + 1] = p2Y - distance * axisY - out.baseOffset[1];
                    contactPoints1[outIndex + 2] = p2Z - distance * axisZ - out.baseOffset[2];
                    contactPoints2[outIndex] = p2X - out.baseOffset[0];
                    contactPoints2[outIndex + 1] = p2Y - out.baseOffset[1];
                    contactPoints2[outIndex + 2] = p2Z - out.baseOffset[2];

                    out.numContactPoints++;

                    if (out.numContactPoints >= MAX_CLIPPING_CONTACT_POINTS) {
                        break; // safety limit
                    }
                }
            }
        }
    }

    // if clipping produced no points, use original contact points
    if (out.numContactPoints === 0) {
        out.relativeContactPointsOnA[0] = inContactPoint1[0] - out.baseOffset[0];
        out.relativeContactPointsOnA[1] = inContactPoint1[1] - out.baseOffset[1];
        out.relativeContactPointsOnA[2] = inContactPoint1[2] - out.baseOffset[2];
        out.relativeContactPointsOnB[0] = inContactPoint2[0] - out.baseOffset[0];
        out.relativeContactPointsOnB[1] = inContactPoint2[1] - out.baseOffset[1];
        out.relativeContactPointsOnB[2] = inContactPoint2[2] - out.baseOffset[2];
        out.numContactPoints = 1;
    }
}
