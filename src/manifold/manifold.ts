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
 * In-place reduction: selected points are moved to indices 0-3,
 * manifold.numContactPoints is updated to final count (1-4).
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

    // step 5: copy selected points back (in winding order: [P1, P3, P2, P4])
    // use finalOrder as fixed indices (no array allocation)
    let outIndex = 0;

    // point1
    if (point1 !== -1) {
        if (outIndex !== point1) {
            const srcI = point1 * 3;
            const dstI = outIndex * 3;
            manifold.relativeContactPointsOnA[dstI] = manifold.relativeContactPointsOnA[srcI];
            manifold.relativeContactPointsOnA[dstI + 1] = manifold.relativeContactPointsOnA[srcI + 1];
            manifold.relativeContactPointsOnA[dstI + 2] = manifold.relativeContactPointsOnA[srcI + 2];
            manifold.relativeContactPointsOnB[dstI] = manifold.relativeContactPointsOnB[srcI];
            manifold.relativeContactPointsOnB[dstI + 1] = manifold.relativeContactPointsOnB[srcI + 1];
            manifold.relativeContactPointsOnB[dstI + 2] = manifold.relativeContactPointsOnB[srcI + 2];
        }
        outIndex++;
    }

    // point3
    if (point3 !== -1) {
        if (outIndex !== point3) {
            const srcI = point3 * 3;
            const dstI = outIndex * 3;
            manifold.relativeContactPointsOnA[dstI] = manifold.relativeContactPointsOnA[srcI];
            manifold.relativeContactPointsOnA[dstI + 1] = manifold.relativeContactPointsOnA[srcI + 1];
            manifold.relativeContactPointsOnA[dstI + 2] = manifold.relativeContactPointsOnA[srcI + 2];
            manifold.relativeContactPointsOnB[dstI] = manifold.relativeContactPointsOnB[srcI];
            manifold.relativeContactPointsOnB[dstI + 1] = manifold.relativeContactPointsOnB[srcI + 1];
            manifold.relativeContactPointsOnB[dstI + 2] = manifold.relativeContactPointsOnB[srcI + 2];
        }
        outIndex++;
    }

    // point2
    if (point2 !== -1) {
        if (outIndex !== point2) {
            const srcI = point2 * 3;
            const dstI = outIndex * 3;
            manifold.relativeContactPointsOnA[dstI] = manifold.relativeContactPointsOnA[srcI];
            manifold.relativeContactPointsOnA[dstI + 1] = manifold.relativeContactPointsOnA[srcI + 1];
            manifold.relativeContactPointsOnA[dstI + 2] = manifold.relativeContactPointsOnA[srcI + 2];
            manifold.relativeContactPointsOnB[dstI] = manifold.relativeContactPointsOnB[srcI];
            manifold.relativeContactPointsOnB[dstI + 1] = manifold.relativeContactPointsOnB[srcI + 1];
            manifold.relativeContactPointsOnB[dstI + 2] = manifold.relativeContactPointsOnB[srcI + 2];
        }
        outIndex++;
    }

    // point4
    if (point4 !== -1) {
        if (outIndex !== point4) {
            const srcI = point4 * 3;
            const dstI = outIndex * 3;
            manifold.relativeContactPointsOnA[dstI] = manifold.relativeContactPointsOnA[srcI];
            manifold.relativeContactPointsOnA[dstI + 1] = manifold.relativeContactPointsOnA[srcI + 1];
            manifold.relativeContactPointsOnA[dstI + 2] = manifold.relativeContactPointsOnA[srcI + 2];
            manifold.relativeContactPointsOnB[dstI] = manifold.relativeContactPointsOnB[srcI];
            manifold.relativeContactPointsOnB[dstI + 1] = manifold.relativeContactPointsOnB[srcI + 1];
            manifold.relativeContactPointsOnB[dstI + 2] = manifold.relativeContactPointsOnB[srcI + 2];
        }
        outIndex++;
    }

    manifold.numContactPoints = outIndex;
}

const _manifoldBetweenTwoFaces_clippedFace = /* @__PURE__ */ createFace();

/**
 * Generates contact manifold from two supporting faces using polygon clipping.
 *
 * Algorithm:
 * 1. Clip face 2 (incident) against face 1 (reference)
 * 2. Project clipped points onto face 1's plane
 * 3. Filter by max contact distance
 * 4. Fallback to original contact points if clipping produces nothing
 *
 * Contact points are stored RELATIVE to manifold.baseOffset for numerical precision.
 *
 * @param out output contact manifold (points/numContactPoints populated)
 * @param inContactPoint1 initial contact point on shape 1 (world space)
 * @param inContactPoint2 initial contact point on shape 2 (world space)
 * @param inPenetrationAxis penetration direction (unnormalized, from shape 1 to 2, world space)
 * @param inMaxContactDistance max distance for contact filtering
 * @param inShape1Face supporting face on shape 1 (reference face, world space)
 * @param inShape2Face supporting face on shape 2 (incident face, world space)
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
    // reset output
    out.numContactPoints = 0;

    // check if both shapes have polygon faces
    // face 1 needs >= 2 vertices (edge or polygon)
    // face 2 needs >= 3 vertices (polygon)
    if (inShape1Face.numVertices < 2 || inShape2Face.numVertices < 3) {
        // not enough vertices for clipping, use original contact points
        out.relativeContactPointsOnA[0] = inContactPoint1[0] - out.baseOffset[0];
        out.relativeContactPointsOnA[1] = inContactPoint1[1] - out.baseOffset[1];
        out.relativeContactPointsOnA[2] = inContactPoint1[2] - out.baseOffset[2];
        out.relativeContactPointsOnB[0] = inContactPoint2[0] - out.baseOffset[0];
        out.relativeContactPointsOnB[1] = inContactPoint2[1] - out.baseOffset[1];
        out.relativeContactPointsOnB[2] = inContactPoint2[2] - out.baseOffset[2];
        out.numContactPoints = 1;
        return;
    }

    // clip face 2 against face 1
    _manifoldBetweenTwoFaces_clippedFace.numVertices = 0;

    if (inShape1Face.numVertices >= 3) {
        // polygon vs polygon
        clipPolyVsPoly(_manifoldBetweenTwoFaces_clippedFace, inShape2Face, inShape1Face, inPenetrationAxis);
    } else if (inShape1Face.numVertices === 2) {
        // edge vs polygon - extract edge endpoints from shape 1 face
        const edgeV1X = inShape1Face.vertices[0];
        const edgeV1Y = inShape1Face.vertices[1];
        const edgeV1Z = inShape1Face.vertices[2];
        const edgeV2X = inShape1Face.vertices[3];
        const edgeV2Y = inShape1Face.vertices[4];
        const edgeV2Z = inShape1Face.vertices[5];

        const edgeV1 = vec3.set(_tmpEdgeV1, edgeV1X, edgeV1Y, edgeV1Z);
        const edgeV2 = vec3.set(_tmpEdgeV2, edgeV2X, edgeV2Y, edgeV2Z);

        clipPolyVsEdge(_manifoldBetweenTwoFaces_clippedFace, inShape2Face, edgeV1, edgeV2, inPenetrationAxis);
    }

    // extract plane origin from first vertex of shape 1 face
    const planeOriginX = inShape1Face.vertices[0];
    const planeOriginY = inShape1Face.vertices[1];
    const planeOriginZ = inShape1Face.vertices[2];

    // extract second vertex and compute first edge vector
    const v1X = inShape1Face.vertices[3];
    const v1Y = inShape1Face.vertices[4];
    const v1Z = inShape1Face.vertices[5];

    // firstEdge = v1 - planeOrigin
    const firstEdgeX = v1X - planeOriginX;
    const firstEdgeY = v1Y - planeOriginY;
    const firstEdgeZ = v1Z - planeOriginZ;

    // calculate plane normal
    let planeNormalX: number;
    let planeNormalY: number;
    let planeNormalZ: number;

    if (inShape1Face.numVertices >= 3) {
        // three+ vertices: calculate normal from cross product of two edge vectors
        // extract third vertex
        const v2X = inShape1Face.vertices[6];
        const v2Y = inShape1Face.vertices[7];
        const v2Z = inShape1Face.vertices[8];

        // secondEdge = v2 - planeOrigin
        const secondEdgeX = v2X - planeOriginX;
        const secondEdgeY = v2Y - planeOriginY;
        const secondEdgeZ = v2Z - planeOriginZ;

        // planeNormal = firstEdge × secondEdge
        // cross product: (a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
        planeNormalX = firstEdgeY * secondEdgeZ - firstEdgeZ * secondEdgeY;
        planeNormalY = firstEdgeZ * secondEdgeX - firstEdgeX * secondEdgeZ;
        planeNormalZ = firstEdgeX * secondEdgeY - firstEdgeY * secondEdgeX;
    } else {
        // two vertices (edge): normal = (firstEdge × penetrationAxis) × firstEdge
        // this gives a vector perpendicular to firstEdge that lies in the plane containing both the edge and penetration axis

        // perp = firstEdge × penetrationAxis
        const perpX = firstEdgeY * inPenetrationAxis[2] - firstEdgeZ * inPenetrationAxis[1];
        const perpY = firstEdgeZ * inPenetrationAxis[0] - firstEdgeX * inPenetrationAxis[2];
        const perpZ = firstEdgeX * inPenetrationAxis[1] - firstEdgeY * inPenetrationAxis[0];

        // planeNormal = perp × firstEdge
        planeNormalX = perpY * firstEdgeZ - perpZ * firstEdgeY;
        planeNormalY = perpZ * firstEdgeX - perpX * firstEdgeZ;
        planeNormalZ = perpX * firstEdgeY - perpY * firstEdgeX;
    }

    // check if penetration axis and plane normal are perpendicular
    // dot product: a · b = a.x * b.x + a.y * b.y + a.z * b.z
    const penetrationAxisDotPlaneNormal =
        inPenetrationAxis[0] * planeNormalX + inPenetrationAxis[1] * planeNormalY + inPenetrationAxis[2] * planeNormalZ;

    if (penetrationAxisDotPlaneNormal !== 0.0) {
        // length of penetration axis (to convert distance to world units)
        const penetrationAxisLen = Math.sqrt(
            inPenetrationAxis[0] * inPenetrationAxis[0] +
                inPenetrationAxis[1] * inPenetrationAxis[1] +
                inPenetrationAxis[2] * inPenetrationAxis[2],
        );

        // project clipped points onto face 1 plane and filter by distance
        for (let i = 0; i < _manifoldBetweenTwoFaces_clippedFace.numVertices; i++) {
            // extract p2 from clipped face vertices
            const vertexIndex = i * 3;
            const p2X = _manifoldBetweenTwoFaces_clippedFace.vertices[vertexIndex];
            const p2Y = _manifoldBetweenTwoFaces_clippedFace.vertices[vertexIndex + 1];
            const p2Z = _manifoldBetweenTwoFaces_clippedFace.vertices[vertexIndex + 2];

            // project p2 onto face 1 plane:
            // solve: p1 = p2 + distance * penetrationAxis / |penetrationAxis|
            //        (p1 - planeOrigin) · planeNormal = 0
            // result: distance = (p2 - planeOrigin) · planeNormal / (penetrationAxis · planeNormal)

            // diff = p2 - planeOrigin
            const diffX = p2X - planeOriginX;
            const diffY = p2Y - planeOriginY;
            const diffZ = p2Z - planeOriginZ;

            // dot(diff, planeNormal) / dot(penetrationAxis, planeNormal)
            const distance = (diffX * planeNormalX + diffY * planeNormalY + diffZ * planeNormalZ) / penetrationAxisDotPlaneNormal;

            // filter by max contact distance (distance is scaled by penetrationAxisLen to convert to world units)
            if (distance * penetrationAxisLen < inMaxContactDistance) {
                // p1 = p2 - distance * penetrationAxis
                const p1X = p2X - distance * inPenetrationAxis[0];
                const p1Y = p2Y - distance * inPenetrationAxis[1];
                const p1Z = p2Z - distance * inPenetrationAxis[2];

                // set contact point
                // convert world space to relative coordinates
                const outIndex = out.numContactPoints * 3;
                out.relativeContactPointsOnA[outIndex] = p1X - out.baseOffset[0];
                out.relativeContactPointsOnA[outIndex + 1] = p1Y - out.baseOffset[1];
                out.relativeContactPointsOnA[outIndex + 2] = p1Z - out.baseOffset[2];
                out.relativeContactPointsOnB[outIndex] = p2X - out.baseOffset[0];
                out.relativeContactPointsOnB[outIndex + 1] = p2Y - out.baseOffset[1];
                out.relativeContactPointsOnB[outIndex + 2] = p2Z - out.baseOffset[2];

                out.numContactPoints++;

                if (out.numContactPoints >= MAX_CLIPPING_CONTACT_POINTS) {
                    break; // safety limit
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

const _tmpEdgeV1 = /* @__PURE__ */ vec3.create();
const _tmpEdgeV2 = /* @__PURE__ */ vec3.create();
