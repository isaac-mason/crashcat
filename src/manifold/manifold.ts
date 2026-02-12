import { type Vec3, vec3 } from 'mathcat';
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

/**
 * Sets relative contact point (relative to baseOffset).
 * World-space points are converted to relative coordinates.
 */
export function setContactPoint(manifold: ContactManifold, index: number, worldPointOn1: Vec3, worldPointOn2: Vec3): void {
    const i = index * 3;

    // convert world space to relative coordinates
    manifold.relativeContactPointsOnA[i] = worldPointOn1[0] - manifold.baseOffset[0];
    manifold.relativeContactPointsOnA[i + 1] = worldPointOn1[1] - manifold.baseOffset[1];
    manifold.relativeContactPointsOnA[i + 2] = worldPointOn1[2] - manifold.baseOffset[2];

    manifold.relativeContactPointsOnB[i] = worldPointOn2[0] - manifold.baseOffset[0];
    manifold.relativeContactPointsOnB[i + 1] = worldPointOn2[1] - manifold.baseOffset[1];
    manifold.relativeContactPointsOnB[i + 2] = worldPointOn2[2] - manifold.baseOffset[2];
}

const _pruneContactPoints_projected: Vec3[] = [];
const _pruneContactPoints_penetrationDepthSq: number[] = [];
const _pruneContactPoints_v1 = /* @__PURE__ */ vec3.create();
const _pruneContactPoints_v2 = /* @__PURE__ */ vec3.create();
const _pruneContactPoints_projectedV1 = /* @__PURE__ */ vec3.create();
const _pruneContactPoints_edgeVec = /* @__PURE__ */ vec3.create();
const _pruneContactPoints_perpendicular = /* @__PURE__ */ vec3.create();
const _pruneContactPoints_toPoint = /* @__PURE__ */ vec3.create();
const _pruneContactPoints_depthVec = /* @__PURE__ */ vec3.create();

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
 * @param worldOptions world configuration (for debug flags and renderer)
 * @param manifold contactManifold with contact points to prune (modifies in-place)
 * @param penetrationAxis normalized penetration direction (from shape 1 to shape 2)
 */
export function pruneContactPoints(manifold: ContactManifold, penetrationAxis: Vec3): void {
    assert(manifold.numContactPoints > MAX_CONTACT_POINTS, 'pruneContactPoints should only be called with > 4 contact points');

    // early exit if already <= 4 points (safety fallback)
    if (manifold.numContactPoints <= MAX_CONTACT_POINTS) {
        return;
    }

    // clear temporary arrays
    _pruneContactPoints_projected.length = 0;
    _pruneContactPoints_penetrationDepthSq.length = 0;

    // step 1: project all contact points onto plane perpendicular to penetrationAxis
    for (let i = 0; i < manifold.numContactPoints; i++) {
        // get contact points on both shapes (relative to baseOffset)
        getRelativeContactPointOnA(_pruneContactPoints_v1, manifold, i);
        getRelativeContactPointOnB(_pruneContactPoints_v2, manifold, i);

        // project v1 onto plane with normal = penetrationAxis
        // projected = v1 - (v1 · axis) * axis
        const dotProduct = vec3.dot(_pruneContactPoints_v1, penetrationAxis);
        vec3.scaleAndAdd(_pruneContactPoints_projectedV1, _pruneContactPoints_v1, penetrationAxis, -dotProduct);
        // TODO: avoid vec3.clone
        _pruneContactPoints_projected.push(vec3.clone(_pruneContactPoints_projectedV1));

        // penetration depth = distance between points on shapes
        vec3.sub(_pruneContactPoints_depthVec, _pruneContactPoints_v2, _pruneContactPoints_v1);
        const depthSq = Math.max(MIN_DISTANCE_SQ, vec3.squaredLength(_pruneContactPoints_depthVec));
        _pruneContactPoints_penetrationDepthSq.push(depthSq);
    }

    // step 2: select point 1 (maximum metric in plane)
    let point1 = 0;
    let maxMetric =
        Math.max(MIN_DISTANCE_SQ, vec3.squaredLength(_pruneContactPoints_projected[0])) *
        _pruneContactPoints_penetrationDepthSq[0];

    for (let i = 1; i < manifold.numContactPoints; i++) {
        const metric =
            Math.max(MIN_DISTANCE_SQ, vec3.squaredLength(_pruneContactPoints_projected[i])) *
            _pruneContactPoints_penetrationDepthSq[i];
        if (metric > maxMetric) {
            maxMetric = metric;
            point1 = i;
        }
    }

    // step 3: select point 2 (furthest from point 1)
    let point2 = -1;
    maxMetric = -Infinity;
    const point1Projected = _pruneContactPoints_projected[point1];

    for (let i = 0; i < manifold.numContactPoints; i++) {
        if (i !== point1) {
            vec3.sub(_pruneContactPoints_toPoint, _pruneContactPoints_projected[i], point1Projected);
            const metric =
                Math.max(MIN_DISTANCE_SQ, vec3.squaredLength(_pruneContactPoints_toPoint)) *
                _pruneContactPoints_penetrationDepthSq[i];
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
    const point2Projected = _pruneContactPoints_projected[point2];
    vec3.sub(_pruneContactPoints_edgeVec, point2Projected, point1Projected);

    // calculate perpendicular direction: perp = (edge × axis) normalized direction in plane
    // actually we need perpendicular within the plane perpendicular to axis
    // perp = edge × axis (gives a vector perpendicular to both edge and axis)
    vec3.cross(_pruneContactPoints_perpendicular, _pruneContactPoints_edgeVec, penetrationAxis);

    for (let i = 0; i < manifold.numContactPoints; i++) {
        if (i !== point1 && i !== point2) {
            vec3.sub(_pruneContactPoints_toPoint, _pruneContactPoints_projected[i], point1Projected);
            const d = vec3.dot(_pruneContactPoints_toPoint, _pruneContactPoints_perpendicular);

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
    const finalOrder = [point1, point3, point2, point4];
    let outIndex = 0;

    for (const srcIndex of finalOrder) {
        if (srcIndex !== -1) {
            if (outIndex !== srcIndex) {
                // copy both On1 and On2 points in-place
                const srcI = srcIndex * 3;
                const dstI = outIndex * 3;

                // copy points on shape 1
                manifold.relativeContactPointsOnA[dstI] = manifold.relativeContactPointsOnA[srcI];
                manifold.relativeContactPointsOnA[dstI + 1] = manifold.relativeContactPointsOnA[srcI + 1];
                manifold.relativeContactPointsOnA[dstI + 2] = manifold.relativeContactPointsOnA[srcI + 2];

                // copy points on shape 2
                manifold.relativeContactPointsOnB[dstI] = manifold.relativeContactPointsOnB[srcI];
                manifold.relativeContactPointsOnB[dstI + 1] = manifold.relativeContactPointsOnB[srcI + 1];
                manifold.relativeContactPointsOnB[dstI + 2] = manifold.relativeContactPointsOnB[srcI + 2];
            }
            outIndex++;
        }
    }

    manifold.numContactPoints = outIndex;
}

const _manifoldBetweenTwoFaces_clippedFace = /* @__PURE__ */ createFace();
const _manifoldBetweenTwoFaces_planeOrigin = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_edgeV1 = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_edgeV2 = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_firstEdge = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_secondEdge = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_planeNormal = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_perp = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_p1 = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_p2 = /* @__PURE__ */ vec3.create();
const _manifoldBetweenTwoFaces_diff = /* @__PURE__ */ vec3.create();

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
 * @param worldOptions world configuration (for debug flags and renderer)
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
        setContactPoint(out, 0, inContactPoint1, inContactPoint2);
        out.numContactPoints = 1;
        return;
    }

    // clip face 2 against face 1
    _manifoldBetweenTwoFaces_clippedFace.numVertices = 0;

    if (inShape1Face.numVertices >= 3) {
        // polygon vs polygon
        clipPolyVsPoly(_manifoldBetweenTwoFaces_clippedFace, inShape2Face, inShape1Face, inPenetrationAxis);
    } else if (inShape1Face.numVertices === 2) {
        // edge vs polygon
        vec3.fromBuffer(_manifoldBetweenTwoFaces_edgeV1, inShape1Face.vertices, 0 * 3);
        vec3.fromBuffer(_manifoldBetweenTwoFaces_edgeV2, inShape1Face.vertices, 1 * 3);
        clipPolyVsEdge(
            _manifoldBetweenTwoFaces_clippedFace,
            inShape2Face,
            _manifoldBetweenTwoFaces_edgeV1,
            _manifoldBetweenTwoFaces_edgeV2,
            inPenetrationAxis,
        );
    }

    // determine plane origin and normal for shape 1
    vec3.fromBuffer(_manifoldBetweenTwoFaces_planeOrigin, inShape1Face.vertices, 0 * 3);

    // calculate plane normal
    vec3.fromBuffer(_manifoldBetweenTwoFaces_edgeV1, inShape1Face.vertices, 1 * 3);
    vec3.sub(_manifoldBetweenTwoFaces_firstEdge, _manifoldBetweenTwoFaces_edgeV1, _manifoldBetweenTwoFaces_planeOrigin);

    if (inShape1Face.numVertices >= 3) {
        // three+ vertices: calculate normal from cross product
        // normal = (v2 - v0) × (v1 - v0)
        vec3.fromBuffer(_manifoldBetweenTwoFaces_edgeV2, inShape1Face.vertices, 2 * 3);
        vec3.sub(_manifoldBetweenTwoFaces_secondEdge, _manifoldBetweenTwoFaces_edgeV2, _manifoldBetweenTwoFaces_planeOrigin);
        vec3.cross(_manifoldBetweenTwoFaces_planeNormal, _manifoldBetweenTwoFaces_firstEdge, _manifoldBetweenTwoFaces_secondEdge);
    } else {
        // two vertices (edge):
        // normal = (firstEdge × penetrationAxis) × firstEdge
        vec3.cross(_manifoldBetweenTwoFaces_perp, _manifoldBetweenTwoFaces_firstEdge, inPenetrationAxis);
        vec3.cross(_manifoldBetweenTwoFaces_planeNormal, _manifoldBetweenTwoFaces_perp, _manifoldBetweenTwoFaces_firstEdge);
    }

    // check if penetration axis and plane normal are perpendicular
    const penetrationAxisDotPlaneNormal = vec3.dot(inPenetrationAxis, _manifoldBetweenTwoFaces_planeNormal);

    if (penetrationAxisDotPlaneNormal !== 0.0) {
        const penetrationAxisLen = vec3.length(inPenetrationAxis);

        // project clipped points onto face 1 plane and filter by distance
        for (let i = 0; i < _manifoldBetweenTwoFaces_clippedFace.numVertices; i++) {
            vec3.fromBuffer(_manifoldBetweenTwoFaces_p2, _manifoldBetweenTwoFaces_clippedFace.vertices, i * 3);

            // project p2 onto face 1 plane:
            // solve: p1 = p2 + distance * penetrationAxis / |penetrationAxis|
            //        (p1 - planeOrigin) · planeNormal = 0
            // result: distance = (p2 - planeOrigin) · planeNormal / (penetrationAxis · planeNormal)
            vec3.sub(_manifoldBetweenTwoFaces_diff, _manifoldBetweenTwoFaces_p2, _manifoldBetweenTwoFaces_planeOrigin);
            const distance =
                vec3.dot(_manifoldBetweenTwoFaces_diff, _manifoldBetweenTwoFaces_planeNormal) / penetrationAxisDotPlaneNormal;

            // filter by max contact distance (distance is scaled by penetrationAxisLen to convert to world units)
            if (distance * penetrationAxisLen < inMaxContactDistance) {
                // p1 = p2 - distance * penetrationAxis
                vec3.scaleAndAdd(_manifoldBetweenTwoFaces_p1, _manifoldBetweenTwoFaces_p2, inPenetrationAxis, -distance);

                setContactPoint(out, out.numContactPoints, _manifoldBetweenTwoFaces_p1, _manifoldBetweenTwoFaces_p2);
                out.numContactPoints++;

                if (out.numContactPoints >= MAX_CLIPPING_CONTACT_POINTS) {
                    break; // safety limit
                }
            }
        }
    }

    // if clipping produced no points, use original contact points
    if (out.numContactPoints === 0) {
        setContactPoint(out, 0, inContactPoint1, inContactPoint2);
        out.numContactPoints = 1;
    }
}
