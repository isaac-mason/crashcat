import { degreesToRadians, type Quat, quat, type Vec3, vec3 } from 'mathcat';
import { assert } from '../utils/assert';
import * as hull from './epa-convex-hull-builder';
import { createGjkClosestPoints, type GjkCastShapeResult, gjkCastShape, gjkClosestPoints } from './gjk';
import { copySimplex, type Simplex } from './simplex';
import type { Support } from './support';
import {
    createAddConvexRadiusSupport,
    createTransformedSupport,
    setAddConvexRadiusSupport,
    setTransformedSupport,
} from './support';

export enum PenetrationDepthStatus {
    NOT_COLLIDING,
    COLLIDING,
    INDETERMINATE,
}

export type PenetrationDepth = {
    status: PenetrationDepthStatus;
    penetrationAxis: Vec3;
    pointA: Vec3;
    pointB: Vec3;
};

export const createPenetrationDepth = (): PenetrationDepth => ({
    status: PenetrationDepthStatus.NOT_COLLIDING,
    penetrationAxis: vec3.create(),
    pointA: vec3.create(),
    pointB: vec3.create(),
});

const _gjk_closestPoints = /* @__PURE__ */ createGjkClosestPoints();

export function penetrationDepthStepGJK(
    outPenetrationDepth: PenetrationDepth,
    outSimplex: Simplex,
    supportA: Support,
    supportB: Support,
    convexRadiusA: number,
    convexRadiusB: number,
    direction: Vec3,
    tolerance: number,
): void {
    assert(vec3.squaredLength(direction) >= tolerance * tolerance, 'direction must not be zero in penetrationDepthStepGJK');

    const combinedRadius = convexRadiusA + convexRadiusB;
    const combinedRadiusSquared = combinedRadius * combinedRadius;

    // run GJK to find the closest points between shapes
    gjkClosestPoints(_gjk_closestPoints, supportA, supportB, tolerance, direction, combinedRadiusSquared);

    // copy results to output
    vec3.copy(outPenetrationDepth.pointA, _gjk_closestPoints.pointA);
    vec3.copy(outPenetrationDepth.pointB, _gjk_closestPoints.pointB);
    vec3.copy(outPenetrationDepth.penetrationAxis, _gjk_closestPoints.penetrationAxis);
    copySimplex(outSimplex, _gjk_closestPoints.simplex);

    // check if shapes are separated by more than the combined convex radius
    // first check GJK result, then verify with distance
    if (_gjk_closestPoints.squaredDistance > combinedRadiusSquared) {
        // shapes are separated - no collision
        // explicitly clear contact data to avoid stale data from previous collisions
        vec3.set(outPenetrationDepth.pointA, 0, 0, 0);
        vec3.set(outPenetrationDepth.pointB, 0, 0, 0);
        vec3.set(outPenetrationDepth.penetrationAxis, 0, 0, 0);
        outPenetrationDepth.status = PenetrationDepthStatus.NOT_COLLIDING;
        return;
    }

    if (_gjk_closestPoints.squaredDistance > 0.0) {
        // collision within convex radius - adjust contact points based on convex radii
        const vLength = Math.sqrt(_gjk_closestPoints.squaredDistance);

        // move pointA along penetration axis by convexRadiusA
        vec3.scaleAndAdd(
            outPenetrationDepth.pointA,
            outPenetrationDepth.pointA,
            outPenetrationDepth.penetrationAxis,
            convexRadiusA / vLength,
        );

        // move pointB along negative penetration axis by convexRadiusB
        vec3.scaleAndAdd(
            outPenetrationDepth.pointB,
            outPenetrationDepth.pointB,
            outPenetrationDepth.penetrationAxis,
            -(convexRadiusB / vLength),
        );

        outPenetrationDepth.status = PenetrationDepthStatus.COLLIDING;
        return;
    }

    // distance is zero or very small - need EPA to determine penetration depth
    outPenetrationDepth.status = PenetrationDepthStatus.INDETERMINATE;
}

const EPA_MAX_POINTS_TO_INCLUDE_ORIGIN_IN_HULL = 32;
const EPA_MAX_POINTS = 128;

const _epa_d1 = /* @__PURE__ */ vec3.fromValues(0, 1, 0);
const _epa_d2 = /* @__PURE__ */ vec3.fromValues(-1, -1, -1);
const _epa_d3 = /* @__PURE__ */ vec3.fromValues(1, -1, -1);
const _epa_d4 = /* @__PURE__ */ vec3.fromValues(0, -1, 1);

const _epa_axis = /* @__PURE__ */ vec3.create();
const _epa_dir1 = /* @__PURE__ */ vec3.create();
const _epa_dir2 = /* @__PURE__ */ vec3.create();
const _epa_dir3 = /* @__PURE__ */ vec3.create();
const _epa_quat = /* @__PURE__ */ quat.create();
const _epa_p = /* @__PURE__ */ vec3.create();
const _epa_q = /* @__PURE__ */ vec3.create();
const _epa_y = /* @__PURE__ */ vec3.create();
const _epa_negatedDirection = /* @__PURE__ */ vec3.create();
const _epa_negatedNormal = /* @__PURE__ */ vec3.create();
const _epa_p2 = /* @__PURE__ */ vec3.create();
const _epa_q2 = /* @__PURE__ */ vec3.create();
const _epa_w2 = /* @__PURE__ */ vec3.create();
const _epa_p01 = /* @__PURE__ */ vec3.create();
const _epa_p02 = /* @__PURE__ */ vec3.create();
const _epa_q01 = /* @__PURE__ */ vec3.create();
const _epa_q02 = /* @__PURE__ */ vec3.create();
const _epa_p10 = /* @__PURE__ */ vec3.create();
const _epa_p12 = /* @__PURE__ */ vec3.create();
const _epa_q10 = /* @__PURE__ */ vec3.create();
const _epa_q12 = /* @__PURE__ */ vec3.create();

const _epa_penetrationNormal = /* @__PURE__ */ vec3.create();
const _epa_contactPointA = /* @__PURE__ */ vec3.create();
const _epa_contactPointB = /* @__PURE__ */ vec3.create();

type EpaSupportPoints = {
    /** minkowski difference points */
    y: hull.Points;
    /** shape A support points */
    p: hull.Points;
    /** shape B support points */
    q: hull.Points;
};

const createEpaSupportPoints = (capacity: number): EpaSupportPoints => ({
    y: hull.createPoints(capacity),
    p: hull.createPoints(capacity),
    q: hull.createPoints(capacity),
});

const clearEpaSupportPoints = (points: EpaSupportPoints) => {
    points.y.size = 0;
    points.p.size = 0;
    points.q.size = 0;
};

/** add a support point in the given direction */
const addEpaSupportPoint = (points: EpaSupportPoints, supportA: Support, supportB: Support, direction: Vec3): number => {
    vec3.negate(_epa_negatedDirection, direction);

    supportA.getSupport(direction, _epa_p);
    supportB.getSupport(_epa_negatedDirection, _epa_q);
    vec3.subtract(_epa_y, _epa_p, _epa_q);

    // store new point
    const idx = points.y.size;
    vec3.copy(points.y.values[idx], _epa_y);
    vec3.copy(points.p.values[idx], _epa_p);
    vec3.copy(points.q.values[idx], _epa_q);
    points.y.size++;
    points.p.size++;
    points.q.size++;

    return idx;
};

const popBackEpaSupportPoint = (points: EpaSupportPoints) => {
    if (points.y.size > 0) points.y.size--;
    if (points.p.size > 0) points.p.size--;
    if (points.q.size > 0) points.q.size--;
};

const computeNormalizedPerpendicular = (out: Vec3, v: Vec3) => {
    // chooses between two perpendicular constructions based on which component is larger
    const absX = Math.abs(v[0]);
    const absY = Math.abs(v[1]);

    if (absX > absY) {
        // use X and Z components
        const len = Math.sqrt(v[0] * v[0] + v[2] * v[2]);
        out[0] = v[2] / len;
        out[1] = 0.0;
        out[2] = -v[0] / len;
    } else {
        // use Y and Z components
        const len = Math.sqrt(v[1] * v[1] + v[2] * v[2]);
        out[0] = 0.0;
        out[1] = v[2] / len;
        out[2] = -v[1] / len;
    }
};

const rotateByAxisAngle = (out: Vec3, v: Vec3, axis: Vec3, angleRadians: number) => {
    quat.setAxisAngle(_epa_quat, axis, angleRadians);
    vec3.transformQuat(out, v, _epa_quat);
};

const _epa_supportPoints = /* @__PURE__ */ createEpaSupportPoints(256);
const _epa_hullState = /* @__PURE__ */ hull.init();
const _epa_newTriangles: hull.NewTriangles = [];

// set hull positions once - the array reference stays the same, we just clear/refill it
_epa_hullState.positions = _epa_supportPoints.y.values;

const RADIANS_120_DEG = /* @__PURE__ */ degreesToRadians(120.0);

/**
 * EPA penetration depth step.
 * This function expects shapes that INCLUDE their convex radius.
 * The caller should wrap base shapes with AddConvexRadiusSupport before calling,
 * typically in the EPA fallback path. This matches Jolt's EPAPenetrationDepth.h pattern.
 */
export function penetrationDepthStepEPA(
    out: PenetrationDepth,
    supportAIncludingRadius: Support,
    supportBIncludingRadius: Support,
    tolerance: number,
    simplex: Simplex,
): boolean {
    // fetch the simplex from GJK and convert to support points
    const supportPoints = _epa_supportPoints;
    clearEpaSupportPoints(supportPoints);

    // copy simplex points to support points
    for (let i = 0; i < simplex.size; i++) {
        vec3.copy(supportPoints.y.values[i], simplex.points[i].y);
        vec3.copy(supportPoints.p.values[i], simplex.points[i].p);
        vec3.copy(supportPoints.q.values[i], simplex.points[i].q);
    }
    supportPoints.y.size = simplex.size;
    supportPoints.p.size = simplex.size;
    supportPoints.q.size = simplex.size;

    // fill up to 4 support points if needed
    switch (supportPoints.y.size) {
        case 1: {
            // 1 vertex at origin - add 4 new points
            popBackEpaSupportPoint(supportPoints);
            addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, _epa_d1);
            addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, _epa_d2);
            addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, _epa_d3);
            addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, _epa_d4);
            break;
        }

        case 2: {
            // two vertices - create 3 extra by rotating perpendicular axis
            vec3.subtract(_epa_axis, supportPoints.y.values[1], supportPoints.y.values[0]);
            vec3.normalize(_epa_axis, _epa_axis);

            computeNormalizedPerpendicular(_epa_dir1, _epa_axis);
            rotateByAxisAngle(_epa_dir2, _epa_dir1, _epa_axis, RADIANS_120_DEG);
            rotateByAxisAngle(_epa_dir3, _epa_dir2, _epa_axis, RADIANS_120_DEG);

            addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, _epa_dir1);
            addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, _epa_dir2);
            addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, _epa_dir3);
            break;
        }

        case 3:
        case 4: {
            // already have enough points
            break;
        }
    }

    // create hull from initial points
    const hullState = _epa_hullState;
    hull.initialize(hullState, 0, 1, 2);

    // add remaining points to hull
    for (let i = 3; i < supportPoints.y.size; i++) {
        const distSq = { value: -1 };
        const t = hull.findFacingTriangle(hullState, supportPoints.y.values[i], distSq);
        if (t !== null) {
            const newTriangles = _epa_newTriangles;
            newTriangles.length = 0;

            if (!hull.addPoint(hullState, t, i, Number.MAX_VALUE, newTriangles)) {
                out.status = PenetrationDepthStatus.NOT_COLLIDING;
                return false;
            }
        }
    }

    // loop until origin is inside hull
    while (true) {
        const triangle = hull.peekClosestTriangleInQueue(hullState);

        if (!triangle) {
            out.status = PenetrationDepthStatus.NOT_COLLIDING;
            return false;
        }

        // skip removed triangles
        if (triangle.removed) {
            hull.popClosestTriangleFromQueue(hullState);

            if (!hull.hasNextTriangle(hullState)) {
                out.status = PenetrationDepthStatus.NOT_COLLIDING;
                return false;
            }

            hull.freeTriangle(hullState, triangle);
            continue;
        }

        // if closest distance is >= 0, origin is in hull
        if (triangle.closestLengthSq >= 0.0) {
            break;
        }

        hull.popClosestTriangleFromQueue(hullState);

        // add support point to get origin inside hull
        const newIndex = addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, triangle.normal);
        const w = supportPoints.y.values[newIndex];

        _epa_newTriangles.length = 0;

        if (
            !hull.triangleIsFacing(triangle, w) ||
            !hull.addPoint(hullState, triangle, newIndex, Number.MAX_VALUE, _epa_newTriangles)
        ) {
            out.status = PenetrationDepthStatus.NOT_COLLIDING;
            return false;
        }

        hull.freeTriangle(hullState, triangle);

        if (!hull.hasNextTriangle(hullState) || supportPoints.y.size >= EPA_MAX_POINTS_TO_INCLUDE_ORIGIN_IN_HULL) {
            out.status = PenetrationDepthStatus.NOT_COLLIDING;
            return false;
        }
    }

    // main EPA loop - find closest point
    let closestDistSq = Number.MAX_VALUE;
    let last: hull.Triangle | null = null;
    let flipVSign = false;

    // let iter = 0;

    do {
        // iter++;
        const triangle = hull.popClosestTriangleFromQueue(hullState);

        if (!triangle) {
            out.status = PenetrationDepthStatus.NOT_COLLIDING;
            return false;
        }

        // skip removed triangles
        if (triangle.removed) {
            hull.freeTriangle(hullState, triangle);
            continue;
        }

        // check if we found the closest point
        if (triangle.closestLengthSq >= closestDistSq) {
            break;
        }

        // replace last good triangle
        if (last !== null) {
            hull.freeTriangle(hullState, last);
        }
        last = triangle;

        // add support point in direction of normal
        const newIndex = addEpaSupportPoint(supportPoints, supportAIncludingRadius, supportBIncludingRadius, triangle.normal);
        const w = supportPoints.y.values[newIndex];

        // project w onto triangle normal
        const dot = vec3.dot(triangle.normal, w);

        // check for separating axis
        if (dot < 0.0) {
            out.status = PenetrationDepthStatus.NOT_COLLIDING;
            return false;
        }

        // get distance squared along normal
        const distSq = (dot * dot) / vec3.squaredLength(triangle.normal);

        // check for convergence
        if (distSq - triangle.closestLengthSq < triangle.closestLengthSq * tolerance) {
            break;
        }

        closestDistSq = Math.min(closestDistSq, distSq);

        // check if triangle thinks point is not front facing
        if (!hull.triangleIsFacing(triangle, w)) {
            break;
        }

        // add point to hull
        const newTriangles = _epa_newTriangles;
        newTriangles.length = 0;

        if (!hull.addPoint(hullState, triangle, newIndex, closestDistSq, newTriangles)) {
            break;
        }

        // check for hull defects
        let hasDefect = false;
        for (let i = 0; i < newTriangles.length; i++) {
            const nt = newTriangles[i];
            if (nt && hull.triangleIsFacingOrigin(nt)) {
                hasDefect = true;
                break;
            }
        }

        if (hasDefect) {
            // check if we need to flip penetration sign
            vec3.negate(_epa_negatedNormal, triangle.normal);
            supportAIncludingRadius.getSupport(_epa_negatedNormal, _epa_p2);
            supportBIncludingRadius.getSupport(triangle.normal, _epa_q2);
            vec3.subtract(_epa_w2, _epa_p2, _epa_q2);
            const dot2 = vec3.dot(_epa_negatedNormal, _epa_w2);
            if (dot2 < dot) {
                flipVSign = true;
            }
            break;
        }
    } while (hull.hasNextTriangle(hullState) && supportPoints.y.size < EPA_MAX_POINTS);

    // console.log('epa main loop iters', iter, 'simplex size', simplex.size);

    // calculate results
    if (last === null) {
        out.status = PenetrationDepthStatus.NOT_COLLIDING;
        return false;
    }

    // calculate penetration normal and depth
    const normalLengthSq = vec3.squaredLength(last.normal);
    const centroidDotNormal = vec3.dot(last.centroid, last.normal);

    // penetration normal calculation
    const penetrationNormal = _epa_penetrationNormal;
    vec3.scale(penetrationNormal, last.normal, centroidDotNormal / normalLengthSq);

    // check for near-zero penetration
    if (vec3.squaredLength(penetrationNormal) < 1e-10) {
        out.status = PenetrationDepthStatus.NOT_COLLIDING;
        return false;
    }

    if (flipVSign) {
        vec3.negate(penetrationNormal, penetrationNormal);
    }

    // calculate contact points using barycentric coordinates
    const xp0 = supportPoints.p.values[last.edge[0].startIndex];
    const xp1 = supportPoints.p.values[last.edge[1].startIndex];
    const xp2 = supportPoints.p.values[last.edge[2].startIndex];

    const xq0 = supportPoints.q.values[last.edge[0].startIndex];
    const xq1 = supportPoints.q.values[last.edge[1].startIndex];
    const xq2 = supportPoints.q.values[last.edge[2].startIndex];

    const contactPointA = _epa_contactPointA;
    const contactPointB = _epa_contactPointB;

    if (last.lambdaRelativeTo0) {
        vec3.subtract(_epa_p01, xp1, xp0);
        vec3.subtract(_epa_p02, xp2, xp0);
        vec3.subtract(_epa_q01, xq1, xq0);
        vec3.subtract(_epa_q02, xq2, xq0);

        vec3.copy(contactPointA, xp0);
        vec3.scaleAndAdd(contactPointA, contactPointA, _epa_p01, last.lambda[0]);
        vec3.scaleAndAdd(contactPointA, contactPointA, _epa_p02, last.lambda[1]);

        vec3.copy(contactPointB, xq0);
        vec3.scaleAndAdd(contactPointB, contactPointB, _epa_q01, last.lambda[0]);
        vec3.scaleAndAdd(contactPointB, contactPointB, _epa_q02, last.lambda[1]);
    } else {
        vec3.subtract(_epa_p10, xp0, xp1);
        vec3.subtract(_epa_p12, xp2, xp1);
        vec3.subtract(_epa_q10, xq0, xq1);
        vec3.subtract(_epa_q12, xq2, xq1);

        vec3.copy(contactPointA, xp1);
        vec3.scaleAndAdd(contactPointA, contactPointA, _epa_p10, last.lambda[0]);
        vec3.scaleAndAdd(contactPointA, contactPointA, _epa_p12, last.lambda[1]);

        vec3.copy(contactPointB, xq1);
        vec3.scaleAndAdd(contactPointB, contactPointB, _epa_q10, last.lambda[0]);
        vec3.scaleAndAdd(contactPointB, contactPointB, _epa_q12, last.lambda[1]);
    }

    // write results to out
    out.penetrationAxis[0] = penetrationNormal[0];
    out.penetrationAxis[1] = penetrationNormal[1];
    out.penetrationAxis[2] = penetrationNormal[2];

    out.pointA[0] = contactPointA[0];
    out.pointA[1] = contactPointA[1];
    out.pointA[2] = contactPointA[2];

    out.pointB[0] = contactPointB[0];
    out.pointB[1] = contactPointB[1];
    out.pointB[2] = contactPointB[2];

    out.status = PenetrationDepthStatus.COLLIDING;

    return true;
}

const _castShape_penetrationDepth = /* @__PURE__ */ createPenetrationDepth();
const _castShape_addRadiusA = /* @__PURE__ */ createAddConvexRadiusSupport();
const _castShape_addRadiusB = /* @__PURE__ */ createAddConvexRadiusSupport();
const _castShape_transformedA = /* @__PURE__ */ createTransformedSupport();

export function penetrationCastShape(
    out: GjkCastShapeResult,
    posAInB: Vec3,
    quatAInB: Quat,
    shapeASupport: Support,
    shapeBSupport: Support,
    displacement: Vec3,
    collisionTolerance: number,
    penetrationTolerance: number,
    convexRadiusA: number,
    convexRadiusB: number,
    maxLambda: number,
    returnDeepestPoint: boolean,
): void {
    // first determine if there's a collision at all
    gjkCastShape(
        out,
        posAInB,
        quatAInB,
        shapeASupport,
        shapeBSupport,
        displacement,
        collisionTolerance,
        convexRadiusA,
        convexRadiusB,
        maxLambda,
    );

    if (!out.hit) {
        return;
    }

    // when our contact normal is too small, we don't have an accurate result
    const squaredTolerance = collisionTolerance * collisionTolerance;
    const contactNormalInvalid = vec3.squaredLength(out.separatingAxis) < squaredTolerance;

    const combinedRadius = convexRadiusA + convexRadiusB;
    // only when lambda = 0 we can have the bodies overlap
    // when no convex radius was provided we can never trust contact points at lambda = 0
    const shouldDoEPA = returnDeepestPoint && out.lambda === 0.0 && (combinedRadius === 0.0 || contactNormalInvalid);

    if (shouldDoEPA) {
        // if we're initially intersecting, we need to run the EPA algorithm in order to find the deepest contact point
        setAddConvexRadiusSupport(_castShape_addRadiusA, convexRadiusA, shapeASupport);
        setAddConvexRadiusSupport(_castShape_addRadiusB, convexRadiusB, shapeBSupport);
        setTransformedSupport(_castShape_transformedA, posAInB, quatAInB, _castShape_addRadiusA);

        if (
            !penetrationDepthStepEPA(
                _castShape_penetrationDepth,
                _castShape_transformedA,
                _castShape_addRadiusB,
                penetrationTolerance,
                out.simplex,
            )
        ) {
            out.hit = false;
            return;
        }

        vec3.copy(out.separatingAxis, _castShape_penetrationDepth.penetrationAxis);
        vec3.copy(out.pointA, _castShape_penetrationDepth.pointA);
        vec3.copy(out.pointB, _castShape_penetrationDepth.pointB);
    } else if (contactNormalInvalid) {
        // if we weren't able to calculate a contact normal, use the cast direction instead
        vec3.copy(out.separatingAxis, displacement);
    }
}
