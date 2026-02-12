import { degreesToRadians, type Quat, type Vec3, vec3 } from 'mathcat';
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
    const combinedRadius = convexRadiusA + convexRadiusB;
    const combinedRadiusSquared = combinedRadius * combinedRadius;

    // run GJK to find the closest points between shapes
    gjkClosestPoints(_gjk_closestPoints, supportA, supportB, tolerance, direction, combinedRadiusSquared);

    // check if shapes are separated by more than the combined convex radius
    if (_gjk_closestPoints.squaredDistance > combinedRadiusSquared) {
        // shapes are separated - no collision
        // explicitly clear contact data to avoid stale data from previous collisions
        outPenetrationDepth.pointA[0] = 0;
        outPenetrationDepth.pointA[1] = 0;
        outPenetrationDepth.pointA[2] = 0;
        outPenetrationDepth.pointB[0] = 0;
        outPenetrationDepth.pointB[1] = 0;
        outPenetrationDepth.pointB[2] = 0;
        outPenetrationDepth.penetrationAxis[0] = 0;
        outPenetrationDepth.penetrationAxis[1] = 0;
        outPenetrationDepth.penetrationAxis[2] = 0;
        outPenetrationDepth.status = PenetrationDepthStatus.NOT_COLLIDING;
        return;
    }

    // copy results to output

    outPenetrationDepth.pointA[0] = _gjk_closestPoints.pointA[0];
    outPenetrationDepth.pointA[1] = _gjk_closestPoints.pointA[1];
    outPenetrationDepth.pointA[2] = _gjk_closestPoints.pointA[2];

    outPenetrationDepth.pointB[0] = _gjk_closestPoints.pointB[0];
    outPenetrationDepth.pointB[1] = _gjk_closestPoints.pointB[1];
    outPenetrationDepth.pointB[2] = _gjk_closestPoints.pointB[2];

    outPenetrationDepth.penetrationAxis[0] = _gjk_closestPoints.penetrationAxis[0];
    outPenetrationDepth.penetrationAxis[1] = _gjk_closestPoints.penetrationAxis[1];
    outPenetrationDepth.penetrationAxis[2] = _gjk_closestPoints.penetrationAxis[2];

    copySimplex(outSimplex, _gjk_closestPoints.simplex);

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

const _epa_dir1 = /* @__PURE__ */ vec3.create();
const _epa_dir2 = /* @__PURE__ */ vec3.create();
const _epa_dir3 = /* @__PURE__ */ vec3.create();
const _epa_p = /* @__PURE__ */ vec3.create();
const _epa_q = /* @__PURE__ */ vec3.create();
const _epa_negatedDirection = /* @__PURE__ */ vec3.create();
const _epa_negatedNormal = /* @__PURE__ */ vec3.create();
const _epa_p2 = /* @__PURE__ */ vec3.create();
const _epa_q2 = /* @__PURE__ */ vec3.create();

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

    // store new point
    const idx = points.y.size;
    const yOut = points.y.values[idx];
    const pOut = points.p.values[idx];
    const qOut = points.q.values[idx];

    // y = p - q (minkowski difference)
    yOut[0] = _epa_p[0] - _epa_q[0];
    yOut[1] = _epa_p[1] - _epa_q[1];
    yOut[2] = _epa_p[2] - _epa_q[2];

    pOut[0] = _epa_p[0];
    pOut[1] = _epa_p[1];
    pOut[2] = _epa_p[2];

    qOut[0] = _epa_q[0];
    qOut[1] = _epa_q[1];
    qOut[2] = _epa_q[2];

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

const _epa_supportPoints = /* @__PURE__ */ createEpaSupportPoints(256);
const _epa_hullState = /* @__PURE__ */ (() => {
    const state = hull.init();
    // set hull positions once - the array reference stays the same, we just clear/refill it
    state.positions = _epa_supportPoints.y.values;
    return state;
})();
const _epa_newTriangles: hull.NewTriangles = [];

// rotation matrix constants for 120° rotation
const COS_120_DEG = Math.cos(degreesToRadians(120));
const SIN_120_DEG = Math.sin(degreesToRadians(120));
const ONE_MINUS_COS_120_DEG = 1 - COS_120_DEG;

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
        const point = simplex.points[i];
        const yOut = supportPoints.y.values[i];
        const pOut = supportPoints.p.values[i];
        const qOut = supportPoints.q.values[i];
        yOut[0] = point.y[0];
        yOut[1] = point.y[1];
        yOut[2] = point.y[2];
        pOut[0] = point.p[0];
        pOut[1] = point.p[1];
        pOut[2] = point.p[2];
        qOut[0] = point.q[0];
        qOut[1] = point.q[1];
        qOut[2] = point.q[2];
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
            const y0 = supportPoints.y.values[0];
            const y1 = supportPoints.y.values[1];

            // axis = normalize(y1 - y0)
            const axisx = y1[0] - y0[0];
            const axisy = y1[1] - y0[1];
            const axisz = y1[2] - y0[2];
            const axisLen = Math.sqrt(axisx * axisx + axisy * axisy + axisz * axisz);
            const axisNormx = axisx / axisLen;
            const axisNormy = axisy / axisLen;
            const axisNormz = axisz / axisLen;

            // compute normalized perpendicular to axis
            const absAxisx = Math.abs(axisNormx);
            const absAxisy = Math.abs(axisNormy);

            let dir1x: number, dir1y: number, dir1z: number;
            if (absAxisx > absAxisy) {
                // use X and Z components
                const perpLen = Math.sqrt(axisNormx * axisNormx + axisNormz * axisNormz);
                dir1x = axisNormz / perpLen;
                dir1y = 0.0;
                dir1z = -axisNormx / perpLen;
            } else {
                // use Y and Z components
                const perpLen = Math.sqrt(axisNormy * axisNormy + axisNormz * axisNormz);
                dir1x = 0.0;
                dir1y = axisNormz / perpLen;
                dir1z = -axisNormy / perpLen;
            }

            // construct rotation matrix for 120° rotation about axis
            // R = cos(θ)I + sin(θ)[k]× + (1-cos(θ))kk^T
            const r00 = COS_120_DEG + axisNormx * axisNormx * ONE_MINUS_COS_120_DEG;
            const r01 = axisNormx * axisNormy * ONE_MINUS_COS_120_DEG - axisNormz * SIN_120_DEG;
            const r02 = axisNormx * axisNormz * ONE_MINUS_COS_120_DEG + axisNormy * SIN_120_DEG;
            const r10 = axisNormy * axisNormx * ONE_MINUS_COS_120_DEG + axisNormz * SIN_120_DEG;
            const r11 = COS_120_DEG + axisNormy * axisNormy * ONE_MINUS_COS_120_DEG;
            const r12 = axisNormy * axisNormz * ONE_MINUS_COS_120_DEG - axisNormx * SIN_120_DEG;
            const r20 = axisNormz * axisNormx * ONE_MINUS_COS_120_DEG - axisNormy * SIN_120_DEG;
            const r21 = axisNormz * axisNormy * ONE_MINUS_COS_120_DEG + axisNormx * SIN_120_DEG;
            const r22 = COS_120_DEG + axisNormz * axisNormz * ONE_MINUS_COS_120_DEG;

            // dir2 = R * dir1
            const dir2x = r00 * dir1x + r01 * dir1y + r02 * dir1z;
            const dir2y = r10 * dir1x + r11 * dir1y + r12 * dir1z;
            const dir2z = r20 * dir1x + r21 * dir1y + r22 * dir1z;

            // dir3 = R * dir2
            const dir3x = r00 * dir2x + r01 * dir2y + r02 * dir2z;
            const dir3y = r10 * dir2x + r11 * dir2y + r12 * dir2z;
            const dir3z = r20 * dir2x + r21 * dir2y + r22 * dir2z;

            _epa_dir1[0] = dir1x;
            _epa_dir1[1] = dir1y;
            _epa_dir1[2] = dir1z;

            _epa_dir2[0] = dir2x;
            _epa_dir2[1] = dir2y;
            _epa_dir2[2] = dir2z;

            _epa_dir3[0] = dir3x;
            _epa_dir3[1] = dir3y;
            _epa_dir3[2] = dir3z;

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
        const dot = triangle.normal[0] * w[0] + triangle.normal[1] * w[1] + triangle.normal[2] * w[2];

        // check for separating axis
        if (dot < 0.0) {
            out.status = PenetrationDepthStatus.NOT_COLLIDING;
            return false;
        }

        // get distance squared along normal
        const normalLenSq =
            triangle.normal[0] * triangle.normal[0] +
            triangle.normal[1] * triangle.normal[1] +
            triangle.normal[2] * triangle.normal[2];
        const distSq = (dot * dot) / normalLenSq;

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
            // triangleIsFacingOrigin: check if triangle normal points toward origin
            // if dot(normal, centroid) < 0, origin is on front side (defect)
            if (nt && nt.normal[0] * nt.centroid[0] + nt.normal[1] * nt.centroid[1] + nt.normal[2] * nt.centroid[2] < 0.0) {
                hasDefect = true;
                break;
            }
        }

        if (hasDefect) {
            // check if we need to flip penetration sign
            _epa_negatedNormal[0] = -triangle.normal[0];
            _epa_negatedNormal[1] = -triangle.normal[1];
            _epa_negatedNormal[2] = -triangle.normal[2];
            supportAIncludingRadius.getSupport(_epa_negatedNormal, _epa_p2);
            supportBIncludingRadius.getSupport(triangle.normal, _epa_q2);
            const w2x = _epa_p2[0] - _epa_q2[0];
            const w2y = _epa_p2[1] - _epa_q2[1];
            const w2z = _epa_p2[2] - _epa_q2[2];
            const dot2 = _epa_negatedNormal[0] * w2x + _epa_negatedNormal[1] * w2y + _epa_negatedNormal[2] * w2z;
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
    const normalLengthSq = last.normal[0] * last.normal[0] + last.normal[1] * last.normal[1] + last.normal[2] * last.normal[2];
    const centroidDotNormal =
        last.centroid[0] * last.normal[0] + last.centroid[1] * last.normal[1] + last.centroid[2] * last.normal[2];

    // penetration normal calculation
    const penetrationNormal = _epa_penetrationNormal;
    vec3.scale(penetrationNormal, last.normal, centroidDotNormal / normalLengthSq);

    // check for near-zero penetration
    const pnLenSq =
        penetrationNormal[0] * penetrationNormal[0] +
        penetrationNormal[1] * penetrationNormal[1] +
        penetrationNormal[2] * penetrationNormal[2];
    if (pnLenSq < 1e-10) {
        out.status = PenetrationDepthStatus.NOT_COLLIDING;
        return false;
    }

    if (flipVSign) {
        // penetrationNormal = -penetrationNormal
        penetrationNormal[0] = -penetrationNormal[0];
        penetrationNormal[1] = -penetrationNormal[1];
        penetrationNormal[2] = -penetrationNormal[2];
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
        // cache vertex components
        const [xp0x, xp0y, xp0z] = xp0;
        const [xp1x, xp1y, xp1z] = xp1;
        const [xp2x, xp2y, xp2z] = xp2;
        const [xq0x, xq0y, xq0z] = xq0;
        const [xq1x, xq1y, xq1z] = xq1;
        const [xq2x, xq2y, xq2z] = xq2;

        // contactPointA = xp0 + (xp1 - xp0) * lambda[0] + (xp2 - xp0) * lambda[1]
        const lambda0 = last.lambda[0];
        const lambda1 = last.lambda[1];
        contactPointA[0] = xp0x + (xp1x - xp0x) * lambda0 + (xp2x - xp0x) * lambda1;
        contactPointA[1] = xp0y + (xp1y - xp0y) * lambda0 + (xp2y - xp0y) * lambda1;
        contactPointA[2] = xp0z + (xp1z - xp0z) * lambda0 + (xp2z - xp0z) * lambda1;

        // contactPointB = xq0 + (xq1 - xq0) * lambda[0] + (xq2 - xq0) * lambda[1]
        contactPointB[0] = xq0x + (xq1x - xq0x) * lambda0 + (xq2x - xq0x) * lambda1;
        contactPointB[1] = xq0y + (xq1y - xq0y) * lambda0 + (xq2y - xq0y) * lambda1;
        contactPointB[2] = xq0z + (xq1z - xq0z) * lambda0 + (xq2z - xq0z) * lambda1;
    } else {
        // cache vertex components
        const [xp0x, xp0y, xp0z] = xp0;
        const [xp1x, xp1y, xp1z] = xp1;
        const [xp2x, xp2y, xp2z] = xp2;
        const [xq0x, xq0y, xq0z] = xq0;
        const [xq1x, xq1y, xq1z] = xq1;
        const [xq2x, xq2y, xq2z] = xq2;

        // contactPointA = xp1 + (xp0 - xp1) * lambda[0] + (xp2 - xp1) * lambda[1]
        const lambda0 = last.lambda[0];
        const lambda1 = last.lambda[1];
        contactPointA[0] = xp1x + (xp0x - xp1x) * lambda0 + (xp2x - xp1x) * lambda1;
        contactPointA[1] = xp1y + (xp0y - xp1y) * lambda0 + (xp2y - xp1y) * lambda1;
        contactPointA[2] = xp1z + (xp0z - xp1z) * lambda0 + (xp2z - xp1z) * lambda1;

        // contactPointB = xq1 + (xq0 - xq1) * lambda[0] + (xq2 - xq1) * lambda[1]
        contactPointB[0] = xq1x + (xq0x - xq1x) * lambda0 + (xq2x - xq1x) * lambda1;
        contactPointB[1] = xq1y + (xq0y - xq1y) * lambda0 + (xq2y - xq1y) * lambda1;
        contactPointB[2] = xq1z + (xq0z - xq1z) * lambda0 + (xq2z - xq1z) * lambda1;
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
