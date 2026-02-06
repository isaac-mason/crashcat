import type { Vec3 } from 'mathcat';
import { vec3 } from 'mathcat';

/**
 * Feature bits for getClosestPointOnTriangle result.
 * Indicates which part of the triangle contains the closest point.
 */

export enum TriangleFeature {
    /** closest to vertex A (v0) */
    VERTEX_A = 0b001,
    /** closest to vertex B (v1) */
    VERTEX_B = 0b010,
    /** closest to vertex C (v2) */
    VERTEX_C = 0b100,
    /** closest to edge AB (v0-v1) */
    EDGE_AB = 0b011,
    /** closest to edge AC (v0-v2) */
    EDGE_AC = 0b101,
    /** closest to edge BC (v1-v2) */
    EDGE_BC = 0b110,
    /** closest to interior face */
    FACE = 0b111,
}

export type ClosestPointOnTriangleResult = {
    /** closest point on triangle (in same coordinate space as input vertices) */
    point: Vec3;
    /** squared distance from origin to closest point */
    distanceSq: number;
    /** feature bitmask indicating which part of triangle is closest (see TriangleFeature) */
    feature: number;
};

export function createClosestPointOnTriangleResult(): ClosestPointOnTriangleResult {
    return {
        point: vec3.create(),
        distanceSq: 0,
        feature: 0,
    };
}
const _cpt_ab = vec3.create();
const _cpt_ac = vec3.create();
const _cpt_bc = vec3.create();
const _cpt_ap = vec3.create();
const _cpt_bp = vec3.create();
const _cpt_cp = vec3.create();
const _cpt_n = vec3.create();
const _cpt_q = vec3.create();

/**
 * Get the closest point on a triangle to the origin.
 * Based on JoltPhysics ClosestPoint::GetClosestPointOnTriangle and
 * "Real-Time Collision Detection" by Christer Ericson.
 *
 * @param out result object to store the closest point, squared distance, and feature
 * @param a first vertex of triangle (relative to query point, i.e., query point is at origin)
 * @param b second vertex of triangle (relative to query point)
 * @param c third vertex of triangle (relative to query point)
 */

export function getClosestPointOnTriangle(out: ClosestPointOnTriangleResult, a: Vec3, b: Vec3, c: Vec3): void {
    // edge vectors
    const ab = _cpt_ab;
    ab[0] = b[0] - a[0];
    ab[1] = b[1] - a[1];
    ab[2] = b[2] - a[2];

    const ac = _cpt_ac;
    ac[0] = c[0] - a[0];
    ac[1] = c[1] - a[1];
    ac[2] = c[2] - a[2];

    // check for degenerate triangle by computing normal
    const n = _cpt_n;
    n[0] = ab[1] * ac[2] - ab[2] * ac[1];
    n[1] = ab[2] * ac[0] - ab[0] * ac[2];
    n[2] = ab[0] * ac[1] - ab[1] * ac[0];
    const nLenSq = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];

    if (nLenSq < 1e-10) {
        // degenerate triangle - fallback to vertices and edges
        handleDegenerateTriangle(out, a, b, c, ab, ac);
        return;
    }

    // point relative to a (which is -a since p = origin = 0)
    const ap = _cpt_ap;
    ap[0] = -a[0];
    ap[1] = -a[1];
    ap[2] = -a[2];

    const d1 = ab[0] * ap[0] + ab[1] * ap[1] + ab[2] * ap[2]; // ab.dot(ap)
    const d2 = ac[0] * ap[0] + ac[1] * ap[1] + ac[2] * ap[2]; // ac.dot(ap)

    // check if P in vertex region outside A
    if (d1 <= 0 && d2 <= 0) {
        out.point[0] = a[0];
        out.point[1] = a[1];
        out.point[2] = a[2];
        out.distanceSq = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
        out.feature = TriangleFeature.VERTEX_A;
        return;
    }

    // point relative to b (which is -b since p = origin = 0)
    const bp = _cpt_bp;
    bp[0] = -b[0];
    bp[1] = -b[1];
    bp[2] = -b[2];

    const d3 = ab[0] * bp[0] + ab[1] * bp[1] + ab[2] * bp[2]; // ab.dot(bp)
    const d4 = ac[0] * bp[0] + ac[1] * bp[1] + ac[2] * bp[2]; // ac.dot(bp)

    // check if P in vertex region outside B
    if (d3 >= 0 && d4 <= d3) {
        out.point[0] = b[0];
        out.point[1] = b[1];
        out.point[2] = b[2];
        out.distanceSq = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
        out.feature = TriangleFeature.VERTEX_B;
        return;
    }

    // check if P in edge region of AB
    if (d1 * d4 <= d3 * d2 && d1 >= 0 && d3 <= 0) {
        const v = d1 / (d1 - d3);
        out.point[0] = a[0] + v * ab[0];
        out.point[1] = a[1] + v * ab[1];
        out.point[2] = a[2] + v * ab[2];
        out.distanceSq = out.point[0] * out.point[0] + out.point[1] * out.point[1] + out.point[2] * out.point[2];
        out.feature = TriangleFeature.EDGE_AB;
        return;
    }

    // point relative to c (which is -c since p = origin = 0)
    const cp = _cpt_cp;
    cp[0] = -c[0];
    cp[1] = -c[1];
    cp[2] = -c[2];

    const d5 = ab[0] * cp[0] + ab[1] * cp[1] + ab[2] * cp[2]; // ab.dot(cp)
    const d6 = ac[0] * cp[0] + ac[1] * cp[1] + ac[2] * cp[2]; // ac.dot(cp)

    // check if P in vertex region outside C
    if (d6 >= 0 && d5 <= d6) {
        out.point[0] = c[0];
        out.point[1] = c[1];
        out.point[2] = c[2];
        out.distanceSq = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
        out.feature = TriangleFeature.VERTEX_C;
        return;
    }

    // check if P in edge region of AC
    if (d5 * d2 <= d1 * d6 && d2 >= 0 && d6 <= 0) {
        const w = d2 / (d2 - d6);
        out.point[0] = a[0] + w * ac[0];
        out.point[1] = a[1] + w * ac[1];
        out.point[2] = a[2] + w * ac[2];
        out.distanceSq = out.point[0] * out.point[0] + out.point[1] * out.point[1] + out.point[2] * out.point[2];
        out.feature = TriangleFeature.EDGE_AC;
        return;
    }

    // check if P in edge region of BC
    const d4_d3 = d4 - d3;
    const d5_d6 = d5 - d6;
    if (d3 * d6 <= d5 * d4 && d4_d3 >= 0 && d5_d6 >= 0) {
        const w = d4_d3 / (d4_d3 + d5_d6);
        const bc = _cpt_bc;
        bc[0] = c[0] - b[0];
        bc[1] = c[1] - b[1];
        bc[2] = c[2] - b[2];
        out.point[0] = b[0] + w * bc[0];
        out.point[1] = b[1] + w * bc[1];
        out.point[2] = b[2] + w * bc[2];
        out.distanceSq = out.point[0] * out.point[0] + out.point[1] * out.point[1] + out.point[2] * out.point[2];
        out.feature = TriangleFeature.EDGE_BC;
        return;
    }

    // P inside face region.
    // compute closest point using the more accurate method from JoltPhysics:
    // distance = (centroid - origin) . normal / |normal|
    // closest = distance * normal / |normal|
    const centroidDotN = (a[0] + b[0] + c[0]) * n[0] + (a[1] + b[1] + c[1]) * n[1] + (a[2] + b[2] + c[2]) * n[2];
    const scale = centroidDotN / (3 * nLenSq);
    out.point[0] = n[0] * scale;
    out.point[1] = n[1] * scale;
    out.point[2] = n[2] * scale;
    out.distanceSq = out.point[0] * out.point[0] + out.point[1] * out.point[1] + out.point[2] * out.point[2];
    out.feature = TriangleFeature.FACE;
}
function handleDegenerateTriangle(out: ClosestPointOnTriangleResult, a: Vec3, b: Vec3, c: Vec3, ab: Vec3, ac: Vec3): void {
    const q = _cpt_q;

    // start with vertex C being the closest
    let closestFeature = TriangleFeature.VERTEX_C;
    out.point[0] = c[0];
    out.point[1] = c[1];
    out.point[2] = c[2];
    let bestDistSq = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];

    // try vertex A
    const aLenSq = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    if (aLenSq < bestDistSq) {
        closestFeature = TriangleFeature.VERTEX_A;
        out.point[0] = a[0];
        out.point[1] = a[1];
        out.point[2] = a[2];
        bestDistSq = aLenSq;
    }

    // try vertex B
    const bLenSq = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    if (bLenSq < bestDistSq) {
        closestFeature = TriangleFeature.VERTEX_B;
        out.point[0] = b[0];
        out.point[1] = b[1];
        out.point[2] = b[2];
        bestDistSq = bLenSq;
    }

    const FLT_EPSILON_SQ = 1.1920929e-7 * 1.1920929e-7;

    // edge AC
    const acLenSq = ac[0] * ac[0] + ac[1] * ac[1] + ac[2] * ac[2];
    if (acLenSq > FLT_EPSILON_SQ) {
        const aDotAc = a[0] * ac[0] + a[1] * ac[1] + a[2] * ac[2];
        const v = Math.max(0, Math.min(1, -aDotAc / acLenSq));
        q[0] = a[0] + v * ac[0];
        q[1] = a[1] + v * ac[1];
        q[2] = a[2] + v * ac[2];
        const distSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
        if (distSq < bestDistSq) {
            closestFeature = TriangleFeature.EDGE_AC;
            out.point[0] = q[0];
            out.point[1] = q[1];
            out.point[2] = q[2];
            bestDistSq = distSq;
        }
    }

    // edge BC
    const bc = _cpt_bc;
    bc[0] = c[0] - b[0];
    bc[1] = c[1] - b[1];
    bc[2] = c[2] - b[2];
    const bcLenSq = bc[0] * bc[0] + bc[1] * bc[1] + bc[2] * bc[2];
    if (bcLenSq > FLT_EPSILON_SQ) {
        const bDotBc = b[0] * bc[0] + b[1] * bc[1] + b[2] * bc[2];
        const v = Math.max(0, Math.min(1, -bDotBc / bcLenSq));
        q[0] = b[0] + v * bc[0];
        q[1] = b[1] + v * bc[1];
        q[2] = b[2] + v * bc[2];
        const distSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
        if (distSq < bestDistSq) {
            closestFeature = TriangleFeature.EDGE_BC;
            out.point[0] = q[0];
            out.point[1] = q[1];
            out.point[2] = q[2];
            bestDistSq = distSq;
        }
    }

    // edge AB
    const abLenSq = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    if (abLenSq > FLT_EPSILON_SQ) {
        const aDotAb = a[0] * ab[0] + a[1] * ab[1] + a[2] * ab[2];
        const v = Math.max(0, Math.min(1, -aDotAb / abLenSq));
        q[0] = a[0] + v * ab[0];
        q[1] = a[1] + v * ab[1];
        q[2] = a[2] + v * ab[2];
        const distSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
        if (distSq < bestDistSq) {
            closestFeature = TriangleFeature.EDGE_AB;
            out.point[0] = q[0];
            out.point[1] = q[1];
            out.point[2] = q[2];
            bestDistSq = distSq;
        }
    }

    out.distanceSq = bestDistSq;
    out.feature = closestFeature;
}
