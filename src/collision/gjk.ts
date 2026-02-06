import { clamp, type Quat, type Vec3, vec3 } from 'mathcat';
import {
    computeBarycentricCoordinates2d,
    computeBarycentricCoordinates3d,
    createBarycentricCoordinatesResult,
} from './closest-points';
import { copySimplex, createSimplex, type Simplex } from './simplex';
import type { Support } from './support';
import { createTransformedSupport, setTransformedSupport } from './support';

/*
References:
- Jolt Physics
- Bounce Physics

Originally based on: A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen
*/

type ClosestPointResult = {
    point: Vec3;
    pointSet: number;
};

function createClosestPointResult(): ClosestPointResult {
    return {
        point: vec3.create(),
        pointSet: 0,
    };
}

const _barycentric_line = createBarycentricCoordinatesResult();

export function computeClosestPointOnLine(out: ClosestPointResult, a: Vec3, b: Vec3, squaredTolerance: number): void {
    computeBarycentricCoordinates2d(_barycentric_line, a, b, squaredTolerance);

    if (_barycentric_line.v <= 0.0) {
        // a is closest point
        // vec3.copy(out.point, a);
        out.point[0] = a[0];
        out.point[1] = a[1];
        out.point[2] = a[2];
        out.pointSet = 0b0001;
    } else if (_barycentric_line.u <= 0.0) {
        // b is closest point
        // vec3.copy(out.point, b);
        out.point[0] = b[0];
        out.point[1] = b[1];
        out.point[2] = b[2];
        out.pointSet = 0b0010;
    } else {
        // closest point lies on line ab
        // vec3.zero(out.point);
        // vec3.scaleAndAdd(out.point, out.point, a, _barycentric_line.u);
        // vec3.scaleAndAdd(out.point, out.point, b, _barycentric_line.v);
        out.point[0] = a[0] * _barycentric_line.u + b[0] * _barycentric_line.v;
        out.point[1] = a[1] * _barycentric_line.u + b[1] * _barycentric_line.v;
        out.point[2] = a[2] * _barycentric_line.u + b[2] * _barycentric_line.v;
        out.pointSet = 0b0011;
    }
}

const _ac_tri = vec3.create();
const _ab_tri = vec3.create();
const _bc_tri = vec3.create();
const _a_tri = vec3.create();
const _c_tri = vec3.create();
const _n_tri = vec3.create();
const _ap_tri = vec3.create();
const _bp_tri = vec3.create();
const _cp_tri = vec3.create();
const _closestPoint_tri = vec3.create();
const _q_tri = vec3.create();
const _tempVector_tri = vec3.create();

export function computeClosestPointOnTriangle(
    out: ClosestPointResult,
    inA: Vec3,
    inB: Vec3,
    inC: Vec3,
    mustIncludeC: boolean,
    squaredTolerance: number,
): void {
    // the most accurate normal is calculated by using the two shortest edges
    // vec3.subtract(_ac_tri, inC, inA);
    _ac_tri[0] = inC[0] - inA[0];
    _ac_tri[1] = inC[1] - inA[1];
    _ac_tri[2] = inC[2] - inA[2];
    // vec3.subtract(_bc_tri, inC, inB);
    _bc_tri[0] = inC[0] - inB[0];
    _bc_tri[1] = inC[1] - inB[1];
    _bc_tri[2] = inC[2] - inB[2];
    // const swapAC = vec3.dot(_bc_tri, _bc_tri) < vec3.dot(_ac_tri, _ac_tri);
    const swapAC =
        _bc_tri[0] * _bc_tri[0] + _bc_tri[1] * _bc_tri[1] + _bc_tri[2] * _bc_tri[2] <
        _ac_tri[0] * _ac_tri[0] + _ac_tri[1] * _ac_tri[1] + _ac_tri[2] * _ac_tri[2];

    // vec3.copy(_a_tri, swapAC ? inC : inA);
    // vec3.copy(_c_tri, swapAC ? inA : inC);
    if (swapAC) {
        _a_tri[0] = inC[0];
        _a_tri[1] = inC[1];
        _a_tri[2] = inC[2];
        _c_tri[0] = inA[0];
        _c_tri[1] = inA[1];
        _c_tri[2] = inA[2];
    } else {
        _a_tri[0] = inA[0];
        _a_tri[1] = inA[1];
        _a_tri[2] = inA[2];
        _c_tri[0] = inC[0];
        _c_tri[1] = inC[1];
        _c_tri[2] = inC[2];
    }

    // calculate normal

    // vec3.subtract(_ab_tri, inB, _a_tri);
    _ab_tri[0] = inB[0] - _a_tri[0];
    _ab_tri[1] = inB[1] - _a_tri[1];
    _ab_tri[2] = inB[2] - _a_tri[2];

    // vec3.subtract(_ac_tri, _c_tri, _a_tri);
    _ac_tri[0] = _c_tri[0] - _a_tri[0];
    _ac_tri[1] = _c_tri[1] - _a_tri[1];
    _ac_tri[2] = _c_tri[2] - _a_tri[2];

    // vec3.cross(_n_tri, _ab_tri, _ac_tri);
    _n_tri[0] = _ab_tri[1] * _ac_tri[2] - _ab_tri[2] * _ac_tri[1];
    _n_tri[1] = _ab_tri[2] * _ac_tri[0] - _ab_tri[0] * _ac_tri[2];
    _n_tri[2] = _ab_tri[0] * _ac_tri[1] - _ab_tri[1] * _ac_tri[0];

    // const normalLengthSquared = vec3.squaredLength(_n_tri);
    const normalLengthSquared = _n_tri[0] * _n_tri[0] + _n_tri[1] * _n_tri[1] + _n_tri[2] * _n_tri[2];

    // check degenerate
    if (normalLengthSquared < 1.0e-10) {
        // degenerate, fallback to vertices and edges
        let closestSet = 0b0100;
        // vec3.copy(_closestPoint_tri, inC);
        _closestPoint_tri[0] = inC[0];
        _closestPoint_tri[1] = inC[1];
        _closestPoint_tri[2] = inC[2];
        // let bestDistanceSquared = vec3.squaredLength(inC);
        let bestDistanceSquared = inC[0] * inC[0] + inC[1] * inC[1] + inC[2] * inC[2];

        if (!mustIncludeC) {
            // try vertex A
            // const aLengthSquared = vec3.squaredLength(inA);
            const aLengthSquared = inA[0] * inA[0] + inA[1] * inA[1] + inA[2] * inA[2];

            if (aLengthSquared < bestDistanceSquared) {
                closestSet = 0b0001;
                // vec3.copy(_closestPoint_tri, inA);
                _closestPoint_tri[0] = inA[0];
                _closestPoint_tri[1] = inA[1];
                _closestPoint_tri[2] = inA[2];
                bestDistanceSquared = aLengthSquared;
            }

            // try vertex B
            // const bLengthSquared = vec3.squaredLength(inB);
            const bLengthSquared = inB[0] * inB[0] + inB[1] * inB[1] + inB[2] * inB[2];
            if (bLengthSquared < bestDistanceSquared) {
                closestSet = 0b0010;
                // vec3.copy(_closestPoint_tri, inB);
                _closestPoint_tri[0] = inB[0];
                _closestPoint_tri[1] = inB[1];
                _closestPoint_tri[2] = inB[2];
                bestDistanceSquared = bLengthSquared;
            }
        }

        // edge AC
        // const acLengthSquared = vec3.squaredLength(_ac_tri);
        const acLengthSquared = _ac_tri[0] * _ac_tri[0] + _ac_tri[1] * _ac_tri[1] + _ac_tri[2] * _ac_tri[2];

        if (acLengthSquared > squaredTolerance) {
            const v = clamp(-vec3.dot(_a_tri, _ac_tri) / acLengthSquared, 0.0, 1.0);
            // vec3.scaleAndAdd(_q_tri, _a_tri, _ac_tri, v);
            _q_tri[0] = _a_tri[0] + _ac_tri[0] * v;
            _q_tri[1] = _a_tri[1] + _ac_tri[1] * v;
            _q_tri[2] = _a_tri[2] + _ac_tri[2] * v;

            // const distanceSquared = vec3.squaredLength(_q_tri);
            const distanceSquared = _q_tri[0] * _q_tri[0] + _q_tri[1] * _q_tri[1] + _q_tri[2] * _q_tri[2];

            if (distanceSquared < bestDistanceSquared) {
                closestSet = 0b0101;
                // vec3.copy(_closestPoint_tri, _q_tri);
                _closestPoint_tri[0] = _q_tri[0];
                _closestPoint_tri[1] = _q_tri[1];
                _closestPoint_tri[2] = _q_tri[2];
                bestDistanceSquared = distanceSquared;
            }
        }

        // edge BC
        // vec3.subtract(_bc_tri, inC, inB);
        _bc_tri[0] = inC[0] - inB[0];
        _bc_tri[1] = inC[1] - inB[1];
        _bc_tri[2] = inC[2] - inB[2];

        // const bcLengthSquared = vec3.squaredLength(_bc_tri);
        const bcLengthSquared = _bc_tri[0] * _bc_tri[0] + _bc_tri[1] * _bc_tri[1] + _bc_tri[2] * _bc_tri[2];

        if (bcLengthSquared > squaredTolerance) {
            const v = clamp(-vec3.dot(inB, _bc_tri) / bcLengthSquared, 0.0, 1.0);

            // vec3.scaleAndAdd(_q_tri, inB, _bc_tri, v);
            _q_tri[0] = inB[0] + _bc_tri[0] * v;
            _q_tri[1] = inB[1] + _bc_tri[1] * v;
            _q_tri[2] = inB[2] + _bc_tri[2] * v;

            // const distanceSquared = vec3.squaredLength(_q_tri);
            const distanceSquared = _q_tri[0] * _q_tri[0] + _q_tri[1] * _q_tri[1] + _q_tri[2] * _q_tri[2];

            if (distanceSquared < bestDistanceSquared) {
                closestSet = 0b0110;
                // vec3.copy(_closestPoint_tri, _q_tri);
                _closestPoint_tri[0] = _q_tri[0];
                _closestPoint_tri[1] = _q_tri[1];
                _closestPoint_tri[2] = _q_tri[2];
                bestDistanceSquared = distanceSquared;
            }
        }

        if (!mustIncludeC) {
            // edge AB
            // vec3.subtract(_ab_tri, inB, inA);
            _ab_tri[0] = inB[0] - inA[0];
            _ab_tri[1] = inB[1] - inA[1];
            _ab_tri[2] = inB[2] - inA[2];

            // const abLengthSquared = vec3.squaredLength(_ab_tri);
            const abLengthSquared = _ab_tri[0] * _ab_tri[0] + _ab_tri[1] * _ab_tri[1] + _ab_tri[2] * _ab_tri[2];

            if (abLengthSquared > squaredTolerance) {
                const v = clamp(-vec3.dot(inA, _ab_tri) / abLengthSquared, 0.0, 1.0);

                // vec3.scaleAndAdd(_q_tri, inA, _ab_tri, v);
                _q_tri[0] = inA[0] + _ab_tri[0] * v;
                _q_tri[1] = inA[1] + _ab_tri[1] * v;
                _q_tri[2] = inA[2] + _ab_tri[2] * v;

                // const distanceSquared = vec3.squaredLength(_q_tri);
                const distanceSquared = _q_tri[0] * _q_tri[0] + _q_tri[1] * _q_tri[1] + _q_tri[2] * _q_tri[2];

                if (distanceSquared < bestDistanceSquared) {
                    closestSet = 0b0011;
                    // vec3.copy(_closestPoint_tri, _q_tri);
                    _closestPoint_tri[0] = _q_tri[0];
                    _closestPoint_tri[1] = _q_tri[1];
                    _closestPoint_tri[2] = _q_tri[2];
                }
            }
        }

        out.pointSet = closestSet;

        // vec3.copy(out.point, _closestPoint_tri);
        out.point[0] = _closestPoint_tri[0];
        out.point[1] = _closestPoint_tri[1];
        out.point[2] = _closestPoint_tri[2];

        return;
    }

    // check if P in vertex region outside A
    // vec3.negate(_ap_tri, _a_tri);
    _ap_tri[0] = -_a_tri[0];
    _ap_tri[1] = -_a_tri[1];
    _ap_tri[2] = -_a_tri[2];

    // const d1 = vec3.dot(_ab_tri, _ap_tri);
    const d1 = _ab_tri[0] * _ap_tri[0] + _ab_tri[1] * _ap_tri[1] + _ab_tri[2] * _ap_tri[2];

    // const d2 = vec3.dot(_ac_tri, _ap_tri);
    const d2 = _ac_tri[0] * _ap_tri[0] + _ac_tri[1] * _ap_tri[1] + _ac_tri[2] * _ap_tri[2];

    if (d1 <= 0.0 && d2 <= 0.0) {
        out.pointSet = swapAC ? 0b0100 : 0b0001;

        // vec3.copy(out.point, _a_tri);
        out.point[0] = _a_tri[0];
        out.point[1] = _a_tri[1];
        out.point[2] = _a_tri[2];

        return;
    }

    // check if P in vertex region outside B
    // vec3.negate(_bp_tri, inB);
    _bp_tri[0] = -inB[0];
    _bp_tri[1] = -inB[1];
    _bp_tri[2] = -inB[2];

    // const d3 = vec3.dot(_ab_tri, _bp_tri);
    const d3 = _ab_tri[0] * _bp_tri[0] + _ab_tri[1] * _bp_tri[1] + _ab_tri[2] * _bp_tri[2];

    // const d4 = vec3.dot(_ac_tri, _bp_tri);
    const d4 = _ac_tri[0] * _bp_tri[0] + _ac_tri[1] * _bp_tri[1] + _ac_tri[2] * _bp_tri[2];

    if (d3 >= 0.0 && d4 <= d3) {
        out.pointSet = 0b0010;

        // vec3.copy(out.point, inB);
        out.point[0] = inB[0];
        out.point[1] = inB[1];
        out.point[2] = inB[2];

        return;
    }

    // check if P in edge region of AB
    if (d1 * d4 <= d3 * d2 && d1 >= 0.0 && d3 <= 0.0) {
        const v = d1 / (d1 - d3);
        out.pointSet = swapAC ? 0b0110 : 0b0011;

        // vec3.scaleAndAdd(out.point, _a_tri, _ab_tri, v);
        out.point[0] = _a_tri[0] + _ab_tri[0] * v;
        out.point[1] = _a_tri[1] + _ab_tri[1] * v;
        out.point[2] = _a_tri[2] + _ab_tri[2] * v;

        return;
    }

    // check if P in vertex region outside C
    // vec3.negate(_cp_tri, _c_tri);
    _cp_tri[0] = -_c_tri[0];
    _cp_tri[1] = -_c_tri[1];
    _cp_tri[2] = -_c_tri[2];

    // const d5 = vec3.dot(_ab_tri, _cp_tri);
    const d5 = _ab_tri[0] * _cp_tri[0] + _ab_tri[1] * _cp_tri[1] + _ab_tri[2] * _cp_tri[2];

    // const d6 = vec3.dot(_ac_tri, _cp_tri);
    const d6 = _ac_tri[0] * _cp_tri[0] + _ac_tri[1] * _cp_tri[1] + _ac_tri[2] * _cp_tri[2];

    if (d6 >= 0.0 && d5 <= d6) {
        out.pointSet = swapAC ? 0b0001 : 0b0100;

        // vec3.copy(out.point, _c_tri);
        out.point[0] = _c_tri[0];
        out.point[1] = _c_tri[1];
        out.point[2] = _c_tri[2];

        return;
    }

    // check if P in edge region of AC
    if (d5 * d2 <= d1 * d6 && d2 >= 0.0 && d6 <= 0.0) {
        const w = d2 / (d2 - d6);
        out.pointSet = 0b0101;

        // vec3.scaleAndAdd(out.point, _a_tri, _ac_tri, w);
        out.point[0] = _a_tri[0] + _ac_tri[0] * w;
        out.point[1] = _a_tri[1] + _ac_tri[1] * w;
        out.point[2] = _a_tri[2] + _ac_tri[2] * w;

        return;
    }

    // check if P in edge region of BC
    const diff_d4_d3 = d4 - d3;
    const diff_d5_d6 = d5 - d6;
    if (d3 * d6 <= d5 * d4 && diff_d4_d3 >= 0.0 && diff_d5_d6 >= 0.0) {
        const w = diff_d4_d3 / (diff_d4_d3 + diff_d5_d6);
        out.pointSet = swapAC ? 0b0011 : 0b0110;

        // vec3.subtract(_tempVector_tri, _c_tri, inB);
        _tempVector_tri[0] = _c_tri[0] - inB[0];
        _tempVector_tri[1] = _c_tri[1] - inB[1];
        _tempVector_tri[2] = _c_tri[2] - inB[2];

        // vec3.scaleAndAdd(out.point, inB, _tempVector_tri, w);
        out.point[0] = inB[0] + _tempVector_tri[0] * w;
        out.point[1] = inB[1] + _tempVector_tri[1] * w;
        out.point[2] = inB[2] + _tempVector_tri[2] * w;

        return;
    }

    // P inside face region
    out.pointSet = 0b0111;

    // vec3.add(_tempVector_tri, _a_tri, inB);
    _tempVector_tri[0] = _a_tri[0] + inB[0];
    _tempVector_tri[1] = _a_tri[1] + inB[1];
    _tempVector_tri[2] = _a_tri[2] + inB[2];

    // vec3.add(_tempVector_tri, _tempVector_tri, _c_tri);
    _tempVector_tri[0] += _c_tri[0];
    _tempVector_tri[1] += _c_tri[1];
    _tempVector_tri[2] += _c_tri[2];

    // vec3.scale(out.point, _n_tri, vec3.dot(_tempVector_tri, _n_tri) / (3 * normalLengthSquared));
    const scale =
        (_tempVector_tri[0] * _n_tri[0] + _tempVector_tri[1] * _n_tri[1] + _tempVector_tri[2] * _n_tri[2]) /
        (3 * normalLengthSquared);
    out.point[0] = _n_tri[0] * scale;
    out.point[1] = _n_tri[1] * scale;
    out.point[2] = _n_tri[2] * scale;
}

const _ab_planes = vec3.create();
const _ac_planes = vec3.create();
const _ad_planes = vec3.create();
const _bd_planes = vec3.create();
const _bc_planes = vec3.create();
const _ab_cross_ac = vec3.create();
const _ac_cross_ad = vec3.create();
const _ad_cross_ab = vec3.create();
const _bd_cross_bc = vec3.create();
const _signP = { x: 0, y: 0, z: 0, w: 0 };
const _signD = { x: 0, y: 0, z: 0, w: 0 };

// helper type for tracking which triangle planes the origin is outside of
type TrianglePlaneFlags = { x: number; y: number; z: number; w: number };

function isOriginOutsideOfTrianglePlanes(out: TrianglePlaneFlags, a: Vec3, b: Vec3, c: Vec3, d: Vec3, tolerance: number): void {
    // vec3.subtract(_ab_planes, b, a);
    _ab_planes[0] = b[0] - a[0];
    _ab_planes[1] = b[1] - a[1];
    _ab_planes[2] = b[2] - a[2];

    // vec3.subtract(_ac_planes, c, a);
    _ac_planes[0] = c[0] - a[0];
    _ac_planes[1] = c[1] - a[1];
    _ac_planes[2] = c[2] - a[2];

    // vec3.subtract(_ad_planes, d, a);
    _ad_planes[0] = d[0] - a[0];
    _ad_planes[1] = d[1] - a[1];
    _ad_planes[2] = d[2] - a[2];

    // vec3.subtract(_bd_planes, d, b);
    _bd_planes[0] = d[0] - b[0];
    _bd_planes[1] = d[1] - b[1];
    _bd_planes[2] = d[2] - b[2];

    // vec3.subtract(_bc_planes, c, b);
    _bc_planes[0] = c[0] - b[0];
    _bc_planes[1] = c[1] - b[1];
    _bc_planes[2] = c[2] - b[2];

    // vec3.cross(_ab_cross_ac, _ab_planes, _ac_planes);
    _ab_cross_ac[0] = _ab_planes[1] * _ac_planes[2] - _ab_planes[2] * _ac_planes[1];
    _ab_cross_ac[1] = _ab_planes[2] * _ac_planes[0] - _ab_planes[0] * _ac_planes[2];
    _ab_cross_ac[2] = _ab_planes[0] * _ac_planes[1] - _ab_planes[1] * _ac_planes[0];

    // vec3.cross(_ac_cross_ad, _ac_planes, _ad_planes);
    _ac_cross_ad[0] = _ac_planes[1] * _ad_planes[2] - _ac_planes[2] * _ad_planes[1];
    _ac_cross_ad[1] = _ac_planes[2] * _ad_planes[0] - _ac_planes[0] * _ad_planes[2];
    _ac_cross_ad[2] = _ac_planes[0] * _ad_planes[1] - _ac_planes[1] * _ad_planes[0];

    // vec3.cross(_ad_cross_ab, _ad_planes, _ab_planes);
    _ad_cross_ab[0] = _ad_planes[1] * _ab_planes[2] - _ad_planes[2] * _ab_planes[1];
    _ad_cross_ab[1] = _ad_planes[2] * _ab_planes[0] - _ad_planes[0] * _ab_planes[2];
    _ad_cross_ab[2] = _ad_planes[0] * _ab_planes[1] - _ad_planes[1] * _ab_planes[0];

    // vec3.cross(_bd_cross_bc, _bd_planes, _bc_planes);
    _bd_cross_bc[0] = _bd_planes[1] * _bc_planes[2] - _bd_planes[2] * _bc_planes[1];
    _bd_cross_bc[1] = _bd_planes[2] * _bc_planes[0] - _bd_planes[0] * _bc_planes[2];
    _bd_cross_bc[2] = _bd_planes[0] * _bc_planes[1] - _bd_planes[1] * _bc_planes[0];

    // for each plane get the side on which the origin is
    // _signP.x = vec3.dot(a, _ab_cross_ac); // ABC
    _signP.x = a[0] * _ab_cross_ac[0] + a[1] * _ab_cross_ac[1] + a[2] * _ab_cross_ac[2];

    // _signP.y = vec3.dot(a, _ac_cross_ad); // ACD
    _signP.y = a[0] * _ac_cross_ad[0] + a[1] * _ac_cross_ad[1] + a[2] * _ac_cross_ad[2];

    // _signP.z = vec3.dot(a, _ad_cross_ab); // ADB
    _signP.z = a[0] * _ad_cross_ab[0] + a[1] * _ad_cross_ab[1] + a[2] * _ad_cross_ab[2];

    // _signP.w = vec3.dot(b, _bd_cross_bc); // BDC
    _signP.w = b[0] * _bd_cross_bc[0] + b[1] * _bd_cross_bc[1] + b[2] * _bd_cross_bc[2];

    // for each plane get the side that is outside (determined by the 4th point)
    // _signD.x = vec3.dot(_ad_planes, _ab_cross_ac); // D
    _signD.x = _ad_planes[0] * _ab_cross_ac[0] + _ad_planes[1] * _ab_cross_ac[1] + _ad_planes[2] * _ab_cross_ac[2];

    // _signD.y = vec3.dot(_ab_planes, _ac_cross_ad); // B
    _signD.y = _ab_planes[0] * _ac_cross_ad[0] + _ab_planes[1] * _ac_cross_ad[1] + _ab_planes[2] * _ac_cross_ad[2];

    // _signD.z = vec3.dot(_ac_planes, _ad_cross_ab); // C
    _signD.z = _ac_planes[0] * _ad_cross_ab[0] + _ac_planes[1] * _ad_cross_ab[1] + _ac_planes[2] * _ad_cross_ab[2];

    // _signD.w = -vec3.dot(_ab_planes, _bd_cross_bc); // A
    _signD.w = -(_ab_planes[0] * _bd_cross_bc[0] + _ab_planes[1] * _bd_cross_bc[1] + _ab_planes[2] * _bd_cross_bc[2]);

    const allPositive = _signD.x > 0 && _signD.y > 0 && _signD.z > 0 && _signD.w > 0;

    if (allPositive) {
        out.x = _signP.x >= -tolerance ? 1 : 0;
        out.y = _signP.y >= -tolerance ? 1 : 0;
        out.z = _signP.z >= -tolerance ? 1 : 0;
        out.w = _signP.w >= -tolerance ? 1 : 0;
        return;
    }

    const allNegative = _signD.x < 0 && _signD.y < 0 && _signD.z < 0 && _signD.w < 0;

    if (allNegative) {
        out.x = _signP.x <= tolerance ? 1 : 0;
        out.y = _signP.y <= tolerance ? 1 : 0;
        out.z = _signP.z <= tolerance ? 1 : 0;
        out.w = _signP.w <= tolerance ? 1 : 0;
        return;
    }

    // mixed signs, degenerate tetrahedron
    out.x = 1;
    out.y = 1;
    out.z = 1;
    out.w = 1;
}

const _otherResult_tet = createClosestPointResult();
const _originOutOfPlanes: TrianglePlaneFlags = { x: 0, y: 0, z: 0, w: 0 };

export function computeClosestPointOnTetrahedron(
    out: ClosestPointResult,
    inA: Vec3,
    inB: Vec3,
    inC: Vec3,
    inD: Vec3,
    mustIncludeD: boolean,
    tolerance: number,
): void {
    const squaredTolerance = tolerance * tolerance;

    // start out assuming point inside all halfspaces
    out.pointSet = 0b1111;
    // vec3.zero(out.point);
    out.point[0] = 0;
    out.point[1] = 0;
    out.point[2] = 0;

    let bestDistanceSquared = Infinity;

    // determine for each of the faces if the origin is outside
    isOriginOutsideOfTrianglePlanes(_originOutOfPlanes, inA, inB, inC, inD, tolerance);

    // if point outside face abc
    if (_originOutOfPlanes.x) {
        if (mustIncludeD) {
            out.pointSet = 0b0001;
            // vec3.copy(out.point, inA);
            out.point[0] = inA[0];
            out.point[1] = inA[1];
            out.point[2] = inA[2];
        } else {
            computeClosestPointOnTriangle(out, inA, inB, inC, false, squaredTolerance);
        }
        // bestDistanceSquared = vec3.squaredLength(out.point);
        bestDistanceSquared = out.point[0] * out.point[0] + out.point[1] * out.point[1] + out.point[2] * out.point[2];
    }

    // face acd
    if (_originOutOfPlanes.y) {
        _otherResult_tet.pointSet = 0;
        // vec3.zero(_otherResult_tet.point);
        _otherResult_tet.point[0] = 0;
        _otherResult_tet.point[1] = 0;
        _otherResult_tet.point[2] = 0;
        computeClosestPointOnTriangle(_otherResult_tet, inA, inC, inD, mustIncludeD, squaredTolerance);
        // const distanceSquared = vec3.squaredLength(_otherResult_tet.point);
        const distanceSquared =
            _otherResult_tet.point[0] * _otherResult_tet.point[0] +
            _otherResult_tet.point[1] * _otherResult_tet.point[1] +
            _otherResult_tet.point[2] * _otherResult_tet.point[2];
        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            // vec3.copy(out.point, _otherResult_tet.point);
            out.point[0] = _otherResult_tet.point[0];
            out.point[1] = _otherResult_tet.point[1];
            out.point[2] = _otherResult_tet.point[2];
            out.pointSet = (_otherResult_tet.pointSet & 0b0001) + ((_otherResult_tet.pointSet & 0b0110) << 1);
        }
    }

    // face adb
    if (_originOutOfPlanes.z) {
        _otherResult_tet.pointSet = 0;
        // vec3.zero(_otherResult_tet.point);
        _otherResult_tet.point[0] = 0;
        _otherResult_tet.point[1] = 0;
        _otherResult_tet.point[2] = 0;
        computeClosestPointOnTriangle(_otherResult_tet, inA, inB, inD, mustIncludeD, squaredTolerance);
        // const distanceSquared = vec3.squaredLength(_otherResult_tet.point);
        const distanceSquared =
            _otherResult_tet.point[0] * _otherResult_tet.point[0] +
            _otherResult_tet.point[1] * _otherResult_tet.point[1] +
            _otherResult_tet.point[2] * _otherResult_tet.point[2];
        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            // vec3.copy(out.point, _otherResult_tet.point);
            out.point[0] = _otherResult_tet.point[0];
            out.point[1] = _otherResult_tet.point[1];
            out.point[2] = _otherResult_tet.point[2];
            out.pointSet = (_otherResult_tet.pointSet & 0b0011) + ((_otherResult_tet.pointSet & 0b0100) << 1);
        }
    }

    // face bdc
    if (_originOutOfPlanes.w) {
        _otherResult_tet.pointSet = 0;
        // vec3.zero(_otherResult_tet.point);
        _otherResult_tet.point[0] = 0;
        _otherResult_tet.point[1] = 0;
        _otherResult_tet.point[2] = 0;
        computeClosestPointOnTriangle(_otherResult_tet, inB, inC, inD, mustIncludeD, squaredTolerance);
        // const distanceSquared = vec3.squaredLength(_otherResult_tet.point);
        const distanceSquared =
            _otherResult_tet.point[0] * _otherResult_tet.point[0] +
            _otherResult_tet.point[1] * _otherResult_tet.point[1] +
            _otherResult_tet.point[2] * _otherResult_tet.point[2];
        if (distanceSquared < bestDistanceSquared) {
            // vec3.copy(out.point, _otherResult_tet.point);
            out.point[0] = _otherResult_tet.point[0];
            out.point[1] = _otherResult_tet.point[1];
            out.point[2] = _otherResult_tet.point[2];
            out.pointSet = _otherResult_tet.pointSet << 1;
        }
    }
}

const GJK_TOLERANCE = 1e-5;
const GJK_MAX_ITERATIONS = 100;

const _p = vec3.create();
const _q = vec3.create();
const _w = vec3.create();
const _x = vec3.create();
const _v = vec3.create();
const _directionA = vec3.create();
const _directionB = vec3.create();
const _pq = vec3.create();
const _prevV = vec3.create();
const _normalizedV = vec3.create();
const _simplex = createSimplex();
const _bary = createBarycentricCoordinatesResult();
const _closestPoint = createClosestPointResult();
const _closestPointToSimplex = createClosestPointToSimplexResult();
const _transformedSupportA = createTransformedSupport();

type ClosestPointToSimplexResult = {
    point: Vec3;
    squaredDistance: number;
    pointSet: number;
    closestPointFound: boolean;
};

function createClosestPointToSimplexResult(): ClosestPointToSimplexResult {
    return {
        point: vec3.create(),
        squaredDistance: 0,
        pointSet: 0,
        closestPointFound: false,
    };
}

function computeClosestPointToSimplex(
    result: ClosestPointToSimplexResult,
    prevSquaredDist: number,
    lastPointPartOfClosest: boolean,
    simplex: Simplex,
): boolean {
    switch (simplex.size) {
        case 1: {
            // single point
            _closestPoint.pointSet = 0b0001;
            // vec3.copy(_closestPoint.point, simplex.points[0].y);
            const y = simplex.points[0].y;
            const point = _closestPoint.point;
            point[0] = y[0];
            point[1] = y[1];
            point[2] = y[2];
            break;
        }

        case 2: {
            // line segment
            computeClosestPointOnLine(_closestPoint, simplex.points[0].y, simplex.points[1].y, 1e-10);
            break;
        }

        case 3: {
            // triangle
            computeClosestPointOnTriangle(
                _closestPoint,
                simplex.points[0].y,
                simplex.points[1].y,
                simplex.points[2].y,
                lastPointPartOfClosest,
                1e-10,
            );
            break;
        }

        case 4: {
            // tetrahedron
            computeClosestPointOnTetrahedron(
                _closestPoint,
                simplex.points[0].y,
                simplex.points[1].y,
                simplex.points[2].y,
                simplex.points[3].y,
                lastPointPartOfClosest,
                1e-5,
            );
            break;
        }

        default: {
            throw new Error('Invalid number of points in simplex');
        }
    }

    const squaredDistance = vec3.squaredLength(_closestPoint.point);

    // check if we found a closer point
    if (squaredDistance < prevSquaredDist) {
        vec3.copy(result.point, _closestPoint.point);
        result.squaredDistance = squaredDistance;
        result.pointSet = _closestPoint.pointSet;
        result.closestPointFound = true;
        return true;
    }

    // no better match found
    result.closestPointFound = false;
    return false;
}

export type GjkCastRayResult = {
    isHitFound: boolean;
    lambda: number;
    simplex: Simplex;
};

export function createGjkCastRayResult(): GjkCastRayResult {
    return {
        isHitFound: false,
        lambda: 0,
        simplex: createSimplex(),
    };
}

/**
 * Cast a ray against a convex shape using GJK.
 *
 * @param out output result object
 * @param rayOrigin the starting point of the ray
 * @param rayDirection the direction of the ray
 * @param tolerance convergence tolerance
 * @param support support function for the shape
 * @param maxLambda maximum lambda to check (default 1.0). Result lambda will not exceed this.
 */
export function gjkCastRay(
    out: GjkCastRayResult,
    rayOrigin: Vec3,
    rayDirection: Vec3,
    tolerance: number,
    support: Support,
    maxLambda: number = 1.0,
): void {
    const squaredTolerance = tolerance * tolerance;

    // reset state
    _simplex.size = 0;

    let lambda = 0.0;
    // vec3.copy(_x, rayOrigin);
    _x[0] = rayOrigin[0];
    _x[1] = rayOrigin[1];
    _x[2] = rayOrigin[2];

    // v = x - support(0)
    // vec3.zero(_directionA);
    _directionA[0] = 0;
    _directionA[1] = 0;
    _directionA[2] = 0;
    support.getSupport(_directionA, _p);

    // vec3.subtract(_v, _x, _p);
    _v[0] = _x[0] - _p[0];
    _v[1] = _x[1] - _p[1];
    _v[2] = _x[2] - _p[2];

    let v_len_sq = Number.MAX_VALUE;
    let allowRestart = false;

    let iterations = 0;
    while (iterations < GJK_MAX_ITERATIONS) {
        iterations++;

        // get new support point
        support.getSupport(_v, _p);
        // vec3.subtract(_w, _x, _p);
        _w[0] = _x[0] - _p[0];
        _w[1] = _x[1] - _p[1];
        _w[2] = _x[2] - _p[2];

        // const vDotW = vec3.dot(_v, _w);
        const vDotW = _v[0] * _w[0] + _v[1] * _w[1] + _v[2] * _w[2];

        if (vDotW > 0.0) {
            // if ray and normal are in the same direction, we've passed A and there's no collision
            // const vDotR = vec3.dot(_v, rayDirection);
            const vDotR = _v[0] * rayDirection[0] + _v[1] * rayDirection[1] + _v[2] * rayDirection[2];

            // instead of checking >= 0, check with epsilon as we don't want the division below to overflow to infinity as it can cause a float exception
            if (vDotR >= -1.0e-18) {
                out.isHitFound = false;
                out.lambda = 0;
                return;
            }

            // update the lower bound for lambda
            const delta = vDotW / vDotR;
            const oldLambda = lambda;
            lambda -= delta;

            // if lambda didn't change, we cannot converge any further and we assume a hit
            if (oldLambda === lambda) {
                break;
            }

            // if lambda is bigger or equal than max, we don't have a hit
            if (lambda >= maxLambda) {
                out.isHitFound = false;
                out.lambda = 0;
                return;
            }

            // update x to new closest point on the ray
            // vec3.scaleAndAdd(_x, rayOrigin, rayDirection, lambda);
            _x[0] = rayOrigin[0] + rayDirection[0] * lambda;
            _x[1] = rayOrigin[1] + rayDirection[1] * lambda;
            _x[2] = rayOrigin[2] + rayDirection[2] * lambda;

            // we've shifted x, so reset v_len_sq so that it is not used as early out for GetClosest
            v_len_sq = Number.MAX_VALUE;

            // we allow rebuilding the simplex once after x changes because the simplex was built
            // for another x and numerical round off builds up as you keep adding points to an
            // existing simplex
            allowRestart = true;
        }

        // add p to set P: P = P U {p}
        // vec3.copy(_simplex.points[_simplex.size].p, _p);
        _simplex.points[_simplex.size].p[0] = _p[0];
        _simplex.points[_simplex.size].p[1] = _p[1];
        _simplex.points[_simplex.size].p[2] = _p[2];
        _simplex.size++;

        // calculate Y = {x} - P
        for (let i = 0; i < _simplex.size; i++) {
            // vec3.subtract(_simplex.points[i].y, _x, _simplex.points[i].p);
            _simplex.points[i].y[0] = _x[0] - _simplex.points[i].p[0];
            _simplex.points[i].y[1] = _x[1] - _simplex.points[i].p[1];
            _simplex.points[i].y[2] = _x[2] - _simplex.points[i].p[2];
        }

        // determine the new closest point from Y to origin
        const found = computeClosestPointToSimplex(_closestPointToSimplex, v_len_sq, false, _simplex);
        if (found) {
            v_len_sq = _closestPointToSimplex.squaredDistance;
            // vec3.copy(_v, _closestPointToSimplex.point);
            _v[0] = _closestPointToSimplex.point[0];
            _v[1] = _closestPointToSimplex.point[1];
            _v[2] = _closestPointToSimplex.point[2];
        }

        if (!found) {
            // only allow 1 restart, if we still can't get a closest point
            // we're so close that we return this as a hit
            if (!allowRestart) {
                break;
            }

            // if we fail to converge, we start again with the last point as simplex
            allowRestart = false;
            // vec3.copy(_simplex.points[0].p, _p);
            _simplex.points[0].p[0] = _p[0];
            _simplex.points[0].p[1] = _p[1];
            _simplex.points[0].p[2] = _p[2];
            _simplex.size = 1;
            // vec3.subtract(_v, _x, _p);
            _v[0] = _x[0] - _p[0];
            _v[1] = _x[1] - _p[1];
            _v[2] = _x[2] - _p[2];
            v_len_sq = Number.MAX_VALUE;
            continue;
        } else if (_closestPointToSimplex.pointSet === 0xf) {
            // we're inside the tetrahedron, we have a hit (verify that length of v is 0)
            break;
        }

        // update the points P to form the new simplex
        // note: we're not updating Y as Y will shift with x so we have to calculate it every iteration
        let newSize = 0;

        for (let i = 0; i < _simplex.size; i++) {
            if ((_closestPointToSimplex.pointSet & (1 << i)) !== 0) {
                if (newSize !== i) {
                    // copy point i to position newSize
                    // vec3.copy(simplex.points[newSize].p, simplex.points[i].p);
                    const pSrc = _simplex.points[i].p;
                    const pDst = _simplex.points[newSize].p;
                    pDst[0] = pSrc[0];
                    pDst[1] = pSrc[1];
                    pDst[2] = pSrc[2];
                }
                newSize++;
            }
        }

        _simplex.size = newSize;

        // check if x is close enough to A
        if (v_len_sq <= squaredTolerance) {
            break;
        }
    }

    // store hit fraction
    out.isHitFound = true;
    out.lambda = lambda;
    copySimplex(out.simplex, _simplex);
}

const updatePointSetPQ = (simplex: Simplex, inSet: number): void => {
    let newSize = 0;

    for (let i = 0; i < simplex.size; i++) {
        if ((inSet & (1 << i)) !== 0) {
            if (newSize !== i) {
                vec3.copy(simplex.points[newSize].p, simplex.points[i].p);
                vec3.copy(simplex.points[newSize].q, simplex.points[i].q);
            }
            newSize++;
        }
    }

    simplex.size = newSize;
};

export type GjkCastShapeResult = {
    hit: boolean;
    lambda: number;
    pointA: Vec3;
    pointB: Vec3;
    separatingAxis: Vec3;
    simplex: Simplex;
};

export function createGjkCastShapeResult(): GjkCastShapeResult {
    return {
        hit: false,
        lambda: 0,
        pointA: vec3.create(),
        pointB: vec3.create(),
        separatingAxis: vec3.create(),
        simplex: createSimplex(),
    };
}

/**
 * Cast a convex shape against another convex shape using GJK.
 * Shape A is moving in direction `displacement`.
 * Shape B is stationary.
 *
 * Creates transform wrapper internally around shape A.
 *
 * @param out output result object
 * @param posAInB position of shape A in shape B's local space
 * @param quatAInB rotation of shape A in shape B's local space
 * @param shapeASupport support function for shape A (WITHOUT position/rotation transform)
 * @param shapeBSupport support function for shape B
 * @param displacement direction and distance to move shape A
 * @param tolerance convergence tolerance for GJK
 * @param convexRadiusA convex radius of shape A
 * @param convexRadiusB convex radius of shape B
 * @param maxLambda the max fraction along the sweep
 */
export function gjkCastShape(
    out: GjkCastShapeResult,
    posAInB: Vec3,
    quatAInB: Quat,
    shapeASupport: Support,
    shapeBSupport: Support,
    displacement: Vec3,
    tolerance: number,
    convexRadiusA: number,
    convexRadiusB: number,
    maxLambda: number,
): void {
    // calculate how close A and B (without their convex radius) need to be to each other in order for us to consider this a collision
    let squaredTolerance = tolerance * tolerance;
    const sumConvexRadius = convexRadiusA + convexRadiusB;

    // wrap shapeA with transform
    setTransformedSupport(_transformedSupportA, posAInB, quatAInB, shapeASupport);

    // reset state
    _simplex.size = 0;

    let lambda = 0.0;

    // since A is already transformed we can start the cast from zero
    // vec3.zero(_x);
    _x[0] = 0;
    _x[1] = 0;
    _x[2] = 0;

    // v = -support_B + support_A (Minkowski difference B - A in the space of A)
    // vec3.zero(_directionB);
    _directionB[0] = 0;
    _directionB[1] = 0;
    _directionB[2] = 0;
    shapeBSupport.getSupport(_directionB, _q);
    // vec3.negate(_q, _q);
    _q[0] = -_q[0];
    _q[1] = -_q[1];
    _q[2] = -_q[2];

    // vec3.zero(_directionA);
    _directionA[0] = 0;
    _directionA[1] = 0;
    _directionA[2] = 0;
    _transformedSupportA.getSupport(_directionA, _p);

    // vec3.subtract(_v, _q, _p);
    _v[0] = _q[0] - _p[0];
    _v[1] = _q[1] - _p[1];
    _v[2] = _q[2] - _p[2];
    let vLenSq = Number.MAX_VALUE;
    let allowRestart = false;

    // keeps track of separating axis of the previous iteration.
    // initialized at zero as we don't know if our first v is actually a separating axis.
    // vec3.zero(_prevV);
    _prevV[0] = 0;
    _prevV[1] = 0;
    _prevV[2] = 0;

    let iterations = 0;
    while (iterations < GJK_MAX_ITERATIONS) {
        iterations++;

        // calculate the minkowski difference B - A
        // A is moving, so we need to add the back side of B to the front side of A
        // keep the support points on A and B separate so that in the end we can calculate a contact point
        // vec3.negate(_directionA, _v);
        _directionA[0] = -_v[0];
        _directionA[1] = -_v[1];
        _directionA[2] = -_v[2];
        _transformedSupportA.getSupport(_directionA, _p);

        // vec3.copy(_directionB, _v);
        _directionB[0] = _v[0];
        _directionB[1] = _v[1];
        _directionB[2] = _v[2];
        shapeBSupport.getSupport(_directionB, _q);

        // vec3.subtract(_pq, _q, _p);
        _pq[0] = _q[0] - _p[0];
        _pq[1] = _q[1] - _p[1];
        _pq[2] = _q[2] - _p[2];

        // vec3.subtract(_w, _x, _pq);
        _w[0] = _x[0] - _pq[0];
        _w[1] = _x[1] - _pq[1];
        _w[2] = _x[2] - _pq[2];

        // difference from article to this code:
        //
        // we did not include the convex radius in p and q in order to be able to calculate a good separating axis at the end of the algorithm.
        // however when moving forward along displacement we do need to take this into account so that we keep A and B separated by the sum of their convex radii.
        //
        // from p we have to subtract: convexRadiusA * v / |v|
        // to q we have to add: convexRadiusB * v / |v|
        // this means that to w we have to add: -(convexRadiusA + convexRadiusB) * v / |v|
        // so to v . w we have to add: v . (-(convexRadiusA + convexRadiusB) * v / |v|) = -(convexRadiusA + convexRadiusB) * |v|
        // const vDotW = vec3.dot(_v, _w) - sumConvexRadius * vec3.length(_v);
        const vDotW =
            _v[0] * _w[0] +
            _v[1] * _w[1] +
            _v[2] * _w[2] -
            sumConvexRadius * Math.sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);

        if (vDotW > 0.0) {
            // if ray and normal are in the same direction, we've passed A and there's no collision
            // const vDotR = vec3.dot(_v, displacement);
            const vDotR = _v[0] * displacement[0] + _v[1] * displacement[1] + _v[2] * displacement[2];

            // instead of checking >= 0, check with epsilon as we don't want the division below to overflow to infinity as it can cause a float exception
            if (vDotR >= -1.0e-18) {
                out.hit = false;
                return;
            }

            // update the lower bound for lambda
            const delta = vDotW / vDotR;
            const oldLambda = lambda;
            lambda -= delta;

            // if lambda didn't change, we cannot converge any further and we assume a hit
            if (oldLambda === lambda) {
                break;
            }

            // if lambda is bigger or equal than max, we don't have a hit
            if (lambda >= maxLambda) {
                out.hit = false;
                return;
            }

            // update x to new closest point on the ray
            // vec3.scale(_x, displacement, lambda);
            _x[0] = displacement[0] * lambda;
            _x[1] = displacement[1] * lambda;
            _x[2] = displacement[2] * lambda;

            // we've shifted x, so reset v_len_sq so that it is not used as early out when GetClosest returns false
            vLenSq = Number.MAX_VALUE;

            // now that we've moved, we know that A and B are not intersecting at lambda = 0, so we can update our tolerance to stop iterating
            // as soon as A and B are convexRadiusA + convexRadiusB apart
            squaredTolerance = tolerance + sumConvexRadius;
            squaredTolerance = squaredTolerance * squaredTolerance;

            // we allow rebuilding the simplex once after x changes because the simplex was built for another x and numerical round off builds
            // up as you keep adding points to an existing simplex
            allowRestart = true;
        }

        // add p to set P, q to set Q: P = P U {p}, Q = Q U {q}
        // vec3.copy(_simplex.points[_simplex.size].p, _p);
        _simplex.points[_simplex.size].p[0] = _p[0];
        _simplex.points[_simplex.size].p[1] = _p[1];
        _simplex.points[_simplex.size].p[2] = _p[2];

        // vec3.copy(_simplex.points[_simplex.size].q, _q);
        _simplex.points[_simplex.size].q[0] = _q[0];
        _simplex.points[_simplex.size].q[1] = _q[1];
        _simplex.points[_simplex.size].q[2] = _q[2];
        _simplex.size++;

        // calculate Y = {x} - (Q - P)
        for (let i = 0; i < _simplex.size; i++) {
            // vec3.subtract(_simplex.points[i].y, _x, _simplex.points[i].q);
            _simplex.points[i].y[0] = _x[0] - _simplex.points[i].q[0];
            _simplex.points[i].y[1] = _x[1] - _simplex.points[i].q[1];
            _simplex.points[i].y[2] = _x[2] - _simplex.points[i].q[2];

            // vec3.add(_simplex.points[i].y, _simplex.points[i].y, _simplex.points[i].p);
            _simplex.points[i].y[0] += _simplex.points[i].p[0];
            _simplex.points[i].y[1] += _simplex.points[i].p[1];
            _simplex.points[i].y[2] += _simplex.points[i].p[2];
        }

        // determine the new closest point from Y to origin
        const found = computeClosestPointToSimplex(_closestPointToSimplex, vLenSq, false, _simplex);

        if (found) {
            vLenSq = _closestPointToSimplex.squaredDistance;
            // vec3.copy(_v, _closestPointToSimplex.point);
            _v[0] = _closestPointToSimplex.point[0];
            _v[1] = _closestPointToSimplex.point[1];
            _v[2] = _closestPointToSimplex.point[2];
        }

        if (!found) {
            // only allow 1 restart, if we still can't get a closest point we're so close that we return this as a hit
            if (!allowRestart) {
                break;
            }

            // if we fail to converge, we start again with the last point as simplex
            allowRestart = false;
            // vec3.copy(_simplex.points[0].p, _p);
            _simplex.points[0].p[0] = _p[0];
            _simplex.points[0].p[1] = _p[1];
            _simplex.points[0].p[2] = _p[2];

            // vec3.copy(_simplex.points[0].q, _q);
            _simplex.points[0].q[0] = _q[0];
            _simplex.points[0].q[1] = _q[1];
            _simplex.points[0].q[2] = _q[2];
            _simplex.size = 1;

            // vec3.subtract(_v, _x, _q);
            _v[0] = _x[0] - _q[0];
            _v[1] = _x[1] - _q[1];
            _v[2] = _x[2] - _q[2];
            vLenSq = Number.MAX_VALUE;
            continue;
        } else if (_closestPointToSimplex.pointSet === 0xf) {
            // we're inside the tetrahedron, we have a hit (verify that length of v is 0)
            break;
        }

        // update the points P and Q to form the new simplex
        // note: we're not updating Y as Y will shift with x so we have to calculate it every iteration
        updatePointSetPQ(_simplex, _closestPointToSimplex.pointSet);

        // check if A and B are touching according to our tolerance
        if (vLenSq <= squaredTolerance) {
            break;
        }

        // store our v to return as separating axis
        // vec3.copy(_prevV, _v);
        _prevV[0] = _v[0];
        _prevV[1] = _v[1];
        _prevV[2] = _v[2];
    }

    // calculate Y = {x} - (Q - P) again so we can calculate the contact points
    for (let i = 0; i < _simplex.size; i++) {
        // vec3.subtract(_simplex.points[i].y, _x, _simplex.points[i].q);
        _simplex.points[i].y[0] = _x[0] - _simplex.points[i].q[0];
        _simplex.points[i].y[1] = _x[1] - _simplex.points[i].q[1];
        _simplex.points[i].y[2] = _x[2] - _simplex.points[i].q[2];

        // vec3.add(_simplex.points[i].y, _simplex.points[i].y, _simplex.points[i].p);
        _simplex.points[i].y[0] += _simplex.points[i].p[0];
        _simplex.points[i].y[1] += _simplex.points[i].p[1];
        _simplex.points[i].y[2] += _simplex.points[i].p[2];
    }

    // compute normalized v for separating axis
    // const vLen = vec3.length(_v);
    const vLen = Math.sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    if (vLen > 0) {
        // vec3.scale(_normalizedV, _v, 1 / vLen);
        const invVLen = 1 / vLen;
        _normalizedV[0] = _v[0] * invVLen;
        _normalizedV[1] = _v[1] * invVLen;
        _normalizedV[2] = _v[2] * invVLen;
    } else {
        // vec3.zero(_normalizedV);
        _normalizedV[0] = 0;
        _normalizedV[1] = 0;
        _normalizedV[2] = 0;
    }

    // compute contact points from simplex
    // vec3.zero(out.pointA);
    out.pointA[0] = 0;
    out.pointA[1] = 0;
    out.pointA[2] = 0;

    // vec3.zero(out.pointB);
    out.pointB[0] = 0;
    out.pointB[1] = 0;
    out.pointB[2] = 0;

    switch (_simplex.size) {
        case 1: {
            // vec3.scaleAndAdd(out.pointB, _simplex.points[0].q, _normalizedV, convexRadiusB);
            out.pointB[0] = _simplex.points[0].q[0] + _normalizedV[0] * convexRadiusB;
            out.pointB[1] = _simplex.points[0].q[1] + _normalizedV[1] * convexRadiusB;
            out.pointB[2] = _simplex.points[0].q[2] + _normalizedV[2] * convexRadiusB;

            if (lambda > 0.0) {
                // vec3.copy(out.pointA, out.pointB);
                out.pointA[0] = out.pointB[0];
                out.pointA[1] = out.pointB[1];
                out.pointA[2] = out.pointB[2];
            } else {
                // vec3.scaleAndAdd(out.pointA, _simplex.points[0].p, _normalizedV, -convexRadiusA);
                out.pointA[0] = _simplex.points[0].p[0] + _normalizedV[0] * -convexRadiusA;
                out.pointA[1] = _simplex.points[0].p[1] + _normalizedV[1] * -convexRadiusA;
                out.pointA[2] = _simplex.points[0].p[2] + _normalizedV[2] * -convexRadiusA;
            }
            break;
        }
        case 2: {
            computeBarycentricCoordinates2d(_bary, _simplex.points[0].y, _simplex.points[1].y, 1e-10);

            // vec3.scaleAndAdd(out.pointB, out.pointB, _simplex.points[0].q, _bary.u);
            out.pointB[0] += _simplex.points[0].q[0] * _bary.u;
            out.pointB[1] += _simplex.points[0].q[1] * _bary.u;
            out.pointB[2] += _simplex.points[0].q[2] * _bary.u;

            // vec3.scaleAndAdd(out.pointB, out.pointB, _simplex.points[1].q, _bary.v);
            out.pointB[0] += _simplex.points[1].q[0] * _bary.v;
            out.pointB[1] += _simplex.points[1].q[1] * _bary.v;
            out.pointB[2] += _simplex.points[1].q[2] * _bary.v;

            // vec3.scaleAndAdd(out.pointB, out.pointB, _normalizedV, convexRadiusB);
            out.pointB[0] += _normalizedV[0] * convexRadiusB;
            out.pointB[1] += _normalizedV[1] * convexRadiusB;
            out.pointB[2] += _normalizedV[2] * convexRadiusB;

            if (lambda > 0.0) {
                // vec3.copy(out.pointA, out.pointB);
                out.pointA[0] = out.pointB[0];
                out.pointA[1] = out.pointB[1];
                out.pointA[2] = out.pointB[2];
            } else {
                // vec3.scaleAndAdd(out.pointA, out.pointA, _simplex.points[0].p, _bary.u);
                out.pointA[0] += _simplex.points[0].p[0] * _bary.u;
                out.pointA[1] += _simplex.points[0].p[1] * _bary.u;
                out.pointA[2] += _simplex.points[0].p[2] * _bary.u;

                // vec3.scaleAndAdd(out.pointA, out.pointA, _simplex.points[1].p, _bary.v);
                out.pointA[0] += _simplex.points[1].p[0] * _bary.v;
                out.pointA[1] += _simplex.points[1].p[1] * _bary.v;
                out.pointA[2] += _simplex.points[1].p[2] * _bary.v;

                // vec3.scaleAndAdd(out.pointA, out.pointA, _normalizedV, -convexRadiusA);
                out.pointA[0] += _normalizedV[0] * -convexRadiusA;
                out.pointA[1] += _normalizedV[1] * -convexRadiusA;
                out.pointA[2] += _normalizedV[2] * -convexRadiusA;
            }
            break;
        }
        case 3:
        case 4: {
            computeBarycentricCoordinates3d(_bary, _simplex.points[0].y, _simplex.points[1].y, _simplex.points[2].y, 1e-10);

            // vec3.scaleAndAdd(out.pointB, out.pointB, _simplex.points[0].q, _bary.u);
            out.pointB[0] += _simplex.points[0].q[0] * _bary.u;
            out.pointB[1] += _simplex.points[0].q[1] * _bary.u;
            out.pointB[2] += _simplex.points[0].q[2] * _bary.u;

            // vec3.scaleAndAdd(out.pointB, out.pointB, _simplex.points[1].q, _bary.v);
            out.pointB[0] += _simplex.points[1].q[0] * _bary.v;
            out.pointB[1] += _simplex.points[1].q[1] * _bary.v;
            out.pointB[2] += _simplex.points[1].q[2] * _bary.v;

            // vec3.scaleAndAdd(out.pointB, out.pointB, _simplex.points[2].q, _bary.w);
            out.pointB[0] += _simplex.points[2].q[0] * _bary.w;
            out.pointB[1] += _simplex.points[2].q[1] * _bary.w;
            out.pointB[2] += _simplex.points[2].q[2] * _bary.w;

            // vec3.scaleAndAdd(out.pointB, out.pointB, _normalizedV, convexRadiusB);
            out.pointB[0] += _normalizedV[0] * convexRadiusB;
            out.pointB[1] += _normalizedV[1] * convexRadiusB;
            out.pointB[2] += _normalizedV[2] * convexRadiusB;

            if (lambda > 0.0) {
                // vec3.copy(out.pointA, out.pointB);
                out.pointA[0] = out.pointB[0];
                out.pointA[1] = out.pointB[1];
                out.pointA[2] = out.pointB[2];
            } else {
                // vec3.scaleAndAdd(out.pointA, out.pointA, _simplex.points[0].p, _bary.u);
                out.pointA[0] += _simplex.points[0].p[0] * _bary.u;
                out.pointA[1] += _simplex.points[0].p[1] * _bary.u;
                out.pointA[2] += _simplex.points[0].p[2] * _bary.u;

                // vec3.scaleAndAdd(out.pointA, out.pointA, _simplex.points[1].p, _bary.v);
                out.pointA[0] += _simplex.points[1].p[0] * _bary.v;
                out.pointA[1] += _simplex.points[1].p[1] * _bary.v;
                out.pointA[2] += _simplex.points[1].p[2] * _bary.v;

                // vec3.scaleAndAdd(out.pointA, out.pointA, _simplex.points[2].p, _bary.w);
                out.pointA[0] += _simplex.points[2].p[0] * _bary.w;
                out.pointA[1] += _simplex.points[2].p[1] * _bary.w;
                out.pointA[2] += _simplex.points[2].p[2] * _bary.w;

                // vec3.scaleAndAdd(out.pointA, out.pointA, _normalizedV, -convexRadiusA);
                out.pointA[0] += _normalizedV[0] * -convexRadiusA;
                out.pointA[1] += _normalizedV[1] * -convexRadiusA;
                out.pointA[2] += _normalizedV[2] * -convexRadiusA;
            }
            break;
        }
    }

    // store results
    out.lambda = lambda;
    out.hit = true;

    // use current v if we have convex radius, otherwise use previous v as approximation
    // (when there's no convex radius, the current v might be inaccurate due to numerical rounding)
    if (sumConvexRadius > 0.0) {
        // vec3.negate(out.separatingAxis, _v);
        out.separatingAxis[0] = -_v[0];
        out.separatingAxis[1] = -_v[1];
        out.separatingAxis[2] = -_v[2];
    } else {
        // vec3.negate(out.separatingAxis, _prevV);
        out.separatingAxis[0] = -_prevV[0];
        out.separatingAxis[1] = -_prevV[1];
        out.separatingAxis[2] = -_prevV[2];
    }
}

export type GjkClosestPoints = {
    squaredDistance: number;
    penetrationAxis: Vec3;
    pointA: Vec3;
    pointB: Vec3;
    simplex: Simplex;
};

export function createGjkClosestPoints(): GjkClosestPoints {
    return {
        squaredDistance: 0,
        penetrationAxis: vec3.create(),
        pointA: vec3.create(),
        pointB: vec3.create(),
        simplex: createSimplex(),
    };
}

const _closestPointsInSimplex_diff = vec3.create();

/**
 * Get closest points between two convex shapes using GJK.
 *
 * @param out output object containing pointA, pointB, squaredDistance, penetrationAxis, and simplex.
 *            On output:
 *            - squaredDistance = 0: shapes are colliding
 *            - squaredDistance > 0 && < Number.MAX_VALUE: shapes separated, penetrationAxis is separating axis
 *            - squaredDistance = Number.MAX_VALUE: shapes far apart (exceeded maxDistanceSquared)
 * @param supportA pre-configured support function for shape A
 * @param supportB pre-configured support function for shape B
 * @param tolerance minimal distance between A and B before the objects are considered colliding
 * @param direction initial guess for the separating axis
 * @param maxDistanceSquared maximum squared distance between A and B before objects are considered infinitely far away.
 *                           If exceeded, out.squaredDistance will be set to Number.MAX_VALUE
 */
export function gjkClosestPoints(
    out: GjkClosestPoints,
    supportA: Support,
    supportB: Support,
    tolerance: number,
    direction: Vec3,
    maxDistanceSquared: number,
): void {
    const squaredTolerance = tolerance * tolerance;

    // reset state
    _simplex.size = 0;

    // length^2 of v
    // vec3.copy(_closestPointToSimplex.point, direction);
    _closestPointToSimplex.point[0] = direction[0];
    _closestPointToSimplex.point[1] = direction[1];
    _closestPointToSimplex.point[2] = direction[2];
    // _closestPointToSimplex.squaredDistance = vec3.squaredLength(direction);
    _closestPointToSimplex.squaredDistance =
        direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
    _closestPointToSimplex.pointSet = 0;
    _closestPointToSimplex.closestPointFound = true;
    // previous length^2 of v
    let previousSquaredDistance = Number.MAX_VALUE;

    let iterations = 0;
    while (iterations++ < GJK_MAX_ITERATIONS) {
        // get support points for shape A and B in the direction of v
        // vec3.copy(_directionA, _closestPointToSimplex.point);
        _directionA[0] = _closestPointToSimplex.point[0];
        _directionA[1] = _closestPointToSimplex.point[1];
        _directionA[2] = _closestPointToSimplex.point[2];
        // vec3.negate(_directionB, _closestPointToSimplex.point);
        _directionB[0] = -_closestPointToSimplex.point[0];
        _directionB[1] = -_closestPointToSimplex.point[1];
        _directionB[2] = -_closestPointToSimplex.point[2];

        supportA.getSupport(_directionA, _p);
        supportB.getSupport(_directionB, _q);

        // get support point of the minkowski sum A - B of v
        // vec3.subtract(_w, _p, _q);
        _w[0] = _p[0] - _q[0];
        _w[1] = _p[1] - _q[1];
        _w[2] = _p[2] - _q[2];

        // const dot = vec3.dot(_closestPointToSimplex.point, _w);
        const dot =
            _closestPointToSimplex.point[0] * _w[0] +
            _closestPointToSimplex.point[1] * _w[1] +
            _closestPointToSimplex.point[2] * _w[2];

        // test if we have a separation of more than inMaxDistSq, in which case we terminate early
        if (dot < 0.0 && dot * dot > _closestPointToSimplex.squaredDistance * maxDistanceSquared) {
            out.squaredDistance = Number.MAX_VALUE;
            // vec3.copy(out.penetrationAxis, _closestPointToSimplex.point);
            out.penetrationAxis[0] = _closestPointToSimplex.point[0];
            out.penetrationAxis[1] = _closestPointToSimplex.point[1];
            out.penetrationAxis[2] = _closestPointToSimplex.point[2];
            return;
        }

        // store the point for later use
        // vec3.copy(_simplex.points[_simplex.size].y, _w);
        const y = _simplex.points[_simplex.size].y;
        y[0] = _w[0];
        y[1] = _w[1];
        y[2] = _w[2];
        // vec3.copy(_simplex.points[_simplex.size].p, _p);
        const p = _simplex.points[_simplex.size].p;
        p[0] = _p[0];
        p[1] = _p[1];
        p[2] = _p[2];
        // vec3.copy(_simplex.points[_simplex.size].q, _q);
        const q = _simplex.points[_simplex.size].q;
        q[0] = _q[0];
        q[1] = _q[1];
        q[2] = _q[2];
        _simplex.size++;

        computeClosestPointToSimplex(_closestPointToSimplex, previousSquaredDistance, true, _simplex);

        if (!_closestPointToSimplex.closestPointFound) {
            // remove last added point from simplex
            _simplex.size--;
            break;
        }

        // if there are 4 points, the origin is inside the tetrahedron and we're done
        if (_closestPointToSimplex.pointSet === 0xf) {
            // vec3.zero(_closestPointToSimplex.point);
            _closestPointToSimplex.point[0] = 0;
            _closestPointToSimplex.point[1] = 0;
            _closestPointToSimplex.point[2] = 0;
            _closestPointToSimplex.squaredDistance = 0.0;
            break;
        }

        // update the points of the simplex
        let newSize = 0;

        for (let i = 0; i < _simplex.size; i++) {
            if ((_closestPointToSimplex.pointSet & (1 << i)) !== 0) {
                if (newSize !== i) {
                    // copy point i to position newSize
                    const { y: srcY, p: srcP, q: srcQ } = _simplex.points[i];
                    const { y: dstY, p: dstP, q: dstQ } = _simplex.points[newSize];
                    // vec3.copy(simplex.points[newSize].y, simplex.points[i].y);
                    dstY[0] = srcY[0];
                    dstY[1] = srcY[1];
                    dstY[2] = srcY[2];
                    // vec3.copy(simplex.points[newSize].p, simplex.points[i].p);
                    dstP[0] = srcP[0];
                    dstP[1] = srcP[1];
                    dstP[2] = srcP[2];
                    // vec3.copy(simplex.points[newSize].q, simplex.points[i].q);
                    dstQ[0] = srcQ[0];
                    dstQ[1] = srcQ[1];
                    dstQ[2] = srcQ[2];
                }
                newSize++;
            }
        }

        _simplex.size = newSize;

        // if v is very close to zero, we consider this a collision
        if (_closestPointToSimplex.squaredDistance <= squaredTolerance) {
            // vec3.zero(_closestPointToSimplex.point);
            _closestPointToSimplex.point[0] = 0;
            _closestPointToSimplex.point[1] = 0;
            _closestPointToSimplex.point[2] = 0;
            _closestPointToSimplex.squaredDistance = 0.0;
            break;
        }

        // if v is very small compared to the length of y, we also consider this a collision
        let yMaxLengthSquared = 0;
        for (let i = 0; i < _simplex.size; i++) {
            const y = _simplex.points[i].y;

            // const squaredLength = vec3.squaredLength(y);
            const squaredLength = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];

            yMaxLengthSquared = Math.max(yMaxLengthSquared, squaredLength);
        }

        if (_closestPointToSimplex.squaredDistance <= GJK_TOLERANCE * yMaxLengthSquared) {
            // vec3.zero(_closestPointToSimplex.point);
            _closestPointToSimplex.point[0] = 0;
            _closestPointToSimplex.point[1] = 0;
            _closestPointToSimplex.point[2] = 0;
            _closestPointToSimplex.squaredDistance = 0.0;
            break;
        }

        // the next separation axis to test is the negative of the closest point of the Minkowski sum to the origin
        // note: this must be done before terminating as converged since the separating axis is -v
        // vec3.negate(_closestPointToSimplex.point, _closestPointToSimplex.point);
        _closestPointToSimplex.point[0] = -_closestPointToSimplex.point[0];
        _closestPointToSimplex.point[1] = -_closestPointToSimplex.point[1];
        _closestPointToSimplex.point[2] = -_closestPointToSimplex.point[2];

        // if the squared length of v is not changing enough, we've converged and there is no collision
        if (previousSquaredDistance - _closestPointToSimplex.squaredDistance <= GJK_TOLERANCE * previousSquaredDistance) {
            // v is a separating axis
            break;
        }

        previousSquaredDistance = _closestPointToSimplex.squaredDistance;
    }

    // extract the closest points
    // vec3.zero(out.pointA);
    out.pointA[0] = 0;
    out.pointA[1] = 0;
    out.pointA[2] = 0;

    // vec3.zero(out.pointB);
    out.pointB[0] = 0;
    out.pointB[1] = 0;
    out.pointB[2] = 0;

    copySimplex(out.simplex, _simplex);

    // handle early termination case: if simplex is empty, GJK terminated early (shapes are far apart)
    // return a large distance to indicate separation
    if (_simplex.size === 0) {
        out.squaredDistance = Number.MAX_VALUE;
        // vec3.zero(out.penetrationAxis);
        out.penetrationAxis[0] = 0;
        out.penetrationAxis[1] = 0;
        out.penetrationAxis[2] = 0;
        return;
    }

    switch (_simplex.size) {
        case 1: {
            // single point in simplex
            // vec3.copy(out.pointA, simplex.points[0].p);
            out.pointA[0] = _simplex.points[0].p[0];
            out.pointA[1] = _simplex.points[0].p[1];
            out.pointA[2] = _simplex.points[0].p[2];

            // vec3.copy(out.pointB, simplex.points[0].q);
            out.pointB[0] = _simplex.points[0].q[0];
            out.pointB[1] = _simplex.points[0].q[1];
            out.pointB[2] = _simplex.points[0].q[2];
            break;
        }

        case 2: {
            // line segment in simplex
            computeBarycentricCoordinates2d(_bary, _simplex.points[0].y, _simplex.points[1].y, 1e-10);

            // vec3.scaleAndAdd(out.pointA, out.pointA, simplex.points[0].p, _bary.u);
            out.pointA[0] += _simplex.points[0].p[0] * _bary.u;
            out.pointA[1] += _simplex.points[0].p[1] * _bary.u;
            out.pointA[2] += _simplex.points[0].p[2] * _bary.u;

            // vec3.scaleAndAdd(out.pointA, out.pointA, simplex.points[1].p, _bary.v);
            out.pointA[0] += _simplex.points[1].p[0] * _bary.v;
            out.pointA[1] += _simplex.points[1].p[1] * _bary.v;
            out.pointA[2] += _simplex.points[1].p[2] * _bary.v;

            // vec3.scaleAndAdd(out.pointB, out.pointB, simplex.points[0].q, _bary.u);
            out.pointB[0] += _simplex.points[0].q[0] * _bary.u;
            out.pointB[1] += _simplex.points[0].q[1] * _bary.u;
            out.pointB[2] += _simplex.points[0].q[2] * _bary.u;

            // vec3.scaleAndAdd(out.pointB, out.pointB, simplex.points[1].q, _bary.v);
            out.pointB[0] += _simplex.points[1].q[0] * _bary.v;
            out.pointB[1] += _simplex.points[1].q[1] * _bary.v;
            out.pointB[2] += _simplex.points[1].q[2] * _bary.v;
            break;
        }

        case 3: {
            // triangle in simplex
            computeBarycentricCoordinates3d(_bary, _simplex.points[0].y, _simplex.points[1].y, _simplex.points[2].y, 1e-10);

            // vec3.scaleAndAdd(out.pointA, out.pointA, simplex.points[0].p, _bary.u);
            out.pointA[0] += _simplex.points[0].p[0] * _bary.u;
            out.pointA[1] += _simplex.points[0].p[1] * _bary.u;
            out.pointA[2] += _simplex.points[0].p[2] * _bary.u;

            // vec3.scaleAndAdd(out.pointA, out.pointA, simplex.points[1].p, _bary.v);
            out.pointA[0] += _simplex.points[1].p[0] * _bary.v;
            out.pointA[1] += _simplex.points[1].p[1] * _bary.v;
            out.pointA[2] += _simplex.points[1].p[2] * _bary.v;

            // vec3.scaleAndAdd(out.pointA, out.pointA, simplex.points[2].p, _bary.w);
            out.pointA[0] += _simplex.points[2].p[0] * _bary.w;
            out.pointA[1] += _simplex.points[2].p[1] * _bary.w;
            out.pointA[2] += _simplex.points[2].p[2] * _bary.w;

            // vec3.scaleAndAdd(out.pointB, out.pointB, simplex.points[0].q, _bary.u);
            out.pointB[0] += _simplex.points[0].q[0] * _bary.u;
            out.pointB[1] += _simplex.points[0].q[1] * _bary.u;
            out.pointB[2] += _simplex.points[0].q[2] * _bary.u;

            // vec3.scaleAndAdd(out.pointB, out.pointB, simplex.points[1].q, _bary.v);
            out.pointB[0] += _simplex.points[1].q[0] * _bary.v;
            out.pointB[1] += _simplex.points[1].q[1] * _bary.v;
            out.pointB[2] += _simplex.points[1].q[2] * _bary.v;

            // vec3.scaleAndAdd(out.pointB, out.pointB, simplex.points[2].q, _bary.w);
            out.pointB[0] += _simplex.points[2].q[0] * _bary.w;
            out.pointB[1] += _simplex.points[2].q[1] * _bary.w;
            out.pointB[2] += _simplex.points[2].q[2] * _bary.w;
            break;
        }

        default: {
            // for a full simplex (4 points = tetrahedron), the origin is inside the Minkowski difference
            // in this case, pointA and pointB remain at zero (as initialized)
            break;
        }
    }

    // compute squared distance and penetration axis
    // penetration axis points from A to B (direction to push B out of collision with A)
    const diff = _closestPointsInSimplex_diff;
    // vec3.subtract(diff, out.pointB, out.pointA);
    diff[0] = out.pointB[0] - out.pointA[0];
    diff[1] = out.pointB[1] - out.pointA[1];
    diff[2] = out.pointB[2] - out.pointA[2];

    // out.squaredDistance = vec3.squaredLength(diff);
    out.squaredDistance = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];

    // vec3.copy(out.penetrationAxis, diff);
    out.penetrationAxis[0] = diff[0];
    out.penetrationAxis[1] = diff[1];
    out.penetrationAxis[2] = diff[2];

    // store the separating axis / penetration axis
    // vec3.copy(out.penetrationAxis, _closestPointToSimplex.point);
    out.penetrationAxis[0] = _closestPointToSimplex.point[0];
    out.penetrationAxis[1] = _closestPointToSimplex.point[1];
    out.penetrationAxis[2] = _closestPointToSimplex.point[2];
    out.squaredDistance = _closestPointToSimplex.squaredDistance;
}
