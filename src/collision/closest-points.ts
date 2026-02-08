import { type Vec3, vec3 } from 'mathcat';

export type BarycentricCoordinatesResult = {
    u: number;
    v: number;
    w: number;
    isValid: boolean;
};

export function createBarycentricCoordinatesResult(): BarycentricCoordinatesResult {
    return {
        u: 0,
        v: 0,
        w: 0,
        isValid: false,
    };
}

const _ab_2d = /* @__PURE__ */ vec3.create();

export function computeBarycentricCoordinates2d(
    out: BarycentricCoordinatesResult,
    a: Vec3,
    b: Vec3,
    squaredTolerance: number,
) {
    // vec3.subtract(_ab_2d, b, a);
    _ab_2d[0] = b[0] - a[0];
    _ab_2d[1] = b[1] - a[1];
    _ab_2d[2] = b[2] - a[2];

    // const denominator = vec3.squaredLength(_ab_2d);
    const denominator = _ab_2d[0] * _ab_2d[0] + _ab_2d[1] * _ab_2d[1] + _ab_2d[2] * _ab_2d[2];

    if (denominator < squaredTolerance) {
        // degenerate line segment, fallback to points
        // if (vec3.squaredLength(a) < vec3.squaredLength(b)) {
        if (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2]) {
            // A closest
            out.u = 1.0;
            out.v = 0.0;
        } else {
            // B closest
            out.u = 0.0;
            out.v = 1.0;
        }
        out.isValid = false;
        return;
    } else {
        // out.v = -vec3.dot(a, _ab_2d) / denominator;
        out.v = -(a[0] * _ab_2d[0] + a[1] * _ab_2d[1] + a[2] * _ab_2d[2]) / denominator;
        out.u = 1.0 - out.v;
    }
    out.isValid = true;
};

const _ab_3d = /* @__PURE__ */ vec3.create();
const _ac_3d = /* @__PURE__ */ vec3.create();
const _bc_3d = /* @__PURE__ */ vec3.create();
const _otherBarycentric = /* @__PURE__ */ createBarycentricCoordinatesResult();

export function computeBarycentricCoordinates3d(
    out: BarycentricCoordinatesResult,
    a: Vec3,
    b: Vec3,
    c: Vec3,
    squaredTolerance: number,
) {
    // vec3.subtract(_ab_3d, b, a);
    _ab_3d[0] = b[0] - a[0];
    _ab_3d[1] = b[1] - a[1];
    _ab_3d[2] = b[2] - a[2];

    // vec3.subtract(_ac_3d, c, a);
    _ac_3d[0] = c[0] - a[0];
    _ac_3d[1] = c[1] - a[1];
    _ac_3d[2] = c[2] - a[2];

    // vec3.subtract(_bc_3d, c, b);
    _bc_3d[0] = c[0] - b[0];
    _bc_3d[1] = c[1] - b[1];
    _bc_3d[2] = c[2] - b[2];

    // const d00 = vec3.dot(_ab_3d, _ab_3d);
    const d00 = _ab_3d[0] * _ab_3d[0] + _ab_3d[1] * _ab_3d[1] + _ab_3d[2] * _ab_3d[2];

    // const d11 = vec3.dot(_ac_3d, _ac_3d);
    const d11 = _ac_3d[0] * _ac_3d[0] + _ac_3d[1] * _ac_3d[1] + _ac_3d[2] * _ac_3d[2];

    // const d22 = vec3.dot(_bc_3d, _bc_3d);
    const d22 = _bc_3d[0] * _bc_3d[0] + _bc_3d[1] * _bc_3d[1] + _bc_3d[2] * _bc_3d[2];

    if (d00 <= d22) {
        // use v0 and v1 to calculate barycentric coordinates
        // const d01 = vec3.dot(_ab_3d, _ac_3d);
        const d01 = _ab_3d[0] * _ac_3d[0] + _ab_3d[1] * _ac_3d[1] + _ab_3d[2] * _ac_3d[2];

        const denominator = d00 * d11 - d01 * d01;
        if (Math.abs(denominator) < 1.0e-12) {
            // degenerate triangle, return coordinates along longest edge
            if (d00 > d11) {
                computeBarycentricCoordinates2d(_otherBarycentric, a, b, squaredTolerance);
                out.u = _otherBarycentric.u;
                out.v = _otherBarycentric.v;
                out.w = 0.0;
            } else {
                computeBarycentricCoordinates2d(_otherBarycentric, a, c, squaredTolerance);
                out.u = _otherBarycentric.u;
                out.w = _otherBarycentric.v;
                out.v = 0.0;
            }
            out.isValid = false;
            return;
        } else {
            // const a0 = vec3.dot(a, _ab_3d);
            const a0 = a[0] * _ab_3d[0] + a[1] * _ab_3d[1] + a[2] * _ab_3d[2];

            // const a1 = vec3.dot(a, _ac_3d);
            const a1 = a[0] * _ac_3d[0] + a[1] * _ac_3d[1] + a[2] * _ac_3d[2];

            out.v = (d01 * a1 - d11 * a0) / denominator;
            out.w = (d01 * a0 - d00 * a1) / denominator;
            out.u = 1.0 - out.v - out.w;
        }
    } else {
        // use v1 and v2 to calculate barycentric coordinates
        // const d12 = vec3.dot(_ac_3d, _bc_3d);
        const d12 = _ac_3d[0] * _bc_3d[0] + _ac_3d[1] * _bc_3d[1] + _ac_3d[2] * _bc_3d[2];

        const denominator = d11 * d22 - d12 * d12;
        if (Math.abs(denominator) < 1.0e-12) {
            // degenerate triangle, return coordinates along longest edge
            if (d11 > d22) {
                computeBarycentricCoordinates2d(_otherBarycentric, a, c, squaredTolerance);
                out.u = _otherBarycentric.u;
                out.w = _otherBarycentric.v;
                out.v = 0.0;
            } else {
                computeBarycentricCoordinates2d(_otherBarycentric, b, c, squaredTolerance);
                out.v = _otherBarycentric.u;
                out.w = _otherBarycentric.v;
                out.u = 0.0;
            }
            out.isValid = false;
            return;
        } else {
            // const c1 = vec3.dot(c, _ac_3d);
            const c1 = c[0] * _ac_3d[0] + c[1] * _ac_3d[1] + c[2] * _ac_3d[2];

            // const c2 = vec3.dot(c, _bc_3d);
            const c2 = c[0] * _bc_3d[0] + c[1] * _bc_3d[1] + c[2] * _bc_3d[2];

            out.u = (d22 * c1 - d12 * c2) / denominator;
            out.v = (d11 * c2 - d12 * c1) / denominator;
            out.w = 1.0 - out.u - out.v;
        }
    }
    out.isValid = true;
};


