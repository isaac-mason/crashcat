import type { Vec3 } from 'mathcat';

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

export function computeBarycentricCoordinates2d(
    out: BarycentricCoordinatesResult,
    a: Vec3,
    b: Vec3,
    squaredTolerance: number,
) {
    // ab = b - a
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const abz = b[2] - a[2];

    // denominator = dot(ab, ab)
    const denominator = abx * abx + aby * aby + abz * abz;

    if (denominator < squaredTolerance) {
        // degenerate line segment, fallback to points
        // compare dot(a, a) vs dot(b, b)
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
        // v = -dot(a, ab) / denominator
        out.v = -(a[0] * abx + a[1] * aby + a[2] * abz) / denominator;
        out.u = 1.0 - out.v;
    }
    out.isValid = true;
};

const _otherBarycentric = /* @__PURE__ */ createBarycentricCoordinatesResult();

export function computeBarycentricCoordinates3d(
    out: BarycentricCoordinatesResult,
    a: Vec3,
    b: Vec3,
    c: Vec3,
    squaredTolerance: number,
) {
    // ab = b - a
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const abz = b[2] - a[2];

    // ac = c - a
    const acx = c[0] - a[0];
    const acy = c[1] - a[1];
    const acz = c[2] - a[2];

    // bc = c - b
    const bcx = c[0] - b[0];
    const bcy = c[1] - b[1];
    const bcz = c[2] - b[2];

    // d00 = dot(ab, ab)
    const d00 = abx * abx + aby * aby + abz * abz;
    // d11 = dot(ac, ac)
    const d11 = acx * acx + acy * acy + acz * acz;
    // d22 = dot(bc, bc)
    const d22 = bcx * bcx + bcy * bcy + bcz * bcz;

    if (d00 <= d22) {
        // use v0 and v1 to calculate barycentric coordinates
        // d01 = dot(ab, ac)
        const d01 = abx * acx + aby * acy + abz * acz;

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
            // a0 = dot(a, ab)
            const a0 = a[0] * abx + a[1] * aby + a[2] * abz;
            // a1 = dot(a, ac)
            const a1 = a[0] * acx + a[1] * acy + a[2] * acz;

            out.v = (d01 * a1 - d11 * a0) / denominator;
            out.w = (d01 * a0 - d00 * a1) / denominator;
            out.u = 1.0 - out.v - out.w;
        }
    } else {
        // use v1 and v2 to calculate barycentric coordinates
        // d12 = dot(ac, bc)
        const d12 = acx * bcx + acy * bcy + acz * bcz;

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
            // c1 = dot(c, ac)
            const c1 = c[0] * acx + c[1] * acy + c[2] * acz;
            // c2 = dot(c, bc)
            const c2 = c[0] * bcx + c[1] * bcy + c[2] * bcz;

            out.u = (d22 * c1 - d12 * c2) / denominator;
            out.v = (d11 * c2 - d12 * c1) / denominator;
            out.w = 1.0 - out.u - out.v;
        }
    }
    out.isValid = true;
};


