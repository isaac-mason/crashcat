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

const _barycentric_line = /* @__PURE__ */ createBarycentricCoordinatesResult();

export function computeClosestPointOnLine(out: ClosestPointResult, a: Vec3, b: Vec3, squaredTolerance: number): void {
    computeBarycentricCoordinates2d(_barycentric_line, a, b, squaredTolerance);

    if (_barycentric_line.v <= 0.0) {
        // a is closest point
        out.point[0] = a[0];
        out.point[1] = a[1];
        out.point[2] = a[2];
        out.pointSet = 0b0001;
    } else if (_barycentric_line.u <= 0.0) {
        // b is closest point
        out.point[0] = b[0];
        out.point[1] = b[1];
        out.point[2] = b[2];
        out.pointSet = 0b0010;
    } else {
        // closest point lies on line ab
        out.point[0] = a[0] * _barycentric_line.u + b[0] * _barycentric_line.v;
        out.point[1] = a[1] * _barycentric_line.u + b[1] * _barycentric_line.v;
        out.point[2] = a[2] * _barycentric_line.u + b[2] * _barycentric_line.v;
        out.pointSet = 0b0011;
    }
}

export function computeClosestPointOnTriangle(
    out: ClosestPointResult,
    inA: Vec3,
    inB: Vec3,
    inC: Vec3,
    mustIncludeC: boolean,
    squaredTolerance: number,
): void {
    // the most accurate normal is calculated by using the two shortest edges
    const acx = inC[0] - inA[0];
    const acy = inC[1] - inA[1];
    const acz = inC[2] - inA[2];
    
    const bcx = inC[0] - inB[0];
    const bcy = inC[1] - inB[1];
    const bcz = inC[2] - inB[2];
    
    const swapAC = bcx * bcx + bcy * bcy + bcz * bcz < acx * acx + acy * acy + acz * acz;

    // choose a and c based on swap
    const ax = swapAC ? inC[0] : inA[0];
    const ay = swapAC ? inC[1] : inA[1];
    const az = swapAC ? inC[2] : inA[2];
    const cx = swapAC ? inA[0] : inC[0];
    const cy = swapAC ? inA[1] : inC[1];
    const cz = swapAC ? inA[2] : inC[2];

    // calculate normal
    const abx = inB[0] - ax;
    const aby = inB[1] - ay;
    const abz = inB[2] - az;

    const ac_x = cx - ax;
    const ac_y = cy - ay;
    const ac_z = cz - az;

    const nx = aby * ac_z - abz * ac_y;
    const ny = abz * ac_x - abx * ac_z;
    const nz = abx * ac_y - aby * ac_x;

    const normalLengthSquared = nx * nx + ny * ny + nz * nz;

    // check degenerate
    if (normalLengthSquared < 1.0e-10) {
        // degenerate, fallback to vertices and edges
        let closestSet = 0b0100;
        let closestX = inC[0];
        let closestY = inC[1];
        let closestZ = inC[2];
        let bestDistanceSquared = inC[0] * inC[0] + inC[1] * inC[1] + inC[2] * inC[2];

        if (!mustIncludeC) {
            // try vertex A
            const aLengthSquared = inA[0] * inA[0] + inA[1] * inA[1] + inA[2] * inA[2];

            if (aLengthSquared < bestDistanceSquared) {
                closestSet = 0b0001;
                closestX = inA[0];
                closestY = inA[1];
                closestZ = inA[2];
                bestDistanceSquared = aLengthSquared;
            }

            // try vertex B
            const bLengthSquared = inB[0] * inB[0] + inB[1] * inB[1] + inB[2] * inB[2];
            if (bLengthSquared < bestDistanceSquared) {
                closestSet = 0b0010;
                closestX = inB[0];
                closestY = inB[1];
                closestZ = inB[2];
                bestDistanceSquared = bLengthSquared;
            }
        }

        // edge AC
        const ac2x = cx - ax;
        const ac2y = cy - ay;
        const ac2z = cz - az;
        const acLengthSquared = ac2x * ac2x + ac2y * ac2y + ac2z * ac2z;

        if (acLengthSquared > squaredTolerance) {
            const v = clamp(-(ax * ac2x + ay * ac2y + az * ac2z) / acLengthSquared, 0.0, 1.0);
            const qx = ax + ac2x * v;
            const qy = ay + ac2y * v;
            const qz = az + ac2z * v;

            const distanceSquared = qx * qx + qy * qy + qz * qz;

            if (distanceSquared < bestDistanceSquared) {
                closestSet = 0b0101;
                closestX = qx;
                closestY = qy;
                closestZ = qz;
                bestDistanceSquared = distanceSquared;
            }
        }

        // edge BC
        const bc2x = inC[0] - inB[0];
        const bc2y = inC[1] - inB[1];
        const bc2z = inC[2] - inB[2];

        const bcLengthSquared = bc2x * bc2x + bc2y * bc2y + bc2z * bc2z;

        if (bcLengthSquared > squaredTolerance) {
            const v = clamp(-(inB[0] * bc2x + inB[1] * bc2y + inB[2] * bc2z) / bcLengthSquared, 0.0, 1.0);

            const qx = inB[0] + bc2x * v;
            const qy = inB[1] + bc2y * v;
            const qz = inB[2] + bc2z * v;

            const distanceSquared = qx * qx + qy * qy + qz * qz;

            if (distanceSquared < bestDistanceSquared) {
                closestSet = 0b0110;
                closestX = qx;
                closestY = qy;
                closestZ = qz;
                bestDistanceSquared = distanceSquared;
            }
        }

        if (!mustIncludeC) {
            // edge AB
            const ab2x = inB[0] - inA[0];
            const ab2y = inB[1] - inA[1];
            const ab2z = inB[2] - inA[2];

            const abLengthSquared = ab2x * ab2x + ab2y * ab2y + ab2z * ab2z;

            if (abLengthSquared > squaredTolerance) {
                const v = clamp(-(inA[0] * ab2x + inA[1] * ab2y + inA[2] * ab2z) / abLengthSquared, 0.0, 1.0);

                const qx = inA[0] + ab2x * v;
                const qy = inA[1] + ab2y * v;
                const qz = inA[2] + ab2z * v;

                const distanceSquared = qx * qx + qy * qy + qz * qz;

                if (distanceSquared < bestDistanceSquared) {
                    closestSet = 0b0011;
                    closestX = qx;
                    closestY = qy;
                    closestZ = qz;
                }
            }
        }

        out.pointSet = closestSet;
        out.point[0] = closestX;
        out.point[1] = closestY;
        out.point[2] = closestZ;

        return;
    }

    // check if P in vertex region outside A
    const apx = -ax;
    const apy = -ay;
    const apz = -az;

    const d1 = abx * apx + aby * apy + abz * apz;
    const d2 = ac_x * apx + ac_y * apy + ac_z * apz;

    if (d1 <= 0.0 && d2 <= 0.0) {
        out.pointSet = swapAC ? 0b0100 : 0b0001;
        out.point[0] = ax;
        out.point[1] = ay;
        out.point[2] = az;
        return;
    }

    // check if P in vertex region outside B
    const bpx = -inB[0];
    const bpy = -inB[1];
    const bpz = -inB[2];

    const d3 = abx * bpx + aby * bpy + abz * bpz;
    const d4 = ac_x * bpx + ac_y * bpy + ac_z * bpz;

    if (d3 >= 0.0 && d4 <= d3) {
        out.pointSet = 0b0010;
        out.point[0] = inB[0];
        out.point[1] = inB[1];
        out.point[2] = inB[2];
        return;
    }

    // check if P in edge region of AB
    if (d1 * d4 <= d3 * d2 && d1 >= 0.0 && d3 <= 0.0) {
        const v = d1 / (d1 - d3);
        out.pointSet = swapAC ? 0b0110 : 0b0011;
        out.point[0] = ax + abx * v;
        out.point[1] = ay + aby * v;
        out.point[2] = az + abz * v;
        return;
    }

    // check if P in vertex region outside C
    const cpx = -cx;
    const cpy = -cy;
    const cpz = -cz;

    const d5 = abx * cpx + aby * cpy + abz * cpz;
    const d6 = ac_x * cpx + ac_y * cpy + ac_z * cpz;

    if (d6 >= 0.0 && d5 <= d6) {
        out.pointSet = swapAC ? 0b0001 : 0b0100;
        out.point[0] = cx;
        out.point[1] = cy;
        out.point[2] = cz;
        return;
    }

    // check if P in edge region of AC
    if (d5 * d2 <= d1 * d6 && d2 >= 0.0 && d6 <= 0.0) {
        const w = d2 / (d2 - d6);
        out.pointSet = 0b0101;
        out.point[0] = ax + ac_x * w;
        out.point[1] = ay + ac_y * w;
        out.point[2] = az + ac_z * w;
        return;
    }

    // check if P in edge region of BC
    const diff_d4_d3 = d4 - d3;
    const diff_d5_d6 = d5 - d6;
    if (d3 * d6 <= d5 * d4 && diff_d4_d3 >= 0.0 && diff_d5_d6 >= 0.0) {
        const w = diff_d4_d3 / (diff_d4_d3 + diff_d5_d6);
        out.pointSet = swapAC ? 0b0011 : 0b0110;
        
        const tempx = cx - inB[0];
        const tempy = cy - inB[1];
        const tempz = cz - inB[2];

        out.point[0] = inB[0] + tempx * w;
        out.point[1] = inB[1] + tempy * w;
        out.point[2] = inB[2] + tempz * w;
        return;
    }

    // P inside face region
    out.pointSet = 0b0111;

    const tempx = ax + inB[0] + cx;
    const tempy = ay + inB[1] + cy;
    const tempz = az + inB[2] + cz;

    const scale = (tempx * nx + tempy * ny + tempz * nz) / (3 * normalLengthSquared);
    out.point[0] = nx * scale;
    out.point[1] = ny * scale;
    out.point[2] = nz * scale;
}

// helper type for tracking which triangle planes the origin is outside of
type TrianglePlaneFlags = { x: number; y: number; z: number; w: number };

function isOriginOutsideOfTrianglePlanes(out: TrianglePlaneFlags, a: Vec3, b: Vec3, c: Vec3, d: Vec3, tolerance: number): void {
    // compute edge vectors
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const abz = b[2] - a[2];

    const acx = c[0] - a[0];
    const acy = c[1] - a[1];
    const acz = c[2] - a[2];

    const adx = d[0] - a[0];
    const ady = d[1] - a[1];
    const adz = d[2] - a[2];

    const bdx = d[0] - b[0];
    const bdy = d[1] - b[1];
    const bdz = d[2] - b[2];

    const bcx = c[0] - b[0];
    const bcy = c[1] - b[1];
    const bcz = c[2] - b[2];

    // compute cross products (triangle normals)
    // ab x ac
    const abac_x = aby * acz - abz * acy;
    const abac_y = abz * acx - abx * acz;
    const abac_z = abx * acy - aby * acx;

    // ac x ad
    const acad_x = acy * adz - acz * ady;
    const acad_y = acz * adx - acx * adz;
    const acad_z = acx * ady - acy * adx;

    // ad x ab
    const adab_x = ady * abz - adz * aby;
    const adab_y = adz * abx - adx * abz;
    const adab_z = adx * aby - ady * abx;

    // bd x bc
    const bdbc_x = bdy * bcz - bdz * bcy;
    const bdbc_y = bdz * bcx - bdx * bcz;
    const bdbc_z = bdx * bcy - bdy * bcx;

    // for each plane get the side on which the origin is
    const signP_x = a[0] * abac_x + a[1] * abac_y + a[2] * abac_z; // ABC
    const signP_y = a[0] * acad_x + a[1] * acad_y + a[2] * acad_z; // ACD
    const signP_z = a[0] * adab_x + a[1] * adab_y + a[2] * adab_z; // ADB
    const signP_w = b[0] * bdbc_x + b[1] * bdbc_y + b[2] * bdbc_z; // BDC

    // for each plane get the side that is outside (determined by the 4th point)
    const signD_x = adx * abac_x + ady * abac_y + adz * abac_z; // D
    const signD_y = abx * acad_x + aby * acad_y + abz * acad_z; // B
    const signD_z = acx * adab_x + acy * adab_y + acz * adab_z; // C
    const signD_w = -(abx * bdbc_x + aby * bdbc_y + abz * bdbc_z); // A

    const allPositive = signD_x > 0 && signD_y > 0 && signD_z > 0 && signD_w > 0;

    if (allPositive) {
        out.x = signP_x >= -tolerance ? 1 : 0;
        out.y = signP_y >= -tolerance ? 1 : 0;
        out.z = signP_z >= -tolerance ? 1 : 0;
        out.w = signP_w >= -tolerance ? 1 : 0;
        return;
    }

    const allNegative = signD_x < 0 && signD_y < 0 && signD_z < 0 && signD_w < 0;

    if (allNegative) {
        out.x = signP_x <= tolerance ? 1 : 0;
        out.y = signP_y <= tolerance ? 1 : 0;
        out.z = signP_z <= tolerance ? 1 : 0;
        out.w = signP_w <= tolerance ? 1 : 0;
        return;
    }

    // mixed signs, degenerate tetrahedron
    out.x = 1;
    out.y = 1;
    out.z = 1;
    out.w = 1;
}

const _otherResult_tet = /* @__PURE__ */ createClosestPointResult();
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
        computeClosestPointOnTriangle(_otherResult_tet, inA, inC, inD, mustIncludeD, squaredTolerance);
        const distanceSquared =
            _otherResult_tet.point[0] * _otherResult_tet.point[0] +
            _otherResult_tet.point[1] * _otherResult_tet.point[1] +
            _otherResult_tet.point[2] * _otherResult_tet.point[2];
        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            out.point[0] = _otherResult_tet.point[0];
            out.point[1] = _otherResult_tet.point[1];
            out.point[2] = _otherResult_tet.point[2];
            out.pointSet = (_otherResult_tet.pointSet & 0b0001) + ((_otherResult_tet.pointSet & 0b0110) << 1);
        }
    }

    // face adb
    if (_originOutOfPlanes.z) {
        computeClosestPointOnTriangle(_otherResult_tet, inA, inB, inD, mustIncludeD, squaredTolerance);
        const distanceSquared =
            _otherResult_tet.point[0] * _otherResult_tet.point[0] +
            _otherResult_tet.point[1] * _otherResult_tet.point[1] +
            _otherResult_tet.point[2] * _otherResult_tet.point[2];
        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            out.point[0] = _otherResult_tet.point[0];
            out.point[1] = _otherResult_tet.point[1];
            out.point[2] = _otherResult_tet.point[2];
            out.pointSet = (_otherResult_tet.pointSet & 0b0011) + ((_otherResult_tet.pointSet & 0b0100) << 1);
        }
    }

    // face bdc
    if (_originOutOfPlanes.w) {
        _otherResult_tet.pointSet = 0;
        _otherResult_tet.point[0] = 0;
        _otherResult_tet.point[1] = 0;
        _otherResult_tet.point[2] = 0;
        computeClosestPointOnTriangle(_otherResult_tet, inB, inC, inD, mustIncludeD, squaredTolerance);
        const distanceSquared =
            _otherResult_tet.point[0] * _otherResult_tet.point[0] +
            _otherResult_tet.point[1] * _otherResult_tet.point[1] +
            _otherResult_tet.point[2] * _otherResult_tet.point[2];
        if (distanceSquared < bestDistanceSquared) {
            out.point[0] = _otherResult_tet.point[0];
            out.point[1] = _otherResult_tet.point[1];
            out.point[2] = _otherResult_tet.point[2];
            out.pointSet = _otherResult_tet.pointSet << 1;
        }
    }
}

const GJK_TOLERANCE = 1e-5;
const GJK_MAX_ITERATIONS = 100;

const _p = /* @__PURE__ */ vec3.create();
const _q = /* @__PURE__ */ vec3.create();
const _w = /* @__PURE__ */ vec3.create();
const _x = /* @__PURE__ */ vec3.create();
const _v = /* @__PURE__ */ vec3.create();
const _directionA = /* @__PURE__ */ vec3.create();
const _directionB = /* @__PURE__ */ vec3.create();
const _pq = /* @__PURE__ */ vec3.create();
const _prevV = /* @__PURE__ */ vec3.create();
const _normalizedV = /* @__PURE__ */ vec3.create();
const _simplex = /* @__PURE__ */ createSimplex();
const _bary = /* @__PURE__ */ createBarycentricCoordinatesResult();
const _closestPoint = /* @__PURE__ */ createClosestPointResult();
const _closestPointToSimplex = /* @__PURE__ */ createClosestPointToSimplexResult();
const _transformedSupportA = /* @__PURE__ */ createTransformedSupport();

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
        result.point[0] = _closestPoint.point[0];
        result.point[1] = _closestPoint.point[1];
        result.point[2] = _closestPoint.point[2];
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

    _x[0] = rayOrigin[0];
    _x[1] = rayOrigin[1];
    _x[2] = rayOrigin[2];

    // v = x - support(0)
    // vec3.zero(_directionA);
    _directionA[0] = 0;
    _directionA[1] = 0;
    _directionA[2] = 0;
    support.getSupport(_directionA, _p);

    // _v = _x - _p
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

        // _w = _x - _p
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
        const newPoint = _simplex.points[_simplex.size];
        newPoint.p[0] = _p[0];
        newPoint.p[1] = _p[1];
        newPoint.p[2] = _p[2];
        _simplex.size++;

        // calculate Y = {x} - P
        for (let i = 0; i < _simplex.size; i++) {
            const point = _simplex.points[i];
            // y = x - p
            point.y[0] = _x[0] - point.p[0];
            point.y[1] = _x[1] - point.p[1];
            point.y[2] = _x[2] - point.p[2];
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
    _closestPointToSimplex.point[0] = direction[0];
    _closestPointToSimplex.point[1] = direction[1];
    _closestPointToSimplex.point[2] = direction[2];
    _closestPointToSimplex.squaredDistance =
        direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
    _closestPointToSimplex.pointSet = 0;
    _closestPointToSimplex.closestPointFound = true;
    // previous length^2 of v
    let previousSquaredDistance = Number.MAX_VALUE;

    let iterations = 0;
    while (iterations++ < GJK_MAX_ITERATIONS) {
        // get support points for shape A and B in the direction of v
        _directionA[0] = _closestPointToSimplex.point[0];
        _directionA[1] = _closestPointToSimplex.point[1];
        _directionA[2] = _closestPointToSimplex.point[2];
        _directionB[0] = -_closestPointToSimplex.point[0];
        _directionB[1] = -_closestPointToSimplex.point[1];
        _directionB[2] = -_closestPointToSimplex.point[2];

        supportA.getSupport(_directionA, _p);
        supportB.getSupport(_directionB, _q);

        // get support point of the minkowski sum A - B of v
        _w[0] = _p[0] - _q[0];
        _w[1] = _p[1] - _q[1];
        _w[2] = _p[2] - _q[2];

        const dot =
            _closestPointToSimplex.point[0] * _w[0] +
            _closestPointToSimplex.point[1] * _w[1] +
            _closestPointToSimplex.point[2] * _w[2];

        // test if we have a separation of more than inMaxDistSq, in which case we terminate early
        if (dot < 0.0 && dot * dot > _closestPointToSimplex.squaredDistance * maxDistanceSquared) {
            out.squaredDistance = Number.MAX_VALUE;
            out.penetrationAxis[0] = _closestPointToSimplex.point[0];
            out.penetrationAxis[1] = _closestPointToSimplex.point[1];
            out.penetrationAxis[2] = _closestPointToSimplex.point[2];
            return;
        }

        // store the point for later use
        const y = _simplex.points[_simplex.size].y;
        y[0] = _w[0];
        y[1] = _w[1];
        y[2] = _w[2];

        const p = _simplex.points[_simplex.size].p;
        p[0] = _p[0];
        p[1] = _p[1];
        p[2] = _p[2];

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

                    dstY[0] = srcY[0];
                    dstY[1] = srcY[1];
                    dstY[2] = srcY[2];

                    dstP[0] = srcP[0];
                    dstP[1] = srcP[1];
                    dstP[2] = srcP[2];

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

            const squaredLength = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];

            yMaxLengthSquared = Math.max(yMaxLengthSquared, squaredLength);
        }

        if (_closestPointToSimplex.squaredDistance <= GJK_TOLERANCE * yMaxLengthSquared) {
            _closestPointToSimplex.point[0] = 0;
            _closestPointToSimplex.point[1] = 0;
            _closestPointToSimplex.point[2] = 0;
            _closestPointToSimplex.squaredDistance = 0.0;
            break;
        }

        // the next separation axis to test is the negative of the closest point of the Minkowski sum to the origin
        // note: this must be done before terminating as converged since the separating axis is -v
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
    out.pointA[0] = 0;
    out.pointA[1] = 0;
    out.pointA[2] = 0;

    out.pointB[0] = 0;
    out.pointB[1] = 0;
    out.pointB[2] = 0;

    copySimplex(out.simplex, _simplex);

    // handle early termination case: if simplex is empty, GJK terminated early (shapes are far apart)
    // return a large distance to indicate separation
    if (_simplex.size === 0) {
        out.squaredDistance = Number.MAX_VALUE;
        out.penetrationAxis[0] = 0;
        out.penetrationAxis[1] = 0;
        out.penetrationAxis[2] = 0;
        return;
    }

    switch (_simplex.size) {
        case 1: {
            // single point in simplex
            const { p, q } = _simplex.points[0];

            out.pointA[0] = p[0];
            out.pointA[1] = p[1];
            out.pointA[2] = p[2];

            out.pointB[0] = q[0];
            out.pointB[1] = q[1];
            out.pointB[2] = q[2];
            break;
        }

        case 2: {
            // line segment in simplex
            const p0 = _simplex.points[0];
            const p1 = _simplex.points[1];
            computeBarycentricCoordinates2d(_bary, p0.y, p1.y, 1e-10);

            const p0p = p0.p;
            const p1p = p1.p;
            out.pointA[0] = p0p[0] * _bary.u + p1p[0] * _bary.v;
            out.pointA[1] = p0p[1] * _bary.u + p1p[1] * _bary.v;
            out.pointA[2] = p0p[2] * _bary.u + p1p[2] * _bary.v;

            const p0q = p0.q;
            const p1q = p1.q;
            out.pointB[0] = p0q[0] * _bary.u + p1q[0] * _bary.v;
            out.pointB[1] = p0q[1] * _bary.u + p1q[1] * _bary.v;
            out.pointB[2] = p0q[2] * _bary.u + p1q[2] * _bary.v;
            break;
        }

        case 3: {
            // triangle in simplex
            const p0 = _simplex.points[0];
            const p1 = _simplex.points[1];
            const p2 = _simplex.points[2];
            computeBarycentricCoordinates3d(_bary, p0.y, p1.y, p2.y, 1e-10);

            const p0p = p0.p;
            const p1p = p1.p;
            const p2p = p2.p;
            out.pointA[0] = p0p[0] * _bary.u + p1p[0] * _bary.v + p2p[0] * _bary.w;
            out.pointA[1] = p0p[1] * _bary.u + p1p[1] * _bary.v + p2p[1] * _bary.w;
            out.pointA[2] = p0p[2] * _bary.u + p1p[2] * _bary.v + p2p[2] * _bary.w;

            const p0q = p0.q;
            const p1q = p1.q;
            const p2q = p2.q;
            out.pointB[0] = p0q[0] * _bary.u + p1q[0] * _bary.v + p2q[0] * _bary.w;
            out.pointB[1] = p0q[1] * _bary.u + p1q[1] * _bary.v + p2q[1] * _bary.w;
            out.pointB[2] = p0q[2] * _bary.u + p1q[2] * _bary.v + p2q[2] * _bary.w;
            break;
        }

        default: {
            // for a full simplex (4 points = tetrahedron), the origin is inside the Minkowski difference
            // in this case, pointA and pointB remain at zero (as initialized)
            break;
        }
    }

    // store the separating axis / penetration axis
    out.penetrationAxis[0] = _closestPointToSimplex.point[0];
    out.penetrationAxis[1] = _closestPointToSimplex.point[1];
    out.penetrationAxis[2] = _closestPointToSimplex.point[2];
    out.squaredDistance = _closestPointToSimplex.squaredDistance;
}
