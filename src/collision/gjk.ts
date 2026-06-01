import { clamp, type Mat4, type Vec3, vec3 } from 'mathcat';
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

const _lineBary = /* @__PURE__ */ createBarycentricCoordinatesResult();

/**
 * @optimize
 */
export function computeClosestPointOnLine(out: ClosestPointResult, a: Vec3, b: Vec3, squaredTolerance: number): void {
    computeBarycentricCoordinates2d(_lineBary, a, b, squaredTolerance);
    const u = _lineBary.u;
    const v = _lineBary.v;

    if (v <= 0.0) {
        // a is closest point
        vec3.copy(out.point, a);
        out.pointSet = 0b0001;
    } else if (u <= 0.0) {
        // b is closest point
        vec3.copy(out.point, b);
        out.pointSet = 0b0010;
    } else {
        // closest point lies on line ab: out = a * u + b * v == lerp(a, b, v)
        vec3.lerp(out.point, a, b, v);
        out.pointSet = 0b0011;
    }
}

/**
 * @optimize
 */
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

        const bcx = cx - inB[0];
        const bcy = cy - inB[1];
        const bcz = cz - inB[2];

        out.point[0] = inB[0] + bcx * w;
        out.point[1] = inB[1] + bcy * w;
        out.point[2] = inB[2] + bcz * w;
        return;
    }

    // P inside face region
    out.pointSet = 0b0111;

    const sumx = ax + inB[0] + cx;
    const sumy = ay + inB[1] + cy;
    const sumz = az + inB[2] + cz;

    const scale = (sumx * nx + sumy * ny + sumz * nz) / (3 * normalLengthSquared);
    out.point[0] = nx * scale;
    out.point[1] = ny * scale;
    out.point[2] = nz * scale;
}

const _otherResult_tet = /* @__PURE__ */ createClosestPointResult();

/**
 * @optimize
 */
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
    out.point[0] = 0;
    out.point[1] = 0;
    out.point[2] = 0;

    let bestDistanceSquared = Infinity;

    // determine for each of the faces if the origin is outside
    const abx = inB[0] - inA[0];
    const aby = inB[1] - inA[1];
    const abz = inB[2] - inA[2];

    const acx = inC[0] - inA[0];
    const acy = inC[1] - inA[1];
    const acz = inC[2] - inA[2];

    const adx = inD[0] - inA[0];
    const ady = inD[1] - inA[1];
    const adz = inD[2] - inA[2];

    const bdx = inD[0] - inB[0];
    const bdy = inD[1] - inB[1];
    const bdz = inD[2] - inB[2];

    const bcx = inC[0] - inB[0];
    const bcy = inC[1] - inB[1];
    const bcz = inC[2] - inB[2];

    // triangle normals (cross products)
    const abac_x = aby * acz - abz * acy;
    const abac_y = abz * acx - abx * acz;
    const abac_z = abx * acy - aby * acx;

    const acad_x = acy * adz - acz * ady;
    const acad_y = acz * adx - acx * adz;
    const acad_z = acx * ady - acy * adx;

    const adab_x = ady * abz - adz * aby;
    const adab_y = adz * abx - adx * abz;
    const adab_z = adx * aby - ady * abx;

    const bdbc_x = bdy * bcz - bdz * bcy;
    const bdbc_y = bdz * bcx - bdx * bcz;
    const bdbc_z = bdx * bcy - bdy * bcx;

    // side of the origin for each plane
    const signP_x = inA[0] * abac_x + inA[1] * abac_y + inA[2] * abac_z; // ABC
    const signP_y = inA[0] * acad_x + inA[1] * acad_y + inA[2] * acad_z; // ACD
    const signP_z = inA[0] * adab_x + inA[1] * adab_y + inA[2] * adab_z; // ADB
    const signP_w = inB[0] * bdbc_x + inB[1] * bdbc_y + inB[2] * bdbc_z; // BDC

    // side that is outside (determined by the 4th point)
    const signD_x = adx * abac_x + ady * abac_y + adz * abac_z; // D
    const signD_y = abx * acad_x + aby * acad_y + abz * acad_z; // B
    const signD_z = acx * adab_x + acy * adab_y + acz * adab_z; // C
    const signD_w = -(abx * bdbc_x + aby * bdbc_y + abz * bdbc_z); // A

    let originOutABC: number;
    let originOutACD: number;
    let originOutADB: number;
    let originOutBDC: number;

    if (signD_x > 0 && signD_y > 0 && signD_z > 0 && signD_w > 0) {
        originOutABC = signP_x >= -tolerance ? 1 : 0;
        originOutACD = signP_y >= -tolerance ? 1 : 0;
        originOutADB = signP_z >= -tolerance ? 1 : 0;
        originOutBDC = signP_w >= -tolerance ? 1 : 0;
    } else if (signD_x < 0 && signD_y < 0 && signD_z < 0 && signD_w < 0) {
        originOutABC = signP_x <= tolerance ? 1 : 0;
        originOutACD = signP_y <= tolerance ? 1 : 0;
        originOutADB = signP_z <= tolerance ? 1 : 0;
        originOutBDC = signP_w <= tolerance ? 1 : 0;
    } else {
        // mixed signs, degenerate tetrahedron — consider every face
        originOutABC = 1;
        originOutACD = 1;
        originOutADB = 1;
        originOutBDC = 1;
    }

    // if point outside face abc
    if (originOutABC) {
        if (mustIncludeD) {
            out.pointSet = 0b0001;
            out.point[0] = inA[0];
            out.point[1] = inA[1];
            out.point[2] = inA[2];
        } else {
            computeClosestPointOnTriangle(out, inA, inB, inC, false, squaredTolerance);
        }
        bestDistanceSquared = out.point[0] * out.point[0] + out.point[1] * out.point[1] + out.point[2] * out.point[2];
    }

    // face acd
    if (originOutACD) {
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
    if (originOutADB) {
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
    if (originOutBDC) {
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

// scratch vec3s used to extract simplex y/p/q points into a Vec3-shaped arg for
// the closest-point kernels, which still take (a: Vec3, b: Vec3, ...) today.
const _simplexY0 = /* @__PURE__ */ vec3.create();
const _simplexY1 = /* @__PURE__ */ vec3.create();
const _simplexY2 = /* @__PURE__ */ vec3.create();
const _simplexY3 = /* @__PURE__ */ vec3.create();

/**
 * Recompute simplex.y from y = x - p (ray cast variant).
 *
 * @optimize
 */
function recomputeSimplexYFromP(simplex: Simplex, x: Vec3): void {
    const end = simplex.size * 3;
    const py = simplex.y;
    const pp = simplex.p;
    for (let i = 0; i < end; i += 3) {
        py[i] = x[0] - pp[i];
        py[i + 1] = x[1] - pp[i + 1];
        py[i + 2] = x[2] - pp[i + 2];
    }
}

/**
 * Recompute simplex.y from y = x - (q - p) (shape cast variant).
 *
 * @optimize
 */
function recomputeSimplexYFromPQ(simplex: Simplex, x: Vec3): void {
    const end = simplex.size * 3;
    const py = simplex.y;
    const pp = simplex.p;
    const qq = simplex.q;
    for (let i = 0; i < end; i += 3) {
        py[i] = x[0] - qq[i] + pp[i];
        py[i + 1] = x[1] - qq[i + 1] + pp[i + 1];
        py[i + 2] = x[2] - qq[i + 2] + pp[i + 2];
    }
}

/**
 * Compact simplex.p down to the subset selected by inSet (bit i selects point i).
 *
 * @optimize
 */
function updatePointSetP(simplex: Simplex, inSet: number): void {
    let newSize = 0;
    const pp = simplex.p;
    for (let i = 0; i < simplex.size; i++) {
        if ((inSet & (1 << i)) !== 0) {
            if (newSize !== i) {
                const srcOff = i * 3;
                const dstOff = newSize * 3;
                pp[dstOff] = pp[srcOff];
                pp[dstOff + 1] = pp[srcOff + 1];
                pp[dstOff + 2] = pp[srcOff + 2];
            }
            newSize++;
        }
    }
    simplex.size = newSize;
}

/**
 * Compact simplex.y, simplex.p, simplex.q down to the subset selected by inSet.
 *
 * @optimize
 */
function updatePointSetYPQ(simplex: Simplex, inSet: number): void {
    let newSize = 0;
    const yy = simplex.y;
    const pp = simplex.p;
    const qq = simplex.q;
    for (let i = 0; i < simplex.size; i++) {
        if ((inSet & (1 << i)) !== 0) {
            if (newSize !== i) {
                const srcOff = i * 3;
                const dstOff = newSize * 3;
                yy[dstOff] = yy[srcOff];
                yy[dstOff + 1] = yy[srcOff + 1];
                yy[dstOff + 2] = yy[srcOff + 2];
                pp[dstOff] = pp[srcOff];
                pp[dstOff + 1] = pp[srcOff + 1];
                pp[dstOff + 2] = pp[srcOff + 2];
                qq[dstOff] = qq[srcOff];
                qq[dstOff + 1] = qq[srcOff + 1];
                qq[dstOff + 2] = qq[srcOff + 2];
            }
            newSize++;
        }
    }
    simplex.size = newSize;
}

/**
 * @optimize
 */
function computeClosestPointToSimplex(
    result: ClosestPointToSimplexResult,
    prevSquaredDist: number,
    lastPointPartOfClosest: boolean,
    simplex: Simplex,
): boolean {
    const y = simplex.y;
    switch (simplex.size) {
        case 1: {
            // single point
            _closestPoint.pointSet = 0b0001;
            const point = _closestPoint.point;
            point[0] = y[0];
            point[1] = y[1];
            point[2] = y[2];
            break;
        }

        case 2: {
            // line segment
            _simplexY0[0] = y[0]; _simplexY0[1] = y[1]; _simplexY0[2] = y[2];
            _simplexY1[0] = y[3]; _simplexY1[1] = y[4]; _simplexY1[2] = y[5];
            computeClosestPointOnLine(_closestPoint, _simplexY0, _simplexY1, 1e-10);
            break;
        }

        case 3: {
            // triangle
            _simplexY0[0] = y[0]; _simplexY0[1] = y[1]; _simplexY0[2] = y[2];
            _simplexY1[0] = y[3]; _simplexY1[1] = y[4]; _simplexY1[2] = y[5];
            _simplexY2[0] = y[6]; _simplexY2[1] = y[7]; _simplexY2[2] = y[8];
            computeClosestPointOnTriangle(
                _closestPoint,
                _simplexY0,
                _simplexY1,
                _simplexY2,
                lastPointPartOfClosest,
                1e-10,
            );
            break;
        }

        case 4: {
            // tetrahedron
            _simplexY0[0] = y[0]; _simplexY0[1] = y[1]; _simplexY0[2] = y[2];
            _simplexY1[0] = y[3]; _simplexY1[1] = y[4]; _simplexY1[2] = y[5];
            _simplexY2[0] = y[6]; _simplexY2[1] = y[7]; _simplexY2[2] = y[8];
            _simplexY3[0] = y[9]; _simplexY3[1] = y[10]; _simplexY3[2] = y[11];
            computeClosestPointOnTetrahedron(
                _closestPoint,
                _simplexY0,
                _simplexY1,
                _simplexY2,
                _simplexY3,
                lastPointPartOfClosest,
                1e-5,
            );
            break;
        }

        default: {
            throw new Error('Invalid number of points in simplex');
        }
    }

    const squaredDistance =
        _closestPoint.point[0] * _closestPoint.point[0] +
        _closestPoint.point[1] * _closestPoint.point[1] +
        _closestPoint.point[2] * _closestPoint.point[2];

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
 *
 * @optimize
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

    _simplex.size = 0;

    let lambda = 0.0;

    vec3.copy(_x, rayOrigin);

    // v = x - support(0)
    vec3.set(_directionA, 0, 0, 0);
    support.getSupport(_directionA, _p);
    vec3.subtract(_v, _x, _p);

    let v_len_sq = Number.MAX_VALUE;
    let allowRestart = false;

    let iterations = 0;
    while (iterations < GJK_MAX_ITERATIONS) {
        iterations++;

        // get new support point
        support.getSupport(_v, _p);

        vec3.subtract(_w, _x, _p);

        const vDotW = vec3.dot(_v, _w);

        if (vDotW > 0.0) {
            // if ray and normal are in the same direction, we've passed A and there's no collision
            const vDotR = vec3.dot(_v, rayDirection);

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
            vec3.scaleAndAdd(_x, rayOrigin, rayDirection, lambda);

            // we've shifted x, so reset v_len_sq so that it is not used as early out for GetClosest
            v_len_sq = Number.MAX_VALUE;

            // we allow rebuilding the simplex once after x changes because the simplex was built
            // for another x and numerical round off builds up as you keep adding points to an
            // existing simplex
            allowRestart = true;
        }

        // add p to set P: P = P U {p}
        {
            const off = _simplex.size * 3;
            _simplex.p[off] = _p[0];
            _simplex.p[off + 1] = _p[1];
            _simplex.p[off + 2] = _p[2];
            _simplex.size++;
        }

        // calculate Y = {x} - P
        recomputeSimplexYFromP(_simplex, _x);

        // determine the new closest point from Y to origin
        const found = computeClosestPointToSimplex(_closestPointToSimplex, v_len_sq, false, _simplex);
        if (found) {
            v_len_sq = _closestPointToSimplex.squaredDistance;
            vec3.copy(_v, _closestPointToSimplex.point);
        }

        if (!found) {
            // only allow 1 restart, if we still can't get a closest point
            // we're so close that we return this as a hit
            if (!allowRestart) {
                break;
            }

            // if we fail to converge, we start again with the last point as simplex
            allowRestart = false;

            _simplex.p[0] = _p[0];
            _simplex.p[1] = _p[1];
            _simplex.p[2] = _p[2];
            _simplex.size = 1;

            vec3.subtract(_v, _x, _p);
            v_len_sq = Number.MAX_VALUE;
            continue;
        } else if (_closestPointToSimplex.pointSet === 0xf) {
            // we're inside the tetrahedron, we have a hit (verify that length of v is 0)
            break;
        }

        // update the points P to form the new simplex
        // note: we're not updating Y as Y will shift with x so we have to calculate it every iteration
        updatePointSetP(_simplex, _closestPointToSimplex.pointSet);

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
    const pp = simplex.p;
    const qq = simplex.q;

    for (let i = 0; i < simplex.size; i++) {
        if ((inSet & (1 << i)) !== 0) {
            if (newSize !== i) {
                const srcOff = i * 3;
                const dstOff = newSize * 3;
                pp[dstOff] = pp[srcOff];
                pp[dstOff + 1] = pp[srcOff + 1];
                pp[dstOff + 2] = pp[srcOff + 2];

                qq[dstOff] = qq[srcOff];
                qq[dstOff + 1] = qq[srcOff + 1];
                qq[dstOff + 2] = qq[srcOff + 2];
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
 * @param out output result object
 * @param transformAtoB transform matrix from shape A's local space to shape B's local space
 * @param shapeASupport support function for shape A (WITHOUT position/rotation transform)
 * @param shapeBSupport support function for shape B
 * @param displacement direction and distance to move shape A
 * @param tolerance convergence tolerance for GJK
 * @param convexRadiusA convex radius of shape A
 * @param convexRadiusB convex radius of shape B
 * @param maxLambda the max fraction along the sweep
 *
 * @optimize
 */
export function gjkCastShape(
    out: GjkCastShapeResult,
    transformAtoB: Mat4,
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
    setTransformedSupport(_transformedSupportA, transformAtoB, shapeASupport);

    _simplex.size = 0;

    let lambda = 0.0;

    // since A is already transformed we can start the cast from zero
    vec3.set(_x, 0, 0, 0);

    // v = -support_B + support_A (Minkowski difference B - A in the space of A)
    vec3.set(_directionB, 0, 0, 0);
    shapeBSupport.getSupport(_directionB, _q);
    vec3.negate(_q, _q);

    vec3.set(_directionA, 0, 0, 0);
    _transformedSupportA.getSupport(_directionA, _p);

    vec3.subtract(_v, _q, _p);

    let vLenSq = Number.MAX_VALUE;
    let allowRestart = false;

    // keeps track of separating axis of the previous iteration.
    // initialized at zero as we don't know if our first v is actually a separating axis.
    vec3.set(_prevV, 0, 0, 0);

    let iterations = 0;
    while (iterations < GJK_MAX_ITERATIONS) {
        iterations++;

        // calculate the minkowski difference B - A
        // A is moving, so we need to add the back side of B to the front side of A
        // keep the support points on A and B separate so that in the end we can calculate a contact point
        vec3.negate(_directionA, _v);
        _transformedSupportA.getSupport(_directionA, _p);

        vec3.copy(_directionB, _v);
        shapeBSupport.getSupport(_directionB, _q);

        vec3.subtract(_pq, _q, _p);
        vec3.subtract(_w, _x, _pq);

        // difference from article to this code:
        //
        // we did not include the convex radius in p and q in order to be able to calculate a good separating axis at the end of the algorithm.
        // however when moving forward along displacement we do need to take this into account so that we keep A and B separated by the sum of their convex radii.
        //
        // from p we have to subtract: convexRadiusA * v / |v|
        // to q we have to add: convexRadiusB * v / |v|
        // this means that to w we have to add: -(convexRadiusA + convexRadiusB) * v / |v|
        // so to v . w we have to add: v . (-(convexRadiusA + convexRadiusB) * v / |v|) = -(convexRadiusA + convexRadiusB) * |v|
        const vDotW = vec3.dot(_v, _w) - sumConvexRadius * Math.sqrt(vec3.squaredLength(_v));

        if (vDotW > 0.0) {
            // if ray and normal are in the same direction, we've passed A and there's no collision
            const vDotR = vec3.dot(_v, displacement);

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
            vec3.scale(_x, displacement, lambda);

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
        {
            const off = _simplex.size * 3;
            _simplex.p[off] = _p[0];
            _simplex.p[off + 1] = _p[1];
            _simplex.p[off + 2] = _p[2];

            _simplex.q[off] = _q[0];
            _simplex.q[off + 1] = _q[1];
            _simplex.q[off + 2] = _q[2];
            _simplex.size++;
        }

        // calculate Y = {x} - (Q - P)
        recomputeSimplexYFromPQ(_simplex, _x);

        // determine the new closest point from Y to origin
        const found = computeClosestPointToSimplex(_closestPointToSimplex, vLenSq, false, _simplex);

        if (found) {
            vLenSq = _closestPointToSimplex.squaredDistance;
            vec3.copy(_v, _closestPointToSimplex.point);
        }

        if (!found) {
            // only allow 1 restart, if we still can't get a closest point we're so close that we return this as a hit
            if (!allowRestart) {
                break;
            }

            // if we fail to converge, we start again with the last point as simplex
            allowRestart = false;

            _simplex.p[0] = _p[0];
            _simplex.p[1] = _p[1];
            _simplex.p[2] = _p[2];

            _simplex.q[0] = _q[0];
            _simplex.q[1] = _q[1];
            _simplex.q[2] = _q[2];

            _simplex.size = 1;

            vec3.subtract(_v, _x, _q);

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
        vec3.copy(_prevV, _v);
    }

    // calculate Y = {x} - (Q - P) again so we can calculate the contact points
    recomputeSimplexYFromPQ(_simplex, _x);

    // compute normalized v for separating axis
    const vLen = Math.sqrt(vec3.squaredLength(_v));
    if (vLen > 0) {
        vec3.scale(_normalizedV, _v, 1 / vLen);
    } else {
        vec3.set(_normalizedV, 0, 0, 0);
    }

    // compute contact points from simplex
    vec3.set(out.pointA, 0, 0, 0);
    vec3.set(out.pointB, 0, 0, 0);

    switch (_simplex.size) {
        case 1: {
            const pp = _simplex.p;
            const qq = _simplex.q;

            // out.pointB = q[0] + _normalizedV * convexRadiusB;
            out.pointB[0] = qq[0] + _normalizedV[0] * convexRadiusB;
            out.pointB[1] = qq[1] + _normalizedV[1] * convexRadiusB;
            out.pointB[2] = qq[2] + _normalizedV[2] * convexRadiusB;

            if (lambda > 0.0) {
                vec3.copy(out.pointA, out.pointB);
            } else {
                // out.pointA = p[0] + _normalizedV * -convexRadiusA;
                out.pointA[0] = pp[0] + _normalizedV[0] * -convexRadiusA;
                out.pointA[1] = pp[1] + _normalizedV[1] * -convexRadiusA;
                out.pointA[2] = pp[2] + _normalizedV[2] * -convexRadiusA;
            }
            break;
        }
        case 2: {
            const yy = _simplex.y;
            const pp = _simplex.p;
            const qq = _simplex.q;
            _simplexY0[0] = yy[0]; _simplexY0[1] = yy[1]; _simplexY0[2] = yy[2];
            _simplexY1[0] = yy[3]; _simplexY1[1] = yy[4]; _simplexY1[2] = yy[5];
            computeBarycentricCoordinates2d(_bary, _simplexY0, _simplexY1, 1e-10);

            // out.pointB += q[0] * u + q[1] * v + _normalizedV * convexRadiusB
            out.pointB[0] += qq[0] * _bary.u + qq[3] * _bary.v + _normalizedV[0] * convexRadiusB;
            out.pointB[1] += qq[1] * _bary.u + qq[4] * _bary.v + _normalizedV[1] * convexRadiusB;
            out.pointB[2] += qq[2] * _bary.u + qq[5] * _bary.v + _normalizedV[2] * convexRadiusB;

            if (lambda > 0.0) {
                vec3.copy(out.pointA, out.pointB);
            } else {
                out.pointA[0] += pp[0] * _bary.u + pp[3] * _bary.v + _normalizedV[0] * -convexRadiusA;
                out.pointA[1] += pp[1] * _bary.u + pp[4] * _bary.v + _normalizedV[1] * -convexRadiusA;
                out.pointA[2] += pp[2] * _bary.u + pp[5] * _bary.v + _normalizedV[2] * -convexRadiusA;
            }
            break;
        }
        case 3:
        case 4: {
            const yy = _simplex.y;
            const pp = _simplex.p;
            const qq = _simplex.q;
            _simplexY0[0] = yy[0]; _simplexY0[1] = yy[1]; _simplexY0[2] = yy[2];
            _simplexY1[0] = yy[3]; _simplexY1[1] = yy[4]; _simplexY1[2] = yy[5];
            _simplexY2[0] = yy[6]; _simplexY2[1] = yy[7]; _simplexY2[2] = yy[8];
            computeBarycentricCoordinates3d(_bary, _simplexY0, _simplexY1, _simplexY2, 1e-10);

            out.pointB[0] += qq[0] * _bary.u + qq[3] * _bary.v + qq[6] * _bary.w + _normalizedV[0] * convexRadiusB;
            out.pointB[1] += qq[1] * _bary.u + qq[4] * _bary.v + qq[7] * _bary.w + _normalizedV[1] * convexRadiusB;
            out.pointB[2] += qq[2] * _bary.u + qq[5] * _bary.v + qq[8] * _bary.w + _normalizedV[2] * convexRadiusB;

            if (lambda > 0.0) {
                vec3.copy(out.pointA, out.pointB);
            } else {
                out.pointA[0] += pp[0] * _bary.u + pp[3] * _bary.v + pp[6] * _bary.w + _normalizedV[0] * -convexRadiusA;
                out.pointA[1] += pp[1] * _bary.u + pp[4] * _bary.v + pp[7] * _bary.w + _normalizedV[1] * -convexRadiusA;
                out.pointA[2] += pp[2] * _bary.u + pp[5] * _bary.v + pp[8] * _bary.w + _normalizedV[2] * -convexRadiusA;
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
        vec3.negate(out.separatingAxis, _v);
    } else {
        vec3.negate(out.separatingAxis, _prevV);
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
 *
 * @optimize
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

    _simplex.size = 0;

    // length^2 of v
    vec3.copy(_closestPointToSimplex.point, direction);
    _closestPointToSimplex.squaredDistance = vec3.squaredLength(direction);
    _closestPointToSimplex.pointSet = 0;
    _closestPointToSimplex.closestPointFound = true;
    // previous length^2 of v
    let previousSquaredDistance = Number.MAX_VALUE;

    let iterations = 0;
    while (iterations++ < GJK_MAX_ITERATIONS) {
        // get support points for shape A and B in the direction of v
        vec3.copy(_directionA, _closestPointToSimplex.point);
        vec3.negate(_directionB, _closestPointToSimplex.point);

        supportA.getSupport(_directionA, _p);
        supportB.getSupport(_directionB, _q);

        // get support point of the minkowski sum A - B of v
        vec3.subtract(_w, _p, _q);

        const dot = vec3.dot(_closestPointToSimplex.point, _w);

        // test if we have a separation of more than inMaxDistSq, in which case we terminate early
        if (dot < 0.0 && dot * dot > _closestPointToSimplex.squaredDistance * maxDistanceSquared) {
            out.squaredDistance = Number.MAX_VALUE;
            vec3.copy(out.penetrationAxis, _closestPointToSimplex.point);
            return;
        }

        // store the point for later use
        const off = _simplex.size * 3;
        _simplex.y[off] = _w[0];
        _simplex.y[off + 1] = _w[1];
        _simplex.y[off + 2] = _w[2];

        _simplex.p[off] = _p[0];
        _simplex.p[off + 1] = _p[1];
        _simplex.p[off + 2] = _p[2];

        _simplex.q[off] = _q[0];
        _simplex.q[off + 1] = _q[1];
        _simplex.q[off + 2] = _q[2];

        _simplex.size++;

        computeClosestPointToSimplex(_closestPointToSimplex, previousSquaredDistance, true, _simplex);

        if (!_closestPointToSimplex.closestPointFound) {
            // remove last added point from simplex
            _simplex.size--;
            break;
        }

        // if there are 4 points, the origin is inside the tetrahedron and we're done
        if (_closestPointToSimplex.pointSet === 0xf) {
            vec3.set(_closestPointToSimplex.point, 0, 0, 0);
            _closestPointToSimplex.squaredDistance = 0.0;
            break;
        }

        // update the points of the simplex
        updatePointSetYPQ(_simplex, _closestPointToSimplex.pointSet);

        // if v is very close to zero, we consider this a collision
        if (_closestPointToSimplex.squaredDistance <= squaredTolerance) {
            vec3.set(_closestPointToSimplex.point, 0, 0, 0);
            _closestPointToSimplex.squaredDistance = 0.0;
            break;
        }

        // if v is very small compared to the length of y, we also consider this a collision
        let yMaxLengthSquared = 0;
        {
            const yy = _simplex.y;
            const end = _simplex.size * 3;
            for (let i = 0; i < end; i += 3) {
                const yx = yy[i];
                const yYy = yy[i + 1];
                const yz = yy[i + 2];
                const squaredLength = yx * yx + yYy * yYy + yz * yz;
                yMaxLengthSquared = Math.max(yMaxLengthSquared, squaredLength);
            }
        }

        if (_closestPointToSimplex.squaredDistance <= GJK_TOLERANCE * yMaxLengthSquared) {
            vec3.set(_closestPointToSimplex.point, 0, 0, 0);
            _closestPointToSimplex.squaredDistance = 0.0;
            break;
        }

        // the next separation axis to test is the negative of the closest point of the Minkowski sum to the origin
        // note: this must be done before terminating as converged since the separating axis is -v
        vec3.negate(_closestPointToSimplex.point, _closestPointToSimplex.point);

        // if the squared length of v is not changing enough, we've converged and there is no collision
        if (previousSquaredDistance - _closestPointToSimplex.squaredDistance <= GJK_TOLERANCE * previousSquaredDistance) {
            // v is a separating axis
            break;
        }

        previousSquaredDistance = _closestPointToSimplex.squaredDistance;
    }

    // extract the closest points
    vec3.set(out.pointA, 0, 0, 0);
    vec3.set(out.pointB, 0, 0, 0);

    copySimplex(out.simplex, _simplex);

    // handle early termination case: if simplex is empty, GJK terminated early (shapes are far apart)
    // return a large distance to indicate separation
    if (_simplex.size === 0) {
        out.squaredDistance = Number.MAX_VALUE;
        vec3.set(out.penetrationAxis, 0, 0, 0);
        return;
    }

    switch (_simplex.size) {
        case 1: {
            // single point in simplex
            const pp = _simplex.p;
            const qq = _simplex.q;

            out.pointA[0] = pp[0];
            out.pointA[1] = pp[1];
            out.pointA[2] = pp[2];

            out.pointB[0] = qq[0];
            out.pointB[1] = qq[1];
            out.pointB[2] = qq[2];
            break;
        }

        case 2: {
            // line segment in simplex
            const yy = _simplex.y;
            const pp = _simplex.p;
            const qq = _simplex.q;
            _simplexY0[0] = yy[0]; _simplexY0[1] = yy[1]; _simplexY0[2] = yy[2];
            _simplexY1[0] = yy[3]; _simplexY1[1] = yy[4]; _simplexY1[2] = yy[5];
            computeBarycentricCoordinates2d(_bary, _simplexY0, _simplexY1, 1e-10);

            out.pointA[0] = pp[0] * _bary.u + pp[3] * _bary.v;
            out.pointA[1] = pp[1] * _bary.u + pp[4] * _bary.v;
            out.pointA[2] = pp[2] * _bary.u + pp[5] * _bary.v;

            out.pointB[0] = qq[0] * _bary.u + qq[3] * _bary.v;
            out.pointB[1] = qq[1] * _bary.u + qq[4] * _bary.v;
            out.pointB[2] = qq[2] * _bary.u + qq[5] * _bary.v;
            break;
        }

        case 3: {
            // triangle in simplex
            const yy = _simplex.y;
            const pp = _simplex.p;
            const qq = _simplex.q;
            _simplexY0[0] = yy[0]; _simplexY0[1] = yy[1]; _simplexY0[2] = yy[2];
            _simplexY1[0] = yy[3]; _simplexY1[1] = yy[4]; _simplexY1[2] = yy[5];
            _simplexY2[0] = yy[6]; _simplexY2[1] = yy[7]; _simplexY2[2] = yy[8];
            computeBarycentricCoordinates3d(_bary, _simplexY0, _simplexY1, _simplexY2, 1e-10);

            out.pointA[0] = pp[0] * _bary.u + pp[3] * _bary.v + pp[6] * _bary.w;
            out.pointA[1] = pp[1] * _bary.u + pp[4] * _bary.v + pp[7] * _bary.w;
            out.pointA[2] = pp[2] * _bary.u + pp[5] * _bary.v + pp[8] * _bary.w;

            out.pointB[0] = qq[0] * _bary.u + qq[3] * _bary.v + qq[6] * _bary.w;
            out.pointB[1] = qq[1] * _bary.u + qq[4] * _bary.v + qq[7] * _bary.w;
            out.pointB[2] = qq[2] * _bary.u + qq[5] * _bary.v + qq[8] * _bary.w;
            break;
        }

        default: {
            // for a full simplex (4 points = tetrahedron), the origin is inside the Minkowski difference
            // in this case, pointA and pointB remain at zero (as initialized)
            break;
        }
    }

    // store the separating axis / penetration axis
    vec3.copy(out.penetrationAxis, _closestPointToSimplex.point);
    out.squaredDistance = _closestPointToSimplex.squaredDistance;
}
