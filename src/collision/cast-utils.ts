export const INITIAL_EARLY_OUT_FRACTION = 1.0 + 1e-4;
export const SHOULD_EARLY_OUT_FRACTION = 0.0;

/** result of a {@link rayIntersectsTriangle} test */
export type RayIntersectsTriangleResult = {
    hit: boolean;
    /** fraction along the ray (0-1) where the hit occurred; 0 when no hit */
    fraction: number;
    /** true when the triangle's front face was hit */
    frontFacing: boolean;
};

export function createRayIntersectsTriangleResult(): RayIntersectsTriangleResult {
    return {
        hit: false,
        fraction: 0,
        frontFacing: false,
    };
}

/**
 * Ray-triangle intersection test with scalar ray and vertex args (no ray struct, no Vec3 copies).
 * Based on https://github.com/pmjoniak/GeometricTools/blob/master/GTEngine/Include/Mathematics/GteIntrRay3Triangle3.h
 *
 * `maxT` is the largest hit distance still of interest (ray units, at most `length`): a closest-hit
 * traversal passes the current best hit so triangles behind it are rejected before the divide.
 */
export function rayIntersectsTriangle(
    out: RayIntersectsTriangleResult,
    originX: number,
    originY: number,
    originZ: number,
    directionX: number,
    directionY: number,
    directionZ: number,
    length: number,
    maxT: number,
    ax: number,
    ay: number,
    az: number,
    bx: number,
    by: number,
    bz: number,
    cx: number,
    cy: number,
    cz: number,
    backfaceCulling: boolean,
): void {
    // edge1 = b - a, edge2 = c - a
    const e1x = bx - ax;
    const e1y = by - ay;
    const e1z = bz - az;
    const e2x = cx - ax;
    const e2y = cy - ay;
    const e2z = cz - az;

    // normal = edge1 × edge2
    const nx = e1y * e2z - e1z * e2y;
    const ny = e1z * e2x - e1x * e2z;
    const nz = e1x * e2y - e1y * e2x;

    // determine front vs back facing
    let DdN = directionX * nx + directionY * ny + directionZ * nz;
    let sign: number;
    if (DdN > 0) {
        // backface
        if (backfaceCulling) {
            out.hit = false;
            out.fraction = 0;
            out.frontFacing = false;
            return;
        }
        sign = 1;
    } else if (DdN < 0) {
        // frontface
        sign = -1;
        DdN = -DdN;
    } else {
        // ray is parallel to triangle
        out.hit = false;
        out.fraction = 0;
        out.frontFacing = false;
        return;
    }

    // diff = origin - a
    const diffx = originX - ax;
    const diffy = originY - ay;
    const diffz = originZ - az;

    // barycentric coordinate b1: DdQxE2 = sign * D · (diff × edge2)
    const diffCrossE2x = diffy * e2z - diffz * e2y;
    const diffCrossE2y = diffz * e2x - diffx * e2z;
    const diffCrossE2z = diffx * e2y - diffy * e2x;
    const DdQxE2 = sign * (directionX * diffCrossE2x + directionY * diffCrossE2y + directionZ * diffCrossE2z);
    if (DdQxE2 < 0) {
        out.hit = false;
        out.fraction = 0;
        out.frontFacing = false;
        return;
    }

    // barycentric coordinate b2: DdE1xQ = sign * D · (edge1 × diff)
    const e1CrossDiffx = e1y * diffz - e1z * diffy;
    const e1CrossDiffy = e1z * diffx - e1x * diffz;
    const e1CrossDiffz = e1x * diffy - e1y * diffx;
    const DdE1xQ = sign * (directionX * e1CrossDiffx + directionY * e1CrossDiffy + directionZ * e1CrossDiffz);
    if (DdE1xQ < 0) {
        out.hit = false;
        out.fraction = 0;
        out.frontFacing = false;
        return;
    }

    // b1 + b2 must not exceed 1
    if (DdQxE2 + DdE1xQ > DdN) {
        out.hit = false;
        out.fraction = 0;
        out.frontFacing = false;
        return;
    }

    // intersection distance along the ray
    const QdN = -sign * (diffx * nx + diffy * ny + diffz * nz);
    if (QdN < 0) {
        out.hit = false;
        out.fraction = 0;
        out.frontFacing = false;
        return;
    }

    // t = QdN / DdN; reject against the caller's bound before dividing (DdN > 0 here)
    if (QdN > DdN * maxT) {
        out.hit = false;
        out.fraction = 0;
        out.frontFacing = false;
        return;
    }

    out.hit = true;
    out.fraction = QdN / DdN / length;
    out.frontFacing = sign < 0;
}

/**
 * Reciprocal of a ray displacement component for {@link rayFractionToBox3}, computed once per query.
 * A (nearly) zero component yields 0, which no finite displacement can produce, and marks the axis
 * as parallel: the slab test then checks the origin against the slab instead of multiplying.
 */
export function safeReciprocal(d: number): number {
    return d > 1e-30 || d < -1e-30 ? 1 / d : 0;
}

/**
 * Entry fraction of a ray segment into a box, in [0, 1] of the segment, or Infinity when the segment
 * misses the box or the box lies entirely behind the origin.
 *
 * The ray is given as an origin and the reciprocals of its displacement (`direction * length`,
 * see {@link safeReciprocal}), precomputed once per query, so the per-box work is six multiplies and
 * the compares: no divides, and the parallel-axis branches are constant per query. Box args are
 * scalars so callers read them straight out of flat node arrays.
 */
export function rayFractionToBox3(
    originX: number,
    originY: number,
    originZ: number,
    invDispX: number,
    invDispY: number,
    invDispZ: number,
    minX: number,
    minY: number,
    minZ: number,
    maxX: number,
    maxY: number,
    maxZ: number,
): number {
    let tMin = 0;
    let tMax = 1;

    if (invDispX === 0) {
        // parallel to the slab: the segment stays at originX, which must lie inside it
        if (originX < minX || originX > maxX) return Infinity;
    } else {
        const t0 = (minX - originX) * invDispX;
        const t1 = (maxX - originX) * invDispX;
        if (t0 < t1) {
            if (t0 > tMin) tMin = t0;
            if (t1 < tMax) tMax = t1;
        } else {
            if (t1 > tMin) tMin = t1;
            if (t0 < tMax) tMax = t0;
        }
        if (tMax < tMin) return Infinity;
    }

    if (invDispY === 0) {
        // parallel to the slab: the segment stays at originY, which must lie inside it
        if (originY < minY || originY > maxY) return Infinity;
    } else {
        const t0 = (minY - originY) * invDispY;
        const t1 = (maxY - originY) * invDispY;
        if (t0 < t1) {
            if (t0 > tMin) tMin = t0;
            if (t1 < tMax) tMax = t1;
        } else {
            if (t1 > tMin) tMin = t1;
            if (t0 < tMax) tMax = t0;
        }
        if (tMax < tMin) return Infinity;
    }

    if (invDispZ === 0) {
        // parallel to the slab: the segment stays at originZ, which must lie inside it
        if (originZ < minZ || originZ > maxZ) return Infinity;
    } else {
        const t0 = (minZ - originZ) * invDispZ;
        const t1 = (maxZ - originZ) * invDispZ;
        if (t0 < t1) {
            if (t0 > tMin) tMin = t0;
            if (t1 < tMax) tMax = t1;
        } else {
            if (t1 > tMin) tMin = t1;
            if (t0 < tMax) tMax = t0;
        }
        if (tMax < tMin) return Infinity;
    }

    return tMin;
}

/**
 * Compute normalized distance fraction along the ray to a box's entry point. Fully-scalar box args
 * (matching {@link rayHitsBox3}) so callers pass components straight from any storage — flat node
 * arrays (`bounds[base + k]`), transformed/expanded boxes, or a Box3 (`box[k]`) — with no scratch
 * copy, and so compilecat inlines the slab math with no array indexing inside the body.
 *
 * @returns Normalized distance (0-1) to the box entry point, or Infinity if the ray misses the box
 *          or the box is behind the origin.
 */
export function rayDistanceToBox3(
    originX: number,
    originY: number,
    originZ: number,
    directionX: number,
    directionY: number,
    directionZ: number,
    length: number,
    minX: number,
    minY: number,
    minZ: number,
    maxX: number,
    maxY: number,
    maxZ: number,
): number {
    let tMin = 0;
    let tMax = length;

    // unrolled loop for x axis
    if (Math.abs(directionX) < 1e-10) {
        // ray is parallel to slab - check if origin is within slab
        if (originX < minX || originX > maxX) {
            return Infinity;
        }
    } else {
        const invD = 1.0 / directionX;
        const t0 = (minX - originX) * invD;
        const t1 = (maxX - originX) * invD;

        const tNear = t0 < t1 ? t0 : t1;
        const tFar = t0 < t1 ? t1 : t0;

        tMin = tNear > tMin ? tNear : tMin;
        tMax = tFar < tMax ? tFar : tMax;

        if (tMax < tMin) {
            return Infinity;
        }
    }

    // unrolled loop for y axis
    if (Math.abs(directionY) < 1e-10) {
        if (originY < minY || originY > maxY) {
            return Infinity;
        }
    } else {
        const invD = 1.0 / directionY;
        const t0 = (minY - originY) * invD;
        const t1 = (maxY - originY) * invD;

        const tNear = t0 < t1 ? t0 : t1;
        const tFar = t0 < t1 ? t1 : t0;

        tMin = tNear > tMin ? tNear : tMin;
        tMax = tFar < tMax ? tFar : tMax;

        if (tMax < tMin) {
            return Infinity;
        }
    }

    // unrolled loop for z axis
    if (Math.abs(directionZ) < 1e-10) {
        if (originZ < minZ || originZ > maxZ) {
            return Infinity;
        }
    } else {
        const invD = 1.0 / directionZ;
        const t0 = (minZ - originZ) * invD;
        const t1 = (maxZ - originZ) * invD;

        const tNear = t0 < t1 ? t0 : t1;
        const tFar = t0 < t1 ? t1 : t0;

        tMin = tNear > tMin ? tNear : tMin;
        tMax = tFar < tMax ? tFar : tMax;

        if (tMax < tMin) {
            return Infinity;
        }
    }

    return tMin >= 0 ? tMin / length : Infinity;
}

/**
 * Ray-AABB slab test. Returns true iff the ray segment intersects the box.
 *
 * Authored in early-return form — clearer to read and the natural shape for
 * an exit-on-miss slab test. compilecat's BLOCK inliner rewrites the early
 * returns into labeled-break exits at each call site.
 */
export function rayHitsBox3(
    originX: number,
    originY: number,
    originZ: number,
    dirX: number,
    dirY: number,
    dirZ: number,
    length: number,
    minX: number,
    minY: number,
    minZ: number,
    maxX: number,
    maxY: number,
    maxZ: number,
): boolean {
    let tNear = 0;
    let tFar = length;

    if (Math.abs(dirX) < 1e-10) {
        if (originX < minX || originX > maxX) return false;
    } else {
        const invX = 1 / dirX;
        let tEnterX = (minX - originX) * invX;
        let tExitX = (maxX - originX) * invX;
        if (invX < 0) {
            const tmp = tEnterX;
            tEnterX = tExitX;
            tExitX = tmp;
        }
        if (tEnterX > tNear) tNear = tEnterX;
        if (tExitX < tFar) tFar = tExitX;
        if (tFar < tNear) return false;
    }

    if (Math.abs(dirY) < 1e-10) {
        if (originY < minY || originY > maxY) return false;
    } else {
        const invY = 1 / dirY;
        let tEnterY = (minY - originY) * invY;
        let tExitY = (maxY - originY) * invY;
        if (invY < 0) {
            const tmp = tEnterY;
            tEnterY = tExitY;
            tExitY = tmp;
        }
        if (tEnterY > tNear) tNear = tEnterY;
        if (tExitY < tFar) tFar = tExitY;
        if (tFar < tNear) return false;
    }

    if (Math.abs(dirZ) < 1e-10) {
        if (originZ < minZ || originZ > maxZ) return false;
    } else {
        const invZ = 1 / dirZ;
        let tEnterZ = (minZ - originZ) * invZ;
        let tExitZ = (maxZ - originZ) * invZ;
        if (invZ < 0) {
            const tmp = tEnterZ;
            tEnterZ = tExitZ;
            tExitZ = tmp;
        }
        if (tEnterZ > tNear) tNear = tEnterZ;
        if (tExitZ < tFar) tFar = tExitZ;
        if (tFar < tNear) return false;
    }

    return true;
}
