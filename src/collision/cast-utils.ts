import type { Box3 } from 'mathcat';

export const INITIAL_EARLY_OUT_FRACTION = 1.0 + 1e-4;
export const SHOULD_EARLY_OUT_FRACTION = 0.0;

/**
 * Compute distance fraction along ray to box entry point.
 * Returns Infinity if ray doesn't intersect box or if box is behind ray origin.
 *
 * @param originX Ray origin X coordinate
 * @param originY Ray origin Y coordinate
 * @param originZ Ray origin Z coordinate
 * @param directionX Ray direction X component
 * @param directionY Ray direction Y component
 * @param directionZ Ray direction Z component
 * @param length Ray length
 * @param box The bounding box to test against
 * @returns Normalized distance (0-1) to box entry point, or Infinity if no hit
 */
export function rayDistanceToBox3(
    originX: number,
    originY: number,
    originZ: number,
    directionX: number,
    directionY: number,
    directionZ: number,
    length: number,
    box: Box3,
): number {
    const boxMinX = box[0];
    const boxMinY = box[1];
    const boxMinZ = box[2];
    const boxMaxX = box[3];
    const boxMaxY = box[4];
    const boxMaxZ = box[5];

    let tMin = 0;
    let tMax = length;

    // unrolled loop for x axis
    if (Math.abs(directionX) < 1e-10) {
        // ray is parallel to slab - check if origin is within slab
        if (originX < boxMinX || originX > boxMaxX) {
            return Infinity;
        }
    } else {
        const invD = 1.0 / directionX;
        const t0 = (boxMinX - originX) * invD;
        const t1 = (boxMaxX - originX) * invD;

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
        if (originY < boxMinY || originY > boxMaxY) {
            return Infinity;
        }
    } else {
        const invD = 1.0 / directionY;
        const t0 = (boxMinY - originY) * invD;
        const t1 = (boxMaxY - originY) * invD;

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
        if (originZ < boxMinZ || originZ > boxMaxZ) {
            return Infinity;
        }
    } else {
        const invD = 1.0 / directionZ;
        const t0 = (boxMinZ - originZ) * invD;
        const t1 = (boxMaxZ - originZ) * invD;

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
/* @inline */
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
