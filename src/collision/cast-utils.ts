import type { Box3, Raycast3 } from 'mathcat';

export const INITIAL_EARLY_OUT_FRACTION = 1.0 + 1e-4;
export const SHOULD_EARLY_OUT_FRACTION = 0.0;

/**
 * Compute distance fraction along ray to box entry point.
 * Returns Infinity if ray doesn't intersect box or if box is behind ray origin.
 * 
 * @param ray The ray to test
 * @param box The bounding box to test against
 * @returns Normalized distance (0-1) to box entry point, or Infinity if no hit
 */
export function rayDistanceToBox3(ray: Raycast3, box: Box3): number {
    const origin = ray.origin;
    const dir = ray.direction;
    const rayLength = ray.length;
    const boxMin = box[0];
    const boxMax = box[1];
    
    let tMin = 0;
    let tMax = rayLength;
    
    for (let i = 0; i < 3; i++) {
        if (Math.abs(dir[i]) < 1e-10) {
            // ray is parallel to slab - check if origin is within slab
            if (origin[i] < boxMin[i] || origin[i] > boxMax[i]) {
                return Infinity;
            }
        } else {
            const invD = 1.0 / dir[i];
            const t0 = (boxMin[i] - origin[i]) * invD;
            const t1 = (boxMax[i] - origin[i]) * invD;
            
            // compute min/max without conditional swap
            const tNear = t0 < t1 ? t0 : t1;
            const tFar = t0 < t1 ? t1 : t0;
            
            tMin = tNear > tMin ? tNear : tMin;
            tMax = tFar < tMax ? tFar : tMax;
            
            if (tMax < tMin) {
                return Infinity;
            }
        }
    }
    
    return tMin >= 0 ? tMin / rayLength : Infinity;
}