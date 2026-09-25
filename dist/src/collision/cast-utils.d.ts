export declare const INITIAL_EARLY_OUT_FRACTION: number;
export declare const SHOULD_EARLY_OUT_FRACTION = 0;
/** result of a {@link rayIntersectsTriangle} test */
export type RayIntersectsTriangleResult = {
    hit: boolean;
    /** fraction along the ray (0-1) where the hit occurred; 0 when no hit */
    fraction: number;
    /** true when the triangle's front face was hit */
    frontFacing: boolean;
};
export declare function createRayIntersectsTriangleResult(): RayIntersectsTriangleResult;
/**
 * Ray-triangle intersection test with scalar ray and vertex args (no ray struct, no Vec3 copies).
 * Based on https://github.com/pmjoniak/GeometricTools/blob/master/GTEngine/Include/Mathematics/GteIntrRay3Triangle3.h
 *
 * `maxT` is the largest hit distance still of interest (ray units, at most `length`): a closest-hit
 * traversal passes the current best hit so triangles behind it are rejected before the divide.
 */
export declare function rayIntersectsTriangle(out: RayIntersectsTriangleResult, originX: number, originY: number, originZ: number, directionX: number, directionY: number, directionZ: number, length: number, maxT: number, ax: number, ay: number, az: number, bx: number, by: number, bz: number, cx: number, cy: number, cz: number, backfaceCulling: boolean): void;
/**
 * Reciprocal of a ray displacement component for {@link rayFractionToBox3}, computed once per query.
 * A (nearly) zero component yields 0, which no finite displacement can produce, and marks the axis
 * as parallel: the slab test then checks the origin against the slab instead of multiplying.
 */
export declare function safeReciprocal(d: number): number;
/**
 * Entry fraction of a ray segment into a box, in [0, 1] of the segment, or Infinity when the segment
 * misses the box or the box lies entirely behind the origin.
 *
 * The ray is given as an origin and the reciprocals of its displacement (`direction * length`,
 * see {@link safeReciprocal}), precomputed once per query, so the per-box work is six multiplies and
 * the compares: no divides, and the parallel-axis branches are constant per query. Box args are
 * scalars so callers read them straight out of flat node arrays.
 */
export declare function rayFractionToBox3(originX: number, originY: number, originZ: number, invDispX: number, invDispY: number, invDispZ: number, minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): number;
/**
 * Compute normalized distance fraction along the ray to a box's entry point. Fully-scalar box args
 * (matching {@link rayHitsBox3}) so callers pass components straight from any storage — flat node
 * arrays (`bounds[base + k]`), transformed/expanded boxes, or a Box3 (`box[k]`) — with no scratch
 * copy, and so compilecat inlines the slab math with no array indexing inside the body.
 *
 * @returns Normalized distance (0-1) to the box entry point, or Infinity if the ray misses the box
 *          or the box is behind the origin.
 */
export declare function rayDistanceToBox3(originX: number, originY: number, originZ: number, directionX: number, directionY: number, directionZ: number, length: number, minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): number;
/**
 * Ray-AABB slab test. Returns true iff the ray segment intersects the box.
 *
 * Authored in early-return form — clearer to read and the natural shape for
 * an exit-on-miss slab test. compilecat's BLOCK inliner rewrites the early
 * returns into labeled-break exits at each call site.
 */
export declare function rayHitsBox3(originX: number, originY: number, originZ: number, dirX: number, dirY: number, dirZ: number, length: number, minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): boolean;
