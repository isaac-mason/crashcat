import { type Mat4, type Vec3 } from 'math';
import type { BoxShape } from '../shapes/box.js';
import type { CapsuleShape } from '../shapes/capsule.js';
import type { ConvexHullShape } from '../shapes/convex-hull.js';
import type { CylinderShape } from '../shapes/cylinder.js';
import type { SphereShape } from '../shapes/sphere.js';
export declare const DEFAULT_CONVEX_RADIUS = 0.05;
export declare enum SupportFunctionMode {
    INCLUDE_CONVEX_RADIUS = 0,
    EXCLUDE_CONVEX_RADIUS = 1,
    DEFAULT = 2
}
/**
 * Support evaluation through the struct.
 *
 * A single {@link Support} struct (one hidden class) is filled once per collision pair. the fill
 * installs the shape's evaluator on the struct next to its parameters, and gjk/epa call it many
 * times per pair as `support.getSupport(out, support, direction)`. there is no kind tag and no
 * switch: the evaluator *is* the kind.
 *
 * Every evaluator is self-contained — it rotates the direction into local space, finds the core
 * support, adds the radius along the local direction and transforms the point back — so each
 * evaluation is one straight-line optimisation unit with nothing shared across a call boundary.
 *
 * Radius contract:
 *  - `convexRadius` is the *reported* radius. no evaluator adds it; the collision driver reads it and
 *    passes it to `gjkClosestPoints`/EPA for the shrunk-core-plus-radius distance math.
 *  - `addRadius` is an extra radius added along the (local) direction by the evaluator itself (the
 *    EPA speculative-separation / cast convex radius). 0 on the GJK path.
 *  - "mode" (include vs exclude convex radius) is baked into the params by the fill: exclude uses the
 *    shrunk core + reports `convexRadius`; include uses the full/rounded core + `convexRadius = 0`.
 */
/** evaluate the support point of `support` in direction `direction`, writing it to `out` */
export type SupportFunction = (out: Vec3, support: Support, direction: Vec3) => void;
export type Support = {
    /** the shape's evaluator, installed by the fill. the hot call site is `support.getSupport(out, support, direction)` */
    getSupport: SupportFunction;
    /** reported convex radius — read by the driver, never added by the evaluator (0 in include mode) */
    convexRadius: number;
    /** extra radius added along the local direction by the evaluator (EPA separation / cast radius) */
    addRadius: number;
    /** whether the support has a transform */
    hasTransform: boolean;
    /** B-in-A transform, applied when hasTransform is true (identity otherwise) */
    transform: Mat4;
    /** axis-aligned box: support is the corner picked by the sign of the direction on each axis (±halfExtents) */
    box: {
        halfExtents: Vec3;
    };
    /** sphere core: the origin, rounded by `radius`. 0 in exclude mode, where the radius is reported instead. */
    sphere: {
        radius: number;
    };
    /** capsule core: a segment of half-length `halfHeight` along local Y, rounded by `radius` (0 in exclude mode) */
    capsule: {
        halfHeight: number;
        radius: number;
    };
    /**
     * cylinder: `radius` is the radial extent in the local XZ plane, `halfHeight` the axial extent along
     * local Y. support = the radial extreme (`radius·dir̂ₓ_z`) combined with the near/far axial cap.
     */
    cylinder: {
        radius: number;
        halfHeight: number;
    };
    /** triangle (mesh operand): three vertices as a flat `[ax,ay,az, bx,by,bz, cx,cy,cz]` buffer, owned and copied into by the fill */
    triangle: {
        vertices: number[];
    };
    /** single point (collidePoint operand), owned and copied into by the fill */
    point: Vec3;
    /** convex vertex set (a hull, or a borrowed polygon face) — support is the vertex with the greatest dot product against the direction */
    hull: {
        /** flat `[x,y,z,...]` vertices scanned by the evaluator; read-only borrow valid for the current pair */
        vertices: number[];
        /** number of vertices in `vertices` (it may be longer than `vertexCount * 3`) */
        vertexCount: number;
        /** per-vertex output scale — the winning vertex is multiplied by this (uniform-scale fast path); 1 when `vertices` are already baked */
        outputScale: number;
        /** owned grow-once buffer that `vertices` points at for the non-uniform scaled slow path */
        scratch: number[];
        /** borrowed CSR neighbour prefix offsets (shape-owned, length numPoints+1); empty ⇔ brute scan */
        neighborsStart: number[];
        /** borrowed CSR flat neighbour indices (shape-owned), indexed via `neighborsStart` */
        neighbors: number[];
        /** warm-start hint: the last winning vertex index, carried across support calls within one pair; -1 = cold */
        lastVertex: number;
    };
};
/**
 * Allocate a reusable {@link Support}. A driver holds a small fixed number of these (e.g. one per
 * operand slot) and refills them per pair via the fill functions. The sub-objects, transform, and
 * scratch buffer are pre-allocated so filling never allocates. starts as a zero-radius sphere at the origin.
 */
export declare function createSupport(): Support;
/**
 * sphere evaluator. a sphere's support is rotation invariant: R·(r·dir̂_local) where
 * dir̂_local = Rᵀ·dir̂ is just r·dir̂, so the direction transform and the rotation half of the
 * transform-back cancel and only the translation survives. addRadius is applied along that same
 * direction, so it folds into the radius.
 */
export declare function sphereSupport(out: Vec3, support: Support, direction: Vec3): void;
/** box evaluator: the corner picked by the sign of the local direction on each axis */
export declare function boxSupport(out: Vec3, support: Support, direction: Vec3): void;
/** capsule evaluator: the near end of the core segment, rounded by the capsule radius */
export declare function capsuleSupport(out: Vec3, support: Support, direction: Vec3): void;
/** cylinder evaluator: the radial extreme in the local XZ plane on the near/far cap */
export declare function cylinderSupport(out: Vec3, support: Support, direction: Vec3): void;
/** triangle evaluator: three points with a last-maximal tie-break — mesh contact quality depends on it */
export declare function triangleSupport(out: Vec3, support: Support, direction: Vec3): void;
/** point evaluator: the point itself, rounded by addRadius along the local direction */
export declare function pointSupport(out: Vec3, support: Support, direction: Vec3): void;
/**
 * convex point-set evaluator: a hull or a polygon face. brute scan when no adjacency is baked or
 * the pair is cold; otherwise a warm hill-climb over the baked 1-ring.
 */
export declare function hullSupport(out: Vec3, support: Support, direction: Vec3): void;
export declare function setBoxSupport(out: Support, shape: BoxShape, mode: SupportFunctionMode, scale: Vec3): void;
export declare function setSphereSupport(out: Support, shape: SphereShape, mode: SupportFunctionMode, scale: Vec3): void;
export declare function setCapsuleSupport(out: Support, shape: CapsuleShape, mode: SupportFunctionMode, scale: Vec3): void;
export declare function setCylinderSupport(out: Support, shape: CylinderShape, mode: SupportFunctionMode, scale: Vec3): void;
/** triangle operand (mesh) — copies the 3 verts */
export declare function setTriangleSupport(out: Support, a: Vec3, b: Vec3, c: Vec3): void;
/** polygon face (KCC) — borrows the face's vertex array (read-only, valid for this pair) */
export declare function setPolygonSupport(out: Support, vertices: number[], vertexCount: number): void;
/** point operand (collidePoint) — copies the point */
export declare function setPointSupport(out: Support, point: Vec3): void;
/**
 * Compute the convex-radius-shrunk hull vertices (unscaled) into `dst` as a flat [x,y,z,...] array.
 * Each neighbouring face plane is offset inward by the convex radius (constant += r) and the up-to-3
 * planes are intersected (Cramer's rule). For a 2-face vertex the third plane is perpendicular to the
 * first two through the vertex; its `n1 × n2` normal is left unnormalized (the intersection is
 * invariant to per-plane scale).
 *
 * Computed once per shape at create time (see convex-hull.ts) and borrowed by the exclude-mode fill,
 * so `convexRadius` is passed in explicitly rather than read off the (not-yet-built) shape.
 */
export declare function computeShrunkHullPoints(shape: Pick<ConvexHullShape, 'numPoints' | 'pointPositions' | 'pointNumFaces' | 'pointFaces' | 'planes'>, convexRadius: number, dst: number[]): void;
/**
 * Fill a HULL support for the given mode + scale. Include (or zero-radius) uses the raw vertices;
 * exclude uses the convex-radius-shrunk vertices. Uniform positive scale borrows the shape-owned
 * arrays and scales the support point in the evaluator (fast path); non-uniform / mirrored scale bakes
 * scaled vertices into scratch per pair (slow path). `vertices` is a read-only borrow valid for the
 * current pair.
 */
export declare function setHullSupport(out: Support, shape: ConvexHullShape, mode: SupportFunctionMode, scale: Vec3): void;
