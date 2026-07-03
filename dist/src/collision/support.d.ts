import { type Mat4, type Vec3 } from 'mathcat';
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
 * Monomorphic support evaluation.
 *
 * A single {@link Support} struct (one hidden class) is filled once per collision pair, then
 * {@link getSupport} — a single, monomorphic function — is called many times per pair by GJK/EPA.
 * The per-shape polymorphism lives entirely in the fill (cold, once per pair); the hot path is one
 * function with a `switch` on `kind`.
 *
 * Radius contract:
 *  - `convexRadius` is the *reported* radius. `getSupport` never adds it; the collision driver
 *    reads it and passes it to `gjkClosestPoints`/EPA for the shrunk-core-plus-radius distance math.
 *  - `addRadius` is an extra radius added along the (local) direction by `getSupport` itself (the EPA
 *    speculative-separation / cast convex radius). 0 on the GJK path.
 *  - "mode" (include vs exclude convex radius) is baked into the params by the fill: exclude uses the
 *    shrunk core + reports `convexRadius`; include uses the full/rounded core + `convexRadius = 0`.
 */
export declare enum SupportKind {
    BOX = 0,
    SPHERE = 1,
    CAPSULE = 2,
    CYLINDER = 3,
    HULL = 4,
    TRIANGLE = 5,
    POINT = 6
}
export type Support = {
    /** which sub-object holds this support's parameters; selects the branch taken in {@link getSupport} */
    kind: SupportKind;
    /** reported convex radius — read by the driver, never added by getSupport (0 in include mode) */
    convexRadius: number;
    /** extra radius added along the local direction by getSupport (EPA separation / cast radius) */
    addRadius: number;
    /** B-in-A transform, applied when hasTransform is true (identity otherwise) */
    hasTransform: boolean;
    transform: Mat4;
    /** axis-aligned box: support is the corner picked by the sign of the direction on each axis (±halfExtents) */
    box: {
        halfExtents: Vec3;
    };
    /**
     * sphere as a single radius. 0 → the core is the origin (exclude mode: a sphere is pure convex radius);
     * r → the rounded surface point `r·dir̂` (include mode).
     */
    sphere: {
        radius: number;
    };
    /**
     * capsule as a segment of half-length `halfHeight` along local Y, optionally rounded by `radius`.
     * radius 0 → the bare segment endpoint (exclude); r → segment endpoint + `r·dir̂` (include).
     */
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
    /** convex vertex set — support is the vertex with the greatest dot product against the direction */
    hull: {
        /** flat `[x,y,z,...]` vertices scanned by getSupport; read-only borrow valid for the current pair */
        vertices: number[];
        /** number of vertices in `vertices` (it may be longer than `vertexCount * 3`) */
        vertexCount: number;
        /** per-vertex output scale — the winning vertex is multiplied by this (uniform-scale fast path); 1 when `vertices` are already baked */
        outputScale: number;
        /** owned grow-once buffer that `vertices` points at for the non-uniform scaled slow path */
        scratch: number[];
    };
    /** triangle (mesh face) — support is whichever of the three vertices has the greatest dot with the direction */
    triangle: {
        a: Vec3;
        b: Vec3;
        c: Vec3;
    };
    /** single point — the support is always this point, regardless of direction */
    point: {
        position: Vec3;
    };
};
/**
 * Allocate a reusable {@link Support}. A driver holds a small fixed number of these (e.g. one per
 * operand slot) and refills them per pair via the fill functions. The sub-objects, transform, and
 * scratch buffer are pre-allocated so filling never allocates.
 */
export declare function createSupport(): Support;
/**
 * Evaluate the support point of `support` in direction `direction`, writing it to `out`.
 * The single hot GJK/EPA call site — monomorphic.
 */
export declare function getSupport(out: Vec3, support: Support, direction: Vec3): void;
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
 * arrays and scales the support point in getSupport (fast path); non-uniform / mirrored scale bakes
 * scaled vertices into scratch per pair (slow path). `vertices` is a read-only borrow valid for the
 * current pair.
 */
export declare function setHullSupport(out: Support, shape: ConvexHullShape, mode: SupportFunctionMode, scale: Vec3): void;
