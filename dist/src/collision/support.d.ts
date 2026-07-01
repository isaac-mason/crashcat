import { type Mat4, type Vec3 } from 'mathcat';
import type { BoxShape } from '../shapes/box.js';
import type { CapsuleShape } from '../shapes/capsule.js';
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
 * Radius contract (mirrors the legacy support objects exactly):
 *  - `convexRadius` is the *reported* radius. `getSupport` never adds it; the collision driver
 *    reads it and passes it to `gjkClosestPoints`/EPA for the shrunk-core-plus-radius distance math.
 *  - `addRadius` is the folded `AddConvexRadius` wrapper: an amount added along the (local) direction
 *    by `getSupport` itself (the EPA speculative-separation / cast convex radius). 0 on the GJK path.
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
    kind: SupportKind;
    /** reported convex radius — read by the driver, never added by getSupport (0 in include mode) */
    convexRadius: number;
    /** folded AddConvexRadius: added along the local direction by getSupport (EPA separation / cast radius) */
    addRadius: number;
    /** B-in-A transform, folded (replaces TransformedSupport). identity semantics when hasTransform is false */
    hasTransform: boolean;
    transform: Mat4;
    box: {
        halfExtents: Vec3;
    };
    sphere: {
        radius: number;
    };
    capsule: {
        halfHeight: number;
        radius: number;
    };
    cylinder: {
        radius: number;
        halfHeight: number;
    };
    hull: {
        /** read-only borrow, valid for the current pair (ref to shape.pointPositions / face verts, or `scratch`) */
        vertices: number[];
        vertexCount: number;
        /** owned grow-once buffer for shrunk / scaled hull verts */
        scratch: number[];
    };
    triangle: {
        a: Vec3;
        b: Vec3;
        c: Vec3;
    };
    point: {
        position: Vec3;
    };
};
/**
 * Allocate a reusable {@link Support} — this replaces the entire per-shape-type pool machinery.
 * A driver holds a small fixed number of these (e.g. one per operand slot) and refills them per
 * pair via the fill functions; there are no pools keyed by shape type and no per-shape allocation.
 * The sub-objects, transform, and scratch buffer are pre-allocated so filling never allocates.
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
/** triangle operand (mesh) — copies the 3 verts, as `setTriangleSupport` does today */
export declare function setTriangleSupport(out: Support, a: Vec3, b: Vec3, c: Vec3): void;
/** polygon face (KCC) — borrows the face's vertex array, as `setPolygonSupport` does today */
export declare function setPolygonSupport(out: Support, vertices: number[], vertexCount: number): void;
/** point operand (collidePoint) — copies the point, as `setPointSupport` does today */
export declare function setPointSupport(out: Support, point: Vec3): void;
