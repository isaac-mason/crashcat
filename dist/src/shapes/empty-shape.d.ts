import { type Box3, type Vec3 } from 'mathcat';
import { ShapeType } from './shapes.js';
/**
 * Empty shape - has no volume, no collision geometry.
 * Used for anchor points in constraints or as placeholders.
 *
 * EmptyShape bodies:
 * - Do not produce any collision contacts (no-op collision/cast functions)
 * - Have zero mass and zero inertia
 * - Have zero-size AABB
 * - Are added to broadphase but filtered out during narrowphase via dispatch
 * - Can still be used in constraints (e.g., for "fixed to world" constraints)
 */
export type EmptyShape = {
    type: ShapeType.EMPTY;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
/**
 * Create an empty shape.
 * Useful for creating static anchor bodies that don't need collision geometry.
 */
export declare function create(): EmptyShape;
export declare const def: import("./shapes").ShapeDef<EmptyShape>;
