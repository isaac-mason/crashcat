import { type Vec3 } from 'math';
import { type Box3 } from 'math/shapes';
import { type Shape, ShapeType } from './shapes.js';
/**
 * settings for creating a scaled shape.
 *
 * note that some shapes only support uniform scaling:
 * - sphere
 *
 * compound shapes scale their children's offsets as well as the children themselves. under a
 * non-uniform scale the children must not be rotated: a rotated child would need its scale rotated
 * into its own frame, which only exists for axis-aligned rotations and is not done here.
 *
 * non-uniform scaling of a convex hull is supported but significantly slower than uniform scaling:
 * the convex-radius-shrunk vertex set is rebaked per collision pair every frame. prefer uniform scale,
 * or bake the scaled geometry into the hull points directly.
 */
export type ScaledShapeSettings = {
    shape: Shape;
    scale: Vec3;
};
/** a scaled shape */
export type ScaledShape = {
    type: ShapeType.SCALED;
    shape: Shape;
    scale: Vec3;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
/** create a scaled shape */
export declare function create(o: ScaledShapeSettings): ScaledShape;
/** updates a scaled shape after it's properties have changed */
export declare function update(shape: ScaledShape): void;
export declare const def: import("./shapes").ShapeDef<ScaledShape>;
