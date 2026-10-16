import { type Vec3 } from 'math';
import { type Box3 } from 'math/shapes';
import { type Shape, ShapeType } from './shapes.js';
/**
 * settings for creating a scaled shape.
 *
 * note that some shapes only support uniform scaling:
 * - sphere
 *
 * compound and transformed shapes scale their children's offsets as well as the children themselves,
 * and a rotated child gets the scale rotated into its own frame. under a non-uniform scale that only
 * exists for rotations that keep the scale's axes aligned (multiples of 90 degrees); any other rotation
 * would need a shear, which no shape supports.
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
