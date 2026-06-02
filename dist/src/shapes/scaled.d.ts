import { type Box3, type Vec3 } from 'mathcat';
import { type Shape, ShapeType } from './shapes.js';
/**
 * settings for creating a scaled shape.
 *
 * note that some shapes only support uniform scaling:
 * - sphere
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
