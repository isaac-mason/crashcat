import { type Quat, type Vec3 } from 'mathcat';
import { type Box3 } from 'mathcat/shapes';
import { type Shape, ShapeType } from './shapes.js';
/** settings for creating a transformed shape */
export type TransformedShapeSettings = {
    shape: Shape;
    position: Vec3;
    quaternion: Quat;
};
/** transformed shape - applies a rigid transform to an inner shape */
export type TransformedShape = {
    type: ShapeType.TRANSFORMED;
    shape: Shape;
    position: Vec3;
    quaternion: Quat;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
/** create a transformed shape */
export declare function create(o: TransformedShapeSettings): TransformedShape;
/** updates a transformed shape after it's properties have changed */
export declare function update(shape: TransformedShape): void;
export declare const def: import("./shapes").ShapeDef<TransformedShape>;
