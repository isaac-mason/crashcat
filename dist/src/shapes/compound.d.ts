import { type Quat, type Vec3 } from 'mathcat';
import { type Box3 } from 'mathcat/shapes';
import { type Shape, ShapeType } from './shapes.js';
export type CompoundShapeChild = {
    position: Vec3;
    quaternion: Quat;
    shape: Shape;
};
export type CompoundShape = {
    type: ShapeType.COMPOUND;
    children: CompoundShapeChild[];
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
export type CompoundShapeSettings = {
    children: CompoundShapeChild[];
};
export declare function create(o: CompoundShapeSettings): CompoundShape;
/** updates a compound shape after it's properties have changed or children have changed */
export declare function update(shape: CompoundShape): void;
export declare const def: import("./shapes").ShapeDef<CompoundShape>;
