import { type Box3, type Vec3 } from 'mathcat';
import { type Shape, ShapeType } from './shapes.js';
/** settings for creating an offset center of mass shape */
export type OffsetCenterOfMassShapeSettings = {
    shape: Shape;
    offset: Vec3;
};
/**
 * offset center of mass shape - shifts the center of mass of a child shape
 * without affecting its collision geometry.
 *
 * use case: stabilizing unstable objects (e.g., cars, boats) by lowering their center of mass.
 *
 * in crashcat's coordinate system:
 * - collision geometry stays at origin
 * - centerOfMass property is offset by the specified amount
 * - only affects physics calculations (inertia, stability), not collision detection
 */
export type OffsetCenterOfMassShape = {
    type: ShapeType.OFFSET_CENTER_OF_MASS;
    shape: Shape;
    offset: Vec3;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
/** create an offset center of mass shape */
export declare function create(o: OffsetCenterOfMassShapeSettings): OffsetCenterOfMassShape;
/** update an offset center of mass shape after its properties have changed */
export declare function update(shape: OffsetCenterOfMassShape): void;
export declare const def: import("./shapes").ShapeDef<OffsetCenterOfMassShape>;
