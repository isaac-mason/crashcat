import { type Vec3 } from 'math';
import { type Box3 } from 'math/shapes';
import { ShapeType } from './shapes.js';
/** settings for creating a capsule shape */
export type CapsuleShapeSettings = {
    /** half height of the central cylinder (excluding hemisphere caps) */
    halfHeightOfCylinder: number;
    /** radius of the capsule */
    radius: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};
/** a capsule shape */
export type CapsuleShape = {
    /** capsule shape type */
    type: ShapeType.CAPSULE;
    /** half height of the central cylinder (excluding hemisphere caps) */
    halfHeightOfCylinder: number;
    /** radius of the capsule (the convex radius) */
    radius: number;
    /** the shape density */
    density: number;
    /** material identifier */
    materialId: number;
    /** local bounds */
    aabb: Box3;
    /** center of mass */
    centerOfMass: Vec3;
    /** volume */
    volume: number;
};
/** create a capsule shape from settings */
export declare function create(o: CapsuleShapeSettings): CapsuleShape;
/** updates a capsule shape after it's properties have changed */
export declare function update(shape: CapsuleShape): void;
export declare const def: import("./shapes").ShapeDef<CapsuleShape>;
