import { type Box3, type Vec3 } from 'mathcat';
import { ShapeType } from './shapes.js';
/** settings for creating a box shape */
export type BoxShapeSettings = {
    /** half extents of the box */
    halfExtents: Vec3;
    /** @default 0.05 @see DEFAULT_CONVEX_RADIUS */
    convexRadius?: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};
/** a box shape */
export type BoxShape = {
    type: ShapeType.BOX;
    halfExtents: Vec3;
    convexRadius: number;
    density: number;
    materialId: number;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
/** create a box shape from settings */
export declare function create(o: BoxShapeSettings): BoxShape;
/** updates a box shape after it's properties have changed */
export declare function update(shape: BoxShape): void;
export declare const def: import("./shapes").ShapeDef<BoxShape>;
