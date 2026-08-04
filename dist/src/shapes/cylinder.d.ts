import type { Vec3 } from 'mathcat';
import { type Box3 } from 'mathcat/shapes';
import { ShapeType } from './shapes.js';
/** settings for creating a cylinder shape */
export type CylinderShapeSettings = {
    halfHeight: number;
    radius: number;
    /** @default 0.05 @see DEFAULT_CONVEX_RADIUS */
    convexRadius?: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};
/** cylinder shape aligned with Y-axis */
export type CylinderShape = {
    type: ShapeType.CYLINDER;
    halfHeight: number;
    radius: number;
    convexRadius: number;
    density: number;
    materialId: number;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
/** create cylinder shape from settings */
export declare function create(o: CylinderShapeSettings): CylinderShape;
/** update cylinder shape's derived properties */
export declare function update(shape: CylinderShape): void;
export declare const def: import("./shapes").ShapeDef<CylinderShape>;
