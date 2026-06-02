import { type Box3, type Vec3 } from 'mathcat';
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
/**
 * Cylinder support for EXCLUDE_CONVEX_RADIUS mode.
 * Used by GJK - returns cylinder surface points, convexRadius stored separately.
 */
export type CylinderNoConvexSupport = {
    halfHeight: number;
    radius: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createCylinderNoConvexSupport(): CylinderNoConvexSupport;
export declare function setCylinderNoConvexSupport(out: CylinderNoConvexSupport, halfHeight: number, radius: number, convexRadius: number, scale: Vec3): void;
/**
 * Cylinder support for INCLUDE_CONVEX_RADIUS mode.
 * Used by EPA and raycasting - returns surface points, convexRadius is 0.
 */
export type CylinderWithConvexSupport = {
    halfHeight: number;
    radius: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createCylinderWithConvexSupport(): CylinderWithConvexSupport;
export declare function setCylinderWithConvexSupport(out: CylinderWithConvexSupport, halfHeight: number, radius: number, _convexRadius: number, scale: Vec3): void;
