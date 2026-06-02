import { type Box3, type Vec3 } from 'mathcat';
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
/**
 * Capsule support for EXCLUDE_CONVEX_RADIUS mode.
 * Used by GJK - returns line segment endpoints, stores radius in convexRadius.
 */
export type CapsuleNoConvexSupport = {
    halfHeightOfCylinder: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createCapsuleNoConvexSupport(): CapsuleNoConvexSupport;
export declare function setCapsuleNoConvexSupport(out: CapsuleNoConvexSupport, halfHeightOfCylinder: number, radius: number, scale: Vec3): void;
/**
 * Capsule support for INCLUDE_CONVEX_RADIUS mode.
 * Used by EPA and raycasting - returns surface points, convexRadius is 0.
 */
export type CapsuleWithConvexSupport = {
    halfHeightOfCylinder: Vec3;
    radius: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createCapsuleWithConvexSupport(): CapsuleWithConvexSupport;
export declare function setCapsuleWithConvexSupport(out: CapsuleWithConvexSupport, halfHeightOfCylinder: number, radius: number, scale: Vec3): void;
