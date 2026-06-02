import { type Box3, type Vec3 } from 'mathcat';
import { type CollideShapeCollector, type CollideShapeSettings } from '../collision/collide-shape-vs-shape.js';
import type { Shape } from './shapes.js';
import { ShapeType } from './shapes.js';
/** settings for creating a sphere shape */
export type SphereShapeSettings = {
    /** the radius of the sphere */
    radius: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};
/** sphere shape */
export type SphereShape = {
    type: ShapeType.SPHERE;
    /** the radius of the sphere */
    radius: number;
    /** the shape density */
    density: number;
    /** material identifier */
    materialId: number;
    /** shape local bounds */
    aabb: Box3;
    /** shape center of mass */
    centerOfMass: Vec3;
    /** shape volume */
    volume: number;
};
/** create a sphere shape */
export declare function create(o: SphereShapeSettings): SphereShape;
/** updates a sphere shape after it's properties have changed */
export declare function update(shape: SphereShape): void;
export declare const def: import("./shapes").ShapeDef<SphereShape>;
/**
 * Sphere support for EXCLUDE_CONVEX_RADIUS mode.
 * Used by GJK - returns zero vector, stores sphere radius in convexRadius.
 */
export type SphereNoConvexSupport = {
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createSphereNoConvexSupport(): SphereNoConvexSupport;
export declare function setSphereNoConvexSupport(out: SphereNoConvexSupport, radius: number, scale: Vec3): void;
/**
 * Sphere support for INCLUDE_CONVEX_RADIUS mode.
 * Used by raycasting and epa - returns surface points, convexRadius is 0.
 */
export type SphereWithConvexSupport = {
    radius: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createSphereWithConvexSupport(): SphereWithConvexSupport;
export declare function setSphereWithConvexSupport(out: SphereWithConvexSupport, radius: number, scale: Vec3): void;
export declare function collideSphereVsSphere(collector: CollideShapeCollector, _settings: CollideShapeSettings, shapeA: Shape, subShapeIdA: number, _subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, _quatAX: number, _quatAY: number, _quatAZ: number, _quatAW: number, scaleAX: number, _scaleAY: number, _scaleAZ: number, shapeB: Shape, subShapeIdB: number, _subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, _quatBX: number, _quatBY: number, _quatBZ: number, _quatBW: number, scaleBX: number, _scaleBY: number, _scaleBZ: number): void;
