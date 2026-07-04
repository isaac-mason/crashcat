import { type Box3, type Vec3 } from 'mathcat';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape.js';
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
/**
 * Analytic ray-vs-box (slab test), replacing the generic GJK convex cast for boxes — the same
 * specialization jolt (BoxShape::CastRay → RayAABox) and meep (ray_box_local) ship. This is not an
 * approximation: the gjk path already casts against a sharp box of |scale|·halfExtents with zero
 * convex radius (setBoxSupport under INCLUDE_CONVEX_RADIUS), so this is bit-equivalent geometry,
 * exact rather than iterated to a 1e-3 tolerance. Reporting matches castRayVsConvex (entry hit,
 * treatConvexAsSolid gate); the hit carries no normal, matching CastRayHit.
 */
export declare function castRayVsBox(collector: CastRayCollector, settings: CastRaySettings, originX: number, originY: number, originZ: number, directionX: number, directionY: number, directionZ: number, length: number, shape: BoxShape, subShapeId: number, _subShapeIdBits: number, posX: number, posY: number, posZ: number, quatX: number, quatY: number, quatZ: number, quatW: number, scaleX: number, scaleY: number, scaleZ: number): void;
