import type { Mat4, Vec3 } from 'math';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape.js';
import type { CastShapeCollector, CastShapeSettings } from '../collision/cast-shape-vs-shape.js';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape.js';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape.js';
import type { Shape } from './shapes.js';
/** cast ray implementation for convex shapes */
export declare function castRayVsConvex(collector: CastRayCollector, settings: CastRaySettings, originX: number, originY: number, originZ: number, directionX: number, directionY: number, directionZ: number, length: number, shape: Shape, subShapeId: number, _subShapeIdBits: number, posX: number, posY: number, posZ: number, quatX: number, quatY: number, quatZ: number, quatW: number, scaleX: number, scaleY: number, scaleZ: number): void;
/** collide point implementation for convex shapes */
export declare function collidePointVsConvex(collector: CollidePointCollector, settings: CollidePointSettings, pointX: number, pointY: number, pointZ: number, shapeB: Shape, subShapeIdB: number, _subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
/**
 * World-space entry point for convex vs convex collision detection.
 * Transforms shapes into local space and delegates to collideConvexVsConvexLocal.
 */
export declare function collideConvexVsConvex(collector: CollideShapeCollector, settings: CollideShapeSettings, shapeA: Shape, subShapeIdA: number, _subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, quatAX: number, quatAY: number, quatAZ: number, quatAW: number, scaleAX: number, scaleAY: number, scaleAZ: number, shapeB: Shape, subShapeIdB: number, _subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
/**
 * Local-space collision detection for convex vs convex.
 * Operates in shape A's local space. Shape B's transform is relative to A.
 */
export declare function collideConvexVsConvexLocal(collector: CollideShapeCollector, settings: CollideShapeSettings, shapeA: Shape, subShapeIdA: number, shapeB: Shape, subShapeIdB: number, transformBInA: Mat4, transformAInWorld: Mat4, scaleA: Vec3, scaleB: Vec3): void;
export declare function castConvexVsConvex(collector: CastShapeCollector, settings: CastShapeSettings, shapeA: Shape, subShapeIdA: number, _subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, quatAX: number, quatAY: number, quatAZ: number, quatAW: number, scaleAX: number, scaleAY: number, scaleAZ: number, dispAX: number, dispAY: number, dispAZ: number, shapeB: Shape, subShapeIdB: number, _subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
export declare function castConvexVsConvexLocal(collector: CastShapeCollector, settings: CastShapeSettings, shapeA: Shape, subShapeIdA: number, shapeB: Shape, subShapeIdB: number, castTransform: Mat4, scaleA: Vec3, displacementInB: Vec3, scaleB: Vec3, targetTransform: Mat4): void;
