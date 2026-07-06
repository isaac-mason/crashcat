import { type Shape } from '../shapes/shapes.js';
import type { CastRayCollector, CastRaySettings } from './cast-ray-vs-shape.js';
import type { CastShapeCollector, CastShapeSettings } from './cast-shape-vs-shape.js';
import type { CollidePointCollector, CollidePointSettings } from './collide-point-vs-shape.js';
import { type CollideShapeCollector, type CollideShapeSettings } from './collide-shape-vs-shape.js';
export type { CastRayCollector, CastRayHit, CastRaySettings } from './cast-ray-vs-shape.js';
export { CastRayStatus, copyCastRayHit, createAllCastRayCollector, createAnyCastRayCollector, createCastRayHit, createClosestCastRayCollector, createDefaultCastRaySettings, } from './cast-ray-vs-shape.js';
export declare function castRayVsShape(collector: CastRayCollector, settings: CastRaySettings, originX: number, originY: number, originZ: number, directionX: number, directionY: number, directionZ: number, length: number, shape: Shape, subShapeId: number, subShapeIdBits: number, posX: number, posY: number, posZ: number, quatX: number, quatY: number, quatZ: number, quatW: number, scaleX: number, scaleY: number, scaleZ: number): void;
export type { CastShapeCollector, CastShapeHit, CastShapeSettings } from './cast-shape-vs-shape.js';
export { CastShapeStatus, copyCastShapeHit, createAllCastShapeCollector, createAnyCastShapeCollector, createCastShapeHit, createClosestCastShapeCollector, createDefaultCastShapeSettings, } from './cast-shape-vs-shape.js';
export declare function castShapeVsShape(collector: CastShapeCollector, settings: CastShapeSettings, shapeA: Shape, subShapeIdA: number, subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, quatAX: number, quatAY: number, quatAZ: number, quatAW: number, scaleAX: number, scaleAY: number, scaleAZ: number, dispAX: number, dispAY: number, dispAZ: number, shapeB: Shape, subShapeIdB: number, subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
export type { CollideShapeCollector, CollideShapeHit, CollideShapeSettings } from './collide-shape-vs-shape.js';
export { copyCollideShapeHit, createAllCollideShapeCollector, createAnyCollideShapeCollector, createClosestCollideShapeCollector, createCollideShapeHit, createDefaultCollideShapeSettings, } from './collide-shape-vs-shape.js';
export { InternalEdgeRemovingCollector, type VoidedFeature } from './internal-edge-removing-collector.js';
/**
 * Collide two shapes using the registered collision dispatch handlers.
 * If no handler is registered for the shape pair, treats as no-op (no collision).
 */
export declare function collideShapeVsShape(collector: CollideShapeCollector, settings: CollideShapeSettings, shapeA: Shape, subShapeIdA: number, subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, quatAX: number, quatAY: number, quatAZ: number, quatAW: number, scaleAX: number, scaleAY: number, scaleAZ: number, shapeB: Shape, subShapeIdB: number, subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
/**
 * collide two shapes with enhanced internal edge removal to eliminate "ghost collisions"
 * when sliding across internal edges of triangle meshes or compound shapes.
 *
 * this wrapper:
 * - forces collectFaces=true and collideOnlyWithActiveEdges=false (required for algorithm)
 * - wraps the collector with InternalEdgeRemovingCollector
 * - automatically calls flush() after collision
 *
 * use this for objects that need smooth sliding across multi-primitive surfaces
 * (e.g., character controllers, rolling objects on triangle meshes).
 */
export declare function collideShapeVsShapeWithInternalEdgeRemoval(collector: CollideShapeCollector, settings: CollideShapeSettings, shapeA: Shape, subShapeIdA: number, subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, quatAX: number, quatAY: number, quatAZ: number, quatAW: number, scaleAX: number, scaleAY: number, scaleAZ: number, shapeB: Shape, subShapeIdB: number, subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
export type { CollidePointCollector, CollidePointHit, CollidePointSettings } from './collide-point-vs-shape.js';
export { copyCollidePointHit, createAllCollidePointCollector, createAnyCollidePointCollector, createCollidePointHit, createDefaultCollidePointSettings, } from './collide-point-vs-shape.js';
export declare function collidePointVsShape(collector: CollidePointCollector, settings: CollidePointSettings, pointX: number, pointY: number, pointZ: number, shapeB: Shape, subShapeIdB: number, subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
