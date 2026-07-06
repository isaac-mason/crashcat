import { type Quat, type Vec3 } from 'mathcat';
import type { CastRayCollector, CastRaySettings, CollidePointCollector, CollidePointSettings } from './collision/narrowphase.js';
import { type CastShapeCollector, type CastShapeSettings, type CollideShapeCollector, type CollideShapeSettings } from './collision/narrowphase.js';
import type { Filter } from './filter.js';
import type { Shape } from './shapes/shapes.js';
import type { World } from './world.js';
export declare function castRay(world: World, collector: CastRayCollector, settings: CastRaySettings, origin: Vec3, direction: Vec3, length: number, filter: Filter): void;
export declare function castShape(world: World, collector: CastShapeCollector, settings: CastShapeSettings, shape: Shape, position: Vec3, quaternion: Quat, scale: Vec3, displacement: Vec3, filter: Filter): void;
export declare function collideShape(world: World, collector: CollideShapeCollector, settings: CollideShapeSettings, shape: Shape, position: Vec3, quaternion: Quat, scale: Vec3, filter: Filter): void;
/**
 * collide a shape against the world with enhanced internal edge removal.
 *
 * this eliminates "ghost collisions" when sliding across internal edges of triangle meshes
 * or compound shapes, preventing stuttering and catches on smooth surfaces.
 *
 * automatically forces collectFaces=true and collideOnlyWithActiveEdges=false.
 *
 * use this for character controllers, rolling objects, and anything that needs smooth
 * sliding across multi-primitive surfaces.
 */
export declare function collideShapeWithInternalEdgeRemoval(world: World, collector: CollideShapeCollector, settings: CollideShapeSettings, shape: Shape, position: Vec3, quaternion: Quat, scale: Vec3, filter: Filter): void;
export declare function collidePoint(world: World, collector: CollidePointCollector, settings: CollidePointSettings, point: Vec3, filter: Filter): void;
