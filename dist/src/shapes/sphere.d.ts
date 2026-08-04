import { type Vec3 } from 'mathcat';
import { type Box3 } from 'mathcat/shapes';
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
export declare function collideSphereVsSphere(collector: CollideShapeCollector, _settings: CollideShapeSettings, shapeA: Shape, subShapeIdA: number, _subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, _quatAX: number, _quatAY: number, _quatAZ: number, _quatAW: number, scaleAX: number, _scaleAY: number, _scaleAZ: number, shapeB: Shape, subShapeIdB: number, _subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, _quatBX: number, _quatBY: number, _quatBZ: number, _quatBW: number, scaleBX: number, _scaleBY: number, _scaleBZ: number): void;
