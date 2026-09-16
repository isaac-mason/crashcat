import { type Vec3 } from 'math';
import { type Box3, type Plane3 } from 'math/shapes';
import { type CastShapeCollector, type CastShapeSettings } from '../collision/cast-shape-vs-shape.js';
import { type Shape, ShapeType } from './shapes.js';
export declare const DEFAULT_PLANE_HALF_EXTENT = 1000;
/** settings for creating a plane shape */
export type PlaneShapeSettings = {
    /** plane definition (normal + constant) */
    plane: Plane3;
    /** half-extent of the plane's bounded representation @default 1000 */
    halfExtent?: number;
    /** material identifier @default -1 */
    materialId?: number;
};
/**
 * a plane shape - infinite plane but with bounded representation for broad-phase.
 * the negative half-space (where signedDistance < 0) is considered solid.
 *
 * planes have no mass properties and should only be used on static bodies.
 */
export type PlaneShape = {
    /** plane shape type */
    type: ShapeType.PLANE;
    /** plane equation: point · normal + constant = 0 */
    plane: Plane3;
    /** half-extent for bounded representation */
    halfExtent: number;
    /** material identifier */
    materialId: number;
    /** shape local bounds (computed) */
    aabb: Box3;
    /** shape center of mass (always at origin) */
    centerOfMass: Vec3;
    /** shape volume (always 0 - infinite/static) */
    volume: number;
};
/** create a plane shape from settings */
export declare function create(o: PlaneShapeSettings): PlaneShape;
/** updates a plane shape after it's properties have changed */
export declare function update(shape: PlaneShape): void;
export declare const def: import("./shapes").ShapeDef<PlaneShape>;
export declare function castConvexVsPlane(collector: CastShapeCollector, settings: CastShapeSettings, shapeA: Shape, subShapeIdA: number, _subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, quatAX: number, quatAY: number, quatAZ: number, quatAW: number, scaleAX: number, scaleAY: number, scaleAZ: number, dispAX: number, dispAY: number, dispAZ: number, shapeB: Shape, subShapeIdB: number, _subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
