import { type Vec3 } from 'math';
import { type Box3 } from 'math/shapes';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape.js';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape.js';
import { type Shape, ShapeType } from './shapes.js';
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
 * Analytical sphere vs box collision. A is the sphere, B is the box.
 *
 * Closed-form clamp of the sphere centre to the box's shrunk core (half-extents minus convex
 * radius, mirroring setBoxSupport EXCLUDE_CONVEX_RADIUS), with the combined radius handling the
 * rounded shell. Skips GJK/EPA entirely; the deep (centre-inside-core) case degrades to a per-axis
 * SAT scan rather than EPA. Agrees with convex.collideConvexVsConvex on shallow contacts to within
 * floating point — it used to claim BIT-equivalence, but nothing tested that and the basis rewrite
 * below changes the last bits, so the weaker claim is the one that is actually known.
 *
 * The frame transforms are written out in scalars against a rotation BASIS rather than rotating
 * vectors by the quaternion one at a time — the same composition jolt uses for a body transform
 * (`Body::GetCenterOfMassTransform` is `Mat44::sRotationTranslation(rotation, position)`). One
 * quaternion-to-basis conversion serves all four transforms, the inverse is the transpose, and
 * nothing round-trips through module scratch. The faces branch keeps its arrays — they feed
 * getShapeSupportingFace, and it only runs when faces were asked for.
 */
export declare function collideSphereVsBox(collector: CollideShapeCollector, settings: CollideShapeSettings, shapeA: Shape, subShapeIdA: number, _subShapeIdBitsA: number, posAX: number, posAY: number, posAZ: number, _quatAX: number, _quatAY: number, _quatAZ: number, _quatAW: number, scaleAX: number, _scaleAY: number, _scaleAZ: number, shapeB: Shape, subShapeIdB: number, _subShapeIdBitsB: number, posBX: number, posBY: number, posBZ: number, quatBX: number, quatBY: number, quatBZ: number, quatBW: number, scaleBX: number, scaleBY: number, scaleBZ: number): void;
/**
 * Analytic ray-vs-box (slab test), replacing the generic GJK convex cast for boxes. Not an
 * approximation: the gjk path already casts against a sharp box of |scale|·halfExtents with zero
 * convex radius (setBoxSupport under INCLUDE_CONVEX_RADIUS), so this is bit-equivalent geometry,
 * exact rather than iterated to a 1e-3 tolerance. Reporting matches castRayVsConvex (entry hit,
 * treatConvexAsSolid gate); the hit carries no normal, matching CastRayHit.
 */
export declare function castRayVsBox(collector: CastRayCollector, settings: CastRaySettings, originX: number, originY: number, originZ: number, directionX: number, directionY: number, directionZ: number, length: number, shape: BoxShape, subShapeId: number, _subShapeIdBits: number, posX: number, posY: number, posZ: number, quatX: number, quatY: number, quatZ: number, quatW: number, scaleX: number, scaleY: number, scaleZ: number): void;
