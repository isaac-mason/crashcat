import { type Mat4, type Vec3 } from 'mathcat';
import { type Simplex } from './simplex.js';
import type { Support } from './support.js';
type ClosestPointResult = {
    point: Vec3;
    pointSet: number;
};
/**
 * @optimize
 */
export declare function computeClosestPointOnLine(out: ClosestPointResult, a: Vec3, b: Vec3, squaredTolerance: number): void;
/**
 * @optimize
 */
export declare function computeClosestPointOnTriangle(out: ClosestPointResult, inA: Vec3, inB: Vec3, inC: Vec3, mustIncludeC: boolean, squaredTolerance: number): void;
/**
 * @optimize
 */
export declare function computeClosestPointOnTetrahedron(out: ClosestPointResult, inA: Vec3, inB: Vec3, inC: Vec3, inD: Vec3, mustIncludeD: boolean, tolerance: number): void;
export type GjkCastRayResult = {
    isHitFound: boolean;
    lambda: number;
    simplex: Simplex;
};
export declare function createGjkCastRayResult(): GjkCastRayResult;
/**
 * Cast a ray against a convex shape using GJK.
 *
 * @param out output result object
 * @param rayOrigin the starting point of the ray
 * @param rayDirection the direction of the ray
 * @param tolerance convergence tolerance
 * @param support support function for the shape
 * @param maxLambda maximum lambda to check (default 1.0). Result lambda will not exceed this.
 *
 * @optimize
 */
export declare function gjkCastRay(out: GjkCastRayResult, rayOrigin: Vec3, rayDirection: Vec3, tolerance: number, support: Support, maxLambda?: number): void;
export type GjkCastShapeResult = {
    hit: boolean;
    lambda: number;
    pointA: Vec3;
    pointB: Vec3;
    separatingAxis: Vec3;
    simplex: Simplex;
};
export declare function createGjkCastShapeResult(): GjkCastShapeResult;
/**
 * Cast a convex shape against another convex shape using GJK.
 * Shape A is moving in direction `displacement`.
 * Shape B is stationary.
 *
 * @param out output result object
 * @param transformAtoB transform matrix from shape A's local space to shape B's local space
 * @param shapeASupport support function for shape A (WITHOUT position/rotation transform)
 * @param shapeBSupport support function for shape B
 * @param displacement direction and distance to move shape A
 * @param tolerance convergence tolerance for GJK
 * @param convexRadiusA convex radius of shape A
 * @param convexRadiusB convex radius of shape B
 * @param maxLambda the max fraction along the sweep
 *
 * @optimize
 */
export declare function gjkCastShape(out: GjkCastShapeResult, transformAtoB: Mat4, shapeASupport: Support, shapeBSupport: Support, displacement: Vec3, tolerance: number, convexRadiusA: number, convexRadiusB: number, maxLambda: number): void;
export type GjkClosestPoints = {
    squaredDistance: number;
    penetrationAxis: Vec3;
    pointA: Vec3;
    pointB: Vec3;
    simplex: Simplex;
};
export declare function createGjkClosestPoints(): GjkClosestPoints;
/**
 * Get closest points between two convex shapes using GJK.
 *
 * @param out output object containing pointA, pointB, squaredDistance, penetrationAxis, and simplex.
 *            On output:
 *            - squaredDistance = 0: shapes are colliding
 *            - squaredDistance > 0 && < Number.MAX_VALUE: shapes separated, penetrationAxis is separating axis
 *            - squaredDistance = Number.MAX_VALUE: shapes far apart (exceeded maxDistanceSquared)
 * @param supportA pre-configured support function for shape A
 * @param supportB pre-configured support function for shape B
 * @param tolerance minimal distance between A and B before the objects are considered colliding
 * @param direction initial guess for the separating axis
 * @param maxDistanceSquared maximum squared distance between A and B before objects are considered infinitely far away.
 *                           If exceeded, out.squaredDistance will be set to Number.MAX_VALUE
 *
 * @optimize
 */
export declare function gjkClosestPoints(out: GjkClosestPoints, supportA: Support, supportB: Support, tolerance: number, direction: Vec3, maxDistanceSquared: number): void;
export {};
