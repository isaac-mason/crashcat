import { type Mat4, type Vec3 } from 'mathcat';
import { type GjkCastShapeResult } from './gjk.js';
import { type Simplex } from './simplex.js';
import { type Support } from './support.js';
export declare enum PenetrationDepthStatus {
    NOT_COLLIDING = 0,
    COLLIDING = 1,
    INDETERMINATE = 2
}
export type PenetrationDepth = {
    status: PenetrationDepthStatus;
    penetrationAxis: Vec3;
    pointA: Vec3;
    pointB: Vec3;
};
export declare const createPenetrationDepth: () => PenetrationDepth;
export declare function penetrationDepthStepGJK(outPenetrationDepth: PenetrationDepth, outSimplex: Simplex, supportA: Support, supportB: Support, convexRadiusA: number, convexRadiusB: number, direction: Vec3, tolerance: number): void;
/**
 * EPA penetration depth step.
 * This function expects supports that INCLUDE their convex radius. The caller fills the support
 * structs in INCLUDE mode and folds any extra separation onto `addRadius` before calling, typically
 * in the EPA fallback path. This matches Jolt's EPAPenetrationDepth.h pattern.
 */
export declare function penetrationDepthStepEPA(out: PenetrationDepth, supportAIncludingRadius: Support, supportBIncludingRadius: Support, tolerance: number, simplex: Simplex): boolean;
export declare function penetrationCastShape(out: GjkCastShapeResult, transformAtoB: Mat4, shapeASupport: Support, shapeBSupport: Support, displacement: Vec3, collisionTolerance: number, penetrationTolerance: number, convexRadiusA: number, convexRadiusB: number, maxLambda: number, returnDeepestPoint: boolean): void;
