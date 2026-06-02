import type { Vec3 } from 'mathcat';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
import { type ConstraintId } from './constraint-id.js';
import type { AngleConstraintPart } from './constraint-part/angle-constraint-part.js';
import type { PointConstraintPart } from './constraint-part/point-constraint-part.js';
/**
 * Cone constraint constrains 2 bodies to a single point and limits the swing between twist axes within a cone.
 *
 * Constraint: t1 · t2 >= cos(θ)
 *
 * Where:
 * - t1 = twist axis of body 1 (world space)
 * - t2 = twist axis of body 2 (world space)
 * - θ = half cone angle (angle from principal axis of cone to edge)
 *
 * This removes 3 translation DOF (point constraint) and limits rotation to a cone (1 DOF limit).
 * Note: Unlike SwingTwistConstraint, this does NOT constrain twist rotation around the axis.
 */
export type ConeConstraint = ConstraintBase & {
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;
    /** twist axis on body 1 in local space (normalized) */
    localSpaceTwistAxis1: Vec3;
    /** twist axis on body 2 in local space (normalized) */
    localSpaceTwistAxis2: Vec3;
    /** cosine of half cone angle (cos(θ)) - constraint active when t1·t2 < cosHalfConeAngle */
    cosHalfConeAngle: number;
    /** world space rotation axis (t2 × t1, normalized) for angular constraint */
    worldSpaceRotationAxis: Vec3;
    /** current cosine of angle between twist axes (t1 · t2) */
    cosTheta: number;
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** angle constraint for cone limit (1 DOF) */
    angleConstraintPart: AngleConstraintPart;
};
/** Settings for creating a cone constraint */
export type ConeConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    /** pivot point on body a */
    pointA: Vec3;
    /** twist axis on body a (will be normalized) */
    twistAxisA: Vec3;
    /** pivot point on body b */
    pointB: Vec3;
    /** twist axis on body b (will be normalized) */
    twistAxisB: Vec3;
    /** half cone angle in radians (0 = no swing, π/2 = 90° cone) @default 0 */
    halfConeAngle?: number;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** Create a cone constraint */
export declare function create(world: World, settings: ConeConstraintSettings): ConeConstraint;
/** remove a cone constraint */
export declare function remove(world: World, constraint: ConeConstraint): void;
/** Get cone constraint by id */
export declare function get(world: World, id: ConstraintId): ConeConstraint | undefined;
/**
 * Set the half cone angle.
 * @param constraint the constraint to modify
 * @param halfConeAngle half cone angle in radians (0 to π)
 */
export declare function setHalfConeAngle(constraint: ConeConstraint, halfConeAngle: number): void;
/** get the cosine of half cone angle */
export declare function getCosHalfConeAngle(constraint: ConeConstraint): number;
/** get the half cone angle in radians */
export declare function getHalfConeAngle(constraint: ConeConstraint): number;
/** Get total lambda for position constraint */
export declare function getTotalLambdaPosition(out: Vec3, constraint: ConeConstraint): Vec3;
/** Get total lambda for rotation constraint */
export declare function getTotalLambdaRotation(constraint: ConeConstraint): number;
/** the constraint definition for cone constraint */
export declare const def: import("./constraints").ConstraintDef<ConeConstraint>;
