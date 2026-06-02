import type { Vec3 } from 'mathcat';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
import { type ConstraintId } from './constraint-id.js';
import type { AxisConstraintPart } from './constraint-part/axis-constraint-part.js';
import type { SpringSettings } from './constraint-part/spring-settings.js';
/** distance constraint removes 1 translational DOF (distance between two points) */
export type DistanceConstraint = ConstraintBase & {
    localSpacePosition1: Vec3;
    localSpacePosition2: Vec3;
    minDistance: number;
    maxDistance: number;
    limitsSpringSettings: SpringSettings;
    worldSpacePosition1: Vec3;
    worldSpacePosition2: Vec3;
    worldSpaceNormal: Vec3;
    minLambda: number;
    maxLambda: number;
    axisConstraint: AxisConstraintPart;
};
/** settings for creating a distance constraint */
export type DistanceConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    /**
     * body a constraint reference frame (space determined by `space`).
     * constraint will keep point1 (a point on body A) and point2 (a point on body B) at the same distance.
     * note that this constraint can be used as a cheap PointConstraint by setting point1 = point2 (but this removes only 1 degree of freedom instead of 3).
     */
    pointA: Vec3;
    /** body b constraint reference frame (space determined by `space`) */
    pointB: Vec3;
    /** minimum distance (-1 = use initial distance) @default -1 */
    minDistance?: number;
    /** maximum distance (-1 = use initial distance) @default -1 */
    maxDistance?: number;
    /** spring settings for soft limits */
    springSettings?: SpringSettings;
    /** constraint space@default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** create a distance constraint */
export declare function create(world: World, settings: DistanceConstraintSettings): DistanceConstraint;
/** remove a distance constraint */
export declare function remove(world: World, constraint: DistanceConstraint): void;
/** get distance constraint by id */
export declare function get(world: World, id: ConstraintId): DistanceConstraint | undefined;
/**
 * Get total accumulated impulse (lambda) from distance constraint.
 * Used for debugging, telemetry, or breaking constraints based on force.
 */
export declare function getTotalLambda(constraint: DistanceConstraint): number;
/**
 * Set the distance limits for a distance constraint.
 * @param constraint - The constraint to modify
 * @param minDistance - Minimum distance (use -1 to keep current)
 * @param maxDistance - Maximum distance (use -1 to keep current)
 */
export declare function setDistance(constraint: DistanceConstraint, minDistance: number, maxDistance: number): void;
/** the constraint definition for distance constraint */
export declare const def: import("./constraints").ConstraintDef<DistanceConstraint>;
