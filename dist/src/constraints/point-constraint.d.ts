import type { Vec3 } from 'math';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintId } from './constraint-id.js';
import type { PointConstraintPart } from './constraint-part/point-constraint-part.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
/** point constraint removes 3 translational DOF */
export type PointConstraint = ConstraintBase & {
    localSpacePosition1: Vec3;
    localSpacePosition2: Vec3;
    pointConstraintPart: PointConstraintPart;
};
/** settings for creating a point constraint */
export type PointConstraintSettings = {
    /** body a id */
    bodyIdA: BodyId;
    /** body b id */
    bodyIdB: BodyId;
    /** point on body a in world space or local space depending on `space` */
    pointA: Vec3;
    /** point on body b in world space or local space depending on `space` */
    pointB: Vec3;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** create a point constraint */
export declare function create(world: World, settings: PointConstraintSettings): PointConstraint;
/** remove a point constraint */
export declare function remove(world: World, constraint: PointConstraint): void;
/** get point constraint by id */
export declare function get(world: World, id: ConstraintId): PointConstraint | undefined;
/**
 * Get total accumulated impulse (lambda) from constraint
 * Used for debugging, telemetry, or breaking constraints based on force
 */
export declare function getTotalLambda(constraint: PointConstraint): Vec3;
/** the constraint definition for point constraint */
export declare const def: import("./constraints").ConstraintDef<PointConstraint>;
