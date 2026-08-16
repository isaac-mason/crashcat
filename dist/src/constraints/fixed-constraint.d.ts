import type { Quat, Vec3 } from 'math';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintId } from './constraint-id.js';
import type { PointConstraintPart } from './constraint-part/point-constraint-part.js';
import type { RotationEulerConstraintPart } from './constraint-part/rotation-euler-constraint-part.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
/**
 * Fixed constraint removes 6 DOF (3 translation + 3 rotation).
 * Welds two bodies together so they move as one rigid body.
 */
export type FixedConstraint = ConstraintBase & {
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;
    /** inverse of initial relative orientation from body 1 to body 2 */
    invInitialOrientation: Quat;
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** rotation constraint (3 DOF) - keeps orientations locked */
    rotationConstraintPart: RotationEulerConstraintPart;
};
/** settings for creating a fixed constraint */
export type FixedConstraintSettings = {
    /** body a id */
    bodyIdA: BodyId;
    /** body b id */
    bodyIdB: BodyId;
    /** pivot point on body 1 (required unless autoDetectPoint is true) */
    point1?: Vec3;
    /** reference axis X for body 1 (for orientation) */
    axisX1?: Vec3;
    /** reference axis Y for body 1 (for orientation) */
    axisY1?: Vec3;
    /** pivot point on body 2 (required unless autoDetectPoint is true) */
    point2?: Vec3;
    /** reference axis X for body 2 (for orientation) */
    axisX2?: Vec3;
    /** reference axis Y for body 2 (for orientation) */
    axisY2?: Vec3;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /**
     * auto-detect attachment point based on body positions/masses.
     * when true, point1 and point2 are ignored.
     * @default false
     */
    autoDetectPoint?: boolean;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** create a fixed constraint */
export declare function create(world: World, settings: FixedConstraintSettings): FixedConstraint;
/** remove a fixed constraint */
export declare function remove(world: World, constraint: FixedConstraint): void;
/** get fixed constraint by id */
export declare function get(world: World, id: ConstraintId): FixedConstraint | undefined;
/** Get total lambda for position constraint (for debugging/inspection) */
export declare function getTotalLambdaPosition(out: Vec3, constraint: FixedConstraint): Vec3;
/** Get total lambda for rotation constraint (for debugging/inspection) */
export declare function getTotalLambdaRotation(out: Vec3, constraint: FixedConstraint): Vec3;
/** the constraint definition for fixed constraint */
export declare const def: import("./constraints").ConstraintDef<FixedConstraint>;
