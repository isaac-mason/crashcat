import type { Quat, Vec3 } from 'mathcat';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
import { type ConstraintId } from './constraint-id.js';
import type { AngleConstraintPart } from './constraint-part/angle-constraint-part.js';
import type { HingeRotationConstraintPart } from './constraint-part/hinge-rotation-constraint-part.js';
import type { MotorSettings } from './constraint-part/motor-settings.js';
import { MotorState } from './constraint-part/motor-settings.js';
import type { PointConstraintPart } from './constraint-part/point-constraint-part.js';
import type { SpringSettings } from './constraint-part/spring-settings.js';
/**
 * Hinge constraint removes 5 DOF (3 translation + 2 rotation).
 * Allows rotation only around the hinge axis, like a door hinge or wheel.
 */
export type HingeConstraint = ConstraintBase & {
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;
    /** hinge axis on body 1 in local space (normalized) */
    localSpaceHingeAxis1: Vec3;
    /** hinge axis on body 2 in local space (normalized) */
    localSpaceHingeAxis2: Vec3;
    /** normal axis perpendicular to hinge on body 1 (for angle reference) */
    localSpaceNormalAxis1: Vec3;
    /** normal axis perpendicular to hinge on body 2 (for angle reference) */
    localSpaceNormalAxis2: Vec3;
    /** inverse of initial relative orientation, for calculating current angle */
    invInitialOrientation: Quat;
    /** current hinge angle (radians) - cached during setup */
    theta: number;
    /** world space hinge axis (cached during setup) */
    worldSpaceHingeAxis1: Vec3;
    /** whether angle limits are enabled */
    hasLimits: boolean;
    /** minimum angle limit (radians) */
    limitsMin: number;
    /** maximum angle limit (radians) */
    limitsMax: number;
    /** spring settings for soft limits */
    limitsSpringSettings: SpringSettings;
    /** motor state */
    motorState: MotorState;
    /** target angular velocity (rad/s) for velocity mode */
    targetAngularVelocity: number;
    /** target angle (radians) for position mode */
    targetAngle: number;
    /** maximum friction torque when motor is off */
    maxFrictionTorque: number;
    /** motor settings (spring + torque limits) */
    motorSettings: MotorSettings;
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** hinge rotation constraint (2 DOF) - keeps axes aligned */
    rotationConstraintPart: HingeRotationConstraintPart;
    /** angle limit constraint (1 DOF) */
    rotationLimitsConstraintPart: AngleConstraintPart;
    /** motor constraint (1 DOF) */
    motorConstraintPart: AngleConstraintPart;
};
/** settings for creating a hinge constraint */
export type HingeConstraintSettings = {
    /** body id for body a */
    bodyIdA: BodyId;
    /** body id for body b */
    bodyIdB: BodyId;
    /** pivot point on body a */
    pointA: Vec3;
    /** pivot point on body b */
    pointB: Vec3;
    /** hinge axis on body a (will be normalized) */
    hingeAxisA: Vec3;
    /** hinge axis on body b (will be normalized) */
    hingeAxisB: Vec3;
    /** normal axis perpendicular to hinge on body a (for angle reference) */
    normalAxisA: Vec3;
    /** normal axis perpendicular to hinge on body b (for angle reference) */
    normalAxisB: Vec3;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** minimum angle limit in radians (default: -PI) */
    limitsMin?: number;
    /** maximum angle limit in radians (default: PI) */
    limitsMax?: number;
    /** spring settings for soft limits */
    limitsSpringSettings?: SpringSettings;
    /** maximum friction torque when motor is off */
    maxFrictionTorque?: number;
    /** motor settings (spring + torque limits) */
    motorSettings?: MotorSettings;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** create a hinge constraint */
export declare function create(world: World, settings: HingeConstraintSettings): HingeConstraint;
/** remove a hinge constraint */
export declare function remove(world: World, constraint: HingeConstraint): void;
/** get hinge constraint by id */
export declare function get(world: World, id: ConstraintId): HingeConstraint | undefined;
/**
 * Set hinge constraint limits.
 * @param constraint the constraint to modify
 * @param min minimum angle in radians
 * @param max maximum angle in radians
 */
export declare function setLimits(constraint: HingeConstraint, min: number, max: number): void;
/** Set motor state for hinge constraint. */
export declare function setMotorState(constraint: HingeConstraint, state: MotorState): void;
/**
 * Set target angular velocity for velocity motor mode.
 * @param constraint the constraint to modify
 * @param velocity target angular velocity in rad/s
 */
export declare function setTargetAngularVelocity(constraint: HingeConstraint, velocity: number): void;
/**
 * Set target angle for position motor mode.
 * @param constraint the constraint to modify
 * @param angle target angle in radians
 */
export declare function setTargetAngle(constraint: HingeConstraint, angle: number): void;
/**
 * Get current hinge angle.
 * @returns current angle in radians
 */
export declare function getCurrentAngle(constraint: HingeConstraint): number;
/** the constraint definition for hinge constraint */
export declare const def: import("./constraints").ConstraintDef<HingeConstraint>;
