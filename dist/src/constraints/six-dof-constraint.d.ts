import type { Quat, Vec3 } from 'math';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintId } from './constraint-id.js';
import type { AngleConstraintPart } from './constraint-part/angle-constraint-part.js';
import type { AxisConstraintPart } from './constraint-part/axis-constraint-part.js';
import { MotorState } from './constraint-part/motor-settings.js';
import type { PointConstraintPart } from './constraint-part/point-constraint-part.js';
import type { RotationEulerConstraintPart } from './constraint-part/rotation-euler-constraint-part.js';
import type { SpringSettings } from './constraint-part/spring-settings.js';
import type { SwingTwistConstraintPart } from './constraint-part/swing-twist-constraint-part.js';
import { SwingType } from './constraint-part/swing-twist-constraint-part.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
/** axis indices for 6DOF constraint */
export declare enum SixDOFAxis {
    TRANSLATION_X = 0,
    TRANSLATION_Y = 1,
    TRANSLATION_Z = 2,
    ROTATION_X = 3,
    ROTATION_Y = 4,
    ROTATION_Z = 5,
    NUM = 6,
    NUM_TRANSLATION = 3
}
/**
 * SixDOFConstraint allows per-axis control over all 6 degrees of freedom.
 * Each axis can be: free, fixed, or limited with optional motors.
 */
export type SixDOFConstraint = ConstraintBase & {
    localSpacePosition1: Vec3;
    localSpacePosition2: Vec3;
    constraintToBody1: Quat;
    constraintToBody2: Quat;
    /** min limits for each axis (Infinity = fixed at 0, -Infinity = free) */
    limitMin: [number, number, number, number, number, number];
    /** max limits for each axis (-Infinity = fixed at 0, Infinity = free) */
    limitMax: [number, number, number, number, number, number];
    /** spring settings for translation limits */
    limitsSpringSettings: SpringSettings[];
    /** max friction per axis (0 = no friction) */
    maxFriction: [number, number, number, number, number, number];
    motorState: MotorState[];
    motorSpringSettings: SpringSettings[];
    motorMinForceLimit: [number, number, number, number, number, number];
    motorMaxForceLimit: [number, number, number, number, number, number];
    targetVelocity: Vec3;
    targetAngularVelocity: Vec3;
    targetPosition: Vec3;
    targetOrientation: Quat;
    swingType: SwingType;
    translationAxis: [Vec3, Vec3, Vec3];
    rotationAxis: [Vec3, Vec3, Vec3];
    displacement: [number, number, number];
    freeAxis: number;
    fixedAxis: number;
    translationMotorActive: boolean;
    rotationMotorActive: boolean;
    rotationPositionMotorActive: number;
    hasSpringLimits: boolean;
    translationConstraintPart: [AxisConstraintPart, AxisConstraintPart, AxisConstraintPart];
    pointConstraintPart: PointConstraintPart;
    swingTwistConstraintPart: SwingTwistConstraintPart;
    rotationConstraintPart: RotationEulerConstraintPart;
    motorTranslationConstraintPart: [AxisConstraintPart, AxisConstraintPart, AxisConstraintPart];
    motorRotationConstraintPart: [AngleConstraintPart, AngleConstraintPart, AngleConstraintPart];
};
/** Settings for creating a SixDOF constraint */
export type SixDOFConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    position1: Vec3;
    position2: Vec3;
    axisX1: Vec3;
    axisY1: Vec3;
    axisX2: Vec3;
    axisY2: Vec3;
    space?: ConstraintSpace;
    swingType?: SwingType;
    /** per-axis min limits */
    limitMin?: number[];
    /** per-axis max limits */
    limitMax?: number[];
    /** per-axis max friction */
    maxFriction?: number[];
    /** translation limit spring settings */
    limitsSpringSettings?: SpringSettings[];
    /** per-axis motor spring settings */
    motorSpringSettings?: SpringSettings[];
    /** per-axis motor min force limits */
    motorMinForceLimit?: number[];
    /** per-axis motor max force limits */
    motorMaxForceLimit?: number[];
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** Create a SixDOF constraint */
export declare function create(world: World, settings: SixDOFConstraintSettings): SixDOFConstraint;
/** Remove a SixDOF constraint */
export declare function remove(world: World, constraint: SixDOFConstraint): void;
/** Get SixDOF constraint by id */
export declare function get(world: World, id: ConstraintId): SixDOFConstraint | undefined;
/** Set translation limits */
export declare function setTranslationLimits(constraint: SixDOFConstraint, limitMin: Vec3, limitMax: Vec3): void;
/** Set rotation limits */
export declare function setRotationLimits(constraint: SixDOFConstraint, limitMin: Vec3, limitMax: Vec3): void;
/** Set motor state for an axis */
export declare function setMotorState(constraint: SixDOFConstraint, axis: SixDOFAxis, state: MotorState): void;
/** Set target velocity for translation motors */
export declare function setTargetVelocityCS(constraint: SixDOFConstraint, velocity: Vec3): void;
/** Set target angular velocity for rotation motors */
export declare function setTargetAngularVelocityCS(constraint: SixDOFConstraint, angularVelocity: Vec3): void;
/** Set target position for translation motors */
export declare function setTargetPositionCS(constraint: SixDOFConstraint, position: Vec3): void;
/** Set target orientation for rotation motors */
export declare function setTargetOrientationCS(constraint: SixDOFConstraint, orientation: Quat): void;
/** Set max friction for an axis */
export declare function setMaxFriction(constraint: SixDOFConstraint, axis: number, friction: number): void;
/** Make an axis free (unconstrained) */
export declare function makeFreeAxis(constraint: SixDOFConstraint, axis: number): void;
/** Make an axis fixed (locked at 0) */
export declare function makeFixedAxis(constraint: SixDOFConstraint, axis: number): void;
/** Set limits for an axis */
export declare function setLimitedAxis(constraint: SixDOFConstraint, axis: number, min: number, max: number): void;
/** the constraint definition for six-dof constraint */
export declare const def: import("./constraints").ConstraintDef<SixDOFConstraint>;
