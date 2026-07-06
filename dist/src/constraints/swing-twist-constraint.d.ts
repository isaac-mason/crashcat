import type { Quat, Vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies.js';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintId } from './constraint-id.js';
import type { AngleConstraintPart } from './constraint-part/angle-constraint-part.js';
import type { MotorSettings } from './constraint-part/motor-settings.js';
import { MotorState } from './constraint-part/motor-settings.js';
import type { PointConstraintPart } from './constraint-part/point-constraint-part.js';
import type { SwingTwistConstraintPart } from './constraint-part/swing-twist-constraint-part.js';
import { SwingType } from './constraint-part/swing-twist-constraint-part.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
/**
 * SwingTwistConstraint is a sophisticated constraint for humanoid ragdoll joints.
 * It allows limited rotation with separate swing and twist limits.
 *
 * - Swing: rotation around Y and Z axes, limited by cone or pyramid shape
 * - Twist: rotation around X axis (the twist axis), limited by min/max angles
 */
export type SwingTwistConstraint = ConstraintBase & {
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;
    /** transform from constraint space to body 1 space */
    constraintToBody1: Quat;
    /** transform from constraint space to body 2 space */
    constraintToBody2: Quat;
    /** normal half cone angle (radians) - swing limit around Z axis */
    normalHalfConeAngle: number;
    /** plane half cone angle (radians) - swing limit around Y axis */
    planeHalfConeAngle: number;
    /** minimum twist angle (radians), in [-PI, PI] */
    twistMinAngle: number;
    /** maximum twist angle (radians), in [-PI, PI] */
    twistMaxAngle: number;
    /** maximum friction torque (N*m) when not powered by motor */
    maxFrictionTorque: number;
    /** swing motor state */
    swingMotorState: MotorState;
    /** twist motor state */
    twistMotorState: MotorState;
    /** target angular velocity in constraint space of body 2 */
    targetAngularVelocity: Vec3;
    /** target orientation in constraint space */
    targetOrientation: Quat;
    /** swing motor settings (spring + torque limits) */
    swingMotorSettings: MotorSettings;
    /** twist motor settings (spring + torque limits) */
    twistMotorSettings: MotorSettings;
    /** world space motor axes (X, Y, Z) */
    worldSpaceMotorAxis: [Vec3, Vec3, Vec3];
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** swing-twist constraint part for rotation limits */
    swingTwistConstraintPart: SwingTwistConstraintPart;
    /** motor constraint parts (twist=0, swingY=1, swingZ=2) */
    motorConstraintParts: [AngleConstraintPart, AngleConstraintPart, AngleConstraintPart];
};
/** Settings for creating a swing-twist constraint */
export type SwingTwistConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    /** pivot point on body 1 */
    position1: Vec3;
    /** pivot point on body 2 */
    position2: Vec3;
    /** twist axis on body 1 (X axis of constraint space) */
    twistAxis1: Vec3;
    /** plane axis on body 1 (Z axis of constraint space, perpendicular to twist) */
    planeAxis1: Vec3;
    /** twist axis on body 2 */
    twistAxis2: Vec3;
    /** plane axis on body 2 */
    planeAxis2: Vec3;
    /** @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** swing limit type @default SwingType.Cone */
    swingType?: SwingType;
    /** normal half cone angle (radians) - swing limit around Z axis */
    normalHalfConeAngle?: number;
    /** plane half cone angle (radians) - swing limit around Y axis */
    planeHalfConeAngle?: number;
    /** minimum twist angle (radians) @default 0 */
    twistMinAngle?: number;
    /** maximum twist angle (radians) @default 0 */
    twistMaxAngle?: number;
    /** maximum friction torque @default 0 */
    maxFrictionTorque?: number;
    /** swing motor settings (spring + torque limits) */
    swingMotorSettings?: MotorSettings;
    /** twist motor settings (spring + torque limits) */
    twistMotorSettings?: MotorSettings;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** Create a swing-twist constraint */
export declare function create(world: World, settings: SwingTwistConstraintSettings): SwingTwistConstraint;
/** Remove a swing-twist constraint */
export declare function remove(world: World, constraint: SwingTwistConstraint): void;
/** Get swing-twist constraint by id */
export declare function get(world: World, id: ConstraintId): SwingTwistConstraint | undefined;
/** Set normal half cone angle (swing limit around Z axis) */
export declare function setNormalHalfConeAngle(constraint: SwingTwistConstraint, angle: number): void;
/** Set plane half cone angle (swing limit around Y axis) */
export declare function setPlaneHalfConeAngle(constraint: SwingTwistConstraint, angle: number): void;
/** Set twist min angle */
export declare function setTwistMinAngle(constraint: SwingTwistConstraint, angle: number): void;
/** Set twist max angle */
export declare function setTwistMaxAngle(constraint: SwingTwistConstraint, angle: number): void;
/** Set maximum friction torque */
export declare function setMaxFrictionTorque(constraint: SwingTwistConstraint, torque: number): void;
/** Set swing motor state */
export declare function setSwingMotorState(constraint: SwingTwistConstraint, state: MotorState): void;
/** Set twist motor state */
export declare function setTwistMotorState(constraint: SwingTwistConstraint, state: MotorState): void;
/** Set target angular velocity in constraint space of body 2 */
export declare function setTargetAngularVelocityCS(constraint: SwingTwistConstraint, velocity: Vec3): void;
/** Set target orientation in constraint space */
export declare function setTargetOrientationCS(constraint: SwingTwistConstraint, orientation: Quat): void;
/** Set target orientation in body space (R2 = R1 * inOrientation) */
export declare function setTargetOrientationBS(constraint: SwingTwistConstraint, orientation: Quat): void;
/** Get current rotation of constraint in constraint space */
export declare function getRotationInConstraintSpace(out: Quat, constraint: SwingTwistConstraint, bodies: Bodies): void;
/** Get total lambda for position constraint */
export declare function getTotalLambdaPosition(out: Vec3, constraint: SwingTwistConstraint): Vec3;
/** Get total lambda for twist limit */
export declare function getTotalLambdaTwist(constraint: SwingTwistConstraint): number;
/** Get total lambda for swing Y limit */
export declare function getTotalLambdaSwingY(constraint: SwingTwistConstraint): number;
/** Get total lambda for swing Z limit */
export declare function getTotalLambdaSwingZ(constraint: SwingTwistConstraint): number;
/** Get total lambda for motors */
export declare function getTotalLambdaMotor(out: Vec3, constraint: SwingTwistConstraint): Vec3;
/** the constraint definition for swing-twist constraint */
export declare const def: import("./constraints").ConstraintDef<SwingTwistConstraint>;
