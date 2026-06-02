import type { Quat, Vec2, Vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies.js';
import { type BodyId } from '../body/body-id.js';
import type { World } from '../world.js';
import { type ConstraintBase, ConstraintSpace } from './constraints.js';
import { type ConstraintId } from './constraint-id.js';
import type { AxisConstraintPart } from './constraint-part/axis-constraint-part.js';
import type { DualAxisConstraintPart } from './constraint-part/dual-axis-constraint-part.js';
import type { MotorSettings } from './constraint-part/motor-settings.js';
import { MotorState } from './constraint-part/motor-settings.js';
import type { RotationEulerConstraintPart } from './constraint-part/rotation-euler-constraint-part.js';
import type { SpringSettings } from './constraint-part/spring-settings.js';
/**
 * Slider constraint (prismatic) removes 5 DOF (2 translation perpendicular to slider + 3 rotation).
 * Allows movement only along a single axis, like a piston or rail.
 */
export type SliderConstraint = ConstraintBase & {
    /** attachment point on body a in local space */
    localSpacePositionA: Vec3;
    /** attachment point on body b in local space */
    localSpacePositionB: Vec3;
    /** slider axis on body a in local space (normalized) - direction of allowed movement */
    localSpaceSliderAxisA: Vec3;
    /** normal axis perpendicular to slider on body a (for frame definition) */
    localSpaceNormalA: Vec3;
    /** second normal axis (slider × normal1) */
    localSpaceNormalB: Vec3;
    /** inverse of initial relative orientation, for rotation constraint */
    invInitialOrientation: Quat;
    /** r1 = rotation1 × localSpacePosition1 (world space offset from COM to attachment) */
    r1: Vec3;
    /** r2 = rotation2 × localSpacePosition2 (world space offset from COM to attachment) */
    r2: Vec3;
    /** u = x2 + r2 - x1 - r1 = p2 - p1 (separation vector in world space) */
    u: Vec3;
    /** world space slider axis */
    worldSpaceSliderAxis: Vec3;
    /** world space normal axis 1 for position constraint */
    n1: Vec3;
    /** world space normal axis 2 for position constraint */
    n2: Vec3;
    /** current distance along slider axis (d = u · sliderAxis) */
    d: number;
    /** whether position limits are enabled */
    hasLimits: boolean;
    /** minimum position limit (≤ 0) */
    limitsMin: number;
    /** maximum position limit (≥ 0) */
    limitsMax: number;
    /** spring settings for soft limits */
    limitsSpringSettings: SpringSettings;
    /** motor state */
    motorState: MotorState;
    /** target velocity (m/s) for velocity mode */
    targetVelocity: number;
    /** target position for position mode */
    targetPosition: number;
    /** maximum friction force when motor is off */
    maxFrictionForce: number;
    /** motor settings (spring + force limits) */
    motorSettings: MotorSettings;
    /** position constraint for 2 DOF perpendicular to slider */
    positionConstraintPart: DualAxisConstraintPart;
    /** rotation constraint for 3 DOF (locks all rotation) */
    rotationConstraintPart: RotationEulerConstraintPart;
    /** position limits constraint along slider axis */
    positionLimitsConstraintPart: AxisConstraintPart;
    /** motor constraint along slider axis */
    motorConstraintPart: AxisConstraintPart;
};
/** Settings for creating a slider constraint */
export type SliderConstraintSettings = {
    /** body a id */
    bodyIdA: BodyId;
    /** body b id */
    bodyIdB: BodyId;
    /** pivot point on body a */
    pointA?: Vec3;
    /** pivot point on body b */
    pointB?: Vec3;
    /** slider axis (direction of allowed movement) on body a (will be normalized) */
    sliderAxisA: Vec3;
    /** slider axis on body b (will be normalized) */
    sliderAxisB: Vec3;
    /** normal axis perpendicular to slider on body a (for frame definition) */
    normalAxisA: Vec3;
    /** normal axis perpendicular to slider on body b */
    normalAxisB: Vec3;
    /** @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** automatically detect anchor point based on body positions */
    autoDetectPoint?: boolean;
    /** minimum position limit (default: -Infinity, must be ≤ 0) */
    limitsMin?: number;
    /** maximum position limit (default: Infinity, must be ≥ 0) */
    limitsMax?: number;
    /** spring settings for soft limits */
    limitsSpringSettings?: SpringSettings;
    /** maximum friction force when motor is off */
    maxFrictionForce?: number;
    /** motor settings (spring + force limits) */
    motorSettings?: MotorSettings;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};
/** Create a slider constraint */
export declare function create(world: World, settings: SliderConstraintSettings): SliderConstraint;
/** remove a slider constraint */
export declare function remove(world: World, constraint: SliderConstraint): void;
/** get slider constraint by id */
export declare function get(world: World, id: ConstraintId): SliderConstraint | undefined;
/**
 * Get current position along the slider axis.
 * Recomputes the position from the current body transforms (not cached).
 * @param constraint the slider constraint
 * @param bodies the bodies collection
 * @returns current distance from the rest position
 */
export declare function getCurrentPosition(constraint: SliderConstraint, bodies: Bodies): number;
/**
 * Set slider constraint limits.
 * @param constraint the constraint to modify
 * @param min minimum position (must be ≤ 0)
 * @param max maximum position (must be ≥ 0)
 */
export declare function setLimits(constraint: SliderConstraint, min: number, max: number): void;
/** Set motor state for slider constraint. */
export declare function setMotorState(constraint: SliderConstraint, state: MotorState): void;
/**
 * Set target velocity for velocity motor mode.
 * @param constraint the constraint to modify
 * @param velocity target velocity in m/s
 */
export declare function setTargetVelocity(constraint: SliderConstraint, velocity: number): void;
/**
 * Set target position for position motor mode.
 * @param constraint the constraint to modify
 * @param position target position (will be clamped to limits if enabled)
 */
export declare function setTargetPosition(constraint: SliderConstraint, position: number): void;
export declare function setMaxFrictionForce(constraint: SliderConstraint, force: number): void;
/** Get maximum friction force */
export declare function getMaxFrictionForce(constraint: SliderConstraint): number;
/** Get total lambda for position constraint */
export declare function getTotalLambdaPosition(out: Vec2, constraint: SliderConstraint): Vec2;
/** Get total lambda for position limits constraint */
export declare function getTotalLambdaPositionLimits(constraint: SliderConstraint): number;
/** Get total lambda for rotation constraint */
export declare function getTotalLambdaRotation(out: Vec3, constraint: SliderConstraint): Vec3;
/** Get total lambda for motor constraint */
export declare function getTotalLambdaMotor(constraint: SliderConstraint): number;
/** the constraint definition for slider constraint */
export declare const def: import("./constraints").ConstraintDef<SliderConstraint>;
