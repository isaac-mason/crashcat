import type { Vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body.js';
import { type SpringPart } from './spring-part.js';
import type { SpringSettings } from './spring-settings.js';
/**
 * Constrains rotation along 1 axis.
 *
 * Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.4.5
 *
 * Constraint equation (eq 108):
 * C = θ(t) - θ_target
 *
 * Jacobian (eq 109):
 * J = [0, -a^T, 0, a^T]
 *
 * where:
 * - a = axis around which rotation is constrained (normalized, world space)
 *
 * This is a 1-DOF angular constraint used for:
 * - Hinge limits (min/max angle)
 * - Hinge motors (driving to target angle or velocity)
 * - Angular friction
 */
export type AngleConstraintPart = {
    /** I1^-1 * axis (cached for velocity/position integration) */
    invI1_Axis: Vec3;
    /** I2^-1 * axis (cached for velocity/position integration) */
    invI2_Axis: Vec3;
    /** Effective mass: 1 / (J M^-1 J^T) */
    effectiveMass: number;
    /** Spring part for soft constraints */
    springPart: SpringPart;
    /** Accumulated impulse (for warm starting) */
    totalLambda: number;
};
/** create a new AngleConstraintPart with zero-initialized values */
export declare function create(): AngleConstraintPart;
/** deactivate this constraint part (zero out effective mass and lambda) */
export declare function deactivate(part: AngleConstraintPart): void;
/** check if constraint is active (has non-zero effective mass) */
export declare function isActive(part: AngleConstraintPart): boolean;
/**
 * Calculate constraint properties (hard constraint, no spring).
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation along which the constraint acts (normalized)
 * @param bias bias term (b) for the constraint impulse: lambda = J v + b
 */
export declare function calculateConstraintProperties(part: AngleConstraintPart, bodyA: RigidBody, bodyB: RigidBody, worldSpaceAxis: Vec3, bias?: number): void;
/**
 * Calculate constraint properties with frequency and damping (soft constraint).
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param bias bias term
 * @param C value of the constraint equation (angle error)
 * @param frequency oscillation frequency (Hz)
 * @param damping damping factor (0 = no damping, 1 = critical damping)
 */
export declare function calculateConstraintPropertiesWithFrequencyAndDamping(part: AngleConstraintPart, deltaTime: number, bodyA: RigidBody, bodyB: RigidBody, worldSpaceAxis: Vec3, bias: number, C: number, frequency: number, damping: number): void;
/**
 * Calculate constraint properties with stiffness and damping (soft constraint).
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param bias bias term
 * @param C value of the constraint equation (angle error)
 * @param stiffness spring stiffness k
 * @param damping spring damping coefficient c
 */
export declare function calculateConstraintPropertiesWithStiffnessAndDamping(part: AngleConstraintPart, deltaTime: number, bodyA: RigidBody, bodyB: RigidBody, worldSpaceAxis: Vec3, bias: number, C: number, stiffness: number, damping: number): void;
/**
 * Calculate constraint properties using SpringSettings.
 * Selects the appropriate calculation method based on the spring mode.
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param bias bias term
 * @param C value of the constraint equation (angle error)
 * @param settings spring settings (mode, frequency/stiffness, damping)
 */
export declare function calculateConstraintPropertiesWithSettings(part: AngleConstraintPart, deltaTime: number, bodyA: RigidBody, bodyB: RigidBody, worldSpaceAxis: Vec3, bias: number, C: number, settings: SpringSettings): void;
/** apply warm start impulse from previous frame */
export declare function warmStart(part: AngleConstraintPart, bodyA: RigidBody, bodyB: RigidBody, warmStartImpulseRatio: number): void;
/**
 * Solve the velocity constraint.
 * Enforces d/dt C(...) = 0 where C is the constraint equation.
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param minLambda minimum angular impulse (N m s)
 * @param maxLambda maximum angular impulse (N m s)
 * @returns true if impulse was applied
 */
export declare function solveVelocityConstraint(part: AngleConstraintPart, bodyA: RigidBody, bodyB: RigidBody, worldSpaceAxis: Vec3, minLambda: number, maxLambda: number): boolean;
/**
 * Solve the position constraint (Baumgarte stabilization).
 * Enforces C(...) = 0.
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param C constraint error (angle error in radians)
 * @param baumgarte Baumgarte stabilization factor
 * @returns true if correction was applied
 */
export declare function solvePositionConstraint(part: AngleConstraintPart, bodyA: RigidBody, bodyB: RigidBody, C: number, baumgarte: number): boolean;
/** Get total accumulated lambda (impulse) */
export declare function getTotalLambda(part: AngleConstraintPart): number;
/** Set total lambda (for warm starting with specific value) */
export declare function setTotalLambda(part: AngleConstraintPart, lambda: number): void;
