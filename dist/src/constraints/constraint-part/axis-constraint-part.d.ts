import { type Mat4, type Vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body.js';
import { type SpringPart } from './spring-part.js';
import type { SpringSettings } from './spring-settings.js';
/**
 * Constraint part that constrains motion along 1 axis.
 *
 * This is the core building block for contact constraints:
 * - Normal constraint: prevents penetration
 * - Friction constraints: 2 tangential constraints (friction1, friction2)
 *
 * Also supports soft constraints (springs) for joints.
 *
 * Stores intermediate calculations for efficient solving.
 */
export type AxisConstraintPart = {
    /** r1 × axis (cross product, cached) */
    r1PlusUxAxis: Vec3;
    /** r2 × axis (cross product, cached) */
    r2xAxis: Vec3;
    /** I1^-1 × (r1 × axis) - cached angular jacobian for body 1 */
    invI1_r1PlusUxAxis: Vec3;
    /** I2^-1 × (r2 × axis) - cached angular jacobian for body 2 */
    invI2_r2xAxis: Vec3;
    /** effective mass: 1 / (J × M^-1 × J^T) (adjusted for spring softness) */
    effectiveMass: number;
    /** accumulated impulse (warm started from previous frame) */
    totalLambda: number;
    /** spring part for soft constraints (contains bias and softness) */
    springPart: SpringPart;
};
/** create a new AxisConstraintPart with zero-initialized values */
export declare function create(): AxisConstraintPart;
/** reset an AxisConstraintPart to zero values */
export declare function resetAxisConstraintPart(part: AxisConstraintPart): void;
/**
 * Calculate constraint properties (effective mass and cached jacobian terms).
 * Hard constraint version (no spring).
 * Call this during constraint initialization.
 *
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A (0 if static)
 * @param invMassB inverse mass of body B (0 if static)
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A: (contactPoint - centerOfMassA)
 * @param r2 moment arm for body B: (contactPoint - centerOfMassB)
 * @param axis constraint axis (normalized, e.g., contact normal or friction tangent)
 * @param bias velocity bias (for restitution or speculative contacts)
 */
export declare function calculateConstraintProperties(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, invInertiaA: Mat4, invInertiaB: Mat4, r1PlusU: Vec3, r2: Vec3, axis: Vec3, bias: number): void;
/**
 * Calculate constraint properties with mass override (effective mass and cached jacobian terms).
 * Hard constraint version (no spring), allows custom inverse mass and inertia scaling.
 * Call this during constraint initialization when you need to override mass properties.
 *
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A (only used when body A is dynamic)
 * @param invMassB inverse mass override for body B (only used when body B is dynamic)
 * @param invInertiaScaleA scale factor for the inverse inertia of body A
 * @param invInertiaScaleB scale factor for the inverse inertia of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A: (contactPoint - centerOfMassA)
 * @param r2 moment arm for body B: (contactPoint - centerOfMassB)
 * @param axis constraint axis (normalized, e.g., contact normal or friction tangent)
 * @param bias velocity bias (for restitution or speculative contacts)
 */
export declare function calculateConstraintPropertiesWithMassOverride(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, invInertiaScaleA: number, invInertiaScaleB: number, invInertiaA: Mat4, invInertiaB: Mat4, r1PlusU: Vec3, r2: Vec3, axis: Vec3, bias: number): void;
/**
 * Calculate constraint properties with frequency and damping (soft constraint).
 *
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A
 * @param r2 moment arm for body B
 * @param axis constraint axis (normalized)
 * @param bias velocity bias
 * @param C value of the constraint equation (C)
 * @param frequency oscillation frequency (Hz)
 * @param damping damping factor (0 = no damping, 1 = critical damping)
 */
export declare function calculateConstraintPropertiesWithFrequencyAndDamping(part: AxisConstraintPart, deltaTime: number, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, invInertiaA: Mat4, invInertiaB: Mat4, r1PlusU: Vec3, r2: Vec3, axis: Vec3, bias: number, C: number, frequency: number, damping: number): void;
/**
 * Calculate constraint properties with stiffness and damping (soft constraint).
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A
 * @param r2 moment arm for body B
 * @param axis constraint axis (normalized)
 * @param bias velocity bias
 * @param C value of the constraint equation (C)
 * @param stiffness spring stiffness k
 * @param damping spring damping coefficient c
 */
export declare function calculateConstraintPropertiesWithStiffnessAndDamping(part: AxisConstraintPart, deltaTime: number, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, invInertiaA: Mat4, invInertiaB: Mat4, r1PlusU: Vec3, r2: Vec3, axis: Vec3, bias: number, C: number, stiffness: number, damping: number): void;
/**
 * Calculate constraint properties using SpringSettings.
 * Selects the appropriate calculation method based on the spring mode.
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A
 * @param r2 moment arm for body B
 * @param axis constraint axis (normalized)
 * @param bias velocity bias
 * @param C value of the constraint equation (C)
 * @param settings spring settings (mode, frequency/stiffness, damping)
 */
export declare function calculateConstraintPropertiesWithSettings(part: AxisConstraintPart, deltaTime: number, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, invInertiaA: Mat4, invInertiaB: Mat4, r1PlusU: Vec3, r2: Vec3, axis: Vec3, bias: number, C: number, settings: SpringSettings): void;
/** Check if constraint is active (has non-zero effective mass) */
export declare function isActive(part: AxisConstraintPart): boolean;
/** Deactivate the constraint (zero out effective mass and lambda) */
export declare function deactivate(part: AxisConstraintPart): void;
/**
 * Override total lagrange multiplier.
 * Can be used to set the initial value for warm starting.
 *
 * @param part the constraint part
 * @param lambda new total lambda value
 */
export declare function setTotalLambda(part: AxisConstraintPart, lambda: number): void;
/**
 * Get the current total lagrange multiplier.
 *
 * @param part the constraint part
 * @returns Current total lambda value
 */
export declare function getTotalLambdaValue(part: AxisConstraintPart): number;
/**
 * Apply warm start impulse from previous frame.
 * Call this once before velocity iterations.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param axis constraint axis (same as used in calculateConstraintProperties)
 * @param warmStartRatio scale factor for warm start (dt_new / dt_old)
 */
export declare function warmStart(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, axis: Vec3, warmStartRatio: number): void;
/**
 * Solve velocity constraint (one iteration).
 * Standard version - uses body's actual inverse mass.
 * Call this during velocity solver iterations.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param axis constraint axis
 * @param minLambda minimum lambda (typically -Infinity for friction, 0 for normal)
 * @param maxLambda maximum lambda (typically +Infinity for normal, friction_coeff × normalLambda for friction)
 * @returns True if impulse was applied
 */
export declare function solveVelocityConstraint(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, axis: Vec3, minLambda: number, maxLambda: number): boolean;
/**
 * Calculate what the total lambda would be (without applying impulse).
 * Part 1 of two-step solve process.
 *
 * note: caller must check isActive() before calling this function
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param axis constraint axis
 * @returns new total lambda (unclamped)
 */
export declare function getTotalLambda(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, axis: Vec3): number;
/**
 * Apply a total lambda value (calculates delta and applies impulse).
 * Part 2 of two-step solve process.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A
 * @param invMassB inverse mass override for body B
 * @param axis constraint axis
 * @param totalLambda new total lambda to apply
 * @returns true if impulse was applied
 */
export declare function applyLambda(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, axis: Vec3, totalLambda: number): boolean;
/**
 * Solve velocity constraint (one iteration) with mass override.
 * Combines getTotalLambda + clamp + applyLambda in one call.
 * Mass override version - allows custom inverse mass values (for soft contacts, etc.).
 * Call this during velocity solver iterations.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A (only used when body A is dynamic)
 * @param invMassB inverse mass override for body B (only used when body B is dynamic)
 * @param axis constraint axis
 * @param minLambda minimum lambda (typically -Infinity for friction, 0 for normal)
 * @param maxLambda maximum lambda (typically +Infinity for normal, friction_coeff × normalLambda for friction)
 * @returns true if impulse was applied
 */
export declare function solveVelocityConstraintWithMassOverride(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, axis: Vec3, minLambda: number, maxLambda: number): boolean;
/**
 * Solve position constraint (Baumgarte stabilization).
 * Standard version - uses body's actual inverse mass.
 * Call this during position solver iterations.
 *
 * Note: Position constraints are only applied for hard constraints, not soft springs.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param axis constraint axis
 * @param C constraint error (penetration depth, or 0 if separated)
 * @param baumgarte baumgarte stabilization factor (typically 0.2)
 * @returns true if position correction was applied
 */
export declare function solvePositionConstraint(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, axis: Vec3, C: number, baumgarte: number): boolean;
/**
 * Solve position constraint (Baumgarte stabilization) with mass override.
 * Mass override version - allows custom inverse mass values (for soft contacts, etc.).
 * Call this during position solver iterations.
 *
 * Note: Position constraints are only applied for hard constraints, not soft springs.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A (only used when body A is dynamic)
 * @param invMassB inverse mass override for body B (only used when body B is dynamic)
 * @param axis constraint axis
 * @param C constraint error (penetration depth, or 0 if separated)
 * @param baumgarte baumgarte stabilization factor (typically 0.2)
 * @returns true if position correction was applied
 */
export declare function solvePositionConstraintWithMassOverride(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invMassA: number, invMassB: number, axis: Vec3, C: number, baumgarte: number): boolean;
