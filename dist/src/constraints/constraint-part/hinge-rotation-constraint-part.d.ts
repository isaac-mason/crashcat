import type { Mat4, Vec3 } from 'math';
import type { RigidBody } from '../../body/rigid-body.js';
/**
 * Constrains rotation around 2 axes so that it only allows rotation around 1 axis (the hinge axis).
 *
 * Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.4.1
 *
 * Constraint equation (eq 87):
 * C = [a1 · b2, a1 · c2]
 *
 * where:
 * - a1 = hinge axis on body A (world space)
 * - b2, c2 = axes perpendicular to hinge axis on body B (world space)
 *
 * Jacobian (eq 90):
 * J = [0, -b2 × a1, 0, b2 × a1]
 *     [0, -c2 × a1, 0, c2 × a1]
 *
 * This is a 2-DOF angular constraint that keeps the hinge axes aligned.
 */
export type HingeRotationConstraintPart = {
    /** world space hinge axis for body A */
    a1: Vec3;
    /** world space perpendicular to hinge axis for body B */
    b2: Vec3;
    /** world space perpendicular to hinge axis for body B (orthogonal to b2) */
    c2: Vec3;
    /** I1^-1 cached for velocity/position integration */
    invI1: Mat4;
    /** I2^-1 cached for velocity/position integration */
    invI2: Mat4;
    /** b2 × a1 (cross product, cached for jacobian) */
    b2xA1: Vec3;
    /** c2 × a1 (cross product, cached for jacobian) */
    c2xA1: Vec3;
    /** 2x2 effective mass matrix K^-1 = (J M^-1 J^T)^-1, stored as [m00, m01, m10, m11] */
    effectiveMass: [number, number, number, number];
    /** accumulated 2D impulse (for warm starting) [lambda0, lambda1] */
    totalLambda: [number, number];
};
/** Create a new HingeRotationConstraintPart with zero-initialized values */
export declare function create(): HingeRotationConstraintPart;
/** Deactivate this constraint part (zero out effective mass and lambda) */
export declare function deactivate(part: HingeRotationConstraintPart): void;
/** Check if constraint is active (has non-zero effective mass) */
export declare function isActive(part: HingeRotationConstraintPart): boolean;
/**
 * Calculate constraint properties for the hinge rotation constraint.
 * This sets up the effective mass matrix and cached cross products.
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceHingeAxis1 hinge axis for body A in world space (normalized)
 * @param worldSpaceHingeAxis2 hinge axis for body B in world space (normalized)
 */
export declare function calculateConstraintProperties(part: HingeRotationConstraintPart, bodyA: RigidBody, bodyB: RigidBody, worldSpaceHingeAxis1: Vec3, worldSpaceHingeAxis2: Vec3): void;
/** Apply warm start impulse from previous frame */
export declare function warmStart(part: HingeRotationConstraintPart, bodyA: RigidBody, bodyB: RigidBody, warmStartImpulseRatio: number): void;
/**
 * Solve the velocity constraint.
 * Enforces d/dt C(...) = 0 where C is the constraint equation.
 */
export declare function solveVelocityConstraint(part: HingeRotationConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean;
/**
 * Solve the position constraint (Baumgarte stabilization).
 * Enforces C(...) = 0.
 */
export declare function solvePositionConstraint(part: HingeRotationConstraintPart, bodyA: RigidBody, bodyB: RigidBody, baumgarte: number): boolean;
