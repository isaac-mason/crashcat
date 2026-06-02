import type { Mat4, Quat, Vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body.js';
/**
 * RotationEulerConstraintPart removes 3 rotational degrees of freedom.
 *
 * Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.5.1
 *
 * Constraint equation (eq 129):
 * C = [Δθ_x, Δθ_y, Δθ_z]
 *
 * Jacobian (eq 131):
 * J = [0, -E, 0, E]
 *
 * where E is the identity matrix.
 */
export type RotationEulerConstraintPart = {
    /** I1^-1 in world space (for position solve) */
    invI1: Mat4;
    /** I2^-1 in world space (for position solve) */
    invI2: Mat4;
    /** 3x3 effective mass matrix K^-1 = (J M^-1 J^T)^-1 */
    effectiveMass: Mat4;
    /** accumulated 3D angular impulse (for warm starting) */
    totalLambda: Vec3;
};
/** creates a new rotation euler constraint part */
export declare function create(): RotationEulerConstraintPart;
/** deactivates the constraint part (resets state) */
export declare function deactivate(part: RotationEulerConstraintPart): void;
/** checks if the constraint part is active */
export declare function isActive(part: RotationEulerConstraintPart): boolean;
/**
 * Return inverse of initial rotation from body 1 to body 2 in body 1 space.
 *
 * q20 = q10 * r0
 * => r0 = q10^-1 * q20
 * => r0^-1 = q20^-1 * q10
 *
 * where:
 * q10 = initial orientation of body 1
 * q20 = initial orientation of body 2
 * r0 = initial rotation from body 1 to body 2
 */
export declare function getInvInitialOrientation(rotation1: Quat, rotation2: Quat): Quat;
/**
 * Return inverse of initial rotation from body 1 to body 2 in body 1 space,
 * given reference axes X and Y for both bodies.
 *
 * This is used when the constraint is specified in world space with explicit axes.
 *
 * r0^-1 = c2 * c1^-1
 *
 * where c1, c2 are rotation matrices built from the reference axes.
 */
export declare function getInvInitialOrientationXY(axisX1: Vec3, axisY1: Vec3, axisX2: Vec3, axisY2: Vec3): Quat;
/**
 * Calculate constraint properties for the rotation constraint.
 * Computes the 3x3 effective mass matrix.
 *
 * For the rotation-only Jacobian J = [0, -E, 0, E]:
 * K = J * M^-1 * J^T = I1^-1 + I2^-1
 *
 * We then invert K to get the effective mass.
 */
export declare function calculateConstraintProperties(part: RotationEulerConstraintPart, bodyA: RigidBody, _rotationA: Mat4, bodyB: RigidBody, _rotationB: Mat4): void;
/**
 * Warm start the velocity constraint by applying cached impulses.
 * Scales the previous frame's impulses by the warm start ratio.
 */
export declare function warmStart(part: RotationEulerConstraintPart, bodyA: RigidBody, bodyB: RigidBody, warmStartImpulseRatio: number): void;
/**
 * Solve the velocity constraint.
 *
 * Computes Jacobian velocity: J * v = w1 - w2
 * Solves for lambda: λ = -K^-1 * (J * v)
 * Applies impulse to correct angular velocities.
 *
 * Note: Both dynamic and kinematic bodies contribute velocity to the Jacobian calculation,
 * but only dynamic bodies receive impulse corrections.
 */
export declare function solveVelocityConstraint(part: RotationEulerConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean;
/**
 * Solve the position constraint using Baumgarte stabilization.
 *
 * Computes rotation error:
 * diff = q2 * invInitialOrientation * q1^-1
 *
 * The rotation should be: q2 = q1 * r0
 * But due to drift: q2 = diff * q1 * r0
 * => diff = q2 * r0^-1 * q1^-1
 *
 * For small angles, error ≈ 2 * diff.xyz (assuming diff.w is positive)
 *
 * Solves for lambda: λ = -K^-1 * β * error
 * Applies rotation correction directly.
 */
export declare function solvePositionConstraint(part: RotationEulerConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invInitialOrientation: Quat, baumgarte: number): boolean;
