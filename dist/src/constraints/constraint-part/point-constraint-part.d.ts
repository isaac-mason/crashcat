import type { Mat4, Vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body.js';
/**
 * PointConstraintPart removes 3 translational degrees of freedom.
 * Used by PointConstraint, HingeConstraint (for point attachment), etc.
 *
 * This is the 3-axis version of AxisConstraintPart, using a 3x3 effective mass matrix.
 */
export type PointConstraintPart = {
    /** r1 in world space (moment arm from body1 COM to constraint point) */
    r1: Vec3;
    /** r2 in world space (moment arm from body2 COM to constraint point) */
    r2: Vec3;
    /** I1^-1 * [r1]× - cached for impulse application */
    invI1_r1X: Mat4;
    /** I2^-1 * [r2]× - cached for impulse application */
    invI2_r2X: Mat4;
    /** 3x3 effective mass matrix K^-1 = (J M^-1 J^T)^-1 */
    effectiveMass: Mat4;
    /** accumulated 3D impulse (for warm starting) */
    totalLambda: Vec3;
};
/** creates a new point constraint part */
export declare function create(): PointConstraintPart;
/** deactivates the constraint part (resets state) */
export declare function deactivate(part: PointConstraintPart): void;
/** checks if the constraint part is active */
export declare function isActive(part: PointConstraintPart): boolean;
/**
 * Calculate constraint properties for the point constraint.
 * Computes the 3x3 effective mass matrix and cached angular terms.
 *
 * Formula: K = (J M^-1 J^T)
 * where J = [-I, -[r1]×, I, [r2]×] is the constraint Jacobian
 *
 * For 3-axis constraint:
 * K^-1 = m1^-1 * I + [r1]× * I1^-1 * [r1]×^T + m2^-1 * I + [r2]× * I2^-1 * [r2]×^T
 *
 * We then invert K^-1 to get the effective mass K.
 */
export declare function calculateConstraintProperties(part: PointConstraintPart, bodyA: RigidBody, rotationA: Mat4, r1Local: Vec3, bodyB: RigidBody, rotationB: Mat4, r2Local: Vec3): void;
/**
 * Warm start the velocity constraint by applying cached impulses.
 * Scales the previous frame's impulses by the warm start ratio.
 */
export declare function warmStart(part: PointConstraintPart, bodyA: RigidBody, bodyB: RigidBody, warmStartImpulseRatio: number): void;
/**
 * Solve the velocity constraint.
 *
 * Computes Jacobian velocity: J * v = v1 - [r1]× * w1 - v2 + [r2]× * w2
 * Solves for lambda: λ = -K^-1 * (J * v)
 * Applies impulse to correct velocities.
 */
export declare function solveVelocityConstraint(part: PointConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean;
/**
 * Solve the position constraint using Baumgarte stabilization.
 *
 * Computes separation: C = (x2 + r2) - (x1 + r1) where x is centerOfMassPosition
 * Solves for lambda: λ = -K^-1 * β * C
 * Applies position correction directly (not to velocities).
 *
 * Note: We don't accumulate velocities for stabilization. This uses the approach
 * described in 'Modeling and Solving Constraints' by Erin Catto (GDC 2007).
 * We combine an Euler velocity integrate + position integrate and discard velocity change.
 */
export declare function solvePositionConstraint(part: PointConstraintPart, bodyA: RigidBody, bodyB: RigidBody, baumgarte: number): boolean;
