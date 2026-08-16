import type { Mat2, Mat4, Vec2, Vec3 } from 'math';
import type { RigidBody } from '../../body/rigid-body.js';
/**
 * Constrains movement on 2 axes perpendicular to a sliding axis.
 *
 * Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.3.1
 *
 * Constraint equation (eq 51):
 * C = [(p2 - p1) · n1, (p2 - p1) · n2]
 *
 * Jacobian (transposed) (eq 55):
 * J^T = [
 *   -n1               -n2
 *   -(r1 + u) × n1    -(r1 + u) × n2
 *   n1                n2
 *   r2 × n1           r2 × n2
 * ]
 *
 * where:
 * n1, n2 = constraint axes perpendicular to slider axis (normalized)
 * p1, p2 = constraint points
 * r1 = p1 - x1
 * r2 = p2 - x2
 * u = x2 + r2 - x1 - r1 = p2 - p1
 * x1, x2 = center of mass for the bodies
 */
export type DualAxisConstraintPart = {
    /** (r1 + u) × n1 */
    r1PlusUxN1: Vec3;
    /** (r1 + u) × n2 */
    r1PlusUxN2: Vec3;
    /** r2 × n1 */
    r2xN1: Vec3;
    /** r2 × n2 */
    r2xN2: Vec3;
    /** I1^-1 × ((r1 + u) × n1) */
    invI1_r1PlusUxN1: Vec3;
    /** I1^-1 × ((r1 + u) × n2) */
    invI1_r1PlusUxN2: Vec3;
    /** I2^-1 × (r2 × n1) */
    invI2_r2xN1: Vec3;
    /** I2^-1 × (r2 × n2) */
    invI2_r2xN2: Vec3;
    /** 2x2 effective mass matrix K^-1 = (J M^-1 J^T)^-1 */
    effectiveMass: Mat2;
    /** Accumulated 2D impulse (lambda) for warm starting */
    totalLambda: Vec2;
};
/** Creates a new DualAxisConstraintPart with zero-initialized values */
export declare function create(): DualAxisConstraintPart;
/** Deactivates the constraint part (resets state) */
export declare function deactivate(part: DualAxisConstraintPart): void;
/** Checks if the constraint part is active (has invertible effective mass) */
export declare function isActive(part: DualAxisConstraintPart): boolean;
/**
 * Calculate constraint properties for the dual axis constraint.
 * All input vectors are in world space.
 *
 * @param part The constraint part to initialize
 * @param bodyA First body
 * @param rotationA Rotation matrix for body A
 * @param r1PlusU Moment arm for body A: attachment point - COM (world space)
 * @param bodyB Second body
 * @param rotationB Rotation matrix for body B
 * @param r2 Moment arm for body B: attachment point - COM (world space)
 * @param n1 First constraint axis (normalized, perpendicular to slider axis)
 * @param n2 Second constraint axis (normalized, perpendicular to slider axis and n1)
 */
export declare function calculateConstraintProperties(part: DualAxisConstraintPart, bodyA: RigidBody, rotationA: Mat4, r1PlusU: Vec3, bodyB: RigidBody, rotationB: Mat4, r2: Vec3, n1: Vec3, n2: Vec3): void;
/**
 * Warm start the velocity constraint by applying cached impulses.
 * Scales the previous frame's impulses by the warm start ratio.
 */
export declare function warmStart(part: DualAxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, n1: Vec3, n2: Vec3, warmStartImpulseRatio: number): void;
/**
 * Solve the velocity constraint.
 * Iteratively update to make d/dt C(...) = 0.
 *
 * Note: Both dynamic and kinematic bodies contribute velocity to the Jacobian calculation,
 * but only dynamic bodies receive impulse corrections.
 *
 * @returns True if any impulse was applied
 */
export declare function solveVelocityConstraint(part: DualAxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, n1: Vec3, n2: Vec3): boolean;
/**
 * Solve the position constraint.
 * Iteratively update to make C(...) = 0.
 *
 * @param u The separation vector (p2 - p1) in world space
 * @param baumgarte Baumgarte stabilization factor
 * @returns True if any correction was applied
 */
export declare function solvePositionConstraint(part: DualAxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, u: Vec3, n1: Vec3, n2: Vec3, baumgarte: number): boolean;
