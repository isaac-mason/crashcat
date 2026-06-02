import { type Mat4, type Vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body.js';
/**
 * Angular friction constraint part: 1-DOF angular constraint around the contact normal.
 *
 * Models friction torque that resists spin between two bodies in contact. Paired with the
 * two linear friction constraints (anchored at the manifold's average "friction point") to
 * form the per-manifold friction model.
 *
 * Jacobian: J = [0, -axis, 0, axis] (angular only, no linear terms)
 * Effective mass: 1 / (axis · (I1^-1 + I2^-1) · axis)
 *
 * Solve method is velocity-local: callers pass cached angular velocity vectors and the
 * function mutates them rather than reading/writing body motion properties directly.
 * Mirrors the contactConstraintPart pattern for axis constraints.
 */
export type AngularFrictionConstraintPart = {
    /** I1^-1 × axis (cached angular jacobian for body A) */
    invI1_Axis: Vec3;
    /** I2^-1 × axis (cached angular jacobian for body B) */
    invI2_Axis: Vec3;
    /** effective mass: 1 / (J × M^-1 × J^T) */
    effectiveMass: number;
    /** velocity bias (used for relative angular surface velocity along the normal) */
    bias: number;
    /** accumulated impulse (warm started from previous frame) */
    totalLambda: number;
};
/** create a new AngularFrictionConstraintPart with zero-initialized values */
export declare function create(): AngularFrictionConstraintPart;
/** reset to zero values (does not clear totalLambda, used for re-init each frame) */
export declare function reset(part: AngularFrictionConstraintPart): void;
/** deactivate this constraint part (zero out effective mass) */
export declare function deactivate(part: AngularFrictionConstraintPart): void;
/** check if constraint is active (has non-zero effective mass) */
export declare function isActive(part: AngularFrictionConstraintPart): boolean;
/**
 * Calculate constraint properties (effective mass and cached I^-1 × axis terms).
 *
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param invInertiaA inverse inertia of body A (world space, mass-scaled)
 * @param invInertiaB inverse inertia of body B (world space, mass-scaled)
 * @param worldSpaceAxis axis around which friction torque acts (normalized — the contact normal)
 * @param bias velocity bias (e.g. relative angular surface velocity along the axis)
 */
export declare function calculateConstraintProperties(part: AngularFrictionConstraintPart, bodyA: RigidBody, bodyB: RigidBody, invInertiaA: Mat4, invInertiaB: Mat4, worldSpaceAxis: Vec3, bias: number): void;
/**
 * Calculate what the total lambda would be (without applying impulse).
 * Velocity-local — reads from cached angular velocity vectors.
 *
 * @param part the constraint part
 * @param angVelA angular velocity of body A (local copy)
 * @param angVelB angular velocity of body B (local copy)
 * @param movingA true if body A is not static (dynamic or kinematic)
 * @param movingB true if body B is not static (dynamic or kinematic)
 * @param axis constraint axis (contact normal)
 * @returns new total lambda (unclamped)
 *
 * @inline
 */
export declare function getTotalLambda(part: AngularFrictionConstraintPart, angVelA: Vec3, angVelB: Vec3, movingA: boolean, movingB: boolean, axis: Vec3): number;
/**
 * Apply a total lambda value to cached angular velocity locals.
 * Velocity-local — mutates the passed-in vectors. Does NOT apply DOF masking.
 *
 * @param part the constraint part
 * @param angVelA angular velocity of body A (local copy, mutated)
 * @param angVelB angular velocity of body B (local copy, mutated)
 * @param isDynamicA true if body A is dynamic
 * @param isDynamicB true if body B is dynamic
 * @param totalLambda new total lambda to apply
 * @returns true if impulse was applied
 *
 * @inline
 */
export declare function applyLambda(part: AngularFrictionConstraintPart, angVelA: Vec3, angVelB: Vec3, isDynamicA: boolean, isDynamicB: boolean, totalLambda: number): boolean;
/**
 * Apply warm start impulse from previous frame.
 * Velocity-local — mutates the passed-in vectors. Does NOT apply DOF masking.
 *
 * @param part the constraint part
 * @param angVelA angular velocity of body A (local copy, mutated)
 * @param angVelB angular velocity of body B (local copy, mutated)
 * @param isDynamicA true if body A is dynamic
 * @param isDynamicB true if body B is dynamic
 * @param warmStartRatio scale factor for warm start (dt_new / dt_old)
 * @returns true if impulse was applied
 *
 * @inline
 */
export declare function warmStart(part: AngularFrictionConstraintPart, angVelA: Vec3, angVelB: Vec3, isDynamicA: boolean, isDynamicB: boolean, warmStartRatio: number): boolean;
