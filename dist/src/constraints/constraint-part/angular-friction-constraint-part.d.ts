import { type Mat4, type Vec3 } from 'math';
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
 * Turn a jacobian-velocity product into the part's new total lambda. The caller computes `jv` from
 * its own angular velocity locals; this owns the constraint math.
 */
export declare function totalLambdaFor(part: AngularFrictionConstraintPart, jv: number): number;
/**
 * Commit a new total lambda and hand back the delta to apply; `0` means there is nothing to apply.
 * The caller applies the delta to its own velocity locals.
 */
export declare function deltaLambdaFor(part: AngularFrictionConstraintPart, totalLambda: number): number;
/**
 * Scale the stored impulse for the new timestep and hand it back; `0` means there is nothing to
 * apply. The caller applies it to its own angular velocity locals — see the note on
 * `contactConstraintPart.warmStartLambda` for why the split is worth having.
 */
export declare function warmStartLambda(part: AngularFrictionConstraintPart, warmStartRatio: number): number;
