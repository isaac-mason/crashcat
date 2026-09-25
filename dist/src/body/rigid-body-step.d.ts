import type { Vec3 } from 'math';
import { type RigidBody } from './rigid-body.js';
/**
 * Apply a position step (linear velocity * dt) to the body.
 * Used in position solver for Baumgarte stabilization.
 *
 * The translation dof mask applies to the step, not to the resulting position: a locked axis stops
 * the body moving along it, it does not snap the body to zero there.
 *
 * NOTE: This modifies centerOfMassPosition directly (the primary property for physics).
 * `rigidBody.derivePositionAndBounds` syncs the derived position and aabb from it.
 *
 * @param body - Body to update
 * @param linearVelocityTimesDeltaTime - Linear velocity × deltaTime (v × dt)
 */
export declare function addPositionStep(body: RigidBody, linearVelocityTimesDeltaTime: Vec3): void;
/**
 * Subtract a position step (linear velocity * dt) from the body.
 * Used in position solver for Baumgarte stabilization.
 *
 * See {@link addPositionStep} on why the dof mask applies to the step and not the position.
 *
 * @param body - Body to update
 * @param linearVelocityTimesDeltaTime - Linear velocity × deltaTime (v × dt)
 */
export declare function subPositionStep(body: RigidBody, linearVelocityTimesDeltaTime: Vec3): void;
/**
 * Update rotation using an Euler step (used during position solver).
 *
 * This uses a proper axis-angle quaternion construction instead of a first-order
 * approximation, which is more accurate for large rotations (important for kinematic bodies).
 *
 * @param body - Body to update
 * @param angularVelocityTimesDeltaTime - Angular velocity × deltaTime (ω × dt)
 */
export declare function addRotationStep(body: RigidBody, angularVelocityTimesDeltaTime: Vec3): void;
/**
 * Update rotation using an Euler step in the opposite direction (used during position solver).
 *
 * @param body - Body to update
 * @param angularVelocityTimesDeltaTime - Angular velocity × deltaTime (ω × dt)
 */
export declare function subRotationStep(body: RigidBody, angularVelocityTimesDeltaTime: Vec3): void;
/**
 * re-derive the cached world transform from the authoritative state the step helpers mutate:
 * `position` from `centerOfMassPosition` and `quaternion`, then the world `aabb` from that.
 * whatever moves the centre of mass owes a call to this before the next reader of either.
 *
 * does not publish to the broadphase - see `broadphase.notifyBodyBoundsChanged`.
 */
export declare function deriveTransform(body: RigidBody): void;
