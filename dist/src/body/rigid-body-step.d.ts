import type { Vec3 } from 'mathcat';
import type { RigidBody } from './rigid-body.js';
/**
 * Apply a position step (linear velocity * dt) to the body.
 * Used in position solver for Baumgarte stabilization.
 *
 * NOTE: This modifies centerOfMassPosition directly (the primary property for physics).
 * Call updatePosition() at the end of the physics step to sync the derived position property.
 *
 * @param body - Body to update
 * @param linearVelocityTimesDeltaTime - Linear velocity × deltaTime (v × dt)
 */
export declare function addPositionStep(body: RigidBody, linearVelocityTimesDeltaTime: Vec3): void;
/**
 * Subtract a position step (linear velocity * dt) from the body.
 * Used in position solver for Baumgarte stabilization.
 *
 * NOTE: This modifies centerOfMassPosition directly (the primary property for physics).
 * Call updatePosition() at the end of the physics step to sync the derived position property.
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
