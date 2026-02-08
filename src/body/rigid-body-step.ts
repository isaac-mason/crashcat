import type { Vec3 } from 'mathcat';
import { quat, vec3 } from 'mathcat';
import * as motionProperties from './motion-properties';
import type { RigidBody } from './rigid-body';

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
export function addPositionStep(body: RigidBody, linearVelocityTimesDeltaTime: Vec3): void {
    // vec3.add(body.centerOfMassPosition, body.centerOfMassPosition, linearVelocityTimesDeltaTime);
    body.centerOfMassPosition[0] += linearVelocityTimesDeltaTime[0];
    body.centerOfMassPosition[1] += linearVelocityTimesDeltaTime[1];
    body.centerOfMassPosition[2] += linearVelocityTimesDeltaTime[2];
    motionProperties.applyTranslationDOFConstraint(body.centerOfMassPosition, body.motionProperties.allowedDegreesOfFreedom);
}

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
export function subPositionStep(body: RigidBody, linearVelocityTimesDeltaTime: Vec3): void {
    // vec3.sub(body.centerOfMassPosition, body.centerOfMassPosition, linearVelocityTimesDeltaTime);
    body.centerOfMassPosition[0] -= linearVelocityTimesDeltaTime[0];
    body.centerOfMassPosition[1] -= linearVelocityTimesDeltaTime[1];
    body.centerOfMassPosition[2] -= linearVelocityTimesDeltaTime[2];

    motionProperties.applyTranslationDOFConstraint(body.centerOfMassPosition, body.motionProperties.allowedDegreesOfFreedom);
}

const _addRotationStep_axis = /* @__PURE__ */ vec3.create();
const _addRotationStep_rotation = /* @__PURE__ */ quat.create();

/**
 * Update rotation using an Euler step (used during position solver).
 *
 * This uses a proper axis-angle quaternion construction instead of a first-order
 * approximation, which is more accurate for large rotations (important for kinematic bodies).
 *
 * @param body - Body to update
 * @param angularVelocityTimesDeltaTime - Angular velocity × deltaTime (ω × dt)
 */
export function addRotationStep(body: RigidBody, angularVelocityTimesDeltaTime: Vec3): void {
    // This used to use the equation: d/dt R(t) = 1/2 * w(t) * R(t) so that R(t + dt) = R(t) + 1/2 * w(t) * R(t) * dt
    // See: Appendix B of An Introduction to Physically Based Modeling: Rigid Body Simulation II-Nonpenetration Constraints
    // URL: https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf
    // But this is a first order approximation and does not work well for kinematic ragdolls that are driven to a new
    // pose if the poses differ enough. So now we split w(t) * dt into an axis and angle part and create a quaternion with it.
    // Note that the resulting quaternion is normalized since otherwise numerical drift will eventually make the rotation non-normalized.

    const len = vec3.length(angularVelocityTimesDeltaTime);
    if (len > 1.0e-6) {
        // axis = (ω × dt) / |ω × dt|
        vec3.scale(_addRotationStep_axis, angularVelocityTimesDeltaTime, 1 / len);

        // create rotation quaternion: q = (axis, angle)
        quat.setAxisAngle(_addRotationStep_rotation, _addRotationStep_axis, len);

        // apply rotation: q' = rotation × q
        quat.multiply(body.quaternion, _addRotationStep_rotation, body.quaternion);
        quat.normalize(body.quaternion, body.quaternion);
    }
}

/**
 * Update rotation using an Euler step in the opposite direction (used during position solver).
 *
 * @param body - Body to update
 * @param angularVelocityTimesDeltaTime - Angular velocity × deltaTime (ω × dt)
 */
export function subRotationStep(body: RigidBody, angularVelocityTimesDeltaTime: Vec3): void {
    // See comment at addRotationStep
    const len = vec3.length(angularVelocityTimesDeltaTime);
    if (len > 1.0e-6) {
        // axis = (ω × dt) / |ω × dt|
        vec3.scale(_addRotationStep_axis, angularVelocityTimesDeltaTime, 1 / len);

        // create rotation quaternion with negative angle: q = (axis, -angle)
        quat.setAxisAngle(_addRotationStep_rotation, _addRotationStep_axis, -len);

        // apply rotation: q' = rotation × q
        quat.multiply(body.quaternion, _addRotationStep_rotation, body.quaternion);
        quat.normalize(body.quaternion, body.quaternion);
    }
}
