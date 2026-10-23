import type { Vec3 } from 'math';
import { quat, vec3 } from 'math';
import { type RigidBody, updateAABB } from './rigid-body';

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
export function addPositionStep(body: RigidBody, linearVelocityTimesDeltaTime: Vec3): void {
    const allowedTranslation = body.motionProperties.allowedDegreesOfFreedom & 0b111;
    if (allowedTranslation & 0b001) body.centerOfMassPosition[0] += linearVelocityTimesDeltaTime[0];
    if (allowedTranslation & 0b010) body.centerOfMassPosition[1] += linearVelocityTimesDeltaTime[1];
    if (allowedTranslation & 0b100) body.centerOfMassPosition[2] += linearVelocityTimesDeltaTime[2];
}

/**
 * Subtract a position step (linear velocity * dt) from the body.
 * Used in position solver for Baumgarte stabilization.
 *
 * See {@link addPositionStep} on why the dof mask applies to the step and not the position.
 *
 * @param body - Body to update
 * @param linearVelocityTimesDeltaTime - Linear velocity × deltaTime (v × dt)
 */
export function subPositionStep(body: RigidBody, linearVelocityTimesDeltaTime: Vec3): void {
    const allowedTranslation = body.motionProperties.allowedDegreesOfFreedom & 0b111;
    if (allowedTranslation & 0b001) body.centerOfMassPosition[0] -= linearVelocityTimesDeltaTime[0];
    if (allowedTranslation & 0b010) body.centerOfMassPosition[1] -= linearVelocityTimesDeltaTime[1];
    if (allowedTranslation & 0b100) body.centerOfMassPosition[2] -= linearVelocityTimesDeltaTime[2];
}

const _addRotationStep_axis = /* @__PURE__ */ vec3.create();
const _addRotationStep_rotation = /* @__PURE__ */ quat.create();

/**
 * Turns the body by the rotation vector: a rotation of its length about its direction, before the current one,
 * normalized. Jolt's Body::AddRotationStep; the position solve's step.
 *
 * @param body - Body to update
 * @param angularVelocityTimesDeltaTime - Angular velocity × deltaTime (ω × dt)
 */
export function addRotationStep(body: RigidBody, angularVelocityTimesDeltaTime: Vec3): void {
    // The exact rotation, not the first-order R + 1/2 w R dt, which falls apart for a large step (a kinematic ragdoll
    // driven to a new pose); normalized, or drift would make it stop being a rotation.
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
 * addRotationStep the other way.
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

const _deriveTransform_shapeCenterOfMassInWorldSpace = /* @__PURE__ */ vec3.create();

/**
 * re-derive the cached world transform from the authoritative state the step helpers mutate:
 * `position` from `centerOfMassPosition` and `quaternion`, then the world `aabb` from that.
 * whatever moves the centre of mass owes a call to this before the next reader of either.
 *
 * does not publish to the broadphase - see `broadphase.notifyBodyBoundsChanged`.
 */
export function deriveTransform(body: RigidBody): void {
    const shapeCenterOfMassInWorldSpace = _deriveTransform_shapeCenterOfMassInWorldSpace;
    vec3.copy(shapeCenterOfMassInWorldSpace, body.shape.centerOfMass);
    vec3.transformQuat(shapeCenterOfMassInWorldSpace, shapeCenterOfMassInWorldSpace, body.quaternion);

    vec3.sub(body.position, body.centerOfMassPosition, shapeCenterOfMassInWorldSpace);

    updateAABB(body);
}
