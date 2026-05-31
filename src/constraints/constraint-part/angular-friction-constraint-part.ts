import { type Mat4, type Vec3, vec3 } from 'mathcat';
import { MotionType } from '../../body/motion-type';
import type { RigidBody } from '../../body/rigid-body';

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
export function create(): AngularFrictionConstraintPart {
    return {
        invI1_Axis: vec3.create(),
        invI2_Axis: vec3.create(),
        effectiveMass: 0,
        bias: 0,
        totalLambda: 0,
    };
}

/** reset to zero values (does not clear totalLambda, used for re-init each frame) */
export function reset(part: AngularFrictionConstraintPart): void {
    vec3.set(part.invI1_Axis, 0, 0, 0);
    vec3.set(part.invI2_Axis, 0, 0, 0);
    part.effectiveMass = 0;
    part.bias = 0;
}

/** deactivate this constraint part (zero out effective mass) */
export function deactivate(part: AngularFrictionConstraintPart): void {
    part.effectiveMass = 0;
    part.totalLambda = 0;
}

/** check if constraint is active (has non-zero effective mass) */
export function isActive(part: AngularFrictionConstraintPart): boolean {
    return part.effectiveMass !== 0;
}

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
export function calculateConstraintProperties(
    part: AngularFrictionConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invInertiaA: Mat4,
    invInertiaB: Mat4,
    worldSpaceAxis: Vec3,
    bias: number,
): void {
    const ax = worldSpaceAxis[0];
    const ay = worldSpaceAxis[1];
    const az = worldSpaceAxis[2];

    let invEffectiveMass = 0;

    if (bodyA.motionType === MotionType.DYNAMIC) {
        const i1x = invInertiaA[0] * ax + invInertiaA[4] * ay + invInertiaA[8] * az;
        const i1y = invInertiaA[1] * ax + invInertiaA[5] * ay + invInertiaA[9] * az;
        const i1z = invInertiaA[2] * ax + invInertiaA[6] * ay + invInertiaA[10] * az;
        part.invI1_Axis[0] = i1x;
        part.invI1_Axis[1] = i1y;
        part.invI1_Axis[2] = i1z;
        invEffectiveMass += ax * i1x + ay * i1y + az * i1z;
    } else {
        part.invI1_Axis[0] = 0;
        part.invI1_Axis[1] = 0;
        part.invI1_Axis[2] = 0;
    }

    if (bodyB.motionType === MotionType.DYNAMIC) {
        const i2x = invInertiaB[0] * ax + invInertiaB[4] * ay + invInertiaB[8] * az;
        const i2y = invInertiaB[1] * ax + invInertiaB[5] * ay + invInertiaB[9] * az;
        const i2z = invInertiaB[2] * ax + invInertiaB[6] * ay + invInertiaB[10] * az;
        part.invI2_Axis[0] = i2x;
        part.invI2_Axis[1] = i2y;
        part.invI2_Axis[2] = i2z;
        invEffectiveMass += ax * i2x + ay * i2y + az * i2z;
    } else {
        part.invI2_Axis[0] = 0;
        part.invI2_Axis[1] = 0;
        part.invI2_Axis[2] = 0;
    }

    if (invEffectiveMass === 0) {
        deactivate(part);
    } else {
        part.effectiveMass = 1 / invEffectiveMass;
        part.bias = bias;
    }
}

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
export function getTotalLambda(
    part: AngularFrictionConstraintPart,
    angVelA: Vec3,
    angVelB: Vec3,
    movingA: boolean,
    movingB: boolean,
    axis: Vec3,
): number {
    let jv = 0;
    if (movingA) {
        jv += axis[0] * angVelA[0] + axis[1] * angVelA[1] + axis[2] * angVelA[2];
    }
    if (movingB) {
        jv -= axis[0] * angVelB[0] + axis[1] * angVelB[1] + axis[2] * angVelB[2];
    }

    const lambda = part.effectiveMass * (jv - part.bias);
    return part.totalLambda + lambda;
}

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
export function applyLambda(
    part: AngularFrictionConstraintPart,
    angVelA: Vec3,
    angVelB: Vec3,
    isDynamicA: boolean,
    isDynamicB: boolean,
    totalLambda: number,
): boolean {
    const deltaLambda = totalLambda - part.totalLambda;
    part.totalLambda = totalLambda;

    if (deltaLambda === 0) return false;

    if (isDynamicA) {
        angVelA[0] -= part.invI1_Axis[0] * deltaLambda;
        angVelA[1] -= part.invI1_Axis[1] * deltaLambda;
        angVelA[2] -= part.invI1_Axis[2] * deltaLambda;
    }

    if (isDynamicB) {
        angVelB[0] += part.invI2_Axis[0] * deltaLambda;
        angVelB[1] += part.invI2_Axis[1] * deltaLambda;
        angVelB[2] += part.invI2_Axis[2] * deltaLambda;
    }

    return true;
}

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
export function warmStart(
    part: AngularFrictionConstraintPart,
    angVelA: Vec3,
    angVelB: Vec3,
    isDynamicA: boolean,
    isDynamicB: boolean,
    warmStartRatio: number,
): boolean {
    part.totalLambda *= warmStartRatio;

    if (part.totalLambda === 0) return false;

    if (isDynamicA) {
        angVelA[0] -= part.invI1_Axis[0] * part.totalLambda;
        angVelA[1] -= part.invI1_Axis[1] * part.totalLambda;
        angVelA[2] -= part.invI1_Axis[2] * part.totalLambda;
    }

    if (isDynamicB) {
        angVelB[0] += part.invI2_Axis[0] * part.totalLambda;
        angVelB[1] += part.invI2_Axis[1] * part.totalLambda;
        angVelB[2] += part.invI2_Axis[2] * part.totalLambda;
    }

    return true;
}
