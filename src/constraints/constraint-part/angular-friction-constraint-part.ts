import { type Mat4, type Vec3, vec3 } from 'math';
import { MotionType } from '../../body/motion-type';
import type { RigidBody } from '../../body/rigid-body';
import { MIN_NORMAL } from '../../utils/float';

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

    if (invEffectiveMass < MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effectiveMass = 1 / invEffectiveMass;
        part.bias = bias;
    }
}



/**
 * Turn a jacobian-velocity product into the part's new total lambda. The caller computes `jv` from
 * its own angular velocity locals; this owns the constraint math.
 */
export function totalLambdaFor(part: AngularFrictionConstraintPart, jv: number): number {
    return part.totalLambda + part.effectiveMass * (jv - part.bias);
}

/**
 * Commit a new total lambda and hand back the delta to apply; `0` means there is nothing to apply.
 * The caller applies the delta to its own velocity locals.
 */
export function deltaLambdaFor(part: AngularFrictionConstraintPart, totalLambda: number): number {
    const deltaLambda = totalLambda - part.totalLambda;
    part.totalLambda = totalLambda;
    return deltaLambda;
}

/**
 * Scale the stored impulse for the new timestep and hand it back; `0` means there is nothing to
 * apply. The caller applies it to its own angular velocity locals — see the note on
 * `contactConstraintPart.warmStartLambda` for why the split is worth having.
 */
export function warmStartLambda(part: AngularFrictionConstraintPart, warmStartRatio: number): number {
    part.totalLambda *= warmStartRatio;
    return part.totalLambda;
}

