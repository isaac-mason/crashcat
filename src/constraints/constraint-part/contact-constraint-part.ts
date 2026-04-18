import type { Vec3 } from 'mathcat';
import type { AxisConstraintPart } from './axis-constraint-part';
import { getSpringBias } from './spring-part';

/**
 * contact-specific constraint part functions that operate on cached velocity locals
 * instead of reading/writing body motion properties directly.
 *
 * this mirrors jolt's ContactConstraintPart — a thin layer over AxisConstraintPart
 * that avoids repeated body property access during the contact solve loop.
 *
 * the caller is responsible for:
 * 1. loading velocities from bodies into locals before the solve loop
 * 2. passing these locals through all getTotalLambda/applyLambda calls
 * 3. writing velocities back to bodies + applying DOF masking once after the loop
 *
 * uses the same AxisConstraintPart data type — no new data structure needed.
 */

/**
 * calculate what the total lambda would be (without applying impulse).
 * velocity-local version — reads from cached velocity vectors instead of body properties.
 *
 * @param part the constraint part
 * @param linVelA linear velocity of body A (local copy)
 * @param angVelA angular velocity of body A (local copy)
 * @param linVelB linear velocity of body B (local copy)
 * @param angVelB angular velocity of body B (local copy)
 * @param movingA true if body A is not static (dynamic or kinematic)
 * @param movingB true if body B is not static (dynamic or kinematic)
 * @param axis constraint axis
 * @returns new total lambda (unclamped)
 */
export function getTotalLambda(
    part: AxisConstraintPart,
    linVelA: Vec3,
    angVelA: Vec3,
    linVelB: Vec3,
    angVelB: Vec3,
    movingA: boolean,
    movingB: boolean,
    axis: Vec3,
): number {
    // calculate jacobian multiplied by linear velocity
    let jv: number;
    if (movingA && movingB) {
        const dx = linVelA[0] - linVelB[0];
        const dy = linVelA[1] - linVelB[1];
        const dz = linVelA[2] - linVelB[2];
        jv = axis[0] * dx + axis[1] * dy + axis[2] * dz;
    } else if (movingA) {
        jv = axis[0] * linVelA[0] + axis[1] * linVelA[1] + axis[2] * linVelA[2];
    } else if (movingB) {
        jv = -(axis[0] * linVelB[0] + axis[1] * linVelB[1] + axis[2] * linVelB[2]);
    } else {
        jv = 0;
    }

    // calculate jacobian multiplied by angular velocity
    if (movingA) {
        jv += part.r1PlusUxAxis[0] * angVelA[0] + part.r1PlusUxAxis[1] * angVelA[1] + part.r1PlusUxAxis[2] * angVelA[2];
    }

    if (movingB) {
        jv -= part.r2xAxis[0] * angVelB[0] + part.r2xAxis[1] * angVelB[1] + part.r2xAxis[2] * angVelB[2];
    }

    // calculate lambda
    const lambda = part.effectiveMass * (jv - getSpringBias(part.springPart, part.totalLambda));

    return part.totalLambda + lambda;
}

/**
 * apply a total lambda value to cached velocity locals.
 * velocity-local version — writes to cached velocity vectors instead of body properties.
 * does NOT apply DOF masking — the caller handles that once after all constraint parts are solved.
 *
 * @param part the constraint part
 * @param linVelA linear velocity of body A (local copy, mutated)
 * @param angVelA angular velocity of body A (local copy, mutated)
 * @param linVelB linear velocity of body B (local copy, mutated)
 * @param angVelB angular velocity of body B (local copy, mutated)
 * @param isDynamicA true if body A is dynamic
 * @param isDynamicB true if body B is dynamic
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param axis constraint axis
 * @param totalLambda new total lambda to apply
 * @returns true if impulse was applied
 */
export function applyLambda(
    part: AxisConstraintPart,
    linVelA: Vec3,
    angVelA: Vec3,
    linVelB: Vec3,
    angVelB: Vec3,
    isDynamicA: boolean,
    isDynamicB: boolean,
    invMassA: number,
    invMassB: number,
    axis: Vec3,
    totalLambda: number,
): boolean {
    const deltaLambda = totalLambda - part.totalLambda;
    part.totalLambda = totalLambda;

    if (deltaLambda === 0) return false;

    // body A: subtract impulse
    if (isDynamicA) {
        const linearScale = deltaLambda * invMassA;
        linVelA[0] -= axis[0] * linearScale;
        linVelA[1] -= axis[1] * linearScale;
        linVelA[2] -= axis[2] * linearScale;

        const angularImpulse = part.invI1_r1PlusUxAxis;
        angVelA[0] -= angularImpulse[0] * deltaLambda;
        angVelA[1] -= angularImpulse[1] * deltaLambda;
        angVelA[2] -= angularImpulse[2] * deltaLambda;
    }

    // body B: add impulse
    if (isDynamicB) {
        const linearScale = deltaLambda * invMassB;
        linVelB[0] += axis[0] * linearScale;
        linVelB[1] += axis[1] * linearScale;
        linVelB[2] += axis[2] * linearScale;

        const angularImpulse = part.invI2_r2xAxis;
        angVelB[0] += angularImpulse[0] * deltaLambda;
        angVelB[1] += angularImpulse[1] * deltaLambda;
        angVelB[2] += angularImpulse[2] * deltaLambda;
    }

    return true;
}

/**
 * apply warm start impulse to cached velocity locals.
 * velocity-local version — writes to cached velocity vectors instead of body properties.
 * does NOT apply DOF masking — the caller handles that once after all constraint parts are warmed.
 *
 * @param part the constraint part
 * @param linVelA linear velocity of body A (local copy, mutated)
 * @param angVelA angular velocity of body A (local copy, mutated)
 * @param linVelB linear velocity of body B (local copy, mutated)
 * @param angVelB angular velocity of body B (local copy, mutated)
 * @param isDynamicA true if body A is dynamic
 * @param isDynamicB true if body B is dynamic
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param axis constraint axis
 * @param warmStartRatio scale factor for warm start (dt_new / dt_old)
 * @returns true if impulse was applied
 */
export function warmStart(
    part: AxisConstraintPart,
    linVelA: Vec3,
    angVelA: Vec3,
    linVelB: Vec3,
    angVelB: Vec3,
    isDynamicA: boolean,
    isDynamicB: boolean,
    invMassA: number,
    invMassB: number,
    axis: Vec3,
    warmStartRatio: number,
): boolean {
    // scale previous frame's lambda
    part.totalLambda *= warmStartRatio;

    if (part.totalLambda === 0) return false;

    // body A: subtract impulse
    if (isDynamicA) {
        const linearScaleA = part.totalLambda * invMassA;
        linVelA[0] -= axis[0] * linearScaleA;
        linVelA[1] -= axis[1] * linearScaleA;
        linVelA[2] -= axis[2] * linearScaleA;

        const angularImpulseA = part.invI1_r1PlusUxAxis;
        angVelA[0] -= angularImpulseA[0] * part.totalLambda;
        angVelA[1] -= angularImpulseA[1] * part.totalLambda;
        angVelA[2] -= angularImpulseA[2] * part.totalLambda;
    }

    // body B: add impulse
    if (isDynamicB) {
        const linearScaleB = part.totalLambda * invMassB;
        linVelB[0] += axis[0] * linearScaleB;
        linVelB[1] += axis[1] * linearScaleB;
        linVelB[2] += axis[2] * linearScaleB;

        const angularImpulseB = part.invI2_r2xAxis;
        angVelB[0] += angularImpulseB[0] * part.totalLambda;
        angVelB[1] += angularImpulseB[1] * part.totalLambda;
        angVelB[2] += angularImpulseB[2] * part.totalLambda;
    }

    return true;
}
