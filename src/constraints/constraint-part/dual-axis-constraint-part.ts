import type { Mat2, Mat4, Vec2, Vec3 } from 'mathcat';
import { mat2, mat4, vec2, vec3 } from 'mathcat';
import {
    addAngularVelocityStep,
    addLinearVelocityStep,
    getInverseInertiaForRotation,
    subAngularVelocityStep,
    subLinearVelocityStep,
} from '../../body/motion-properties';
import { MotionType } from '../../body/motion-type';
import type { RigidBody } from '../../body/rigid-body';
import { addPositionStep, addRotationStep, subPositionStep, subRotationStep } from '../../body/rigid-body-step';

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
    // Cached cross products for Jacobian computation
    /** (r1 + u) × n1 */
    r1PlusUxN1: Vec3;
    /** (r1 + u) × n2 */
    r1PlusUxN2: Vec3;
    /** r2 × n1 */
    r2xN1: Vec3;
    /** r2 × n2 */
    r2xN2: Vec3;

    // Cached I^-1 × (r × n) terms for velocity/position application
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
export function create(): DualAxisConstraintPart {
    return {
        r1PlusUxN1: vec3.create(),
        r1PlusUxN2: vec3.create(),
        r2xN1: vec3.create(),
        r2xN2: vec3.create(),
        invI1_r1PlusUxN1: vec3.create(),
        invI1_r1PlusUxN2: vec3.create(),
        invI2_r2xN1: vec3.create(),
        invI2_r2xN2: vec3.create(),
		effectiveMass: mat2.set(mat2.create(), 0, 0, 0, 0),
		totalLambda: vec2.create(),
    };
}

/** Deactivates the constraint part (resets state) */
export function deactivate(part: DualAxisConstraintPart): void {
    vec2.zero(part.totalLambda);
    mat2.set(part.effectiveMass, 0, 0, 0, 0);
}

/** Checks if the constraint part is active (has invertible effective mass) */
export function isActive(part: DualAxisConstraintPart): boolean {
    // check if the effective mass matrix is non-zero by checking if any element is non-zero
    // a zero matrix indicates the constraint is degenerate/inactive
    return (
        part.effectiveMass[0] !== 0 ||
        part.effectiveMass[1] !== 0 ||
        part.effectiveMass[2] !== 0 ||
        part.effectiveMass[3] !== 0
    );
}

const _calc_invEffectiveMass = mat2.create();
const _calc_invI = mat4.create();
const _calc_crossN1 = vec3.create();
const _calc_crossN2 = vec3.create();
const _calc_invI_crossN1 = vec3.create();
const _calc_invI_crossN2 = vec3.create();

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
export function calculateConstraintProperties(
    part: DualAxisConstraintPart,
    bodyA: RigidBody,
    rotationA: Mat4,
    r1PlusU: Vec3,
    bodyB: RigidBody,
    rotationB: Mat4,
    r2: Vec3,
    n1: Vec3,
    n2: Vec3,
): void {
    // initialize inverse effective mass to zero
    mat2.set(_calc_invEffectiveMass, 0, 0, 0, 0);

    // calculate contributions from body a
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const mpA = bodyA.motionProperties;

        // calculate cross products: (r1 + u) × n
        vec3.cross(_calc_crossN1, r1PlusU, n1);
        vec3.cross(_calc_crossN2, r1PlusU, n2);
        vec3.copy(part.r1PlusUxN1, _calc_crossN1);
        vec3.copy(part.r1PlusUxN2, _calc_crossN2);

        // get inverse inertia in world space
        getInverseInertiaForRotation(_calc_invI, mpA, rotationA);

        // calculate I^-1 × (r × n) (3x3 only)
        mat4.multiply3x3Vec(_calc_invI_crossN1, _calc_invI, _calc_crossN1);
        mat4.multiply3x3Vec(_calc_invI_crossN2, _calc_invI, _calc_crossN2);
        vec3.copy(part.invI1_r1PlusUxN1, _calc_invI_crossN1);
        vec3.copy(part.invI1_r1PlusUxN2, _calc_invI_crossN2);

        // add to inverse effective mass: K^-1 = (J M^-1 J^T)
        // store in column-major order: [m00, m10, m01, m11]
        // inv_effective_mass(0,0) += invMass + (r1+u)×n1 · I1^-1 × (r1+u)×n1
        // inv_effective_mass(1,0) += (r1+u)×n2 · I1^-1 × (r1+u)×n1
        // inv_effective_mass(0,1) += (r1+u)×n1 · I1^-1 × (r1+u)×n2
        // inv_effective_mass(1,1) += invMass + (r1+u)×n2 · I1^-1 × (r1+u)×n2
        _calc_invEffectiveMass[0] += mpA.invMass + vec3.dot(_calc_crossN1, _calc_invI_crossN1); // m00
        _calc_invEffectiveMass[1] += vec3.dot(_calc_crossN2, _calc_invI_crossN1); // m10
        _calc_invEffectiveMass[2] += vec3.dot(_calc_crossN1, _calc_invI_crossN2); // m01
        _calc_invEffectiveMass[3] += mpA.invMass + vec3.dot(_calc_crossN2, _calc_invI_crossN2); // m11
    }

    // calculate contributions from body b
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const mpB = bodyB.motionProperties;

        // calculate cross products: r2 × n
        vec3.cross(_calc_crossN1, r2, n1);
        vec3.cross(_calc_crossN2, r2, n2);
        vec3.copy(part.r2xN1, _calc_crossN1);
        vec3.copy(part.r2xN2, _calc_crossN2);

        // get inverse inertia in world space
        getInverseInertiaForRotation(_calc_invI, mpB, rotationB);

        // calculate I^-1 × (r × n) (3x3 only)
        mat4.multiply3x3Vec(_calc_invI_crossN1, _calc_invI, _calc_crossN1);
        mat4.multiply3x3Vec(_calc_invI_crossN2, _calc_invI, _calc_crossN2);
        vec3.copy(part.invI2_r2xN1, _calc_invI_crossN1);
        vec3.copy(part.invI2_r2xN2, _calc_invI_crossN2);

        // add to inverse effective mass (column-major order: [m00, m10, m01, m11])
        _calc_invEffectiveMass[0] += mpB.invMass + vec3.dot(_calc_crossN1, _calc_invI_crossN1); // m00
        _calc_invEffectiveMass[1] += vec3.dot(_calc_crossN2, _calc_invI_crossN1); // m10
        _calc_invEffectiveMass[2] += vec3.dot(_calc_crossN1, _calc_invI_crossN2); // m01
        _calc_invEffectiveMass[3] += mpB.invMass + vec3.dot(_calc_crossN2, _calc_invI_crossN2); // m11
    }

    // invert to get effective mass
    const inverted = mat2.invert(part.effectiveMass, _calc_invEffectiveMass);

    if (inverted === null) {
        deactivate(part);
    }
}

const _warmStart_impulse = vec3.create();
const _warmStart_scaledLambda = vec3.create();

/**
 * Warm start the velocity constraint by applying cached impulses.
 * Scales the previous frame's impulses by the warm start ratio.
 */
export function warmStart(
    part: DualAxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    n1: Vec3,
    n2: Vec3,
    warmStartImpulseRatio: number,
): void {
    if (!isActive(part)) return;

    // scale previous frame's lambda
    part.totalLambda[0] *= warmStartImpulseRatio;
    part.totalLambda[1] *= warmStartImpulseRatio;

    if (part.totalLambda[0] === 0 && part.totalLambda[1] === 0) return;

    // calculate impulse: P = n1 * lambda[0] + n2 * lambda[1]
    vec3.scale(_warmStart_impulse, n1, part.totalLambda[0]);
    vec3.scaleAndAdd(_warmStart_impulse, _warmStart_impulse, n2, part.totalLambda[1]);

    // apply to body a (subtract)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const mpA = bodyA.motionProperties;

        // linear velocity
        vec3.scale(_warmStart_scaledLambda, _warmStart_impulse, mpA.invMass);
        subLinearVelocityStep(mpA, _warmStart_scaledLambda);

        // angular velocity: ω -= lambda[0] * invI1_r1PlusUxN1 + lambda[1] * invI1_r1PlusUxN2
        vec3.scale(_warmStart_scaledLambda, part.invI1_r1PlusUxN1, part.totalLambda[0]);
        vec3.scaleAndAdd(_warmStart_scaledLambda, _warmStart_scaledLambda, part.invI1_r1PlusUxN2, part.totalLambda[1]);
        subAngularVelocityStep(mpA, _warmStart_scaledLambda);
    }

    // apply to body b (add)
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const mpB = bodyB.motionProperties;

        // linear velocity
        vec3.scale(_warmStart_scaledLambda, _warmStart_impulse, mpB.invMass);
        addLinearVelocityStep(mpB, _warmStart_scaledLambda);

        // angular velocity: ω += lambda[0] * invI2_r2xN1 + lambda[1] * invI2_r2xN2
        vec3.scale(_warmStart_scaledLambda, part.invI2_r2xN1, part.totalLambda[0]);
        vec3.scaleAndAdd(_warmStart_scaledLambda, _warmStart_scaledLambda, part.invI2_r2xN2, part.totalLambda[1]);
        addAngularVelocityStep(mpB, _warmStart_scaledLambda);
    }
}

const _sv_jv: [number, number] = [0, 0];
const _sv_lambda: [number, number] = [0, 0];
const _sv_impulse = vec3.create();
const _sv_angularImpulse = vec3.create();

/**
 * Solve the velocity constraint.
 * Iteratively update to make d/dt C(...) = 0.
 *
 * Note: Both dynamic and kinematic bodies contribute velocity to the Jacobian calculation,
 * but only dynamic bodies receive impulse corrections.
 *
 * @returns True if any impulse was applied
 */
export function solveVelocityConstraint(
    part: DualAxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    n1: Vec3,
    n2: Vec3,
): boolean {
    if (!isActive(part)) return false;

    // calculate J * v (velocity constraint error)
    // jv[0] = n1 · (v1 - v2) + (r1+u)×n1 · ω1 - r2×n1 · ω2
    // jv[1] = n2 · (v1 - v2) + (r1+u)×n2 · ω1 - r2×n2 · ω2

    // For velocity calculation: include both dynamic AND kinematic bodies
    const mpAForVelocity = bodyA.motionType !== MotionType.STATIC ? bodyA.motionProperties : null;
    const mpBForVelocity = bodyB.motionType !== MotionType.STATIC ? bodyB.motionProperties : null;

    // For impulse application: only dynamic bodies
    const mpAForImpulse = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties : null;
    const mpBForImpulse = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties : null;

    // linear contribution
    _sv_jv[0] = 0;
    _sv_jv[1] = 0;

    if (mpAForVelocity && mpBForVelocity) {
        // v1 - v2
        _sv_jv[0] = vec3.dot(n1, mpAForVelocity.linearVelocity) - vec3.dot(n1, mpBForVelocity.linearVelocity);
        _sv_jv[1] = vec3.dot(n2, mpAForVelocity.linearVelocity) - vec3.dot(n2, mpBForVelocity.linearVelocity);
    } else if (mpAForVelocity) {
        _sv_jv[0] = vec3.dot(n1, mpAForVelocity.linearVelocity);
        _sv_jv[1] = vec3.dot(n2, mpAForVelocity.linearVelocity);
    } else if (mpBForVelocity) {
        _sv_jv[0] = -vec3.dot(n1, mpBForVelocity.linearVelocity);
        _sv_jv[1] = -vec3.dot(n2, mpBForVelocity.linearVelocity);
    }

    // angular contribution
    if (mpAForVelocity) {
        _sv_jv[0] += vec3.dot(part.r1PlusUxN1, mpAForVelocity.angularVelocity);
        _sv_jv[1] += vec3.dot(part.r1PlusUxN2, mpAForVelocity.angularVelocity);
    }
    if (mpBForVelocity) {
        _sv_jv[0] -= vec3.dot(part.r2xN1, mpBForVelocity.angularVelocity);
        _sv_jv[1] -= vec3.dot(part.r2xN2, mpBForVelocity.angularVelocity);
    }

    // calculate lambda: lambda = K * jv
    // note: jv already computes the velocity constraint error in the correct direction
    vec2.transformMat2(_sv_lambda, _sv_jv, part.effectiveMass);

    // store accumulated lambda
    part.totalLambda[0] += _sv_lambda[0];
    part.totalLambda[1] += _sv_lambda[1];

    // check if any impulse to apply
    if (_sv_lambda[0] === 0 && _sv_lambda[1] === 0) return false;

    // apply impulse
    // P = n1 * lambda[0] + n2 * lambda[1]
    vec3.scale(_sv_impulse, n1, _sv_lambda[0]);
    vec3.scaleAndAdd(_sv_impulse, _sv_impulse, n2, _sv_lambda[1]);

    // body a (subtract) - only dynamic bodies receive impulses
    if (mpAForImpulse) {
        // linear
        vec3.scale(_sv_angularImpulse, _sv_impulse, mpAForImpulse.invMass);
        subLinearVelocityStep(mpAForImpulse, _sv_angularImpulse);

        // angular
        vec3.scale(_sv_angularImpulse, part.invI1_r1PlusUxN1, _sv_lambda[0]);
        vec3.scaleAndAdd(_sv_angularImpulse, _sv_angularImpulse, part.invI1_r1PlusUxN2, _sv_lambda[1]);
        subAngularVelocityStep(mpAForImpulse, _sv_angularImpulse);
    }

    // body b (add) - only dynamic bodies receive impulses
    if (mpBForImpulse) {
        // linear
        vec3.scale(_sv_angularImpulse, _sv_impulse, mpBForImpulse.invMass);
        addLinearVelocityStep(mpBForImpulse, _sv_angularImpulse);

        // angular
        vec3.scale(_sv_angularImpulse, part.invI2_r2xN1, _sv_lambda[0]);
        vec3.scaleAndAdd(_sv_angularImpulse, _sv_angularImpulse, part.invI2_r2xN2, _sv_lambda[1]);
        addAngularVelocityStep(mpBForImpulse, _sv_angularImpulse);
    }

    return true;
}

const _sp_c = vec2.create();
const _sp_lambda = vec2.create();
const _sp_impulse = vec3.create();
const _sp_angularImpulse = vec3.create();

/**
 * Solve the position constraint.
 * Iteratively update to make C(...) = 0.
 *
 * @param u The separation vector (p2 - p1) in world space
 * @param baumgarte Baumgarte stabilization factor
 * @returns True if any correction was applied
 */
export function solvePositionConstraint(
    part: DualAxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    u: Vec3,
    n1: Vec3,
    n2: Vec3,
    baumgarte: number,
): boolean {
    if (!isActive(part)) return false;

    // calculate constraint error: C = [u · n1, u · n2]
    _sp_c[0] = vec3.dot(u, n1);
    _sp_c[1] = vec3.dot(u, n2);

    if (_sp_c[0] === 0 && _sp_c[1] === 0) return false;

    // calculate lambda: lambda = -baumgarte * K^-1 * C
    vec2.transformMat2(_sp_lambda, _sp_c, part.effectiveMass);
    vec2.scale(_sp_lambda, _sp_lambda, -baumgarte);

    if (_sp_lambda[0] === 0 && _sp_lambda[1] === 0) return false;

    // apply position correction
    // P = n1 * lambda[0] + n2 * lambda[1]
    vec3.scale(_sp_impulse, n1, _sp_lambda[0]);
    vec3.scaleAndAdd(_sp_impulse, _sp_impulse, n2, _sp_lambda[1]);

    // body a (subtract)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const mpA = bodyA.motionProperties;

        // position
        vec3.scale(_sp_angularImpulse, _sp_impulse, mpA.invMass);
        subPositionStep(bodyA, _sp_angularImpulse);

        // rotation
        vec3.scale(_sp_angularImpulse, part.invI1_r1PlusUxN1, _sp_lambda[0]);
        vec3.scaleAndAdd(_sp_angularImpulse, _sp_angularImpulse, part.invI1_r1PlusUxN2, _sp_lambda[1]);
        subRotationStep(bodyA, _sp_angularImpulse);
    }

    // body b (add)
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const mpB = bodyB.motionProperties;

        // position
        vec3.scale(_sp_angularImpulse, _sp_impulse, mpB.invMass);
        addPositionStep(bodyB, _sp_angularImpulse);

        // rotation
        vec3.scale(_sp_angularImpulse, part.invI2_r2xN1, _sp_lambda[0]);
        vec3.scaleAndAdd(_sp_angularImpulse, _sp_angularImpulse, part.invI2_r2xN2, _sp_lambda[1]);
        addRotationStep(bodyB, _sp_angularImpulse);
    }

    return true;
}
