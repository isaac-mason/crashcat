import type { Mat4, Quat, Vec3 } from 'mathcat';
import { mat3, mat4, quat, vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body';
import { addRotationStep, subRotationStep } from '../../body/rigid-body-step';
import { MotionType } from '../../body/motion-type';
import * as motionProperties from '../../body/motion-properties';

/**
 * RotationEulerConstraintPart removes 3 rotational degrees of freedom.
 *
 * Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.5.1
 *
 * Constraint equation (eq 129):
 * C = [Δθ_x, Δθ_y, Δθ_z]
 *
 * Jacobian (eq 131):
 * J = [0, -E, 0, E]
 *
 * where E is the identity matrix.
 */
export type RotationEulerConstraintPart = {
    /** I1^-1 in world space (for position solve) */
    invI1: Mat4;

    /** I2^-1 in world space (for position solve) */
    invI2: Mat4;

    /** 3x3 effective mass matrix K^-1 = (J M^-1 J^T)^-1 */
    effectiveMass: Mat4;

    /** accumulated 3D angular impulse (for warm starting) */
    totalLambda: Vec3;
};

/** creates a new rotation euler constraint part */
export function create(): RotationEulerConstraintPart {
	const effectiveMass = mat4.create();
	mat4.zero(effectiveMass);
	return {
		invI1: mat4.create(),
		invI2: mat4.create(),
		effectiveMass,
		totalLambda: vec3.create(),
	};
}

/** deactivates the constraint part (resets state) */
export function deactivate(part: RotationEulerConstraintPart): void {
    mat4.zero(part.effectiveMass);
    vec3.set(part.totalLambda, 0, 0, 0);
}

/** checks if the constraint part is active */
export function isActive(part: RotationEulerConstraintPart): boolean {
    return part.effectiveMass[15] !== 0.0;
}

/**
 * Return inverse of initial rotation from body 1 to body 2 in body 1 space.
 *
 * q20 = q10 * r0
 * => r0 = q10^-1 * q20
 * => r0^-1 = q20^-1 * q10
 *
 * where:
 * q10 = initial orientation of body 1
 * q20 = initial orientation of body 2
 * r0 = initial rotation from body 1 to body 2
 */
export function getInvInitialOrientation(rotation1: Quat, rotation2: Quat): Quat {
    const result = quat.create();
    const q2Conj = quat.create();
    quat.conjugate(q2Conj, rotation2);
    quat.multiply(result, q2Conj, rotation1);
    return result;
}

const _getInvInitialOrientationXY_constraint1 = mat3.create();
const _getInvInitialOrientationXY_constraint2 = mat3.create();
const _getInvInitialOrientationXY_z1 = vec3.create();
const _getInvInitialOrientationXY_z2 = vec3.create();
const _getInvInitialOrientationXY_q1 = quat.create();
const _getInvInitialOrientationXY_q2 = quat.create();

/**
 * Return inverse of initial rotation from body 1 to body 2 in body 1 space,
 * given reference axes X and Y for both bodies.
 *
 * This is used when the constraint is specified in world space with explicit axes.
 *
 * r0^-1 = c2 * c1^-1
 *
 * where c1, c2 are rotation matrices built from the reference axes.
 */
export function getInvInitialOrientationXY(axisX1: Vec3, axisY1: Vec3, axisX2: Vec3, axisY2: Vec3): Quat {
    // if axes are identical, return identity
    if (vec3.equals(axisX1, axisX2) && vec3.equals(axisY1, axisY2)) {
        return quat.create(); // identity
    }

    // build rotation matrices from axes
    // z axis is x cross y
    vec3.cross(_getInvInitialOrientationXY_z1, axisX1, axisY1);
    vec3.cross(_getInvInitialOrientationXY_z2, axisX2, axisY2);

    // Create Mat33 rotation matrices (column-major)
    // Mat44(Vec4(axisX, 0), Vec4(axisY, 0), Vec4(axisX.Cross(axisY), 0), Vec4(0, 0, 0, 1))
    // We use mat3 to avoid scaling extraction in mat4.getRotation
    
    // biome-ignore format: pretty
    mat3.set(
        _getInvInitialOrientationXY_constraint1,
        axisX1[0], axisX1[1], axisX1[2], // column 0
        axisY1[0], axisY1[1], axisY1[2], // column 1
        _getInvInitialOrientationXY_z1[0],     _getInvInitialOrientationXY_z1[1],     _getInvInitialOrientationXY_z1[2]      // column 2
    );

    // biome-ignore format: pretty
    mat3.set(
        _getInvInitialOrientationXY_constraint2,
        axisX2[0], axisX2[1], axisX2[2], // column 0
        axisY2[0], axisY2[1], axisY2[2], // column 1
        _getInvInitialOrientationXY_z2[0],     _getInvInitialOrientationXY_z2[1],     _getInvInitialOrientationXY_z2[2]      // column 2
    );

    // convert mat3 to quaternions
    quat.fromMat3(_getInvInitialOrientationXY_q1, _getInvInitialOrientationXY_constraint1);
    quat.fromMat3(_getInvInitialOrientationXY_q2, _getInvInitialOrientationXY_constraint2);

    // return constraint2.GetQuaternion() * constraint1.GetQuaternion().Conjugated()
    const q1Conj = quat.create();
    quat.conjugate(q1Conj, _getInvInitialOrientationXY_q1);

    const result = quat.create();
    quat.multiply(result, _getInvInitialOrientationXY_q2, q1Conj);

    return result;
}

const _calc_invEffectiveMass = mat4.create();

/**
 * Calculate constraint properties for the rotation constraint.
 * Computes the 3x3 effective mass matrix.
 *
 * For the rotation-only Jacobian J = [0, -E, 0, E]:
 * K = J * M^-1 * J^T = I1^-1 + I2^-1
 *
 * We then invert K to get the effective mass.
 */
export function calculateConstraintProperties(
    part: RotationEulerConstraintPart,
    bodyA: RigidBody,
    _rotationA: Mat4,
    bodyB: RigidBody,
    _rotationB: Mat4,
): void {
    // get inverse inertia in world space for both bodies
    if (bodyA.motionType === MotionType.DYNAMIC) {
        motionProperties.getInverseInertiaForRotation(part.invI1, bodyA.motionProperties, _rotationA);
    } else {
        mat4.zero(part.invI1);
    }

    if (bodyB.motionType === MotionType.DYNAMIC) {
        motionProperties.getInverseInertiaForRotation(part.invI2, bodyB.motionProperties, _rotationB);
    } else {
        mat4.zero(part.invI2);
    }

    // compute inverse effective mass: K^-1 = I1^-1 + I2^-1 (3x3 only)
    const invEffectiveMass = _calc_invEffectiveMass;
    mat4.add(invEffectiveMass, part.invI1, part.invI2);

    // calculate effective mass: K^-1 = (J M^-1 J^T)^-1
    if (!mat4.invert3x3(part.effectiveMass, invEffectiveMass)) {
        // singular matrix - both bodies are static/kinematic
        deactivate(part);
    }
}

/**
 * Warm start the velocity constraint by applying cached impulses.
 * Scales the previous frame's impulses by the warm start ratio.
 */
export function warmStart(
    part: RotationEulerConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    warmStartImpulseRatio: number,
): void {
    // scale cached impulse
    vec3.scale(part.totalLambda, part.totalLambda, warmStartImpulseRatio);

    // apply impulse
    applyVelocityStep(part, bodyA, bodyB, part.totalLambda);
}

const _solveVel_jv = vec3.create();
const _solveVel_lambda = vec3.create();
const _solveVel_w1 = vec3.create();
const _solveVel_w2 = vec3.create();

/**
 * Solve the velocity constraint.
 *
 * Computes Jacobian velocity: J * v = w1 - w2
 * Solves for lambda: λ = -K^-1 * (J * v)
 * Applies impulse to correct angular velocities.
 *
 * Note: Both dynamic and kinematic bodies contribute velocity to the Jacobian calculation,
 * but only dynamic bodies receive impulse corrections.
 */
export function solveVelocityConstraint(part: RotationEulerConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean {
    // compute Jacobian velocity: J * v = w1 - w2
    // include velocities from both dynamic AND kinematic bodies (not static)
    const jv = _solveVel_jv;

    // get angular velocities - zero for static bodies
    const w1 = _solveVel_w1;
    const w2 = _solveVel_w2;

    if (bodyA.motionType !== MotionType.STATIC) {
        vec3.copy(w1, bodyA.motionProperties.angularVelocity);
    } else {
        vec3.zero(w1);
    }

    if (bodyB.motionType !== MotionType.STATIC) {
        vec3.copy(w2, bodyB.motionProperties.angularVelocity);
    } else {
        vec3.zero(w2);
    }

    vec3.subtract(jv, w1, w2);

    // compute lambda: λ = K * (J * v) (3x3 only)
    // the negation is handled by subtracting from body A
    const lambda = _solveVel_lambda;
    mat4.multiply3x3Vec(lambda, part.effectiveMass, jv);

    // accumulate impulse
    vec3.add(part.totalLambda, part.totalLambda, lambda);

    // apply impulse
    return applyVelocityStep(part, bodyA, bodyB, lambda);
}

const _applyVel_angularImpulseA = vec3.create();
const _applyVel_angularImpulseB = vec3.create();

/**
 * Apply an angular impulse to both bodies.
 *
 * Angular impulse: Δw = I^-1 * λ
 */
function applyVelocityStep(
    part: RotationEulerConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    lambda: Vec3,
): boolean {
    // check for zero impulse
    if (vec3.squaredLength(lambda) === 0) {
        return false;
    }

    // body a: subtract angular impulse (3x3 only)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const angularImpulse = _applyVel_angularImpulseA;
        mat4.multiply3x3Vec(angularImpulse, part.invI1, lambda);
        vec3.subtract(bodyA.motionProperties.angularVelocity, bodyA.motionProperties.angularVelocity, angularImpulse);
    }

    // body b: add angular impulse (3x3 only)
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const angularImpulse = _applyVel_angularImpulseB;
        mat4.multiply3x3Vec(angularImpulse, part.invI2, lambda);
        vec3.add(bodyB.motionProperties.angularVelocity, bodyB.motionProperties.angularVelocity, angularImpulse);
    }

    return true;
}

const _solvePos_diff = quat.create();
const _solvePos_error = vec3.create();
const _solvePos_lambda = vec3.create();
const _solvePos_angularStep = vec3.create();
const _solvePos_q1Conj = quat.create();

/**
 * Solve the position constraint using Baumgarte stabilization.
 *
 * Computes rotation error:
 * diff = q2 * invInitialOrientation * q1^-1
 *
 * The rotation should be: q2 = q1 * r0
 * But due to drift: q2 = diff * q1 * r0
 * => diff = q2 * r0^-1 * q1^-1
 *
 * For small angles, error ≈ 2 * diff.xyz (assuming diff.w is positive)
 *
 * Solves for lambda: λ = -K^-1 * β * error
 * Applies rotation correction directly.
 */
export function solvePositionConstraint(
    part: RotationEulerConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invInitialOrientation: Quat,
    baumgarte: number,
): boolean {
    // calculate rotation error: diff = q2 * invInitialOrientation * q1^-1
    const diff = _solvePos_diff;

    // q2 * invInitialOrientation
    quat.multiply(diff, bodyB.quaternion, invInitialOrientation);

    // * q1^-1
    const q1Conj = _solvePos_q1Conj;
    quat.conjugate(q1Conj, bodyA.quaternion);
    quat.multiply(diff, diff, q1Conj);

    // ensure positive w for consistent angle extraction
    if (diff[3] < 0) {
        quat.scale(diff, diff, -1);
    }

    // extract error from quaternion
    // for small angles: error ≈ 2 * [x, y, z]
    const error = _solvePos_error;
    vec3.set(error, 2 * diff[0], 2 * diff[1], 2 * diff[2]);

    // check for zero error
    if (vec3.squaredLength(error) < 1e-12) {
        return false;
    }

    // compute lambda: λ = -K * β * error (3x3 only)
    const lambda = _solvePos_lambda;
    mat4.multiply3x3Vec(lambda, part.effectiveMass, error);
    vec3.scale(lambda, lambda, -baumgarte);

    // apply rotation correction using body step functions
    // body a: subtract rotation (3x3 only)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const angularStep = _solvePos_angularStep;
        mat4.multiply3x3Vec(angularStep, part.invI1, lambda);
        subRotationStep(bodyA, angularStep);
    }

    // body b: add rotation (3x3 only)
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const angularStep = _solvePos_angularStep;
        mat4.multiply3x3Vec(angularStep, part.invI2, lambda);
        addRotationStep(bodyB, angularStep);
    }

    return true;
}