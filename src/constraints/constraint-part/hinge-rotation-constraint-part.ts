import type { Mat4, Vec3 } from 'mathcat';
import { mat2, mat4, vec3 } from 'mathcat';
import { addAngularVelocityStep, getInverseInertiaForRotation, subAngularVelocityStep } from '../../body/motion-properties';
import { MotionType } from '../../body/motion-type';
import type { RigidBody } from '../../body/rigid-body';
import { addRotationStep, subRotationStep } from '../../body/rigid-body-step';

/**
 * Constrains rotation around 2 axes so that it only allows rotation around 1 axis (the hinge axis).
 *
 * Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.4.1
 *
 * Constraint equation (eq 87):
 * C = [a1 · b2, a1 · c2]
 *
 * where:
 * - a1 = hinge axis on body A (world space)
 * - b2, c2 = axes perpendicular to hinge axis on body B (world space)
 *
 * Jacobian (eq 90):
 * J = [0, -b2 × a1, 0, b2 × a1]
 *     [0, -c2 × a1, 0, c2 × a1]
 *
 * This is a 2-DOF angular constraint that keeps the hinge axes aligned.
 */
export type HingeRotationConstraintPart = {
    /** world space hinge axis for body A */
    a1: Vec3;

    /** world space perpendicular to hinge axis for body B */
    b2: Vec3;

    /** world space perpendicular to hinge axis for body B (orthogonal to b2) */
    c2: Vec3;

    /** I1^-1 cached for velocity/position integration */
    invI1: Mat4;

    /** I2^-1 cached for velocity/position integration */
    invI2: Mat4;

    /** b2 × a1 (cross product, cached for jacobian) */
    b2xA1: Vec3;

    /** c2 × a1 (cross product, cached for jacobian) */
    c2xA1: Vec3;

    /** 2x2 effective mass matrix K^-1 = (J M^-1 J^T)^-1, stored as [m00, m01, m10, m11] */
    effectiveMass: [number, number, number, number];

    /** accumulated 2D impulse (for warm starting) [lambda0, lambda1] */
    totalLambda: [number, number];
};

/** Create a new HingeRotationConstraintPart with zero-initialized values */
export function create(): HingeRotationConstraintPart {
    return {
        a1: vec3.create(),
        b2: vec3.create(),
        c2: vec3.create(),
        invI1: mat4.create(),
        invI2: mat4.create(),
        b2xA1: vec3.create(),
        c2xA1: vec3.create(),
        effectiveMass: [0, 0, 0, 0], // 2x2 matrix
        totalLambda: [0, 0], // 2D impulse
    };
}

/** Deactivate this constraint part (zero out effective mass and lambda) */
export function deactivate(part: HingeRotationConstraintPart): void {
    part.effectiveMass[0] = 0;
    part.effectiveMass[1] = 0;
    part.effectiveMass[2] = 0;
    part.effectiveMass[3] = 0;
    part.totalLambda[0] = 0;
    part.totalLambda[1] = 0;
}

/** Check if constraint is active (has non-zero effective mass) */
export function isActive(part: HingeRotationConstraintPart): boolean {
    // check if any element of effective mass is non-zero
    return (
        part.effectiveMass[0] !== 0 || part.effectiveMass[1] !== 0 || part.effectiveMass[2] !== 0 || part.effectiveMass[3] !== 0
    );
}

const _calc_a2 = vec3.create();
const _calc_perp = vec3.create();
const _calc_summedInvInertia = mat4.create();
const _calc_temp = vec3.create();
const _calc_invEffMass = mat2.create();
const _calc_rotA = mat4.create();
const _calc_rotB = mat4.create();

/**
 * Calculate constraint properties for the hinge rotation constraint.
 * This sets up the effective mass matrix and cached cross products.
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceHingeAxis1 hinge axis for body A in world space (normalized)
 * @param worldSpaceHingeAxis2 hinge axis for body B in world space (normalized)
 */
export function calculateConstraintProperties(
    part: HingeRotationConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    worldSpaceHingeAxis1: Vec3,
    worldSpaceHingeAxis2: Vec3,
): void {
    // store a1 (body A's hinge axis)
    vec3.copy(part.a1, worldSpaceHingeAxis1);

    // calculate a2, handling case where axes are more than 90 degrees apart
    vec3.copy(_calc_a2, worldSpaceHingeAxis2);
    const dot = vec3.dot(part.a1, _calc_a2);

    if (dot <= 1.0e-3) {
        // world space axes are more than 90 degrees apart
        // get a perpendicular vector in the plane formed by a1 and a2
        vec3.scaleAndAdd(_calc_perp, _calc_a2, part.a1, -dot);

        if (vec3.squaredLength(_calc_perp) < 1.0e-6) {
            // a1 ~ -a2, take random perpendicular
            getPerpendicularVector(_calc_perp, part.a1);
        }

        // blend in a little bit from a1 so we're less than 90 degrees apart
        // result: 0.01 * a1 + 0.99 * perp (normalized)
        vec3.normalize(_calc_perp, _calc_perp);
        vec3.scale(_calc_a2, part.a1, 0.01);
        vec3.scaleAndAdd(_calc_a2, _calc_a2, _calc_perp, 0.99);
        vec3.normalize(_calc_a2, _calc_a2);
    }

    // calculate b2 and c2 (perpendicular to a2)
    getPerpendicularVector(part.b2, _calc_a2);
    vec3.cross(part.c2, _calc_a2, part.b2);

    // get inverse inertias
    if (bodyA.motionType === MotionType.DYNAMIC) {
        mat4.fromQuat(_calc_rotA, bodyA.quaternion);
        getInverseInertiaForRotation(part.invI1, bodyA.motionProperties, _calc_rotA);
    } else {
        mat4.zero(part.invI1);
    }

    if (bodyB.motionType === MotionType.DYNAMIC) {
        mat4.fromQuat(_calc_rotB, bodyB.quaternion);
        getInverseInertiaForRotation(part.invI2, bodyB.motionProperties, _calc_rotB);
    } else {
        mat4.zero(part.invI2);
    }

    // calculate cross products for jacobian
    vec3.cross(part.b2xA1, part.b2, part.a1);
    vec3.cross(part.c2xA1, part.c2, part.a1);

    // calculate summed inverse inertia (3x3 only)
    mat4.add(_calc_summedInvInertia, part.invI1, part.invI2);

    // calculate inverse effective mass: K = J M^-1 J^T (2x2 matrix)
    // K[0,0] = b2xA1 · (I_sum^-1 · b2xA1)
    // K[0,1] = b2xA1 · (I_sum^-1 · c2xA1)
    // K[1,0] = c2xA1 · (I_sum^-1 · b2xA1)
    // K[1,1] = c2xA1 · (I_sum^-1 · c2xA1)

    // compute I_sum^-1 · b2xA1 (3x3 only)
    mat4.multiply3x3Vec(_calc_temp, _calc_summedInvInertia, part.b2xA1);
    _calc_invEffMass[0] = vec3.dot(part.b2xA1, _calc_temp); // K[0,0]
    _calc_invEffMass[2] = vec3.dot(part.c2xA1, _calc_temp); // K[1,0]

    // compute I_sum^-1 · c2xA1 (3x3 only)
    mat4.multiply3x3Vec(_calc_temp, _calc_summedInvInertia, part.c2xA1);
    _calc_invEffMass[1] = vec3.dot(part.b2xA1, _calc_temp); // K[0,1]
    _calc_invEffMass[3] = vec3.dot(part.c2xA1, _calc_temp); // K[1,1]

    // invert 2x2 matrix to get effective mass
    const det = _calc_invEffMass[0] * _calc_invEffMass[3] - _calc_invEffMass[1] * _calc_invEffMass[2];
    if (Math.abs(det) < 1e-10) {
        deactivate(part);
        return;
    }

    const invDet = 1.0 / det;
    part.effectiveMass[0] = _calc_invEffMass[3] * invDet;
    part.effectiveMass[1] = -_calc_invEffMass[1] * invDet;
    part.effectiveMass[2] = -_calc_invEffMass[2] * invDet;
    part.effectiveMass[3] = _calc_invEffMass[0] * invDet;
}

const _ws_impulse = vec3.create();
const _ws_angularDelta = vec3.create();

/** Apply warm start impulse from previous frame */
export function warmStart(
    part: HingeRotationConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    warmStartImpulseRatio: number,
): void {
    part.totalLambda[0] *= warmStartImpulseRatio;
    part.totalLambda[1] *= warmStartImpulseRatio;

    applyVelocityStep(part, bodyA, bodyB, part.totalLambda[0], part.totalLambda[1]);
}

/** Internal helper to apply velocity step */
function applyVelocityStep(
    part: HingeRotationConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    lambda0: number,
    lambda1: number,
): boolean {
    if (lambda0 === 0 && lambda1 === 0) {
        return false;
    }

    // calculate impulse: P = J^T lambda = b2xA1 * lambda0 + c2xA1 * lambda1
    vec3.scale(_ws_impulse, part.b2xA1, lambda0);
    vec3.scaleAndAdd(_ws_impulse, _ws_impulse, part.c2xA1, lambda1);

    // body a: ω -= I1^-1 * impulse (3x3 only)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        mat4.multiply3x3Vec(_ws_angularDelta, part.invI1, _ws_impulse);
        subAngularVelocityStep(bodyA.motionProperties, _ws_angularDelta);
    }

    // body b: ω += I2^-1 * impulse (3x3 only)
    if (bodyB.motionType === MotionType.DYNAMIC) {
        mat4.multiply3x3Vec(_ws_angularDelta, part.invI2, _ws_impulse);
        addAngularVelocityStep(bodyB.motionProperties, _ws_angularDelta);
    }

    return true;
}

/**
 * Solve the velocity constraint.
 * Enforces d/dt C(...) = 0 where C is the constraint equation.
 */
export function solveVelocityConstraint(part: HingeRotationConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean {
    if (!isActive(part)) {
        return false;
    }

    // get angular velocity difference
    const mpA = bodyA.motionType !== MotionType.STATIC ? bodyA.motionProperties : null;
    const mpB = bodyB.motionType !== MotionType.STATIC ? bodyB.motionProperties : null;

    // calculate J * v (constraint velocity)
    // jv[0] = b2xA1 · (ω1 - ω2)
    // jv[1] = c2xA1 · (ω1 - ω2)
    let jv0 = 0;
    let jv1 = 0;

    if (mpA) {
        jv0 += vec3.dot(part.b2xA1, mpA.angularVelocity);
        jv1 += vec3.dot(part.c2xA1, mpA.angularVelocity);
    }
    if (mpB) {
        jv0 -= vec3.dot(part.b2xA1, mpB.angularVelocity);
        jv1 -= vec3.dot(part.c2xA1, mpB.angularVelocity);
    }

    // calculate lambda: lambda = -K^-1 * (J * v)
    // using 2x2 matrix multiply
    const lambda0 = part.effectiveMass[0] * jv0 + part.effectiveMass[1] * jv1;
    const lambda1 = part.effectiveMass[2] * jv0 + part.effectiveMass[3] * jv1;

    // accumulate
    part.totalLambda[0] += lambda0;
    part.totalLambda[1] += lambda1;

    return applyVelocityStep(part, bodyA, bodyB, lambda0, lambda1);
}

const _pos_impulse = vec3.create();
const _pos_angularDelta = vec3.create();

/**
 * Solve the position constraint (Baumgarte stabilization).
 * Enforces C(...) = 0.
 */
export function solvePositionConstraint(
    part: HingeRotationConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    baumgarte: number,
): boolean {
    if (!isActive(part)) {
        return false;
    }

    // calculate constraint error
    // C[0] = a1 · b2
    // C[1] = a1 · c2
    const c0 = vec3.dot(part.a1, part.b2);
    const c1 = vec3.dot(part.a1, part.c2);

    if (Math.abs(c0) < 1e-10 && Math.abs(c1) < 1e-10) {
        return false;
    }

    // calculate lambda: lambda = -K^-1 * beta * C
    // we don't divide by dt because we multiply by dt in position integration (cancelled out)
    const lambda0 = -baumgarte * (part.effectiveMass[0] * c0 + part.effectiveMass[1] * c1);
    const lambda1 = -baumgarte * (part.effectiveMass[2] * c0 + part.effectiveMass[3] * c1);

    // calculate impulse: P = J^T lambda
    vec3.scale(_pos_impulse, part.b2xA1, lambda0);
    vec3.scaleAndAdd(_pos_impulse, _pos_impulse, part.c2xA1, lambda1);

    // apply position correction (3x3 only)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        mat4.multiply3x3Vec(_pos_angularDelta, part.invI1, _pos_impulse);
        subRotationStep(bodyA, _pos_angularDelta);
    }
    if (bodyB.motionType === MotionType.DYNAMIC) {
        mat4.multiply3x3Vec(_pos_angularDelta, part.invI2, _pos_impulse);
        addRotationStep(bodyB, _pos_angularDelta);
    }

    return true;
}

/**
 * Helper to get a normalized vector perpendicular to the input vector.
 * Chooses perpendicular based on largest components for numerical stability.
 */
function getPerpendicularVector(out: Vec3, v: Vec3): Vec3 {
    const absX = Math.abs(v[0]);
    const absY = Math.abs(v[1]);

    if (absX > absY) {
        // Perpendicular in XZ plane: [z, 0, -x] / len
        const len = Math.sqrt(v[0] * v[0] + v[2] * v[2]);
        vec3.set(out, v[2] / len, 0, -v[0] / len);
    } else {
        // Perpendicular in YZ plane: [0, z, -y] / len
        const len = Math.sqrt(v[1] * v[1] + v[2] * v[2]);
        vec3.set(out, 0, v[2] / len, -v[1] / len);
    }

    return out;
}
