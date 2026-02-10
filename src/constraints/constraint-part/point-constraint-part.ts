import type { Mat4, Vec3 } from 'mathcat';
import { mat4, vec3 } from 'mathcat';
import * as motionProperties from '../../body/motion-properties';
import { MotionType } from '../../body/motion-type';
import type { RigidBody } from '../../body/rigid-body';
import { addPositionStep, addRotationStep, subPositionStep, subRotationStep } from '../../body/rigid-body-step';

/**
 * PointConstraintPart removes 3 translational degrees of freedom.
 * Used by PointConstraint, HingeConstraint (for point attachment), etc.
 *
 * This is the 3-axis version of AxisConstraintPart, using a 3x3 effective mass matrix.
 */
export type PointConstraintPart = {
    /** r1 in world space (moment arm from body1 COM to constraint point) */
    r1: Vec3;

    /** r2 in world space (moment arm from body2 COM to constraint point) */
    r2: Vec3;

    /** I1^-1 * [r1]× - cached for impulse application */
    invI1_r1X: Mat4;

    /** I2^-1 * [r2]× - cached for impulse application */
    invI2_r2X: Mat4;

    /** 3x3 effective mass matrix K^-1 = (J M^-1 J^T)^-1 */
    effectiveMass: Mat4;

    /** accumulated 3D impulse (for warm starting) */
    totalLambda: Vec3;
};

/** creates a new point constraint part */
export function create(): PointConstraintPart {
    return {
        r1: vec3.create(),
        r2: vec3.create(),
        invI1_r1X: mat4.create(),
        invI2_r2X: mat4.create(),
        effectiveMass: mat4.create(),
        totalLambda: vec3.create(),
    };
}

/** deactivates the constraint part (resets state) */
export function deactivate(part: PointConstraintPart): void {
    mat4.zero(part.effectiveMass);
    vec3.set(part.totalLambda, 0, 0, 0);
}

/** checks if the constraint part is active */
export function isActive(part: PointConstraintPart): boolean {
    return part.effectiveMass[15] !== 0;
}

const _calc_r1Cross = /* @__PURE__ */ mat4.create();
const _calc_r2Cross = /* @__PURE__ */ mat4.create();
const _calc_invEffectiveMass = /* @__PURE__ */ mat4.create();
const _calc_temp1 = /* @__PURE__ */ mat4.create();
const _calc_temp2 = /* @__PURE__ */ mat4.create();
const _calc_invInertia1 = /* @__PURE__ */ mat4.create();
const _calc_invInertia2 = /* @__PURE__ */ mat4.create();

/**
 * Calculate constraint properties for the point constraint.
 * Computes the 3x3 effective mass matrix and cached angular terms.
 *
 * Formula: K = (J M^-1 J^T)
 * where J = [-I, -[r1]×, I, [r2]×] is the constraint Jacobian
 *
 * For 3-axis constraint:
 * K^-1 = m1^-1 * I + [r1]× * I1^-1 * [r1]×^T + m2^-1 * I + [r2]× * I2^-1 * [r2]×^T
 *
 * We then invert K^-1 to get the effective mass K.
 */
export function calculateConstraintProperties(
    part: PointConstraintPart,
    bodyA: RigidBody,
    rotationA: Mat4,
    r1Local: Vec3,
    bodyB: RigidBody,
    rotationB: Mat4,
    r2Local: Vec3,
): void {
    // transform local space moment arms to world space
    mat4.multiply3x3Vec(part.r1, rotationA, r1Local);
    mat4.multiply3x3Vec(part.r2, rotationB, r2Local);

    // build inverse effective mass matrix
    const invEffectiveMass = _calc_invEffectiveMass;
    mat4.zero(invEffectiveMass);

    let summedInvMass = 0;

    // body a contribution
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const mp1 = bodyA.motionProperties;
        summedInvMass += mp1.invMass;

        // get inverse inertia in world space
        const invInertia1 = _calc_invInertia1;
        motionProperties.getInverseInertiaForRotation(invInertia1, mp1, rotationA);

        // create [r1]× cross product matrix
        const r1Cross = _calc_r1Cross;
        mat4.crossProductMatrix(r1Cross, part.r1);

        // cache I1^-1 * [r1]× for impulse application (3x3 only)
        mat4.multiply3x3(part.invI1_r1X, invInertia1, r1Cross);

        // add [r1]× * I1^-1 * [r1]×^T to inverse effective mass
        // This is: [r1]× * I1^-1 * [r1]×^T (3x3 only)
        const temp1 = _calc_temp1;
        mat4.multiply3x3(temp1, r1Cross, invInertia1);

        const temp2 = _calc_temp2;
        // multiply with transposed r1Cross (3x3 only)
        mat4.multiply3x3RightTransposed(temp2, temp1, r1Cross);

        mat4.add(invEffectiveMass, invEffectiveMass, temp2);
    } else {
        // static body - zero contribution
        mat4.zero(part.invI1_r1X);
    }

    // body B contribution
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const mp2 = bodyB.motionProperties;
        summedInvMass += mp2.invMass;

        // get inverse inertia in world space
        const invInertia2 = _calc_invInertia2;
        motionProperties.getInverseInertiaForRotation(invInertia2, mp2, rotationB);

        // create [r2]× cross product matrix
        const r2Cross = _calc_r2Cross;
        mat4.crossProductMatrix(r2Cross, part.r2);

        // cache I2^-1 * [r2]× for impulse application (3x3 only)
        mat4.multiply3x3(part.invI2_r2X, invInertia2, r2Cross);

        // add [r2]× * I2^-1 * [r2]×^T to inverse effective mass
        // This is: [r2]× * I2^-1 * [r2]×^T (3x3 only)
        const temp1 = _calc_temp1;
        mat4.multiply3x3(temp1, r2Cross, invInertia2);

        const temp2 = _calc_temp2;
        // multiply with transposed r2Cross (3x3 only)
        mat4.multiply3x3RightTransposed(temp2, temp1, r2Cross);

        mat4.add(invEffectiveMass, invEffectiveMass, temp2);
    } else {
        // static body - zero contribution
        mat4.zero(part.invI2_r2X);
    }

    // add linear mass term (diagonal)
    // Mat4 is column-major, so diagonal indices are [0, 5, 10, 15]
    invEffectiveMass[0] += summedInvMass;
    invEffectiveMass[5] += summedInvMass;
    invEffectiveMass[10] += summedInvMass;

    // invert to get effective mass
    const effectiveMass = mat4.invert3x3(part.effectiveMass, invEffectiveMass);

    if (effectiveMass === null) {
        // singular matrix - constraint is degenerate
        deactivate(part);
    }
}

/**
 * Warm start the velocity constraint by applying cached impulses.
 * Scales the previous frame's impulses by the warm start ratio.
 */
export function warmStart(part: PointConstraintPart, bodyA: RigidBody, bodyB: RigidBody, warmStartImpulseRatio: number): void {
    if (!isActive(part)) {
        return;
    }

    // scale cached impulse (no negation - apply accumulated lambda directly)
    vec3.scale(part.totalLambda, part.totalLambda, warmStartImpulseRatio);

    // apply impulse
    applyVelocityStep(part, bodyA, bodyB, part.totalLambda);
}

// temp variables for solve velocity
const _solveVel_jv = /* @__PURE__ */ vec3.create();
const _solveVel_lambda = /* @__PURE__ */ vec3.create();
const _solveVel_angA = /* @__PURE__ */ vec3.create();
const _solveVel_angB = /* @__PURE__ */ vec3.create();
const _solveVel_crossA = /* @__PURE__ */ vec3.create();
const _solveVel_crossB = /* @__PURE__ */ vec3.create();

/**
 * Solve the velocity constraint.
 *
 * Computes Jacobian velocity: J * v = v1 - [r1]× * w1 - v2 + [r2]× * w2
 * Solves for lambda: λ = -K^-1 * (J * v)
 * Applies impulse to correct velocities.
 */
export function solveVelocityConstraint(part: PointConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean {
    if (!isActive(part)) {
        return false;
    }

    // compute jacobian velocity: J * v
    // J * v = v1 - [r1]× * w1 - v2 + [r2]× * w2
    // Note: [r1]× * w = r × w

    const jv = _solveVel_jv;
    vec3.copy(jv, bodyA.motionProperties.linearVelocity);

    // subtract r1 × w1
    const angA = _solveVel_angA;
    vec3.copy(angA, bodyA.motionProperties.angularVelocity);
    const crossA = _solveVel_crossA;
    vec3.cross(crossA, part.r1, angA);
    vec3.subtract(jv, jv, crossA);

    // subtract v2
    vec3.subtract(jv, jv, bodyB.motionProperties.linearVelocity);

    // add r2 × w2
    const angB = _solveVel_angB;
    vec3.copy(angB, bodyB.motionProperties.angularVelocity);
    const crossB = _solveVel_crossB;
    vec3.cross(crossB, part.r2, angB);
    vec3.add(jv, jv, crossB);

    // compute lambda: λ = K * (v1 - r1×ω1 - v2 + r2×ω2) (3x3 only)
    // Note: this computes -J·v, so no negation needed
    const lambda = _solveVel_lambda;
    mat4.multiply3x3Vec(lambda, part.effectiveMass, jv);

    // accumulate impulse
    vec3.add(part.totalLambda, part.totalLambda, lambda);

    // apply impulse
    return applyVelocityStep(part, bodyA, bodyB, lambda);
}

// temp variables for apply velocity step
const _applyVel_angularImpulseA = /* @__PURE__ */ vec3.create();
const _applyVel_angularImpulseB = /* @__PURE__ */ vec3.create();

/**
 * Apply an impulse to both bodies.
 *
 * Linear impulse: ΔP = λ
 * Angular impulse: ΔL = [r]× * λ = I^-1 * [r]× * λ (precomputed)
 */
function applyVelocityStep(part: PointConstraintPart, bodyA: RigidBody, bodyB: RigidBody, lambda: Vec3): boolean {
    // check for zero impulse
    if (vec3.squaredLength(lambda) === 0) {
        return false;
    }

    // body A: subtract impulse (constraint reaction)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const mp1 = bodyA.motionProperties;

        // linear: v -= λ * invMass
        vec3.scaleAndAdd(mp1.linearVelocity, mp1.linearVelocity, lambda, -mp1.invMass);

        // angular: w -= I^-1 * [r1]× * λ (3x3 only)
        const angularImpulse = _applyVel_angularImpulseA;
        mat4.multiply3x3Vec(angularImpulse, part.invI1_r1X, lambda);
        vec3.subtract(mp1.angularVelocity, mp1.angularVelocity, angularImpulse);
    }

    // body B: add impulse
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const mp2 = bodyB.motionProperties;

        // linear: v += λ * invMass
        vec3.scaleAndAdd(mp2.linearVelocity, mp2.linearVelocity, lambda, mp2.invMass);

        // angular: w += I^-1 * [r2]× * λ (3x3 only)
        const angularImpulse = _applyVel_angularImpulseB;
        mat4.multiply3x3Vec(angularImpulse, part.invI2_r2X, lambda);
        vec3.add(mp2.angularVelocity, mp2.angularVelocity, angularImpulse);
    }

    return true;
}

// temp variables for solve position
const _solvePos_separation = /* @__PURE__ */ vec3.create();
const _solvePos_lambda = /* @__PURE__ */ vec3.create();
const _solvePos_linearStep = /* @__PURE__ */ vec3.create();
const _solvePos_angularStep = /* @__PURE__ */ vec3.create();

/**
 * Solve the position constraint using Baumgarte stabilization.
 *
 * Computes separation: C = (x2 + r2) - (x1 + r1) where x is centerOfMassPosition
 * Solves for lambda: λ = -K^-1 * β * C
 * Applies position correction directly (not to velocities).
 *
 * Note: We don't accumulate velocities for stabilization. This uses the approach
 * described in 'Modeling and Solving Constraints' by Erin Catto (GDC 2007).
 * We combine an Euler velocity integrate + position integrate and discard velocity change.
 */
export function solvePositionConstraint(
    part: PointConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    baumgarte: number,
): boolean {
    if (!isActive(part)) {
        return false;
    }

    // compute separation vector: C = (x2 + r2) - (x1 + r1)
    // Using centerOfMassPosition (the physics-primary position)
    const separation = _solvePos_separation;
    vec3.add(separation, bodyB.centerOfMassPosition, part.r2);
    vec3.sub(separation, separation, bodyA.centerOfMassPosition);
    vec3.sub(separation, separation, part.r1);

    // check for zero separation (use small epsilon for floating point)
    if (vec3.squaredLength(separation) < 1e-12) {
        return false;
    }

    // compute lambda: λ = -K * β * C (3x3 only)
    const lambda = _solvePos_lambda;
    mat4.multiply3x3Vec(lambda, part.effectiveMass, separation);
    vec3.scale(lambda, lambda, -baumgarte);

    // apply position correction using body step functions
    // These handle DOF constraints and proper quaternion integration

    // body A: subtract correction
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const mp1 = bodyA.motionProperties;

        // linear: position -= λ * invMass
        const linearStep = _solvePos_linearStep;
        vec3.scale(linearStep, lambda, mp1.invMass);
        subPositionStep(bodyA, linearStep);

        // angular: rotation -= I^-1 * [r1]× * λ (3x3 only)
        const angularStep = _solvePos_angularStep;
        mat4.multiply3x3Vec(angularStep, part.invI1_r1X, lambda);
        subRotationStep(bodyA, angularStep);
    }

    // body B: add correction
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const mp2 = bodyB.motionProperties;

        // linear: position += λ * invMass
        const linearStep = _solvePos_linearStep;
        vec3.scale(linearStep, lambda, mp2.invMass);
        addPositionStep(bodyB, linearStep);

        // angular: rotation += I^-1 * [r2]× * λ (3x3 only)
        const angularStep = _solvePos_angularStep;
        mat4.multiply3x3Vec(angularStep, part.invI2_r2X, lambda);
        addRotationStep(bodyB, angularStep);
    }

    return true;
}
