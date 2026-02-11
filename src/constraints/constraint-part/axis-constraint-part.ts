import { type Mat4, mat4, type Vec3, vec3 } from 'mathcat';
import {
    addAngularVelocityStep,
    addLinearVelocityStep,
    subAngularVelocityStep,
    subLinearVelocityStep,
} from '../../body/motion-properties';
import { MotionType } from '../../body/motion-type';
import type { RigidBody } from '../../body/rigid-body';
import { addPositionStep, addRotationStep, subPositionStep, subRotationStep } from '../../body/rigid-body-step';
import {
    calculateSpringPropertiesWithBias,
    calculateSpringPropertiesWithFrequencyAndDamping,
    calculateSpringPropertiesWithSettings,
    calculateSpringPropertiesWithStiffnessAndDamping,
    createSpringPart,
    getSpringBias,
    isSpringActive,
    type SpringPart,
} from './spring-part';
import type { SpringSettings } from './spring-settings';

/**
 * Constraint part that constrains motion along 1 axis.
 *
 * This is the core building block for contact constraints:
 * - Normal constraint: prevents penetration
 * - Friction constraints: 2 tangential constraints (friction1, friction2)
 *
 * Also supports soft constraints (springs) for joints.
 *
 * Stores intermediate calculations for efficient solving.
 */
export type AxisConstraintPart = {
    // cached cross products (for jacobian computation)
    /** r1 × axis (cross product, cached) */
    r1PlusUxAxis: Vec3;

    /** r2 × axis (cross product, cached) */
    r2xAxis: Vec3;

    /** I1^-1 × (r1 × axis) - cached angular jacobian for body 1 */
    invI1_r1PlusUxAxis: Vec3;

    /** I2^-1 × (r2 × axis) - cached angular jacobian for body 2 */
    invI2_r2xAxis: Vec3;

    /** effective mass: 1 / (J × M^-1 × J^T) (adjusted for spring softness) */
    effectiveMass: number;

    /** accumulated impulse (warm started from previous frame) */
    totalLambda: number;

    /** spring part for soft constraints (contains bias and softness) */
    springPart: SpringPart;
};

/** create a new AxisConstraintPart with zero-initialized values */
export function create(): AxisConstraintPart {
    return {
        r1PlusUxAxis: vec3.create(),
        r2xAxis: vec3.create(),
        invI1_r1PlusUxAxis: vec3.create(),
        invI2_r2xAxis: vec3.create(),
        effectiveMass: 0,
        totalLambda: 0,
        springPart: createSpringPart(),
    };
}

/** reset an AxisConstraintPart to zero values */
export function resetAxisConstraintPart(part: AxisConstraintPart): void {
    vec3.set(part.r1PlusUxAxis, 0, 0, 0);
    vec3.set(part.r2xAxis, 0, 0, 0);
    vec3.set(part.invI1_r1PlusUxAxis, 0, 0, 0);
    vec3.set(part.invI2_r2xAxis, 0, 0, 0);
    part.effectiveMass = 0;
    part.totalLambda = 0;
    part.springPart.bias = 0;
    part.springPart.softness = 0;
}

const _acp_r1PlusUxAxis = /* @__PURE__ */ vec3.create();
const _acp_r2xAxis = /* @__PURE__ */ vec3.create();
const _acp_invI1_r1PlusUxAxis = /* @__PURE__ */ vec3.create();
const _acp_invI2_r2xAxis = /* @__PURE__ */ vec3.create();

const _acp_scaledInvInertiaA = /* @__PURE__ */ mat4.create();
const _acp_scaledInvInertiaB = /* @__PURE__ */ mat4.create();

/**
 * Helper to calculate inverse effective mass (cached jacobian terms).
 * Internal function used by all calculateConstraintProperties variants.
 */
function calculateInverseEffectiveMass(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    invInertiaA: Mat4,
    invInertiaB: Mat4,
    r1PlusU: Vec3,
    r2: Vec3,
    axis: Vec3,
): number {
    // calculate cross products: r × axis
    // (used in jacobian: J = [-axis, -(r1 × axis), axis, (r2 × axis)])
    let invEffectiveMass = 0;

    if (bodyA.motionType === MotionType.DYNAMIC) {
        // r1 × axis
        vec3.cross(_acp_r1PlusUxAxis, r1PlusU, axis);
        vec3.copy(part.r1PlusUxAxis, _acp_r1PlusUxAxis);

        // I1^-1 × (r1 × axis) (3x3 only)
        mat4.multiply3x3Vec(_acp_invI1_r1PlusUxAxis, invInertiaA, _acp_r1PlusUxAxis);
        vec3.copy(part.invI1_r1PlusUxAxis, _acp_invI1_r1PlusUxAxis);

        // add to inverse effective mass: invMass + (r × axis) · (I^-1 × (r × axis))
        invEffectiveMass += invMassA + vec3.dot(_acp_invI1_r1PlusUxAxis, _acp_r1PlusUxAxis);
    }

    if (bodyB.motionType === MotionType.DYNAMIC) {
        // r2 × axis
        vec3.cross(_acp_r2xAxis, r2, axis);
        vec3.copy(part.r2xAxis, _acp_r2xAxis);

        // I2^-1 × (r2 × axis) (3x3 only)
        mat4.multiply3x3Vec(_acp_invI2_r2xAxis, invInertiaB, _acp_r2xAxis);
        vec3.copy(part.invI2_r2xAxis, _acp_invI2_r2xAxis);

        // Add to inverse effective mass
        const rotContribB = vec3.dot(_acp_invI2_r2xAxis, _acp_r2xAxis);
        invEffectiveMass += invMassB + rotContribB;
    }

    return invEffectiveMass;
}

/**
 * Calculate constraint properties (effective mass and cached jacobian terms).
 * Hard constraint version (no spring).
 * Call this during constraint initialization.
 *
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A (0 if static)
 * @param invMassB inverse mass of body B (0 if static)
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A: (contactPoint - centerOfMassA)
 * @param r2 moment arm for body B: (contactPoint - centerOfMassB)
 * @param axis constraint axis (normalized, e.g., contact normal or friction tangent)
 * @param bias velocity bias (for restitution or speculative contacts)
 */
export function calculateConstraintProperties(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    invInertiaA: Mat4,
    invInertiaB: Mat4,
    r1PlusU: Vec3,
    r2: Vec3,
    axis: Vec3,
    bias: number,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(
        part,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        invInertiaA,
        invInertiaB,
        r1PlusU,
        r2,
        axis,
    );

    if (invEffectiveMass === 0) {
        deactivate(part);
    } else {
        part.effectiveMass = 1 / invEffectiveMass;
        calculateSpringPropertiesWithBias(part.springPart, bias);
    }
}

/**
 * Calculate constraint properties with mass override (effective mass and cached jacobian terms).
 * Hard constraint version (no spring), allows custom inverse mass and inertia scaling.
 * Call this during constraint initialization when you need to override mass properties.
 *
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A (only used when body A is dynamic)
 * @param invMassB inverse mass override for body B (only used when body B is dynamic)
 * @param invInertiaScaleA scale factor for the inverse inertia of body A
 * @param invInertiaScaleB scale factor for the inverse inertia of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A: (contactPoint - centerOfMassA)
 * @param r2 moment arm for body B: (contactPoint - centerOfMassB)
 * @param axis constraint axis (normalized, e.g., contact normal or friction tangent)
 * @param bias velocity bias (for restitution or speculative contacts)
 */
export function calculateConstraintPropertiesWithMassOverride(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    invInertiaScaleA: number,
    invInertiaScaleB: number,
    invInertiaA: Mat4,
    invInertiaB: Mat4,
    r1PlusU: Vec3,
    r2: Vec3,
    axis: Vec3,
    bias: number,
): void {
    // scale inverse inertia matrices only for dynamic bodies
    // non-dynamic bodies don't contribute to inverse effective mass, so their inertia is never read
    if (bodyA.motionType === MotionType.DYNAMIC) {
        mat4.multiplyScalar(_acp_scaledInvInertiaA, invInertiaA, invInertiaScaleA);
    }
    if (bodyB.motionType === MotionType.DYNAMIC) {
        mat4.multiplyScalar(_acp_scaledInvInertiaB, invInertiaB, invInertiaScaleB);
    }

    const invEffectiveMass = calculateInverseEffectiveMass(
        part,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        _acp_scaledInvInertiaA,
        _acp_scaledInvInertiaB,
        r1PlusU,
        r2,
        axis,
    );

    if (invEffectiveMass === 0) {
        deactivate(part);
    } else {
        part.effectiveMass = 1 / invEffectiveMass;
        calculateSpringPropertiesWithBias(part.springPart, bias);
    }
}

/**
 * Calculate constraint properties with frequency and damping (soft constraint).
 *
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A
 * @param r2 moment arm for body B
 * @param axis constraint axis (normalized)
 * @param bias velocity bias
 * @param C value of the constraint equation (C)
 * @param frequency oscillation frequency (Hz)
 * @param damping damping factor (0 = no damping, 1 = critical damping)
 */
export function calculateConstraintPropertiesWithFrequencyAndDamping(
    part: AxisConstraintPart,
    deltaTime: number,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    invInertiaA: Mat4,
    invInertiaB: Mat4,
    r1PlusU: Vec3,
    r2: Vec3,
    axis: Vec3,
    bias: number,
    C: number,
    frequency: number,
    damping: number,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(
        part,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        invInertiaA,
        invInertiaB,
        r1PlusU,
        r2,
        axis,
    );

    if (invEffectiveMass === 0) {
        deactivate(part);
    } else {
        part.effectiveMass = calculateSpringPropertiesWithFrequencyAndDamping(
            part.springPart,
            deltaTime,
            invEffectiveMass,
            bias,
            C,
            frequency,
            damping,
        );
    }
}

/**
 * Calculate constraint properties with stiffness and damping (soft constraint).
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A
 * @param r2 moment arm for body B
 * @param axis constraint axis (normalized)
 * @param bias velocity bias
 * @param C value of the constraint equation (C)
 * @param stiffness spring stiffness k
 * @param damping spring damping coefficient c
 */
export function calculateConstraintPropertiesWithStiffnessAndDamping(
    part: AxisConstraintPart,
    deltaTime: number,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    invInertiaA: Mat4,
    invInertiaB: Mat4,
    r1PlusU: Vec3,
    r2: Vec3,
    axis: Vec3,
    bias: number,
    C: number,
    stiffness: number,
    damping: number,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(
        part,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        invInertiaA,
        invInertiaB,
        r1PlusU,
        r2,
        axis,
    );

    if (invEffectiveMass === 0) {
        deactivate(part);
    } else {
        part.effectiveMass = calculateSpringPropertiesWithStiffnessAndDamping(
            part.springPart,
            deltaTime,
            invEffectiveMass,
            bias,
            C,
            stiffness,
            damping,
        );
    }
}

/**
 * Calculate constraint properties using SpringSettings.
 * Selects the appropriate calculation method based on the spring mode.
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param invInertiaA inverse inertia of body A (world space)
 * @param invInertiaB inverse inertia of body B (world space)
 * @param r1PlusU moment arm for body A
 * @param r2 moment arm for body B
 * @param axis constraint axis (normalized)
 * @param bias velocity bias
 * @param C value of the constraint equation (C)
 * @param settings spring settings (mode, frequency/stiffness, damping)
 */
export function calculateConstraintPropertiesWithSettings(
    part: AxisConstraintPart,
    deltaTime: number,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    invInertiaA: Mat4,
    invInertiaB: Mat4,
    r1PlusU: Vec3,
    r2: Vec3,
    axis: Vec3,
    bias: number,
    C: number,
    settings: SpringSettings,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(
        part,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        invInertiaA,
        invInertiaB,
        r1PlusU,
        r2,
        axis,
    );

    if (invEffectiveMass === 0) {
        deactivate(part);
    } else {
        part.effectiveMass = calculateSpringPropertiesWithSettings(
            part.springPart,
            deltaTime,
            invEffectiveMass,
            bias,
            C,
            settings,
        );
    }
}

/** Check if constraint is active (has non-zero effective mass) */
export function isActive(part: AxisConstraintPart): boolean {
    return part.effectiveMass !== 0;
}

/** Deactivate the constraint (zero out effective mass and lambda) */
export function deactivate(part: AxisConstraintPart): void {
    part.effectiveMass = 0;
    part.totalLambda = 0;
}

/**
 * Override total lagrange multiplier.
 * Can be used to set the initial value for warm starting.
 *
 * @param part the constraint part
 * @param lambda new total lambda value
 */
export function setTotalLambda(part: AxisConstraintPart, lambda: number): void {
    part.totalLambda = lambda;
}

/**
 * Get the current total lagrange multiplier.
 *
 * @param part the constraint part
 * @returns Current total lambda value
 */
export function getTotalLambdaValue(part: AxisConstraintPart): number {
    return part.totalLambda;
}

const _acp_ws_impulse = /* @__PURE__ */ vec3.create();

/**
 * Apply warm start impulse from previous frame.
 * Call this once before velocity iterations.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param axis constraint axis (same as used in calculateConstraintProperties)
 * @param warmStartRatio scale factor for warm start (dt_new / dt_old)
 */
export function warmStart(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    axis: Vec3,
    warmStartRatio: number,
): void {
    // scale previous frame's lambda
    part.totalLambda *= warmStartRatio;

    if (part.totalLambda === 0) return;

    // body a: subtract impulse (opposite direction)
    if (bodyA.motionType === MotionType.DYNAMIC) {
        const mpA = bodyA.motionProperties;
        // linear: v -= (axis × totalLambda) × invMassA
        vec3.scale(_acp_ws_impulse, axis, part.totalLambda * invMassA);
        subLinearVelocityStep(mpA, _acp_ws_impulse);

        // angular: ω -= lambda × (I^-1 × (r × axis))
        vec3.scale(_acp_ws_impulse, part.invI1_r1PlusUxAxis, part.totalLambda);
        subAngularVelocityStep(mpA, _acp_ws_impulse);
    }

    // body b: add impulse
    if (bodyB.motionType === MotionType.DYNAMIC) {
        const mpB = bodyB.motionProperties;
        // linear: v += (axis × totalLambda) × invMassB
        vec3.scale(_acp_ws_impulse, axis, part.totalLambda * invMassB);
        addLinearVelocityStep(mpB, _acp_ws_impulse);

        // angular: ω += lambda × (I^-1 × (r × axis))
        vec3.scale(_acp_ws_impulse, part.invI2_r2xAxis, part.totalLambda);
        addAngularVelocityStep(mpB, _acp_ws_impulse);
    }
}

const _acp_sv_impulse = /* @__PURE__ */ vec3.create();
const _acp_gtl_velDiff = /* @__PURE__ */ vec3.create();

/**
 * Solve velocity constraint (one iteration).
 * Standard version - uses body's actual inverse mass.
 * Call this during velocity solver iterations.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param axis constraint axis
 * @param minLambda minimum lambda (typically -Infinity for friction, 0 for normal)
 * @param maxLambda maximum lambda (typically +Infinity for normal, friction_coeff × normalLambda for friction)
 * @returns True if impulse was applied
 */
export function solveVelocityConstraint(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    axis: Vec3,
    minLambda: number,
    maxLambda: number,
): boolean {
    const mpA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties : null;
    const mpB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties : null;
    const invMassA = mpA ? mpA.invMass : 0;
    const invMassB = mpB ? mpB.invMass : 0;

    return solveVelocityConstraintWithMassOverride(part, bodyA, bodyB, invMassA, invMassB, axis, minLambda, maxLambda);
}

/**
 * Calculate what the total lambda would be (without applying impulse).
 * Part 1 of two-step solve process.
 *
 * note: caller must check isActive() before calling this function
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param axis constraint axis
 * @returns new total lambda (unclamped)
 */
export function getTotalLambda(part: AxisConstraintPart, bodyA: RigidBody, bodyB: RigidBody, axis: Vec3): number {
    // get motion properties for non-static bodies (both dynamic and kinematic contribute velocity)
    const mpA = bodyA.motionType !== MotionType.STATIC ? bodyA.motionProperties : null;
    const mpB = bodyB.motionType !== MotionType.STATIC ? bodyB.motionProperties : null;

    // calculate jacobian multiplied by linear velocity
    // optimization: compute velocity difference first, then single dot product (matches JoltPhysics)
    let jv: number;
    if (mpA && mpB) {
        vec3.subtract(_acp_gtl_velDiff, mpA.linearVelocity, mpB.linearVelocity);
        jv = vec3.dot(axis, _acp_gtl_velDiff);
    } else if (mpA) {
        jv = vec3.dot(axis, mpA.linearVelocity);
    } else if (mpB) {
        jv = -vec3.dot(axis, mpB.linearVelocity);
    } else {
        jv = 0;
    }

    // calculate jacobian multiplied by angular velocity
    if (mpA) {
        jv += vec3.dot(part.r1PlusUxAxis, mpA.angularVelocity);
    }
    if (mpB) {
        jv -= vec3.dot(part.r2xAxis, mpB.angularVelocity);
    }

    // calculate lambda
    const lambda = part.effectiveMass * (jv - getSpringBias(part.springPart, part.totalLambda));

    return part.totalLambda + lambda;
}

/**
 * Apply a total lambda value (calculates delta and applies impulse).
 * Part 2 of two-step solve process.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A
 * @param invMassB inverse mass override for body B
 * @param axis constraint axis
 * @param totalLambda new total lambda to apply
 * @returns true if impulse was applied
 */
export function applyLambda(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    axis: Vec3,
    totalLambda: number,
): boolean {
    const deltaLambda = totalLambda - part.totalLambda;
    part.totalLambda = totalLambda;

    if (deltaLambda === 0) return false;

    const mpA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties : null;
    const mpB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties : null;

    // body A: subtract impulse (mpA non-null already implies DYNAMIC)
    if (mpA) {
        // linear: v -= (axis × deltaLambda) × invMassA
        vec3.scale(_acp_sv_impulse, axis, deltaLambda * invMassA);
        subLinearVelocityStep(mpA, _acp_sv_impulse);
        // angular: ω -= deltaLambda × (I^-1 × (r × axis))
        vec3.scale(_acp_sv_impulse, part.invI1_r1PlusUxAxis, deltaLambda);
        subAngularVelocityStep(mpA, _acp_sv_impulse);
    }

    // body B: add impulse (mpB non-null already implies DYNAMIC)
    if (mpB) {
        // linear: v += (axis × deltaLambda) × invMassB
        vec3.scale(_acp_sv_impulse, axis, deltaLambda * invMassB);
        addLinearVelocityStep(mpB, _acp_sv_impulse);
        // angular: ω += deltaLambda × (I^-1 × (r × axis))
        vec3.scale(_acp_sv_impulse, part.invI2_r2xAxis, deltaLambda);
        addAngularVelocityStep(mpB, _acp_sv_impulse);
    }

    return true;
}

/**
 * Solve velocity constraint (one iteration) with mass override.
 * Combines getTotalLambda + clamp + applyLambda in one call.
 * Mass override version - allows custom inverse mass values (for soft contacts, etc.).
 * Call this during velocity solver iterations.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A (only used when body A is dynamic)
 * @param invMassB inverse mass override for body B (only used when body B is dynamic)
 * @param axis constraint axis
 * @param minLambda minimum lambda (typically -Infinity for friction, 0 for normal)
 * @param maxLambda maximum lambda (typically +Infinity for normal, friction_coeff × normalLambda for friction)
 * @returns true if impulse was applied
 */
export function solveVelocityConstraintWithMassOverride(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    axis: Vec3,
    minLambda: number,
    maxLambda: number,
): boolean {
    // use two-step process: get total lambda, clamp, apply
    const totalLambda = getTotalLambda(part, bodyA, bodyB, axis);
    const clampedLambda = Math.max(minLambda, Math.min(maxLambda, totalLambda));
    return applyLambda(part, bodyA, bodyB, invMassA, invMassB, axis, clampedLambda);
}

const _acp_sp_impulse = /* @__PURE__ */ vec3.create();
const _acp_sp_angularStep = /* @__PURE__ */ vec3.create();

/**
 * Solve position constraint (Baumgarte stabilization).
 * Standard version - uses body's actual inverse mass.
 * Call this during position solver iterations.
 *
 * Note: Position constraints are only applied for hard constraints, not soft springs.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param axis constraint axis
 * @param C constraint error (penetration depth, or 0 if separated)
 * @param baumgarte baumgarte stabilization factor (typically 0.2)
 * @returns true if position correction was applied
 */
export function solvePositionConstraint(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    axis: Vec3,
    C: number,
    baumgarte: number,
): boolean {
    const invMassA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties.invMass : 0;
    const invMassB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties.invMass : 0;

    return solvePositionConstraintWithMassOverride(part, bodyA, bodyB, invMassA, invMassB, axis, C, baumgarte);
}

/**
 * Solve position constraint (Baumgarte stabilization) with mass override.
 * Mass override version - allows custom inverse mass values (for soft contacts, etc.).
 * Call this during position solver iterations.
 *
 * Note: Position constraints are only applied for hard constraints, not soft springs.
 *
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param invMassA inverse mass override for body A (only used when body A is dynamic)
 * @param invMassB inverse mass override for body B (only used when body B is dynamic)
 * @param axis constraint axis
 * @param C constraint error (penetration depth, or 0 if separated)
 * @param baumgarte baumgarte stabilization factor (typically 0.2)
 * @returns true if position correction was applied
 */
export function solvePositionConstraintWithMassOverride(
    part: AxisConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    invMassA: number,
    invMassB: number,
    axis: Vec3,
    C: number,
    baumgarte: number,
): boolean {
    // Only apply position constraint when the constraint is hard
    // Soft constraints (springs) use velocity bias instead
    if (C === 0 || !isActive(part) || isSpringActive(part.springPart)) {
        return false;
    }

    // Calculate lambda: -K^-1 × β × C
    // We should divide by deltaTime, but we should multiply by deltaTime in the Euler step below so they're cancelled out
    const lambda = -part.effectiveMass * baumgarte * C;

    // Directly integrate velocity change for one time step
    //
    // Euler velocity integration:
    // dv = M^-1 P
    //
    // Impulse:
    // P = J^T lambda
    //
    // Euler position integration:
    // x' = x + dv * dt
    //
    // Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
    // Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
    // stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
    // integrate + a position integrate and then discard the velocity change.

    if (bodyA.motionType === MotionType.DYNAMIC) {
        // linear position correction: x -= (lambda × invMass) × axis
        vec3.scale(_acp_sp_impulse, axis, lambda * invMassA);
        subPositionStep(bodyA, _acp_sp_impulse);

        // angular position correction: q' = q - lambda × (I^-1 × (r × axis))
        vec3.scale(_acp_sp_angularStep, part.invI1_r1PlusUxAxis, lambda);
        subRotationStep(bodyA, _acp_sp_angularStep);
    }

    if (bodyB.motionType === MotionType.DYNAMIC) {
        // linear position correction: x += (lambda × invMass) × axis
        vec3.scale(_acp_sp_impulse, axis, lambda * invMassB);
        addPositionStep(bodyB, _acp_sp_impulse);

        // angular position correction: q' = q + lambda × (I^-1 × (r × axis))
        vec3.scale(_acp_sp_angularStep, part.invI2_r2xAxis, lambda);
        addRotationStep(bodyB, _acp_sp_angularStep);
    }

    return true;
}
