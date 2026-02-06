import type { Vec3 } from 'mathcat';
import { vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body';
import { addRotationStep, subRotationStep } from '../../body/rigid-body-step';
import { MotionType } from '../../body/motion-type';
import {
    addAngularVelocityStep,
    subAngularVelocityStep,
    multiplyWorldSpaceInverseInertiaByVector,
} from '../../body/motion-properties';
import {
    createSpringPart,
    calculateSpringPropertiesWithBias,
    calculateSpringPropertiesWithFrequencyAndDamping,
    calculateSpringPropertiesWithStiffnessAndDamping,
    calculateSpringPropertiesWithSettings,
    isSpringActive,
    getSpringBias,
    type SpringPart,
} from './spring-part';
import type { SpringSettings } from './spring-settings';

/**
 * Constrains rotation along 1 axis.
 *
 * Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.4.5
 *
 * Constraint equation (eq 108):
 * C = θ(t) - θ_target
 *
 * Jacobian (eq 109):
 * J = [0, -a^T, 0, a^T]
 *
 * where:
 * - a = axis around which rotation is constrained (normalized, world space)
 *
 * This is a 1-DOF angular constraint used for:
 * - Hinge limits (min/max angle)
 * - Hinge motors (driving to target angle or velocity)
 * - Angular friction
 */
export type AngleConstraintPart = {
    /** I1^-1 * axis (cached for velocity/position integration) */
    invI1_Axis: Vec3;

    /** I2^-1 * axis (cached for velocity/position integration) */
    invI2_Axis: Vec3;

    /** Effective mass: 1 / (J M^-1 J^T) */
    effectiveMass: number;

    /** Spring part for soft constraints */
    springPart: SpringPart;

    /** Accumulated impulse (for warm starting) */
    totalLambda: number;
};

/** create a new AngleConstraintPart with zero-initialized values */
export function create(): AngleConstraintPart {
    return {
        invI1_Axis: vec3.create(),
        invI2_Axis: vec3.create(),
        effectiveMass: 0,
        springPart: createSpringPart(),
        totalLambda: 0,
    };
}

/** deactivate this constraint part (zero out effective mass and lambda) */
export function deactivate(part: AngleConstraintPart): void {
    part.effectiveMass = 0;
    part.totalLambda = 0;
}

/** check if constraint is active (has non-zero effective mass) */
export function isActive(part: AngleConstraintPart): boolean {
    return part.effectiveMass !== 0;
}

const _calc_invI1_Axis = vec3.create();
const _calc_invI2_Axis = vec3.create();

/** helper to calculate inverse effective mass */
function calculateInverseEffectiveMass(
    part: AngleConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    worldSpaceAxis: Vec3,
): number {
    // calculate I^-1 * axis for both bodies
    if (bodyA.motionType === MotionType.DYNAMIC) {
        multiplyWorldSpaceInverseInertiaByVector(
            _calc_invI1_Axis,
            bodyA.motionProperties,
            bodyA.quaternion,
            worldSpaceAxis,
        );
        vec3.copy(part.invI1_Axis, _calc_invI1_Axis);
    } else {
        vec3.set(part.invI1_Axis, 0, 0, 0);
        vec3.set(_calc_invI1_Axis, 0, 0, 0);
    }

    if (bodyB.motionType === MotionType.DYNAMIC) {
        multiplyWorldSpaceInverseInertiaByVector(
            _calc_invI2_Axis,
            bodyB.motionProperties,
            bodyB.quaternion,
            worldSpaceAxis,
        );
        vec3.copy(part.invI2_Axis, _calc_invI2_Axis);
    } else {
        vec3.set(part.invI2_Axis, 0, 0, 0);
        vec3.set(_calc_invI2_Axis, 0, 0, 0);
    }

    // calculate inverse effective mass: K = J M^-1 J^T = axis · (I1^-1 + I2^-1) · axis
    // since J = [0, -a, 0, a], we get: K = a · I1^-1 · a + a · I2^-1 · a
    vec3.add(_calc_invI1_Axis, _calc_invI1_Axis, _calc_invI2_Axis);
    return vec3.dot(worldSpaceAxis, _calc_invI1_Axis);
}

/**
 * Calculate constraint properties (hard constraint, no spring).
 * @param part the constraint part to initialize
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation along which the constraint acts (normalized)
 * @param bias bias term (b) for the constraint impulse: lambda = J v + b
 */
export function calculateConstraintProperties(
    part: AngleConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    worldSpaceAxis: Vec3,
    bias = 0,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(part, bodyA, bodyB, worldSpaceAxis);

    if (invEffectiveMass === 0) {
        deactivate(part);
    } else {
        part.effectiveMass = 1 / invEffectiveMass;
        calculateSpringPropertiesWithBias(part.springPart, bias);
    }
}

/**
 * Calculate constraint properties with frequency and damping (soft constraint).
 * @param part the constraint part to initialize
 * @param deltaTime time step
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param bias bias term
 * @param C value of the constraint equation (angle error)
 * @param frequency oscillation frequency (Hz)
 * @param damping damping factor (0 = no damping, 1 = critical damping)
 */
export function calculateConstraintPropertiesWithFrequencyAndDamping(
    part: AngleConstraintPart,
    deltaTime: number,
    bodyA: RigidBody,
    bodyB: RigidBody,
    worldSpaceAxis: Vec3,
    bias: number,
    C: number,
    frequency: number,
    damping: number,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(part, bodyA, bodyB, worldSpaceAxis);

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
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param bias bias term
 * @param C value of the constraint equation (angle error)
 * @param stiffness spring stiffness k
 * @param damping spring damping coefficient c
 */
export function calculateConstraintPropertiesWithStiffnessAndDamping(
    part: AngleConstraintPart,
    deltaTime: number,
    bodyA: RigidBody,
    bodyB: RigidBody,
    worldSpaceAxis: Vec3,
    bias: number,
    C: number,
    stiffness: number,
    damping: number,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(part, bodyA, bodyB, worldSpaceAxis);

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
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param bias bias term
 * @param C value of the constraint equation (angle error)
 * @param settings spring settings (mode, frequency/stiffness, damping)
 */
export function calculateConstraintPropertiesWithSettings(
    part: AngleConstraintPart,
    deltaTime: number,
    bodyA: RigidBody,
    bodyB: RigidBody,
    worldSpaceAxis: Vec3,
    bias: number,
    C: number,
    settings: SpringSettings,
): void {
    const invEffectiveMass = calculateInverseEffectiveMass(part, bodyA, bodyB, worldSpaceAxis);

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

const _ws_angularDelta = vec3.create();

/** Apply warm start impulse from previous frame */
export function warmStart(
    part: AngleConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    warmStartImpulseRatio: number,
): void {
    part.totalLambda *= warmStartImpulseRatio;
    applyVelocityStep(part, bodyA, bodyB, part.totalLambda);
}

/** Internal helper to apply velocity step */
function applyVelocityStep(part: AngleConstraintPart, bodyA: RigidBody, bodyB: RigidBody, lambda: number): boolean {
    if (lambda === 0) {
        return false;
    }

    // apply impulse: P = J^T * lambda
    // body a: ω -= lambda * I1^-1 * axis
    if (bodyA.motionType === MotionType.DYNAMIC) {
        vec3.scale(_ws_angularDelta, part.invI1_Axis, lambda);
        subAngularVelocityStep(bodyA.motionProperties, _ws_angularDelta);
    }

    // body b: ω += lambda * I2^-1 * axis
    if (bodyB.motionType === MotionType.DYNAMIC) {
        vec3.scale(_ws_angularDelta, part.invI2_Axis, lambda);
        addAngularVelocityStep(bodyB.motionProperties, _ws_angularDelta);
    }

    return true;
}

/**
 * Solve the velocity constraint.
 * Enforces d/dt C(...) = 0 where C is the constraint equation.
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param worldSpaceAxis axis of rotation (normalized)
 * @param minLambda minimum angular impulse (N m s)
 * @param maxLambda maximum angular impulse (N m s)
 * @returns true if impulse was applied
 */
export function solveVelocityConstraint(
    part: AngleConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    worldSpaceAxis: Vec3,
    minLambda: number,
    maxLambda: number,
): boolean {
    if (!isActive(part)) {
        return false;
    }

    // get angular velocities
    const mpA = bodyA.motionType !== MotionType.STATIC ? bodyA.motionProperties : null;
    const mpB = bodyB.motionType !== MotionType.STATIC ? bodyB.motionProperties : null;

    // calculate J * v = axis · (ω1 - ω2)
    let jv = 0;
    if (mpA) {
        jv += vec3.dot(worldSpaceAxis, mpA.angularVelocity);
    }
    if (mpB) {
        jv -= vec3.dot(worldSpaceAxis, mpB.angularVelocity);
    }

    // calculate lambda: lambda = -K^-1 * (J v + b)
    const lambda = part.effectiveMass * (jv - getSpringBias(part.springPart, part.totalLambda));

    // clamp impulse
    const newLambda = Math.max(minLambda, Math.min(maxLambda, part.totalLambda + lambda));
    const deltaLambda = newLambda - part.totalLambda;
    part.totalLambda = newLambda;

    return applyVelocityStep(part, bodyA, bodyB, deltaLambda);
}

const _pos_angularDelta = vec3.create();

/**
 * Solve the position constraint (Baumgarte stabilization).
 * Enforces C(...) = 0.
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param C constraint error (angle error in radians)
 * @param baumgarte Baumgarte stabilization factor
 * @returns true if correction was applied
 */
export function solvePositionConstraint(
    part: AngleConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    C: number,
    baumgarte: number,
): boolean {
    // only apply position constraint when the constraint is hard
    // soft constraints (springs) use velocity bias instead
    if (C === 0 || !isActive(part) || isSpringActive(part.springPart)) {
        return false;
    }

    // calculate lambda: lambda = -K^-1 * beta * C
    // we don't divide by dt because we multiply by dt in position integration (cancelled out)
    const lambda = -part.effectiveMass * baumgarte * C;

    // apply position correction
    // body a: Δq -= lambda * I1^-1 * axis
    if (bodyA.motionType === MotionType.DYNAMIC) {
        vec3.scale(_pos_angularDelta, part.invI1_Axis, lambda);
        subRotationStep(bodyA, _pos_angularDelta);
    }

    // body b: Δq += lambda * I2^-1 * axis
    if (bodyB.motionType === MotionType.DYNAMIC) {
        vec3.scale(_pos_angularDelta, part.invI2_Axis, lambda);
        addRotationStep(bodyB, _pos_angularDelta);
    }

    return true;
}

/** Get total accumulated lambda (impulse) */
export function getTotalLambda(part: AngleConstraintPart): number {
    return part.totalLambda;
}

/** Set total lambda (for warm starting with specific value) */
export function setTotalLambda(part: AngleConstraintPart, lambda: number): void {
    part.totalLambda = lambda;
}
