import type { Quat, Vec3 } from 'mathcat';
import { mat3, mat4, quat, vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies';
import { type BodyId, getBodyIdIndex } from '../body/body-id';
import type { RigidBody } from '../body/rigid-body';
import type { World } from '../world';
import { type ConstraintBase, ConstraintSpace, makeConstraintBase, removeConstraintIdFromBody } from './constraint-base';
import {
    type ConstraintId,
    ConstraintType,
    getConstraintIdIndex,
    getConstraintIdSequence,
    INVALID_CONSTRAINT_ID,
    SEQUENCE_MASK,
    serConstraintId,
} from './constraint-id';
import type { AngleConstraintPart } from './constraint-part/angle-constraint-part';
import * as angleConstraintPart from './constraint-part/angle-constraint-part';
import type { HingeRotationConstraintPart } from './constraint-part/hinge-rotation-constraint-part';
import * as hingeRotationConstraintPart from './constraint-part/hinge-rotation-constraint-part';
import type { MotorSettings } from './constraint-part/motor-settings';
import * as motorSettings from './constraint-part/motor-settings';
import { MotorState } from './constraint-part/motor-settings';
import type { PointConstraintPart } from './constraint-part/point-constraint-part';
import * as pointConstraintPart from './constraint-part/point-constraint-part';
import type { SpringSettings } from './constraint-part/spring-settings';
import * as springSettings from './constraint-part/spring-settings';
import { type ConstraintPool, defineUserConstraint, ensurePool } from './constraints';

/**
 * Hinge constraint removes 5 DOF (3 translation + 2 rotation).
 * Allows rotation only around the hinge axis, like a door hinge or wheel.
 */
export type HingeConstraint = ConstraintBase & {
    // local space configuration (stored relative to body COM)
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;
    /** hinge axis on body 1 in local space (normalized) */
    localSpaceHingeAxis1: Vec3;
    /** hinge axis on body 2 in local space (normalized) */
    localSpaceHingeAxis2: Vec3;
    /** normal axis perpendicular to hinge on body 1 (for angle reference) */
    localSpaceNormalAxis1: Vec3;
    /** normal axis perpendicular to hinge on body 2 (for angle reference) */
    localSpaceNormalAxis2: Vec3;

    // angle tracking
    /** inverse of initial relative orientation, for calculating current angle */
    invInitialOrientation: Quat;
    /** current hinge angle (radians) - cached during setup */
    theta: number;
    /** world space hinge axis (cached during setup) */
    worldSpaceHingeAxis1: Vec3;

    // limits
    /** whether angle limits are enabled */
    hasLimits: boolean;
    /** minimum angle limit (radians) */
    limitsMin: number;
    /** maximum angle limit (radians) */
    limitsMax: number;
    /** spring settings for soft limits */
    limitsSpringSettings: SpringSettings;

    // motor ---
    /** motor state */
    motorState: MotorState;
    /** target angular velocity (rad/s) for velocity mode */
    targetAngularVelocity: number;
    /** target angle (radians) for position mode */
    targetAngle: number;
    /** maximum friction torque when motor is off */
    maxFrictionTorque: number;
    /** motor settings (spring + torque limits) */
    motorSettings: MotorSettings;

    // constraint parts
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** hinge rotation constraint (2 DOF) - keeps axes aligned */
    rotationConstraintPart: HingeRotationConstraintPart;
    /** angle limit constraint (1 DOF) */
    rotationLimitsConstraintPart: AngleConstraintPart;
    /** motor constraint (1 DOF) */
    motorConstraintPart: AngleConstraintPart;
};

/** creates default hinge constraint */
function makeHingeConstraint(): HingeConstraint {
    return {
        ...makeConstraintBase(),
        // local space
        localSpacePosition1: vec3.create(),
        localSpacePosition2: vec3.create(),
        localSpaceHingeAxis1: vec3.fromValues(0, 1, 0),
        localSpaceHingeAxis2: vec3.fromValues(0, 1, 0),
        localSpaceNormalAxis1: vec3.fromValues(1, 0, 0),
        localSpaceNormalAxis2: vec3.fromValues(1, 0, 0),
        // angle tracking
        invInitialOrientation: quat.create(),
        theta: 0,
        worldSpaceHingeAxis1: vec3.create(),
        // limits
        hasLimits: false,
        limitsMin: -Math.PI,
        limitsMax: Math.PI,
        limitsSpringSettings: springSettings.create(),
        // motor
        motorState: MotorState.OFF,
        targetAngularVelocity: 0,
        targetAngle: 0,
        maxFrictionTorque: 0,
        motorSettings: motorSettings.create(),
        // constraint parts
        pointConstraintPart: pointConstraintPart.create(),
        rotationConstraintPart: hingeRotationConstraintPart.create(),
        rotationLimitsConstraintPart: angleConstraintPart.create(),
        motorConstraintPart: angleConstraintPart.create(),
    };
}

/** settings for creating a hinge constraint */
export type HingeConstraintSettings = {
    /** body id for body a */
    bodyIdA: BodyId;
    /** body id for body b */
    bodyIdB: BodyId;
    /** pivot point on body a */
    pointA: Vec3;
    /** pivot point on body b */
    pointB: Vec3;
    /** hinge axis on body a (will be normalized) */
    hingeAxisA: Vec3;
    /** hinge axis on body b (will be normalized) */
    hingeAxisB: Vec3;
    /** normal axis perpendicular to hinge on body a (for angle reference) */
    normalAxisA: Vec3;
    /** normal axis perpendicular to hinge on body b (for angle reference) */
    normalAxisB: Vec3;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;

    // limits
    /** minimum angle limit in radians (default: -PI) */
    limitsMin?: number;
    /** maximum angle limit in radians (default: PI) */
    limitsMax?: number;
    /** spring settings for soft limits */
    limitsSpringSettings?: SpringSettings;

    // motor
    /** maximum friction torque when motor is off */
    maxFrictionTorque?: number;
    /** motor settings (spring + torque limits) */
    motorSettings?: MotorSettings;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

/** Calculate the inverse initial orientation for angle tracking */
function getInvInitialOrientationXZ(normalAxis1: Vec3, hingeAxis1: Vec3, normalAxis2: Vec3, hingeAxis2: Vec3): Quat {
    // if axes are identical, return identity
    const eps = 1e-6;
    if (
        Math.abs(normalAxis1[0] - normalAxis2[0]) < eps &&
        Math.abs(normalAxis1[1] - normalAxis2[1]) < eps &&
        Math.abs(normalAxis1[2] - normalAxis2[2]) < eps &&
        Math.abs(hingeAxis1[0] - hingeAxis2[0]) < eps &&
        Math.abs(hingeAxis1[1] - hingeAxis2[1]) < eps &&
        Math.abs(hingeAxis1[2] - hingeAxis2[2]) < eps
    ) {
        return quat.create(); // identity
    }

    // build 3x3 rotation matrices from axes
    const y1 = vec3.create();
    vec3.cross(y1, hingeAxis1, normalAxis1);

    const y2 = vec3.create();
    vec3.cross(y2, hingeAxis2, normalAxis2);

    // create rotation matrices (column-major in mathcat)
    const mat1 = mat3.fromValues(
        normalAxis1[0],
        normalAxis1[1],
        normalAxis1[2],
        y1[0],
        y1[1],
        y1[2],
        hingeAxis1[0],
        hingeAxis1[1],
        hingeAxis1[2],
    );

    const mat2 = mat3.fromValues(
        normalAxis2[0],
        normalAxis2[1],
        normalAxis2[2],
        y2[0],
        y2[1],
        y2[2],
        hingeAxis2[0],
        hingeAxis2[1],
        hingeAxis2[2],
    );

    // convert to quaternions
    const q1 = quat.create();
    const q2 = quat.create();
    quat.fromMat3(q1, mat1);
    quat.fromMat3(q2, mat2);

    // return q2 * conjugate(q1)
    const q1Conj = quat.create();
    quat.conjugate(q1Conj, q1);

    const result = quat.create();
    quat.multiply(result, q2, q1Conj);
    return result;
}

/** Reset constraint state for pooling reuse - not exported */
function resetConstraint(constraint: HingeConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
    hingeRotationConstraintPart.deactivate(constraint.rotationConstraintPart);
    angleConstraintPart.deactivate(constraint.rotationLimitsConstraintPart);
    angleConstraintPart.deactivate(constraint.motorConstraintPart);
}

/** create a hinge constraint */
export function create(world: World, settings: HingeConstraintSettings): HingeConstraint {
    const pool = ensurePool<HingeConstraint>(world.constraints, ConstraintType.HINGE);
    const bodies = world.bodies;

    // get next sequence
    const sequence = pool.nextSequence;
    pool.nextSequence = (pool.nextSequence + 1) & SEQUENCE_MASK;

    // get constraint from pool
    let index: number;
    let constraint: HingeConstraint;
    if (pool.freeIndices.length > 0) {
        index = pool.freeIndices.pop()!;
        constraint = pool.constraints[index];
    } else {
        index = pool.constraints.length;
        constraint = makeHingeConstraint();
        pool.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    // set constraint id, index, sequence
    constraint.id = serConstraintId(index, ConstraintType.HINGE, sequence);
    constraint.index = index;
    constraint.sequence = sequence;

    // set base constraint properties
    constraint.constraintPriority = settings.constraintPriority ?? 0;
    constraint.numVelocityStepsOverride = settings.numVelocityStepsOverride ?? 0;
    constraint.numPositionStepsOverride = settings.numPositionStepsOverride ?? 0;

    // extract body indices
    constraint.bodyIndexA = getBodyIdIndex(settings.bodyIdA);
    constraint.bodyIndexB = getBodyIdIndex(settings.bodyIdB);

    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    // normalize axes
    const hingeAxis1 = vec3.create();
    const hingeAxis2 = vec3.create();
    const normalAxis1 = vec3.create();
    const normalAxis2 = vec3.create();
    vec3.normalize(hingeAxis1, settings.hingeAxisA);
    vec3.normalize(hingeAxis2, settings.hingeAxisB);
    vec3.normalize(normalAxis1, settings.normalAxisA);
    vec3.normalize(normalAxis2, settings.normalAxisB);

    // convert to local space if needed
    const space = settings.space ?? ConstraintSpace.WORLD;

    if (space === ConstraintSpace.WORLD) {
        if (bodyA && !bodyA._pooled && bodyB && !bodyB._pooled) {
            const invQuatA = quat.create();
            const invQuatB = quat.create();
            quat.conjugate(invQuatA, bodyA.quaternion);
            quat.conjugate(invQuatB, bodyB.quaternion);

            // transform positions to local space
            vec3.subtract(constraint.localSpacePosition1, settings.pointA, bodyA.centerOfMassPosition);
            vec3.transformQuat(constraint.localSpacePosition1, constraint.localSpacePosition1, invQuatA);

            vec3.subtract(constraint.localSpacePosition2, settings.pointB, bodyB.centerOfMassPosition);
            vec3.transformQuat(constraint.localSpacePosition2, constraint.localSpacePosition2, invQuatB);

            // transform axes to local space
            vec3.transformQuat(constraint.localSpaceHingeAxis1, hingeAxis1, invQuatA);
            vec3.transformQuat(constraint.localSpaceHingeAxis2, hingeAxis2, invQuatB);
            vec3.transformQuat(constraint.localSpaceNormalAxis1, normalAxis1, invQuatA);
            vec3.transformQuat(constraint.localSpaceNormalAxis2, normalAxis2, invQuatB);

            // calculate inverse initial orientation from world space axes
            const r0 = getInvInitialOrientationXZ(normalAxis1, hingeAxis1, normalAxis2, hingeAxis2);

            // transform to body-relative space: q20^-1 * r0 * q10
            // this is needed because at runtime we compute: diff = q2 * invInitialOrientation * q1^-1
            // with body-relative invInitialOrientation, when bodies are at their initial orientations (q1=q10, q2=q20):
            // diff = q20 * (q20^-1 * r0 * q10) * q10^-1 = r0
            quat.multiply(constraint.invInitialOrientation, invQuatB, r0);
            quat.multiply(constraint.invInitialOrientation, constraint.invInitialOrientation, bodyA.quaternion);
        }
    } else {
        // already in local space
        vec3.copy(constraint.localSpacePosition1, settings.pointA);
        vec3.copy(constraint.localSpacePosition2, settings.pointB);
        vec3.copy(constraint.localSpaceHingeAxis1, hingeAxis1);
        vec3.copy(constraint.localSpaceHingeAxis2, hingeAxis2);
        vec3.copy(constraint.localSpaceNormalAxis1, normalAxis1);
        vec3.copy(constraint.localSpaceNormalAxis2, normalAxis2);

        // need to compute world space axes for invInitialOrientation
        if (bodyA && !bodyA._pooled && bodyB && !bodyB._pooled) {
            const worldNormal1 = vec3.create();
            const worldHinge1 = vec3.create();
            const worldNormal2 = vec3.create();
            const worldHinge2 = vec3.create();
            vec3.transformQuat(worldNormal1, normalAxis1, bodyA.quaternion);
            vec3.transformQuat(worldHinge1, hingeAxis1, bodyA.quaternion);
            vec3.transformQuat(worldNormal2, normalAxis2, bodyB.quaternion);
            vec3.transformQuat(worldHinge2, hingeAxis2, bodyB.quaternion);

            constraint.invInitialOrientation = getInvInitialOrientationXZ(worldNormal1, worldHinge1, worldNormal2, worldHinge2);
        }
    }

    // set limits
    const limitsMin = settings.limitsMin ?? -Math.PI;
    const limitsMax = settings.limitsMax ?? Math.PI;

    if (limitsMin <= -Math.PI && limitsMax >= Math.PI) {
        constraint.hasLimits = false;
        constraint.limitsMin = -Math.PI;
        constraint.limitsMax = Math.PI;
    } else {
        constraint.hasLimits = true;
        constraint.limitsMin = Math.max(limitsMin, -Math.PI);
        constraint.limitsMax = Math.min(limitsMax, Math.PI);
    }

    if (settings.limitsSpringSettings) {
        springSettings.copy(constraint.limitsSpringSettings, settings.limitsSpringSettings);
    }

    // set motor defaults
    constraint.motorState = MotorState.OFF;
    constraint.targetAngularVelocity = 0;
    constraint.targetAngle = 0;
    constraint.maxFrictionTorque = settings.maxFrictionTorque ?? 0;

    // reset motor settings to defaults
    motorSettings.reset(constraint.motorSettings);

    // copy motor settings from input if provided
    if (settings.motorSettings) {
        motorSettings.copy(constraint.motorSettings, settings.motorSettings);
    }

    // track constraint on both bodies
    bodyA.constraintIds.push(constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB) {
        bodyB.constraintIds.push(constraint.id);
    }

    return constraint;
}

/** remove a hinge constraint */
export function remove(world: World, constraint: HingeConstraint): void {
    const pool = ensurePool<HingeConstraint>(world.constraints, ConstraintType.HINGE);
    const bodies = world.bodies;

    // remove from bodies constraintIds arrays
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];
    if (bodyA && !bodyA._pooled) {
        removeConstraintIdFromBody(bodyA, constraint.id);
    }
    if (constraint.bodyIndexA !== constraint.bodyIndexB && bodyB && !bodyB._pooled) {
        removeConstraintIdFromBody(bodyB, constraint.id);
    }

    // reset constraint state for pooling reuse
    resetConstraint(constraint);

    constraint._pooled = true;
    constraint.id = INVALID_CONSTRAINT_ID;
    pool.freeIndices.push(constraint.index);
}

/** get hinge constraint by id */
export function get(world: World, id: ConstraintId): HingeConstraint | undefined {
    const pool = world.constraints.pools[ConstraintType.HINGE] as ConstraintPool<HingeConstraint> | undefined;
    if (!pool) return undefined;
    const index = getConstraintIdIndex(id);
    const constraint = pool.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

/**
 * Set hinge constraint limits.
 * @param constraint the constraint to modify
 * @param min minimum angle in radians
 * @param max maximum angle in radians
 */
export function setLimits(constraint: HingeConstraint, min: number, max: number): void {
    constraint.limitsMin = Math.max(min, -Math.PI);
    constraint.limitsMax = Math.min(max, Math.PI);
    constraint.hasLimits = constraint.limitsMin > -Math.PI || constraint.limitsMax < Math.PI;

    // clamp target angle to new limits
    if (constraint.hasLimits) {
        constraint.targetAngle = Math.max(constraint.limitsMin, Math.min(constraint.limitsMax, constraint.targetAngle));
    }
}

/** Set motor state for hinge constraint. */
export function setMotorState(constraint: HingeConstraint, state: MotorState): void {
    constraint.motorState = state;
}

/**
 * Set target angular velocity for velocity motor mode.
 * @param constraint the constraint to modify
 * @param velocity target angular velocity in rad/s
 */
export function setTargetAngularVelocity(constraint: HingeConstraint, velocity: number): void {
    constraint.targetAngularVelocity = velocity;
}

/**
 * Set target angle for position motor mode.
 * @param constraint the constraint to modify
 * @param angle target angle in radians
 */
export function setTargetAngle(constraint: HingeConstraint, angle: number): void {
    constraint.targetAngle = constraint.hasLimits ? Math.max(constraint.limitsMin, Math.min(constraint.limitsMax, angle)) : angle;
}

/**
 * Get current hinge angle.
 * @returns current angle in radians
 */
export function getCurrentAngle(constraint: HingeConstraint): number {
    return constraint.theta;
}

const _hingeConstraint_rotA = /* @__PURE__ */ mat4.create();
const _hingeConstraint_rotB = /* @__PURE__ */ mat4.create();
const _hingeConstraint_worldHingeAxis1 = /* @__PURE__ */ vec3.create();
const _hingeConstraint_worldHingeAxis2 = /* @__PURE__ */ vec3.create();
const _hingeConstraint_q1 = /* @__PURE__ */ quat.create();
const _hingeConstraint_q2 = /* @__PURE__ */ quat.create();
const _hingeConstraint_qRel = /* @__PURE__ */ quat.create();

/** calculate the current hinge angle theta */
function calculateA1AndTheta(constraint: HingeConstraint, bodyA: RigidBody, bodyB: RigidBody): void {
    if (!constraint.hasLimits && constraint.motorState === MotorState.OFF && constraint.maxFrictionTorque <= 0) {
        return;
    }

    // calculate world space hinge axis
    vec3.transformQuat(_hingeConstraint_worldHingeAxis1, constraint.localSpaceHingeAxis1, bodyA.quaternion);
    vec3.copy(constraint.worldSpaceHingeAxis1, _hingeConstraint_worldHingeAxis1);

    // calculate relative orientation
    // qRel = q2 * invInitialOrientation * conjugate(q1)
    quat.copy(_hingeConstraint_q1, bodyA.quaternion);
    quat.copy(_hingeConstraint_q2, bodyB.quaternion);

    // q2 * invInitialOrientation
    quat.multiply(_hingeConstraint_qRel, _hingeConstraint_q2, constraint.invInitialOrientation);
    // * conjugate(q1)
    quat.conjugate(_hingeConstraint_q1, _hingeConstraint_q1);
    quat.multiply(_hingeConstraint_qRel, _hingeConstraint_qRel, _hingeConstraint_q1);

    // get angle from quaternion (rotation around hinge axis)
    // 2 * atan(dot(axis, xyz) / w) — must use atan (not atan2) for correct [-PI, PI] range
    const qw = _hingeConstraint_qRel[3];
    const axisDot =
        _hingeConstraint_worldHingeAxis1[0] * _hingeConstraint_qRel[0] +
        _hingeConstraint_worldHingeAxis1[1] * _hingeConstraint_qRel[1] +
        _hingeConstraint_worldHingeAxis1[2] * _hingeConstraint_qRel[2];

    constraint.theta = qw === 0 ? Math.PI : 2 * Math.atan(axisDot / qw);
}

/** center angle around zero (wrap to [-PI, PI]). */
function centerAngleAroundZero(angle: number): number {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
}

/** get the smallest angle to the closest limit. */
function getSmallestAngleToLimit(constraint: HingeConstraint): number {
    const distToMin = centerAngleAroundZero(constraint.theta - constraint.limitsMin);
    const distToMax = centerAngleAroundZero(constraint.theta - constraint.limitsMax);
    return Math.abs(distToMin) < Math.abs(distToMax) ? distToMin : distToMax;
}

/** check if min limit is the closest limit. */
function isMinLimitClosest(constraint: HingeConstraint): boolean {
    const distToMin = centerAngleAroundZero(constraint.theta - constraint.limitsMin);
    const distToMax = centerAngleAroundZero(constraint.theta - constraint.limitsMax);
    return Math.abs(distToMin) < Math.abs(distToMax);
}

/** calculate rotation limits constraint properties. */
function calculateRotationLimitsConstraintProperties(
    constraint: HingeConstraint,
    bodyA: RigidBody,
    bodyB: RigidBody,
    deltaTime: number,
): void {
    // activate when at or beyond limits
    if (constraint.hasLimits && (constraint.theta <= constraint.limitsMin || constraint.theta >= constraint.limitsMax)) {
        angleConstraintPart.calculateConstraintPropertiesWithSettings(
            constraint.rotationLimitsConstraintPart,
            deltaTime,
            bodyA,
            bodyB,
            constraint.worldSpaceHingeAxis1,
            0,
            getSmallestAngleToLimit(constraint),
            constraint.limitsSpringSettings,
        );
    } else {
        angleConstraintPart.deactivate(constraint.rotationLimitsConstraintPart);
    }
}

/** calculate motor constraint properties. */
function calculateMotorConstraintProperties(
    constraint: HingeConstraint,
    bodyA: RigidBody,
    bodyB: RigidBody,
    deltaTime: number,
): void {
    switch (constraint.motorState) {
        case MotorState.OFF:
            if (constraint.maxFrictionTorque > 0) {
                angleConstraintPart.calculateConstraintProperties(
                    constraint.motorConstraintPart,
                    bodyA,
                    bodyB,
                    constraint.worldSpaceHingeAxis1,
                    0,
                );
            } else {
                angleConstraintPart.deactivate(constraint.motorConstraintPart);
            }
            break;

        case MotorState.VELOCITY:
            angleConstraintPart.calculateConstraintProperties(
                constraint.motorConstraintPart,
                bodyA,
                bodyB,
                constraint.worldSpaceHingeAxis1,
                -constraint.targetAngularVelocity,
            );
            break;

        case MotorState.POSITION:
            if (constraint.motorSettings.springSettings.frequencyOrStiffness > 0) {
                const error = centerAngleAroundZero(constraint.theta - constraint.targetAngle);
                angleConstraintPart.calculateConstraintPropertiesWithSettings(
                    constraint.motorConstraintPart,
                    deltaTime,
                    bodyA,
                    bodyB,
                    constraint.worldSpaceHingeAxis1,
                    0,
                    error,
                    constraint.motorSettings.springSettings,
                );
            } else {
                angleConstraintPart.deactivate(constraint.motorConstraintPart);
            }
            break;
    }
}

function setupVelocity(constraint: HingeConstraint, bodies: Bodies, deltaTime: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // convert quaternions to rotation matrices
    mat4.fromQuat(_hingeConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_hingeConstraint_rotB, bodyB.quaternion);

    // calculate world space axes
    vec3.transformQuat(_hingeConstraint_worldHingeAxis1, constraint.localSpaceHingeAxis1, bodyA.quaternion);
    vec3.transformQuat(_hingeConstraint_worldHingeAxis2, constraint.localSpaceHingeAxis2, bodyB.quaternion);

    // setup point constraint
    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _hingeConstraint_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _hingeConstraint_rotB,
        constraint.localSpacePosition2,
    );

    // setup rotation constraint (keeps hinge axes aligned)
    hingeRotationConstraintPart.calculateConstraintProperties(
        constraint.rotationConstraintPart,
        bodyA,
        bodyB,
        _hingeConstraint_worldHingeAxis1,
        _hingeConstraint_worldHingeAxis2,
    );

    // calculate current angle
    calculateA1AndTheta(constraint, bodyA, bodyB);

    // setup limits constraint
    calculateRotationLimitsConstraintProperties(constraint, bodyA, bodyB, deltaTime);

    // setup motor constraint
    calculateMotorConstraintProperties(constraint, bodyA, bodyB, deltaTime);
}

function warmStartVelocity(constraint: HingeConstraint, bodies: Bodies, warmStartImpulseRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // warm start all constraint parts
    angleConstraintPart.warmStart(constraint.motorConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    pointConstraintPart.warmStart(constraint.pointConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    hingeRotationConstraintPart.warmStart(constraint.rotationConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    angleConstraintPart.warmStart(constraint.rotationLimitsConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
}

function solveVelocity(constraint: HingeConstraint, bodies: Bodies, deltaTime: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    let motor = false;
    let pos = false;
    let rot = false;
    let limit = false;

    // solve motor
    if (angleConstraintPart.isActive(constraint.motorConstraintPart)) {
        switch (constraint.motorState) {
            case MotorState.OFF: {
                const maxLambda = constraint.maxFrictionTorque * deltaTime;
                motor = angleConstraintPart.solveVelocityConstraint(
                    constraint.motorConstraintPart,
                    bodyA,
                    bodyB,
                    constraint.worldSpaceHingeAxis1,
                    -maxLambda,
                    maxLambda,
                );
                break;
            }
            case MotorState.VELOCITY:
            case MotorState.POSITION: {
                motor = angleConstraintPart.solveVelocityConstraint(
                    constraint.motorConstraintPart,
                    bodyA,
                    bodyB,
                    constraint.worldSpaceHingeAxis1,
                    deltaTime * constraint.motorSettings.minTorqueLimit,
                    deltaTime * constraint.motorSettings.maxTorqueLimit,
                );
                break;
            }
        }
    }

    // solve point constraint
    pos = pointConstraintPart.solveVelocityConstraint(constraint.pointConstraintPart, bodyA, bodyB);

    // solve rotation constraint
    rot = hingeRotationConstraintPart.solveVelocityConstraint(constraint.rotationConstraintPart, bodyA, bodyB);

    // solve rotation limits
    if (angleConstraintPart.isActive(constraint.rotationLimitsConstraintPart)) {
        let minLambda: number;
        let maxLambda: number;

        if (constraint.limitsMin === constraint.limitsMax) {
            minLambda = -Infinity;
            maxLambda = Infinity;
        } else if (isMinLimitClosest(constraint)) {
            minLambda = 0;
            maxLambda = Infinity;
        } else {
            minLambda = -Infinity;
            maxLambda = 0;
        }

        limit = angleConstraintPart.solveVelocityConstraint(
            constraint.rotationLimitsConstraintPart,
            bodyA,
            bodyB,
            constraint.worldSpaceHingeAxis1,
            minLambda,
            maxLambda,
        );
    }

    return motor || pos || rot || limit;
}

function solvePosition(constraint: HingeConstraint, bodies: Bodies, deltaTime: number, baumgarteFactor: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // motor operates on velocities only - no position solve

    // solve point constraint position
    mat4.fromQuat(_hingeConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_hingeConstraint_rotB, bodyB.quaternion);

    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _hingeConstraint_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _hingeConstraint_rotB,
        constraint.localSpacePosition2,
    );

    const pos = pointConstraintPart.solvePositionConstraint(constraint.pointConstraintPart, bodyA, bodyB, baumgarteFactor);

    // solve rotation constraint position (recalculate since bodies moved)
    mat4.fromQuat(_hingeConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_hingeConstraint_rotB, bodyB.quaternion);

    vec3.transformQuat(_hingeConstraint_worldHingeAxis1, constraint.localSpaceHingeAxis1, bodyA.quaternion);
    vec3.transformQuat(_hingeConstraint_worldHingeAxis2, constraint.localSpaceHingeAxis2, bodyB.quaternion);

    hingeRotationConstraintPart.calculateConstraintProperties(
        constraint.rotationConstraintPart,
        bodyA,
        bodyB,
        _hingeConstraint_worldHingeAxis1,
        _hingeConstraint_worldHingeAxis2,
    );

    const rot = hingeRotationConstraintPart.solvePositionConstraint(
        constraint.rotationConstraintPart,
        bodyA,
        bodyB,
        baumgarteFactor,
    );

    // solve rotation limits position (only if not using spring)
    let limit = false;
    if (constraint.hasLimits && constraint.limitsSpringSettings.frequencyOrStiffness <= 0) {
        calculateA1AndTheta(constraint, bodyA, bodyB);
        calculateRotationLimitsConstraintProperties(constraint, bodyA, bodyB, deltaTime);

        if (angleConstraintPart.isActive(constraint.rotationLimitsConstraintPart)) {
            limit = angleConstraintPart.solvePositionConstraint(
                constraint.rotationLimitsConstraintPart,
                bodyA,
                bodyB,
                getSmallestAngleToLimit(constraint),
                baumgarteFactor,
            );
        }
    }

    return pos || rot || limit;
}

function resetWarmStart(constraint: HingeConstraint): void {
    angleConstraintPart.deactivate(constraint.motorConstraintPart);
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
    hingeRotationConstraintPart.deactivate(constraint.rotationConstraintPart);
    angleConstraintPart.deactivate(constraint.rotationLimitsConstraintPart);
}

/** the constraint definition for hinge constraint */
export const def = /* @__PURE__ */ (() =>
    defineUserConstraint<HingeConstraint>({
        type: ConstraintType.HINGE,
        setupVelocity,
        warmStartVelocity,
        solveVelocity,
        solvePosition,
        resetWarmStart,
        getIterationOverrides: (out, constraint) => {
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
        },
        getSortFields: (out, constraint) => {
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
        },
    }))();
