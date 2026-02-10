import type { Quat, Vec3 } from 'mathcat';
import { euler, mat3, mat4, quat, vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies';
import { type BodyId, getBodyIdIndex } from '../body/body-id';
import { getInverseInertiaForRotation } from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
import type { World } from '../world';
import { type ConstraintBase, ConstraintSpace, makeConstraintBase, removeConstraintIdFromBody } from './constraints';
import { MotorState } from './constraint-part/motor-settings';
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
import type { AxisConstraintPart } from './constraint-part/axis-constraint-part';
import * as axisConstraintPart from './constraint-part/axis-constraint-part';
import type { PointConstraintPart } from './constraint-part/point-constraint-part';
import * as pointConstraintPart from './constraint-part/point-constraint-part';
import type { RotationEulerConstraintPart } from './constraint-part/rotation-euler-constraint-part';
import * as rotationEulerConstraintPart from './constraint-part/rotation-euler-constraint-part';
import type { SpringSettings } from './constraint-part/spring-settings';
import * as springSettings from './constraint-part/spring-settings';
import type { SwingTwistConstraintPart } from './constraint-part/swing-twist-constraint-part';
import * as swingTwistConstraintPart from './constraint-part/swing-twist-constraint-part';
import { type ConstraintPool, defineConstraint, ensurePool } from './constraints';
import { getSwingTwist, SwingType } from './constraint-part/swing-twist-constraint-part';

const _twist_temp = /* @__PURE__ */ quat.create();

/** extract the twist component of a quaternion around a specific axis */
function getTwistAroundAxis(q: Quat, axis: Vec3, outTwist: Quat): void {
    // Project the quaternion rotation onto the axis
    // twist = normalize([q.xyz · axis * axis, q.w])
    const dot = q[0] * axis[0] + q[1] * axis[1] + q[2] * axis[2];
    const lenSq = dot * dot + q[3] * q[3];
    if (lenSq > 1e-10) {
        const invLen = 1 / Math.sqrt(lenSq);
        quat.set(outTwist, dot * axis[0] * invLen, dot * axis[1] * invLen, dot * axis[2] * invLen, q[3] * invLen);
    } else {
        quat.identity(outTwist);
    }
}

/** axis indices for 6DOF constraint */
export enum SixDOFAxis {
    TRANSLATION_X = 0,
    TRANSLATION_Y = 1,
    TRANSLATION_Z = 2,
    ROTATION_X = 3,
    ROTATION_Y = 4,
    ROTATION_Z = 5,
    NUM = 6,
    NUM_TRANSLATION = 3,
}

/**
 * SixDOFConstraint allows per-axis control over all 6 degrees of freedom.
 * Each axis can be: free, fixed, or limited with optional motors.
 */
export type SixDOFConstraint = ConstraintBase & {
    // local space configuration
    localSpacePosition1: Vec3;
    localSpacePosition2: Vec3;
    constraintToBody1: Quat;
    constraintToBody2: Quat;

    // per-axis configuration
    /** min limits for each axis (Infinity = fixed at 0, -Infinity = free) */
    limitMin: [number, number, number, number, number, number]; // [6]
    /** max limits for each axis (-Infinity = fixed at 0, Infinity = free) */
    limitMax: [number, number, number, number, number, number];
    /** spring settings for translation limits */
    limitsSpringSettings: SpringSettings[];
    /** max friction per axis (0 = no friction) */
    maxFriction: [number, number, number, number, number, number];

    // motor configuration
    motorState: MotorState[];
    motorSpringSettings: SpringSettings[];
    motorMinForceLimit: [number, number, number, number, number, number];
    motorMaxForceLimit: [number, number, number, number, number, number];

    // motor targets
    targetVelocity: Vec3;
    targetAngularVelocity: Vec3;
    targetPosition: Vec3;
    targetOrientation: Quat;

    // swing type
    swingType: SwingType;

    // runtime cached values
    translationAxis: [Vec3, Vec3, Vec3];
    rotationAxis: [Vec3, Vec3, Vec3];
    displacement: [number, number, number];
    freeAxis: number; // bitmask
    fixedAxis: number; // bitmask
    translationMotorActive: boolean;
    rotationMotorActive: boolean;
    rotationPositionMotorActive: number; // bitmask
    hasSpringLimits: boolean;

    // constraint parts
    translationConstraintPart: [AxisConstraintPart, AxisConstraintPart, AxisConstraintPart];
    pointConstraintPart: PointConstraintPart;
    swingTwistConstraintPart: SwingTwistConstraintPart;
    rotationConstraintPart: RotationEulerConstraintPart;
    motorTranslationConstraintPart: [AxisConstraintPart, AxisConstraintPart, AxisConstraintPart];
    motorRotationConstraintPart: [AngleConstraintPart, AngleConstraintPart, AngleConstraintPart];
};

/** Creates default SixDOF constraint */
function makeSixDOFConstraint(): SixDOFConstraint {
    return {
        ...makeConstraintBase(),
        localSpacePosition1: vec3.create(),
        localSpacePosition2: vec3.create(),
        constraintToBody1: quat.create(),
        constraintToBody2: quat.create(),
        limitMin: [-Infinity, -Infinity, -Infinity, -Math.PI, -Math.PI, -Math.PI],
        limitMax: [Infinity, Infinity, Infinity, Math.PI, Math.PI, Math.PI],
        limitsSpringSettings: [springSettings.create(), springSettings.create(), springSettings.create()],
        maxFriction: [0, 0, 0, 0, 0, 0],
        motorState: [MotorState.OFF, MotorState.OFF, MotorState.OFF, MotorState.OFF, MotorState.OFF, MotorState.OFF],
        motorSpringSettings: [
            springSettings.create(),
            springSettings.create(),
            springSettings.create(),
            springSettings.create(),
            springSettings.create(),
            springSettings.create(),
        ],
        motorMinForceLimit: [-Infinity, -Infinity, -Infinity, -Infinity, -Infinity, -Infinity],
        motorMaxForceLimit: [Infinity, Infinity, Infinity, Infinity, Infinity, Infinity],
        targetVelocity: vec3.create(),
        targetAngularVelocity: vec3.create(),
        targetPosition: vec3.create(),
        targetOrientation: quat.create(),
        swingType: SwingType.CONE,
        translationAxis: [vec3.create(), vec3.create(), vec3.create()],
        rotationAxis: [vec3.create(), vec3.create(), vec3.create()],
        displacement: [0, 0, 0],
        freeAxis: 0,
        fixedAxis: 0,
        translationMotorActive: false,
        rotationMotorActive: false,
        rotationPositionMotorActive: 0,
        hasSpringLimits: false,
        translationConstraintPart: [axisConstraintPart.create(), axisConstraintPart.create(), axisConstraintPart.create()],
        pointConstraintPart: pointConstraintPart.create(),
        swingTwistConstraintPart: swingTwistConstraintPart.create(),
        rotationConstraintPart: rotationEulerConstraintPart.create(),
        motorTranslationConstraintPart: [axisConstraintPart.create(), axisConstraintPart.create(), axisConstraintPart.create()],
        motorRotationConstraintPart: [angleConstraintPart.create(), angleConstraintPart.create(), angleConstraintPart.create()],
    };
}

/** reset constraint state for pooling reuse */
function resetConstraint(constraint: SixDOFConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
    rotationEulerConstraintPart.deactivate(constraint.rotationConstraintPart);
    swingTwistConstraintPart.deactivate(constraint.swingTwistConstraintPart);
    for (const part of constraint.translationConstraintPart) axisConstraintPart.deactivate(part);
    for (const part of constraint.motorTranslationConstraintPart) axisConstraintPart.deactivate(part);
    for (const part of constraint.motorRotationConstraintPart) angleConstraintPart.deactivate(part);
}

/** Settings for creating a SixDOF constraint */
export type SixDOFConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    position1: Vec3;
    position2: Vec3;
    axisX1: Vec3;
    axisY1: Vec3;
    axisX2: Vec3;
    axisY2: Vec3;
    space?: ConstraintSpace;
    swingType?: SwingType;
    /** per-axis min limits */
    limitMin?: number[];
    /** per-axis max limits */
    limitMax?: number[];
    /** per-axis max friction */
    maxFriction?: number[];
    /** translation limit spring settings */
    limitsSpringSettings?: SpringSettings[];
    /** per-axis motor spring settings */
    motorSpringSettings?: SpringSettings[];
    /** per-axis motor min force limits */
    motorMinForceLimit?: number[];
    /** per-axis motor max force limits */
    motorMaxForceLimit?: number[];
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

// Helper functions
function isFixedAxis(constraint: SixDOFConstraint, axis: number): boolean {
    return (constraint.fixedAxis & (1 << axis)) !== 0;
}

function isFreeAxis(constraint: SixDOFConstraint, axis: number): boolean {
    return (constraint.freeAxis & (1 << axis)) !== 0;
}

function isTranslationConstrained(constraint: SixDOFConstraint): boolean {
    return (constraint.freeAxis & 0b111) !== 0b111;
}

function isTranslationFullyConstrained(constraint: SixDOFConstraint): boolean {
    return (constraint.fixedAxis & 0b111) === 0b111 && !constraint.hasSpringLimits;
}

function isRotationConstrained(constraint: SixDOFConstraint): boolean {
    return (constraint.freeAxis & 0b111000) !== 0b111000;
}

function isRotationFullyConstrained(constraint: SixDOFConstraint): boolean {
    return (constraint.fixedAxis & 0b111000) === 0b111000;
}

function hasFriction(constraint: SixDOFConstraint, axis: number): boolean {
    return !isFixedAxis(constraint, axis) && constraint.maxFriction[axis] > 0;
}

function updateTranslationLimits(constraint: SixDOFConstraint): void {
    for (let i = 0; i < 3; i++) {
        if (constraint.limitMin[i] > constraint.limitMax[i]) {
            constraint.limitMin[i] = constraint.limitMax[i] = 0;
        }
    }
}

function updateRotationLimits(constraint: SixDOFConstraint): void {
    if (constraint.swingType === SwingType.CONE) {
        constraint.limitMax[SixDOFAxis.ROTATION_Y] = Math.max(0, constraint.limitMax[SixDOFAxis.ROTATION_Y]);
        constraint.limitMax[SixDOFAxis.ROTATION_Z] = Math.max(0, constraint.limitMax[SixDOFAxis.ROTATION_Z]);
        constraint.limitMin[SixDOFAxis.ROTATION_Y] = -constraint.limitMax[SixDOFAxis.ROTATION_Y];
        constraint.limitMin[SixDOFAxis.ROTATION_Z] = -constraint.limitMax[SixDOFAxis.ROTATION_Z];
    }
    for (let i = 3; i < 6; i++) {
        constraint.limitMin[i] = Math.max(-Math.PI, Math.min(Math.PI, constraint.limitMin[i]));
        constraint.limitMax[i] = Math.max(-Math.PI, Math.min(Math.PI, constraint.limitMax[i]));
        if (constraint.limitMin[i] > constraint.limitMax[i]) {
            constraint.limitMin[i] = constraint.limitMax[i] = 0;
        }
    }
    // Update swing-twist constraint part limits
    swingTwistConstraintPart.setLimits(
        constraint.swingTwistConstraintPart,
        constraint.limitMin[SixDOFAxis.ROTATION_X],
        constraint.limitMax[SixDOFAxis.ROTATION_X],
        constraint.limitMin[SixDOFAxis.ROTATION_Y],
        constraint.limitMax[SixDOFAxis.ROTATION_Y],
        constraint.limitMin[SixDOFAxis.ROTATION_Z],
        constraint.limitMax[SixDOFAxis.ROTATION_Z],
    );
}

function updateFixedFreeAxis(constraint: SixDOFConstraint): void {
    constraint.freeAxis = 0;
    constraint.fixedAxis = 0;
    for (let a = 0; a < 6; a++) {
        const limit = a >= 3 ? Math.PI : Infinity;
        if (constraint.limitMin[a] >= constraint.limitMax[a]) {
            constraint.fixedAxis |= 1 << a;
        } else if (constraint.limitMin[a] <= -limit && constraint.limitMax[a] >= limit) {
            constraint.freeAxis |= 1 << a;
        }
    }
}

function cacheTranslationMotorActive(constraint: SixDOFConstraint): void {
    constraint.translationMotorActive =
        constraint.motorState[0] !== MotorState.OFF ||
        constraint.motorState[1] !== MotorState.OFF ||
        constraint.motorState[2] !== MotorState.OFF ||
        hasFriction(constraint, SixDOFAxis.TRANSLATION_X) ||
        hasFriction(constraint, SixDOFAxis.TRANSLATION_Y) ||
        hasFriction(constraint, SixDOFAxis.TRANSLATION_Z);
}

function cacheRotationMotorActive(constraint: SixDOFConstraint): void {
    constraint.rotationMotorActive =
        constraint.motorState[3] !== MotorState.OFF ||
        constraint.motorState[4] !== MotorState.OFF ||
        constraint.motorState[5] !== MotorState.OFF ||
        hasFriction(constraint, SixDOFAxis.ROTATION_X) ||
        hasFriction(constraint, SixDOFAxis.ROTATION_Y) ||
        hasFriction(constraint, SixDOFAxis.ROTATION_Z);
}

function cacheRotationPositionMotorActive(constraint: SixDOFConstraint): void {
    constraint.rotationPositionMotorActive = 0;
    for (let i = 0; i < 3; i++) {
        if (constraint.motorState[SixDOFAxis.ROTATION_X + i] === MotorState.POSITION) {
            constraint.rotationPositionMotorActive |= 1 << i;
        }
    }
}

function cacheHasSpringLimits(constraint: SixDOFConstraint): void {
    constraint.hasSpringLimits =
        constraint.limitsSpringSettings[0].frequencyOrStiffness > 0 ||
        constraint.limitsSpringSettings[1].frequencyOrStiffness > 0 ||
        constraint.limitsSpringSettings[2].frequencyOrStiffness > 0;
}

const _c_to_b1 = /* @__PURE__ */ mat3.create();
const _c_to_b2 = /* @__PURE__ */ mat3.create();
const _axisZ1 = /* @__PURE__ */ vec3.create();
const _axisZ2 = /* @__PURE__ */ vec3.create();
const _create_axisX1 = /* @__PURE__ */ vec3.create();
const _create_axisY1 = /* @__PURE__ */ vec3.create();
const _create_axisX2 = /* @__PURE__ */ vec3.create();
const _create_axisY2 = /* @__PURE__ */ vec3.create();
const _create_invQuatA = /* @__PURE__ */ quat.create();
const _create_invQuatB = /* @__PURE__ */ quat.create();

/** Create a SixDOF constraint */
export function create(world: World, settings: SixDOFConstraintSettings): SixDOFConstraint {
    const pool = ensurePool<SixDOFConstraint>(world.constraints, ConstraintType.SIX_DOF);
    const bodies = world.bodies;

    const sequence = pool.nextSequence;
    pool.nextSequence = (pool.nextSequence + 1) & SEQUENCE_MASK;

    let index: number;
    let constraint: SixDOFConstraint;
    if (pool.freeIndices.length > 0) {
        index = pool.freeIndices.pop()!;
        constraint = pool.constraints[index];
    } else {
        index = pool.constraints.length;
        constraint = makeSixDOFConstraint();
        pool.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    constraint.id = serConstraintId(index, ConstraintType.SIX_DOF, sequence);
    constraint.index = index;
    constraint.sequence = sequence;

    // set base constraint properties
    constraint.constraintPriority = settings.constraintPriority ?? 0;
    constraint.numVelocityStepsOverride = settings.numVelocityStepsOverride ?? 0;
    constraint.numPositionStepsOverride = settings.numPositionStepsOverride ?? 0;

    constraint.bodyIndexA = getBodyIdIndex(settings.bodyIdA);
    constraint.bodyIndexB = getBodyIdIndex(settings.bodyIdB);

    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    // setup swing type
    constraint.swingType = settings.swingType ?? SwingType.CONE;
    constraint.swingTwistConstraintPart.swingType = constraint.swingType;

    // build constraint-to-body rotations from axes
    const axisX1 = _create_axisX1;
    vec3.normalize(axisX1, settings.axisX1);
    const axisY1 = _create_axisY1;
    vec3.normalize(axisY1, settings.axisY1);
    const axisX2 = _create_axisX2;
    vec3.normalize(axisX2, settings.axisX2);
    const axisY2 = _create_axisY2;
    vec3.normalize(axisY2, settings.axisY2);

    vec3.cross(_axisZ1, axisX1, axisY1);
    vec3.cross(_axisZ2, axisX2, axisY2);

    mat3.set(_c_to_b1, axisX1[0], axisX1[1], axisX1[2], axisY1[0], axisY1[1], axisY1[2], _axisZ1[0], _axisZ1[1], _axisZ1[2]);
    mat3.set(_c_to_b2, axisX2[0], axisX2[1], axisX2[2], axisY2[0], axisY2[1], axisY2[2], _axisZ2[0], _axisZ2[1], _axisZ2[2]);

    quat.fromMat3(constraint.constraintToBody1, _c_to_b1);
    quat.fromMat3(constraint.constraintToBody2, _c_to_b2);

    const space = settings.space ?? ConstraintSpace.WORLD;
    if (space === ConstraintSpace.WORLD) {
        const invQuatA = _create_invQuatA;
        const invQuatB = _create_invQuatB;
        quat.conjugate(invQuatA, bodyA.quaternion);
        quat.conjugate(invQuatB, bodyB.quaternion);

        vec3.subtract(constraint.localSpacePosition1, settings.position1, bodyA.centerOfMassPosition);
        vec3.transformQuat(constraint.localSpacePosition1, constraint.localSpacePosition1, invQuatA);

        vec3.subtract(constraint.localSpacePosition2, settings.position2, bodyB.centerOfMassPosition);
        vec3.transformQuat(constraint.localSpacePosition2, constraint.localSpacePosition2, invQuatB);

        quat.multiply(constraint.constraintToBody1, invQuatA, constraint.constraintToBody1);
        quat.multiply(constraint.constraintToBody2, invQuatB, constraint.constraintToBody2);
    } else {
        vec3.copy(constraint.localSpacePosition1, settings.position1);
        vec3.copy(constraint.localSpacePosition2, settings.position2);
    }

    // Copy limits
    if (settings.limitMin) {
        for (let i = 0; i < 6; i++) constraint.limitMin[i] = settings.limitMin[i] ?? constraint.limitMin[i];
    }
    if (settings.limitMax) {
        for (let i = 0; i < 6; i++) constraint.limitMax[i] = settings.limitMax[i] ?? constraint.limitMax[i];
    }
    if (settings.maxFriction) {
        for (let i = 0; i < 6; i++) constraint.maxFriction[i] = settings.maxFriction[i] ?? 0;
    }
    if (settings.limitsSpringSettings) {
        for (let i = 0; i < 3; i++) {
            if (settings.limitsSpringSettings[i]) {
                springSettings.copy(constraint.limitsSpringSettings[i], settings.limitsSpringSettings[i]!);
            }
        }
    }
    // set motor spring settings (defaults: 2.0 Hz for rotation, 10.0 Hz for translation)
    if (settings.motorSpringSettings) {
        for (let i = 0; i < 6; i++) {
            if (settings.motorSpringSettings[i]) {
                springSettings.copy(constraint.motorSpringSettings[i], settings.motorSpringSettings[i]!);
            } else {
                // set defaults: translation axes (0-2) use 10.0 Hz, rotation axes (3-5) use 2.0 Hz
                constraint.motorSpringSettings[i].mode = springSettings.SpringMode.FREQUENCY_AND_DAMPING;
                constraint.motorSpringSettings[i].frequencyOrStiffness = i < 3 ? 10.0 : 2.0;
                constraint.motorSpringSettings[i].damping = 1.0;
            }
        }
    } else {
        // no settings provided, set all defaults
        for (let i = 0; i < 6; i++) {
            constraint.motorSpringSettings[i].mode = springSettings.SpringMode.FREQUENCY_AND_DAMPING;
            constraint.motorSpringSettings[i].frequencyOrStiffness = i < 3 ? 10.0 : 2.0;
            constraint.motorSpringSettings[i].damping = 1.0;
        }
    }
    if (settings.motorMinForceLimit) {
        for (let i = 0; i < 6; i++) constraint.motorMinForceLimit[i] = settings.motorMinForceLimit[i] ?? -Infinity;
    }
    if (settings.motorMaxForceLimit) {
        for (let i = 0; i < 6; i++) constraint.motorMaxForceLimit[i] = settings.motorMaxForceLimit[i] ?? Infinity;
    }

    updateTranslationLimits(constraint);
    updateRotationLimits(constraint);
    updateFixedFreeAxis(constraint);
    cacheHasSpringLimits(constraint);
    cacheTranslationMotorActive(constraint);
    cacheRotationMotorActive(constraint);
    cacheRotationPositionMotorActive(constraint);

    // reset targets
    vec3.zero(constraint.targetVelocity);
    vec3.zero(constraint.targetAngularVelocity);
    vec3.zero(constraint.targetPosition);
    quat.identity(constraint.targetOrientation);

    // track on bodies
    bodyA.constraintIds.push(constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB) {
        bodyB.constraintIds.push(constraint.id);
    }

    return constraint;
}

/** Remove a SixDOF constraint */
export function remove(world: World, constraint: SixDOFConstraint): void {
    const pool = ensurePool<SixDOFConstraint>(world.constraints, ConstraintType.SIX_DOF);
    const bodies = world.bodies;

    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];
    if (bodyA && !bodyA._pooled) removeConstraintIdFromBody(bodyA, constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB && bodyB && !bodyB._pooled) {
        removeConstraintIdFromBody(bodyB, constraint.id);
    }

    // reset constraint state for pooling reuse
    resetConstraint(constraint);

    constraint._pooled = true;
    constraint.id = INVALID_CONSTRAINT_ID;
    pool.freeIndices.push(constraint.index);
}

/** Get SixDOF constraint by id */
export function get(world: World, id: ConstraintId): SixDOFConstraint | undefined {
    const pool = world.constraints.pools[ConstraintType.SIX_DOF] as ConstraintPool<SixDOFConstraint> | undefined;
    if (!pool) return undefined;
    const index = getConstraintIdIndex(id);
    const constraint = pool.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

const _setup_rotA = /* @__PURE__ */ mat4.create();
const _setup_rotB = /* @__PURE__ */ mat4.create();
const _setup_r1PlusU = /* @__PURE__ */ vec3.create();
const _setup_r2 = /* @__PURE__ */ vec3.create();
const _setup_u = /* @__PURE__ */ vec3.create();
const _setup_q = /* @__PURE__ */ quat.create();
const _setup_c1ToWorld = /* @__PURE__ */ quat.create();
const _setup_c2ToWorld = /* @__PURE__ */ quat.create();
const _setup_wsAxisMat = /* @__PURE__ */ mat4.create();
const _setup_rotationError = /* @__PURE__ */ vec3.create();
const _setup_diff = /* @__PURE__ */ quat.create();
const _setup_qConj = /* @__PURE__ */ quat.create();
const _setup_projectedDiff = /* @__PURE__ */ quat.create();
const _setup_targetOrientation = /* @__PURE__ */ quat.create();
const _setup_qSwing = /* @__PURE__ */ quat.create();
const _setup_qTwist = /* @__PURE__ */ quat.create();
const _setup_invInertiaA = /* @__PURE__ */ mat4.create();
const _setup_invInertiaB = /* @__PURE__ */ mat4.create();

const _getPositionConstraintProperties_p1 = /* @__PURE__ */ vec3.create();
const _getPositionConstraintProperties_p2 = /* @__PURE__ */ vec3.create();

function getPositionConstraintProperties(
    constraint: SixDOFConstraint,
    bodies: Bodies,
    outR1PlusU: Vec3,
    outR2: Vec3,
    outU: Vec3,
): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    // p1 = x1 + R1 * localPos1
    const p1 = _getPositionConstraintProperties_p1;
    vec3.transformQuat(p1, constraint.localSpacePosition1, bodyA.quaternion);
    vec3.add(p1, p1, bodyA.centerOfMassPosition);

    // p2 = x2 + R2 * localPos2
    const p2 = _getPositionConstraintProperties_p2;
    vec3.transformQuat(p2, constraint.localSpacePosition2, bodyB.quaternion);

    vec3.add(p2, p2, bodyB.centerOfMassPosition);

    // r1PlusU = p2 - x1
    vec3.subtract(outR1PlusU, p2, bodyA.centerOfMassPosition);
    // r2 = p2 - x2
    vec3.subtract(outR2, p2, bodyB.centerOfMassPosition);
    // u = p2 - p1
    vec3.subtract(outU, p2, p1);
}

const AXIS_X = /* @__PURE__ */ vec3.fromValues(1, 0, 0);
const AXIS_Y = /* @__PURE__ */ vec3.fromValues(0, 1, 0);
const AXIS_Z = /* @__PURE__ */ vec3.fromValues(0, 0, 1);

function setupVelocity(constraint: SixDOFConstraint, bodies: Bodies, deltaTime: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];
    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    mat4.fromQuat(_setup_rotA, bodyA.quaternion);
    mat4.fromQuat(_setup_rotB, bodyB.quaternion);

    // constraint body1 to world
    quat.multiply(_setup_c1ToWorld, bodyA.quaternion, constraint.constraintToBody1);

    // store world space translation axes
    mat4.fromQuat(_setup_wsAxisMat, _setup_c1ToWorld);
    for (let i = 0; i < 3; i++) {
        vec3.set(
            constraint.translationAxis[i],
            _setup_wsAxisMat[i * 4],
            _setup_wsAxisMat[i * 4 + 1],
            _setup_wsAxisMat[i * 4 + 2],
        );
    }

    // translation constraints
    if (isTranslationFullyConstrained(constraint)) {
        pointConstraintPart.calculateConstraintProperties(
            constraint.pointConstraintPart,
            bodyA,
            _setup_rotA,
            constraint.localSpacePosition1,
            bodyB,
            _setup_rotB,
            constraint.localSpacePosition2,
        );
    } else if (isTranslationConstrained(constraint) || constraint.translationMotorActive) {
        getPositionConstraintProperties(constraint, bodies, _setup_r1PlusU, _setup_r2, _setup_u);

        for (let i = 0; i < 3; i++) {
            const axis = i as SixDOFAxis;
            const translationAxis = constraint.translationAxis[i];
            const d = vec3.dot(translationAxis, _setup_u);
            constraint.displacement[i] = d;

            // get inverse mass and inertia for bodies
            const mpA = bodyA.motionProperties;
            const mpB = bodyB.motionProperties;
            const invMassA = bodyA.motionType === MotionType.DYNAMIC ? mpA.invMass : 0;
            const invMassB = bodyB.motionType === MotionType.DYNAMIC ? mpB.invMass : 0;
            const invInertiaA = _setup_invInertiaA;
            const invInertiaB = _setup_invInertiaB;

            if (bodyA.motionType === MotionType.DYNAMIC) {
                mat4.fromQuat(_setup_rotA, bodyA.quaternion);
                getInverseInertiaForRotation(invInertiaA, mpA, _setup_rotA);
            }

            if (bodyB.motionType === MotionType.DYNAMIC) {
                mat4.fromQuat(_setup_rotB, bodyB.quaternion);
                getInverseInertiaForRotation(invInertiaB, mpB, _setup_rotB);
            }

            // setup limit constraint
            let constraintActive = false;
            let constraintValue = 0;
            if (isFixedAxis(constraint, axis)) {
                constraintValue = d - constraint.limitMin[i];
                constraintActive = true;
            } else if (!isFreeAxis(constraint, axis)) {
                if (d <= constraint.limitMin[i]) {
                    constraintValue = d - constraint.limitMin[i];
                    constraintActive = true;
                } else if (d >= constraint.limitMax[i]) {
                    constraintValue = d - constraint.limitMax[i];
                    constraintActive = true;
                }
            }

            if (constraintActive) {
                if (constraint.limitsSpringSettings[i].frequencyOrStiffness > 0) {
                    axisConstraintPart.calculateConstraintPropertiesWithSettings(
                        constraint.translationConstraintPart[i],
                        deltaTime,
                        bodyA,
                        bodyB,
                        invMassA,
                        invMassB,
                        invInertiaA,
                        invInertiaB,
                        _setup_r1PlusU,
                        _setup_r2,
                        translationAxis,
                        0,
                        constraintValue,
                        constraint.limitsSpringSettings[i],
                    );
                } else {
                    axisConstraintPart.calculateConstraintProperties(
                        constraint.translationConstraintPart[i],
                        bodyA,
                        bodyB,
                        invMassA,
                        invMassB,
                        invInertiaA,
                        invInertiaB,
                        _setup_r1PlusU,
                        _setup_r2,
                        translationAxis,
                        0,
                    );
                }
            } else {
                axisConstraintPart.deactivate(constraint.translationConstraintPart[i]);
            }

            // setup motor constraint
            switch (constraint.motorState[i]) {
                case MotorState.OFF:
                    if (hasFriction(constraint, axis)) {
                        axisConstraintPart.calculateConstraintProperties(
                            constraint.motorTranslationConstraintPart[i],
                            bodyA,
                            bodyB,
                            invMassA,
                            invMassB,
                            invInertiaA,
                            invInertiaB,
                            _setup_r1PlusU,
                            _setup_r2,
                            translationAxis,
                            0,
                        );
                    } else {
                        axisConstraintPart.deactivate(constraint.motorTranslationConstraintPart[i]);
                    }
                    break;
                case MotorState.VELOCITY:
                    axisConstraintPart.calculateConstraintProperties(
                        constraint.motorTranslationConstraintPart[i],
                        bodyA,
                        bodyB,
                        invMassA,
                        invMassB,
                        invInertiaA,
                        invInertiaB,
                        _setup_r1PlusU,
                        _setup_r2,
                        translationAxis,
                        -constraint.targetVelocity[i],
                    );
                    break;
                case MotorState.POSITION:
                    if (constraint.motorSpringSettings[i].frequencyOrStiffness > 0) {
                        axisConstraintPart.calculateConstraintPropertiesWithSettings(
                            constraint.motorTranslationConstraintPart[i],
                            deltaTime,
                            bodyA,
                            bodyB,
                            invMassA,
                            invMassB,
                            invInertiaA,
                            invInertiaB,
                            _setup_r1PlusU,
                            _setup_r2,
                            translationAxis,
                            0,
                            vec3.dot(translationAxis, _setup_u) - constraint.targetPosition[i],
                            constraint.motorSpringSettings[i],
                        );
                    } else {
                        axisConstraintPart.deactivate(constraint.motorTranslationConstraintPart[i]);
                    }
                    break;
            }
        }
    }

    // rotation constraints
    quat.multiply(_setup_c2ToWorld, bodyB.quaternion, constraint.constraintToBody2);
    quat.conjugate(_setup_qConj, _setup_c1ToWorld);
    quat.multiply(_setup_q, _setup_qConj, _setup_c2ToWorld);

    if (isRotationFullyConstrained(constraint)) {
        rotationEulerConstraintPart.calculateConstraintProperties(
            constraint.rotationConstraintPart,
            bodyA,
            _setup_rotA,
            bodyB,
            _setup_rotB,
        );
    } else if (isRotationConstrained(constraint) || constraint.rotationMotorActive) {
        if (isRotationConstrained(constraint)) {
            swingTwistConstraintPart.calculateConstraintProperties(
                constraint.swingTwistConstraintPart,
                bodyA,
                bodyB,
                _setup_q,
                _setup_c1ToWorld,
            );
        } else {
            swingTwistConstraintPart.deactivate(constraint.swingTwistConstraintPart);
        }

        if (constraint.rotationMotorActive) {
            // calculate rotation motor axes
            mat4.fromQuat(_setup_wsAxisMat, _setup_c2ToWorld);
            for (let i = 0; i < 3; i++) {
                vec3.set(
                    constraint.rotationAxis[i],
                    _setup_wsAxisMat[i * 4],
                    _setup_wsAxisMat[i * 4 + 1],
                    _setup_wsAxisMat[i * 4 + 2],
                );
            }

            // Get target orientation along shortest path
            quat.copy(_setup_targetOrientation, constraint.targetOrientation);
            if (quat.dot(_setup_q, _setup_targetOrientation) < 0) {
                quat.scale(_setup_targetOrientation, _setup_targetOrientation, -1);
            }

            // diff = q^-1 * targetOrientation
            quat.conjugate(_setup_qConj, _setup_q);
            quat.multiply(_setup_diff, _setup_qConj, _setup_targetOrientation);

            // Project diff based on which axes have position motor
            switch (constraint.rotationPositionMotorActive) {
                case 0b001: // keep only rotation around X
                    getTwistAroundAxis(_setup_diff, AXIS_X, _setup_projectedDiff);
                    break;
                case 0b010: // keep only rotation around Y
                    getTwistAroundAxis(_setup_diff, AXIS_Y, _setup_projectedDiff);
                    break;
                case 0b100: // keep only rotation around Z
                    getTwistAroundAxis(_setup_diff, AXIS_Z, _setup_projectedDiff);
                    break;
                case 0b011: // remove rotation around Z (keep X and Y)
                    // swing_xy = q * twist_z^*
                    getTwistAroundAxis(_setup_diff, AXIS_Z, _twist_temp);
                    quat.conjugate(_twist_temp, _twist_temp);
                    quat.multiply(_setup_projectedDiff, _setup_diff, _twist_temp);
                    break;
                case 0b101: // remove rotation around Y (keep X and Z)
                    // swing_xz = q * twist_y^*
                    getTwistAroundAxis(_setup_diff, AXIS_Y, _twist_temp);
                    quat.conjugate(_twist_temp, _twist_temp);
                    quat.multiply(_setup_projectedDiff, _setup_diff, _twist_temp);
                    break;
                case 0b110: // remove rotation around X (keep Y and Z)
                    // swing_yz = q * twist_x^*
                    getTwistAroundAxis(_setup_diff, AXIS_X, _twist_temp);
                    quat.conjugate(_twist_temp, _twist_temp);
                    quat.multiply(_setup_projectedDiff, _setup_diff, _twist_temp);
                    break;
                default: // 0b111 or others: keep entire rotation
                    quat.copy(_setup_projectedDiff, _setup_diff);
                    break;
            }

            // rotation error from quaternion imaginary part (small angle approximation)
            vec3.set(
                _setup_rotationError,
                -2 * _setup_projectedDiff[0],
                -2 * _setup_projectedDiff[1],
                -2 * _setup_projectedDiff[2],
            );

            // setup rotation motors
            for (let i = 0; i < 3; i++) {
                const axis = (SixDOFAxis.ROTATION_X + i) as SixDOFAxis;
                const rotationAxis = constraint.rotationAxis[i];

                switch (constraint.motorState[axis]) {
                    case MotorState.OFF:
                        if (hasFriction(constraint, axis)) {
                            angleConstraintPart.calculateConstraintProperties(
                                constraint.motorRotationConstraintPart[i],
                                bodyA,
                                bodyB,
                                rotationAxis,
                                0,
                            );
                        } else {
                            angleConstraintPart.deactivate(constraint.motorRotationConstraintPart[i]);
                        }
                        break;
                    case MotorState.VELOCITY:
                        angleConstraintPart.calculateConstraintProperties(
                            constraint.motorRotationConstraintPart[i],
                            bodyA,
                            bodyB,
                            rotationAxis,
                            -constraint.targetAngularVelocity[i],
                        );
                        break;
                    case MotorState.POSITION:
                        if (constraint.motorSpringSettings[axis].frequencyOrStiffness > 0) {
                            angleConstraintPart.calculateConstraintPropertiesWithSettings(
                                constraint.motorRotationConstraintPart[i],
                                deltaTime,
                                bodyA,
                                bodyB,
                                rotationAxis,
                                0,
                                _setup_rotationError[i],
                                constraint.motorSpringSettings[axis],
                            );
                        } else {
                            angleConstraintPart.deactivate(constraint.motorRotationConstraintPart[i]);
                        }
                        break;
                }
            }
        }
    }
}

function warmStartVelocity(constraint: SixDOFConstraint, bodies: Bodies, warmStartRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];
    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    const invMassA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties.invMass : 0;
    const invMassB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties.invMass : 0;

    // warm start translation motors
    if (constraint.translationMotorActive) {
        for (let i = 0; i < 3; i++) {
            if (axisConstraintPart.isActive(constraint.motorTranslationConstraintPart[i])) {
                axisConstraintPart.warmStart(
                    constraint.motorTranslationConstraintPart[i],
                    bodyA,
                    bodyB,
                    invMassA,
                    invMassB,
                    constraint.translationAxis[i],
                    warmStartRatio,
                );
            }
        }
    }

    // warm start rotation motors
    if (constraint.rotationMotorActive) {
        for (let i = 0; i < 3; i++) {
            if (angleConstraintPart.isActive(constraint.motorRotationConstraintPart[i])) {
                angleConstraintPart.warmStart(constraint.motorRotationConstraintPart[i], bodyA, bodyB, warmStartRatio);
            }
        }
    }

    // warm start rotation constraints
    if (isRotationFullyConstrained(constraint)) {
        rotationEulerConstraintPart.warmStart(constraint.rotationConstraintPart, bodyA, bodyB, warmStartRatio);
    } else if (isRotationConstrained(constraint)) {
        swingTwistConstraintPart.warmStart(constraint.swingTwistConstraintPart, bodyA, bodyB, warmStartRatio);
    }

    // warm start translation constraints
    if (isTranslationFullyConstrained(constraint)) {
        pointConstraintPart.warmStart(constraint.pointConstraintPart, bodyA, bodyB, warmStartRatio);
    } else if (isTranslationConstrained(constraint)) {
        for (let i = 0; i < 3; i++) {
            if (axisConstraintPart.isActive(constraint.translationConstraintPart[i])) {
                axisConstraintPart.warmStart(
                    constraint.translationConstraintPart[i],
                    bodyA,
                    bodyB,
                    invMassA,
                    invMassB,
                    constraint.translationAxis[i],
                    warmStartRatio,
                );
            }
        }
    }
}

function resetWarmStart(constraint: SixDOFConstraint): void {
    // deactivate translation motor parts
    for (let i = 0; i < 3; i++) {
        axisConstraintPart.deactivate(constraint.motorTranslationConstraintPart[i]);
    }

    // deactivate rotation motor parts
    for (let i = 0; i < 3; i++) {
        angleConstraintPart.deactivate(constraint.motorRotationConstraintPart[i]);
    }

    // deactivate rotation constraint parts
    rotationEulerConstraintPart.deactivate(constraint.rotationConstraintPart);
    swingTwistConstraintPart.deactivate(constraint.swingTwistConstraintPart);

    // deactivate translation constraint parts
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
    for (let i = 0; i < 3; i++) {
        axisConstraintPart.deactivate(constraint.translationConstraintPart[i]);
    }
}

function solveVelocity(constraint: SixDOFConstraint, bodies: Bodies, deltaTime: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];
    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // solve translation motors
    if (constraint.translationMotorActive) {
        for (let i = 0; i < 3; i++) {
            if (axisConstraintPart.isActive(constraint.motorTranslationConstraintPart[i])) {
                switch (constraint.motorState[i]) {
                    case MotorState.OFF: {
                        const maxLambda = constraint.maxFriction[i] * deltaTime;
                        axisConstraintPart.solveVelocityConstraint(
                            constraint.motorTranslationConstraintPart[i],
                            bodyA,
                            bodyB,
                            constraint.translationAxis[i],
                            -maxLambda,
                            maxLambda,
                        );
                        break;
                    }
                    case MotorState.VELOCITY:
                    case MotorState.POSITION:
                        axisConstraintPart.solveVelocityConstraint(
                            constraint.motorTranslationConstraintPart[i],
                            bodyA,
                            bodyB,
                            constraint.translationAxis[i],
                            deltaTime * constraint.motorMinForceLimit[i],
                            deltaTime * constraint.motorMaxForceLimit[i],
                        );
                        break;
                }
            }
        }
    }

    // solve rotation motors
    if (constraint.rotationMotorActive) {
        for (let i = 0; i < 3; i++) {
            const axis = SixDOFAxis.ROTATION_X + i;
            if (angleConstraintPart.isActive(constraint.motorRotationConstraintPart[i])) {
                switch (constraint.motorState[axis]) {
                    case MotorState.OFF: {
                        const maxLambda = constraint.maxFriction[axis] * deltaTime;
                        angleConstraintPart.solveVelocityConstraint(
                            constraint.motorRotationConstraintPart[i],
                            bodyA,
                            bodyB,
                            constraint.rotationAxis[i],
                            -maxLambda,
                            maxLambda,
                        );
                        break;
                    }
                    case MotorState.VELOCITY:
                    case MotorState.POSITION:
                        angleConstraintPart.solveVelocityConstraint(
                            constraint.motorRotationConstraintPart[i],
                            bodyA,
                            bodyB,
                            constraint.rotationAxis[i],
                            deltaTime * constraint.motorMinForceLimit[axis],
                            deltaTime * constraint.motorMaxForceLimit[axis],
                        );
                        break;
                }
            }
        }
    }

    // solve rotation constraint
    if (isRotationFullyConstrained(constraint)) {
        rotationEulerConstraintPart.solveVelocityConstraint(constraint.rotationConstraintPart, bodyA, bodyB);
    } else if (isRotationConstrained(constraint)) {
        swingTwistConstraintPart.solveVelocityConstraint(constraint.swingTwistConstraintPart, bodyA, bodyB);
    }

    // solve translation constraint
    if (isTranslationFullyConstrained(constraint)) {
        pointConstraintPart.solveVelocityConstraint(constraint.pointConstraintPart, bodyA, bodyB);
    } else if (isTranslationConstrained(constraint)) {
        for (let i = 0; i < 3; i++) {
            if (axisConstraintPart.isActive(constraint.translationConstraintPart[i])) {
                let limitMin = -Infinity,
                    limitMax = Infinity;
                if (!isFixedAxis(constraint, i as SixDOFAxis)) {
                    if (constraint.displacement[i] <= constraint.limitMin[i]) limitMin = 0;
                    else if (constraint.displacement[i] >= constraint.limitMax[i]) limitMax = 0;
                }
                axisConstraintPart.solveVelocityConstraint(
                    constraint.translationConstraintPart[i],
                    bodyA,
                    bodyB,
                    constraint.translationAxis[i],
                    limitMin,
                    limitMax,
                );
            }
        }
    }

    return false;
}

const _pos_invInitialOrientation = /* @__PURE__ */ quat.create();
const _pos_q = /* @__PURE__ */ quat.create();
const _pos_c1ToWorld = /* @__PURE__ */ quat.create();
const _pos_translationAxis = /* @__PURE__ */ vec3.create();
const _pos_transLimitsMin = /* @__PURE__ */ vec3.create();
const _pos_localSpacePosition1WithOffset = /* @__PURE__ */ vec3.create();
const _pos_rotLimitsMinEuler = /* @__PURE__ */ euler.create();
const _solvePosition_rotLimitsMin = /* @__PURE__ */ quat.create();
const _solvePosition_constraintToBody1WithOffset = /* @__PURE__ */ quat.create();
const _solvePosition_c2ToWorld = /* @__PURE__ */ quat.create();
const _solvePosition_unitAxis = /* @__PURE__ */ vec3.create();

function solvePosition(constraint: SixDOFConstraint, bodies: Bodies, _deltaTime: number, baumgarte: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];
    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    let impulse = false;

    mat4.fromQuat(_setup_rotA, bodyA.quaternion);
    mat4.fromQuat(_setup_rotB, bodyB.quaternion);

    // rotation position solve
    if (isRotationFullyConstrained(constraint)) {
        // inverse of initial rotation from body 1 to body 2 in body 1 space
        // apply rotation limits min offset to constraint-to-body1
        const rotLimitsMinEuler = euler.set(
            _pos_rotLimitsMinEuler,
            constraint.limitMin[SixDOFAxis.ROTATION_X],
            constraint.limitMin[SixDOFAxis.ROTATION_Y],
            constraint.limitMin[SixDOFAxis.ROTATION_Z],
            'xyz',
        );
        const rotLimitsMin = quat.fromEuler(_solvePosition_rotLimitsMin, rotLimitsMinEuler);
        const constraintToBody1WithOffset = _solvePosition_constraintToBody1WithOffset;
        quat.multiply(constraintToBody1WithOffset, constraint.constraintToBody1, rotLimitsMin);

        quat.conjugate(_pos_invInitialOrientation, constraintToBody1WithOffset);
        quat.multiply(_pos_invInitialOrientation, constraint.constraintToBody2, _pos_invInitialOrientation);

        rotationEulerConstraintPart.calculateConstraintProperties(
            constraint.rotationConstraintPart,
            bodyA,
            _setup_rotA,
            bodyB,
            _setup_rotB,
        );
        impulse =
            rotationEulerConstraintPart.solvePositionConstraint(
                constraint.rotationConstraintPart,
                bodyA,
                bodyB,
                _pos_invInitialOrientation,
                baumgarte,
            ) || impulse;
    } else if (isRotationConstrained(constraint)) {
        quat.multiply(_pos_c1ToWorld, bodyA.quaternion, constraint.constraintToBody1);
        const c2ToWorld = _solvePosition_c2ToWorld;
        quat.multiply(c2ToWorld, bodyB.quaternion, constraint.constraintToBody2);
        quat.conjugate(_setup_qConj, _pos_c1ToWorld);
        quat.multiply(_pos_q, _setup_qConj, c2ToWorld);

        impulse =
            swingTwistConstraintPart.solvePositionConstraint(
                constraint.swingTwistConstraintPart,
                bodyA,
                bodyB,
                _pos_q,
                constraint.constraintToBody1,
                constraint.constraintToBody2,
                baumgarte,
            ) || impulse;
    }

    // translation position solve
    if (isTranslationFullyConstrained(constraint)) {
        // apply translation limits min offset to localSpacePosition1
        vec3.set(
            _pos_transLimitsMin,
            constraint.limitMin[SixDOFAxis.TRANSLATION_X],
            constraint.limitMin[SixDOFAxis.TRANSLATION_Y],
            constraint.limitMin[SixDOFAxis.TRANSLATION_Z],
        );
        vec3.transformQuat(_pos_localSpacePosition1WithOffset, _pos_transLimitsMin, constraint.constraintToBody1);
        vec3.add(_pos_localSpacePosition1WithOffset, constraint.localSpacePosition1, _pos_localSpacePosition1WithOffset);

        pointConstraintPart.calculateConstraintProperties(
            constraint.pointConstraintPart,
            bodyA,
            _setup_rotA,
            _pos_localSpacePosition1WithOffset,
            bodyB,
            _setup_rotB,
            constraint.localSpacePosition2,
        );
        impulse = pointConstraintPart.solvePositionConstraint(constraint.pointConstraintPart, bodyA, bodyB, baumgarte) || impulse;
    } else if (isTranslationConstrained(constraint)) {
        for (let i = 0; i < 3; i++) {
            if (constraint.limitsSpringSettings[i].frequencyOrStiffness <= 0) {
                getPositionConstraintProperties(constraint, bodies, _setup_r1PlusU, _setup_r2, _setup_u);
                quat.multiply(_pos_c1ToWorld, bodyA.quaternion, constraint.constraintToBody1);

                // rotate unit axis by constraint-to-world
                const unitAxis = vec3.set(_solvePosition_unitAxis, i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0);
                vec3.transformQuat(_pos_translationAxis, unitAxis, _pos_c1ToWorld);

                let error = 0;
                const axis = i;
                if (isFixedAxis(constraint, axis)) {
                    error = vec3.dot(_setup_u, _pos_translationAxis) - constraint.limitMin[axis];
                } else if (!isFreeAxis(constraint, axis)) {
                    const displacement = vec3.dot(_setup_u, _pos_translationAxis);
                    if (displacement <= constraint.limitMin[axis]) error = displacement - constraint.limitMin[axis];
                    else if (displacement >= constraint.limitMax[axis]) error = displacement - constraint.limitMax[axis];
                }

                if (error !== 0) {
                    // get inverse mass and inertia for bodies
                    const mpA = bodyA.motionProperties;
                    const mpB = bodyB.motionProperties;
                    const invMassA = bodyA.motionType === MotionType.DYNAMIC ? mpA.invMass : 0;
                    const invMassB = bodyB.motionType === MotionType.DYNAMIC ? mpB.invMass : 0;
                    const invInertiaA = _setup_invInertiaA;
                    const invInertiaB = _setup_invInertiaB;
                    mat4.identity(invInertiaA);
                    mat4.identity(invInertiaB);
                    if (bodyA.motionType === MotionType.DYNAMIC) {
                        mat4.fromQuat(_setup_rotA, bodyA.quaternion);
                        getInverseInertiaForRotation(invInertiaA, mpA, _setup_rotA);
                    }
                    if (bodyB.motionType === MotionType.DYNAMIC) {
                        mat4.fromQuat(_setup_rotB, bodyB.quaternion);
                        getInverseInertiaForRotation(invInertiaB, mpB, _setup_rotB);
                    }

                    axisConstraintPart.calculateConstraintProperties(
                        constraint.translationConstraintPart[i],
                        bodyA,
                        bodyB,
                        invMassA,
                        invMassB,
                        invInertiaA,
                        invInertiaB,
                        _setup_r1PlusU,
                        _setup_r2,
                        _pos_translationAxis,
                        0,
                    );
                    impulse =
                        axisConstraintPart.solvePositionConstraint(
                            constraint.translationConstraintPart[i],
                            bodyA,
                            bodyB,
                            _pos_translationAxis,
                            error,
                            baumgarte,
                        ) || impulse;
                }
            }
        }
    }

    return impulse;
}

/** Set translation limits */
export function setTranslationLimits(constraint: SixDOFConstraint, limitMin: Vec3, limitMax: Vec3): void {
    constraint.limitMin[0] = limitMin[0];
    constraint.limitMin[1] = limitMin[1];
    constraint.limitMin[2] = limitMin[2];
    constraint.limitMax[0] = limitMax[0];
    constraint.limitMax[1] = limitMax[1];
    constraint.limitMax[2] = limitMax[2];
    updateTranslationLimits(constraint);
    updateFixedFreeAxis(constraint);
}

/** Set rotation limits */
export function setRotationLimits(constraint: SixDOFConstraint, limitMin: Vec3, limitMax: Vec3): void {
    constraint.limitMin[3] = limitMin[0];
    constraint.limitMin[4] = limitMin[1];
    constraint.limitMin[5] = limitMin[2];
    constraint.limitMax[3] = limitMax[0];
    constraint.limitMax[4] = limitMax[1];
    constraint.limitMax[5] = limitMax[2];
    updateRotationLimits(constraint);
    updateFixedFreeAxis(constraint);
}

/** Set motor state for an axis */
export function setMotorState(constraint: SixDOFConstraint, axis: SixDOFAxis, state: MotorState): void {
    if (constraint.motorState[axis] !== state) {
        constraint.motorState[axis] = state;
        if (axis === SixDOFAxis.TRANSLATION_X || axis === SixDOFAxis.TRANSLATION_Y || axis === SixDOFAxis.TRANSLATION_Z) {
            axisConstraintPart.deactivate(constraint.motorTranslationConstraintPart[axis]);
            cacheTranslationMotorActive(constraint);
        } else {
            const rotIndex = axis - SixDOFAxis.ROTATION_X;
            angleConstraintPart.deactivate(constraint.motorRotationConstraintPart[rotIndex]);
            cacheRotationMotorActive(constraint);
            cacheRotationPositionMotorActive(constraint);
        }
    }
}

/** Set target velocity for translation motors */
export function setTargetVelocityCS(constraint: SixDOFConstraint, velocity: Vec3): void {
    vec3.copy(constraint.targetVelocity, velocity);
}

/** Set target angular velocity for rotation motors */
export function setTargetAngularVelocityCS(constraint: SixDOFConstraint, angularVelocity: Vec3): void {
    vec3.copy(constraint.targetAngularVelocity, angularVelocity);
}

/** Set target position for translation motors */
export function setTargetPositionCS(constraint: SixDOFConstraint, position: Vec3): void {
    vec3.copy(constraint.targetPosition, position);
}

/** Set target orientation for rotation motors */
export function setTargetOrientationCS(constraint: SixDOFConstraint, orientation: Quat): void {
    // Clamp to valid range
    getSwingTwist(orientation, _setup_qSwing, _setup_qTwist);
    swingTwistConstraintPart.clampSwingTwist(constraint.swingTwistConstraintPart, _setup_qSwing, _setup_qTwist);
    quat.multiply(constraint.targetOrientation, _setup_qSwing, _setup_qTwist);
}

/** Set max friction for an axis */
export function setMaxFriction(constraint: SixDOFConstraint, axis: number, friction: number): void {
    constraint.maxFriction[axis] = friction;
    if (axis < 3) cacheTranslationMotorActive(constraint);
    else cacheRotationMotorActive(constraint);
}

/** Make an axis free (unconstrained) */
export function makeFreeAxis(constraint: SixDOFConstraint, axis: number): void {
    constraint.limitMin[axis] = -Infinity;
    constraint.limitMax[axis] = Infinity;
    updateFixedFreeAxis(constraint);
}

/** Make an axis fixed (locked at 0) */
export function makeFixedAxis(constraint: SixDOFConstraint, axis: number): void {
    constraint.limitMin[axis] = Infinity;
    constraint.limitMax[axis] = -Infinity;
    updateFixedFreeAxis(constraint);
}

/** Set limits for an axis */
export function setLimitedAxis(constraint: SixDOFConstraint, axis: number, min: number, max: number): void {
    constraint.limitMin[axis] = min;
    constraint.limitMax[axis] = max;
    if (axis < 3) updateTranslationLimits(constraint);
    else updateRotationLimits(constraint);
    updateFixedFreeAxis(constraint);
}

/** the constraint definition for six-dof constraint */
export const def = /* @__PURE__ */ (() =>
    defineConstraint<SixDOFConstraint>({
        type: ConstraintType.SIX_DOF,
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
