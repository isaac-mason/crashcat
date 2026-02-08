import type { Quat, Vec3 } from 'mathcat';
import { mat3, mat4, quat, vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies';
import { type BodyId, getBodyIdIndex } from '../body/body-id';
import type { World } from '../world';
import {
    type ConstraintBase,
    ConstraintSpace,
    makeConstraintBase,
    removeConstraintIdFromBody,
} from './constraint-base';
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
import type { MotorSettings } from './constraint-part/motor-settings';
import * as motorSettings from './constraint-part/motor-settings';
import type { PointConstraintPart } from './constraint-part/point-constraint-part';
import * as pointConstraintPart from './constraint-part/point-constraint-part';
import type { SwingTwistConstraintPart } from './constraint-part/swing-twist-constraint-part';
import * as swingTwistConstraintPart from './constraint-part/swing-twist-constraint-part';
import { clampSwingTwist, getSwingTwist, SwingType } from './constraint-part/swing-twist-constraint-part';

/**
 * SwingTwistConstraint is a sophisticated constraint for humanoid ragdoll joints.
 * It allows limited rotation with separate swing and twist limits.
 *
 * - Swing: rotation around Y and Z axes, limited by cone or pyramid shape
 * - Twist: rotation around X axis (the twist axis), limited by min/max angles
 */
export type SwingTwistConstraint = ConstraintBase & {
    // local space configuration (stored relative to body COM)
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;

    /** transform from constraint space to body 1 space */
    constraintToBody1: Quat;
    /** transform from constraint space to body 2 space */
    constraintToBody2: Quat;

    // limits
    /** normal half cone angle (radians) - swing limit around Z axis */
    normalHalfConeAngle: number;
    /** plane half cone angle (radians) - swing limit around Y axis */
    planeHalfConeAngle: number;
    /** minimum twist angle (radians), in [-PI, PI] */
    twistMinAngle: number;
    /** maximum twist angle (radians), in [-PI, PI] */
    twistMaxAngle: number;

    // friction
    /** maximum friction torque (N*m) when not powered by motor */
    maxFrictionTorque: number;

    // motors
    /** swing motor state */
    swingMotorState: MotorState;
    /** twist motor state */
    twistMotorState: MotorState;
    /** target angular velocity in constraint space of body 2 */
    targetAngularVelocity: Vec3;
    /** target orientation in constraint space */
    targetOrientation: Quat;
    /** swing motor settings (spring + torque limits) */
    swingMotorSettings: MotorSettings;
    /** twist motor settings (spring + torque limits) */
    twistMotorSettings: MotorSettings;

    // runtime state
    /** world space motor axes (X, Y, Z) */
    worldSpaceMotorAxis: [Vec3, Vec3, Vec3];

    // constraint parts
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** swing-twist constraint part for rotation limits */
    swingTwistConstraintPart: SwingTwistConstraintPart;
    /** motor constraint parts (twist=0, swingY=1, swingZ=2) */
    motorConstraintParts: [AngleConstraintPart, AngleConstraintPart, AngleConstraintPart];
};

/** creates default swing twist constraint */
function makeSwingTwistConstraint(): SwingTwistConstraint {
    return {
        ...makeConstraintBase(),
        // local space
        localSpacePosition1: vec3.create(),
        localSpacePosition2: vec3.create(),
        constraintToBody1: quat.create(),
        constraintToBody2: quat.create(),
        // limits
        normalHalfConeAngle: 0,
        planeHalfConeAngle: 0,
        twistMinAngle: 0,
        twistMaxAngle: 0,
        // friction
        maxFrictionTorque: 0,
        // motors
        swingMotorState: MotorState.OFF,
        twistMotorState: MotorState.OFF,
        targetAngularVelocity: vec3.create(),
        targetOrientation: quat.create(),
        swingMotorSettings: motorSettings.create(),
        twistMotorSettings: motorSettings.create(),
        // runtime state
        worldSpaceMotorAxis: [vec3.create(), vec3.create(), vec3.create()],
        // constraint parts
        pointConstraintPart: pointConstraintPart.create(),
        swingTwistConstraintPart: swingTwistConstraintPart.create(),
        motorConstraintParts: [angleConstraintPart.create(), angleConstraintPart.create(), angleConstraintPart.create()],
    };
}

/** Settings for creating a swing-twist constraint */
export type SwingTwistConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    /** pivot point on body 1 */
    position1: Vec3;
    /** pivot point on body 2 */
    position2: Vec3;
    /** twist axis on body 1 (X axis of constraint space) */
    twistAxis1: Vec3;
    /** plane axis on body 1 (Z axis of constraint space, perpendicular to twist) */
    planeAxis1: Vec3;
    /** twist axis on body 2 */
    twistAxis2: Vec3;
    /** plane axis on body 2 */
    planeAxis2: Vec3;
    /** @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** swing limit type @default SwingType.Cone */
    swingType?: SwingType;
    /** normal half cone angle (radians) - swing limit around Z axis */
    normalHalfConeAngle?: number;
    /** plane half cone angle (radians) - swing limit around Y axis */
    planeHalfConeAngle?: number;
    /** minimum twist angle (radians) @default 0 */
    twistMinAngle?: number;
    /** maximum twist angle (radians) @default 0 */
    twistMaxAngle?: number;
    /** maximum friction torque @default 0 */
    maxFrictionTorque?: number;
    /** swing motor settings (spring + torque limits) */
    swingMotorSettings?: MotorSettings;
    /** twist motor settings (spring + torque limits) */
    twistMotorSettings?: MotorSettings;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

/** reset constraint state for pooling reuse - not exported */
function resetConstraint(constraint: SwingTwistConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
    swingTwistConstraintPart.deactivate(constraint.swingTwistConstraintPart);
    for (const part of constraint.motorConstraintParts) {
        angleConstraintPart.deactivate(part);
    }
}

const _create_normalAxis1 = /* @__PURE__ */ vec3.create();
const _create_normalAxis2 = /* @__PURE__ */ vec3.create();
const _create_c_to_b1 = /* @__PURE__ */ mat3.create();
const _create_c_to_b2 = /* @__PURE__ */ mat3.create();

/** Create a swing-twist constraint */
export function create(world: World, settings: SwingTwistConstraintSettings): SwingTwistConstraint {
    const swingTwistConstraints = world.constraints.swingTwistConstraints;
    const bodies = world.bodies;

    // get next sequence
    const sequence = swingTwistConstraints.nextSequence;
    swingTwistConstraints.nextSequence = (swingTwistConstraints.nextSequence + 1) & SEQUENCE_MASK;

    // get constraint from pool
    let index: number;
    let constraint: SwingTwistConstraint;
    if (swingTwistConstraints.freeIndices.length > 0) {
        index = swingTwistConstraints.freeIndices.pop()!;
        constraint = swingTwistConstraints.constraints[index];
    } else {
        index = swingTwistConstraints.constraints.length;
        constraint = makeSwingTwistConstraint();
        swingTwistConstraints.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    // set constraint id, index, sequence
    constraint.id = serConstraintId(index, ConstraintType.SWING_TWIST, sequence);
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
    const twistAxis1 = vec3.create();
    const twistAxis2 = vec3.create();
    const planeAxis1 = vec3.create();
    const planeAxis2 = vec3.create();
    vec3.normalize(twistAxis1, settings.twistAxis1);
    vec3.normalize(twistAxis2, settings.twistAxis2);
    vec3.normalize(planeAxis1, settings.planeAxis1);
    vec3.normalize(planeAxis2, settings.planeAxis2);

    // calculate normal axes (perpendicular to twist and plane)
    vec3.cross(_create_normalAxis1, planeAxis1, twistAxis1);
    vec3.cross(_create_normalAxis2, planeAxis2, twistAxis2);

    // build rotation matrices from constraint space to body space
    // constraint space: X = twist axis, Y = normal axis, Z = plane axis
    mat3.set(
        _create_c_to_b1,
        twistAxis1[0],
        twistAxis1[1],
        twistAxis1[2],
        _create_normalAxis1[0],
        _create_normalAxis1[1],
        _create_normalAxis1[2],
        planeAxis1[0],
        planeAxis1[1],
        planeAxis1[2],
    );
    quat.fromMat3(constraint.constraintToBody1, _create_c_to_b1);

    mat3.set(
        _create_c_to_b2,
        twistAxis2[0],
        twistAxis2[1],
        twistAxis2[2],
        _create_normalAxis2[0],
        _create_normalAxis2[1],
        _create_normalAxis2[2],
        planeAxis2[0],
        planeAxis2[1],
        planeAxis2[2],
    );
    quat.fromMat3(constraint.constraintToBody2, _create_c_to_b2);

    // convert to local space if needed
    const space = settings.space ?? ConstraintSpace.WORLD;
    if (space === ConstraintSpace.WORLD) {
        if (bodyA && !bodyA._pooled && bodyB && !bodyB._pooled) {
            const invQuatA = quat.create();
            const invQuatB = quat.create();
            quat.conjugate(invQuatA, bodyA.quaternion);
            quat.conjugate(invQuatB, bodyB.quaternion);

            // transform positions to local space
            vec3.subtract(constraint.localSpacePosition1, settings.position1, bodyA.centerOfMassPosition);
            vec3.transformQuat(constraint.localSpacePosition1, constraint.localSpacePosition1, invQuatA);

            vec3.subtract(constraint.localSpacePosition2, settings.position2, bodyB.centerOfMassPosition);
            vec3.transformQuat(constraint.localSpacePosition2, constraint.localSpacePosition2, invQuatB);

            // transform constraint-to-body rotations by body rotations
            quat.multiply(constraint.constraintToBody1, invQuatA, constraint.constraintToBody1);
            quat.multiply(constraint.constraintToBody2, invQuatB, constraint.constraintToBody2);
        }
    } else {
        // already in local space
        vec3.copy(constraint.localSpacePosition1, settings.position1);
        vec3.copy(constraint.localSpacePosition2, settings.position2);
    }

    // set limits
    constraint.normalHalfConeAngle = settings.normalHalfConeAngle ?? 0;
    constraint.planeHalfConeAngle = settings.planeHalfConeAngle ?? 0;
    constraint.twistMinAngle = settings.twistMinAngle ?? 0;
    constraint.twistMaxAngle = settings.twistMaxAngle ?? 0;

    // configure swing-twist constraint part
    constraint.swingTwistConstraintPart.swingType = settings.swingType ?? SwingType.CONE;
    updateLimits(constraint);

    // set friction
    constraint.maxFrictionTorque = settings.maxFrictionTorque ?? 0;

    // set motor defaults
    constraint.swingMotorState = MotorState.OFF;
    constraint.twistMotorState = MotorState.OFF;
    vec3.zero(constraint.targetAngularVelocity);
    quat.identity(constraint.targetOrientation);

    // reset motor settings to defaults
    motorSettings.reset(constraint.swingMotorSettings);
    motorSettings.reset(constraint.twistMotorSettings);

    // copy motor settings from input if provided
    if (settings.swingMotorSettings) {
        motorSettings.copy(constraint.swingMotorSettings, settings.swingMotorSettings);
    }
    if (settings.twistMotorSettings) {
        motorSettings.copy(constraint.twistMotorSettings, settings.twistMotorSettings);
    }

    // track constraint on both bodies
    bodyA.constraintIds.push(constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB) {
        bodyB.constraintIds.push(constraint.id);
    }

    return constraint;
}

/** Update the limits in the swing-twist constraint part */
function updateLimits(constraint: SwingTwistConstraint): void {
    swingTwistConstraintPart.setLimits(
        constraint.swingTwistConstraintPart,
        constraint.twistMinAngle,
        constraint.twistMaxAngle,
        -constraint.planeHalfConeAngle,
        constraint.planeHalfConeAngle,
        -constraint.normalHalfConeAngle,
        constraint.normalHalfConeAngle,
    );
}

/** Remove a swing-twist constraint */
export function remove(world: World, constraint: SwingTwistConstraint): void {
    const swingTwistConstraints = world.constraints.swingTwistConstraints;
    const bodies = world.bodies;

    // remove from bodies' constraintIds arrays
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
    swingTwistConstraints.freeIndices.push(constraint.index);
}

/** Get swing-twist constraint by id */
export function get(world: World, id: ConstraintId): SwingTwistConstraint | undefined {
    const swingTwistConstraints = world.constraints.swingTwistConstraints;
    const index = getConstraintIdIndex(id);
    const constraint = swingTwistConstraints.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

/** Set normal half cone angle (swing limit around Z axis) */
export function setNormalHalfConeAngle(constraint: SwingTwistConstraint, angle: number): void {
    constraint.normalHalfConeAngle = angle;
    updateLimits(constraint);
}

/** Set plane half cone angle (swing limit around Y axis) */
export function setPlaneHalfConeAngle(constraint: SwingTwistConstraint, angle: number): void {
    constraint.planeHalfConeAngle = angle;
    updateLimits(constraint);
}

/** Set twist min angle */
export function setTwistMinAngle(constraint: SwingTwistConstraint, angle: number): void {
    constraint.twistMinAngle = angle;
    updateLimits(constraint);
}

/** Set twist max angle */
export function setTwistMaxAngle(constraint: SwingTwistConstraint, angle: number): void {
    constraint.twistMaxAngle = angle;
    updateLimits(constraint);
}

/** Set maximum friction torque */
export function setMaxFrictionTorque(constraint: SwingTwistConstraint, torque: number): void {
    constraint.maxFrictionTorque = torque;
}

/** Set swing motor state */
export function setSwingMotorState(constraint: SwingTwistConstraint, state: MotorState): void {
    if (constraint.swingMotorState !== state) {
        constraint.swingMotorState = state;
        // reset motor parts to avoid warm start issues
        angleConstraintPart.deactivate(constraint.motorConstraintParts[0]);
        angleConstraintPart.deactivate(constraint.motorConstraintParts[1]);
        angleConstraintPart.deactivate(constraint.motorConstraintParts[2]);
    }
}

/** Set twist motor state */
export function setTwistMotorState(constraint: SwingTwistConstraint, state: MotorState): void {
    if (constraint.twistMotorState !== state) {
        constraint.twistMotorState = state;
        // reset motor part to avoid warm start issues
        angleConstraintPart.deactivate(constraint.motorConstraintParts[0]);
    }
}

/** Set target angular velocity in constraint space of body 2 */
export function setTargetAngularVelocityCS(constraint: SwingTwistConstraint, velocity: Vec3): void {
    vec3.copy(constraint.targetAngularVelocity, velocity);
}

const _target_q_swing = /* @__PURE__ */ quat.create();
const _target_q_twist = /* @__PURE__ */ quat.create();

/** Set target orientation in constraint space */
export function setTargetOrientationCS(constraint: SwingTwistConstraint, orientation: Quat): void {
    // clamp to valid range
    getSwingTwist(orientation, _target_q_swing, _target_q_twist);
    const clampedAxis = clampSwingTwist(constraint.swingTwistConstraintPart, _target_q_swing, _target_q_twist);

    if (clampedAxis !== 0) {
        quat.multiply(constraint.targetOrientation, _target_q_swing, _target_q_twist);
    } else {
        quat.copy(constraint.targetOrientation, orientation);
    }
}

const _setTargetOrientationBS_temp = /* @__PURE__ */ quat.create();
const _setTargetOrientationBS_c1Conj = /* @__PURE__ */ quat.create();

/** Set target orientation in body space (R2 = R1 * inOrientation) */
export function setTargetOrientationBS(constraint: SwingTwistConstraint, orientation: Quat): void {
    // convert from body space to constraint space
    quat.conjugate(_setTargetOrientationBS_c1Conj, constraint.constraintToBody1);
    quat.multiply(_setTargetOrientationBS_temp, _setTargetOrientationBS_c1Conj, orientation);
    quat.multiply(_setTargetOrientationBS_temp, _setTargetOrientationBS_temp, constraint.constraintToBody2);
    setTargetOrientationCS(constraint, _setTargetOrientationBS_temp);
}

const _getRotationInConstraintSpace_q1 = /* @__PURE__ */ quat.create();
const _getRotationInConstraintSpace_q2 = /* @__PURE__ */ quat.create();

/** Get current rotation of constraint in constraint space */
export function getRotationInConstraintSpace(out: Quat, constraint: SwingTwistConstraint, bodies: Bodies): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    // q = (q1 * c1)^-1 * (q2 * c2)
    quat.multiply(_getRotationInConstraintSpace_q1, bodyA.quaternion, constraint.constraintToBody1);
    quat.multiply(_getRotationInConstraintSpace_q2, bodyB.quaternion, constraint.constraintToBody2);
    quat.conjugate(out, _getRotationInConstraintSpace_q1);
    quat.multiply(out, out, _getRotationInConstraintSpace_q2);
}

const _setup_rotA = /* @__PURE__ */ mat4.create();
const _setup_rotB = /* @__PURE__ */ mat4.create();
const _setup_wsAxis = /* @__PURE__ */ mat4.create();
const _setup_rotationError = /* @__PURE__ */ vec3.create();
const _setup_constraintBody1ToWorld = /* @__PURE__ */ quat.create();
const _setup_constraintBody2ToWorld = /* @__PURE__ */ quat.create();
const _setup_q = /* @__PURE__ */ quat.create();
const _setup_c1Conj = /* @__PURE__ */ quat.create();

/**
 * Setup velocity constraint for swing-twist constraint.
 * Called once per frame before velocity iterations.
 */
export function setupVelocity(constraint: SwingTwistConstraint, bodies: Bodies, deltaTime: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // convert quaternions to rotation matrices
    mat4.fromQuat(_setup_rotA, bodyA.quaternion);
    mat4.fromQuat(_setup_rotB, bodyB.quaternion);

    // setup point constraint
    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _setup_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _setup_rotB,
        constraint.localSpacePosition2,
    );

    // calculate constraint rotation: q = (q1 * c1)^-1 * (q2 * c2)
    quat.multiply(_setup_constraintBody1ToWorld, bodyA.quaternion, constraint.constraintToBody1);
    quat.multiply(_setup_constraintBody2ToWorld, bodyB.quaternion, constraint.constraintToBody2);

    quat.conjugate(_setup_c1Conj, _setup_constraintBody1ToWorld);
    quat.multiply(_setup_q, _setup_c1Conj, _setup_constraintBody2ToWorld);

    // setup swing-twist constraint part
    swingTwistConstraintPart.calculateConstraintProperties(
        constraint.swingTwistConstraintPart,
        bodyA,
        bodyB,
        _setup_q,
        _setup_constraintBody1ToWorld,
    );

    // setup motors and friction
    if (
        constraint.swingMotorState !== MotorState.OFF ||
        constraint.twistMotorState !== MotorState.OFF ||
        constraint.maxFrictionTorque > 0
    ) {
        // calculate world space motor axes from constraint space of body 2
        mat4.fromQuat(_setup_wsAxis, _setup_constraintBody2ToWorld);
        for (let i = 0; i < 3; i++) {
            vec3.set(
                constraint.worldSpaceMotorAxis[i],
                _setup_wsAxis[i * 4 + 0],
                _setup_wsAxis[i * 4 + 1],
                _setup_wsAxis[i * 4 + 2],
            );
        }

        // calculate rotation error for position motors
        if (constraint.swingMotorState === MotorState.POSITION || constraint.twistMotorState === MotorState.POSITION) {
            // get target orientation along shortest path from q
            const targetOrientation = quat.clone(constraint.targetOrientation);
            if (quat.dot(_setup_q, targetOrientation) < 0) {
                quat.scale(targetOrientation, targetOrientation, -1);
            }

            // diff = q^-1 * targetOrientation
            const qConj = quat.create();
            quat.conjugate(qConj, _setup_q);
            const diff = quat.create();
            quat.multiply(diff, qConj, targetOrientation);

            // approximate error angles (small angle approximation)
            vec3.set(_setup_rotationError, -2 * diff[0], -2 * diff[1], -2 * diff[2]);
        }

        // swing motor
        switch (constraint.swingMotorState) {
            case MotorState.OFF:
                if (constraint.maxFrictionTorque > 0) {
                    // enable friction
                    for (let i = 1; i < 3; i++) {
                        angleConstraintPart.calculateConstraintProperties(
                            constraint.motorConstraintParts[i],
                            bodyA,
                            bodyB,
                            constraint.worldSpaceMotorAxis[i],
                            0,
                        );
                    }
                } else {
                    // disable friction - deactivate all motor parts
                    for (let i = 0; i < 3; i++) {
                        angleConstraintPart.deactivate(constraint.motorConstraintParts[i]);
                    }
                }
                break;

            case MotorState.VELOCITY:
                for (let i = 1; i < 3; i++) {
                    angleConstraintPart.calculateConstraintProperties(
                        constraint.motorConstraintParts[i],
                        bodyA,
                        bodyB,
                        constraint.worldSpaceMotorAxis[i],
                        -constraint.targetAngularVelocity[i],
                    );
                }
                break;

            case MotorState.POSITION:
                if (constraint.swingMotorSettings.springSettings.frequencyOrStiffness > 0) {
                    for (let i = 1; i < 3; i++) {
                        angleConstraintPart.calculateConstraintPropertiesWithSettings(
                            constraint.motorConstraintParts[i],
                            deltaTime,
                            bodyA,
                            bodyB,
                            constraint.worldSpaceMotorAxis[i],
                            0,
                            _setup_rotationError[i],
                            constraint.swingMotorSettings.springSettings,
                        );
                    }
                } else {
                    for (let i = 1; i < 3; i++) {
                        angleConstraintPart.deactivate(constraint.motorConstraintParts[i]);
                    }
                }
                break;
        }

        // twist motor
        switch (constraint.twistMotorState) {
            case MotorState.OFF:
                if (constraint.maxFrictionTorque > 0) {
                    angleConstraintPart.calculateConstraintProperties(
                        constraint.motorConstraintParts[0],
                        bodyA,
                        bodyB,
                        constraint.worldSpaceMotorAxis[0],
                        0,
                    );
                } else {
                    angleConstraintPart.deactivate(constraint.motorConstraintParts[0]);
                }
                break;

            case MotorState.VELOCITY:
                angleConstraintPart.calculateConstraintProperties(
                    constraint.motorConstraintParts[0],
                    bodyA,
                    bodyB,
                    constraint.worldSpaceMotorAxis[0],
                    -constraint.targetAngularVelocity[0],
                );
                break;

            case MotorState.POSITION:
                if (constraint.twistMotorSettings.springSettings.frequencyOrStiffness > 0) {
                    angleConstraintPart.calculateConstraintPropertiesWithSettings(
                        constraint.motorConstraintParts[0],
                        deltaTime,
                        bodyA,
                        bodyB,
                        constraint.worldSpaceMotorAxis[0],
                        0,
                        _setup_rotationError[0],
                        constraint.twistMotorSettings.springSettings,
                    );
                } else {
                    angleConstraintPart.deactivate(constraint.motorConstraintParts[0]);
                }
                break;
        }
    } else {
        // disable all motors
        for (const part of constraint.motorConstraintParts) {
            angleConstraintPart.deactivate(part);
        }
    }
}

/**
 * Warm start velocity constraint for swing-twist constraint.
 */
export function warmStartVelocity(constraint: SwingTwistConstraint, bodies: Bodies, warmStartImpulseRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // warm start all constraint parts
    for (const part of constraint.motorConstraintParts) {
        angleConstraintPart.warmStart(part, bodyA, bodyB, warmStartImpulseRatio);
    }
    swingTwistConstraintPart.warmStart(constraint.swingTwistConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    pointConstraintPart.warmStart(constraint.pointConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
}

/**
 * Solve velocity constraint for swing-twist constraint.
 * @returns True if any impulse was applied
 */
export function solveVelocity(constraint: SwingTwistConstraint, bodies: Bodies, deltaTime: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    let impulse = false;

    // solve twist motor
    if (angleConstraintPart.isActive(constraint.motorConstraintParts[0])) {
        let minLimit: number;
        let maxLimit: number;
        if (constraint.twistMotorState === MotorState.OFF) {
            maxLimit = deltaTime * constraint.maxFrictionTorque;
            minLimit = -maxLimit;
        } else {
            minLimit = deltaTime * constraint.twistMotorSettings.minTorqueLimit;
            maxLimit = deltaTime * constraint.twistMotorSettings.maxTorqueLimit;
        }
        impulse =
            angleConstraintPart.solveVelocityConstraint(
                constraint.motorConstraintParts[0],
                bodyA,
                bodyB,
                constraint.worldSpaceMotorAxis[0],
                minLimit,
                maxLimit,
            ) || impulse;
    }

    // solve swing motors
    if (angleConstraintPart.isActive(constraint.motorConstraintParts[1])) {
        let minLimit: number;
        let maxLimit: number;
        if (constraint.swingMotorState === MotorState.OFF) {
            maxLimit = deltaTime * constraint.maxFrictionTorque;
            minLimit = -maxLimit;
        } else {
            minLimit = deltaTime * constraint.swingMotorSettings.minTorqueLimit;
            maxLimit = deltaTime * constraint.swingMotorSettings.maxTorqueLimit;
        }
        for (let i = 1; i < 3; i++) {
            impulse =
                angleConstraintPart.solveVelocityConstraint(
                    constraint.motorConstraintParts[i],
                    bodyA,
                    bodyB,
                    constraint.worldSpaceMotorAxis[i],
                    minLimit,
                    maxLimit,
                ) || impulse;
        }
    }

    // solve swing-twist limits
    impulse = swingTwistConstraintPart.solveVelocityConstraint(constraint.swingTwistConstraintPart, bodyA, bodyB) || impulse;

    // solve point constraint
    impulse = pointConstraintPart.solveVelocityConstraint(constraint.pointConstraintPart, bodyA, bodyB) || impulse;

    return impulse;
}

const _solvePosition_rotA = /* @__PURE__ */ mat4.create();
const _solvePosition_rotB = /* @__PURE__ */ mat4.create();
const _solvePosition_q = /* @__PURE__ */ quat.create();
const _solvePosition_c1ToWorld = /* @__PURE__ */ quat.create();
const _solvePosition_c2ToWorld = /* @__PURE__ */ quat.create();
const _solvePosition_c1Conj = /* @__PURE__ */ quat.create();

/**
 * Solve position constraint for swing-twist constraint.
 * @returns True if any correction was applied
 */
export function solvePosition(
    constraint: SwingTwistConstraint,
    bodies: Bodies,
    _deltaTime: number,
    baumgarteFactor: number,
): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    let impulse = false;

    // solve rotation violations
    quat.multiply(_solvePosition_c1ToWorld, bodyA.quaternion, constraint.constraintToBody1);
    quat.multiply(_solvePosition_c2ToWorld, bodyB.quaternion, constraint.constraintToBody2);
    quat.conjugate(_solvePosition_c1Conj, _solvePosition_c1ToWorld);
    quat.multiply(_solvePosition_q, _solvePosition_c1Conj, _solvePosition_c2ToWorld);

    impulse =
        swingTwistConstraintPart.solvePositionConstraint(
            constraint.swingTwistConstraintPart,
            bodyA,
            bodyB,
            _solvePosition_q,
            constraint.constraintToBody1,
            constraint.constraintToBody2,
            baumgarteFactor,
        ) || impulse;

    // solve position violations
    mat4.fromQuat(_solvePosition_rotA, bodyA.quaternion);
    mat4.fromQuat(_solvePosition_rotB, bodyB.quaternion);

    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _solvePosition_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _solvePosition_rotB,
        constraint.localSpacePosition2,
    );

    impulse =
        pointConstraintPart.solvePositionConstraint(constraint.pointConstraintPart, bodyA, bodyB, baumgarteFactor) || impulse;

    return impulse;
}

/** Reset warm start for swing-twist constraint */
export function resetWarmStart(constraint: SwingTwistConstraint): void {
    for (const part of constraint.motorConstraintParts) {
        angleConstraintPart.deactivate(part);
    }
    swingTwistConstraintPart.deactivate(constraint.swingTwistConstraintPart);
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
}

/** Get total lambda for position constraint */
export function getTotalLambdaPosition(out: Vec3, constraint: SwingTwistConstraint): Vec3 {
    out[0] = constraint.pointConstraintPart.totalLambda[0];
    out[1] = constraint.pointConstraintPart.totalLambda[1];
    out[2] = constraint.pointConstraintPart.totalLambda[2];
    return out;
}

/** Get total lambda for twist limit */
export function getTotalLambdaTwist(constraint: SwingTwistConstraint): number {
    return swingTwistConstraintPart.getTotalTwistLambda(constraint.swingTwistConstraintPart);
}

/** Get total lambda for swing Y limit */
export function getTotalLambdaSwingY(constraint: SwingTwistConstraint): number {
    return swingTwistConstraintPart.getTotalSwingYLambda(constraint.swingTwistConstraintPart);
}

/** Get total lambda for swing Z limit */
export function getTotalLambdaSwingZ(constraint: SwingTwistConstraint): number {
    return swingTwistConstraintPart.getTotalSwingZLambda(constraint.swingTwistConstraintPart);
}

/** Get total lambda for motors */
export function getTotalLambdaMotor(out: Vec3, constraint: SwingTwistConstraint): Vec3 {
    out[0] = constraint.motorConstraintParts[0].totalLambda;
    out[1] = constraint.motorConstraintParts[1].totalLambda;
    out[2] = constraint.motorConstraintParts[2].totalLambda;
    return out;
}
