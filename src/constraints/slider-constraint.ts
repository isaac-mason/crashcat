import type { Mat4, Quat, Vec2, Vec3 } from 'mathcat';
import { mat3, mat4, quat, vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies';
import { type BodyId, getBodyIdIndex } from '../body/body-id';
import { getInverseInertiaForRotation } from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
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
import type { AxisConstraintPart } from './constraint-part/axis-constraint-part';
import * as axisConstraintPart from './constraint-part/axis-constraint-part';
import type { DualAxisConstraintPart } from './constraint-part/dual-axis-constraint-part';
import * as dualAxisConstraintPart from './constraint-part/dual-axis-constraint-part';
import type { MotorSettings } from './constraint-part/motor-settings';
import * as motorSettings from './constraint-part/motor-settings';
import { MotorState } from './constraint-part/motor-settings';
import type { RotationEulerConstraintPart } from './constraint-part/rotation-euler-constraint-part';
import * as rotationEulerConstraintPart from './constraint-part/rotation-euler-constraint-part';
import type { SpringSettings } from './constraint-part/spring-settings';
import * as springSettings from './constraint-part/spring-settings';

/**
 * Slider constraint (prismatic) removes 5 DOF (2 translation perpendicular to slider + 3 rotation).
 * Allows movement only along a single axis, like a piston or rail.
 */
export type SliderConstraint = ConstraintBase & {
    // local space configuration (stored relative to body COM)
    /** attachment point on body a in local space */
    localSpacePositionA: Vec3;
    /** attachment point on body b in local space */
    localSpacePositionB: Vec3;
    /** slider axis on body a in local space (normalized) - direction of allowed movement */
    localSpaceSliderAxisA: Vec3;
    /** normal axis perpendicular to slider on body a (for frame definition) */
    localSpaceNormalA: Vec3;
    /** second normal axis (slider × normal1) */
    localSpaceNormalB: Vec3;

    // rotation tracking
    /** inverse of initial relative orientation, for rotation constraint */
    invInitialOrientation: Quat;

    // runtime cached values
    /** r1 = rotation1 × localSpacePosition1 (world space offset from COM to attachment) */
    r1: Vec3;
    /** r2 = rotation2 × localSpacePosition2 (world space offset from COM to attachment) */
    r2: Vec3;
    /** u = x2 + r2 - x1 - r1 = p2 - p1 (separation vector in world space) */
    u: Vec3;
    /** world space slider axis */
    worldSpaceSliderAxis: Vec3;
    /** world space normal axis 1 for position constraint */
    n1: Vec3;
    /** world space normal axis 2 for position constraint */
    n2: Vec3;
    /** current distance along slider axis (d = u · sliderAxis) */
    d: number;

    // limits
    /** whether position limits are enabled */
    hasLimits: boolean;
    /** minimum position limit (≤ 0) */
    limitsMin: number;
    /** maximum position limit (≥ 0) */
    limitsMax: number;
    /** spring settings for soft limits */
    limitsSpringSettings: SpringSettings;

    // motor
    /** motor state */
    motorState: MotorState;
    /** target velocity (m/s) for velocity mode */
    targetVelocity: number;
    /** target position for position mode */
    targetPosition: number;
    /** maximum friction force when motor is off */
    maxFrictionForce: number;
    /** motor settings (spring + force limits) */
    motorSettings: MotorSettings;

    // parts
    /** position constraint for 2 DOF perpendicular to slider */
    positionConstraintPart: DualAxisConstraintPart;
    /** rotation constraint for 3 DOF (locks all rotation) */
    rotationConstraintPart: RotationEulerConstraintPart;
    /** position limits constraint along slider axis */
    positionLimitsConstraintPart: AxisConstraintPart;
    /** motor constraint along slider axis */
    motorConstraintPart: AxisConstraintPart;
};

/** creates default slider constraint */
function makeSliderConstraint(): SliderConstraint {
    return {
        ...makeConstraintBase(),
        // local space
        localSpacePositionA: vec3.create(),
        localSpacePositionB: vec3.create(),
        localSpaceSliderAxisA: vec3.fromValues(1, 0, 0),
        localSpaceNormalA: vec3.fromValues(0, 1, 0),
        localSpaceNormalB: vec3.fromValues(0, 0, 1),
        // rotation tracking
        invInitialOrientation: quat.create(),
        // runtime cached
        r1: vec3.create(),
        r2: vec3.create(),
        u: vec3.create(),
        worldSpaceSliderAxis: vec3.create(),
        n1: vec3.create(),
        n2: vec3.create(),
        d: 0,
        // limits
        hasLimits: false,
        limitsMin: -Infinity,
        limitsMax: Infinity,
        limitsSpringSettings: springSettings.create(),
        // motor
        motorState: MotorState.OFF,
        targetVelocity: 0,
        targetPosition: 0,
        maxFrictionForce: 0,
        motorSettings: motorSettings.create(),
        // constraint parts
        positionConstraintPart: dualAxisConstraintPart.create(),
        rotationConstraintPart: rotationEulerConstraintPart.create(),
        positionLimitsConstraintPart: axisConstraintPart.create(),
        motorConstraintPart: axisConstraintPart.create(),
    };
}

/** reset constraint state for pooling reuse */
function resetConstraint(constraint: SliderConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    dualAxisConstraintPart.deactivate(constraint.positionConstraintPart);
    rotationEulerConstraintPart.deactivate(constraint.rotationConstraintPart);
    axisConstraintPart.deactivate(constraint.positionLimitsConstraintPart);
    axisConstraintPart.deactivate(constraint.motorConstraintPart);
}

/** Settings for creating a slider constraint */
export type SliderConstraintSettings = {
    /** body a id */
    bodyIdA: BodyId;
    /** body b id */
    bodyIdB: BodyId;
    /** pivot point on body a */
    pointA?: Vec3;
    /** pivot point on body b */
    pointB?: Vec3;
    /** slider axis (direction of allowed movement) on body a (will be normalized) */
    sliderAxisA: Vec3;
    /** slider axis on body b (will be normalized) */
    sliderAxisB: Vec3;
    /** normal axis perpendicular to slider on body a (for frame definition) */
    normalAxisA: Vec3;
    /** normal axis perpendicular to slider on body b */
    normalAxisB: Vec3;
    /** @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** automatically detect anchor point based on body positions */
    autoDetectPoint?: boolean;
    // limits
    /** minimum position limit (default: -Infinity, must be ≤ 0) */
    limitsMin?: number;
    /** maximum position limit (default: Infinity, must be ≥ 0) */
    limitsMax?: number;
    /** spring settings for soft limits */
    limitsSpringSettings?: SpringSettings;
    // motor
    /** maximum friction force when motor is off */
    maxFrictionForce?: number;
    /** motor settings (spring + force limits) */
    motorSettings?: MotorSettings;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

const _getInvInitialOrientationXY_z1 = /* @__PURE__ */ vec3.create();
const _getInvInitialOrientationXY_z2 = /* @__PURE__ */ vec3.create();
const _getInvInitialOrientationXY_mat1 = /* @__PURE__ */ mat3.create();
const _getInvInitialOrientationXY_mat2 = /* @__PURE__ */ mat3.create();
const _getInvInitialOrientationXY_q1 = /* @__PURE__ */ quat.create();
const _getInvInitialOrientationXY_q2 = /* @__PURE__ */ quat.create();
const _getInvInitialOrientationXY_q1Conj = /* @__PURE__ */ quat.create();

/** calculate the inverse initial orientation for rotation tracking */
function getInvInitialOrientationXY(out: Quat, sliderAxis1: Vec3, normalAxis1: Vec3, sliderAxis2: Vec3, normalAxis2: Vec3): Quat {
    const eps = 1e-6;

    // if axes are identical, return identity
    if (
        Math.abs(sliderAxis1[0] - sliderAxis2[0]) < eps &&
        Math.abs(sliderAxis1[1] - sliderAxis2[1]) < eps &&
        Math.abs(sliderAxis1[2] - sliderAxis2[2]) < eps &&
        Math.abs(normalAxis1[0] - normalAxis2[0]) < eps &&
        Math.abs(normalAxis1[1] - normalAxis2[1]) < eps &&
        Math.abs(normalAxis1[2] - normalAxis2[2]) < eps
    ) {
        return quat.identity(out);
    }

    // build 3x3 rotation matrices from axes
    // z axis is slider × normal
    vec3.cross(_getInvInitialOrientationXY_z1, sliderAxis1, normalAxis1);
    vec3.cross(_getInvInitialOrientationXY_z2, sliderAxis2, normalAxis2);

    // create rotation matrices
    // biome-ignore format: readability
    mat3.set(_getInvInitialOrientationXY_mat1,
        sliderAxis1[0], sliderAxis1[1], sliderAxis1[2], // column 0 (X = slider)
        normalAxis1[0], normalAxis1[1], normalAxis1[2], // column 1 (Y = normal)
        _getInvInitialOrientationXY_z1[0], _getInvInitialOrientationXY_z1[1], _getInvInitialOrientationXY_z1[2], // column 2 (Z = slider × normal)
    );

    // biome-ignore format: readability
    mat3.set(_getInvInitialOrientationXY_mat2,
        sliderAxis2[0], sliderAxis2[1], sliderAxis2[2], // column 0
        normalAxis2[0], normalAxis2[1], normalAxis2[2], // column 1
        _getInvInitialOrientationXY_z2[0], _getInvInitialOrientationXY_z2[1], _getInvInitialOrientationXY_z2[2], // column 2
    );

    // convert to quaternions
    quat.fromMat3(_getInvInitialOrientationXY_q1, _getInvInitialOrientationXY_mat1);
    quat.fromMat3(_getInvInitialOrientationXY_q2, _getInvInitialOrientationXY_mat2);

    // return q2 * conjugate(q1)
    quat.conjugate(_getInvInitialOrientationXY_q1Conj, _getInvInitialOrientationXY_q1);

    quat.multiply(out, _getInvInitialOrientationXY_q2, _getInvInitialOrientationXY_q1Conj);
    return out;
}

const V0 = /* @__PURE__ */ vec3.create();

const _create_sliderAxis1 = /* @__PURE__ */ vec3.create();
const _create_sliderAxis2 = /* @__PURE__ */ vec3.create();
const _create_normalAxis1 = /* @__PURE__ */ vec3.create();
const _create_normalAxis2 = /* @__PURE__ */ vec3.create();
const _create_invQuatA = /* @__PURE__ */ quat.create();
const _create_invQuatB = /* @__PURE__ */ quat.create();
const _create_anchor = /* @__PURE__ */ vec3.create();

/** Create a slider constraint */
export function create(world: World, settings: SliderConstraintSettings): SliderConstraint {
    const sliderConstraints = world.constraints.sliderConstraints;
    const bodies = world.bodies;

    // get next sequence
    const sequence = sliderConstraints.nextSequence;
    sliderConstraints.nextSequence = (sliderConstraints.nextSequence + 1) & SEQUENCE_MASK;

    // get constraint from pool
    let index: number;
    let constraint: SliderConstraint;
    if (sliderConstraints.freeIndices.length > 0) {
        index = sliderConstraints.freeIndices.pop()!;
        constraint = sliderConstraints.constraints[index];
    } else {
        index = sliderConstraints.constraints.length;
        constraint = makeSliderConstraint();
        sliderConstraints.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    // set constraint id, index, sequence
    constraint.id = serConstraintId(index, ConstraintType.SLIDER, sequence);
    constraint.index = index;
    constraint.sequence = sequence;

    // set base constraint properties
    constraint.constraintPriority = settings.constraintPriority ?? 0;
    constraint.numVelocityStepsOverride = settings.numVelocityStepsOverride ?? 0;
    constraint.numPositionStepsOverride = settings.numPositionStepsOverride ?? 0;

    // set body indices
    constraint.bodyIndexA = getBodyIdIndex(settings.bodyIdA);
    constraint.bodyIndexB = getBodyIdIndex(settings.bodyIdB);

    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    // normalize axes
    vec3.normalize(_create_sliderAxis1, settings.sliderAxisA);
    vec3.normalize(_create_sliderAxis2, settings.sliderAxisB);
    vec3.normalize(_create_normalAxis1, settings.normalAxisA);
    vec3.normalize(_create_normalAxis2, settings.normalAxisB);

    // calculate invInitialOrientation from world space axes
    getInvInitialOrientationXY(
        constraint.invInitialOrientation,
        _create_sliderAxis1,
        _create_normalAxis1,
        _create_sliderAxis2,
        _create_normalAxis2,
    );

    const point1 = settings.pointA ?? V0;
    const point2 = settings.pointB ?? V0;

    // convert to local space if needed
    const space = settings.space ?? ConstraintSpace.WORLD;
    if (space === ConstraintSpace.WORLD) {
        if (bodyA && !bodyA._pooled && bodyB && !bodyB._pooled) {
            quat.conjugate(_create_invQuatA, bodyA.quaternion);
            quat.conjugate(_create_invQuatB, bodyB.quaternion);

            if (settings.autoDetectPoint) {
                // determine anchor point based on body mass
                const mpA = bodyA.motionProperties;
                const mpB = bodyB.motionProperties;
                const invMassA = mpA.invMass;
                const invMassB = mpB.invMass;
                const totalInvMass = invMassA + invMassB;

                if (totalInvMass !== 0) {
                    // weighted anchor towards lighter body
                    vec3.scaleAndAdd(_create_anchor, vec3.create(), bodyA.centerOfMassPosition, invMassA / totalInvMass);
                    vec3.scaleAndAdd(_create_anchor, _create_anchor, bodyB.centerOfMassPosition, invMassB / totalInvMass);
                } else {
                    vec3.copy(_create_anchor, bodyA.centerOfMassPosition);
                }

                // transform to local space
                vec3.subtract(constraint.localSpacePositionA, _create_anchor, bodyA.centerOfMassPosition);
                vec3.transformQuat(constraint.localSpacePositionA, constraint.localSpacePositionA, _create_invQuatA);

                vec3.subtract(constraint.localSpacePositionB, _create_anchor, bodyB.centerOfMassPosition);
                vec3.transformQuat(constraint.localSpacePositionB, constraint.localSpacePositionB, _create_invQuatB);
            } else {
                // transform positions to local space
                vec3.subtract(constraint.localSpacePositionA, point1, bodyA.centerOfMassPosition);
                vec3.transformQuat(constraint.localSpacePositionA, constraint.localSpacePositionA, _create_invQuatA);

                vec3.subtract(constraint.localSpacePositionB, point2, bodyB.centerOfMassPosition);
                vec3.transformQuat(constraint.localSpacePositionB, constraint.localSpacePositionB, _create_invQuatB);
            }

            // transform slider axis and normal to local space
            vec3.transformQuat(constraint.localSpaceSliderAxisA, _create_sliderAxis1, _create_invQuatA);
            vec3.normalize(constraint.localSpaceSliderAxisA, constraint.localSpaceSliderAxisA);
            vec3.transformQuat(constraint.localSpaceNormalA, _create_normalAxis1, _create_invQuatA);
            vec3.normalize(constraint.localSpaceNormalA, constraint.localSpaceNormalA);

            // calculate second normal as slider × normal1
            vec3.cross(constraint.localSpaceNormalB, constraint.localSpaceSliderAxisA, constraint.localSpaceNormalA);

            // transform invInitialOrientation to body-relative space
            quat.multiply(constraint.invInitialOrientation, _create_invQuatB, constraint.invInitialOrientation);
            quat.multiply(constraint.invInitialOrientation, constraint.invInitialOrientation, bodyA.quaternion);
        }
    } else {
        // already in local space
        vec3.copy(constraint.localSpacePositionA, point1);
        vec3.copy(constraint.localSpacePositionB, point2);
        vec3.copy(constraint.localSpaceSliderAxisA, _create_sliderAxis1);
        vec3.copy(constraint.localSpaceNormalA, _create_normalAxis1);
        vec3.cross(constraint.localSpaceNormalB, constraint.localSpaceSliderAxisA, constraint.localSpaceNormalA);
    }

    // set limits
    const limitsMin = settings.limitsMin ?? -Infinity;
    const limitsMax = settings.limitsMax ?? Infinity;
    constraint.limitsMin = Math.min(limitsMin, 0);
    constraint.limitsMax = Math.max(limitsMax, 0);
    constraint.hasLimits = constraint.limitsMin !== -Infinity || constraint.limitsMax !== Infinity;

    if (settings.limitsSpringSettings) {
        springSettings.copy(constraint.limitsSpringSettings, settings.limitsSpringSettings);
    }

    // set motor defaults
    constraint.motorState = MotorState.OFF;
    constraint.targetVelocity = 0;
    constraint.targetPosition = 0;
    constraint.maxFrictionForce = settings.maxFrictionForce ?? 0;

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

/** remove a slider constraint */
export function remove(world: World, constraint: SliderConstraint): void {
    const sliderConstraints = world.constraints.sliderConstraints;
    const bodies = world.bodies;

    // remove from constraintIds arrays
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
    sliderConstraints.freeIndices.push(constraint.index);
}

/** get slider constraint by id */
export function get(world: World, id: ConstraintId): SliderConstraint | undefined {
    const sliderConstraints = world.constraints.sliderConstraints;
    const index = getConstraintIdIndex(id);
    const constraint = sliderConstraints.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

const _getCurrentPosition_r1 = /* @__PURE__ */ vec3.create();
const _getCurrentPosition_r2 = /* @__PURE__ */ vec3.create();
const _getCurrentPosition_u = /* @__PURE__ */ vec3.create();
const _getCurrentPosition_sliderAxis = /* @__PURE__ */ vec3.create();

/**
 * Get current position along the slider axis.
 * Recomputes the position from the current body transforms (not cached).
 * @param constraint the slider constraint
 * @param bodies the bodies collection
 * @returns current distance from the rest position
 */
export function getCurrentPosition(constraint: SliderConstraint, bodies: Bodies): number {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) {
        return constraint.d; // fallback to cached value
    }

    // r1 = rotation1 × localSpacePosition1
    vec3.transformQuat(_getCurrentPosition_r1, constraint.localSpacePositionA, bodyA.quaternion);
    // r2 = rotation2 × localSpacePosition2
    vec3.transformQuat(_getCurrentPosition_r2, constraint.localSpacePositionB, bodyB.quaternion);
    // u = x2 + r2 - x1 - r1
    vec3.subtract(_getCurrentPosition_u, bodyB.centerOfMassPosition, bodyA.centerOfMassPosition);
    vec3.add(_getCurrentPosition_u, _getCurrentPosition_u, _getCurrentPosition_r2);
    vec3.subtract(_getCurrentPosition_u, _getCurrentPosition_u, _getCurrentPosition_r1);
    // slider axis in world space
    vec3.transformQuat(_getCurrentPosition_sliderAxis, constraint.localSpaceSliderAxisA, bodyA.quaternion);
    // d = u · sliderAxis
    return vec3.dot(_getCurrentPosition_u, _getCurrentPosition_sliderAxis);
}

/**
 * Set slider constraint limits.
 * @param constraint the constraint to modify
 * @param min minimum position (must be ≤ 0)
 * @param max maximum position (must be ≥ 0)
 */
export function setLimits(constraint: SliderConstraint, min: number, max: number): void {
    constraint.limitsMin = Math.min(min, 0);
    constraint.limitsMax = Math.max(max, 0);
    constraint.hasLimits = constraint.limitsMin !== -Infinity || constraint.limitsMax !== Infinity;
}

/** Set motor state for slider constraint. */
export function setMotorState(constraint: SliderConstraint, state: MotorState): void {
    constraint.motorState = state;
}

/**
 * Set target velocity for velocity motor mode.
 * @param constraint the constraint to modify
 * @param velocity target velocity in m/s
 */
export function setTargetVelocity(constraint: SliderConstraint, velocity: number): void {
    constraint.targetVelocity = velocity;
}

/**
 * Set target position for position motor mode.
 * @param constraint the constraint to modify
 * @param position target position (will be clamped to limits if enabled)
 */
export function setTargetPosition(constraint: SliderConstraint, position: number): void {
    if (constraint.hasLimits) {
        constraint.targetPosition = Math.max(constraint.limitsMin, Math.min(constraint.limitsMax, position));
    } else {
        constraint.targetPosition = position;
    }
}

/* Set maximum friction force when motor is off */
export function setMaxFrictionForce(constraint: SliderConstraint, force: number): void {
    constraint.maxFrictionForce = force;
}

/** Get maximum friction force */
export function getMaxFrictionForce(constraint: SliderConstraint): number {
    return constraint.maxFrictionForce;
}

const _sliderConstraint_rotA = /* @__PURE__ */ mat4.create();
const _sliderConstraint_rotB = /* @__PURE__ */ mat4.create();
const _sliderConstraint_r1PlusU = /* @__PURE__ */ vec3.create();
const _sliderConstraint_invInertiaA = /* @__PURE__ */ mat4.create();
const _sliderConstraint_invInertiaB = /* @__PURE__ */ mat4.create();

/**
 * Calculate R1, R2, and U vectors.
 * R1 = rotation1 × localSpacePosition1
 * R2 = rotation2 × localSpacePosition2
 * U = x2 + r2 - x1 - r1
 *
 * @param rotationA - rotation matrix for body A (mat4)
 * @param rotationB - rotation matrix for body B (mat4)
 */
function calculateR1R2U(
    constraint: SliderConstraint,
    bodyA: RigidBody,
    bodyB: RigidBody,
    rotationA: Mat4,
    rotationB: Mat4,
): void {
    // calculate points relative to body
    mat4.multiply3x3Vec(constraint.r1, rotationA, constraint.localSpacePositionA);
    mat4.multiply3x3Vec(constraint.r2, rotationB, constraint.localSpacePositionB);

    // calculate X2 + R2 - X1 - R1
    vec3.subtract(constraint.u, bodyB.centerOfMassPosition, bodyA.centerOfMassPosition);
    vec3.add(constraint.u, constraint.u, constraint.r2);
    vec3.subtract(constraint.u, constraint.u, constraint.r1);
}

/** Calculate sliding axis and position along it */
function calculateSlidingAxisAndPosition(constraint: SliderConstraint, rotationA: Mat4): void {
    if (constraint.hasLimits || constraint.motorState !== MotorState.OFF || constraint.maxFrictionForce > 0) {
        // calculate world space slider axis
        mat4.multiply3x3Vec(constraint.worldSpaceSliderAxis, rotationA, constraint.localSpaceSliderAxisA);

        // calculate distance along axis
        constraint.d = vec3.dot(constraint.u, constraint.worldSpaceSliderAxis);
    }
}

/**
 * Calculate position constraint properties (dual axis perpendicular to slider).
 * @param rotationA rotation matrix for body A (mat4)
 * @param rotationB rotation matrix for body B (mat4)
 */
function calculatePositionConstraintProperties(
    constraint: SliderConstraint,
    bodyA: RigidBody,
    bodyB: RigidBody,
    rotationA: Mat4,
    rotationB: Mat4,
): void {
    // calculate world space normals (perpendicular to slider)
    mat4.multiply3x3Vec(constraint.n1, rotationA, constraint.localSpaceNormalA);
    mat4.multiply3x3Vec(constraint.n2, rotationA, constraint.localSpaceNormalB);

    // CRITICAL: r1 + u for the dual axis constraint
    // r1 = vector from body A's COM to body A's attachment point
    // u = p2 - p1 = vector from body A's attachment to body B's attachment
    // r1 + u = vector from body A's COM to body B's attachment point
    // This is required by the Jacobian: J^T uses -(r1 + u) × n for body A's angular part
    vec3.add(_sliderConstraint_r1PlusU, constraint.r1, constraint.u);

    dualAxisConstraintPart.calculateConstraintProperties(
        constraint.positionConstraintPart,
        bodyA,
        rotationA,
        _sliderConstraint_r1PlusU,
        bodyB,
        rotationB,
        constraint.r2,
        constraint.n1,
        constraint.n2,
    );
}

/** calculate position limits constraint properties */
function calculatePositionLimitsConstraintProperties(
    constraint: SliderConstraint,
    bodyA: RigidBody,
    bodyB: RigidBody,
    rotationA: Mat4,
    rotationB: Mat4,
    deltaTime: number,
): void {
    if (!constraint.hasLimits) {
        axisConstraintPart.deactivate(constraint.positionLimitsConstraintPart);
        return;
    }

    // check if at limits
    const belowMin = constraint.d <= constraint.limitsMin;
    const aboveMax = constraint.d >= constraint.limitsMax;

    if (belowMin || aboveMax) {
        const error = belowMin ? constraint.d - constraint.limitsMin : constraint.d - constraint.limitsMax;

        // r1 + u for the axis constraint
        vec3.add(_sliderConstraint_r1PlusU, constraint.r1, constraint.u);

        if (constraint.limitsSpringSettings.frequencyOrStiffness > 0) {
            const mpA = bodyA.motionProperties;
            const mpB = bodyB.motionProperties;
            const invMassA = bodyA.motionType === MotionType.DYNAMIC ? mpA.invMass : 0;
            const invMassB = bodyB.motionType === MotionType.DYNAMIC ? mpB.invMass : 0;

            if (bodyA.motionType === MotionType.DYNAMIC) {
                getInverseInertiaForRotation(_sliderConstraint_invInertiaA, mpA, rotationA);
            } else {
                mat4.zero(_sliderConstraint_invInertiaA);
            }

            if (bodyB.motionType === MotionType.DYNAMIC) {
                getInverseInertiaForRotation(_sliderConstraint_invInertiaB, mpB, rotationB);
            } else {
                mat4.zero(_sliderConstraint_invInertiaB);
            }

            axisConstraintPart.calculateConstraintPropertiesWithSettings(
                constraint.positionLimitsConstraintPart,
                deltaTime,
                bodyA,
                bodyB,
                invMassA,
                invMassB,
                _sliderConstraint_invInertiaA,
                _sliderConstraint_invInertiaB,
                _sliderConstraint_r1PlusU,
                constraint.r2,
                constraint.worldSpaceSliderAxis,
                0,
                error,
                constraint.limitsSpringSettings,
            );
        } else {
            const mpA = bodyA.motionProperties;
            const mpB = bodyB.motionProperties;
            const invMassA = bodyA.motionType === MotionType.DYNAMIC ? mpA.invMass : 0;
            const invMassB = bodyB.motionType === MotionType.DYNAMIC ? mpB.invMass : 0;

            if (bodyA.motionType === MotionType.DYNAMIC) {
                getInverseInertiaForRotation(_sliderConstraint_invInertiaA, mpA, rotationA);
            } else {
                mat4.zero(_sliderConstraint_invInertiaA);
            }

            if (bodyB.motionType === MotionType.DYNAMIC) {
                getInverseInertiaForRotation(_sliderConstraint_invInertiaB, mpB, rotationB);
            } else {
                mat4.zero(_sliderConstraint_invInertiaB);
            }

            axisConstraintPart.calculateConstraintProperties(
                constraint.positionLimitsConstraintPart,
                bodyA,
                bodyB,
                invMassA,
                invMassB,
                _sliderConstraint_invInertiaA,
                _sliderConstraint_invInertiaB,
                _sliderConstraint_r1PlusU,
                constraint.r2,
                constraint.worldSpaceSliderAxis,
                0,
            );
        }
    } else {
        axisConstraintPart.deactivate(constraint.positionLimitsConstraintPart);
    }
}

/* calculate motor constraint properties */
function calculateMotorConstraintProperties(
    constraint: SliderConstraint,
    bodyA: RigidBody,
    bodyB: RigidBody,
    rotationA: Mat4,
    rotationB: Mat4,
    deltaTime: number,
): void {
    const mpA = bodyA.motionProperties;
    const mpB = bodyB.motionProperties;
    const invMassA = bodyA.motionType === MotionType.DYNAMIC ? mpA.invMass : 0;
    const invMassB = bodyB.motionType === MotionType.DYNAMIC ? mpB.invMass : 0;

    if (bodyA.motionType === MotionType.DYNAMIC) {
        getInverseInertiaForRotation(_sliderConstraint_invInertiaA, mpA, rotationA);
    } else {
        mat4.zero(_sliderConstraint_invInertiaA);
    }
    if (bodyB.motionType === MotionType.DYNAMIC) {
        getInverseInertiaForRotation(_sliderConstraint_invInertiaB, mpB, rotationB);
    } else {
        mat4.zero(_sliderConstraint_invInertiaB);
    }

    // r1 + u for the axis constraint
    vec3.add(_sliderConstraint_r1PlusU, constraint.r1, constraint.u);

    switch (constraint.motorState) {
        case MotorState.OFF: {
            if (constraint.maxFrictionForce > 0) {
                axisConstraintPart.calculateConstraintProperties(
                    constraint.motorConstraintPart,
                    bodyA,
                    bodyB,
                    invMassA,
                    invMassB,
                    _sliderConstraint_invInertiaA,
                    _sliderConstraint_invInertiaB,
                    _sliderConstraint_r1PlusU,
                    constraint.r2,
                    constraint.worldSpaceSliderAxis,
                    0,
                );
            } else {
                axisConstraintPart.deactivate(constraint.motorConstraintPart);
            }
            break;
        }

        case MotorState.VELOCITY: {
            axisConstraintPart.calculateConstraintProperties(
                constraint.motorConstraintPart,
                bodyA,
                bodyB,
                invMassA,
                invMassB,
                _sliderConstraint_invInertiaA,
                _sliderConstraint_invInertiaB,
                _sliderConstraint_r1PlusU,
                constraint.r2,
                constraint.worldSpaceSliderAxis,
                -constraint.targetVelocity,
            );
            break;
        }

        case MotorState.POSITION: {
            if (springSettings.hasStiffness(constraint.motorSettings.springSettings)) {
                const error = constraint.d - constraint.targetPosition;
                axisConstraintPart.calculateConstraintPropertiesWithSettings(
                    constraint.motorConstraintPart,
                    deltaTime,
                    bodyA,
                    bodyB,
                    invMassA,
                    invMassB,
                    _sliderConstraint_invInertiaA,
                    _sliderConstraint_invInertiaB,
                    _sliderConstraint_r1PlusU,
                    constraint.r2,
                    constraint.worldSpaceSliderAxis,
                    0,
                    error,
                    constraint.motorSettings.springSettings,
                );
            } else {
                axisConstraintPart.deactivate(constraint.motorConstraintPart);
            }
            break;
        }
    }
}

/** Setup velocity constraint for slider constraint, called once per frame before velocity iterations */
export function setupVelocity(constraint: SliderConstraint, bodies: Bodies, deltaTime: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // convert quaternions to mat4 rotation matrices
    mat4.fromQuat(_sliderConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_sliderConstraint_rotB, bodyB.quaternion);

    // calculate R1, R2, U
    calculateR1R2U(constraint, bodyA, bodyB, _sliderConstraint_rotA, _sliderConstraint_rotB);

    // setup position constraint (2 DOF perpendicular to slider)
    calculatePositionConstraintProperties(constraint, bodyA, bodyB, _sliderConstraint_rotA, _sliderConstraint_rotB);

    // setup rotation constraint (3 DOF)
    rotationEulerConstraintPart.calculateConstraintProperties(
        constraint.rotationConstraintPart,
        bodyA,
        _sliderConstraint_rotA,
        bodyB,
        _sliderConstraint_rotB,
    );

    // calculate slider axis and position
    calculateSlidingAxisAndPosition(constraint, _sliderConstraint_rotA);

    // setup limits constraint
    calculatePositionLimitsConstraintProperties(
        constraint,
        bodyA,
        bodyB,
        _sliderConstraint_rotA,
        _sliderConstraint_rotB,
        deltaTime,
    );

    // setup motor constraint
    calculateMotorConstraintProperties(constraint, bodyA, bodyB, _sliderConstraint_rotA, _sliderConstraint_rotB, deltaTime);
}

/** Warm start velocity constraint for slider constraint, applies cached impulses from previous frame */
export function warmStartVelocity(constraint: SliderConstraint, bodies: Bodies, warmStartImpulseRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // warm start motor
    if (axisConstraintPart.isActive(constraint.motorConstraintPart)) {
        const mpA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties : null;
        const mpB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties : null;
        axisConstraintPart.warmStart(
            constraint.motorConstraintPart,
            bodyA,
            bodyB,
            mpA,
            mpB,
            mpA?.invMass ?? 0,
            mpB?.invMass ?? 0,
            constraint.worldSpaceSliderAxis,
            warmStartImpulseRatio,
        );
    }

    // warm start position constraint
    dualAxisConstraintPart.warmStart(
        constraint.positionConstraintPart,
        bodyA,
        bodyB,
        constraint.n1,
        constraint.n2,
        warmStartImpulseRatio,
    );

    // warm start rotation constraint
    rotationEulerConstraintPart.warmStart(constraint.rotationConstraintPart, bodyA, bodyB, warmStartImpulseRatio);

    // warm start limits
    if (axisConstraintPart.isActive(constraint.positionLimitsConstraintPart)) {
        const mpA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties : null;
        const mpB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties : null;
        axisConstraintPart.warmStart(
            constraint.positionLimitsConstraintPart,
            bodyA,
            bodyB,
            mpA,
            mpB,
            mpA?.invMass ?? 0,
            mpB?.invMass ?? 0,
            constraint.worldSpaceSliderAxis,
            warmStartImpulseRatio,
        );
    }
}

/**
 * Solve velocity constraint for slider constraint.
 * Called during velocity iterations.
 * @returns True if any impulse was applied
 */
export function solveVelocity(constraint: SliderConstraint, bodies: Bodies, deltaTime: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    let motor = false;
    let pos = false;
    let rot = false;
    let limit = false;

    // solve motor
    if (axisConstraintPart.isActive(constraint.motorConstraintPart)) {
        switch (constraint.motorState) {
            case MotorState.OFF: {
                const maxLambda = constraint.maxFrictionForce * deltaTime;
                motor = axisConstraintPart.solveVelocityConstraint(
                    constraint.motorConstraintPart,
                    bodyA,
                    bodyB,
                    constraint.worldSpaceSliderAxis,
                    -maxLambda,
                    maxLambda,
                );
                break;
            }
            case MotorState.VELOCITY:
            case MotorState.POSITION:
                motor = axisConstraintPart.solveVelocityConstraint(
                    constraint.motorConstraintPart,
                    bodyA,
                    bodyB,
                    constraint.worldSpaceSliderAxis,
                    deltaTime * constraint.motorSettings.minForceLimit,
                    deltaTime * constraint.motorSettings.maxForceLimit,
                );
                break;
        }
    }

    // solve position constraint (2 axes)
    pos = dualAxisConstraintPart.solveVelocityConstraint(
        constraint.positionConstraintPart,
        bodyA,
        bodyB,
        constraint.n1,
        constraint.n2,
    );

    // solve rotation constraint
    rot = rotationEulerConstraintPart.solveVelocityConstraint(constraint.rotationConstraintPart, bodyA, bodyB);

    // solve limits along slider axis
    if (axisConstraintPart.isActive(constraint.positionLimitsConstraintPart)) {
        let minLambda: number;
        let maxLambda: number;

        if (constraint.limitsMin === constraint.limitsMax) {
            minLambda = -Infinity;
            maxLambda = Infinity;
        } else if (constraint.d <= constraint.limitsMin) {
            minLambda = 0;
            maxLambda = Infinity;
        } else {
            minLambda = -Infinity;
            maxLambda = 0;
        }

        limit = axisConstraintPart.solveVelocityConstraint(
            constraint.positionLimitsConstraintPart,
            bodyA,
            bodyB,
            constraint.worldSpaceSliderAxis,
            minLambda,
            maxLambda,
        );
    }

    return motor || pos || rot || limit;
}

/**
 * Solve position constraint for slider constraint.
 * Called during position iterations.
 * @returns True if any correction was applied
 */
export function solvePosition(constraint: SliderConstraint, bodies: Bodies, deltaTime: number, baumgarteFactor: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // motor operates on velocities only - no position solve

    // recalculate constraint properties for position solve
    mat4.fromQuat(_sliderConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_sliderConstraint_rotB, bodyB.quaternion);
    calculateR1R2U(constraint, bodyA, bodyB, _sliderConstraint_rotA, _sliderConstraint_rotB);
    calculatePositionConstraintProperties(constraint, bodyA, bodyB, _sliderConstraint_rotA, _sliderConstraint_rotB);

    // solve position constraint (2 axes)
    const pos = dualAxisConstraintPart.solvePositionConstraint(
        constraint.positionConstraintPart,
        bodyA,
        bodyB,
        constraint.u,
        constraint.n1,
        constraint.n2,
        baumgarteFactor,
    );

    // solve rotation constraint
    rotationEulerConstraintPart.calculateConstraintProperties(
        constraint.rotationConstraintPart,
        bodyA,
        mat4.fromQuat(_sliderConstraint_rotA, bodyA.quaternion),
        bodyB,
        mat4.fromQuat(_sliderConstraint_rotB, bodyB.quaternion),
    );
    const rot = rotationEulerConstraintPart.solvePositionConstraint(
        constraint.rotationConstraintPart,
        bodyA,
        bodyB,
        constraint.invInitialOrientation,
        baumgarteFactor,
    );

    // solve limits position (only if not using spring)
    let limit = false;
    if (constraint.hasLimits && !springSettings.hasStiffness(constraint.limitsSpringSettings)) {
        // recalculate rotation matrices - body quaternions may have changed after rotation solve
        mat4.fromQuat(_sliderConstraint_rotA, bodyA.quaternion);
        mat4.fromQuat(_sliderConstraint_rotB, bodyB.quaternion);
        calculateR1R2U(constraint, bodyA, bodyB, _sliderConstraint_rotA, _sliderConstraint_rotB);
        calculateSlidingAxisAndPosition(constraint, _sliderConstraint_rotA);
        calculatePositionLimitsConstraintProperties(
            constraint,
            bodyA,
            bodyB,
            _sliderConstraint_rotA,
            _sliderConstraint_rotB,
            deltaTime,
        );

        if (axisConstraintPart.isActive(constraint.positionLimitsConstraintPart)) {
            let positionError: number;
            if (constraint.d <= constraint.limitsMin) {
                positionError = constraint.d - constraint.limitsMin;
            } else {
                positionError = constraint.d - constraint.limitsMax;
            }

            limit = axisConstraintPart.solvePositionConstraint(
                constraint.positionLimitsConstraintPart,
                bodyA,
                bodyB,
                constraint.worldSpaceSliderAxis,
                positionError,
                baumgarteFactor,
            );
        }
    }

    return pos || rot || limit;
}

/** Reset warm start for slider constraint, call this when constraint properties change significantly */
export function resetWarmStart(constraint: SliderConstraint): void {
    dualAxisConstraintPart.deactivate(constraint.positionConstraintPart);
    rotationEulerConstraintPart.deactivate(constraint.rotationConstraintPart);
    axisConstraintPart.deactivate(constraint.positionLimitsConstraintPart);
    axisConstraintPart.deactivate(constraint.motorConstraintPart);
}

/** Get total lambda for position constraint */
export function getTotalLambdaPosition(out: Vec2, constraint: SliderConstraint): Vec2 {
    out[0] = constraint.positionConstraintPart.totalLambda[0];
    out[1] = constraint.positionConstraintPart.totalLambda[1];
    return out;
}

/** Get total lambda for position limits constraint */
export function getTotalLambdaPositionLimits(constraint: SliderConstraint): number {
    return axisConstraintPart.getTotalLambdaValue(constraint.positionLimitsConstraintPart);
}

/** Get total lambda for rotation constraint */
export function getTotalLambdaRotation(out: Vec3, constraint: SliderConstraint): Vec3 {
    out[0] = constraint.rotationConstraintPart.totalLambda[0];
    out[1] = constraint.rotationConstraintPart.totalLambda[1];
    out[2] = constraint.rotationConstraintPart.totalLambda[2];
    return out;
}

/** Get total lambda for motor constraint */
export function getTotalLambdaMotor(constraint: SliderConstraint): number {
    return axisConstraintPart.getTotalLambdaValue(constraint.motorConstraintPart);
}
