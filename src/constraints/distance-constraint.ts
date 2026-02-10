import type { Vec3 } from 'mathcat';
import { mat4, quat, vec3 } from 'mathcat';
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
import type { SpringSettings } from './constraint-part/spring-settings';
import * as springSettings from './constraint-part/spring-settings';
import { type ConstraintPool, defineUserConstraint, ensurePool } from './constraints';

/** distance constraint removes 1 translational DOF (distance between two points) */
export type DistanceConstraint = ConstraintBase & {
    localSpacePosition1: Vec3;
    localSpacePosition2: Vec3;
    minDistance: number;
    maxDistance: number;
    limitsSpringSettings: SpringSettings;
    // runtime state
    worldSpacePosition1: Vec3;
    worldSpacePosition2: Vec3;
    worldSpaceNormal: Vec3;
    minLambda: number;
    maxLambda: number;
    axisConstraint: AxisConstraintPart;
};

/** creates default distance constraint */
function makeDistanceConstraint(): DistanceConstraint {
    return {
        ...makeConstraintBase(),
        localSpacePosition1: vec3.create(),
        localSpacePosition2: vec3.create(),
        minDistance: 0,
        maxDistance: 0,
        limitsSpringSettings: springSettings.create(),
        worldSpacePosition1: vec3.create(),
        worldSpacePosition2: vec3.create(),
        worldSpaceNormal: vec3.fromValues(0, 1, 0), // fallback normal
        minLambda: 0,
        maxLambda: 0,
        axisConstraint: axisConstraintPart.create(),
    };
}

/** settings for creating a distance constraint */
export type DistanceConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    /**
     * body a constraint reference frame (space determined by `space`).
     * constraint will keep point1 (a point on body A) and point2 (a point on body B) at the same distance.
     * note that this constraint can be used as a cheap PointConstraint by setting point1 = point2 (but this removes only 1 degree of freedom instead of 3).
     */
    pointA: Vec3;
    /** body b constraint reference frame (space determined by `space`) */
    pointB: Vec3;
    /** minimum distance (-1 = use initial distance) @default -1 */
    minDistance?: number;
    /** maximum distance (-1 = use initial distance) @default -1 */
    maxDistance?: number;
    /** spring settings for soft limits */
    springSettings?: SpringSettings;
    /** constraint space@default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

/** reset constraint state for pooling reuse */
function resetConstraint(constraint: DistanceConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    axisConstraintPart.deactivate(constraint.axisConstraint);
}

/** create a distance constraint */
export function create(world: World, settings: DistanceConstraintSettings): DistanceConstraint {
    const pool = ensurePool<DistanceConstraint>(world.constraints, ConstraintType.DISTANCE);
    const bodies = world.bodies;

    // get next sequence
    const sequence = pool.nextSequence;
    pool.nextSequence = (pool.nextSequence + 1) & SEQUENCE_MASK;

    // get constraint from pool
    let index: number;
    let constraint: DistanceConstraint;
    if (pool.freeIndices.length > 0) {
        // reuse existing pooled constraint
        index = pool.freeIndices.pop()!;
        constraint = pool.constraints[index];
    } else {
        // expand array
        index = pool.constraints.length;
        constraint = makeDistanceConstraint();
        pool.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    // set constraint id, index, sequence
    constraint.id = serConstraintId(index, ConstraintType.DISTANCE, sequence);
    constraint.index = index;
    constraint.sequence = sequence;

    // set base constraint properties
    constraint.constraintPriority = settings.constraintPriority ?? 0;
    constraint.numVelocityStepsOverride = settings.numVelocityStepsOverride ?? 0;
    constraint.numPositionStepsOverride = settings.numPositionStepsOverride ?? 0;

    // extract body indices from IDs
    constraint.bodyIndexA = getBodyIdIndex(settings.bodyIdA);
    constraint.bodyIndexB = getBodyIdIndex(settings.bodyIdB);

    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    // convert to local space if needed
    const space = settings.space ?? ConstraintSpace.WORLD;
    if (space === ConstraintSpace.WORLD) {
        // transform world points to local space relative to body COM
        if (bodyA && !bodyA._pooled && bodyB && !bodyB._pooled) {
            const invQuatA = quat.create();
            const invQuatB = quat.create();
            quat.conjugate(invQuatA, bodyA.quaternion);
            quat.conjugate(invQuatB, bodyB.quaternion);

            // local = quatInverse * (worldPoint - centerOfMassPosition)
            vec3.subtract(constraint.localSpacePosition1, settings.pointA, bodyA.centerOfMassPosition);
            vec3.transformQuat(constraint.localSpacePosition1, constraint.localSpacePosition1, invQuatA);

            vec3.subtract(constraint.localSpacePosition2, settings.pointB, bodyB.centerOfMassPosition);
            vec3.transformQuat(constraint.localSpacePosition2, constraint.localSpacePosition2, invQuatB);

            // store initial world positions for distance calculation
            vec3.copy(constraint.worldSpacePosition1, settings.pointA);
            vec3.copy(constraint.worldSpacePosition2, settings.pointB);
        }
    } else {
        // points specified in local space - copy directly
        vec3.copy(constraint.localSpacePosition1, settings.pointA);
        vec3.copy(constraint.localSpacePosition2, settings.pointB);

        // calculate world positions from local
        if (bodyA && !bodyA._pooled && bodyB && !bodyB._pooled) {
            // world = centerOfMassPosition + rotation * localPoint
            vec3.transformQuat(constraint.worldSpacePosition1, constraint.localSpacePosition1, bodyA.quaternion);
            vec3.add(constraint.worldSpacePosition1, constraint.worldSpacePosition1, bodyA.centerOfMassPosition);

            vec3.transformQuat(constraint.worldSpacePosition2, constraint.localSpacePosition2, bodyB.quaternion);
            vec3.add(constraint.worldSpacePosition2, constraint.worldSpacePosition2, bodyB.centerOfMassPosition);
        }
    }

    // Calculate initial distance
    const delta = vec3.create();
    vec3.subtract(delta, constraint.worldSpacePosition2, constraint.worldSpacePosition1);
    const distance = vec3.length(delta);

    // Set min/max (use initial distance if not specified)
    const minDistance = settings.minDistance ?? -1;
    const maxDistance = settings.maxDistance ?? -1;
    if (minDistance < 0 && maxDistance < 0) {
        constraint.minDistance = distance;
        constraint.maxDistance = distance;
    } else {
        constraint.minDistance = minDistance < 0 ? Math.min(distance, maxDistance) : minDistance;
        constraint.maxDistance = maxDistance < 0 ? Math.max(distance, minDistance) : maxDistance;
    }

    // Set spring settings
    if (settings.springSettings) {
        springSettings.copy(constraint.limitsSpringSettings, settings.springSettings);
    }

    // track constraint on both bodies
    bodyA.constraintIds.push(constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB) {
        bodyB.constraintIds.push(constraint.id);
    }

    return constraint;
}

/** remove a distance constraint */
export function remove(world: World, constraint: DistanceConstraint): void {
    const pool = ensurePool<DistanceConstraint>(world.constraints, ConstraintType.DISTANCE);
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
    pool.freeIndices.push(constraint.index);
}

/** get distance constraint by id */
export function get(world: World, id: ConstraintId): DistanceConstraint | undefined {
    const pool = world.constraints.pools[ConstraintType.DISTANCE] as ConstraintPool<DistanceConstraint> | undefined;
    if (!pool) return undefined;
    const index = getConstraintIdIndex(id);
    const constraint = pool.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

const _distanceConstraint_rotA = /* @__PURE__ */ mat4.create();
const _distanceConstraint_rotB = /* @__PURE__ */ mat4.create();
const _distanceConstraint_invInertiaA = /* @__PURE__ */ mat4.create();
const _distanceConstraint_invInertiaB = /* @__PURE__ */ mat4.create();
const _distanceConstraint_r1PlusU = /* @__PURE__ */ vec3.create();
const _distanceConstraint_r2 = /* @__PURE__ */ vec3.create();
const _distanceConstraint_delta = /* @__PURE__ */ vec3.create();

/** calculate distance constraint properties. Updates world positions, normal, and sets up axis constraint part */
function calculateDistanceConstraintProperties(
    constraint: DistanceConstraint,
    bodyA: RigidBody,
    bodyB: RigidBody,
    deltaTime: number,
): void {
    // update world space positions (the bodies may have moved)
    // worldPos = centerOfMassPosition + rotation * localPos
    vec3.transformQuat(constraint.worldSpacePosition1, constraint.localSpacePosition1, bodyA.quaternion);
    vec3.add(constraint.worldSpacePosition1, constraint.worldSpacePosition1, bodyA.centerOfMassPosition);

    vec3.transformQuat(constraint.worldSpacePosition2, constraint.localSpacePosition2, bodyB.quaternion);
    vec3.add(constraint.worldSpacePosition2, constraint.worldSpacePosition2, bodyB.centerOfMassPosition);

    // calculate world space normal (direction from point1 to point2)
    const delta = _distanceConstraint_delta;
    vec3.subtract(delta, constraint.worldSpacePosition2, constraint.worldSpacePosition1);
    const deltaLen = vec3.length(delta);

    if (deltaLen > 0) {
        vec3.scale(constraint.worldSpaceNormal, delta, 1 / deltaLen);
    }
    // else keep previous normal (handles distance = 0 case)

    // calculate points relative to body COM
    // r1 + u = (p1 - x1) + (p2 - p1) = p2 - x1
    // where p1, p2 are constraint points and x1, x2 are body COMs
    const r1PlusU = _distanceConstraint_r1PlusU;
    vec3.subtract(r1PlusU, constraint.worldSpacePosition2, bodyA.centerOfMassPosition);

    const r2 = _distanceConstraint_r2;
    vec3.subtract(r2, constraint.worldSpacePosition2, bodyB.centerOfMassPosition);

    // get inverse masses and inertias
    const mpA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties : null;
    const mpB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties : null;
    const invMassA = mpA ? mpA.invMass : 0;
    const invMassB = mpB ? mpB.invMass : 0;

    const invInertiaA = _distanceConstraint_invInertiaA;
    const invInertiaB = _distanceConstraint_invInertiaB;

    if (mpA) {
        const rotA = _distanceConstraint_rotA;
        mat4.fromQuat(rotA, bodyA.quaternion);
        getInverseInertiaForRotation(invInertiaA, mpA, rotA);
    }

    if (mpB) {
        const rotB = _distanceConstraint_rotB;
        mat4.fromQuat(rotB, bodyB.quaternion);
        getInverseInertiaForRotation(invInertiaB, mpB, rotB);
    }

    // determine constraint mode based on current distance vs limits
    if (constraint.minDistance === constraint.maxDistance) {
        // fixed distance - constraint acts in both directions
        axisConstraintPart.calculateConstraintPropertiesWithSettings(
            constraint.axisConstraint,
            deltaTime,
            bodyA,
            bodyB,
            invMassA,
            invMassB,
            invInertiaA,
            invInertiaB,
            r1PlusU,
            r2,
            constraint.worldSpaceNormal,
            0, // bias
            deltaLen - constraint.minDistance, // C (constraint error)
            constraint.limitsSpringSettings,
        );
        constraint.minLambda = -Infinity;
        constraint.maxLambda = Infinity;
    } else if (deltaLen <= constraint.minDistance) {
        // too close - push apart (only positive impulse allowed)
        axisConstraintPart.calculateConstraintPropertiesWithSettings(
            constraint.axisConstraint,
            deltaTime,
            bodyA,
            bodyB,
            invMassA,
            invMassB,
            invInertiaA,
            invInertiaB,
            r1PlusU,
            r2,
            constraint.worldSpaceNormal,
            0, // bias
            deltaLen - constraint.minDistance, // C (constraint error, negative)
            constraint.limitsSpringSettings,
        );
        constraint.minLambda = 0;
        constraint.maxLambda = Infinity;
    } else if (deltaLen >= constraint.maxDistance) {
        // too far - pull together (only negative impulse allowed)
        axisConstraintPart.calculateConstraintPropertiesWithSettings(
            constraint.axisConstraint,
            deltaTime,
            bodyA,
            bodyB,
            invMassA,
            invMassB,
            invInertiaA,
            invInertiaB,
            r1PlusU,
            r2,
            constraint.worldSpaceNormal,
            0, // bias
            deltaLen - constraint.maxDistance, // C (constraint error, positive)
            constraint.limitsSpringSettings,
        );
        constraint.minLambda = -Infinity;
        constraint.maxLambda = 0;
    } else {
        // within limits - no constraint active
        axisConstraintPart.deactivate(constraint.axisConstraint);
    }
}

function setupVelocity(constraint: DistanceConstraint, bodies: Bodies, deltaTime: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    calculateDistanceConstraintProperties(constraint, bodyA, bodyB, deltaTime);
}

function warmStartVelocity(constraint: DistanceConstraint, bodies: Bodies, warmStartImpulseRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    const invMassA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties.invMass : 0;
    const invMassB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties.invMass : 0;

    axisConstraintPart.warmStart(
        constraint.axisConstraint,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        constraint.worldSpaceNormal,
        warmStartImpulseRatio,
    );
}

function solveVelocity(constraint: DistanceConstraint, bodies: Bodies, _deltaTime: number): boolean {
    if (!axisConstraintPart.isActive(constraint.axisConstraint)) {
        return false;
    }

    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    return axisConstraintPart.solveVelocityConstraint(
        constraint.axisConstraint,
        bodyA,
        bodyB,
        constraint.worldSpaceNormal,
        constraint.minLambda,
        constraint.maxLambda,
    );
}

function solvePosition(
    constraint: DistanceConstraint,
    bodies: Bodies,
    deltaTime: number,
    baumgarteFactor: number,
): boolean {
    // skip position correction if spring is active (handled by velocity bias)
    if (constraint.limitsSpringSettings.frequencyOrStiffness > 0) {
        return false;
    }

    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // calculate current distance along constraint normal
    const distance = vec3.dot(
        vec3.subtract(_distanceConstraint_delta, constraint.worldSpacePosition2, constraint.worldSpacePosition1),
        constraint.worldSpaceNormal,
    );

    // calculate position error
    let positionError = 0;
    if (distance < constraint.minDistance) {
        positionError = distance - constraint.minDistance;
    } else if (distance > constraint.maxDistance) {
        positionError = distance - constraint.maxDistance;
    }

    if (positionError === 0) {
        return false;
    }

    // recalculate constraint properties (bodies may have moved during position solve)
    calculateDistanceConstraintProperties(constraint, bodyA, bodyB, deltaTime);

    return axisConstraintPart.solvePositionConstraint(
        constraint.axisConstraint,
        bodyA,
        bodyB,
        constraint.worldSpaceNormal,
        positionError,
        baumgarteFactor,
    );
}

function resetWarmStart(constraint: DistanceConstraint): void {
    axisConstraintPart.deactivate(constraint.axisConstraint);
}

/**
 * Get total accumulated impulse (lambda) from distance constraint.
 * Used for debugging, telemetry, or breaking constraints based on force.
 */
export function getTotalLambda(constraint: DistanceConstraint): number {
    return constraint.axisConstraint.totalLambda;
}

/**
 * Set the distance limits for a distance constraint.
 * @param constraint - The constraint to modify
 * @param minDistance - Minimum distance (use -1 to keep current)
 * @param maxDistance - Maximum distance (use -1 to keep current)
 */
export function setDistance(constraint: DistanceConstraint, minDistance: number, maxDistance: number): void {
    constraint.minDistance = minDistance < 0 ? constraint.minDistance : minDistance;
    constraint.maxDistance = maxDistance < 0 ? constraint.maxDistance : maxDistance;
    // Ensure min <= max
    if (constraint.minDistance > constraint.maxDistance) {
        const avg = (constraint.minDistance + constraint.maxDistance) / 2;
        constraint.minDistance = avg;
        constraint.maxDistance = avg;
    }
}

/** the constraint definition for distance constraint */
export const def = /* @__PURE__ */ (() => defineUserConstraint<DistanceConstraint>({
    type: ConstraintType.DISTANCE,
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
