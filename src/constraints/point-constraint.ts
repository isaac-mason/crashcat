import type { Vec3 } from 'mathcat';
import { mat4, quat, vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies';
import { type BodyId, getBodyIdIndex } from '../body/body-id';
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
import type { PointConstraintPart } from './constraint-part/point-constraint-part';
import * as pointConstraintPart from './constraint-part/point-constraint-part';

/** point constraint removes 3 translational DOF */
export type PointConstraint = ConstraintBase & {
    localSpacePosition1: Vec3;
    localSpacePosition2: Vec3;
    pointConstraintPart: PointConstraintPart;
};

/** creates default point constraint */
function makePointConstraint(): PointConstraint {
    return {
        ...makeConstraintBase(),
        localSpacePosition1: vec3.create(),
        localSpacePosition2: vec3.create(),
        pointConstraintPart: pointConstraintPart.create(),
    };
}

/** reset constraint state for pooling reuse */
function resetConstraint(constraint: PointConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
}

/** settings for creating a point constraint */
export type PointConstraintSettings = {
    /** body a id */
    bodyIdA: BodyId;
    /** body b id */
    bodyIdB: BodyId;
    /** point on body a in world space or local space depending on `space` */
    pointA: Vec3;
    /** point on body b in world space or local space depending on `space` */
    pointB: Vec3;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

/** create a point constraint */
export function create(world: World, settings: PointConstraintSettings): PointConstraint {
    const pointConstraints = world.constraints.pointConstraints;
    const bodies = world.bodies;

    // get next sequence
    const sequence = pointConstraints.nextSequence;
    pointConstraints.nextSequence = (pointConstraints.nextSequence + 1) & SEQUENCE_MASK;

    // get constraint from pool
    let index: number;
    let constraint: PointConstraint;
    if (pointConstraints.freeIndices.length > 0) {
        // reuse existing pooled constraint
        index = pointConstraints.freeIndices.pop()!;
        constraint = pointConstraints.constraints[index];
    } else {
        // expand array
        index = pointConstraints.constraints.length;
        constraint = makePointConstraint();
        pointConstraints.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    // set constraint id, index, sequence
    constraint.id = serConstraintId(index, ConstraintType.POINT, sequence);
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
        }
    } else {
        vec3.copy(constraint.localSpacePosition1, settings.pointA);
        vec3.copy(constraint.localSpacePosition2, settings.pointB);
    }

    // track constraint on both bodies
    bodyA.constraintIds.push(constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB) {
        bodyB.constraintIds.push(constraint.id);
    }

    return constraint;
}

/** remove a point constraint */
export function remove(world: World, constraint: PointConstraint): void {
    const pointConstraints = world.constraints.pointConstraints;
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
    pointConstraints.freeIndices.push(constraint.index);
}

/** get point constraint by id */
export function get(world: World, id: ConstraintId): PointConstraint | undefined {
    const pointConstraints = world.constraints.pointConstraints;
    const index = getConstraintIdIndex(id);
    const constraint = pointConstraints.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

const _setupPointConstraintVelocity_rotA = mat4.create();
const _setupPointConstraintVelocity_rotB = mat4.create();

/**
 * Setup velocity constraint - calculate constraint properties from current body poses
 * Called once per frame before velocity iterations
 */
export function setupVelocity(constraint: PointConstraint, bodies: Bodies): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // convert quaternions to mat4 rotation matrices
    mat4.fromQuat(_setupPointConstraintVelocity_rotA, bodyA.quaternion);
    mat4.fromQuat(_setupPointConstraintVelocity_rotB, bodyB.quaternion);

    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _setupPointConstraintVelocity_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _setupPointConstraintVelocity_rotB,
        constraint.localSpacePosition2,
    );
}

/**
 * Warm start velocity constraint - apply cached impulses from previous frame
 * Called after setup, before velocity iterations
 */
export function warmStartVelocity(constraint: PointConstraint, bodies: Bodies, warmStartImpulseRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    pointConstraintPart.warmStart(constraint.pointConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
}

/**
 * Solve velocity constraint - iteratively enforce velocity constraint
 * Called during velocity iterations
 */
export function solveVelocity(constraint: PointConstraint, bodies: Bodies): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    return pointConstraintPart.solveVelocityConstraint(constraint.pointConstraintPart, bodyA, bodyB);
}

const _solvePointConstraintPosition_rotA = mat4.create();
const _solvePointConstraintPosition_rotB = mat4.create();

/**
 * Solve position constraint - Baumgarte stabilization for position drift
 * Called during position iterations
 */
export function solvePosition(constraint: PointConstraint, bodies: Bodies, baumgarte: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    // recalculate constraint properties (bodies may have moved during position solve)
    mat4.fromQuat(_solvePointConstraintPosition_rotA, bodyA.quaternion);
    mat4.fromQuat(_solvePointConstraintPosition_rotB, bodyB.quaternion);

    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _solvePointConstraintPosition_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _solvePointConstraintPosition_rotB,
        constraint.localSpacePosition2,
    );

    return pointConstraintPart.solvePositionConstraint(constraint.pointConstraintPart, bodyA, bodyB, baumgarte);
}

/**
 * Reset warm start - clears accumulated impulses
 * Called when warm start needs to be invalidated (e.g., constraint properties changed significantly)
 */
export function resetWarmStart(constraint: PointConstraint): void {
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
}

/**
 * Get total accumulated impulse (lambda) from constraint
 * Used for debugging, telemetry, or breaking constraints based on force
 */
export function getTotalLambda(constraint: PointConstraint): Vec3 {
    return constraint.pointConstraintPart.totalLambda;
}
