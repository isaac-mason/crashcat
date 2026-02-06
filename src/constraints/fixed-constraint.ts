import type { Quat, Vec3 } from 'mathcat';
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
import type { RotationEulerConstraintPart } from './constraint-part/rotation-euler-constraint-part';
import * as rotationEulerConstraintPart from './constraint-part/rotation-euler-constraint-part';

/**
 * Fixed constraint removes 6 DOF (3 translation + 3 rotation).
 * Welds two bodies together so they move as one rigid body.
 */
export type FixedConstraint = ConstraintBase & {
    // local space configuration (stored relative to body COM) ---
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;

    // rotation tracking
    /** inverse of initial relative orientation from body 1 to body 2 */
    invInitialOrientation: Quat;

    // constraint parts
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** rotation constraint (3 DOF) - keeps orientations locked */
    rotationConstraintPart: RotationEulerConstraintPart;
};

/** creates default fixed constraint */
function makeFixedConstraint(): FixedConstraint {
    return {
        ...makeConstraintBase(),
        // local space
        localSpacePosition1: vec3.create(),
        localSpacePosition2: vec3.create(),
        // rotation tracking
        invInitialOrientation: quat.create(),
        // constraint parts
        pointConstraintPart: pointConstraintPart.create(),
        rotationConstraintPart: rotationEulerConstraintPart.create(),
    };
}

/** reset constraint state for pooling reuse */
function resetConstraint(constraint: FixedConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    rotationEulerConstraintPart.deactivate(constraint.rotationConstraintPart);
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
}

/** settings for creating a fixed constraint */
export type FixedConstraintSettings = {
    /** body a id */
    bodyIdA: BodyId;
    /** body b id */
    bodyIdB: BodyId;
    /** pivot point on body 1 (required unless autoDetectPoint is true) */
    point1?: Vec3;
    /** reference axis X for body 1 (for orientation) */
    axisX1?: Vec3;
    /** reference axis Y for body 1 (for orientation) */
    axisY1?: Vec3;
    /** pivot point on body 2 (required unless autoDetectPoint is true) */
    point2?: Vec3;
    /** reference axis X for body 2 (for orientation) */
    axisX2?: Vec3;
    /** reference axis Y for body 2 (for orientation) */
    axisY2?: Vec3;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /**
     * auto-detect attachment point based on body positions/masses.
     * when true, point1 and point2 are ignored.
     * @default false
     */
    autoDetectPoint?: boolean;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

/** create a fixed constraint */
export function create(world: World, settings: FixedConstraintSettings): FixedConstraint {
    const fixedConstraints = world.constraints.fixedConstraints;
    const bodies = world.bodies;

    // get next sequence
    const sequence = fixedConstraints.nextSequence;
    fixedConstraints.nextSequence = (fixedConstraints.nextSequence + 1) & SEQUENCE_MASK;

    // get constraint from pool
    let index: number;
    let constraint: FixedConstraint;
    if (fixedConstraints.freeIndices.length > 0) {
        index = fixedConstraints.freeIndices.pop()!;
        constraint = fixedConstraints.constraints[index];
    } else {
        index = fixedConstraints.constraints.length;
        constraint = makeFixedConstraint();
        fixedConstraints.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    // set constraint id, index, sequence
    constraint.id = serConstraintId(index, ConstraintType.FIXED, sequence);
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

    // get reference axes (default to world axes)
    const axisX1 = settings.axisX1 ?? vec3.fromValues(1, 0, 0);
    const axisY1 = settings.axisY1 ?? vec3.fromValues(0, 1, 0);
    const axisX2 = settings.axisX2 ?? vec3.fromValues(1, 0, 0);
    const axisY2 = settings.axisY2 ?? vec3.fromValues(0, 1, 0);

    // normalize axes
    const normalizedAxisX1 = vec3.create();
    const normalizedAxisY1 = vec3.create();
    const normalizedAxisX2 = vec3.create();
    const normalizedAxisY2 = vec3.create();
    vec3.normalize(normalizedAxisX1, axisX1);
    vec3.normalize(normalizedAxisY1, axisY1);
    vec3.normalize(normalizedAxisX2, axisX2);
    vec3.normalize(normalizedAxisY2, axisY2);

    // calculate initial orientation from reference axes
    const r0 = rotationEulerConstraintPart.getInvInitialOrientationXY(
        normalizedAxisX1,
        normalizedAxisY1,
        normalizedAxisX2,
        normalizedAxisY2,
    );

    // convert to local space if needed
    const space = settings.space ?? ConstraintSpace.WORLD;
    if (space === ConstraintSpace.WORLD) {
        if (bodyA && !bodyA._pooled && bodyB && !bodyB._pooled) {
            const invQuatA = quat.create();
            const invQuatB = quat.create();
            quat.conjugate(invQuatA, bodyA.quaternion);
            quat.conjugate(invQuatB, bodyB.quaternion);

            if (settings.autoDetectPoint) {
                // auto-detect anchor point based on inverse masses
                // weighted average towards lighter body (higher invMass = more influence)
                const invM1 = bodyA.motionProperties.invMass;
                const invM2 = bodyB.motionProperties.invMass;
                const totalInvMass = invM1 + invM2;

                const anchor = vec3.create();
                if (totalInvMass > 0) {
                    // weighted average: lighter body (higher invMass) pulls anchor towards it
                    const w1 = invM1 / totalInvMass;
                    const w2 = invM2 / totalInvMass;
                    vec3.scaleAndAdd(anchor, anchor, bodyA.centerOfMassPosition, w1);
                    vec3.scaleAndAdd(anchor, anchor, bodyB.centerOfMassPosition, w2);
                } else {
                    // both infinite mass - use body A's position
                    vec3.copy(anchor, bodyA.centerOfMassPosition);
                }

                // transform to local space
                vec3.subtract(constraint.localSpacePosition1, anchor, bodyA.centerOfMassPosition);
                vec3.transformQuat(constraint.localSpacePosition1, constraint.localSpacePosition1, invQuatA);

                vec3.subtract(constraint.localSpacePosition2, anchor, bodyB.centerOfMassPosition);
                vec3.transformQuat(constraint.localSpacePosition2, constraint.localSpacePosition2, invQuatB);
            } else {
                // transform positions to local space
                if (settings.point1) {
                    vec3.subtract(constraint.localSpacePosition1, settings.point1, bodyA.centerOfMassPosition);
                    vec3.transformQuat(constraint.localSpacePosition1, constraint.localSpacePosition1, invQuatA);
                } else {
                    vec3.zero(constraint.localSpacePosition1);
                }

                if (settings.point2) {
                    vec3.subtract(constraint.localSpacePosition2, settings.point2, bodyB.centerOfMassPosition);
                    vec3.transformQuat(constraint.localSpacePosition2, constraint.localSpacePosition2, invQuatB);
                } else {
                    vec3.zero(constraint.localSpacePosition2);
                }
            }

            // transform inverse initial orientation to body-relative space
            // r0^-1 = (q20^-1 * c2) * (q10^-1 * c1)^-1 = q20^-1 * (c2 * c1^-1) * q10
            // => body-relative: invQuatB * r0 * bodyA.quaternion
            quat.multiply(constraint.invInitialOrientation, invQuatB, r0);
            quat.multiply(constraint.invInitialOrientation, constraint.invInitialOrientation, bodyA.quaternion);
        }
    } else {
        // already in local space
        if (settings.point1) {
            vec3.copy(constraint.localSpacePosition1, settings.point1);
        } else {
            vec3.zero(constraint.localSpacePosition1);
        }

        if (settings.point2) {
            vec3.copy(constraint.localSpacePosition2, settings.point2);
        } else {
            vec3.zero(constraint.localSpacePosition2);
        }

        // for local space, invInitialOrientation is just r0
        quat.copy(constraint.invInitialOrientation, r0);
    }

    // track constraint on both bodies
    bodyA.constraintIds.push(constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB) {
        bodyB.constraintIds.push(constraint.id);
    }

    return constraint;
}

/** remove a fixed constraint */
export function remove(world: World, constraint: FixedConstraint): void {
    const fixedConstraints = world.constraints.fixedConstraints;
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
    fixedConstraints.freeIndices.push(constraint.index);
}

/** get fixed constraint by id */
export function get(world: World, id: ConstraintId): FixedConstraint | undefined {
    const fixedConstraints = world.constraints.fixedConstraints;
    const index = getConstraintIdIndex(id);
    const constraint = fixedConstraints.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

const _fixedConstraint_rotA = mat4.create();
const _fixedConstraint_rotB = mat4.create();

/**
 * Setup velocity constraint for fixed constraint.
 * Called once per frame before velocity iterations.
 */
export function setupVelocity(constraint: FixedConstraint, bodies: Bodies, _deltaTime: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // convert quaternions to rotation matrices
    mat4.fromQuat(_fixedConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_fixedConstraint_rotB, bodyB.quaternion);

    // setup rotation constraint (3 DOF - keeps orientations locked)
    rotationEulerConstraintPart.calculateConstraintProperties(
        constraint.rotationConstraintPart,
        bodyA,
        _fixedConstraint_rotA,
        bodyB,
        _fixedConstraint_rotB,
    );

    // setup point constraint (3 DOF - keeps positions together)
    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _fixedConstraint_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _fixedConstraint_rotB,
        constraint.localSpacePosition2,
    );
}

/**
 * Warm start velocity constraint for fixed constraint.
 * Applies cached impulses from previous frame.
 */
export function warmStartVelocity(constraint: FixedConstraint, bodies: Bodies, warmStartImpulseRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // warm start both constraint parts
    rotationEulerConstraintPart.warmStart(constraint.rotationConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    pointConstraintPart.warmStart(constraint.pointConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
}

/**
 * Solve velocity constraint for fixed constraint.
 * Called during velocity iterations.
 * @returns True if any impulse was applied
 */
export function solveVelocity(constraint: FixedConstraint, bodies: Bodies, _deltaTime: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // solve rotation constraint first (to avoid translation being affected by rotation errors)
    const rot = rotationEulerConstraintPart.solveVelocityConstraint(constraint.rotationConstraintPart, bodyA, bodyB);

    // solve point constraint
    const pos = pointConstraintPart.solveVelocityConstraint(constraint.pointConstraintPart, bodyA, bodyB);

    return rot || pos;
}

/**
 * Solve position constraint for fixed constraint.
 * Called during position iterations.
 * @returns True if any correction was applied
 */
export function solvePosition(constraint: FixedConstraint, bodies: Bodies, _deltaTime: number, baumgarteFactor: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // recalculate constraint properties (since bodies may have moved)
    mat4.fromQuat(_fixedConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_fixedConstraint_rotB, bodyB.quaternion);

    // solve rotation constraint
    rotationEulerConstraintPart.calculateConstraintProperties(
        constraint.rotationConstraintPart,
        bodyA,
        _fixedConstraint_rotA,
        bodyB,
        _fixedConstraint_rotB,
    );
    const rot = rotationEulerConstraintPart.solvePositionConstraint(
        constraint.rotationConstraintPart,
        bodyA,
        bodyB,
        constraint.invInitialOrientation,
        baumgarteFactor,
    );

    // solve point constraint (recalculate since rotation may have changed)
    mat4.fromQuat(_fixedConstraint_rotA, bodyA.quaternion);
    mat4.fromQuat(_fixedConstraint_rotB, bodyB.quaternion);

    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _fixedConstraint_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _fixedConstraint_rotB,
        constraint.localSpacePosition2,
    );
    const pos = pointConstraintPart.solvePositionConstraint(constraint.pointConstraintPart, bodyA, bodyB, baumgarteFactor);

    return rot || pos;
}

/**
 * Reset warm start for fixed constraint.
 * Called when constraint properties change significantly.
 */
export function resetWarmStart(constraint: FixedConstraint): void {
    rotationEulerConstraintPart.deactivate(constraint.rotationConstraintPart);
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
}

/** Get total lambda for position constraint (for debugging/inspection) */
export function getTotalLambdaPosition(out: Vec3, constraint: FixedConstraint): Vec3 {
    out[0] = constraint.pointConstraintPart.totalLambda[0];
    out[1] = constraint.pointConstraintPart.totalLambda[1];
    out[2] = constraint.pointConstraintPart.totalLambda[2];
    return out;
}

/** Get total lambda for rotation constraint (for debugging/inspection) */
export function getTotalLambdaRotation(out: Vec3, constraint: FixedConstraint): Vec3 {
    out[0] = constraint.rotationConstraintPart.totalLambda[0];
    out[1] = constraint.rotationConstraintPart.totalLambda[1];
    out[2] = constraint.rotationConstraintPart.totalLambda[2];
    return out;
}
