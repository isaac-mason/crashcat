import type { Vec3 } from 'mathcat';
import { mat4, quat, vec3 } from 'mathcat';
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
import type { PointConstraintPart } from './constraint-part/point-constraint-part';
import * as pointConstraintPart from './constraint-part/point-constraint-part';

/**
 * Cone constraint constrains 2 bodies to a single point and limits the swing between twist axes within a cone.
 *
 * Constraint: t1 · t2 >= cos(θ)
 *
 * Where:
 * - t1 = twist axis of body 1 (world space)
 * - t2 = twist axis of body 2 (world space)
 * - θ = half cone angle (angle from principal axis of cone to edge)
 *
 * This removes 3 translation DOF (point constraint) and limits rotation to a cone (1 DOF limit).
 * Note: Unlike SwingTwistConstraint, this does NOT constrain twist rotation around the axis.
 */
export type ConeConstraint = ConstraintBase & {
    // local space configuration (stored relative to body COM)
    /** attachment point on body 1 in local space */
    localSpacePosition1: Vec3;
    /** attachment point on body 2 in local space */
    localSpacePosition2: Vec3;
    /** twist axis on body 1 in local space (normalized) */
    localSpaceTwistAxis1: Vec3;
    /** twist axis on body 2 in local space (normalized) */
    localSpaceTwistAxis2: Vec3;

    // cone limit
    /** cosine of half cone angle (cos(θ)) - constraint active when t1·t2 < cosHalfConeAngle */
    cosHalfConeAngle: number;

    // runtime cached values
    /** world space rotation axis (t2 × t1, normalized) for angular constraint */
    worldSpaceRotationAxis: Vec3;
    /** current cosine of angle between twist axes (t1 · t2) */
    cosTheta: number;

    // constraint parts
    /** point constraint for translation (3 DOF) */
    pointConstraintPart: PointConstraintPart;
    /** angle constraint for cone limit (1 DOF) */
    angleConstraintPart: AngleConstraintPart;
};

/** Creates default cone constraint */
function makeConeConstraint(): ConeConstraint {
    return {
        ...makeConstraintBase(),
        // local space
        localSpacePosition1: vec3.create(),
        localSpacePosition2: vec3.create(),
        localSpaceTwistAxis1: vec3.fromValues(1, 0, 0), // X axis default
        localSpaceTwistAxis2: vec3.fromValues(1, 0, 0), // X axis default
        // cone limit
        cosHalfConeAngle: 1, // 0 degrees - no swing allowed by default
        // runtime cached
        worldSpaceRotationAxis: vec3.fromValues(0, 1, 0), // perpendicular to X
        cosTheta: 1,
        // constraint parts
        pointConstraintPart: pointConstraintPart.create(),
        angleConstraintPart: angleConstraintPart.create(),
    };
}

/** reset constraint state for pooling reuse */
function resetConstraint(constraint: ConeConstraint): void {
    constraint.enabled = true;
    constraint._sleeping = false;
    constraint._pooled = false;
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
    angleConstraintPart.deactivate(constraint.angleConstraintPart);
}

/** Settings for creating a cone constraint */
export type ConeConstraintSettings = {
    bodyIdA: BodyId;
    bodyIdB: BodyId;
    /** pivot point on body a */
    pointA: Vec3;
    /** twist axis on body a (will be normalized) */
    twistAxisA: Vec3;
    /** pivot point on body b */
    pointB: Vec3;
    /** twist axis on body b (will be normalized) */
    twistAxisB: Vec3;
    /** half cone angle in radians (0 = no swing, π/2 = 90° cone) @default 0 */
    halfConeAngle?: number;
    /** constraint space @default ConstraintSpace.WORLD */
    space?: ConstraintSpace;
    /** constraint priority (higher = solved first) @default 0 */
    constraintPriority?: number;
    /** override number of velocity solver iterations (0 = use default) @default 0 */
    numVelocityStepsOverride?: number;
    /** override number of position solver iterations (0 = use default) @default 0 */
    numPositionStepsOverride?: number;
};

/**
 * Get a normalized perpendicular vector to the given vector.
 */
function getNormalizedPerpendicular(out: Vec3, v: Vec3): Vec3 {
    // find the smallest component and cross with that basis vector
    const ax = Math.abs(v[0]);
    const ay = Math.abs(v[1]);
    const az = Math.abs(v[2]);

    if (ax <= ay && ax <= az) {
        // X is smallest - cross with X axis
        vec3.set(out, 0, -v[2], v[1]);
    } else if (ay <= az) {
        // Y is smallest - cross with Y axis
        vec3.set(out, v[2], 0, -v[0]);
    } else {
        // Z is smallest - cross with Z axis
        vec3.set(out, -v[1], v[0], 0);
    }

    vec3.normalize(out, out);
    return out;
}

/** Create a cone constraint */
export function create(world: World, settings: ConeConstraintSettings): ConeConstraint {
    const coneConstraints = world.constraints.coneConstraints;
    const bodies = world.bodies;

    // get next sequence
    const sequence = coneConstraints.nextSequence;
    coneConstraints.nextSequence = (coneConstraints.nextSequence + 1) & SEQUENCE_MASK;

    // get constraint from pool
    let index: number;
    let constraint: ConeConstraint;
    if (coneConstraints.freeIndices.length > 0) {
        index = coneConstraints.freeIndices.pop()!;
        constraint = coneConstraints.constraints[index];
    } else {
        index = coneConstraints.constraints.length;
        constraint = makeConeConstraint();
        coneConstraints.constraints.push(constraint);
    }

    // reset pooled state
    resetConstraint(constraint);

    // set constraint id, index, sequence
    constraint.id = serConstraintId(index, ConstraintType.CONE, sequence);
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

    // normalize twist axes
    const twistAxis1 = vec3.create();
    const twistAxis2 = vec3.create();
    vec3.normalize(twistAxis1, settings.twistAxisA);
    vec3.normalize(twistAxis2, settings.twistAxisB);

    // set cone limit
    const halfConeAngle = settings.halfConeAngle ?? 0;
    constraint.cosHalfConeAngle = Math.cos(halfConeAngle);

    // initialize rotation axis to perpendicular of twist axis
    // (used when twist axes are nearly parallel and cross product would be zero)
    getNormalizedPerpendicular(constraint.worldSpaceRotationAxis, twistAxis1);

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

            // transform twist axes to local space
            vec3.transformQuat(constraint.localSpaceTwistAxis1, twistAxis1, invQuatA);
            vec3.transformQuat(constraint.localSpaceTwistAxis2, twistAxis2, invQuatB);

            // transform initial rotation axis to world space (it was based on world space twist axis)
            // already in world space, no transform needed
        }
    } else {
        // already in local space
        vec3.copy(constraint.localSpacePosition1, settings.pointA);
        vec3.copy(constraint.localSpacePosition2, settings.pointB);
        vec3.copy(constraint.localSpaceTwistAxis1, twistAxis1);
        vec3.copy(constraint.localSpaceTwistAxis2, twistAxis2);

        // if in local space, transform initial rotation axis to world space
        if (bodyA && !bodyA._pooled) {
            vec3.transformQuat(constraint.worldSpaceRotationAxis, constraint.worldSpaceRotationAxis, bodyA.quaternion);
        }
    }

    // Track constraint on both bodies
    bodyA.constraintIds.push(constraint.id);
    if (constraint.bodyIndexA !== constraint.bodyIndexB) {
        bodyB.constraintIds.push(constraint.id);
    }

    return constraint;
}

/** Remove a cone constraint */
export function remove(world: World, constraint: ConeConstraint): void {
    const coneConstraints = world.constraints.coneConstraints;
    const bodies = world.bodies;

    // Remove from bodies' constraintIds arrays
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
    coneConstraints.freeIndices.push(constraint.index);
}

/** Get cone constraint by id */
export function get(world: World, id: ConstraintId): ConeConstraint | undefined {
    const coneConstraints = world.constraints.coneConstraints;
    const index = getConstraintIdIndex(id);
    const constraint = coneConstraints.constraints[index];
    if (!constraint || constraint._pooled || constraint.sequence !== getConstraintIdSequence(id)) {
        return undefined;
    }
    return constraint;
}

/**
 * Set the half cone angle.
 * @param constraint - The constraint to modify
 * @param halfConeAngle - Half cone angle in radians (0 to π)
 */
export function setHalfConeAngle(constraint: ConeConstraint, halfConeAngle: number): void {
    constraint.cosHalfConeAngle = Math.cos(Math.max(0, Math.min(Math.PI, halfConeAngle)));
}

/**
 * Get the cosine of half cone angle.
 */
export function getCosHalfConeAngle(constraint: ConeConstraint): number {
    return constraint.cosHalfConeAngle;
}

/**
 * Get the half cone angle in radians.
 */
export function getHalfConeAngle(constraint: ConeConstraint): number {
    return Math.acos(constraint.cosHalfConeAngle);
}

const _cone_rotA = /* @__PURE__ */ mat4.create();
const _cone_rotB = /* @__PURE__ */ mat4.create();
const _cone_twist1 = /* @__PURE__ */ vec3.create();
const _cone_twist2 = /* @__PURE__ */ vec3.create();
const _cone_rotAxis = /* @__PURE__ */ vec3.create();

/**
 * Calculate rotation constraint properties.
 * Determines if cone limit is violated and sets up angle constraint.
 */
function calculateRotationConstraintProperties(constraint: ConeConstraint, bodyA: RigidBody, bodyB: RigidBody): void {
    // Get twist axes in world space
    vec3.transformQuat(_cone_twist1, constraint.localSpaceTwistAxis1, bodyA.quaternion);
    vec3.transformQuat(_cone_twist2, constraint.localSpaceTwistAxis2, bodyB.quaternion);

    // Calculate dot product between twist axes
    constraint.cosTheta = vec3.dot(_cone_twist1, _cone_twist2);

    // Check if outside cone limit
    if (constraint.cosTheta < constraint.cosHalfConeAngle) {
        // Rotation axis is t2 × t1 (cross product)
        vec3.cross(_cone_rotAxis, _cone_twist2, _cone_twist1);

        // If we can't find a rotation axis (twist axes nearly parallel), use last frame's axis
        const len = vec3.length(_cone_rotAxis);
        if (len > 1e-6) {
            vec3.scale(constraint.worldSpaceRotationAxis, _cone_rotAxis, 1 / len);
        }
        // else keep previous worldSpaceRotationAxis

        // Setup angle constraint
        angleConstraintPart.calculateConstraintProperties(
            constraint.angleConstraintPart,
            bodyA,
            bodyB,
            constraint.worldSpaceRotationAxis,
        );
    } else {
        // Inside cone - no angular constraint needed
        angleConstraintPart.deactivate(constraint.angleConstraintPart);
    }
}

/**
 * Setup velocity constraint for cone constraint.
 * Called once per frame before velocity iterations.
 */
export function setupVelocity(constraint: ConeConstraint, bodies: Bodies, _deltaTime: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // setup point constraint
    mat4.fromQuat(_cone_rotA, bodyA.quaternion);
    mat4.fromQuat(_cone_rotB, bodyB.quaternion);
    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _cone_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _cone_rotB,
        constraint.localSpacePosition2,
    );

    // setup rotation constraint (cone limit)
    calculateRotationConstraintProperties(constraint, bodyA, bodyB);
}

/**
 * Warm start velocity constraint for cone constraint.
 * Applies cached impulses from previous frame.
 */
export function warmStartVelocity(constraint: ConeConstraint, bodies: Bodies, warmStartImpulseRatio: number): void {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return;

    // Warm start point constraint
    pointConstraintPart.warmStart(constraint.pointConstraintPart, bodyA, bodyB, warmStartImpulseRatio);

    // Warm start angle constraint
    if (angleConstraintPart.isActive(constraint.angleConstraintPart)) {
        angleConstraintPart.warmStart(constraint.angleConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    }
}

/**
 * Solve velocity constraint for cone constraint.
 * Called during velocity iterations.
 * @returns True if any impulse was applied
 */
export function solveVelocity(constraint: ConeConstraint, bodies: Bodies, _deltaTime: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // Solve point constraint
    const pos = pointConstraintPart.solveVelocityConstraint(constraint.pointConstraintPart, bodyA, bodyB);

    // Solve angle constraint (cone limit)
    // Lambda must be >= 0 (can only push bodies apart, not pull them together)
    let rot = false;
    if (angleConstraintPart.isActive(constraint.angleConstraintPart)) {
        rot = angleConstraintPart.solveVelocityConstraint(
            constraint.angleConstraintPart,
            bodyA,
            bodyB,
            constraint.worldSpaceRotationAxis,
            0, // minLambda - can only push apart
            Infinity, // maxLambda
        );
    }

    return pos || rot;
}

/**
 * Solve position constraint for cone constraint.
 * Called during position iterations.
 * @returns True if any correction was applied
 */
export function solvePosition(constraint: ConeConstraint, bodies: Bodies, _deltaTime: number, baumgarteFactor: number): boolean {
    const bodyA = bodies.pool[constraint.bodyIndexA];
    const bodyB = bodies.pool[constraint.bodyIndexB];

    if (!bodyA || !bodyB || bodyA._pooled || bodyB._pooled) return false;

    // recalculate and solve point constraint
    mat4.fromQuat(_cone_rotA, bodyA.quaternion);
    mat4.fromQuat(_cone_rotB, bodyB.quaternion);
    pointConstraintPart.calculateConstraintProperties(
        constraint.pointConstraintPart,
        bodyA,
        _cone_rotA,
        constraint.localSpacePosition1,
        bodyB,
        _cone_rotB,
        constraint.localSpacePosition2,
    );
    const pos = pointConstraintPart.solvePositionConstraint(constraint.pointConstraintPart, bodyA, bodyB, baumgarteFactor);

    // recalculate and solve rotation constraint
    calculateRotationConstraintProperties(constraint, bodyA, bodyB);
    let rot = false;
    if (angleConstraintPart.isActive(constraint.angleConstraintPart)) {
        // position error is cosTheta - cosHalfConeAngle (negative when violated)
        const positionError = constraint.cosTheta - constraint.cosHalfConeAngle;
        rot = angleConstraintPart.solvePositionConstraint(
            constraint.angleConstraintPart,
            bodyA,
            bodyB,
            positionError,
            baumgarteFactor,
        );
    }

    return pos || rot;
}

/**
 * Reset warm start for cone constraint.
 */
export function resetWarmStart(constraint: ConeConstraint): void {
    pointConstraintPart.deactivate(constraint.pointConstraintPart);
    angleConstraintPart.deactivate(constraint.angleConstraintPart);
}

/** Get total lambda for position constraint */
export function getTotalLambdaPosition(out: Vec3, constraint: ConeConstraint): Vec3 {
    out[0] = constraint.pointConstraintPart.totalLambda[0];
    out[1] = constraint.pointConstraintPart.totalLambda[1];
    out[2] = constraint.pointConstraintPart.totalLambda[2];
    return out;
}

/** Get total lambda for rotation constraint */
export function getTotalLambdaRotation(constraint: ConeConstraint): number {
    return constraint.angleConstraintPart.totalLambda;
}
