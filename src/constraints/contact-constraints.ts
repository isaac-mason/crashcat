import { type Mat4, mat4, quat, type Vec3, vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies';
import { getInverseInertiaForRotation } from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
import * as body from '../body/rigid-body';
import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import * as contacts from '../contacts';
import type { Listener } from '../listener';
import type { ContactManifold } from '../manifold/manifold';
import * as manifold from '../manifold/manifold';
import { pool } from '../utils/pool';
import type { WorldSettings } from '../world-settings';
import { combineMaterial } from './combine-material';
import * as axisConstraintPart from './constraint-part/axis-constraint-part';

const constraintPool = /* @__PURE__ */ pool(createContactConstraint);

/** state for contact constraint solving, holds all active constraints and manages constraint lifecycle */
export type ContactConstraints = {
    /** array of active contact constraints (one per colliding body pair) */
    constraints: ContactConstraint[];
};

/**
 * Contact constraint for a body pair.
 * Contains all contact points and shared constraint data.
 *
 * One ContactConstraint per active collision pair per frame.
 * Can have 1-4 contact points.
 * All contact points share the same normal/tangent directions and material properties.
 */
export type ContactConstraint = {
    /** index of first body in the contact constraint */
    bodyIndexA: number;

    /** index of second body in the contact constraint */
    bodyIndexB: number;

    /** sub-shape ID of shape A */
    subShapeIdA: number;

    /** sub-shape ID of shape B */
    subShapeIdB: number;

    /** normalized contact normal (points from A toward B), same for all 4 contact points in this manifold */
    normal: Vec3;

    /** first tangent vector (perpendicular to normal), computed as: normalize(cross(normal, arbitrary_vector)) */
    tangent1: Vec3;

    /** second tangent vector (perpendicular to normal and tangent1), computed as: cross(normal, tangent1) */
    tangent2: Vec3;

    /** combined friction coefficient, computed from both bodies' friction values. typical range: 0.0 (frictionless) to 1.0+ (high friction) */
    friction: number;

    /** combined restitution coefficient, controls bounce behavior. typical range: 0.0 (no bounce) to 1.0+ (perfect bounce) */
    restitution: number;

    /** number of active contact points (0-4), used to track how many of the pre-allocated contactPoints are valid */
    numContactPoints: number;

    /** pool of 4 contact points, only the first numContactPoints entries are valid */
    contactPoints: [WorldContactPoint, WorldContactPoint, WorldContactPoint, WorldContactPoint];

    /** inverse mass of body A (1/massA), 0 if body is static. cached from body at constraint setup time */
    invMassA: number;

    /** inverse mass of body B (1/massB), 0 if body is static. cached from body at constraint setup time */
    invMassB: number;

    /** inverse inertia matrix of body A, used to compute angular impulse response. cached from body at constraint setup time */
    invInertiaA: Mat4;

    /** inverse mass matrix of body B, used to compute angular impulse response. cached from body at constraint setup time */
    invInertiaB: Mat4;

    /** inverse inertia scale for body A, applied during position constraint solving to override inertia */
    invInertiaScaleA: number;

    /** inverse inertia scale for body B, applied during position constraint solving to override inertia */
    invInertiaScaleB: number;

    /** index of the contact in the global contact array, used to look up contact for writing back solved impulses */
    contactIndex: number;

    /** sort key for deterministic ordering */
    sortKey: number;
};

/**
 * A single contact point with full constraint data.
 * Links world-space geometry to constraint solving.
 *
 * Each contact point has 3 constraint axes:
 * - Normal: prevents penetration (1 DOF)
 * - Tangent1: friction in first direction (1 DOF)
 * - Tangent2: friction in second direction (1 DOF)
 * Total: 3 DOF per contact point (max 4 points = 12 DOF per manifold)
 */
export type WorldContactPoint = {
    /**
     * Contact point position on body A (world space).
     * Recomputed each solving iteration from local position + body transform.
     */
    positionA: Vec3;

    /**
     * Contact point position on body B (world space).
     * Recomputed each solving iteration from local position + body transform.
     */
    positionB: Vec3;

    /**
     * Contact point position in body A's local/relative space (center of mass space).
     * Constant - used to recompute world position when body moves.
     */
    localPositionA: Vec3;

    /**
     * Contact point position in body B's local/relative space (center of mass space).
     * Constant - used to recompute world position when body moves.
     */
    localPositionB: Vec3;

    /**
     * Normal constraint: prevents interpenetration.
     * Direction: contact normal (perpendicular to contact surface).
     * Lambda clamping: normalLambda >= 0 (push only, no pull).
     */
    normalConstraint: axisConstraintPart.AxisConstraintPart;

    /**
     * Friction constraint 1: resists motion in first tangent direction.
     * Direction: first tangent (perpendicular to normal).
     * Lambda clamping: |tangentLambda1| <= friction * normalLambda.
     */
    tangentConstraint1: axisConstraintPart.AxisConstraintPart;

    /**
     * Friction constraint 2: resists motion in second tangent direction.
     * Direction: second tangent (perpendicular to normal and tangent1).
     * Lambda clamping: |tangentLambda2| <= friction * normalLambda.
     */
    tangentConstraint2: axisConstraintPart.AxisConstraintPart;
};

/**
 * Settings for a contact constraint, passed to contact listener callbacks.
 * A @see Listener can modify these settings to customize the contact behavior.
 */
export type ContactSettings = {
    /** combined friction coefficient for the contact */
    combinedFriction: number;

    /** combined restitution (bounciness) for the contact */
    combinedRestitution: number;

    /** if true, contact is treated as a sensor (no physical response) */
    isSensor: boolean;

    /** scale factor for body 1's inverse mass (default 1.0) */
    invMassScale1: number;

    /** scale factor for body 2's inverse mass (default 1.0) */
    invMassScale2: number;

    /** scale factor for body 1's inverse inertia (default 1.0) */
    invInertiaScale1: number;

    /** scale factor for body 2's inverse inertia (default 1.0) */
    invInertiaScale2: number;

    /** relative linear surface velocity (body 2 relative to body 1) */
    relativeLinearSurfaceVelocity: Vec3;

    /** relative angular surface velocity (body 2 relative to body 1) */
    relativeAngularSurfaceVelocity: Vec3;
};

/** creates emopty contact constraints state */
export function init(): ContactConstraints {
    return {
        constraints: [],
    };
}

/** create a contact settings object */
export function createContactSettings(): ContactSettings {
    return {
        combinedFriction: 0,
        combinedRestitution: 0,
        isSensor: false,
        invMassScale1: 1,
        invMassScale2: 1,
        invInertiaScale1: 1,
        invInertiaScale2: 1,
        relativeLinearSurfaceVelocity: vec3.create(),
        relativeAngularSurfaceVelocity: vec3.create(),
    };
}

/** copy contact settings properties from a source object */
export function copyContactSettings(out: ContactSettings, source: ContactSettings): ContactSettings {
    out.combinedFriction = source.combinedFriction;
    out.combinedRestitution = source.combinedRestitution;
    out.isSensor = source.isSensor;
    out.invMassScale1 = source.invMassScale1;
    out.invMassScale2 = source.invMassScale2;
    out.invInertiaScale1 = source.invInertiaScale1;
    out.invInertiaScale2 = source.invInertiaScale2;
    vec3.copy(out.relativeLinearSurfaceVelocity, source.relativeLinearSurfaceVelocity);
    vec3.copy(out.relativeAngularSurfaceVelocity, source.relativeAngularSurfaceVelocity);
    return out;
}

function setContactSettings(
    settings: ContactSettings,
    combinedFriction: number,
    combinedRestitution: number,
    isSensor: boolean,
): ContactSettings {
    settings.combinedFriction = combinedFriction;
    settings.combinedRestitution = combinedRestitution;
    settings.isSensor = isSensor;
    settings.invMassScale1 = 1;
    settings.invMassScale2 = 1;
    settings.invInertiaScale1 = 1;
    settings.invInertiaScale2 = 1;
    vec3.zero(settings.relativeLinearSurfaceVelocity);
    vec3.zero(settings.relativeAngularSurfaceVelocity);
    return settings;
}

/**
 * Calculate friction bias with surface velocity support.
 * Enables conveyor belts, rotating platforms, etc.
 * @param settings contact settings containing surface velocities
 * @param rA moment arm from body A's COM to contact point
 * @param tangent friction tangent direction (normalized)
 * @returns friction bias (velocity along tangent)
 */
function calculateFrictionBias(settings: ContactSettings, rA: Vec3, tangent: Vec3): number {
    // surface velocity in world space + angular contribution: ω_surface × r
    const svx =
        settings.relativeLinearSurfaceVelocity[0] +
        (settings.relativeAngularSurfaceVelocity[1] * rA[2] - settings.relativeAngularSurfaceVelocity[2] * rA[1]);
    const svy =
        settings.relativeLinearSurfaceVelocity[1] +
        (settings.relativeAngularSurfaceVelocity[2] * rA[0] - settings.relativeAngularSurfaceVelocity[0] * rA[2]);
    const svz =
        settings.relativeLinearSurfaceVelocity[2] +
        (settings.relativeAngularSurfaceVelocity[0] * rA[1] - settings.relativeAngularSurfaceVelocity[1] * rA[0]);

    // project onto tangent
    return tangent[0] * svx + tangent[1] * svy + tangent[2] * svz;
}

const _calcBias_relativeVelocity = /* @__PURE__ */ vec3.create();
const _calcBias_v1 = /* @__PURE__ */ vec3.create();
const _calcBias_v2 = /* @__PURE__ */ vec3.create();
const _calcBias_diff = /* @__PURE__ */ vec3.create();
const _calcBias_relativeAcceleration = /* @__PURE__ */ vec3.create();

/**
 * Calculate the normal velocity bias for a contact constraint.
 * Includes speculative contacts, restitution, and gravity+force correction.
 *
 * @param bodyA first body in contact
 * @param bodyB second body in contact
 * @param positionA contact position on body A (world space)
 * @param positionB contact position on body B (world space)
 * @param normal contact normal (world space, points from A to B)
 * @param rA moment arm from body A's COM to contact point
 * @param rB moment arm from body B's COM to contact point
 * @param combinedRestitution combined restitution coefficient
 * @param deltaTime physics time step
 * @param gravity gravity vector
 * @param minVelocityForRestitution minimum velocity threshold for restitution
 * @returns Normal velocity bias for constraint solving
 */
function calculateNormalVelocityBias(
    bodyA: body.RigidBody,
    bodyB: body.RigidBody,
    positionA: Vec3,
    positionB: Vec3,
    normal: Vec3,
    rA: Vec3,
    rB: Vec3,
    combinedRestitution: number,
    deltaTime: number,
    gravity: Vec3,
    minVelocityForRestitution: number,
): number {
    // step 1: calculate relative velocity at contact point
    let relativeVelocity: Vec3;

    // check for non-static (both dynamic and kinematic contribute velocity)
    const aIsMoving = bodyA.motionType !== MotionType.STATIC;
    const bIsMoving = bodyB.motionType !== MotionType.STATIC;
    const aIsDynamic = bodyA.motionType === MotionType.DYNAMIC;
    const bIsDynamic = bodyB.motionType === MotionType.DYNAMIC;

    if (aIsMoving && bIsMoving) {
        // both non-static: use both velocities
        body.getVelocityAtPointCOM(_calcBias_v1, bodyA, rA);
        body.getVelocityAtPointCOM(_calcBias_v2, bodyB, rB);
        vec3.subtract(_calcBias_relativeVelocity, _calcBias_v2, _calcBias_v1);
        relativeVelocity = _calcBias_relativeVelocity;
    } else if (aIsMoving) {
        // only A is non-static
        body.getVelocityAtPointCOM(_calcBias_v1, bodyA, rA);
        vec3.negate(_calcBias_relativeVelocity, _calcBias_v1);
        relativeVelocity = _calcBias_relativeVelocity;
    } else if (bIsMoving) {
        // only B is non-static
        body.getVelocityAtPointCOM(_calcBias_v2, bodyB, rB);
        relativeVelocity = _calcBias_v2;
    } else {
        // both static (shouldn't happen for constraints)
        return 0;
    }

    const normalVelocity = vec3.dot(relativeVelocity, normal);

    // step 2: calculate penetration depth
    // penetration = (pos1 - pos2) · normal
    // positive = penetrating, negative = separated
    vec3.subtract(_calcBias_diff, positionA, positionB);
    const penetration = vec3.dot(_calcBias_diff, normal);

    // step 3: speculative contact bias
    // if there is no penetration, this is a speculative contact and we will apply a bias to the contact constraint
    // so that the constraint becomes relative_velocity . contact normal > -penetration / delta_time
    // instead of relative_velocity . contact normal > 0
    // see: GDC 2013: "Physics for Game Programmers; Continuous Collision" - Erin Catto
    const speculativeContactVelocityBias = Math.max(0, -penetration / deltaTime);

    // step 4: determine final bias with restitution
    let normalVelocityBias: number;

    if (combinedRestitution > 0 && normalVelocity < -minVelocityForRestitution) {
        // we have a velocity that is big enough for restitution. This is where speculative contacts don't work
        // great as we have to decide now if we're going to apply the restitution or not.
        // if the relative velocity is big enough for a hit, we apply the restitution (in the end, due to other constraints,
        // the objects may actually not collide and we will have applied restitution incorrectly).
        if (normalVelocity < -speculativeContactVelocityBias) {
            // the gravity and constant forces are applied at the beginning of the time step.
            // if we get here, there was a collision at the beginning of the time step, so we've applied too much force.
            // this means that our calculated restitution can be too high resulting in an increase in energy.
            // so, when we apply restitution, we cancel the added velocity due to these forces.
            vec3.zero(_calcBias_relativeAcceleration);

            // calculate effect of gravity
            // apply to all non-static bodies (both dynamic and kinematic)
            if (aIsMoving && bIsMoving) {
                const gravityFactorA = bodyA.motionProperties.gravityFactor;
                const gravityFactorB = bodyB.motionProperties.gravityFactor;
                vec3.scaleAndAdd(
                    _calcBias_relativeAcceleration,
                    _calcBias_relativeAcceleration,
                    gravity,
                    gravityFactorB - gravityFactorA,
                );
            } else if (aIsMoving) {
                const gravityFactorA = bodyA.motionProperties.gravityFactor;
                vec3.scaleAndAdd(_calcBias_relativeAcceleration, _calcBias_relativeAcceleration, gravity, -gravityFactorA);
            } else if (bIsMoving) {
                const gravityFactorB = bodyB.motionProperties.gravityFactor;
                vec3.scaleAndAdd(_calcBias_relativeAcceleration, _calcBias_relativeAcceleration, gravity, gravityFactorB);
            }

            // calculate effect of accumulated forces (only for dynamic bodies)
            if (aIsDynamic) {
                vec3.scaleAndAdd(
                    _calcBias_relativeAcceleration,
                    _calcBias_relativeAcceleration,
                    bodyA.motionProperties.force,
                    -bodyA.motionProperties.invMass,
                );
            }
            if (bIsDynamic) {
                vec3.scaleAndAdd(
                    _calcBias_relativeAcceleration,
                    _calcBias_relativeAcceleration,
                    bodyB.motionProperties.force,
                    bodyB.motionProperties.invMass,
                );
            }

            // we only compensate forces towards the contact normal
            const forceDeltaVelocity = Math.min(0, vec3.dot(_calcBias_relativeAcceleration, normal) * deltaTime);

            normalVelocityBias = combinedRestitution * (normalVelocity - forceDeltaVelocity);
        } else {
            // In this case we have predicted that we don't hit the other object, but if we do (due to other constraints changing velocities)
            // the speculative contact will prevent penetration but will not apply restitution
            normalVelocityBias = speculativeContactVelocityBias;
        }
    } else {
        // No restitution. We can safely apply our contact velocity bias.
        normalVelocityBias = speculativeContactVelocityBias;
    }

    return normalVelocityBias;
}

export function clear(contactConstraints: ContactConstraints): void {
    for (const constraint of contactConstraints.constraints) {
        constraintPool.release(constraint);
    }
    contactConstraints.constraints.length = 0;
}

function compareContactConstraints(a: ContactConstraint, b: ContactConstraint): number {
    if (a.bodyIndexA !== b.bodyIndexA) return a.bodyIndexA - b.bodyIndexA;
    if (a.bodyIndexB !== b.bodyIndexB) return a.bodyIndexB - b.bodyIndexB;
    if (a.subShapeIdA !== b.subShapeIdA) return a.subShapeIdA - b.subShapeIdA;
    return a.subShapeIdB - b.subShapeIdB;
}

/** sort contact constraints, required for determinism */
export function sortContactConstraints(contactConstraints: ContactConstraints): void {
    contactConstraints.constraints.sort(compareContactConstraints);
}

/** compute tangent vectors perpendicular to contact normal */
function computeTangents(normal: Vec3, tangent1: Vec3, tangent2: Vec3): void {
    const x = normal[0];
    const y = normal[1];
    const z = normal[2];

    if (Math.abs(x) > Math.abs(y)) {
        const len = Math.sqrt(x * x + z * z);
        vec3.set(tangent1, z / len, 0, -x / len);
    } else {
        const len = Math.sqrt(y * y + z * z);
        vec3.set(tangent1, 0, z / len, -y / len);
    }

    vec3.cross(tangent2, normal, tangent1);
}

const _addContactConstraint_invQuatA = /* @__PURE__ */ quat.create();
const _addContactConstraint_invQuatB = /* @__PURE__ */ quat.create();
const _addContactConstraint_relativePointOnA = /* @__PURE__ */ vec3.create();
const _addContactConstraint_relativePointOnB = /* @__PURE__ */ vec3.create();
const _addContactConstraint_worldPosA = /* @__PURE__ */ vec3.create();
const _addContactConstraint_worldPosB = /* @__PURE__ */ vec3.create();
const _addContactConstraint_localPosA = /* @__PURE__ */ vec3.create();
const _addContactConstraint_localPosB = /* @__PURE__ */ vec3.create();
const _addContactConstraint_midpoint = /* @__PURE__ */ vec3.create();
const _addContactConstraint_rA = /* @__PURE__ */ vec3.create();
const _addContactConstraint_rB = /* @__PURE__ */ vec3.create();
const _addContactConstraint_invInertiaA = /* @__PURE__ */ mat4.create();
const _addContactConstraint_invInertiaB = /* @__PURE__ */ mat4.create();
const _addContactConstraint_contactSettings = /* @__PURE__ */ createContactSettings();

/** add a contact constraint from a new manifold */
export function addContactConstraint(
    contactConstraints: ContactConstraints,
    contactsState: contacts.Contacts,
    bodyA: body.RigidBody,
    bodyB: body.RigidBody,
    contactManifold: ContactManifold,
    settings: WorldSettings,
    contactListener: Listener | undefined,
    deltaTime: number,
): boolean {
    // swap bodies so that body 1 id < body 2 id
    if (bodyA.id > bodyB.id) {
        const temp = bodyA;
        bodyA = bodyB;
        bodyB = temp;
        manifold.swapShapes(contactManifold);
    }

    // find or create contact in the contact array
    const existingContact = contacts.findContact(
        contactsState,
        bodyA,
        bodyB,
        contactManifold.subShapeIdA,
        contactManifold.subShapeIdB,
    );
    const contact =
        existingContact ??
        contacts.createContact(contactsState, bodyA, bodyB, contactManifold.subShapeIdA, contactManifold.subShapeIdB);

    // mark contact as processed this frame (for stale contact cleanup)
    contact.processedThisFrame = true;

    // transform the world space normal to body B's local space and store in contact
    quat.conjugate(_addContactConstraint_invQuatB, bodyB.quaternion);
    vec3.transformQuat(contact.contactNormal, contactManifold.worldSpaceNormal, _addContactConstraint_invQuatB);
    vec3.normalize(contact.contactNormal, contact.contactNormal);
    contact.numContactPoints = contactManifold.numContactPoints;

    // prepare contact settings that will be passed to Listener
    const contactSettings = setContactSettings(
        _addContactConstraint_contactSettings,
        combineMaterial(bodyA.friction, bodyB.friction, bodyA.frictionCombineMode, bodyB.frictionCombineMode),
        combineMaterial(bodyA.restitution, bodyB.restitution, bodyA.restitutionCombineMode, bodyB.restitutionCombineMode),
        bodyA.sensor || bodyB.sensor,
    );

    // call contact listener, which can modify ContactSettings to customize contact behavior
    if (existingContact) {
        contact.flags |= contacts.CachedManifoldFlags.ContactPersisted;
        contactListener?.onContactPersisted?.(bodyA, bodyB, contactManifold, contactSettings);
    } else {
        contactListener?.onContactAdded?.(bodyA, bodyB, contactManifold, contactSettings);
    }

    // prepare inverse quaternions for local space transforms (used by both sensors and regular contacts)
    const invQuatA = quat.conjugate(_addContactConstraint_invQuatA, bodyA.quaternion);
    const invQuatB = quat.conjugate(_addContactConstraint_invQuatB, bodyB.quaternion);

    // if one of the bodies is a sensor, don't actually create the constraint
    // one of the bodies must be dynamic and have mass to be able to create a contact constraint
    if (
        !contactSettings.isSensor &&
        ((bodyA.motionType === MotionType.DYNAMIC && bodyA.motionProperties.invMass !== 0) ||
            (bodyB.motionType === MotionType.DYNAMIC && bodyB.motionProperties.invMass !== 0))
    ) {
        // add contact constraint
        const constraint = constraintPool.request();
        contactConstraints.constraints.push(constraint);

        // set body indices and sub-shape IDs
        constraint.bodyIndexA = bodyA.index;
        constraint.bodyIndexB = bodyB.index;
        constraint.subShapeIdA = contactManifold.subShapeIdA;
        constraint.subShapeIdB = contactManifold.subShapeIdB;
        constraint.contactIndex = contact.contactIndex; // store ID for impulse writeback

        // compute sort key for deterministic constraint solving
        constraint.sortKey = computeContactSortKey(
            bodyA.index,
            bodyB.index,
            contactManifold.subShapeIdA,
            contactManifold.subShapeIdB,
        );

        // normal and tangents
        vec3.copy(constraint.normal, contactManifold.worldSpaceNormal);
        computeTangents(constraint.normal, constraint.tangent1, constraint.tangent2);

        // set material properties from settings (may have been modified by listener)
        constraint.friction = contactSettings.combinedFriction;
        constraint.restitution = contactSettings.combinedRestitution;

        // cache body properties (with mass/inertia scaling from settings)
        constraint.invMassA = bodyA.motionProperties.invMass * contactSettings.invMassScale1;
        constraint.invMassB = bodyB.motionProperties.invMass * contactSettings.invMassScale2;
        constraint.invInertiaScaleA = contactSettings.invInertiaScale1;
        constraint.invInertiaScaleB = contactSettings.invInertiaScale2;
        const invInertiaA = _addContactConstraint_invInertiaA;
        const invInertiaB = _addContactConstraint_invInertiaB;
        const rotA = mat4.create();
        const rotB = mat4.create();
        mat4.fromQuat(rotA, bodyA.quaternion);
        mat4.fromQuat(rotB, bodyB.quaternion);
        getInverseInertiaForRotation(invInertiaA, bodyA.motionProperties, rotA);
        getInverseInertiaForRotation(invInertiaB, bodyB.motionProperties, rotB);

        // apply invInertiaScale1/2 to inertia matrices (element-wise)
        if (contactSettings.invInertiaScale1 === 1 && contactSettings.invInertiaScale2 === 1) {
            // no scaling needed
            mat4.copy(constraint.invInertiaA, invInertiaA);
            mat4.copy(constraint.invInertiaB, invInertiaB);
        } else {
            // apply scaling
            for (let i = 0; i < 16; ++i) {
                constraint.invInertiaA[i] = invInertiaA[i] * contactSettings.invInertiaScale1;
                constraint.invInertiaB[i] = invInertiaB[i] * contactSettings.invInertiaScale2;
            }
        }

        // create contact points with matching
        constraint.numContactPoints = 0;

        for (let i = 0; i < contactManifold.numContactPoints; i++) {
            const cp = constraint.contactPoints[constraint.numContactPoints];
            constraint.numContactPoints++;

            // get relative contact points from manifold
            const relativePointOnA = vec3.fromBuffer(
                _addContactConstraint_relativePointOnA,
                contactManifold.relativeContactPointsOnA,
                i * 3,
            );
            const relativePointOnB = vec3.fromBuffer(
                _addContactConstraint_relativePointOnB,
                contactManifold.relativeContactPointsOnB,
                i * 3,
            );

            // get world positions from manifold
            vec3.add(cp.positionA, contactManifold.baseOffset, relativePointOnA);
            vec3.add(cp.positionB, contactManifold.baseOffset, relativePointOnB);

            // transform to local center of mass space
            vec3.subtract(cp.localPositionA, cp.positionA, bodyA.centerOfMassPosition);
            vec3.transformQuat(cp.localPositionA, cp.localPositionA, invQuatA);

            vec3.subtract(cp.localPositionB, cp.positionB, bodyB.centerOfMassPosition);
            vec3.transformQuat(cp.localPositionB, cp.localPositionB, invQuatB);

            // check if we have have a close contact point from last update
            // match to cached contact point (first match wins)
            // checks BOTH position1 AND position2 within threshold
            let lambdaSet = false;

            if (existingContact) {
                for (let j = 0; j < existingContact.numContactPoints; j++) {
                    const point = existingContact.contactPoints[j];

                    // both positions within threshold?
                    const dist1Sq = vec3.squaredDistance(cp.localPositionA, point.position1);
                    const dist2Sq = vec3.squaredDistance(cp.localPositionB, point.position2);

                    const threshold = settings.contacts.contactPointPreserveLambdaMaxDistSq;

                    if (dist1Sq < threshold && dist2Sq < threshold) {
                        // get lambdas from previous frame
                        cp.normalConstraint.totalLambda = point.normalLambda;
                        cp.tangentConstraint1.totalLambda = point.frictionLambda1;
                        cp.tangentConstraint2.totalLambda = point.frictionLambda2;
                        lambdaSet = true;
                        break;
                    }
                }
            }

            if (!lambdaSet) {
                // no match found, cold start lambdas
                cp.normalConstraint.totalLambda = 0;
                cp.tangentConstraint1.totalLambda = 0;
                cp.tangentConstraint2.totalLambda = 0;
            }

            // use normal from constraint
            const smoothedNormal = constraint.normal;

            // calculate collision points relative to body
            const midpoint = _addContactConstraint_midpoint;
            const rA = _addContactConstraint_rA;
            const rB = _addContactConstraint_rB;

            vec3.add(midpoint, cp.positionA, cp.positionB);
            vec3.scale(midpoint, midpoint, 0.5);
            vec3.subtract(rA, midpoint, bodyA.centerOfMassPosition);
            vec3.subtract(rB, midpoint, bodyB.centerOfMassPosition);

            // calculate normal velocity bias with restitution and speculative contacts
            const normalVelocityBias = calculateNormalVelocityBias(
                bodyA,
                bodyB,
                cp.positionA,
                cp.positionB,
                smoothedNormal,
                _addContactConstraint_rA,
                _addContactConstraint_rB,
                contactSettings.combinedRestitution,
                deltaTime,
                settings.gravity,
                settings.solver.minVelocityForRestitution,
            );

            axisConstraintPart.calculateConstraintProperties(
                cp.normalConstraint,
                bodyA,
                bodyB,
                constraint.invMassA,
                constraint.invMassB,
                constraint.invInertiaA,
                constraint.invInertiaB,
                _addContactConstraint_rA,
                _addContactConstraint_rB,
                smoothedNormal,
                normalVelocityBias,
            );

            // calculate friction constraints
            // deactivate friction constraints when friction=0 to skip calculation entirely
            if (contactSettings.combinedFriction > 0) {
                // calculate friction bias with surface velocity support
                const frictionBias1 = calculateFrictionBias(contactSettings, _addContactConstraint_rA, constraint.tangent1);
                const frictionBias2 = calculateFrictionBias(contactSettings, _addContactConstraint_rA, constraint.tangent2);

                axisConstraintPart.calculateConstraintProperties(
                    cp.tangentConstraint1,
                    bodyA,
                    bodyB,
                    constraint.invMassA,
                    constraint.invMassB,
                    constraint.invInertiaA,
                    constraint.invInertiaB,
                    _addContactConstraint_rA,
                    _addContactConstraint_rB,
                    constraint.tangent1,
                    frictionBias1,
                );

                axisConstraintPart.calculateConstraintProperties(
                    cp.tangentConstraint2,
                    bodyA,
                    bodyB,
                    constraint.invMassA,
                    constraint.invMassB,
                    constraint.invInertiaA,
                    constraint.invInertiaB,
                    _addContactConstraint_rA,
                    _addContactConstraint_rB,
                    constraint.tangent2,
                    frictionBias2,
                );
            } else {
                axisConstraintPart.deactivate(cp.tangentConstraint1);
                axisConstraintPart.deactivate(cp.tangentConstraint2);
            }

            // store to contact array for next frame's warm starting
            const cachedPoint = contact.contactPoints[i];
            vec3.copy(cachedPoint.position1, cp.localPositionA);
            vec3.copy(cachedPoint.position2, cp.localPositionB);

            // store impulses (will be updated after solving)
            cachedPoint.normalLambda = cp.normalConstraint.totalLambda;
            cachedPoint.frictionLambda1 = cp.tangentConstraint1.totalLambda;
            cachedPoint.frictionLambda2 = cp.tangentConstraint2.totalLambda;
        }

        return true;
    }

    // if no dynamic bodies, or is a sensor, we skip creating the contact constraint
    // store sensor contact points to contact array
    // this ensures proper onContactAdded/onContactPersisted semantics for sensors
    for (let i = 0; i < contactManifold.numContactPoints; i++) {
        // get relative contact points from manifold
        const relativePointOnA = vec3.fromBuffer(
            _addContactConstraint_relativePointOnA,
            contactManifold.relativeContactPointsOnA,
            i * 3,
        );
        const relativePointOnB = vec3.fromBuffer(
            _addContactConstraint_relativePointOnB,
            contactManifold.relativeContactPointsOnB,
            i * 3,
        );

        // world positions from manifold
        const worldPosA = vec3.add(_addContactConstraint_worldPosA, contactManifold.baseOffset, relativePointOnA);
        const worldPosB = vec3.add(_addContactConstraint_worldPosB, contactManifold.baseOffset, relativePointOnB);

        // transform to local space for caching
        const localPosA = vec3.subtract(_addContactConstraint_localPosA, worldPosA, bodyA.centerOfMassPosition);
        vec3.transformQuat(localPosA, localPosA, invQuatA);

        const localPosB = vec3.subtract(_addContactConstraint_localPosB, worldPosB, bodyB.centerOfMassPosition);
        vec3.transformQuat(localPosB, localPosB, invQuatB);

        // store local positions to contact array
        const cachedPoint = contact.contactPoints[i];
        vec3.copy(cachedPoint.position1, localPosA);
        vec3.copy(cachedPoint.position2, localPosB);

        // reset impulses (sensors don't apply forces)
        cachedPoint.normalLambda = 0;
        cachedPoint.frictionLambda1 = 0;
        cachedPoint.frictionLambda2 = 0;
    }

    // no contact constraint created for sensors
    return false;
}

/**
 * Create a new WorldContactPoint with all fields initialized.
 * Positions at origin, constraints zeroed, no cached point.
 */
export function createWorldContactPoint(): WorldContactPoint {
    return {
        positionA: vec3.create(),
        positionB: vec3.create(),
        localPositionA: vec3.create(),
        localPositionB: vec3.create(),
        normalConstraint: axisConstraintPart.create(),
        tangentConstraint1: axisConstraintPart.create(),
        tangentConstraint2: axisConstraintPart.create(),
    };
}

/**
 * Reset a WorldContactPoint to zero values.
 */
// export function resetWorldContactPoint(point: WorldContactPoint): void {
//     vec3.set(point.positionA, 0, 0, 0);
//     vec3.set(point.positionB, 0, 0, 0);
//     vec3.set(point.localPositionA, 0, 0, 0);
//     vec3.set(point.localPositionB, 0, 0, 0);
//     axisConstraintPart.resetAxisConstraintPart(point.normalConstraint);
//     axisConstraintPart.resetAxisConstraintPart(point.tangentConstraint1);
//     axisConstraintPart.resetAxisConstraintPart(point.tangentConstraint2);
// }

function createContactConstraint(): ContactConstraint {
    return {
        bodyIndexA: -1,
        bodyIndexB: -1,
        subShapeIdA: EMPTY_SUB_SHAPE_ID,
        subShapeIdB: EMPTY_SUB_SHAPE_ID,
        normal: vec3.create(),
        tangent1: vec3.create(),
        tangent2: vec3.create(),
        friction: 0.5,
        restitution: 0.0,
        numContactPoints: 0,
        contactPoints: [
            createWorldContactPoint(),
            createWorldContactPoint(),
            createWorldContactPoint(),
            createWorldContactPoint(),
        ],
        invMassA: 0,
        invMassB: 0,
        invInertiaA: mat4.identity(mat4.create()),
        invInertiaB: mat4.identity(mat4.create()),
        invInertiaScaleA: 1,
        invInertiaScaleB: 1,
        contactIndex: -1,
        sortKey: 0,
    };
}

/**
 * Apply warm start impulses from previous frame to give solver a good initial guess.
 * This significantly improves convergence speed (~3x faster).
 * @param contactConstraints contact constraint state
 * @param warmStartRatio scale factor for warm start impulses (usually 1.0)
 */
export function warmStartVelocityConstraints(
    contactConstraints: ContactConstraints,
    bodies: Bodies,
    warmStartRatio: number,
): void {
    for (const constraint of contactConstraints.constraints) {
        const bodyA = bodies.pool[constraint.bodyIndexA]!;
        const bodyB = bodies.pool[constraint.bodyIndexB]!;
        const { normal, tangent1, tangent2 } = constraint;
        const mpA = bodyA.motionType === MotionType.DYNAMIC ? bodyA.motionProperties : null;
        const mpB = bodyB.motionType === MotionType.DYNAMIC ? bodyB.motionProperties : null;

        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];

            // warm start friction constraints only if active
            if (axisConstraintPart.isActive(cp.tangentConstraint1) || axisConstraintPart.isActive(cp.tangentConstraint2)) {
                axisConstraintPart.warmStart(
                    cp.tangentConstraint1,
                    bodyA,
                    bodyB,
                    mpA,
                    mpB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent1,
                    warmStartRatio,
                );

                axisConstraintPart.warmStart(
                    cp.tangentConstraint2,
                    bodyA,
                    bodyB,
                    mpA,
                    mpB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent2,
                    warmStartRatio,
                );
            }

            // always warm start normal constraint (non-penetration)
            axisConstraintPart.warmStart(
                cp.normalConstraint,
                bodyA,
                bodyB,
                mpA,
                mpB,
                constraint.invMassA,
                constraint.invMassB,
                normal,
                warmStartRatio,
            );
        }
    }
}

/**
 * Solve velocity constraints for a specific island. Only processes constraints at the given indices.
 * @param contactConstraints contact constraint state
 * @param bodies body array
 * @param constraintIndices indices of constraints to solve (from island)
 */
export function solveVelocityConstraintsForIsland(
    contactConstraints: ContactConstraints,
    bodies: Bodies,
    constraintIndices: number[],
): void {
    if (constraintIndices.length === 0) {
        return;
    }

    // PGS (Projected Gauss-Seidel) solver - one pass
    // CRITICAL ORDER: Friction first, then normal (non-penetration is more important so solved last)

    for (const constraintIndex of constraintIndices) {
        const constraint = contactConstraints.constraints[constraintIndex];
        if (!constraint) continue;

        const bodyA = bodies.pool[constraint.bodyIndexA]!;
        const bodyB = bodies.pool[constraint.bodyIndexB]!;
        const { normal, tangent1, tangent2, friction } = constraint;

        // skip if both bodies are static/kinematic
        if (bodyA.motionType !== MotionType.DYNAMIC && bodyB.motionType !== MotionType.DYNAMIC) {
            continue;
        }

        // solve ALL friction constraints for this constraint
        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];
            if (axisConstraintPart.isActive(cp.tangentConstraint1) || axisConstraintPart.isActive(cp.tangentConstraint2)) {
                let lambda1 = axisConstraintPart.getTotalLambda(cp.tangentConstraint1, bodyA, bodyB, tangent1);
                let lambda2 = axisConstraintPart.getTotalLambda(cp.tangentConstraint2, bodyA, bodyB, tangent2);
                const frictionMagnitudeSq = lambda1 * lambda1 + lambda2 * lambda2;

                const maxFriction = friction * cp.normalConstraint.totalLambda;
                const maxFrictionSq = maxFriction * maxFriction;

                if (frictionMagnitudeSq > maxFrictionSq) {
                    const scale = maxFriction / Math.sqrt(frictionMagnitudeSq);
                    lambda1 = lambda1 * scale;
                    lambda2 = lambda2 * scale;
                }

                axisConstraintPart.applyLambda(
                    cp.tangentConstraint1,
                    bodyA,
                    bodyB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent1,
                    lambda1,
                );
                axisConstraintPart.applyLambda(
                    cp.tangentConstraint2,
                    bodyA,
                    bodyB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent2,
                    lambda2,
                );
            }
        }

        // solve ALL normal (non-penetration) constraints
        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];

            axisConstraintPart.solveVelocityConstraintWithMassOverride(
                cp.normalConstraint,
                bodyA,
                bodyB,
                constraint.invMassA,
                constraint.invMassB,
                normal,
                0,
                Infinity,
            );
        }
    }
}

/**
 * Store solved impulses back to Contact array for warm starting next frame.
 * Must be called after solveVelocityConstraints to persist the solved lambda values.
 *
 * @param contactConstraints contact constraint state
 * @param contactsState contacts state
 */
export function storeAppliedImpulses(contactConstraints: ContactConstraints, contactsState: contacts.Contacts): void {
    for (const constraint of contactConstraints.constraints) {
        const contact = contactsState.contacts[constraint.contactIndex];

        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];
            const cachedPoint = contact.contactPoints[i];

            // store solved impulses for next frame's warm starting
            cachedPoint.normalLambda = cp.normalConstraint.totalLambda;
            cachedPoint.frictionLambda1 = cp.tangentConstraint1.totalLambda;
            cachedPoint.frictionLambda2 = cp.tangentConstraint2.totalLambda;
        }
    }
}

const _solvePos_penetrationVector = /* @__PURE__ */ vec3.create();
const _solvePos_pointA = /* @__PURE__ */ vec3.create();
const _solvePos_pointB = /* @__PURE__ */ vec3.create();
const _solvePos_worldRa = /* @__PURE__ */ vec3.create();
const _solvePos_worldRb = /* @__PURE__ */ vec3.create();
const _solvePos_midpoint = /* @__PURE__ */ vec3.create();
const _solvePos_rA = /* @__PURE__ */ vec3.create();
const _solvePos_rB = /* @__PURE__ */ vec3.create();
const _solvePos_invInertiaA = /* @__PURE__ */ mat4.create();
const _solvePos_invInertiaB = /* @__PURE__ */ mat4.create();
const _solvePos_rotA = /* @__PURE__ */ mat4.create();
const _solvePos_rotB = /* @__PURE__ */ mat4.create();

/**
 * Solve position constraints for a specific island. Only processes constraints at the given indices.
 *
 * @param contactConstraints contact constraint state
 * @param bodies body array
 * @param constraintIndices indices of constraints to solve (from island)
 * @param penetrationSlop allowed penetration before correction
 * @param baumgarteFactor position correction factor 0-1
 * @param maxPenetrationDistance maximum distance to correct in a single iteration
 * @returns true if any impulses were applied
 */
export function solvePositionConstraintsForIsland(
    contactConstraints: ContactConstraints,
    bodies: Bodies,
    constraintIndices: number[],
    penetrationSlop: number,
    baumgarteFactor: number,
    maxPenetrationDistance: number,
): boolean {
    if (constraintIndices.length === 0) {
        return false;
    }

    let anyImpulseApplied = false;

    for (const constraintIndex of constraintIndices) {
        const constraint = contactConstraints.constraints[constraintIndex];
        if (!constraint) continue;

        const bodyA = bodies.pool[constraint.bodyIndexA]!;
        const bodyB = bodies.pool[constraint.bodyIndexB]!;
        const { normal } = constraint;

        // skip if both bodies are static/kinematic
        if (bodyA.motionType !== MotionType.DYNAMIC && bodyB.motionType !== MotionType.DYNAMIC) {
            continue;
        }

        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];

            // transform contact points from local to world space
            vec3.transformQuat(_solvePos_worldRa, cp.localPositionA, bodyA.quaternion);
            vec3.add(_solvePos_pointA, bodyA.centerOfMassPosition, _solvePos_worldRa);

            vec3.transformQuat(_solvePos_worldRb, cp.localPositionB, bodyB.quaternion);
            vec3.add(_solvePos_pointB, bodyB.centerOfMassPosition, _solvePos_worldRb);

            // calculate penetration vector and separation
            vec3.subtract(_solvePos_penetrationVector, _solvePos_pointB, _solvePos_pointA);
            let separation = vec3.dot(_solvePos_penetrationVector, normal) + penetrationSlop;
            separation = Math.max(separation, -maxPenetrationDistance);

            // early exit if not penetrating
            if (separation >= 0) {
                continue;
            }

            // get current world-space inverse inertia (rotation may have changed)
            mat4.fromQuat(_solvePos_rotA, bodyA.quaternion);
            mat4.fromQuat(_solvePos_rotB, bodyB.quaternion);
            getInverseInertiaForRotation(_solvePos_invInertiaA, bodyA.motionProperties, _solvePos_rotA);
            getInverseInertiaForRotation(_solvePos_invInertiaB, bodyB.motionProperties, _solvePos_rotB);

            // calculate midpoint and moment arms
            vec3.add(_solvePos_midpoint, _solvePos_pointA, _solvePos_pointB);
            vec3.scale(_solvePos_midpoint, _solvePos_midpoint, 0.5);
            vec3.subtract(_solvePos_rA, _solvePos_midpoint, bodyA.centerOfMassPosition);
            vec3.subtract(_solvePos_rB, _solvePos_midpoint, bodyB.centerOfMassPosition);

            // recalculate constraint properties with current inertia and moment arms
            axisConstraintPart.calculateConstraintPropertiesWithMassOverride(
                cp.normalConstraint,
                bodyA,
                bodyB,
                constraint.invMassA,
                constraint.invMassB,
                constraint.invInertiaScaleA,
                constraint.invInertiaScaleB,
                _solvePos_invInertiaA,
                _solvePos_invInertiaB,
                _solvePos_rA,
                _solvePos_rB,
                normal,
                0,
            );

            // solve position constraint
            if (
                axisConstraintPart.solvePositionConstraintWithMassOverride(
                    cp.normalConstraint,
                    bodyA,
                    bodyB,
                    constraint.invMassA,
                    constraint.invMassB,
                    normal,
                    separation,
                    baumgarteFactor,
                )
            ) {
                anyImpulseApplied = true;
            }
        }
    }

    return anyImpulseApplied;
}

/**
 * Compute sort key for contact constraint.
 * Uses a simple hash of the body indices and sub-shape IDs.
 */
function computeContactSortKey(bodyIndexA: number, bodyIndexB: number, subShapeA: number, subShapeB: number): number {
    // simple hash combining all IDs
    // TODO: subShapeId pair hash?
    let hash = 0;
    hash = (hash * 31 + bodyIndexA) | 0;
    hash = (hash * 31 + bodyIndexB) | 0;
    hash = (hash * 31 + subShapeA) | 0;
    hash = (hash * 31 + subShapeB) | 0;
    return hash >>> 0; // convert to unsigned
}

/**
 * Sort contact constraint indices for deterministic solving.
 * Sorts by:
 * 1. Sort key (hash of body pair + sub-shapes)
 * 2. Body A ID
 * 3. Body B ID
 */
export function sortContactIndices(contactConstraints: ContactConstraints, bodies: Bodies, contactIndices: number[]): void {
    contactIndices.sort((aIdx, bIdx) => {
        const a = contactConstraints.constraints[aIdx];
        const b = contactConstraints.constraints[bIdx];

        if (!a || !b) return 0;

        // Primary sort: sort key
        if (a.sortKey !== b.sortKey) {
            return a.sortKey - b.sortKey;
        }

        // Secondary sort: body A ID
        const bodyAIdA = bodies.pool[a.bodyIndexA]?.id ?? 0;
        const bodyAIdB = bodies.pool[b.bodyIndexA]?.id ?? 0;
        if (bodyAIdA !== bodyAIdB) {
            return bodyAIdA - bodyAIdB;
        }

        // Tertiary sort: body B ID
        const bodyBIdA = bodies.pool[a.bodyIndexB]?.id ?? 0;
        const bodyBIdB = bodies.pool[b.bodyIndexB]?.id ?? 0;
        return bodyBIdA - bodyBIdB;
    });
}
