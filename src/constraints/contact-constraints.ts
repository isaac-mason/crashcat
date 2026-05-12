import { type Mat4, mat4, type Vec3, vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies';
import { getInverseInertiaForRotation } from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
import * as body from '../body/rigid-body';
import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import * as contacts from '../contacts';
import type { Listener } from '../listener';
import type { ContactManifold } from '../manifold/manifold';
import * as manifold from '../manifold/manifold';
import type { WorldSettings } from '../world-settings';
import { combineMaterial } from './combine-material';
import * as axisConstraintPart from './constraint-part/axis-constraint-part';
import * as contactConstraintPart from './constraint-part/contact-constraint-part';

/** state for contact constraint solving, holds all active constraints and manages constraint lifecycle */
export type ContactConstraints = {
    /** pool of contact constraints (grows as needed, never shrinks) */
    pool: ContactConstraint[];

    /** number of active constraints (first count entries in pool array are valid) */
    count: number;
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
        pool: [],
        count: 0,
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
    out.relativeLinearSurfaceVelocity[0] = source.relativeLinearSurfaceVelocity[0];
    out.relativeLinearSurfaceVelocity[1] = source.relativeLinearSurfaceVelocity[1];
    out.relativeLinearSurfaceVelocity[2] = source.relativeLinearSurfaceVelocity[2];
    out.relativeAngularSurfaceVelocity[0] = source.relativeAngularSurfaceVelocity[0];
    out.relativeAngularSurfaceVelocity[1] = source.relativeAngularSurfaceVelocity[1];
    out.relativeAngularSurfaceVelocity[2] = source.relativeAngularSurfaceVelocity[2];
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
    settings.relativeLinearSurfaceVelocity[0] = 0;
    settings.relativeLinearSurfaceVelocity[1] = 0;
    settings.relativeLinearSurfaceVelocity[2] = 0;
    settings.relativeAngularSurfaceVelocity[0] = 0;
    settings.relativeAngularSurfaceVelocity[1] = 0;
    settings.relativeAngularSurfaceVelocity[2] = 0;
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

const _calcBias_v1 = /* @__PURE__ */ vec3.create();
const _calcBias_v2 = /* @__PURE__ */ vec3.create();

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
    // check for non-static (both dynamic and kinematic contribute velocity)
    const aIsMoving = bodyA.motionType !== MotionType.STATIC;
    const bIsMoving = bodyB.motionType !== MotionType.STATIC;
    const aIsDynamic = bodyA.motionType === MotionType.DYNAMIC;
    const bIsDynamic = bodyB.motionType === MotionType.DYNAMIC;

    let relVelX: number;
    let relVelY: number;
    let relVelZ: number;

    if (aIsMoving && bIsMoving) {
        // both non-static: relative = vB - vA
        body.getVelocityAtPointCOM(_calcBias_v1, bodyA, rA);
        body.getVelocityAtPointCOM(_calcBias_v2, bodyB, rB);
        relVelX = _calcBias_v2[0] - _calcBias_v1[0];
        relVelY = _calcBias_v2[1] - _calcBias_v1[1];
        relVelZ = _calcBias_v2[2] - _calcBias_v1[2];
    } else if (aIsMoving) {
        // only A is non-static: relative = 0 - vA = -vA
        body.getVelocityAtPointCOM(_calcBias_v1, bodyA, rA);
        relVelX = -_calcBias_v1[0];
        relVelY = -_calcBias_v1[1];
        relVelZ = -_calcBias_v1[2];
    } else if (bIsMoving) {
        // only B is non-static: relative = vB - 0 = vB
        body.getVelocityAtPointCOM(_calcBias_v2, bodyB, rB);
        relVelX = _calcBias_v2[0];
        relVelY = _calcBias_v2[1];
        relVelZ = _calcBias_v2[2];
    } else {
        // both static (shouldn't happen for constraints)
        return 0;
    }

    // normalVelocity = relativeVelocity · normal
    const normalVelocity = relVelX * normal[0] + relVelY * normal[1] + relVelZ * normal[2];

    // step 2: calculate penetration depth
    // penetration = (posA - posB) · normal
    // positive = penetrating, negative = separated
    const diffX = positionA[0] - positionB[0];
    const diffY = positionA[1] - positionB[1];
    const diffZ = positionA[2] - positionB[2];
    const penetration = diffX * normal[0] + diffY * normal[1] + diffZ * normal[2];

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
            let relAccelX = 0;
            let relAccelY = 0;
            let relAccelZ = 0;

            // calculate effect of gravity
            // apply to all non-static bodies (both dynamic and kinematic)
            if (aIsMoving && bIsMoving) {
                const gravityFactorA = bodyA.motionProperties.gravityFactor;
                const gravityFactorB = bodyB.motionProperties.gravityFactor;
                const gravityScale = gravityFactorB - gravityFactorA;
                relAccelX += gravity[0] * gravityScale;
                relAccelY += gravity[1] * gravityScale;
                relAccelZ += gravity[2] * gravityScale;
            } else if (aIsMoving) {
                const gravityFactorA = bodyA.motionProperties.gravityFactor;
                relAccelX += gravity[0] * -gravityFactorA;
                relAccelY += gravity[1] * -gravityFactorA;
                relAccelZ += gravity[2] * -gravityFactorA;
            } else if (bIsMoving) {
                const gravityFactorB = bodyB.motionProperties.gravityFactor;
                relAccelX += gravity[0] * gravityFactorB;
                relAccelY += gravity[1] * gravityFactorB;
                relAccelZ += gravity[2] * gravityFactorB;
            }

            // calculate effect of accumulated forces (only for dynamic bodies)
            if (aIsDynamic) {
                const forceScale = -bodyA.motionProperties.invMass;
                relAccelX += bodyA.motionProperties.force[0] * forceScale;
                relAccelY += bodyA.motionProperties.force[1] * forceScale;
                relAccelZ += bodyA.motionProperties.force[2] * forceScale;
            }
            if (bIsDynamic) {
                const forceScale = bodyB.motionProperties.invMass;
                relAccelX += bodyB.motionProperties.force[0] * forceScale;
                relAccelY += bodyB.motionProperties.force[1] * forceScale;
                relAccelZ += bodyB.motionProperties.force[2] * forceScale;
            }

            // we only compensate forces towards the contact normal
            // forceDeltaVelocity = (relativeAcceleration · normal) * deltaTime
            const accelDotNormal = relAccelX * normal[0] + relAccelY * normal[1] + relAccelZ * normal[2];
            const forceDeltaVelocity = Math.min(0, accelDotNormal * deltaTime);

            normalVelocityBias = combinedRestitution * (normalVelocity - forceDeltaVelocity);
        } else {
            // in this case we have predicted that we don't hit the other object, but if we do (due to other constraints changing velocities)
            // the speculative contact will prevent penetration but will not apply restitution
            normalVelocityBias = speculativeContactVelocityBias;
        }
    } else {
        // no restitution. We can safely apply our contact velocity bias.
        normalVelocityBias = speculativeContactVelocityBias;
    }

    return normalVelocityBias;
}

const _addContactConstraint_relativePointOnA = /* @__PURE__ */ vec3.create();
const _addContactConstraint_relativePointOnB = /* @__PURE__ */ vec3.create();
const _addContactConstraint_midpoint = /* @__PURE__ */ vec3.create();
const _addContactConstraint_rA = /* @__PURE__ */ vec3.create();
const _addContactConstraint_rB = /* @__PURE__ */ vec3.create();
const _addContactConstraint_invInertiaA = /* @__PURE__ */ mat4.create();
const _addContactConstraint_invInertiaB = /* @__PURE__ */ mat4.create();
const _addContactConstraint_rotA = /* @__PURE__ */ mat4.create();
const _addContactConstraint_rotB = /* @__PURE__ */ mat4.create();
const _addContactConstraint_contactSettings = /* @__PURE__ */ createContactSettings();

/**
 * add a contact constraint from a new manifold
 *
 * @optimize
 */
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

    // compute rotation matrices for both bodies (used for inverse rotation via transpose and inverse inertia)
    const rotA = mat4.fromQuat(_addContactConstraint_rotA, bodyA.quaternion);
    const rotB = mat4.fromQuat(_addContactConstraint_rotB, bodyB.quaternion);

    // transform the world space normal into body B's local space
    // (R^-1 = R^T for orthogonal matrices)
    mat4.multiply3x3TransposedVec(contact.contactNormal, rotB, contactManifold.worldSpaceNormal);
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

    // if one of the bodies is a sensor, don't actually create the constraint
    // one of the bodies must be dynamic and have mass to be able to create a contact constraint
    if (
        !contactSettings.isSensor &&
        ((bodyA.motionType === MotionType.DYNAMIC && bodyA.motionProperties.invMass !== 0) ||
            (bodyB.motionType === MotionType.DYNAMIC && bodyB.motionProperties.invMass !== 0))
    ) {
        // add contact constraint (grow array if needed)
        if (contactConstraints.count >= contactConstraints.pool.length) {
            contactConstraints.pool.push(createContactConstraint());
        }
        const constraint = contactConstraints.pool[contactConstraints.count];
        contactConstraints.count++;

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

        // compute orthonormal tangent basis from normal
        const normalX = constraint.normal[0];
        const normalY = constraint.normal[1];
        const normalZ = constraint.normal[2];

        if (Math.abs(normalX) > Math.abs(normalY)) {
            const tangentLen = Math.sqrt(normalX * normalX + normalZ * normalZ);
            constraint.tangent1[0] = normalZ / tangentLen;
            constraint.tangent1[1] = 0;
            constraint.tangent1[2] = -normalX / tangentLen;
        } else {
            const tangentLen = Math.sqrt(normalY * normalY + normalZ * normalZ);
            constraint.tangent1[0] = 0;
            constraint.tangent1[1] = normalZ / tangentLen;
            constraint.tangent1[2] = -normalY / tangentLen;
        }

        vec3.cross(constraint.tangent2, constraint.normal, constraint.tangent1);

        // set material properties from settings (may have been modified by listener)
        constraint.friction = contactSettings.combinedFriction;
        constraint.restitution = contactSettings.combinedRestitution;

        // cache body properties (with mass/inertia scaling from settings)
        constraint.invMassA = bodyA.motionProperties.invMass * contactSettings.invMassScale1;
        constraint.invMassB = bodyB.motionProperties.invMass * contactSettings.invMassScale2;
        constraint.invInertiaScaleA = contactSettings.invInertiaScale1;
        constraint.invInertiaScaleB = contactSettings.invInertiaScale2;

        // compute inverse inertia only for dynamic bodies (static/kinematic don't contribute)
        // reuse rotation matrices computed earlier
        if (bodyA.motionType === MotionType.DYNAMIC) {
            const invInertiaA = getInverseInertiaForRotation(_addContactConstraint_invInertiaA, bodyA.motionProperties, rotA);
            const scale1 = contactSettings.invInertiaScale1;
            if (scale1 !== 1) {
                for (let j = 0; j < 16; j++) {
                    constraint.invInertiaA[j] = invInertiaA[j] * scale1;
                }
            } else {
                for (let j = 0; j < 16; j++) {
                    constraint.invInertiaA[j] = invInertiaA[j];
                }
            }
        }

        if (bodyB.motionType === MotionType.DYNAMIC) {
            const invInertiaB = getInverseInertiaForRotation(_addContactConstraint_invInertiaB, bodyB.motionProperties, rotB);
            const scale2 = contactSettings.invInertiaScale2;
            if (scale2 !== 1) {
                for (let j = 0; j < 16; j++) {
                    constraint.invInertiaB[j] = invInertiaB[j] * scale2;
                }
            } else {
                for (let j = 0; j < 16; j++) {
                    constraint.invInertiaB[j] = invInertiaB[j];
                }
            }
        }

        // create contact points with matching
        constraint.numContactPoints = 0;

        for (let i = 0; i < contactManifold.numContactPoints; i++) {
            const cp = constraint.contactPoints[constraint.numContactPoints];
            constraint.numContactPoints++;

            // get relative contact points from manifold
            const contactPointIndex = i * 3;
            const scratchA = _addContactConstraint_relativePointOnA;
            const scratchB = _addContactConstraint_relativePointOnB;

            vec3.fromBuffer(scratchA, contactManifold.relativeContactPointsOnA, contactPointIndex);
            vec3.fromBuffer(scratchB, contactManifold.relativeContactPointsOnB, contactPointIndex);

            vec3.add(cp.positionA, contactManifold.baseOffset, scratchA);
            vec3.add(cp.positionB, contactManifold.baseOffset, scratchB);

            // cp.localPosition* = rot*^T * (cp.position* - bodyCOM); reuse the
            // scratch vec3s now that we're done with the relative points.
            vec3.subtract(scratchA, cp.positionA, bodyA.centerOfMassPosition);
            vec3.subtract(scratchB, cp.positionB, bodyB.centerOfMassPosition);
            mat4.multiply3x3TransposedVec(cp.localPositionA, rotA, scratchA);
            mat4.multiply3x3TransposedVec(cp.localPositionB, rotB, scratchB);

            // check if we have have a close contact point from last update
            // match to cached contact point (first match wins)
            // checks BOTH position1 AND position2 within threshold
            let lambdaSet = false;

            if (existingContact) {
                for (let j = 0; j < existingContact.numContactPoints; j++) {
                    const point = existingContact.contactPoints[j];

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

            vec3.lerp(midpoint, cp.positionA, cp.positionB, 0.5);
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
        const contactPointIndex = i * 3;
        const scratchA = _addContactConstraint_relativePointOnA;
        const scratchB = _addContactConstraint_relativePointOnB;
        const cachedPoint = contact.contactPoints[i];

        // scratch = manifold relative point; then add baseOffset for world; then
        // subtract COM for body-local; then rot^T * local for cached position.
        vec3.fromBuffer(scratchA, contactManifold.relativeContactPointsOnA, contactPointIndex);
        vec3.fromBuffer(scratchB, contactManifold.relativeContactPointsOnB, contactPointIndex);
        vec3.add(scratchA, contactManifold.baseOffset, scratchA);
        vec3.add(scratchB, contactManifold.baseOffset, scratchB);
        vec3.subtract(scratchA, scratchA, bodyA.centerOfMassPosition);
        vec3.subtract(scratchB, scratchB, bodyB.centerOfMassPosition);
        mat4.multiply3x3TransposedVec(cachedPoint.position1, rotA, scratchA);
        mat4.multiply3x3TransposedVec(cachedPoint.position2, rotB, scratchB);

        // reset impulses (sensors don't apply forces)
        cachedPoint.normalLambda = 0;
        cachedPoint.frictionLambda1 = 0;
        cachedPoint.frictionLambda2 = 0;
    }

    // no contact constraint created for sensors
    return false;
}

function createWorldContactPoint(): WorldContactPoint {
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

const _linearVelocityA: Vec3 = [0, 0, 0];
const _angularVelocityA: Vec3 = [0, 0, 0];
const _linearVelocityB: Vec3 = [0, 0, 0];
const _angularVelocityB: Vec3 = [0, 0, 0];

/**
 * apply warm start impulses from previous frame to give solver a good initial guess.
 * significantly improves convergence speed (~3x faster).
 *
 * uses cached velocity locals to avoid repeated body property access.
 * velocities are loaded once per constraint, all contact point warm starts operate on locals,
 * then velocities are written back with DOF masking applied once.
 *
 * @param contactConstraints contact constraint state
 * @param warmStartRatio scale factor for warm start impulses (usually 1.0)
 *
 * @optimize
 */
export function warmStartVelocityConstraints(
    contactConstraints: ContactConstraints,
    bodies: Bodies,
    warmStartRatio: number,
): void {
    for (let i = 0; i < contactConstraints.count; i++) {
        const constraint = contactConstraints.pool[i];
        const bodyA = bodies.pool[constraint.bodyIndexA]!;
        const bodyB = bodies.pool[constraint.bodyIndexB]!;
        const isDynamicA = bodyA.motionType === MotionType.DYNAMIC;
        const isDynamicB = bodyB.motionType === MotionType.DYNAMIC;
        const { normal, tangent1, tangent2 } = constraint;

        // load dynamic body velocities into locals
        // warm start only applies impulses to dynamic bodies, so we only need their velocities
        if (isDynamicA) {
            const mpA = bodyA.motionProperties;
            _linearVelocityA[0] = mpA.linearVelocity[0];
            _linearVelocityA[1] = mpA.linearVelocity[1];
            _linearVelocityA[2] = mpA.linearVelocity[2];
            _angularVelocityA[0] = mpA.angularVelocity[0];
            _angularVelocityA[1] = mpA.angularVelocity[1];
            _angularVelocityA[2] = mpA.angularVelocity[2];
        } else {
            _linearVelocityA[0] = 0;
            _linearVelocityA[1] = 0;
            _linearVelocityA[2] = 0;
            _angularVelocityA[0] = 0;
            _angularVelocityA[1] = 0;
            _angularVelocityA[2] = 0;
        }
        if (isDynamicB) {
            const mpB = bodyB.motionProperties;
            _linearVelocityB[0] = mpB.linearVelocity[0];
            _linearVelocityB[1] = mpB.linearVelocity[1];
            _linearVelocityB[2] = mpB.linearVelocity[2];
            _angularVelocityB[0] = mpB.angularVelocity[0];
            _angularVelocityB[1] = mpB.angularVelocity[1];
            _angularVelocityB[2] = mpB.angularVelocity[2];
        } else {
            _linearVelocityB[0] = 0;
            _linearVelocityB[1] = 0;
            _linearVelocityB[2] = 0;
            _angularVelocityB[0] = 0;
            _angularVelocityB[1] = 0;
            _angularVelocityB[2] = 0;
        }

        for (let j = 0; j < constraint.numContactPoints; j++) {
            const cp = constraint.contactPoints[j];

            // warm start friction constraints only if active
            if (axisConstraintPart.isActive(cp.tangentConstraint1) || axisConstraintPart.isActive(cp.tangentConstraint2)) {
                contactConstraintPart.warmStart(
                    cp.tangentConstraint1,
                    _linearVelocityA,
                    _angularVelocityA,
                    _linearVelocityB,
                    _angularVelocityB,
                    isDynamicA,
                    isDynamicB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent1,
                    warmStartRatio,
                );

                contactConstraintPart.warmStart(
                    cp.tangentConstraint2,
                    _linearVelocityA,
                    _angularVelocityA,
                    _linearVelocityB,
                    _angularVelocityB,
                    isDynamicA,
                    isDynamicB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent2,
                    warmStartRatio,
                );
            }

            // always warm start normal constraint (non-penetration)
            contactConstraintPart.warmStart(
                cp.normalConstraint,
                _linearVelocityA,
                _angularVelocityA,
                _linearVelocityB,
                _angularVelocityB,
                isDynamicA,
                isDynamicB,
                constraint.invMassA,
                constraint.invMassB,
                normal,
                warmStartRatio,
            );
        }

        // write back velocities + DOF masking once
        if (isDynamicA) {
            const mpA = bodyA.motionProperties;
            const allowedTranslationA = mpA.allowedDegreesOfFreedom & 0b111;
            mpA.linearVelocity[0] = allowedTranslationA & 0b001 ? _linearVelocityA[0] : 0;
            mpA.linearVelocity[1] = allowedTranslationA & 0b010 ? _linearVelocityA[1] : 0;
            mpA.linearVelocity[2] = allowedTranslationA & 0b100 ? _linearVelocityA[2] : 0;
            mpA.angularVelocity[0] = _angularVelocityA[0];
            mpA.angularVelocity[1] = _angularVelocityA[1];
            mpA.angularVelocity[2] = _angularVelocityA[2];
        }
        if (isDynamicB) {
            const mpB = bodyB.motionProperties;
            const allowedTranslationB = mpB.allowedDegreesOfFreedom & 0b111;
            mpB.linearVelocity[0] = allowedTranslationB & 0b001 ? _linearVelocityB[0] : 0;
            mpB.linearVelocity[1] = allowedTranslationB & 0b010 ? _linearVelocityB[1] : 0;
            mpB.linearVelocity[2] = allowedTranslationB & 0b100 ? _linearVelocityB[2] : 0;
            mpB.angularVelocity[0] = _angularVelocityB[0];
            mpB.angularVelocity[1] = _angularVelocityB[1];
            mpB.angularVelocity[2] = _angularVelocityB[2];
        }
    }
}

/**
 * solve velocity constraints for a specific island. only processes constraints at the given indices.
 *
 * uses cached velocity locals to avoid repeated body property access during the solve loop.
 * for each constraint: velocities are loaded once, all contact point solves operate on locals,
 * then velocities are written back with DOF masking applied once.
 *
 * @param contactConstraints contact constraint state
 * @param bodies body array
 * @param constraintIndices indices of constraints to solve (from island)
 * @returns true if any impulse was applied (not yet converged)
 *
 * @optimize
 */
export function solveVelocityConstraintsForIsland(
    contactConstraints: ContactConstraints,
    bodies: Bodies,
    constraintIndices: number[],
): boolean {
    // PGS (Projected Gauss-Seidel) solver - one pass
    // CRITICAL ORDER: Friction first, then normal (non-penetration is more important so solved last)

    let anyImpulseApplied = false;

    const _linearVelocityA: Vec3 = [0, 0, 0];
    const _angularVelocityA: Vec3 = [0, 0, 0];
    const _linearVelocityB: Vec3 = [0, 0, 0];
    const _angularVelocityB: Vec3 = [0, 0, 0];

    for (const constraintIndex of constraintIndices) {
        const constraint = contactConstraints.pool[constraintIndex];

        const bodyA = bodies.pool[constraint.bodyIndexA]!;
        const bodyB = bodies.pool[constraint.bodyIndexB]!;

        // cache motion types to avoid repeated checks
        const isDynamicA = bodyA.motionType === MotionType.DYNAMIC;
        const isDynamicB = bodyB.motionType === MotionType.DYNAMIC;

        // skip if both bodies are static/kinematic
        if (!isDynamicA && !isDynamicB) {
            continue;
        }

        // "moving" includes kinematic bodies (they have velocity but aren't mutated)
        const movingA = bodyA.motionType !== MotionType.STATIC;
        const movingB = bodyB.motionType !== MotionType.STATIC;

        const { normal, tangent1, tangent2, friction } = constraint;

        // load body velocities into locals once per constraint
        // for getTotalLambda we need kinematic velocities too (they contribute to Jv)
        // but applyLambda only mutates dynamic bodies
        // so we load all moving bodies' velocities for correct Jv computation
        if (movingA) {
            const mpA = bodyA.motionProperties;
            _linearVelocityA[0] = mpA.linearVelocity[0];
            _linearVelocityA[1] = mpA.linearVelocity[1];
            _linearVelocityA[2] = mpA.linearVelocity[2];
            _angularVelocityA[0] = mpA.angularVelocity[0];
            _angularVelocityA[1] = mpA.angularVelocity[1];
            _angularVelocityA[2] = mpA.angularVelocity[2];
        } else {
            _linearVelocityA[0] = 0;
            _linearVelocityA[1] = 0;
            _linearVelocityA[2] = 0;
            _angularVelocityA[0] = 0;
            _angularVelocityA[1] = 0;
            _angularVelocityA[2] = 0;
        }
        if (movingB) {
            const mpB = bodyB.motionProperties;
            _linearVelocityB[0] = mpB.linearVelocity[0];
            _linearVelocityB[1] = mpB.linearVelocity[1];
            _linearVelocityB[2] = mpB.linearVelocity[2];
            _angularVelocityB[0] = mpB.angularVelocity[0];
            _angularVelocityB[1] = mpB.angularVelocity[1];
            _angularVelocityB[2] = mpB.angularVelocity[2];
        } else {
            _linearVelocityB[0] = 0;
            _linearVelocityB[1] = 0;
            _linearVelocityB[2] = 0;
            _angularVelocityB[0] = 0;
            _angularVelocityB[1] = 0;
            _angularVelocityB[2] = 0;
        }

        // solve friction constraints for this constraint
        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];

            // check if either friction constraint is active
            if (axisConstraintPart.isActive(cp.tangentConstraint1) || axisConstraintPart.isActive(cp.tangentConstraint2)) {
                let lambda1 = contactConstraintPart.getTotalLambda(
                    cp.tangentConstraint1,
                    _linearVelocityA,
                    _angularVelocityA,
                    _linearVelocityB,
                    _angularVelocityB,
                    movingA,
                    movingB,
                    tangent1,
                );
                let lambda2 = contactConstraintPart.getTotalLambda(
                    cp.tangentConstraint2,
                    _linearVelocityA,
                    _angularVelocityA,
                    _linearVelocityB,
                    _angularVelocityB,
                    movingA,
                    movingB,
                    tangent2,
                );

                // project onto friction cone: ||λ_friction|| ≤ μ × λ_normal
                const maxFriction = friction * cp.normalConstraint.totalLambda;
                const frictionMagnitudeSq = lambda1 * lambda1 + lambda2 * lambda2;
                const maxFrictionSq = maxFriction * maxFriction;

                if (frictionMagnitudeSq > maxFrictionSq) {
                    const scale = maxFriction / Math.sqrt(frictionMagnitudeSq);
                    lambda1 *= scale;
                    lambda2 *= scale;
                }

                const appliedFriction1 = contactConstraintPart.applyLambda(
                    cp.tangentConstraint1,
                    _linearVelocityA,
                    _angularVelocityA,
                    _linearVelocityB,
                    _angularVelocityB,
                    isDynamicA,
                    isDynamicB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent1,
                    lambda1,
                );
                anyImpulseApplied = anyImpulseApplied || appliedFriction1;

                const appliedFriction2 = contactConstraintPart.applyLambda(
                    cp.tangentConstraint2,
                    _linearVelocityA,
                    _angularVelocityA,
                    _linearVelocityB,
                    _angularVelocityB,
                    isDynamicA,
                    isDynamicB,
                    constraint.invMassA,
                    constraint.invMassB,
                    tangent2,
                    lambda2,
                );
                anyImpulseApplied = anyImpulseApplied || appliedFriction2;
            }
        }

        // solve normal (non-penetration) constraints
        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];

            const totalLambda = contactConstraintPart.getTotalLambda(
                cp.normalConstraint,
                _linearVelocityA,
                _angularVelocityA,
                _linearVelocityB,
                _angularVelocityB,
                movingA,
                movingB,
                normal,
            );
            // clamp to [0, ∞) — contacts can only push, never pull
            const clampedLambda = Math.max(0, totalLambda);
            const appliedNormal = contactConstraintPart.applyLambda(
                cp.normalConstraint,
                _linearVelocityA,
                _angularVelocityA,
                _linearVelocityB,
                _angularVelocityB,
                isDynamicA,
                isDynamicB,
                constraint.invMassA,
                constraint.invMassB,
                normal,
                clampedLambda,
            );
            anyImpulseApplied = anyImpulseApplied || appliedNormal;
        }

        // write back velocities + DOF masking once per constraint
        if (isDynamicA) {
            const mpA = bodyA.motionProperties;
            const allowedTranslationA = mpA.allowedDegreesOfFreedom & 0b111;
            mpA.linearVelocity[0] = allowedTranslationA & 0b001 ? _linearVelocityA[0] : 0;
            mpA.linearVelocity[1] = allowedTranslationA & 0b010 ? _linearVelocityA[1] : 0;
            mpA.linearVelocity[2] = allowedTranslationA & 0b100 ? _linearVelocityA[2] : 0;
            mpA.angularVelocity[0] = _angularVelocityA[0];
            mpA.angularVelocity[1] = _angularVelocityA[1];
            mpA.angularVelocity[2] = _angularVelocityA[2];
        }
        if (isDynamicB) {
            const mpB = bodyB.motionProperties;
            const allowedTranslationB = mpB.allowedDegreesOfFreedom & 0b111;
            mpB.linearVelocity[0] = allowedTranslationB & 0b001 ? _linearVelocityB[0] : 0;
            mpB.linearVelocity[1] = allowedTranslationB & 0b010 ? _linearVelocityB[1] : 0;
            mpB.linearVelocity[2] = allowedTranslationB & 0b100 ? _linearVelocityB[2] : 0;
            mpB.angularVelocity[0] = _angularVelocityB[0];
            mpB.angularVelocity[1] = _angularVelocityB[1];
            mpB.angularVelocity[2] = _angularVelocityB[2];
        }
    }

    return anyImpulseApplied;
}

/**
 * Store solved impulses back to Contact array for warm starting next frame.
 * Must be called after solveVelocityConstraints to persist the solved lambda values.
 * @param contactConstraints contact constraint state
 * @param contactsState contacts state
 */
export function storeAppliedImpulses(contactConstraints: ContactConstraints, contactsState: contacts.Contacts): void {
    for (let i = 0; i < contactConstraints.count; i++) {
        const constraint = contactConstraints.pool[i];
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

const _solvePos_worldRa = /* @__PURE__ */ vec3.create();
const _solvePos_worldRb = /* @__PURE__ */ vec3.create();
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
 *
 * @optimize
 */
export function solvePositionConstraintsForIsland(
    contactConstraints: ContactConstraints,
    bodies: Bodies,
    constraintIndices: number[],
    penetrationSlop: number,
    baumgarteFactor: number,
    maxPenetrationDistance: number,
): boolean {
    let anyImpulseApplied = false;

    for (const constraintIndex of constraintIndices) {
        const constraint = contactConstraints.pool[constraintIndex];
        if (!constraint) continue;

        const bodyA = bodies.pool[constraint.bodyIndexA]!;
        const bodyB = bodies.pool[constraint.bodyIndexB]!;
        const { normal } = constraint;

        // skip if both bodies are static/kinematic
        if (bodyA.motionType !== MotionType.DYNAMIC && bodyB.motionType !== MotionType.DYNAMIC) {
            continue;
        }

        // get transforms once per constraint (bodies may have moved since last iteration)
        // build rotation matrices that will be reused for both transforming contact points and inverse inertia
        mat4.fromQuat(_solvePos_rotA, bodyA.quaternion);
        mat4.fromQuat(_solvePos_rotB, bodyB.quaternion);

        // get inverse inertia for dynamic bodies (static/kinematic don't contribute)
        if (bodyA.motionType === MotionType.DYNAMIC) {
            getInverseInertiaForRotation(_solvePos_invInertiaA, bodyA.motionProperties, _solvePos_rotA);
        }
        if (bodyB.motionType === MotionType.DYNAMIC) {
            getInverseInertiaForRotation(_solvePos_invInertiaB, bodyB.motionProperties, _solvePos_rotB);
        }

        for (let i = 0; i < constraint.numContactPoints; i++) {
            const cp = constraint.contactPoints[i];

            // transform contact points from local to world space (bodies may have moved)
            mat4.multiply3x3Vec(_solvePos_worldRa, _solvePos_rotA, cp.localPositionA);
            // pointA = bodyA.centerOfMassPosition + _solvePos_worldRa
            const pointAX = bodyA.centerOfMassPosition[0] + _solvePos_worldRa[0];
            const pointAY = bodyA.centerOfMassPosition[1] + _solvePos_worldRa[1];
            const pointAZ = bodyA.centerOfMassPosition[2] + _solvePos_worldRa[2];

            mat4.multiply3x3Vec(_solvePos_worldRb, _solvePos_rotB, cp.localPositionB);
            // pointB = bodyB.centerOfMassPosition + _solvePos_worldRb
            const pointBX = bodyB.centerOfMassPosition[0] + _solvePos_worldRb[0];
            const pointBY = bodyB.centerOfMassPosition[1] + _solvePos_worldRb[1];
            const pointBZ = bodyB.centerOfMassPosition[2] + _solvePos_worldRb[2];

            // calculate penetration vector and separation
            // penetrationVector = pointB - pointA
            const penVecX = pointBX - pointAX;
            const penVecY = pointBY - pointAY;
            const penVecZ = pointBZ - pointAZ;
            // separation = penetrationVector · normal + penetrationSlop
            let separation = penVecX * normal[0] + penVecY * normal[1] + penVecZ * normal[2] + penetrationSlop;
            separation = Math.max(separation, -maxPenetrationDistance);

            // early exit if not penetrating
            if (separation >= 0) {
                continue;
            }

            // calculate midpoint and moment arms
            // midpoint = (pointA + pointB) * 0.5
            const midpointX = (pointAX + pointBX) * 0.5;
            const midpointY = (pointAY + pointBY) * 0.5;
            const midpointZ = (pointAZ + pointBZ) * 0.5;

            // _solvePos_rA = midpoint - bodyA.centerOfMassPosition
            _solvePos_rA[0] = midpointX - bodyA.centerOfMassPosition[0];
            _solvePos_rA[1] = midpointY - bodyA.centerOfMassPosition[1];
            _solvePos_rA[2] = midpointZ - bodyA.centerOfMassPosition[2];

            // _solvePos_rB = midpoint - bodyB.centerOfMassPosition
            _solvePos_rB[0] = midpointX - bodyB.centerOfMassPosition[0];
            _solvePos_rB[1] = midpointY - bodyB.centerOfMassPosition[1];
            _solvePos_rB[2] = midpointZ - bodyB.centerOfMassPosition[2];

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
        const a = contactConstraints.pool[aIdx];
        const b = contactConstraints.pool[bIdx];

        if (!a || !b) return 0;

        // primary sort: sort key
        if (a.sortKey !== b.sortKey) {
            return a.sortKey - b.sortKey;
        }

        // secondary sort: body a id
        const bodyAIdA = bodies.pool[a.bodyIndexA]?.id ?? 0;
        const bodyAIdB = bodies.pool[b.bodyIndexA]?.id ?? 0;
        if (bodyAIdA !== bodyAIdB) {
            return bodyAIdA - bodyAIdB;
        }

        // tertiary sort: body b id
        const bodyBIdA = bodies.pool[a.bodyIndexB]?.id ?? 0;
        const bodyBIdB = bodies.pool[b.bodyIndexB]?.id ?? 0;
        return bodyBIdA - bodyBIdB;
    });
}
