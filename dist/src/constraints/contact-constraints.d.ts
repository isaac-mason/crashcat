import { type Vec3 } from 'mathcat';
import type { Bodies } from '../body/bodies.js';
import * as body from '../body/rigid-body.js';
import * as contacts from '../contacts.js';
import type { Listener } from '../listener.js';
import type { ContactManifold } from '../manifold/manifold.js';
import type { Pairs } from '../pairs.js';
import type { WorldSettings } from '../world-settings.js';
import * as angularFrictionConstraintPart from './constraint-part/angular-friction-constraint-part.js';
import * as axisConstraintPart from './constraint-part/axis-constraint-part.js';
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
    /**
     * Average contact point ("friction point") in world space — unweighted mean of contact
     * midpoints. The two linear friction constraints are anchored at this point.
     */
    frictionPoint: Vec3;
    /**
     * Linear friction constraint along tangent1, anchored at frictionPoint.
     * Lambda clamping (joint with frictionConstraint2):
     * λ1² + λ2² ≤ (μ · Σ_i normalLambda_i)²
     */
    frictionConstraint1: axisConstraintPart.AxisConstraintPart;
    /**
     * Linear friction constraint along tangent2, anchored at frictionPoint.
     * Lambda clamping (joint with frictionConstraint1):
     * λ1² + λ2² ≤ (μ · Σ_i normalLambda_i)²
     */
    frictionConstraint2: axisConstraintPart.AxisConstraintPart;
    /**
     * Angular friction constraint around the contact normal.
     * Lambda clamping: |λ_angular| ≤ μ · Σ_i (distanceToFrictionCenter_i · normalLambda_i).
     * Deactivated for single-contact manifolds.
     */
    angularFrictionConstraint: angularFrictionConstraintPart.AngularFrictionConstraintPart;
    /** inverse mass of body A (1/massA), 0 if body is static. cached from body at constraint setup time */
    invMassA: number;
    /** inverse mass of body B (1/massB), 0 if body is static. cached from body at constraint setup time */
    invMassB: number;
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
 * A single contact point with non-penetration constraint data.
 * Links world-space geometry to constraint solving.
 *
 * Friction is no longer per-contact-point — it lives on the parent
 * ContactConstraint (anchored at frictionPoint). Each WorldContactPoint
 * only owns the 1-DOF non-penetration constraint plus a cached moment-arm
 * to the friction point for computing the angular friction cap.
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
     * Distance from this contact point to the manifold's friction point, projected onto
     * the plane perpendicular to the contact normal. Used as the moment arm when computing
     * the angular friction cap: μ · Σ_i (distanceToFrictionCenter_i · normalLambda_i).
     */
    distanceToFrictionCenter: number;
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
export declare function init(): ContactConstraints;
/** create a contact settings object */
export declare function createContactSettings(): ContactSettings;
/** copy contact settings properties from a source object */
export declare function copyContactSettings(out: ContactSettings, source: ContactSettings): ContactSettings;
/**
 * add a contact constraint from a new manifold
 */
export declare function addContactConstraint(contactConstraints: ContactConstraints, contactsState: contacts.Contacts, pairs: Pairs, pairRecordIndex: number, bodyA: body.RigidBody, bodyB: body.RigidBody, contactManifold: ContactManifold, settings: WorldSettings, contactListener: Listener | undefined, deltaTime: number): boolean;
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
export declare function warmStartVelocityConstraints(contactConstraints: ContactConstraints, bodies: Bodies, warmStartRatio: number): void;
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
export declare function solveVelocityConstraintsForIsland(contactConstraints: ContactConstraints, bodies: Bodies, constraintIndices: number[]): boolean;
/**
 * Store solved impulses back to Contact array for warm starting next frame.
 * Must be called after solveVelocityConstraints to persist the solved lambda values.
 * @param contactConstraints contact constraint state
 * @param contactsState contacts state
 */
export declare function storeAppliedImpulses(contactConstraints: ContactConstraints, contactsState: contacts.Contacts): void;
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
export declare function solvePositionConstraintsForIsland(contactConstraints: ContactConstraints, bodies: Bodies, constraintIndices: number[], penetrationSlop: number, baumgarteFactor: number, maxPenetrationDistance: number): boolean;
/**
 * Sort contact constraint indices for deterministic solving.
 * Sorts by:
 * 1. Sort key (hash of body pair + sub-shapes)
 * 2. Body A ID
 * 3. Body B ID
 */
export declare function sortContactIndices(contactConstraints: ContactConstraints, bodies: Bodies, contactIndices: number[]): void;
