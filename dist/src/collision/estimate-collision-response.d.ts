import { type Vec3 } from 'math';
import type { RigidBody } from '../body/rigid-body.js';
import type { ContactSettings } from '../constraints/contact-constraints.js';
import type { ContactManifold } from '../manifold/manifold.js';
/**
 * Result of estimating collision response between two bodies.
 *
 * Contains predicted post-collision velocities, per-point normal (contact) impulses,
 * and the manifold-level friction impulses (matching the per-manifold friction model
 * used by the real solver).
 */
export type CollisionEstimationResult = {
    /** predicted post-collision linear velocity of body 1 */
    linearVelocity1: Vec3;
    /** predicted post-collision angular velocity of body 1 */
    angularVelocity1: Vec3;
    /** predicted post-collision linear velocity of body 2 */
    linearVelocity2: Vec3;
    /** predicted post-collision angular velocity of body 2 */
    angularVelocity2: Vec3;
    /**
     * Where friction acts: halfway between the two bodies' friction points (each the mean of that body's
     * contact points), relative to `manifold.baseOffset` like the manifold's relativeContactPoints arrays.
     */
    frictionPoint: Vec3;
    /** per-contact-point normal impulse (kg⋅m/s) */
    contactImpulse: number[];
    /** manifold-level linear friction impulse (kg⋅m/s), in the contact plane */
    frictionImpulse: Vec3;
    /** manifold-level angular friction impulse around the contact normal */
    angularFrictionImpulse: number;
    /** number of contact points (length of contactImpulse) */
    numImpulses: number;
};
/** create a new collision estimation result */
export declare function createCollisionEstimationResult(): CollisionEstimationResult;
/**
 * Estimate the contact and friction impulses of a collision, and the body velocities they leave, before the solver
 * runs: for the strength of an impact in onContactAdded, to play a sound or spawn particles. Accurate when two bodies
 * collide, not when more collide at once, as it knows only this contact.
 *
 * @param result result object to write to
 * @param body1 first body
 * @param body2 second body
 * @param manifold contact manifold
 * @param settings the contact's settings: its materials, mass and inertia scales and surface velocities (a sensor has no response)
 * @param minVelocityForRestitution minimum relative velocity to apply restitution (default 1.0 m/s)
 * @param numIterations number of PGS iterations (default 10); with 1, no friction is calculated
 */
export declare function estimateCollisionResponse(result: CollisionEstimationResult, body1: RigidBody, body2: RigidBody, manifold: ContactManifold, settings: ContactSettings, minVelocityForRestitution?: number, numIterations?: number): void;
