import { type Vec3 } from 'math';
import type { RigidBody } from '../body/rigid-body.js';
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
    /** first friction tangent direction (perpendicular to normal) */
    tangent1: Vec3;
    /** second friction tangent direction (normal × tangent1) */
    tangent2: Vec3;
    /**
     * Average ("friction") contact point — unweighted mean of contact midpoints, in the
     * same space as `manifold.baseOffset` (i.e. relative to baseOffset, like the manifold
     * relativeContactPoints arrays). Friction impulses act at this point.
     */
    frictionPoint: Vec3;
    /** per-contact-point normal impulse (kg⋅m/s) */
    contactImpulse: number[];
    /** manifold-level linear friction impulse along tangent1 */
    frictionImpulse1: number;
    /** manifold-level linear friction impulse along tangent2 */
    frictionImpulse2: number;
    /** manifold-level angular friction impulse around the contact normal */
    angularFrictionImpulse: number;
    /** number of contact points (length of contactImpulse) */
    numImpulses: number;
};
/** create a new collision estimation result */
export declare function createCollisionEstimationResult(): CollisionEstimationResult;
/**
 * estimate collision response between two bodies
 *
 * predicts post-collision velocities and impulses by running a mini PGS solver
 * on a local copy of the velocities. designed to be called from onContactAdded
 * to estimate impact strength before the actual solver runs.
 *
 * @param result result object to write to
 * @param body1 first body
 * @param body2 second body
 * @param manifold contact manifold
 * @param combinedFriction combined friction coefficient
 * @param combinedRestitution combined restitution coefficient
 * @param minVelocityForRestitution minimum relative velocity to apply restitution (default 1.0 m/s)
 * @param numIterations number of PGS iterations (default 10)
 */
export declare function estimateCollisionResponse(result: CollisionEstimationResult, body1: RigidBody, body2: RigidBody, manifold: ContactManifold, combinedFriction: number, combinedRestitution: number, minVelocityForRestitution?: number, numIterations?: number): void;
