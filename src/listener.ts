import type { Vec3 } from 'mathcat';
import type { RigidBody } from './body/rigid-body';
import type { CollideShapeHit } from './collision/collide-shape-vs-shape';
import type { ContactSettings } from './constraints/contact-constraints';
import type { ContactManifold } from './manifold';

/**
 * Result returned by onContactValidate callback.
 * Determines if contact is processed and whether to continue validation.
 */
export enum ContactValidateResult {
    /** Accept this and all future contacts for this body pair (stop calling callback) */
    ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR = 0,
    /** Accept this contact, continue validating future contacts */
    ACCEPT_CONTACT = 1,
    /** Reject this contact, continue validating future contacts */
    REJECT_CONTACT = 2,
    /** Reject this and all future contacts (early-out from collision detection) */
    REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR = 3,
}

/** A listener that recieves physics events during world update and can modify behavior */
export type Listener = {
    /**
     * Called after broadphase determines a potential collision between two bodies, but before narrowphase collision detection.
     * This is the most efficient place to filter body pairs since narrowphase hasn't run yet.
     * Use this for custom filtering logic like ragdoll self-collision, faction systems, one-way platforms, etc.
     *
     * @param bodyA - First body (motion type >= bodyB.motionType, or same motion type with id < bodyB.id)
     * @param bodyB - Second body
     * @returns true if the bodies should collide, false to skip collision detection for this pair
     */
    onBodyPairValidate?: (bodyA: RigidBody, bodyB: RigidBody) => boolean;

    /**
     * Called after detecting a collision between a body pair, but before calling onContactAdded and before adding the contact constraint.
     * If the function rejects the contact, the contact will not be added and any other contacts between this body pair will not be processed.
     * This function will only be called once per world update per body pair and may not be called again the next update
     * if a contact persists and no new contact pairs between sub shapes are found.
     * This is a rather expensive time to reject a contact point since a lot of the collision detection has happened already, make sure you
     * filter out the majority of undesired body pairs through object layer filters.
     *
     * @param bodyA - First body (motion type >= bodyB.motionType, or same motion type with id < bodyB.id)
     * @param bodyB - Second body
     * @param baseOffset - Base offset (bodyA's position/center of mass) to which contact points are relative
     * @param hit - Collision result from narrowphase detection
     * @returns ValidateResult indicating whether to accept/reject this contact
     */
    onContactValidate?: (bodyA: RigidBody, bodyB: RigidBody, baseOffset: Vec3, hit: CollideShapeHit) => ContactValidateResult;

    /**
     * Called whenever a new contact point is detected.
     * Body 1 and 2 will be sorted such that body 1 ID < body 2 ID, so body 1 may not be dynamic.
     * Note that only active bodies will report contacts, as soon as a body goes to sleep the contacts between that body and all other
     * bodies will receive an onContactRemoved callback, if this is the case then body.sleeping will be true during the callback.
     * When contacts are added, the constraint solver has not run yet, so the collision impulse is unknown at that point.
     * The velocities of bodyA and bodyB are the velocities before the contact has been resolved, so you can use this to
     * estimate the collision impulse to e.g. determine the volume of the impact sound to play.
     *
     * @param bodyA - First body (ID < bodyB.id)
     * @param bodyB - Second body (ID > bodyA.id)
     * @param manifold - Contact manifold containing contact points and collision normal
     * @param settings - Contact settings that can be modified to customize contact behavior
     */
    onContactAdded?: (bodyA: RigidBody, bodyB: RigidBody, manifold: ContactManifold, settings: ContactSettings) => void;

    /**
     * Called whenever a contact is detected that was also detected last update.
     * Body 1 and 2 will be sorted such that body 1 ID < body 2 ID, so body 1 may not be dynamic.
     * If the structure of the shape of a body changes between simulation steps (e.g. by adding/removing a child shape of a compound shape),
     * it is possible that the same sub shape ID used to identify the removed child shape is now reused for a different child shape. The physics
     * system cannot detect this, so may send a 'contact persisted' callback even though the contact is now on a different child shape. You can
     * detect this by keeping the old shape (before adding/removing a part) around until the next world update (when the onContactPersisted
     * callbacks are triggered) and resolving the sub shape ID against both the old and new shape to see if they still refer to the same child shape.
     *
     * @param bodyA - First body (ID < bodyB.id)
     * @param bodyB - Second body (ID > bodyA.id)
     * @param manifold - Contact manifold containing contact points and collision normal
     * @param settings - Contact settings that can be modified to customize contact behavior
     */
    onContactPersisted?: (bodyA: RigidBody, bodyB: RigidBody, manifold: ContactManifold, settings: ContactSettings) => void;

    /**
     * Called whenever a contact was detected last update but is not detected anymore.
     * You should NOT access body properties during this callback because:
     * - The body may have been removed and destroyed (you'll receive an onContactRemoved callback in the world update after the body has been removed).
     * Cache what you need in the onContactAdded and onContactPersisted callbacks and store it in a separate structure to use during this callback.
     * Alternatively, you could just record that the contact was removed and process it after the world update.
     * Body IDs are sorted such that bodyIdA < bodyIdB, so bodyA may not be dynamic.
     * The sub shape IDs were created in the previous simulation step too, so if the structure of a shape changes (e.g. by adding/removing a child shape of a compound shape),
     * the sub shape ID may not be valid / may not point to the same sub shape anymore.
     *
     * @param bodyIdA - First body ID (< bodyIdB)
     * @param bodyIdB - Second body ID (> bodyIdA)
     * @param subShapeIdA - Sub shape ID for bodyA
     * @param subShapeIdB - Sub shape ID for bodyB
     */
    onContactRemoved?: (bodyIdA: number, bodyIdB: number, subShapeIdA: number, subShapeIdB: number) => void;
};
