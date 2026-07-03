import { type Quat, type Vec3 } from 'mathcat';
import type { RigidBody } from './body/rigid-body.js';
import type { Bodies } from './body/bodies.js';
import type { Listener } from './listener.js';
/** contacts state */
export type Contacts = {
    /** packed array of all contacts (active + free) */
    contacts: Contact[];
    /** free list of available contact indices (indices to reuse) */
    contactsFreeIndices: number[];
    /**
     * Index of the manifold buffer that holds the PREVIOUS step's cached data
     * (read side this step).
     *
     * Flip once per step (after solver + storeAppliedImpulses) so that this
     * step's writes become next step's reads. The other buffer (1 - readIdx)
     * is the WRITE side this step.
     */
    readIdx: 0 | 1;
    /**
     * Per-body-pair cache of last frame's relative pose (body B's COM and
     * orientation expressed in body A's local frame). Used to decide whether
     * the narrowphase can be skipped this frame and last frame's manifolds
     * reused verbatim.
     *
     * Key is packed via `bodyPairKey(idA, idB)` (lower id first).
     */
    cachedBodyPairs: Map<number, CachedBodyPair>;
    /**
     * Deferred onContactRemoved event payloads, flat quads
     * [bodyIdA, bodyIdB, subShapeIdA, subShapeIdB, ...].
     *
     * Removing a body destroys its contacts immediately, but the contact listener is an
     * updateWorld argument — so the removal events are queued here as plain ids and fired
     * at the start of the next updateWorld (matching jolt: "you'll receive an
     * OnContactRemoved callback in the update after the body has been removed").
     */
    pendingContactRemoved: number[];
};
/**
 * Per-body-pair cache entry — last frame's relative pose between two bodies.
 *
 * Per-sub-shape-pair manifolds live on the persistent `Contact` records;
 * we walk a body's contact edge list to find them at cache-hit time.
 */
export type CachedBodyPair = {
    /** body B's COM position relative to body A, expressed in body A's local frame */
    deltaPosition: Vec3;
    /** body B's orientation relative to body A, i.e. `inv(rA) * rB` */
    deltaRotation: Quat;
};
/**
 * Cached manifold data — the per-step state we read from last frame and write
 * to this frame.
 *
 * Each Contact double-buffers this (see Contact.manifolds) so reads from the
 * previous step don't alias writes to the current step. This matters for
 * warm-start lambda matching, which compares new contact-point local positions
 * against the previous step's positions.
 */
export type CachedManifold = {
    /** contact normal in body B's local space */
    contactNormal: Vec3;
    /** number of contact points (0-4) */
    numContactPoints: number;
    /** contact points (max 4 for stable manifold) */
    contactPoints: CachedContactPoint[];
    /**
     * Accumulated friction impulse along tangent1 for the entire manifold
     * (used for warm starting next step).
     */
    frictionLambda1: number;
    /**
     * Accumulated friction impulse along tangent2 for the entire manifold
     * (used for warm starting next step).
     */
    frictionLambda2: number;
    /**
     * Accumulated angular friction impulse around the contact normal for the
     * entire manifold (used for warm starting next step).
     */
    angularFrictionLambda: number;
    /** flags bitfield (@see CachedManifoldFlags) */
    flags: number;
};
/** contact between two shapes */
export type Contact = {
    /** contact index (index in contacts.contacts when active, -1 when freed) */
    contactIndex: number;
    /** body A ID (always <= bodyIdB for consistent ordering) */
    bodyIdA: number;
    /** body A index */
    bodyIndexA: number;
    /** body B ID (always >= bodyIdA for consistent ordering) */
    bodyIdB: number;
    /** body B index */
    bodyIndexB: number;
    /** sub-shape A ID */
    subShapeIdA: number;
    /** sub-shape B ID */
    subShapeIdB: number;
    /** whether this contact was processed this frame (for stale contact cleanup) */
    processedThisFrame: boolean;
    /** two edges for intrusive doubly-linked list: edges[0] = edge in bodyA's contact list, edges[1] = edge in bodyB's contact list */
    edges: [ContactEdge, ContactEdge];
    /**
     * Double-buffered cached manifold (read / write side per step).
     * Use getReadManifold / getWriteManifold to access via the Contacts.readIdx
     * flip — never index directly.
     */
    manifolds: [CachedManifold, CachedManifold];
};
/** cached contact point with non-penetration impulse history for warm starting */
export type CachedContactPoint = {
    /** contact position in body A's local center of mass space */
    position1: Vec3;
    /** contact position in body B's local center of mass space */
    position2: Vec3;
    /** accumulated normal impulse from previous frame */
    normalLambda: number;
};
/** contact edge in the intrusive doubly-linked list, each contact has two edges (one per body) */
export type ContactEdge = {
    /** index of the body this edge belongs to */
    bodyIndex: number;
    /** packed key to previous contact in this body's list (or INVALID_CONTACT_KEY) */
    prevKey: number;
    /** packed key to next contact in this body's list (or INVALID_CONTACT_KEY) */
    nextKey: number;
};
/** invalid contact key constant - used to mark end of linked list */
export declare const INVALID_CONTACT_KEY = -1;
/** flags for cached contact manifolds */
export declare enum CachedManifoldFlags {
    /** no flags set */
    None = 0,
    /** contact was matched and reused */
    ContactPersisted = 1,
    /** created by CCD */
    CCDContact = 2
}
/** creates empty contacts state */
export declare function init(): Contacts;
/**
 * Pack a pair of body IDs into a single number key for the cached-body-pair map.
 * Always orders ids ascending so (a, b) and (b, a) hash to the same key.
 * Body IDs are 32-bit; this packs them into the 53-bit-safe integer range.
 */
/** fire and clear deferred onContactRemoved events (queued by body removal) */
export declare function flushPendingContactRemoved(contacts: Contacts, listener: Listener | undefined): void;
export declare function bodyPairKey(idA: number, idB: number): number;
/**
 * Write the current relative pose into the cached-body-pair map, creating the
 * entry if needed. Caller supplies pre-computed deltaPosition / deltaRotation.
 */
export declare function setCachedBodyPair(contactsState: Contacts, idA: number, idB: number, deltaPosition: Vec3, deltaRotation: Quat): void;
/** drop the cached-body-pair entry for the given pair, if any. */
export declare function removeCachedBodyPair(contactsState: Contacts, idA: number, idB: number): void;
/** the manifold buffer that holds last step's cached data (read side). */
export declare function getReadManifold(contact: Contact, contactsState: Contacts): CachedManifold;
/** the manifold buffer being populated this step (write side). */
export declare function getWriteManifold(contact: Contact, contactsState: Contacts): CachedManifold;
/**
 * Swap read/write manifold buffers — call once per step after solving and
 * storeAppliedImpulses, so this step's writes become next step's reads.
 */
export declare function flipManifoldCache(contactsState: Contacts): void;
/**
 * Pack a contact ID and edge index into a single integer key.
 * Layout: [contactId: 31 bits][edgeIndex: 1 bit]
 *
 * @param contactId - The contact ID (0 to 2^31-1)
 * @param edgeIndex - Which edge (0 for bodyA, 1 for bodyB)
 * @returns Packed integer key
 */
export declare function packContactKey(contactId: number, edgeIndex: 0 | 1): number;
/** extract contact ID from packed key */
export declare function getContactKeyId(key: number): number;
/** extract edge index from packed key */
export declare function getContactKeyEdge(key: number): 0 | 1;
/**
 * Create a new contact between two bodies.
 * Links the contact into both bodies' contact lists.
 *
 * @param contacts global contact array
 * @param bodyA first body (must have id <= bodyB.id)
 * @param bodyB second body (must have id >= bodyA.id)
 * @param subShapeIdA sub-shape ID for body A
 * @param subShapeIdB sub-shape ID for body B
 * @returns The newly created contact
 */
export declare function createContact(contacts: Contacts, bodyA: RigidBody, bodyB: RigidBody, subShapeIdA: number, subShapeIdB: number): Contact;
/**
 * Destroy a contact, unlinking it from both bodies' contact lists.
 * Returns the contact to the free list for reuse.
 *
 * @param contacts global contact array
 * @param bodyA first body in contact
 * @param bodyB second body in contact
 * @param contact contact to destroy
 * @param listener optional contact listener to notify of removal
 */
export declare function destroyContact(contacts: Contacts, bodyA: RigidBody, bodyB: RigidBody, contact: Contact, listener: Listener | undefined): void;
/**
 * Check if any contacts exist between two bodies.
 * Used to determine if body pair cache should be destroyed.
 *
 * @param contacts global contact array
 * @param bodyA first body
 * @param bodyB second body
 * @returns true if at least one contact exists between the bodies
 */
export declare function hasContactsBetweenBodies(contacts: Contacts, bodyA: RigidBody, bodyB: RigidBody): boolean;
/**
 * Destroy all contacts for a specific body.
 * Called when a body is destroyed.
 *
 * @param contacts global contact array
 * @param bodies body array for looking up other bodies
 * @param body body whose contacts should be destroyed
 * @param listener optional contact listener to notify of removal
 */
export declare function destroyBodyContacts(contacts: Contacts, bodies: Bodies, body: RigidBody): void;
/**
 * Destroy stale contacts (those not processed this frame) between two bodies.
 *
 * After narrowphase processes a body pair, any contacts that weren't marked as processed
 * are stale (sub-shapes no longer colliding) and should be destroyed.
 *
 * @param contactsState contacts state
 * @param bodyA first body
 * @param bodyB second body
 * @param listener optional contact listener to notify of removal
 */
export declare function destroyStaleContactsBetweenBodies(contactsState: Contacts, bodyA: RigidBody, bodyB: RigidBody, listener: Listener | undefined): void;
/**
 * Destroy all contacts between two bodies (used for unprocessed contacts).
 * More efficient than destroyStaleContactsBetweenBodies when we know ALL contacts should be destroyed.
 *
 * @param contactsState contacts state
 * @param bodyA first body
 * @param bodyB second body
 * @param listener optional contact listener to notify of removal
 */
export declare function destroyAllContactsBetweenBodies(contactsState: Contacts, bodyA: RigidBody, bodyB: RigidBody, listener: Listener | undefined): void;
/**
 * Find a contact between two bodies with specific sub-shapes.
 * Iterates through the smaller body's contact list (O(n) but typically small n).
 *
 * @param contacts global contact array
 * @param bodyA first body
 * @param bodyB second body
 * @param subShapeIdA sub-shape ID for body A
 * @param subShapeIdB sub-shape ID for body B
 * @returns The contact if found, null otherwise
 */
export declare function findContact(contacts: Contacts, bodyA: RigidBody, bodyB: RigidBody, subShapeIdA: number, subShapeIdB: number): Contact | null;
/** marks contacts as unprocessed for the current frame */
export declare function markAllUnprocessed(contacts: Contacts): void;
/**
 * Destroy all contacts that weren't processed this frame.
 * Called after all broadphase pairs have been processed.
 * This cleans up stale contacts between bodies that are no longer near each other.
 *
 * @param contacts contacts state
 * @param bodies world bodies array (needed to look up Body objects by ID)
 * @param listener optional contact listener to notify of removal
 */
export declare function destroyUnprocessedContacts(contacts: Contacts, bodies: Bodies, listener?: Listener): void;
