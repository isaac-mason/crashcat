import { type Vec3 } from 'mathcat';
import type { RigidBody } from './body/rigid-body.js';
import type { Listener } from './listener.js';
import type { Pairs } from './pairs.js';
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
     * Deferred onContactRemoved event payloads, flat quads
     * [bodyIdA, bodyIdB, subShapeIdA, subShapeIdB, ...].
     *
     * Removing a body destroys its contacts immediately, but the contact listener is an
     * updateWorld argument — so the removal events are queued here as plain ids and fired
     * at the start of the next updateWorld (matching jolt: "you'll receive an
     * OnContactRemoved callback in the update after the body has been removed").
     */
    pendingContactRemoved: number[];
    /**
     * Monotonic per-step frame counter, incremented once at the top of updateWorld. A contact is
     * fresh this step iff its lastProcessedFrame === frameStamp; the per-pair stale reconcile uses
     * this instead of a per-step "unprocessed" reset walk over every contact.
     */
    frameStamp: number;
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
/**
 * contact between two shapes.
 *
 * every live contact is nested under exactly one persistent pair record: it lives in that record's
 * chain (headed by pairs.firstContact[pairRecord]) via the prevInPair/nextInPair links. the chain is
 * doubly-linked (unlike jolt's singly-linked manifold chain) so a contact can be unlinked in O(1)
 * when destroyed in place.
 */
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
    /** frameStamp of the last step that processed this contact (for stale contact cleanup); init 0 */
    lastProcessedFrame: number;
    /** owning pair record index; -1 when freed */
    pairRecord: number;
    /** previous contact index in the owning pair's chain (or INVALID_CONTACT_KEY) */
    prevInPair: number;
    /** next contact index in the owning pair's chain (or INVALID_CONTACT_KEY) */
    nextInPair: number;
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
/** invalid contact index constant - used to mark end of a pair's contact chain */
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
/** fire and clear deferred onContactRemoved events (queued by body removal) */
export declare function flushPendingContactRemoved(contacts: Contacts, listener: Listener | undefined): void;
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
 * Create a new contact nested under a pair record, linking it at the head of the record's chain.
 *
 * @param contacts global contact state
 * @param pairs persistent pair state (holds the per-record chain heads)
 * @param rec owning pair record index
 * @param bodyA first body (must have id <= bodyB.id)
 * @param bodyB second body (must have id >= bodyA.id)
 * @param subShapeIdA sub-shape ID for body A
 * @param subShapeIdB sub-shape ID for body B
 * @returns The newly created contact
 */
export declare function createContact(contacts: Contacts, pairs: Pairs, rec: number, bodyA: RigidBody, bodyB: RigidBody, subShapeIdA: number, subShapeIdB: number): Contact;
/**
 * Destroy a contact: fire onContactRemoved, unlink from its pair chain (O(1)), and return the slot
 * to the free list. No body args, no pool scan, no cache bookkeeping — the pair chain is the only
 * navigation.
 *
 * @param contacts global contact state
 * @param pairs persistent pair state (holds the per-record chain heads)
 * @param contact contact to destroy
 * @param listener optional contact listener to notify of removal
 */
export declare function destroyContact(contacts: Contacts, pairs: Pairs, contact: Contact, listener: Listener | undefined): void;
/**
 * Destroy every contact in a pair record's chain.
 *
 * When `queueEvents` is true the removal payloads are pushed onto pendingContactRemoved instead of
 * fired (the body-removal path, where the listener is unavailable — same pattern as the deferred
 * body-removal events). The whole chain is freed and the record's head reset to empty.
 *
 * @param contacts global contact state
 * @param pairs persistent pair state (holds the per-record chain heads)
 * @param rec pair record index whose chain should be destroyed
 * @param listener optional contact listener to notify of removal (ignored when queueEvents)
 * @param queueEvents queue removal events onto pendingContactRemoved instead of firing them
 */
export declare function destroyPairChain(contacts: Contacts, pairs: Pairs, rec: number, listener: Listener | undefined, queueEvents: boolean): void;
/**
 * Destroy stale contacts (lastProcessedFrame !== frameStamp) in a pair record's chain.
 *
 * After narrowphase processes an emitted pair, any chain contacts not re-stamped this frame are
 * stale (sub-shapes no longer colliding) and are destroyed. The chain IS the pair, so no body-id
 * filtering is needed.
 *
 * @param contacts global contact state
 * @param pairs persistent pair state (holds the per-record chain heads)
 * @param rec pair record index whose chain should be reconciled
 * @param listener optional contact listener to notify of removal
 */
export declare function destroyStaleContactsInPair(contacts: Contacts, pairs: Pairs, rec: number, listener: Listener | undefined): void;
/**
 * Find a contact in a pair record's chain by sub-shape ids.
 *
 * All chain contacts share the id-sorted body pair, so the caller's existing body-order sort covers
 * orientation — only the two sub-shape ids need matching.
 *
 * @param contacts global contact state
 * @param pairs persistent pair state (holds the per-record chain heads)
 * @param rec pair record index to search
 * @param subShapeIdA sub-shape ID for body A
 * @param subShapeIdB sub-shape ID for body B
 * @returns The contact if found, null otherwise
 */
export declare function findContactInPair(contacts: Contacts, pairs: Pairs, rec: number, subShapeIdA: number, subShapeIdB: number): Contact | null;
