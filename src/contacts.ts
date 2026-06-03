import { type Quat, quat, type Vec3, vec3 } from 'mathcat';
import type { RigidBody } from './body/rigid-body';
import type { Bodies } from './body/bodies';
import { getBodyIdIndex } from './body/body-id';
import { EMPTY_SUB_SHAPE_ID } from './body/sub-shape';
import type { Listener } from './listener';

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
export const INVALID_CONTACT_KEY = -1;

/** flags for cached contact manifolds */
export enum CachedManifoldFlags {
    /** no flags set */
    None = 0,
    /** contact was matched and reused */
    ContactPersisted = 1 << 0,
    /** created by CCD */
    CCDContact = 1 << 1,
}

/** creates empty contacts state */
export function init(): Contacts {
    return {
        contacts: [],
        contactsFreeIndices: [],
        readIdx: 0,
        cachedBodyPairs: new Map(),
    };
}

/**
 * Pack a pair of body IDs into a single number key for the cached-body-pair map.
 * Always orders ids ascending so (a, b) and (b, a) hash to the same key.
 * Body IDs are 32-bit; this packs them into the 53-bit-safe integer range.
 */
export function bodyPairKey(idA: number, idB: number): number {
    return idA < idB ? idA * 0x1_0000_0000 + idB : idB * 0x1_0000_0000 + idA;
}

/** create a fresh CachedBodyPair with zeroed deltas */
function createCachedBodyPair(): CachedBodyPair {
    return {
        deltaPosition: vec3.create(),
        deltaRotation: quat.create(),
    };
}

/**
 * Write the current relative pose into the cached-body-pair map, creating the
 * entry if needed. Caller supplies pre-computed deltaPosition / deltaRotation.
 */
export function setCachedBodyPair(
    contactsState: Contacts,
    idA: number,
    idB: number,
    deltaPosition: Vec3,
    deltaRotation: Quat,
): void {
    const key = bodyPairKey(idA, idB);
    let entry = contactsState.cachedBodyPairs.get(key);
    if (!entry) {
        entry = createCachedBodyPair();
        contactsState.cachedBodyPairs.set(key, entry);
    }
    vec3.copy(entry.deltaPosition, deltaPosition);
    quat.copy(entry.deltaRotation, deltaRotation);
}

/** drop the cached-body-pair entry for the given pair, if any. */
export function removeCachedBodyPair(contactsState: Contacts, idA: number, idB: number): void {
    contactsState.cachedBodyPairs.delete(bodyPairKey(idA, idB));
}

/** the manifold buffer that holds last step's cached data (read side). */
export function getReadManifold(contact: Contact, contactsState: Contacts): CachedManifold {
    return contact.manifolds[contactsState.readIdx];
}

/** the manifold buffer being populated this step (write side). */
export function getWriteManifold(contact: Contact, contactsState: Contacts): CachedManifold {
    return contact.manifolds[1 - contactsState.readIdx];
}

/**
 * Swap read/write manifold buffers — call once per step after solving and
 * storeAppliedImpulses, so this step's writes become next step's reads.
 */
export function flipManifoldCache(contactsState: Contacts): void {
    contactsState.readIdx = (1 - contactsState.readIdx) as 0 | 1;
}

/**
 * Pack a contact ID and edge index into a single integer key.
 * Layout: [contactId: 31 bits][edgeIndex: 1 bit]
 *
 * @param contactId - The contact ID (0 to 2^31-1)
 * @param edgeIndex - Which edge (0 for bodyA, 1 for bodyB)
 * @returns Packed integer key
 */
export function packContactKey(contactId: number, edgeIndex: 0 | 1): number {
    return (contactId << 1) | edgeIndex;
}

/** extract contact ID from packed key */
export function getContactKeyId(key: number): number {
    return key >> 1;
}

/** extract edge index from packed key */
export function getContactKeyEdge(key: number): 0 | 1 {
    return (key & 1) as 0 | 1;
}

/** create an empty cached contact point */
function createCachedContactPoint(): CachedContactPoint {
    return {
        position1: vec3.create(),
        position2: vec3.create(),
        normalLambda: 0,
    };
}

/** create an empty cached manifold */
function createEmptyCachedManifold(): CachedManifold {
    return {
        contactNormal: vec3.create(),
        numContactPoints: 0,
        contactPoints: [
            createCachedContactPoint(),
            createCachedContactPoint(),
            createCachedContactPoint(),
            createCachedContactPoint(),
        ],
        frictionLambda1: 0,
        frictionLambda2: 0,
        angularFrictionLambda: 0,
        flags: CachedManifoldFlags.None,
    };
}

/** reset a cached manifold to default empty values */
function resetCachedManifold(m: CachedManifold): void {
    vec3.zero(m.contactNormal);
    m.numContactPoints = 0;
    m.frictionLambda1 = 0;
    m.frictionLambda2 = 0;
    m.angularFrictionLambda = 0;
    m.flags = CachedManifoldFlags.None;
    for (const point of m.contactPoints) {
        vec3.zero(point.position1);
        vec3.zero(point.position2);
        point.normalLambda = 0;
    }
}

/** create an empty contact (used for initialization and pooling) */
function createEmptyContact(): Contact {
    return {
        contactIndex: -1,
        bodyIdA: -1,
        bodyIndexA: -1,
        bodyIdB: -1,
        bodyIndexB: -1,
        subShapeIdA: EMPTY_SUB_SHAPE_ID,
        subShapeIdB: EMPTY_SUB_SHAPE_ID,
        processedThisFrame: false,
        edges: [
            { bodyIndex: -1, prevKey: INVALID_CONTACT_KEY, nextKey: INVALID_CONTACT_KEY },
            { bodyIndex: -1, prevKey: INVALID_CONTACT_KEY, nextKey: INVALID_CONTACT_KEY },
        ],
        manifolds: [createEmptyCachedManifold(), createEmptyCachedManifold()],
    };
}

/** set a contact to initial state with new body/subshape IDs */
function setContact(
    contact: Contact,
    contactId: number,
    bodyA: RigidBody,
    bodyB: RigidBody,
    subShapeIdA: number,
    subShapeIdB: number,
): void {
    contact.contactIndex = contactId;
    contact.bodyIdA = bodyA.id;
    contact.bodyIndexA = bodyA.index;
    contact.bodyIdB = bodyB.id;
    contact.bodyIndexB = bodyB.index;
    contact.subShapeIdA = subShapeIdA;
    contact.subShapeIdB = subShapeIdB;
    contact.processedThisFrame = false;

    // reset both manifold buffers — neither holds valid prior data for a fresh contact
    resetCachedManifold(contact.manifolds[0]);
    resetCachedManifold(contact.manifolds[1]);
}

/**
 * Unlink a contact from a body's contact list.
 * Updates neighboring contacts and the body's headContactKey if needed.
 *
 * @param contacts global contact array
 * @param body body to unlink from
 * @param contact contact to unlink
 * @param edgeIndex which edge to unlink (0 for bodyA, 1 for bodyB)
 */
function unlinkContactFromBody(contacts: Contacts, body: RigidBody, contact: Contact, edgeIndex: 0 | 1): void {
    const edge = contact.edges[edgeIndex];
    const prevKey = edge.prevKey;
    const nextKey = edge.nextKey;

    // update previous contact's nextKey (or body's headContactKey if this was head)
    if (prevKey === INVALID_CONTACT_KEY) {
        // this was the head - update body's head pointer
        body.headContactKey = nextKey;
    } else {
        // update previous contact's next pointer
        const prevContactId = getContactKeyId(prevKey);
        const prevEdgeIndex = getContactKeyEdge(prevKey);
        const prevContact = contacts.contacts[prevContactId];
        prevContact.edges[prevEdgeIndex].nextKey = nextKey;
    }

    // update next contact's prevKey
    if (nextKey !== INVALID_CONTACT_KEY) {
        const nextContactId = getContactKeyId(nextKey);
        const nextEdgeIndex = getContactKeyEdge(nextKey);
        const nextContact = contacts.contacts[nextContactId];
        nextContact.edges[nextEdgeIndex].prevKey = prevKey;
    }

    // clear this edge
    edge.bodyIndex = -1;
    edge.prevKey = INVALID_CONTACT_KEY;
    edge.nextKey = INVALID_CONTACT_KEY;

    body.contactCount--;
}

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
export function createContact(contacts: Contacts, bodyA: RigidBody, bodyB: RigidBody, subShapeIdA: number, subShapeIdB: number): Contact {
    // get contact from pool or create new
    let contactId: number;
    let contact: Contact;

    if (contacts.contactsFreeIndices.length > 0) {
        contactId = contacts.contactsFreeIndices.pop()!;
        contact = contacts.contacts[contactId];
    } else {
        contactId = contacts.contacts.length;
        contact = createEmptyContact();
        contacts.contacts.push(contact);
    }

    // set contact state
    setContact(contact, contactId, bodyA, bodyB, subShapeIdA, subShapeIdB);

    // link into bodyA's contact list (edge 0)
    const edgeA = contact.edges[0];
    edgeA.bodyIndex = bodyA.index;
    edgeA.prevKey = INVALID_CONTACT_KEY;
    edgeA.nextKey = bodyA.headContactKey;

    if (bodyA.headContactKey !== INVALID_CONTACT_KEY) {
        const oldHeadContactId = getContactKeyId(bodyA.headContactKey);
        const oldHeadEdgeIndex = getContactKeyEdge(bodyA.headContactKey);
        const oldHeadContact = contacts.contacts[oldHeadContactId];
        oldHeadContact.edges[oldHeadEdgeIndex].prevKey = packContactKey(contactId, 0);
    }

    bodyA.headContactKey = packContactKey(contactId, 0);
    bodyA.contactCount++;

    // link into bodyB's contact list (edge 1)
    const edgeB = contact.edges[1];
    edgeB.bodyIndex = bodyB.index;
    edgeB.prevKey = INVALID_CONTACT_KEY;
    edgeB.nextKey = bodyB.headContactKey;

    if (bodyB.headContactKey !== INVALID_CONTACT_KEY) {
        const oldHeadContactId = getContactKeyId(bodyB.headContactKey);
        const oldHeadEdgeIndex = getContactKeyEdge(bodyB.headContactKey);
        const oldHeadContact = contacts.contacts[oldHeadContactId];
        oldHeadContact.edges[oldHeadEdgeIndex].prevKey = packContactKey(contactId, 1);
    }

    bodyB.headContactKey = packContactKey(contactId, 1);
    bodyB.contactCount++;

    return contact;
}

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
export function destroyContact(
    contacts: Contacts,
    bodyA: RigidBody,
    bodyB: RigidBody,
    contact: Contact,
    listener: Listener | undefined,
): void {
    // notify listener before destroying
    if (listener?.onContactRemoved) {
        listener.onContactRemoved(contact.bodyIdA, contact.bodyIdB, contact.subShapeIdA, contact.subShapeIdB);
    }

    // determine which body corresponds to which edge based on contact's stored body IDs
    // edge[0] belongs to bodyIdA, edge[1] belongs to bodyIdB
    const edgeABody = bodyA.id === contact.bodyIdA ? bodyA : bodyB;
    const edgeBBody = bodyA.id === contact.bodyIdA ? bodyB : bodyA;

    // unlink from edge 0's body list
    unlinkContactFromBody(contacts, edgeABody, contact, 0);

    // unlink from edge 1's body list
    unlinkContactFromBody(contacts, edgeBBody, contact, 1);

    // save IDs before clearing contact (used for cache cleanup below)
    const pairIdA = contact.bodyIdA;
    const pairIdB = contact.bodyIdB;

    // save contact ID before marking as free
    const contactId = contact.contactIndex;
    contact.contactIndex = -1;
    contacts.contactsFreeIndices.push(contactId);

    // if this was the last contact between the pair, drop the cached-body-pair
    // entry so a future broadphase pair re-enters narrowphase from a clean state.
    if (!hasContactsBetweenBodyIds(contacts, pairIdA, pairIdB)) {
        removeCachedBodyPair(contacts, pairIdA, pairIdB);
    }
}

/**
 * Cheap variant of hasContactsBetweenBodies that doesn't need RigidBody refs —
 * scans the contact pool by id. Used during destroyContact after the contact
 * is unlinked, when the caller might already hold stale RigidBody pointers.
 */
function hasContactsBetweenBodyIds(contacts: Contacts, idA: number, idB: number): boolean {
    for (const c of contacts.contacts) {
        if (c.contactIndex === -1) continue;
        if (
            (c.bodyIdA === idA && c.bodyIdB === idB) ||
            (c.bodyIdA === idB && c.bodyIdB === idA)
        ) {
            return true;
        }
    }
    return false;
}

/**
 * Check if any contacts exist between two bodies.
 * Used to determine if body pair cache should be destroyed.
 *
 * @param contacts global contact array
 * @param bodyA first body
 * @param bodyB second body
 * @returns true if at least one contact exists between the bodies
 */
export function hasContactsBetweenBodies(contacts: Contacts, bodyA: RigidBody, bodyB: RigidBody): boolean {
    // search through the smaller body's contact list
    const searchBody = bodyA.contactCount <= bodyB.contactCount ? bodyA : bodyB;
    let contactKey = searchBody.headContactKey;

    while (contactKey !== INVALID_CONTACT_KEY) {
        const contactId = getContactKeyId(contactKey);
        const edgeIndex = getContactKeyEdge(contactKey);
        const contact = contacts.contacts[contactId];

        // check if this contact involves both bodies
        if (
            (contact.bodyIdA === bodyA.id && contact.bodyIdB === bodyB.id) ||
            (contact.bodyIdA === bodyB.id && contact.bodyIdB === bodyA.id)
        ) {
            return true;
        }

        contactKey = contact.edges[edgeIndex].nextKey;
    }

    return false;
}

/**
 * Destroy all contacts for a specific body.
 * Called when a body is destroyed.
 *
 * @param contacts global contact array
 * @param bodies body array for looking up other bodies
 * @param body body whose contacts should be destroyed
 * @param listener optional contact listener to notify of removal
 */
export function destroyBodyContacts(contacts: Contacts, bodies: Bodies, body: RigidBody, listener?: Listener): void {
    // destroy all contacts
    let contactKey = body.headContactKey;
    while (contactKey !== INVALID_CONTACT_KEY) {
        const contactId = getContactKeyId(contactKey);
        const edgeIndex = getContactKeyEdge(contactKey);
        const contact = contacts.contacts[contactId];

        // save next key before destroying contact (list is mutated)
        contactKey = contact.edges[edgeIndex].nextKey;

        // get the other body
        const otherBodyIndex = edgeIndex === 0 ? contact.edges[1].bodyIndex : contact.edges[0].bodyIndex;
        const otherBody = bodies.pool[otherBodyIndex];

        if (otherBody) {
            destroyContact(contacts, body, otherBody, contact, listener);
        }
    }

    // clear body's contact list
    body.headContactKey = INVALID_CONTACT_KEY;
    body.contactCount = 0;
}

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
export function destroyStaleContactsBetweenBodies(
    contactsState: Contacts,
    bodyA: RigidBody,
    bodyB: RigidBody,
    listener: Listener | undefined,
): void {
    // search through the smaller body's contact list
    const searchBody = bodyA.contactCount <= bodyB.contactCount ? bodyA : bodyB;
    let contactKey = searchBody.headContactKey;

    while (contactKey !== INVALID_CONTACT_KEY) {
        const contactId = getContactKeyId(contactKey);
        const edgeIndex = getContactKeyEdge(contactKey);
        const contact = contactsState.contacts[contactId];

        // save next key before destroying (list is mutated)
        contactKey = contact.edges[edgeIndex].nextKey;

        // check if this contact involves both bodies AND was not processed this frame
        if (
            !contact.processedThisFrame &&
            ((contact.bodyIdA === bodyA.id && contact.bodyIdB === bodyB.id) ||
                (contact.bodyIdA === bodyB.id && contact.bodyIdB === bodyA.id))
        ) {
            destroyContact(contactsState, bodyA, bodyB, contact, listener);
        }
    }
}

/**
 * Destroy all contacts between two bodies (used for unprocessed contacts).
 * More efficient than destroyStaleContactsBetweenBodies when we know ALL contacts should be destroyed.
 *
 * @param contactsState contacts state
 * @param bodyA first body
 * @param bodyB second body
 * @param listener optional contact listener to notify of removal
 */
export function destroyAllContactsBetweenBodies(
    contactsState: Contacts,
    bodyA: RigidBody,
    bodyB: RigidBody,
    listener: Listener | undefined,
): void {
    // search through the smaller body's contact list
    const searchBody = bodyA.contactCount <= bodyB.contactCount ? bodyA : bodyB;
    let contactKey = searchBody.headContactKey;

    while (contactKey !== INVALID_CONTACT_KEY) {
        const contactId = getContactKeyId(contactKey);
        const edgeIndex = getContactKeyEdge(contactKey);
        const contact = contactsState.contacts[contactId];

        // save next key before destroying (list is mutated)
        contactKey = contact.edges[edgeIndex].nextKey;

        // check if this contact involves both bodies
        if (
            (contact.bodyIdA === bodyA.id && contact.bodyIdB === bodyB.id) ||
            (contact.bodyIdA === bodyB.id && contact.bodyIdB === bodyA.id)
        ) {
            destroyContact(contactsState, bodyA, bodyB, contact, listener);
        }
    }
}

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
export function findContact(
    contacts: Contacts,
    bodyA: RigidBody,
    bodyB: RigidBody,
    subShapeIdA: number,
    subShapeIdB: number,
): Contact | null {
    // iterate through the body with fewer contacts for better performance
    const searchBody = bodyA.contactCount <= bodyB.contactCount ? bodyA : bodyB;

    let contactKey = searchBody.headContactKey;

    while (contactKey !== INVALID_CONTACT_KEY) {
        const contactId = getContactKeyId(contactKey);
        const edgeIndex = getContactKeyEdge(contactKey);
        const contact = contacts.contacts[contactId];

        // check if this contact matches the body pair and sub-shapes
        if (
            contact.bodyIdA === bodyA.id &&
            contact.bodyIdB === bodyB.id &&
            contact.subShapeIdA === subShapeIdA &&
            contact.subShapeIdB === subShapeIdB
        ) {
            return contact;
        }

        // move to next contact in list
        contactKey = contact.edges[edgeIndex].nextKey;
    }

    return null;
}

/** marks contacts as unprocessed for the current frame */
export function markAllUnprocessed(contacts: Contacts): void {
    // mark all contacts as unprocessed
    for (const contact of contacts.contacts) {
        if (contact.contactIndex !== -1) {
            contact.processedThisFrame = false;
        }
    }
}

/**
 * Destroy all contacts that weren't processed this frame.
 * Called after all broadphase pairs have been processed.
 * This cleans up stale contacts between bodies that are no longer near each other.
 *
 * @param contacts contacts state
 * @param bodies world bodies array (needed to look up Body objects by ID)
 * @param listener optional contact listener to notify of removal
 */
export function destroyUnprocessedContacts(contacts: Contacts, bodies: Bodies, listener?: Listener): void {
    // iterate through all contacts and destroy unprocessed ones directly
    for (let i = 0; i < contacts.contacts.length; i++) {
        const contact = contacts.contacts[i];

        // skip freed contacts and processed contacts
        if (contact.contactIndex === -1 || contact.processedThisFrame) {
            continue;
        }

        const bodyAIndex = getBodyIdIndex(contact.bodyIdA);
        const bodyBIndex = getBodyIdIndex(contact.bodyIdB);
        const bodyA = bodies.pool[bodyAIndex];
        const bodyB = bodies.pool[bodyBIndex];

        if (bodyA && bodyB && !bodyA._pooled && !bodyB._pooled) {
            destroyContact(contacts, bodyA, bodyB, contact, listener);
        }
    }
}
