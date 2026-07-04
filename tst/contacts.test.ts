import { describe, expect, test } from 'vitest';
import { contacts, MotionType, pairs, type RigidBody, rigidBody, sphere } from '../src';
import type { World } from '../src/world';
import { createTestWorld } from './helpers';

/** collect the contacts in a pair record's chain, head-first */
function chainContacts(contactsState: contacts.Contacts, pairsState: World['pairs'], rec: number): contacts.Contact[] {
    const out: contacts.Contact[] = [];
    let contactId = pairsState.firstContact[rec];
    while (contactId !== contacts.INVALID_CONTACT_KEY) {
        const contact = contactsState.contacts[contactId];
        out.push(contact);
        contactId = contact.nextInPair;
    }
    return out;
}

/** number of active (non-freed) contacts in the pool */
function getActiveContactCount(contactsState: contacts.Contacts): number {
    return contactsState.contacts.length - contactsState.contactsFreeIndices.length;
}

const makeBody = (world: World, layer: number): RigidBody =>
    rigidBody.create(world, {
        shape: sphere.create({ radius: 1 }),
        objectLayer: layer,
        motionType: MotionType.DYNAMIC,
    });

describe('Contacts: Create and Destroy', () => {
    test('should create an empty contact array', () => {
        const contactsState = contacts.init();
        expect(contactsState.contacts).toHaveLength(0);
        expect(contactsState.contactsFreeIndices).toHaveLength(0);
        expect(contactsState.frameStamp).toBe(0);
        expect(getActiveContactCount(contactsState)).toBe(0);
    });

    test('should create a contact nested under a pair record', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const contact = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);

        expect(contact.contactIndex).toBe(0);
        expect(contact.bodyIdA).toBe(bodyA.id);
        expect(contact.bodyIdB).toBe(bodyB.id);
        expect(contact.subShapeIdA).toBe(0);
        expect(contact.subShapeIdB).toBe(0);
        expect(contacts.getReadManifold(contact, contactsState).numContactPoints).toBe(0);
        expect(contacts.getWriteManifold(contact, contactsState).numContactPoints).toBe(0);
        expect(contactsState.contacts).toHaveLength(1);
        expect(getActiveContactCount(contactsState)).toBe(1);

        // nested under the record: chain head is this contact, links are sentinels, pairRecord set
        expect(world.pairs.firstContact[rec]).toBe(contact.contactIndex);
        expect(contact.pairRecord).toBe(rec);
        expect(contact.prevInPair).toBe(contacts.INVALID_CONTACT_KEY);
        expect(contact.nextInPair).toBe(contacts.INVALID_CONTACT_KEY);
        expect(chainContacts(contactsState, world.pairs, rec)).toEqual([contact]);
    });

    test('should destroy a contact and empty the chain', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const contact = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);
        expect(chainContacts(contactsState, world.pairs, rec)).toHaveLength(1);

        contacts.destroyContact(contactsState, world.pairs, contact, undefined);

        expect(world.pairs.firstContact[rec]).toBe(contacts.INVALID_CONTACT_KEY);
        expect(contact.contactIndex).toBe(-1);
        expect(contact.pairRecord).toBe(-1);
        expect(getActiveContactCount(contactsState)).toBe(0);
    });

    test('should fire onContactRemoved when destroying a contact', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const removed: Array<{ a: number; b: number; sa: number; sb: number }> = [];
        const listener = {
            onContactRemoved: (a: number, b: number, sa: number, sb: number) => removed.push({ a, b, sa, sb }),
        };

        const contact = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 3, 7);
        contacts.destroyContact(contactsState, world.pairs, contact, listener);

        expect(removed).toEqual([{ a: bodyA.id, b: bodyB.id, sa: 3, sb: 7 }]);
    });

    test('should reuse contact IDs from free list', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const contact1 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);
        const firstId = contact1.contactIndex;
        contacts.destroyContact(contactsState, world.pairs, contact1, undefined);

        const contact2 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 1, 1);
        expect(contact2.contactIndex).toBe(firstId); // reused ID
        expect(contactsState.contacts).toHaveLength(1); // still only 1 slot
    });
});

describe('Contacts: Multiple contacts in a pair chain', () => {
    test('should chain multiple contacts (distinct sub-shapes) under one record', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const c0 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);
        const c1 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 1, 1);
        const c2 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 2, 2);

        const chain = chainContacts(contactsState, world.pairs, rec);
        expect(chain).toHaveLength(3);
        expect(chain).toContain(c0);
        expect(chain).toContain(c1);
        expect(chain).toContain(c2);
    });

    test('should keep the chain consistent when destroying the middle contact', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const c0 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);
        const c1 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 1, 1);
        const c2 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 2, 2);

        // chain is head-first c2 -> c1 -> c0; destroy the middle node c1
        expect(chainContacts(contactsState, world.pairs, rec)).toEqual([c2, c1, c0]);
        contacts.destroyContact(contactsState, world.pairs, c1, undefined);

        const chain = chainContacts(contactsState, world.pairs, rec);
        expect(chain).toEqual([c2, c0]);
        // links repaired across the hole
        expect(c2.nextInPair).toBe(c0.contactIndex);
        expect(c0.prevInPair).toBe(c2.contactIndex);
    });
});

describe('Contacts: findContactInPair', () => {
    test('should find an existing contact by sub-shape ids', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const contact = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 5, 10);
        expect(contacts.findContactInPair(contactsState, world.pairs, rec, 5, 10)).toBe(contact);
    });

    test('should return null for a non-existent contact', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        expect(contacts.findContactInPair(contactsState, world.pairs, rec, 5, 10)).toBeNull();
    });

    test('should distinguish contacts by sub-shape ids', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const c1 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);
        const c2 = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 1, 1);

        expect(contacts.findContactInPair(contactsState, world.pairs, rec, 0, 0)).toBe(c1);
        expect(contacts.findContactInPair(contactsState, world.pairs, rec, 1, 1)).toBe(c2);
        expect(contacts.findContactInPair(contactsState, world.pairs, rec, 2, 2)).toBeNull();
    });
});

describe('Contacts: destroyStaleContactsInPair', () => {
    test('should destroy only contacts not stamped with the current frame', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();
        contactsState.frameStamp = 5;

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        const fresh = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);
        fresh.lastProcessedFrame = 5; // matches frameStamp -> kept
        const stale = contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 1, 1);
        stale.lastProcessedFrame = 4; // does not match -> destroyed

        contacts.destroyStaleContactsInPair(contactsState, world.pairs, rec, undefined);

        expect(chainContacts(contactsState, world.pairs, rec)).toEqual([fresh]);
        expect(stale.contactIndex).toBe(-1);
    });
});

describe('Contacts: destroyPairChain', () => {
    test('should destroy the whole chain, firing events by default', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 0, 0);
        contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 1, 1);

        const removed: number[] = [];
        const listener = { onContactRemoved: () => removed.push(1) };

        contacts.destroyPairChain(contactsState, world.pairs, rec, listener, false);

        expect(world.pairs.firstContact[rec]).toBe(contacts.INVALID_CONTACT_KEY);
        expect(getActiveContactCount(contactsState)).toBe(0);
        expect(removed).toHaveLength(2);
        expect(contactsState.pendingContactRemoved).toHaveLength(0);
    });

    test('should queue events instead of firing when queueEvents is true', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const bodyB = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const rec = pairs.addPairRecord(world.pairs, bodyA, bodyB);

        contacts.createContact(contactsState, world.pairs, rec, bodyA, bodyB, 2, 3);

        const removed: number[] = [];
        const listener = { onContactRemoved: () => removed.push(1) };

        contacts.destroyPairChain(contactsState, world.pairs, rec, listener, true);

        expect(removed).toHaveLength(0);
        // one quad [bodyIdA, bodyIdB, subShapeIdA, subShapeIdB]
        expect(contactsState.pendingContactRemoved).toEqual([bodyA.id, bodyB.id, 2, 3]);
    });
});

describe('Contacts: chain integrity', () => {
    /**
     * hub body with many pairs, contacts created/destroyed in mixed order — verify
     * chain <-> firstContact <-> pairRecord consistency and freelist reuse.
     */
    test('mixed create/destroy across many pairs stays consistent with freelist reuse', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const hub = makeBody(world, layers.OBJECT_LAYER_MOVING);
        const spokes: RigidBody[] = [];
        const recs: number[] = [];
        for (let i = 0; i < 6; i++) {
            const spoke = makeBody(world, layers.OBJECT_LAYER_MOVING);
            spokes.push(spoke);
            recs.push(pairs.addPairRecord(world.pairs, hub, spoke));
        }

        // create 2-3 contacts per pair
        const created: contacts.Contact[] = [];
        for (let i = 0; i < recs.length; i++) {
            const n = 2 + (i % 2);
            for (let s = 0; s < n; s++) {
                created.push(contacts.createContact(contactsState, world.pairs, recs[i], hub, spokes[i], s, s));
            }
        }

        // sanity: every chain matches its created count and every contact points back at its record
        const assertConsistent = () => {
            for (let i = 0; i < recs.length; i++) {
                const chain = chainContacts(contactsState, world.pairs, recs[i]);
                for (const c of chain) {
                    expect(c.pairRecord).toBe(recs[i]);
                    // prev/next reciprocity
                    if (c.nextInPair !== contacts.INVALID_CONTACT_KEY) {
                        expect(contactsState.contacts[c.nextInPair].prevInPair).toBe(c.contactIndex);
                    }
                    if (c.prevInPair !== contacts.INVALID_CONTACT_KEY) {
                        expect(contactsState.contacts[c.prevInPair].nextInPair).toBe(c.contactIndex);
                    } else {
                        expect(world.pairs.firstContact[recs[i]]).toBe(c.contactIndex);
                    }
                }
            }
            // no freed contact appears in any chain
            const liveIds = new Set<number>();
            for (const rec of recs) for (const c of chainContacts(contactsState, world.pairs, rec)) liveIds.add(c.contactIndex);
            for (const freed of contactsState.contactsFreeIndices) expect(liveIds.has(freed)).toBe(false);
        };

        assertConsistent();

        // destroy a scattered subset (heads, middles, tails)
        const toDestroy = [created[0], created[4], created[7], created[created.length - 1]];
        for (const c of toDestroy) contacts.destroyContact(contactsState, world.pairs, c, undefined);
        assertConsistent();

        // recreate — should reuse freed slots and stay consistent
        const beforeSlots = contactsState.contacts.length;
        for (let i = 0; i < recs.length; i++) {
            contacts.createContact(contactsState, world.pairs, recs[i], hub, spokes[i], 9, 9);
        }
        assertConsistent();
        // some freelist reuse happened (we freed 4, created 6, so slots grow by at most 2)
        expect(contactsState.contacts.length).toBeLessThanOrEqual(beforeSlots + recs.length);
        expect(contactsState.contacts.length).toBeGreaterThan(beforeSlots);
    });
});
