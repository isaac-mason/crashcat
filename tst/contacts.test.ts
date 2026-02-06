import { describe, expect, test } from 'vitest';
import { contacts, MotionType, type RigidBody, rigidBody, sphere } from '../src';
import { createTestWorld } from './helpers';

/**
 * Iterate through all contacts for a specific body.
 * @param contacts contacts state
 * @param body body to iterate contacts for
 * @param edgeIndex which edge to follow (0 for bodyA, 1 for bodyB) - optional, will detect automatically
 */
function* iterateBodyContacts(contactState: contacts.Contacts, body: RigidBody): Generator<contacts.Contact> {
    let contactKey = body.headContactKey;

    while (contactKey !== contacts.INVALID_CONTACT_KEY) {
        const contactId = contacts.getContactKeyId(contactKey);
        const currentEdgeIndex = contacts.getContactKeyEdge(contactKey);
        const contact = contactState.contacts[contactId];

        yield contact;

        // move to next contact
        contactKey = contact.edges[currentEdgeIndex].nextKey;
    }
}

/**
 * Get the number of active contacts in the array.
 *
 * @param contactsState contacts state
 * @returns number of active contacts
 */
function getActiveContactCount(contactsState: contacts.Contacts): number {
    return contactsState.contacts.length - contactsState.contactsFreeIndices.length;
}

describe('Contacts: Key Encoding', () => {
    test('should pack and unpack contact keys correctly', () => {
        const contactId = 42;
        const edgeIndex = 0;

        const key = contacts.packContactKey(contactId, edgeIndex);
        expect(contacts.getContactKeyId(key)).toBe(contactId);
        expect(contacts.getContactKeyEdge(key)).toBe(edgeIndex);
    });

    test('should handle edge index 1', () => {
        const contactId = 100;
        const edgeIndex = 1;

        const key = contacts.packContactKey(contactId, edgeIndex);
        expect(contacts.getContactKeyId(key)).toBe(contactId);
        expect(contacts.getContactKeyEdge(key)).toBe(edgeIndex);
    });

    test('should handle large contact IDs', () => {
        const contactId = 1_000_000;
        const edgeIndex = 1;

        const key = contacts.packContactKey(contactId, edgeIndex);
        expect(contacts.getContactKeyId(key)).toBe(contactId);
        expect(contacts.getContactKeyEdge(key)).toBe(edgeIndex);
    });
});

describe('Contacts: Create and Destroy', () => {
    test('should create an empty contact array', () => {
        const contactsState = contacts.init();
        expect(contactsState.contacts).toHaveLength(0);
        expect(contactsState.contactsFreeIndices).toHaveLength(0);
        expect(getActiveContactCount(contactsState)).toBe(0);
    });

    test('should create a contact between two bodies', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const contact = contacts.createContact(contactsState, bodyA, bodyB, 0, 0);

        expect(contact.contactIndex).toBe(0);
        expect(contact.bodyIdA).toBe(bodyA.id);
        expect(contact.bodyIdB).toBe(bodyB.id);
        expect(contact.subShapeIdA).toBe(0);
        expect(contact.subShapeIdB).toBe(0);
        expect(contact.numContactPoints).toBe(0);
        expect(contactsState.contacts).toHaveLength(1);
        expect(getActiveContactCount(contactsState)).toBe(1);

        // Check bodies are linked
        expect(bodyA.headContactKey).toBe(contacts.packContactKey(0, 0));
        expect(bodyA.contactCount).toBe(1);
        expect(bodyB.headContactKey).toBe(contacts.packContactKey(0, 1));
        expect(bodyB.contactCount).toBe(1);

        // Check edges
        expect(contact.edges[0].bodyIndex).toBe(bodyA.index);
        expect(contact.edges[0].prevKey).toBe(contacts.INVALID_CONTACT_KEY);
        expect(contact.edges[0].nextKey).toBe(contacts.INVALID_CONTACT_KEY);
        expect(contact.edges[1].bodyIndex).toBe(bodyB.index);
        expect(contact.edges[1].prevKey).toBe(contacts.INVALID_CONTACT_KEY);
        expect(contact.edges[1].nextKey).toBe(contacts.INVALID_CONTACT_KEY);
    });

    test('should destroy a contact and update linked lists', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const contact = contacts.createContact(contactsState, bodyA, bodyB, 0, 0);
        expect(bodyA.contactCount).toBe(1);
        expect(bodyB.contactCount).toBe(1);

        contacts.destroyContact(contactsState, bodyA, bodyB, contact, undefined);

        expect(bodyA.headContactKey).toBe(contacts.INVALID_CONTACT_KEY);
        expect(bodyA.contactCount).toBe(0);
        expect(bodyB.headContactKey).toBe(contacts.INVALID_CONTACT_KEY);
        expect(bodyB.contactCount).toBe(0);
        expect(contact.contactIndex).toBe(-1);
        expect(getActiveContactCount(contactsState)).toBe(0);
    });

    test('should reuse contact IDs from free list', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Create and destroy contact
        const contact1 = contacts.createContact(contactsState, bodyA, bodyB, 0, 0);
        const firstId = contact1.contactIndex;
        contacts.destroyContact(contactsState, bodyA, bodyB, contact1, undefined);

        // Create new contact - should reuse ID
        const contact2 = contacts.createContact(contactsState, bodyA, bodyB, 1, 1);
        expect(contact2.contactIndex).toBe(firstId); // Reused ID
        expect(contactsState.contacts).toHaveLength(1); // Still only 1 slot
    });
});

describe('Contacts: Multiple Contacts', () => {
    test('should handle multiple contacts per body', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyC = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Create contacts: A-B, A-C
        const contactAB = contacts.createContact(contactsState, bodyA, bodyB, 0, 0);
        const contactAC = contacts.createContact(contactsState, bodyA, bodyC, 0, 0);

        // Body A should have 2 contacts
        expect(bodyA.contactCount).toBe(2);
        expect(bodyB.contactCount).toBe(1);
        expect(bodyC.contactCount).toBe(1);

        // Verify linked list structure for body A
        const contactsA = Array.from(iterateBodyContacts(contactsState, bodyA));
        expect(contactsA).toHaveLength(2);
        expect(contactsA).toContain(contactAB);
        expect(contactsA).toContain(contactAC);
    });

    test('should maintain correct linked list when destroying middle contact', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyC = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyD = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Create contacts: A-B, A-C, A-D (all in body A's list)
        const contactAB = contacts.createContact(contactsState, bodyA, bodyB, 0, 0);
        const contactAC = contacts.createContact(contactsState, bodyA, bodyC, 0, 0);
        const contactAD = contacts.createContact(contactsState, bodyA, bodyD, 0, 0);

        expect(bodyA.contactCount).toBe(3);

        // Destroy middle contact (A-C)
        contacts.destroyContact(contactsState, bodyA, bodyC, contactAC, undefined);

        // Body A should have 2 contacts
        expect(bodyA.contactCount).toBe(2);

        // Verify remaining contacts are still linked
        const contactsA = Array.from(iterateBodyContacts(contactsState, bodyA));
        expect(contactsA).toHaveLength(2);
        expect(contactsA).toContain(contactAB);
        expect(contactsA).toContain(contactAD);
        expect(contactsA).not.toContain(contactAC);
    });
});

describe('Contacts: Find Contact', () => {
    test('should find existing contact', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const contact = contacts.createContact(contactsState, bodyA, bodyB, 5, 10);

        const found = contacts.findContact(contactsState, bodyA, bodyB, 5, 10);
        expect(found).toBe(contact);
    });

    test('should return null for non-existent contact', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const found = contacts.findContact(contactsState, bodyA, bodyB, 5, 10);
        expect(found).toBeNull();
    });

    test('should distinguish contacts by sub-shape IDs', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Create two contacts with different sub-shape IDs
        const contact1 = contacts.createContact(contactsState, bodyA, bodyB, 0, 0);
        const contact2 = contacts.createContact(contactsState, bodyA, bodyB, 1, 1);

        // Should find correct contacts
        expect(contacts.findContact(contactsState, bodyA, bodyB, 0, 0)).toBe(contact1);
        expect(contacts.findContact(contactsState, bodyA, bodyB, 1, 1)).toBe(contact2);
        expect(contacts.findContact(contactsState, bodyA, bodyB, 2, 2)).toBeNull();
    });
});

describe('Contacts: Iteration', () => {
    test('should iterate through all contacts for a body', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyC = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const contactAB = contacts.createContact(contactsState, bodyA, bodyB, 0, 0);
        const contactAC = contacts.createContact(contactsState, bodyA, bodyC, 0, 0);

        const bodyContacts = Array.from(iterateBodyContacts(contactsState, bodyA));
        expect(bodyContacts).toHaveLength(2);
        expect(bodyContacts).toContain(contactAB);
        expect(bodyContacts).toContain(contactAC);
    });

    test('should return empty iterator for body with no contacts', () => {
        const { world, layers } = createTestWorld();
        const contactsState = contacts.init();

        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const bodyContacts = Array.from(iterateBodyContacts(contactsState, bodyA));
        expect(bodyContacts).toHaveLength(0);
    });
});
