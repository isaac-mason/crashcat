import {
    addBroadphaseLayer,
    addObjectLayer,
    contacts,
    createWorld,
    createWorldSettings,
    enableCollision,
    pairs,
    type RigidBody,
} from '../src';
import type { World } from '../src/world';

export const createTestWorld = () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    return {
        world,
        layers: {
            BROADPHASE_LAYER_MOVING,
            BROADPHASE_LAYER_NOT_MOVING,
            OBJECT_LAYER_MOVING,
            OBJECT_LAYER_NOT_MOVING,
        },
    };
};

/**
 * Count the live contacts involving a body by walking its persistent pair edges
 * (body.headPairKey) and, for each pair record, its nested contact chain
 * (world.pairs.firstContact[rec] -> Contact.nextInPair). Replaces the deleted
 * body.contactCount field for tests.
 */
export const bodyContactCount = (world: World, body: RigidBody): number => {
    let count = 0;
    let edgeKey = body.headPairKey;
    while (edgeKey !== pairs.INVALID_PAIR_KEY) {
        const rec = pairs.getPairEdgeRecord(edgeKey);
        const side = pairs.getPairEdgeSide(edgeKey);

        let contactId = world.pairs.firstContact[rec];
        while (contactId !== contacts.INVALID_CONTACT_KEY) {
            count++;
            contactId = world.contacts.contacts[contactId].nextInPair;
        }

        // next edge in this body's pair list: [prev, next] live at RECORD_STRIDE offset 2 + side*2
        edgeKey = world.pairs.records[rec * pairs.RECORD_STRIDE + 2 + side * 2 + 1];
    }
    return count;
};
