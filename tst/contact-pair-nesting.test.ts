import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    type ContactManifold,
    type Listener,
    MotionQuality,
    MotionType,
    pairs,
    type RigidBody,
    rigidBody,
    sphere,
    updateWorld,
} from '../src';
import { createTestWorld } from './helpers';

/** live contacts nested under the pair record for (a, b), or -1/0 if no record */
function pairContactCount(world: ReturnType<typeof createTestWorld>['world'], a: RigidBody, b: RigidBody): number {
    const rec = pairs.findPairRecord(world.pairs, a, b);
    if (rec === -1) return 0;
    let count = 0;
    let id = world.pairs.firstContact[rec];
    while (id !== -1) {
        count++;
        id = world.contacts.contacts[id].nextInPair;
    }
    return count;
}

describe('Contact/pair nesting: sleep lifecycle', () => {
    // A stack settling to sleep must fire onContactRemoved exactly once per contact on the first
    // sleeping step (the sweep sees both bodies inactive -> dedup rejects -> kept-not-emitted ->
    // chain destroyed), then stay silent (steady-state sleeping pair has an empty chain). Waking
    // must re-add contacts cleanly (the read-site chain-non-empty guard blocks a cache-hit into
    // zero contacts even though the pose cache may still read valid).
    test('removed fires once on the first sleeping step; wake re-adds cleanly', () => {
        const { world, layers } = createTestWorld();

        // static floor, top face at y = 0
        const floor = rigidBody.create(world, {
            shape: box.create({ halfExtents: [10, 0.5, 10] }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        // dynamic box resting on the floor
        const boxBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5], density: 1000 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0),
        });

        let added = 0;
        let removed = 0;
        const listener: Listener = {
            onContactAdded: () => added++,
            onContactRemoved: () => removed++,
        };

        // settle to sleep
        let steps = 0;
        for (; steps < 600 && !boxBody.sleeping; steps++) {
            updateWorld(world, listener, 1 / 60);
        }
        expect(boxBody.sleeping).toBe(true);
        // the body is asleep now, but the sweep that destroys the chain runs on the NEXT step,
        // so no removal has fired for the sleep transition yet
        const contactsAtSleep = pairContactCount(world, boxBody, floor);
        expect(contactsAtSleep).toBeGreaterThan(0);

        // isolate the sleep-transition removal and the steady state
        added = 0;
        removed = 0;

        // first sleeping step: both bodies inactive -> chain destroyed, removal fired exactly once
        // per live contact
        updateWorld(world, listener, 1 / 60);
        expect(removed).toBe(contactsAtSleep);
        expect(boxBody.sleeping).toBe(true);
        expect(pairContactCount(world, boxBody, floor)).toBe(0);

        // steady-state sleeping: chain already empty, no further removals
        for (let i = 0; i < 15; i++) updateWorld(world, listener, 1 / 60);
        expect(removed).toBe(contactsAtSleep);
        expect(added).toBe(0);

        // wake: contacts must re-add cleanly (clean narrowphase, not a cache-hit into zero contacts)
        added = 0;
        removed = 0;
        const yBefore = boxBody.position[1];
        rigidBody.wake(world, boxBody);
        updateWorld(world, listener, 1 / 60);

        expect(boxBody.sleeping).toBe(false);
        expect(added).toBeGreaterThan(0);
        expect(pairContactCount(world, boxBody, floor)).toBeGreaterThan(0);
        // position stayed sane: the box did not tunnel through the floor on wake
        expect(boxBody.position[1]).toBeCloseTo(yBefore, 1);
    });
});

describe('Contact/pair nesting: CCD contact for an undiscovered pair', () => {
    // A fast LINEAR_CAST body can collide mid-step with a body its fat AABB never overlapped at the
    // start of the step, so the broadphase sweep never created a pair record. onCCDContactAdded must
    // find-or-create the record so the CCD contact still nests under a live pair, and the next step
    // must reconcile it through the normal path.
    test('CCD hit creates the pair record and nests the contact, reconciled next step', () => {
        const { world, layers } = createTestWorld();

        // thin static wall spanning x in [-0.1, 0.1]
        const wall = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.1, 5, 5), density: 1000 }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // fast bullet, clearly separated from the wall (no fat-AABB overlap at frame start, so the
        // broadphase never discovers this pair)
        const bullet = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.2, density: 1000 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(-5, 0, 0),
        });
        bullet.motionProperties.motionQuality = MotionQuality.LINEAR_CAST;
        vec3.set(bullet.motionProperties.linearVelocity, 300, 0, 0);

        let added = 0;
        const listener: Listener = {
            onContactAdded: (_a: RigidBody, _b: RigidBody, _m: ContactManifold) => {
                added++;
            },
        };

        // sanity: no pair record exists before the step (the pair is undiscovered)
        expect(pairs.findPairRecord(world.pairs, wall, bullet)).toBe(-1);

        updateWorld(world, listener, 1 / 60);

        // CCD produced a contact -> the pair record was find-or-created, with the contact nested
        expect(added).toBe(1);
        const rec = pairs.findPairRecord(world.pairs, wall, bullet);
        expect(rec).not.toBe(-1);
        expect(pairContactCount(world, wall, bullet)).toBe(1);
        // it actually stopped at the wall (did not tunnel through)
        expect(bullet.position[0]).toBeLessThan(0);

        // next step: the bullet now overlaps the wall's fat AABB, so the sweep keeps the record and
        // narrowphase reconciles the pair normally (record survives, no crash, still not tunneled)
        updateWorld(world, listener, 1 / 60);
        expect(pairs.findPairRecord(world.pairs, wall, bullet)).not.toBe(-1);
        expect(bullet.position[0]).toBeLessThan(0);
    });
});
