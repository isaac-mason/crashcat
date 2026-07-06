import { describe, expect, test } from 'vitest';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    pairs,
    registerAll,
    rigidBody,
    updateWorld,
} from '../src';
import { bodyContactCount } from './helpers';

registerAll();

const SPEC = 0.02;

/** a world with a single object layer that collides with itself */
function makeWorld() {
    const settings = createWorldSettings();
    const bpLayer = addBroadphaseLayer(settings);
    const layer = addObjectLayer(settings, bpLayer);
    enableCollision(settings, layer, layer);
    const world = createWorld(settings);
    return { world, layer };
}

function makeBox(
    world: ReturnType<typeof makeWorld>['world'],
    layer: number,
    position: [number, number, number],
    motionType = MotionType.DYNAMIC,
) {
    return rigidBody.create(world, {
        shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
        objectLayer: layer,
        motionType,
        position,
        mass: motionType === MotionType.DYNAMIC ? 1 : undefined,
    });
}

describe('persistent-pair broadphase', () => {
    test('persistence through sleep: pair persists but is not emitted while both bodies sleep; waking one re-emits', () => {
        const { world, layer } = makeWorld();

        makeBox(world, layer, [0, 0, 0]);
        const b = makeBox(world, layer, [0.4, 0, 0]); // overlapping
        const a = world.bodies.pool[0];

        // first pass discovers the pair (both awake dynamic) and emits it
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(1);
        // the persistent record survives across frames
        expect(world.pairs.recordCount).toBe(1);

        // put both bodies to sleep: nobody queries, but the fat boxes still overlap so the
        // record persists — it just isn't emitted (dedup: both have INACTIVE_BODY_INDEX)
        rigidBody.sleep(world, a);
        rigidBody.sleep(world, b);
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(0);
        expect(world.pairs.recordCount).toBe(1);

        // waking one body (without moving it) re-emits the persisted pair next pass
        rigidBody.wake(world, a);
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(1);
    });

    test('teleport-sleeping destruction: teleporting a sleeping body away destroys the separated record the same frame', () => {
        const { world, layer } = makeWorld();

        const a = makeBox(world, layer, [0, 0, 0]);
        const b = makeBox(world, layer, [0.4, 0, 0]);

        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.recordCount).toBe(1);

        rigidBody.sleep(world, a);
        rigidBody.sleep(world, b);

        // settle into the sleeping steady state: the record persists but is not emitted
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.recordCount).toBe(1);
        expect(world.pairs.collidingPairCount).toBe(0);

        // teleport sleeping a far away WITHOUT waking it. the moved flag must route the pair
        // through the full sweep (past the frame-invariant sleeping-pair skip) so the
        // fat-separated record is destroyed this frame, not deferred until a later wake
        rigidBody.setPosition(world, a, [100, 0, 0], false);
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.recordCount).toBe(0);
    });

    test('teleport-sleeping discovery (G5): teleporting a sleeping body onto another discovers the pair; emitted once woken', () => {
        const { world, layer } = makeWorld();

        const a = makeBox(world, layer, [0, 0, 0]);
        const b = makeBox(world, layer, [50, 0, 0]); // far apart

        // discover initial (non-overlapping) state, then sleep both
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(0);
        rigidBody.sleep(world, a);
        rigidBody.sleep(world, b);

        // teleport the sleeping body A onto B WITHOUT waking it. this must still discover the
        // overlap (setPosition -> updateBody -> escape -> moved set -> discovery).
        rigidBody.setPosition(world, a, [50.4, 0, 0], false);
        pairs.findCollidingPairs(world, SPEC, undefined);
        // both still asleep => not emitted, but the pair must now exist in the persistent set
        expect(world.pairs.collidingPairCount).toBe(0);
        expect(world.pairs.recordCount).toBe(1);

        // waking one emits the discovered pair — the gap this fixes
        rigidBody.wake(world, a);
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(1);
    });

    test('removal purge: removing a body drops its pairs with no stale emission and no crash on later steps', () => {
        const { world, layer } = makeWorld();

        const a = makeBox(world, layer, [0, 0, 0]);
        const b = makeBox(world, layer, [0.4, 0, 0]);

        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(1);
        expect(world.pairs.recordCount).toBe(1);

        // remove one body of the pair
        rigidBody.remove(world, b);

        // the persistent pair involving b must have been purged
        expect(world.pairs.recordCount).toBe(0);

        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(0);

        // subsequent full steps must not crash and a still exists
        for (let i = 0; i < 5; i++) {
            updateWorld(world, undefined, 1 / 60);
        }
        expect(a._pooled).toBe(false);
    });

    test('determinism: two identical worlds stepped 60 frames produce bit-identical body positions', () => {
        function buildAndRun() {
            const { world, layer } = makeWorld();

            // a static floor and a small stack of dynamic boxes
            makeBox(world, layer, [0, -1, 0], MotionType.STATIC);
            makeBox(world, layer, [0, 0.2, 0]);
            makeBox(world, layer, [0.05, 1.4, 0.03]);
            makeBox(world, layer, [-0.03, 2.6, 0.02]);

            for (let i = 0; i < 60; i++) {
                updateWorld(world, undefined, 1 / 60);
            }

            return world.bodies.pool.filter((body) => !body._pooled).map((body) => [...body.position]);
        }

        const runA = buildAndRun();
        const runB = buildAndRun();

        expect(runB).toEqual(runA);
    });
});

describe('drift-into-contact discovery', () => {
    /**
     * two bodies can drift into contact entirely INSIDE their fat leaf AABBs (neither ever
     * escapes, so neither re-queries). discovery must therefore register pairs against the
     * candidates' FAT leaf boxes, not their current tight AABBs — with a tight-AABB test at
     * discovery time this pair is never found and the contact is silently missed.
     */
    test('bodies drifting within their fat AABBs still form a contact', () => {
        const { world, layer } = makeWorld();
        const shape = box.create({ halfExtents: [0.5, 0.5, 0.5] });

        // gap of 0.08 between faces: outside tight+speculative reach (0.07) at creation,
        // inside fat+speculative reach (0.12)
        const bodyA = rigidBody.create(world, {
            shape,
            objectLayer: layer,
            motionType: MotionType.DYNAMIC,
            position: [-0.54, 0, 0],
            gravityFactor: 0,
        });
        const bodyB = rigidBody.create(world, {
            shape,
            objectLayer: layer,
            motionType: MotionType.DYNAMIC,
            position: [0.54, 0, 0],
            gravityFactor: 0,
        });

        // drift toward each other at 0.5 m/s: ~0.0083 m/step each, so the 0.08 gap closes to
        // speculative-contact range in ~4 steps while total drift stays under the 0.05 escape
        // margin for both bodies
        rigidBody.setLinearVelocity(world, bodyA, [0.5, 0, 0]);
        rigidBody.setLinearVelocity(world, bodyB, [-0.5, 0, 0]);

        for (let i = 0; i < 6; i++) {
            updateWorld(world, undefined, 1 / 60);
        }

        expect(bodyContactCount(world, bodyA)).toBeGreaterThan(0);
        expect(bodyContactCount(world, bodyB)).toBeGreaterThan(0);
    });
});

describe('pair identity soundness', () => {
    test('pairs among recycled bodies stay distinct', () => {
        const { world, layer } = makeWorld();
        const shape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const mk = (x: number) =>
            rigidBody.create(world, {
                shape,
                objectLayer: layer,
                motionType: MotionType.DYNAMIC,
                position: [x, 0, 0],
                gravityFactor: 0,
            });

        // recycle three slots so all ids carry sequences (ids exceed 32 bits)
        const scratch = [mk(100), mk(103), mk(106)];
        for (const b of scratch) rigidBody.remove(world, b);
        // cluster: all three pairwise overlapping -> 3 distinct pairs expected
        const bodies = [mk(0), mk(0.4), mk(-0.4)];
        expect(Math.max(...bodies.map((b) => b.id))).toBeGreaterThan(0xffffffff);

        pairs.findCollidingPairs(world, 0.02, undefined);

        expect(world.pairs.recordCount).toBe(3);
        expect(world.pairs.collidingPairCount).toBe(3);
    });
});

describe('pair-list integrity', () => {
    /**
     * exercises the per-body intrusive pair lists: a hub body accumulates several records in its
     * list, removing spoke bodies unlinks records mid-list (head and interior cases), freed record
     * slots are reused by later discovery, and the remaining pairs keep emitting.
     */
    test('hub with spokes: removing bodies unlinks their records, freed slots are reused, rest keep emitting', () => {
        const { world, layer } = makeWorld();
        const hubShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const spokeShape = box.create({ halfExtents: [0.1, 0.1, 0.1] });

        const mk = (shape: ReturnType<typeof box.create>, position: [number, number, number]) =>
            rigidBody.create(world, {
                shape,
                objectLayer: layer,
                motionType: MotionType.DYNAMIC,
                position,
                gravityFactor: 0,
            });

        const hub = mk(hubShape, [0, 0, 0]);
        // four spokes overlapping ONLY the hub — pairwise far apart even with fat margins, so
        // exactly one record per spoke
        const spokes = [
            mk(spokeShape, [0.58, 0, 0]),
            mk(spokeShape, [-0.58, 0, 0]),
            mk(spokeShape, [0, 0.58, 0]),
            mk(spokeShape, [0, -0.58, 0]),
        ];

        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.recordCount).toBe(4);
        expect(world.pairs.collidingPairCount).toBe(4);

        // removing two spokes must unlink their records from the hub's pair list
        rigidBody.remove(world, spokes[0]);
        rigidBody.remove(world, spokes[1]);
        expect(world.pairs.recordCount).toBe(2);

        // the hub's remaining pairs still emit
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.collidingPairCount).toBe(2);

        // a new overlapping body reuses a freed record slot and pairs with the hub
        mk(spokeShape, [0.58, 0, 0]);
        pairs.findCollidingPairs(world, SPEC, undefined);
        expect(world.pairs.recordCount).toBe(3);
        expect(world.pairs.collidingPairCount).toBe(3);

        // stepping continues without error
        for (let i = 0; i < 5; i++) {
            updateWorld(world, undefined, 1 / 60);
        }
        expect(hub._pooled).toBe(false);
    });
});
