import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, type Listener, MotionType, rigidBody, sphere, triangleMesh, updateWorld } from '../src';
import { MAX_SEQUENCE, serBodyId } from '../src/body/body-id';
import { EMPTY_SUB_SHAPE_ID } from '../src/body/sub-shape';
import * as contactConstraints from '../src/constraints/contact-constraints';
import { createTestWorld } from './helpers';

describe('contact constraints: contacts from the pair cache', () => {
    test('a resting contact persisted through the cache reports the materials and friction it was built with', () => {
        const { world, layers } = createTestWorld();
        world.settings.sleeping.allowSleeping = false;

        // a quad of two triangles with their own materials
        const floor = triangleMesh.create({
            positions: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10],
            indices: [0, 2, 1, 0, 3, 2],
            materialIndices: [5, 9],
        });
        rigidBody.create(world, {
            shape: floor,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            friction: 0.8,
        });

        // resting in the middle of one triangle, away from the shared diagonal
        rigidBody.create(world, {
            shape: sphere.create({ radius: 0.5, materialId: 3 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(5, 0.5, -3),
            friction: 0.2,
        });

        type Report = { materialIdA: number; materialIdB: number; friction: number };
        const added: Report[] = [];
        const persisted: Report[] = [];
        const listener: Listener = {
            onContactAdded: (_a, _b, manifold, settings) => {
                added.push({
                    materialIdA: manifold.materialIdA,
                    materialIdB: manifold.materialIdB,
                    friction: settings.combinedFriction,
                });
            },
            onContactPersisted: (_a, _b, manifold, settings) => {
                persisted.push({
                    materialIdA: manifold.materialIdA,
                    materialIdB: manifold.materialIdB,
                    friction: settings.combinedFriction,
                });
            },
        };

        for (let i = 0; i < 20; i++) updateWorld(world, listener, 1 / 60);

        expect(added.length).toBe(1);
        // body a is the floor (the lower id)
        expect([5, 9]).toContain(added[0].materialIdA);
        expect(added[0].materialIdB).toBe(3);
        expect(persisted.length).toBe(19);
        for (const report of persisted) {
            expect(report.materialIdA).toBe(added[0].materialIdA);
            expect(report.materialIdB).toBe(added[0].materialIdB);
            expect(report.friction).toBe(added[0].friction);
        }
    });
});

describe('contact constraints: whether to make a constraint', () => {
    const run = (invMassScale: number) => {
        const { world, layers } = createTestWorld();
        world.settings.sleeping.allowSleeping = false;

        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(10, 0.5, 10) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });
        const ball = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.5 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.49, 0),
        });

        // the ball is body b (the higher id)
        const scale: Listener['onContactAdded'] = (_a, _b, _manifold, settings) => {
            settings.invMassScale2 = invMassScale;
        };
        const listener: Listener = { onContactAdded: scale, onContactPersisted: scale };

        const counts: number[] = [];
        for (let i = 0; i < 5; i++) {
            updateWorld(world, listener, 1 / 60);
            counts.push(world.contactConstraints.count);
        }
        return { counts, ball };
    };

    test('a dynamic body with inverse mass scale 0 against a static body makes no constraint', () => {
        const scaled = run(0);
        expect(scaled.counts).toEqual([0, 0, 0, 0, 0]);
        // nothing holds it up
        expect(scaled.ball.position[1]).toBeLessThan(0.45);

        const control = run(1);
        expect(control.counts).toEqual([1, 1, 1, 1, 1]);
        expect(control.ball.position[1]).toBeGreaterThan(0.45);
    });
});

describe('contact constraints: sort key', () => {
    /** 64 bit fnv-1a over the little endian bytes of the given u32 words */
    const fnv1a64 = (words: number[]): bigint => {
        let hash = 0xcbf29ce484222325n;
        for (const word of words) {
            for (let byte = 0; byte < 4; byte++) {
                hash ^= BigInt((word >>> (byte * 8)) & 0xff);
                hash = (hash * 0x100000001b3n) & 0xffffffffffffffffn;
            }
        }
        return hash;
    };

    const key = (bodyIdA: number, subShapeIdA: number, bodyIdB: number, subShapeIdB: number): bigint => {
        const out = { sortKeyHigh: 0, sortKeyLow: 0 };
        contactConstraints.computeContactSortKey(out, bodyIdA, subShapeIdA, bodyIdB, subShapeIdB);
        return (BigInt(out.sortKeyHigh) << 32n) | BigInt(out.sortKeyLow);
    };

    test('is the 64 bit fnv-1a hash of body id a, sub shape a, body id b, sub shape b', () => {
        const cases: [number, number, number, number][] = [
            [serBodyId(0, 0), EMPTY_SUB_SHAPE_ID, serBodyId(1, 0), EMPTY_SUB_SHAPE_ID],
            [serBodyId(3, 1), 0x12345678, serBodyId(7, 2), 0],
            [serBodyId(0xfffffffe, MAX_SEQUENCE), 1, serBodyId(0xffffffff, MAX_SEQUENCE), 0x80000000],
        ];
        for (const [idA, subA, idB, subB] of cases) {
            const words = [idA >>> 0, Math.floor(idA / 2 ** 32), subA, idB >>> 0, Math.floor(idB / 2 ** 32), subB];
            expect(key(idA, subA, idB, subB)).toBe(fnv1a64(words));
        }

        // the known pair: bodies 1 and 2 (sequence 0), no sub shapes. the wasm core asserts the same value
        expect(key(serBodyId(1, 0), EMPTY_SUB_SHAPE_ID, serBodyId(2, 0), EMPTY_SUB_SHAPE_ID)).toBe(0x4a016474c74d95fen);
    });

    test('the sub shape ids and the sequence take part, so contacts of one body pair get distinct keys', () => {
        const idA = serBodyId(4, 0);
        const idB = serBodyId(5, 0);
        const keys = new Set([key(idA, 0, idB, 0), key(idA, 1, idB, 0), key(idA, 0, idB, 1), key(serBodyId(4, 1), 0, idB, 0)]);
        expect(keys.size).toBe(4);
    });

    test('a step solves contacts in the order of their 64 bit keys', () => {
        const { world, layers } = createTestWorld();
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(20, 0.5, 20) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });
        for (let i = 0; i < 12; i++) {
            rigidBody.create(world, {
                shape: sphere.create({ radius: 0.5 }),
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues((i % 4) * 0.9, 0.45 + Math.floor(i / 4) * 0.9, 0),
            });
        }
        updateWorld(world, undefined, 1 / 60);

        let compared = 0;
        for (const island of world.islands.islands) {
            const indices = island.contactIndices;
            for (let i = 1; i < indices.length; i++) {
                const previous = world.contactConstraints.pool[indices[i - 1]];
                const current = world.contactConstraints.pool[indices[i]];
                const previousKey = (BigInt(previous.sortKeyHigh) << 32n) | BigInt(previous.sortKeyLow);
                const currentKey = (BigInt(current.sortKeyHigh) << 32n) | BigInt(current.sortKeyLow);
                expect(previousKey < currentKey).toBe(true);
                compared++;
            }
        }
        expect(compared).toBeGreaterThan(5);
    });
});
