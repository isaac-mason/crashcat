import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    castRayVsShape,
    CastShapeStatus,
    castShapeVsShape,
    collidePointVsShape,
    collideShapeVsShape,
    compound,
    createAllCastRayCollector,
    createAllCastShapeCollector,
    createAllCollideShapeCollector,
    createAnyCollidePointCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    type Shape,
    sphere,
    staticCompound,
} from '../../src';

// every query below uses the same compound: one unit sphere child at local (5, 0, 0), the compound
// scaled by 2, turned a quarter turn about y (local +x -> world -z) and moved to (1, 2, 3). the
// child therefore sits at world (1, 2, -7) with radius 2. before the fix the child was placed at
// (1, 2, -2) with radius 2, so every expectation here distinguishes the two placements.

const identity = quat.create();
const quarterY = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), Math.PI / 2);
const P: [number, number, number] = [1, 2, 3];
const S: [number, number, number] = [2, 2, 2];
const one: [number, number, number] = [1, 1, 1];

const kinds = [
    [
        'static compound',
        () =>
            staticCompound.create({
                children: [
                    { position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
                ],
            }),
    ],
    [
        'compound',
        () =>
            compound.create({
                children: [
                    { position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
                ],
            }),
    ],
] as const;

for (const [name, make] of kinds) {
    describe(`${name}, scaled, rotated and translated`, () => {
        test('collidePoint finds a point inside the scaled child', () => {
            const settings = createDefaultCollidePointSettings();
            const inside = createAnyCollidePointCollector();
            collidePointVsShape(inside, settings, 1, 2, -8.5, make(), EMPTY_SUB_SHAPE_ID, 0, ...P, ...quarterY, ...S);
            expect(inside.hit).not.toBeNull();

            const outside = createAnyCollidePointCollector();
            collidePointVsShape(outside, settings, 1, 2, -10, make(), EMPTY_SUB_SHAPE_ID, 0, ...P, ...quarterY, ...S);
            expect(outside.hit).toBeNull();
        });

        test('collideShape, compound as A and as B', () => {
            const settings = createDefaultCollideShapeSettings();
            // unit sphere at (1, 2, -9.5): its near side reaches z = -8.5, the child surface is at -9
            const ball = sphere.create({ radius: 1 });
            const q: [number, number, number] = [1, 2, -9.5];

            const asA = createAllCollideShapeCollector();
            collideShapeVsShape(
                asA,
                settings,
                make(),
                EMPTY_SUB_SHAPE_ID,
                0,
                ...P,
                ...quarterY,
                ...S,
                ball,
                EMPTY_SUB_SHAPE_ID,
                0,
                ...q,
                ...identity,
                ...one,
            );
            expect(asA.hits.length).toBe(1);
            expect(asA.hits[0].penetration).toBeCloseTo(0.5, 4);

            const asB = createAllCollideShapeCollector();
            collideShapeVsShape(
                asB,
                settings,
                ball,
                EMPTY_SUB_SHAPE_ID,
                0,
                ...q,
                ...identity,
                ...one,
                make(),
                EMPTY_SUB_SHAPE_ID,
                0,
                ...P,
                ...quarterY,
                ...S,
            );
            expect(asB.hits.length).toBe(1);
            expect(asB.hits[0].penetration).toBeCloseTo(0.5, 4);
        });

        test('castShape, compound as A and as B', () => {
            const settings = createDefaultCastShapeSettings();
            const ball = sphere.create({ radius: 1 });

            // ball from (1, 2, -20) moving +20 in z: its centre reaches the child's near side (z = -9)
            // minus its own radius at z = -10, after 10 units -> fraction 0.5
            const ballCast = createAllCastShapeCollector();
            castShapeVsShape(
                ballCast,
                settings,
                ball,
                EMPTY_SUB_SHAPE_ID,
                0,
                1,
                2,
                -20,
                ...identity,
                ...one,
                0,
                0,
                20,
                make(),
                EMPTY_SUB_SHAPE_ID,
                0,
                ...P,
                ...quarterY,
                ...S,
            );
            expect(ballCast.hits.length).toBe(1);
            expect(ballCast.hits[0].status).toBe(CastShapeStatus.COLLIDING);
            expect(ballCast.hits[0].fraction).toBeCloseTo(0.5, 3);

            // the compound moving +20 in z towards a ball at (1, 2, 10): the child's far side (z = -5)
            // meets the ball's near side (z = 9) after 14 units -> fraction 0.7
            const compoundCast = createAllCastShapeCollector();
            castShapeVsShape(
                compoundCast,
                settings,
                make(),
                EMPTY_SUB_SHAPE_ID,
                0,
                ...P,
                ...quarterY,
                ...S,
                0,
                0,
                20,
                ball,
                EMPTY_SUB_SHAPE_ID,
                0,
                1,
                2,
                10,
                ...identity,
                ...one,
            );
            expect(compoundCast.hits.length).toBe(1);
            expect(compoundCast.hits[0].status).toBe(CastShapeStatus.COLLIDING);
            expect(compoundCast.hits[0].fraction).toBeCloseTo(0.7, 3);
        });
    });
}

describe('non-uniform scale on an unrotated compound child', () => {
    // a unit box child at local (5, 0, 0) under scale (3, 1, 1): the box now spans x in [12, 18]
    const cases = [
        [
            'static compound',
            () =>
                staticCompound.create({
                    children: [
                        {
                            position: vec3.fromValues(5, 0, 0),
                            quaternion: quat.create(),
                            shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                        },
                    ],
                }),
        ],
        [
            'compound',
            () =>
                compound.create({
                    children: [
                        {
                            position: vec3.fromValues(5, 0, 0),
                            quaternion: quat.create(),
                            shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                        },
                    ],
                }),
        ],
    ] as const;

    for (const [name, make] of cases) {
        test(`${name}: ray along x hits the stretched box at x = 12`, () => {
            const collector = createAllCastRayCollector();
            castRayVsShape(
                collector,
                createDefaultCastRaySettings(),
                0,
                0.2,
                0.3,
                1,
                0,
                0,
                20,
                make() as Shape,
                EMPTY_SUB_SHAPE_ID,
                0,
                0,
                0,
                0,
                ...identity,
                3,
                1,
                1,
            );
            expect(collector.hits.length).toBe(1);
            expect(collector.hits[0].fraction).toBeCloseTo(12 / 20, 5);
        });
    }
});
