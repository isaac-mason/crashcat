import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    type CastRaySettings,
    castRayVsShape,
    compound,
    createAllCastRayCollector,
    EMPTY_SUB_SHAPE_ID,
    type Shape,
    sphere,
    staticCompound,
} from '../../src';

const settings: CastRaySettings = { collideWithBackfaces: false, treatConvexAsSolid: true };

/** one unit sphere child at local (5, 0, 0) */
function makeStatic(): Shape {
    return staticCompound.create({
        children: [{ position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) }],
    });
}

function makeDynamic(): Shape {
    return compound.create({
        children: [{ position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) }],
    });
}

function cast(
    shape: Shape,
    position: [number, number, number],
    rotation: ReturnType<typeof quat.create>,
    scale: [number, number, number],
    origin: [number, number, number],
    direction: [number, number, number],
    length: number,
) {
    const collector = createAllCastRayCollector();
    castRayVsShape(
        collector,
        settings,
        origin[0],
        origin[1],
        origin[2],
        direction[0],
        direction[1],
        direction[2],
        length,
        shape,
        EMPTY_SUB_SHAPE_ID,
        0,
        position[0],
        position[1],
        position[2],
        rotation[0],
        rotation[1],
        rotation[2],
        rotation[3],
        scale[0],
        scale[1],
        scale[2],
    );
    return collector.hits;
}

const identity = quat.create();
// a quarter turn about y maps local +x to world -z
const quarterY = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), Math.PI / 2);

for (const [name, make] of [
    ['static compound', makeStatic],
    ['compound', makeDynamic],
] as const) {
    describe(`castRay vs transformed ${name}`, () => {
        test('translated compound', () => {
            // child sphere at world (15, 0, 0): a ray from the origin along +x hits its near side at x = 14
            const hits = cast(make(), [10, 0, 0], identity, [1, 1, 1], [0, 0, 0], [1, 0, 0], 20);
            expect(hits.length).toBe(1);
            expect(hits[0].fraction).toBeCloseTo(14 / 20, 6);
        });

        test('rotated compound', () => {
            // child at local (5, 0, 0) -> world (0, 0, -5): a ray from (0, 0, 5) along -z hits at z = -4
            const hits = cast(make(), [0, 0, 0], quarterY, [1, 1, 1], [0, 0, 5], [0, 0, -1], 20);
            expect(hits.length).toBe(1);
            expect(hits[0].fraction).toBeCloseTo(9 / 20, 6);
        });

        test('translated and rotated compound', () => {
            // world child = (1, 2, 3) + rot * (5, 0, 0) = (1, 2, -2): ray from (1, 2, 10) along -z hits at z = -1
            const hits = cast(make(), [1, 2, 3], quarterY, [1, 1, 1], [1, 2, 10], [0, 0, -1], 20);
            expect(hits.length).toBe(1);
            expect(hits[0].fraction).toBeCloseTo(11 / 20, 6);
        });

        test('uniformly scaled compound', () => {
            // scale 2: child centre at world (10, 0, 0) with radius 2, near side at x = 8
            const hits = cast(make(), [0, 0, 0], identity, [2, 2, 2], [0, 0, 0], [1, 0, 0], 20);
            expect(hits.length).toBe(1);
            expect(hits[0].fraction).toBeCloseTo(8 / 20, 6);
        });

        test('scaled, translated and rotated compound', () => {
            // scale 2 -> child at local (10, 0, 0), radius 2; rot -> (0, 0, -10); + (1, 2, 3) -> (1, 2, -7).
            // ray from (1, 2, 10) along -z hits the near side at z = -5
            const hits = cast(make(), [1, 2, 3], quarterY, [2, 2, 2], [1, 2, 10], [0, 0, -1], 20);
            expect(hits.length).toBe(1);
            expect(hits[0].fraction).toBeCloseTo(15 / 20, 6);
        });
    });
}
