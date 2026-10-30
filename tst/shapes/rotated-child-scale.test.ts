import { mat4, type Quat, quat, type Vec3, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    castRayVsShape,
    castShapeVsShape,
    collidePointVsShape,
    collideShapeVsShape,
    compound,
    createAllCollideShapeCollector,
    createAnyCollidePointCollector,
    createClosestCastRayCollector,
    createClosestCastShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    createFace,
    EMPTY_SUB_SHAPE_ID,
    getShapeSupportingFace,
    type Shape,
    sphere,
    staticCompound,
    transformed,
    transformScale,
} from '../../src';

const IDENTITY: Quat = [0, 0, 0, 1];
const QUARTER_TURN_Z: Quat = quat.setAxisAngle(quat.create(), [0, 0, 1], Math.PI / 2);

// a bar long along its own x, turned to lie along y, under a scale that stretches x by 2.
// the scale is applied in the parent's space, so the bar ends up 1 wide in x (0.25 * 2 each side) and 1 tall in y.
// passed through to the child unrotated it would instead stretch the bar's own long axis: 0.25 in x, 2 in y
const bar = () => box.create({ halfExtents: vec3.fromValues(1, 0.25, 0.25), convexRadius: 0 });
const SCALE: Vec3 = [2, 1, 1];

const shapesWithTurnedBar = (): [string, Shape][] => [
    ['compound', compound.create({ children: [{ position: vec3.create(), quaternion: QUARTER_TURN_Z, shape: bar() }] })],
    [
        'static compound',
        staticCompound.create({ children: [{ position: vec3.create(), quaternion: QUARTER_TURN_Z, shape: bar() }] }),
    ],
    ['transformed', transformed.create({ shape: bar(), position: vec3.create(), quaternion: QUARTER_TURN_Z })],
];

const pointHits = (shape: Shape, point: Vec3): boolean => {
    const collector = createAnyCollidePointCollector();
    // biome-ignore format: readability
    collidePointVsShape(
        collector,
        createDefaultCollidePointSettings(),
        point[0], point[1], point[2],
        shape,
        EMPTY_SUB_SHAPE_ID, 0,
        0, 0, 0,
        IDENTITY[0], IDENTITY[1], IDENTITY[2], IDENTITY[3],
        SCALE[0], SCALE[1], SCALE[2],
    );
    return collector.hit !== null;
};

describe('transformScale', () => {
    test('a quarter turn about z swaps the x and y scale', () => {
        const out = transformScale(vec3.create(), QUARTER_TURN_Z, 2, 3, 4);
        expect(out[0]).toBeCloseTo(3, 6);
        expect(out[1]).toBeCloseTo(2, 6);
        expect(out[2]).toBeCloseTo(4, 6);
    });

    test('a uniform scale or an identity rotation passes through', () => {
        expect(transformScale(vec3.create(), QUARTER_TURN_Z, 2, 2, 2)).toEqual([2, 2, 2]);
        expect(transformScale(vec3.create(), IDENTITY, 2, 3, 4)).toEqual([2, 3, 4]);
    });
});

describe('a child turned a quarter about z under scale (2, 1, 1)', () => {
    for (const [name, make] of shapesWithTurnedBar().map(([n], i) => [n, () => shapesWithTurnedBar()[i][1]] as const)) {
        test(`${name}: points are inside where the parent's scale puts the bar`, () => {
            const shape = make();
            // inside: 0.5 wide in x, 1 tall in y
            expect(pointHits(shape, [0.4, 0, 0])).toBe(true);
            expect(pointHits(shape, [0, 0.9, 0])).toBe(true);
            // outside: where an unrotated scale would have stretched the bar to
            expect(pointHits(shape, [0, 1.5, 0])).toBe(false);
            expect(pointHits(shape, [0.6, 0, 0])).toBe(false);
        });

        test(`${name}: a ray along x enters at x = -0.5`, () => {
            const shape = make();
            const collector = createClosestCastRayCollector();
            // biome-ignore format: readability
            castRayVsShape(
                collector,
                createDefaultCastRaySettings(),
                -5, 0.5, 0,
                1, 0, 0,
                10,
                shape,
                EMPTY_SUB_SHAPE_ID, 0,
                0, 0, 0,
                IDENTITY[0], IDENTITY[1], IDENTITY[2], IDENTITY[3],
                SCALE[0], SCALE[1], SCALE[2],
            );
            expect(collector.hit.status).not.toBe(0);
            expect(collector.hit.fraction * 10).toBeCloseTo(4.5, 4);
        });

        test(`${name}: a small sphere at x = 0.6 touches the bar's side`, () => {
            const shape = make();
            const collector = createAllCollideShapeCollector();
            // biome-ignore format: readability
            collideShapeVsShape(
                collector,
                createDefaultCollideShapeSettings(),
                sphere.create({ radius: 0.2 }),
                EMPTY_SUB_SHAPE_ID, 0,
                0.6, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
                shape,
                EMPTY_SUB_SHAPE_ID, 0,
                0, 0, 0,
                IDENTITY[0], IDENTITY[1], IDENTITY[2], IDENTITY[3],
                SCALE[0], SCALE[1], SCALE[2],
            );
            expect(collector.hits.length).toBe(1);
            expect(collector.hits[0].penetration).toBeCloseTo(0.1, 4);
        });

        test(`${name}: the supporting face against -x is the bar's side at x = 0.5`, () => {
            const shape = make();
            const face = createFace();
            // the sub shape id of the only child (one bit for the compounds, none for transformed)
            const subShapeId = name === 'transformed' ? EMPTY_SUB_SHAPE_ID : 0xfffffffe;
            getShapeSupportingFace(face, shape, subShapeId, vec3.fromValues(-1, 0, 0), mat4.create(), SCALE);
            expect(face.numVertices).toBe(4);
            for (let i = 0; i < 4; i++) {
                expect(face.vertices[i * 3]).toBeCloseTo(0.5, 5);
                expect(Math.abs(face.vertices[i * 3 + 1])).toBeCloseTo(1, 5);
                expect(Math.abs(face.vertices[i * 3 + 2])).toBeCloseTo(0.25, 5);
            }
        });
    }
});

describe('transformed shape offset under scale', () => {
    test('the offset is scaled with the parent: a box moved 1 along x under scale 2 sits at x = 2', () => {
        const shape = transformed.create({
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5), convexRadius: 0 }),
            position: vec3.fromValues(1, 0, 0),
            quaternion: IDENTITY,
        });
        expect(pointHits(shape, [2, 0, 0])).toBe(true);
        expect(pointHits(shape, [0.8, 0, 0])).toBe(false);

        const face = createFace();
        // the box spans x 1 to 3: its +x face
        getShapeSupportingFace(face, shape, EMPTY_SUB_SHAPE_ID, vec3.fromValues(-1, 0, 0), mat4.create(), SCALE);
        expect(face.numVertices).toBe(4);
        for (let i = 0; i < 4; i++) {
            expect(face.vertices[i * 3]).toBeCloseTo(3, 5);
        }
    });
});

describe('transformed shape cast', () => {
    test('the displacement is world space whatever the cast shape is rotated by', () => {
        const target = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const ball = transformed.create({ shape: sphere.create({ radius: 0.5 }), position: vec3.create(), quaternion: IDENTITY });

        const collector = createClosestCastShapeCollector();
        // biome-ignore format: readability
        castShapeVsShape(
            collector,
            createDefaultCastShapeSettings(),
            ball,
            EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            QUARTER_TURN_Z[0], QUARTER_TURN_Z[1], QUARTER_TURN_Z[2], QUARTER_TURN_Z[3],
            1, 1, 1,
            5, 0, 0,
            target,
            EMPTY_SUB_SHAPE_ID, 0,
            3, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );
        expect(collector.hit.fraction).toBeCloseTo(0.4, 4);
    });
});
