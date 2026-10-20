import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    CastShapeStatus,
    castShapeVsShape,
    collideShapeVsShape,
    createAllCastShapeCollector,
    createAllCollideShapeCollector,
    createDefaultCastShapeSettings,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    triangleMesh,
} from '../../src';

/** unit quad in the xz plane at y = 1, normal +y, spanning [0,1] x [0,1] */
function makeQuad() {
    return triangleMesh.create({
        positions: [0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        indices: [0, 2, 1, 0, 3, 2],
    });
}

const identity = quat.create();
const cube = () => box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

describe('convex vs scaled triangle mesh', () => {
    test('non-uniform scale: contact at a point outside the unscaled quad', () => {
        // scale (3, 2, 1): quad spans x in [0, 3] at y = 2. cube centred at (2.5, 2.3, 0.5) has its
        // bottom at y = 1.8, 0.2 below the surface, at an x the unscaled quad does not cover
        const collector = createAllCollideShapeCollector();
        collideShapeVsShape(
            collector,
            createDefaultCollideShapeSettings(),
            cube(),
            EMPTY_SUB_SHAPE_ID,
            0,
            2.5,
            2.3,
            0.5,
            ...identity,
            1,
            1,
            1,
            makeQuad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...identity,
            3,
            2,
            1,
        );
        expect(collector.hits.length).toBeGreaterThan(0);
        let deepest = 0;
        for (const hit of collector.hits) deepest = Math.max(deepest, hit.penetration);
        expect(deepest).toBeCloseTo(0.2, 4);

        // same cube against the unscaled quad is off its edge
        const miss = createAllCollideShapeCollector();
        collideShapeVsShape(
            miss,
            createDefaultCollideShapeSettings(),
            cube(),
            EMPTY_SUB_SHAPE_ID,
            0,
            2.5,
            1.3,
            0.5,
            ...identity,
            1,
            1,
            1,
            makeQuad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...identity,
            1,
            1,
            1,
        );
        expect(miss.hits.length).toBe(0);
    });

    // The mesh scale is folded into the B-to-A matrix, so the compose order matters: the scale has
    // to be applied before the rotation. With identity rotation both orders agree, so this is the
    // only case that can tell them apart.
    test('rotated body with non-uniform scale: the scale is applied before the rotation', () => {
        // scale (3, 2, 1) puts the quad surface at y = 2 spanning x in [0, 3], z in [0, 1].
        // rotating the body 90 degrees about +y maps (x, y, z) -> (z, y, -x), so the surface spans
        // x in [0, 1], z in [-3, 0]. rotating before scaling would instead span x in [0, 3],
        // z in [-1, 0], and the cube below would miss entirely.
        const rotY90 = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), Math.PI / 2);
        const collector = createAllCollideShapeCollector();
        collideShapeVsShape(
            collector,
            createDefaultCollideShapeSettings(),
            cube(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0.5,
            2.3,
            -2.5,
            ...identity,
            1,
            1,
            1,
            makeQuad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...rotY90,
            3,
            2,
            1,
        );
        expect(collector.hits.length).toBeGreaterThan(0);
        let deepest = 0;
        for (const hit of collector.hits) deepest = Math.max(deepest, hit.penetration);
        expect(deepest).toBeCloseTo(0.2, 4);

        // the same cube against the other compose order's footprint (x in [0, 3], z in [-1, 0]) is
        // off the surface, so a scale-after-rotation fold would report nothing here
        const miss = createAllCollideShapeCollector();
        collideShapeVsShape(
            miss,
            createDefaultCollideShapeSettings(),
            cube(),
            EMPTY_SUB_SHAPE_ID,
            0,
            2.5,
            2.3,
            -0.5,
            ...identity,
            1,
            1,
            1,
            makeQuad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...rotY90,
            3,
            2,
            1,
        );
        expect(miss.hits.length).toBe(0);
    });

    test('mirrored scale: the front face moves to the other side', () => {
        // scale (1, -1, 1): quad at y = -1 facing -y. a cube below it, top at -0.8, penetrates 0.2
        // through the front face
        const collector = createAllCollideShapeCollector();
        collideShapeVsShape(
            collector,
            createDefaultCollideShapeSettings(),
            cube(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0.5,
            -1.3,
            0.5,
            ...identity,
            1,
            1,
            1,
            makeQuad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...identity,
            1,
            -1,
            1,
        );
        expect(collector.hits.length).toBeGreaterThan(0);
        let deepest = 0;
        for (const hit of collector.hits) deepest = Math.max(deepest, hit.penetration);
        expect(deepest).toBeCloseTo(0.2, 4);
    });

    test('cast onto a scaled mesh: exact fraction', () => {
        // scale (3, 2, 1): surface at y = 2. cube from (2.5, 5, 0.5) moving -5 in y: its bottom
        // starts at 4.5 and meets the surface after 2.5 -> fraction 0.5
        const collector = createAllCastShapeCollector();
        castShapeVsShape(
            collector,
            createDefaultCastShapeSettings(),
            cube(),
            EMPTY_SUB_SHAPE_ID,
            0,
            2.5,
            5,
            0.5,
            ...identity,
            1,
            1,
            1,
            0,
            -5,
            0,
            makeQuad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...identity,
            3,
            2,
            1,
        );
        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].status).toBe(CastShapeStatus.COLLIDING);
        let first = Infinity;
        for (const hit of collector.hits) first = Math.min(first, hit.fraction);
        expect(first).toBeCloseTo(0.5, 3);
    });
});
