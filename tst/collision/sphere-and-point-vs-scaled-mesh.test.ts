import { quat } from 'math';
import { describe, expect, test } from 'vitest';
import {
    CastShapeStatus,
    castShapeVsShape,
    collidePointVsShape,
    collideShapeVsShape,
    createAllCastShapeCollector,
    createAllCollideShapeCollector,
    createAnyCollidePointCollector,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    sphere,
    triangleMesh,
} from '../../src';

/** closed unit cube mesh spanning [0,1]^3, outward winding */
function makeCubeMesh() {
    const p = [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1];
    // faces as two triangles each, counter-clockwise seen from outside
    const i = [
        0,
        2,
        1,
        0,
        3,
        2, // z = 0, normal -z
        4,
        5,
        6,
        4,
        6,
        7, // z = 1, normal +z
        0,
        1,
        5,
        0,
        5,
        4, // y = 0, normal -y
        3,
        7,
        6,
        3,
        6,
        2, // y = 1, normal +y
        0,
        4,
        7,
        0,
        7,
        3, // x = 0, normal -x
        1,
        2,
        6,
        1,
        6,
        5, // x = 1, normal +x
    ];
    return triangleMesh.create({ positions: p, indices: i });
}

/** unit quad in the xz plane at y = 1, normal +y, spanning [0,1] x [0,1] */
function makeQuad() {
    return triangleMesh.create({ positions: [0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1], indices: [0, 2, 1, 0, 3, 2] });
}

const q = quat.create();

describe('sphere and point vs scaled triangle mesh', () => {
    test('sphere vs a non-uniformly scaled quad', () => {
        // scale (3, 2, 1): surface at y = 2 over x in [0, 3]. unit sphere at (2.5, 2.8, 0.5): bottom at 1.8
        const collector = createAllCollideShapeCollector();
        collideShapeVsShape(
            collector,
            createDefaultCollideShapeSettings(),
            sphere.create({ radius: 1 }),
            EMPTY_SUB_SHAPE_ID,
            0,
            2.5,
            2.8,
            0.5,
            ...q,
            1,
            1,
            1,
            makeQuad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...q,
            3,
            2,
            1,
        );
        expect(collector.hits.length).toBeGreaterThan(0);
        let deepest = 0;
        for (const hit of collector.hits) deepest = Math.max(deepest, hit.penetration);
        expect(deepest).toBeCloseTo(0.2, 4);
    });

    test('sphere cast onto a non-uniformly scaled quad', () => {
        // unit sphere from (2.5, 6, 0.5) moving -5 in y: bottom starts at 5, meets y = 2 after 3 -> 0.6
        const collector = createAllCastShapeCollector();
        castShapeVsShape(
            collector,
            createDefaultCastShapeSettings(),
            sphere.create({ radius: 1 }),
            EMPTY_SUB_SHAPE_ID,
            0,
            2.5,
            6,
            0.5,
            ...q,
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
            ...q,
            3,
            2,
            1,
        );
        expect(collector.hits.length).toBeGreaterThan(0);
        expect(collector.hits[0].status).toBe(CastShapeStatus.COLLIDING);
        let first = Infinity;
        for (const hit of collector.hits) first = Math.min(first, hit.fraction);
        expect(first).toBeCloseTo(0.6, 3);
    });

    test('point inside a scaled closed mesh', () => {
        // scale (3, 1, 1): the cube spans x in [0, 3]. (2.5, 0.3, 0.6) is inside it and outside the unscaled
        // cube; the sample points stay off the top face's diagonal, which the parity ray would count twice
        const settings = createDefaultCollidePointSettings();
        const inside = createAnyCollidePointCollector();
        collidePointVsShape(inside, settings, 2.5, 0.3, 0.6, makeCubeMesh(), EMPTY_SUB_SHAPE_ID, 0, 0, 0, 0, ...q, 3, 1, 1);
        expect(inside.hit).not.toBeNull();

        const outside = createAnyCollidePointCollector();
        collidePointVsShape(outside, settings, 3.5, 0.3, 0.6, makeCubeMesh(), EMPTY_SUB_SHAPE_ID, 0, 0, 0, 0, ...q, 3, 1, 1);
        expect(outside.hit).toBeNull();

        // sanity: unscaled
        const unit = createAnyCollidePointCollector();
        collidePointVsShape(unit, settings, 0.3, 0.5, 0.6, makeCubeMesh(), EMPTY_SUB_SHAPE_ID, 0, 0, 0, 0, ...q, 1, 1, 1);
        expect(unit.hit).not.toBeNull();
    });
});
