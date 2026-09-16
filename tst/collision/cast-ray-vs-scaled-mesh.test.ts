import { quat } from 'math';
import { describe, expect, test } from 'vitest';
import {
    type CastRaySettings,
    CastRayStatus,
    castRayVsShape,
    createAllCastRayCollector,
    EMPTY_SUB_SHAPE_ID,
    triangleMesh,
} from '../../src';

const settings: CastRaySettings = { collideWithBackfaces: false, treatConvexAsSolid: true };
const backfaces: CastRaySettings = { collideWithBackfaces: true, treatConvexAsSolid: true };

/** unit quad in the xz plane at y = 1, normal +y, spanning [0,1] x [0,1] */
function makeQuad() {
    return triangleMesh.create({
        positions: [0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        indices: [0, 2, 1, 0, 3, 2],
    });
}

function cast(
    shape: ReturnType<typeof makeQuad>,
    scale: [number, number, number],
    origin: [number, number, number],
    direction: [number, number, number],
    length: number,
    raySettings: CastRaySettings = settings,
) {
    const q = quat.create();
    const collector = createAllCastRayCollector();
    castRayVsShape(
        collector,
        raySettings,
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
        0,
        0,
        0,
        q[0],
        q[1],
        q[2],
        q[3],
        scale[0],
        scale[1],
        scale[2],
    );
    return collector.hits;
}

describe('castRay vs scaled triangle mesh', () => {
    test('unit scale hits the quad at the right fraction', () => {
        const hits = cast(makeQuad(), [1, 1, 1], [0.3, 11, 0.6], [0, -1, 0], 20);
        expect(hits.length).toBe(1);
        expect(hits[0].status).toBe(CastRayStatus.COLLIDING);
        expect(hits[0].fraction).toBeCloseTo(10 / 20, 6);
    });

    test('uniform scale moves the surface and widens the quad', () => {
        // scale 2: quad spans [0,2] x [0,2] at y = 2
        const hits = cast(makeQuad(), [2, 2, 2], [1.5, 11, 1.8], [0, -1, 0], 20);
        expect(hits.length).toBe(1);
        expect(hits[0].fraction).toBeCloseTo(9 / 20, 6);

        // x = 1.5 is outside the unscaled quad, so a unit-scale cast misses there
        expect(cast(makeQuad(), [1, 1, 1], [1.5, 11, 1.8], [0, -1, 0], 20).length).toBe(0);
    });

    test('non-uniform scale', () => {
        // scale (3, 0.5, 1): quad spans [0,3] x [0,1] at y = 0.5
        const shape = makeQuad();
        const hits = cast(shape, [3, 0.5, 1], [2.5, 5.5, 0.5], [0, -1, 0], 10);
        expect(hits.length).toBe(1);
        expect(hits[0].fraction).toBeCloseTo(5 / 10, 6);

        // z = 1.5 is outside the quad (z is unscaled)
        expect(cast(shape, [3, 0.5, 1], [2.5, 5.5, 1.5], [0, -1, 0], 10).length).toBe(0);
    });

    test('diagonal ray against a scaled quad', () => {
        // scale 2: surface at y = 2. ray from (0, 4, 0) along (1, -1, 0)/sqrt2 reaches y = 2 after
        // travelling 2*sqrt2, at x = 2 (on the quad edge) -> use a slightly steeper ray
        const shape = makeQuad();
        const dir: [number, number, number] = [1 / Math.sqrt(5), -2 / Math.sqrt(5), 0];
        // y drops by 2 after t = sqrt5, x advances by 1 -> hit at (1, 2, z0)
        const hits = cast(shape, [2, 2, 2], [0.2, 4, 0.7], dir, 10);
        expect(hits.length).toBe(1);
        expect(hits[0].fraction).toBeCloseTo(Math.sqrt(5) / 10, 6);
    });

    test('a hit at the ray end counts, a hit just past it does not', () => {
        const shape = makeQuad();
        // surface at y = 1, origin at y = 6: a ray of length 5 ends exactly on the surface
        expect(cast(shape, [1, 1, 1], [0.3, 6, 0.6], [0, -1, 0], 5).length).toBe(1);
        expect(cast(shape, [1, 1, 1], [0.3, 6, 0.6], [0, -1, 0], 5)[0].fraction).toBeCloseTo(1, 9);
        // a ray 1e-5 short of the surface must not report a fraction above 1
        expect(cast(shape, [1, 1, 1], [0.3, 6, 0.6], [0, -1, 0], 5 - 1e-5).length).toBe(0);
        // same for the closest-hit path through the mesh's own early-out handling
        expect(cast(shape, [2, 2, 2], [0.3, 7, 0.6], [0, -1, 0], 5 - 1e-5).length).toBe(0);
    });

    test('mirrored scale flips the front face', () => {
        const shape = makeQuad();
        // scale y = -1 puts the quad at y = -1 with its normal pointing -y. a ray coming from above
        // now hits the back face: culled by default, reported with collideWithBackfaces
        expect(cast(shape, [1, -1, 1], [0.3, 9, 0.6], [0, -1, 0], 20).length).toBe(0);
        const hits = cast(shape, [1, -1, 1], [0.3, 9, 0.6], [0, -1, 0], 20, backfaces);
        expect(hits.length).toBe(1);
        expect(hits[0].fraction).toBeCloseTo(10 / 20, 6);
    });
});
