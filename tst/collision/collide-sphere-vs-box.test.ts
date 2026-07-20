import { describe, expect, test } from 'vitest';
import {
    box,
    collideShapeVsShape,
    createAllCollideShapeCollector,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    sphere,
} from '../../src';
import type { CollideShapeHit } from '../../src/collision/collide-shape-vs-shape';
import { collideConvexVsConvex } from '../../src/shapes/convex';

const settings = createDefaultCollideShapeSettings();

type Pose = { pos: number[]; rot: number[]; scale: number[] };

const identity: Pose = { pos: [0, 0, 0], rot: [0, 0, 0, 1], scale: [1, 1, 1] };

function collide(
    fn: typeof collideConvexVsConvex,
    shapeA: unknown,
    a: Pose,
    shapeB: unknown,
    b: Pose,
    s = settings,
): CollideShapeHit[] {
    const collector = createAllCollideShapeCollector();
    // biome-ignore format: readability
    fn(
        collector,
        s,
        shapeA as never,
        EMPTY_SUB_SHAPE_ID, 0,
        a.pos[0], a.pos[1], a.pos[2],
        a.rot[0], a.rot[1], a.rot[2], a.rot[3],
        a.scale[0], a.scale[1], a.scale[2],
        shapeB as never,
        EMPTY_SUB_SHAPE_ID, 0,
        b.pos[0], b.pos[1], b.pos[2],
        b.rot[0], b.rot[1], b.rot[2], b.rot[3],
        b.scale[0], b.scale[1], b.scale[2],
    );
    return collector.hits;
}

function normalize3(v: number[]): number[] {
    const l = Math.hypot(v[0], v[1], v[2]) || 1;
    return [v[0] / l, v[1] / l, v[2] / l];
}

function dot3(a: number[], b: number[]): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function mulberry32(seed: number) {
    let a = seed;
    return () => {
        a |= 0;
        a = (a + 0x6d2b79f5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

describe('collideShapeVsShape - Sphere vs Box (analytical)', () => {
    test('dispatch routes both orientations to a hit', () => {
        const s = sphere.create({ radius: 0.5 });
        const b = box.create({ halfExtents: [1, 1, 1] });
        const spherePose: Pose = { pos: [0, 1.4, 0], rot: [0, 0, 0, 1], scale: [1, 1, 1] };

        // sphere as A, box as B  ->  SPHERE x BOX
        const ab = collide(collideShapeVsShape, s, spherePose, b, identity);
        expect(ab.length).toBe(1);
        expect(ab[0].penetration).toBeGreaterThan(0);

        // box as A, sphere as B  ->  BOX x SPHERE (reversed wrapper)
        const ba = collide(collideShapeVsShape, b, identity, s, spherePose);
        expect(ba.length).toBe(1);
        expect(ba[0].penetration).toBeGreaterThan(0);
    });

    test('no collision when clearly separated', () => {
        const s = sphere.create({ radius: 0.5 });
        const b = box.create({ halfExtents: [1, 1, 1] });
        const hits = collide(collideShapeVsShape, s, { pos: [0, 5, 0], rot: [0, 0, 0, 1], scale: [1, 1, 1] }, b, identity);
        expect(hits.length).toBe(0);
    });

    test('resting on top face: expected penetration and downward axis', () => {
        const s = sphere.create({ radius: 0.5 });
        const b = box.create({ halfExtents: [1, 1, 1], convexRadius: 0 });
        // centre at y=1.4, top face y=1, sphere bottom at 0.9 -> penetration 0.1
        const hits = collide(collideShapeVsShape, s, { pos: [0, 1.4, 0], rot: [0, 0, 0, 1], scale: [1, 1, 1] }, b, identity);
        expect(hits.length).toBe(1);
        expect(hits[0].penetration).toBeCloseTo(0.1, 5);
        // penetration axis is A->B (sphere down toward box)
        const n = normalize3([...hits[0].penetrationAxis]);
        expect(n[1]).toBeLessThan(-0.999);
        // contact point on box top face
        expect(hits[0].pointB[1]).toBeCloseTo(1, 5);
    });

    test('face collection returns the box top face and an empty sphere face', () => {
        const s = sphere.create({ radius: 0.5 });
        const b = box.create({ halfExtents: [1, 1, 1], convexRadius: 0 });
        const withFaces = { ...settings, collectFaces: true };
        const hits = collide(
            collideShapeVsShape,
            s,
            { pos: [0, 1.4, 0], rot: [0, 0, 0, 1], scale: [1, 1, 1] },
            b,
            identity,
            withFaces,
        );
        expect(hits.length).toBe(1);
        expect(hits[0].faceA.numVertices).toBe(0);
        expect(hits[0].faceB.numVertices).toBe(4);
        // all box face vertices sit on the top plane y=1
        for (let i = 0; i < hits[0].faceB.numVertices; i++) {
            expect(hits[0].faceB.vertices[i * 3 + 1]).toBeCloseTo(1, 5);
        }
    });

    // ---- parity vs the generic GJK/EPA path -------------------------------------------

    function assertParity(label: string, s: unknown, sp: Pose, b: unknown, bp: Pose, penTol: number, dotTol: number) {
        const analytical = collide(collideShapeVsShape, s, sp, b, bp);
        const generic = collide(collideConvexVsConvex, s, sp, b, bp);
        expect(analytical.length, `${label}: hit/miss agreement`).toBe(generic.length);
        if (analytical.length === 1 && generic.length === 1) {
            expect(Math.abs(analytical[0].penetration - generic[0].penetration), `${label}: penetration`).toBeLessThan(penTol);
            const na = normalize3([...analytical[0].penetrationAxis]);
            const ng = normalize3([...generic[0].penetrationAxis]);
            expect(dot3(na, ng), `${label}: normal alignment`).toBeGreaterThan(dotTol);
        }
    }

    test('parity: shallow contacts, sharp and rounded boxes', () => {
        for (const cr of [0, 0.05]) {
            const b = box.create({ halfExtents: [1, 1, 1], convexRadius: cr });
            const s = sphere.create({ radius: 0.5 });
            // on top face
            assertParity(
                `top cr=${cr}`,
                s,
                { pos: [0.2, 1.35, -0.1], rot: [0, 0, 0, 1], scale: [1, 1, 1] },
                b,
                identity,
                1e-4,
                0.9995,
            );
            // over an edge
            assertParity(
                `edge cr=${cr}`,
                s,
                { pos: [1.05, 1.05, 0.0], rot: [0, 0, 0, 1], scale: [1, 1, 1] },
                b,
                identity,
                1e-4,
                0.9995,
            );
            // over a corner
            assertParity(
                `corner cr=${cr}`,
                s,
                { pos: [1.05, 1.05, 1.05], rot: [0, 0, 0, 1], scale: [1, 1, 1] },
                b,
                identity,
                1e-4,
                0.9995,
            );
        }
    });

    test('parity: deep penetration (centre inside box)', () => {
        const b = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
        const s = sphere.create({ radius: 0.5 });
        assertParity('deep', s, { pos: [0.15, 0.1, -0.2], rot: [0, 0, 0, 1], scale: [1, 1, 1] }, b, identity, 5e-3, 0.99);
    });

    test('parity: randomized poses (rotation, scale, radius)', () => {
        const rnd = mulberry32(0xc0ffee);
        let checked = 0;
        for (let i = 0; i < 400; i++) {
            const cr = rnd() < 0.5 ? 0 : 0.05;
            const hx = 0.4 + rnd() * 1.2;
            const hy = 0.4 + rnd() * 1.2;
            const hz = 0.4 + rnd() * 1.2;
            const b = box.create({ halfExtents: [hx, hy, hz], convexRadius: cr });

            const radius = 0.2 + rnd() * 0.8;
            const s = sphere.create({ radius });

            // random unit quaternion for the box
            let qx = rnd() * 2 - 1;
            let qy = rnd() * 2 - 1;
            let qz = rnd() * 2 - 1;
            let qw = rnd() * 2 - 1;
            const ql = Math.hypot(qx, qy, qz, qw) || 1;
            qx /= ql;
            qy /= ql;
            qz /= ql;
            qw /= ql;

            const boxScale = [0.6 + rnd() * 1.2, 0.6 + rnd() * 1.2, 0.6 + rnd() * 1.2];
            const sphereScale = 0.6 + rnd() * 1.2; // uniform (sphere supports uniform scale only)

            // place the sphere near a random face by pushing out from the box centre along a
            // random world direction, at a distance that yields a clear (non-boundary) contact.
            const maxHalf = Math.max(hx * boxScale[0], hy * boxScale[1], hz * boxScale[2]);
            const dir = normalize3([rnd() * 2 - 1, rnd() * 2 - 1, rnd() * 2 - 1]);
            const reach = maxHalf + radius * sphereScale;
            const t = 0.55 + rnd() * 0.3; // 0.55..0.85 of reach -> overlapping, away from the boundary
            const pos = [dir[0] * reach * t, dir[1] * reach * t, dir[2] * reach * t];

            const sp: Pose = { pos, rot: [0, 0, 0, 1], scale: [sphereScale, sphereScale, sphereScale] };
            const bp: Pose = { pos: [0, 0, 0], rot: [qx, qy, qz, qw], scale: boxScale };

            const analytical = collide(collideShapeVsShape, s, sp, b, bp);
            const generic = collide(collideConvexVsConvex, s, sp, b, bp);

            expect(analytical.length, `case ${i}: hit/miss`).toBe(generic.length);
            if (analytical.length === 1 && generic.length === 1) {
                checked++;
                expect(Math.abs(analytical[0].penetration - generic[0].penetration), `case ${i}: penetration`).toBeLessThan(1e-3);
                const na = normalize3([...analytical[0].penetrationAxis]);
                const ng = normalize3([...generic[0].penetrationAxis]);
                expect(dot3(na, ng), `case ${i}: normal`).toBeGreaterThan(0.995);
            }
        }
        // sanity: the loop actually exercised real contacts
        expect(checked).toBeGreaterThan(200);
    });
});
