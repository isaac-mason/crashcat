import { type Vec3, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { convexHull } from '../../src';
import { createSupport, getSupport, type Support, SupportFunctionMode, setHullSupport } from '../../src/collision/support';

const { INCLUDE_CONVEX_RADIUS, EXCLUDE_CONVEX_RADIUS } = SupportFunctionMode;

function fibonacciSphere(n: number): number[] {
    const out: number[] = [];
    const phi = Math.PI * (3 - Math.sqrt(5));
    for (let i = 0; i < n; i++) {
        const y = 1 - (i / (n - 1)) * 2;
        const r = Math.sqrt(Math.max(0, 1 - y * y));
        const theta = phi * i;
        out.push(Math.cos(theta) * r, y, Math.sin(theta) * r);
    }
    return out;
}

function slabGrid(): number[] {
    const out: number[] = [];
    for (let ix = 0; ix < 5; ix++) {
        for (let iz = 0; iz < 5; iz++) {
            const x = ix - 2;
            const z = iz - 2;
            out.push(x, 0.1, z);
            out.push(x, -0.1, z);
        }
    }
    return out;
}

const BOX_POINTS = [-1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1];

// seeded PRNG so directions are stable across runs
function mulberry32(seed: number): () => number {
    let a = seed;
    return () => {
        a |= 0;
        a = (a + 0x6d2b79f5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

function randomDirections(count: number, seed: number): Vec3[] {
    const rand = mulberry32(seed);
    const dirs: Vec3[] = [];
    while (dirs.length < count) {
        const d = vec3.fromValues(rand() * 2 - 1, rand() * 2 - 1, rand() * 2 - 1);
        if (vec3.length(d) > 1e-3) dirs.push(d);
    }
    return dirs;
}

/** a slowly-rotating direction sequence, mimicking how GJK/EPA evolve the query direction */
function rotatingDirections(count: number): Vec3[] {
    const dirs: Vec3[] = [];
    for (let i = 0; i < count; i++) {
        const t = (i / count) * Math.PI * 2;
        // small polar drift + azimuthal sweep
        const el = Math.sin(t * 0.37) * 0.9;
        const r = Math.cos(el);
        dirs.push(vec3.fromValues(Math.cos(t) * r, el, Math.sin(t) * r));
    }
    return dirs;
}

const SCALES: Array<[string, Vec3]> = [
    ['unscaled', vec3.fromValues(1, 1, 1)],
    ['uniform', vec3.fromValues(2.37, 2.37, 2.37)],
    ['non-uniform', vec3.fromValues(2, 0.5, 3)],
    ['mirrored uniform', vec3.fromValues(-1.5, -1.5, -1.5)],
    ['mirrored non-uniform', vec3.fromValues(-2, 1, 0.5)],
];

// max |v| over the resolved vertex buffer, for the additive tolerance bound
function maxAbs(support: Support): number {
    const v = support.hull.vertices;
    const len = support.hull.vertexCount * 3;
    let m = 0;
    for (let i = 0; i < len; i++) m = Math.max(m, Math.abs(v[i]));
    return m;
}

function dot(a: Vec3, b: Vec3): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

type HullCase = {
    name: string;
    shape: ReturnType<typeof convexHull.create>;
    strictlyConvex: boolean;
};

const HULLS: HullCase[] = [
    {
        name: 'fibonacci 60',
        shape: convexHull.create({ positions: fibonacciSphere(60), convexRadius: 0.05 }),
        strictlyConvex: true,
    },
    {
        name: 'fibonacci 100',
        shape: convexHull.create({ positions: fibonacciSphere(100), convexRadius: 0.05 }),
        strictlyConvex: true,
    },
    {
        name: 'fibonacci 256',
        shape: convexHull.create({ positions: fibonacciSphere(256), convexRadius: 0.05 }),
        strictlyConvex: true,
    },
    {
        name: 'box (forced bake)',
        shape: convexHull.create({ positions: BOX_POINTS, convexRadius: 0.05, bakeSupportAdjacency: true }),
        strictlyConvex: false,
    },
    {
        name: 'slab grid (forced bake)',
        shape: convexHull.create({ positions: slabGrid(), convexRadius: 0.02, bakeSupportAdjacency: true }),
        strictlyConvex: false,
    },
];

describe('convex hull support — hill climb vs brute equivalence', () => {
    for (const { name, shape, strictlyConvex } of HULLS) {
        // sanity: adjacency must be baked for these cases (else the climb path is never exercised)
        test(`${name}: adjacency is baked`, () => {
            expect(shape.pointNeighbors.length).toBeGreaterThan(0);
        });

        for (const [scaleName, scale] of SCALES) {
            for (const mode of [INCLUDE_CONVEX_RADIUS, EXCLUDE_CONVEX_RADIUS]) {
                const modeName = mode === INCLUDE_CONVEX_RADIUS ? 'INCLUDE' : 'EXCLUDE';

                test(`${name} · ${scaleName} · ${modeName}: cold seed then argmax equivalence`, () => {
                    const dirs = randomDirections(60, 0x1234 + shape.numPoints);

                    const acc = createSupport();
                    setHullSupport(acc, shape, mode, scale);
                    // forced-brute reference: same resolved vertices, adjacency stripped
                    const ref = createSupport();
                    setHullSupport(ref, shape, mode, scale);
                    ref.hull.neighborsStart = [];

                    const tol = 1e-12 * maxAbs(acc);
                    const a = vec3.create();
                    const b = vec3.create();

                    for (const dir of dirs) {
                        // each direction gets a cold accelerated support (first-call brute seeds the hint)
                        setHullSupport(acc, shape, mode, scale);
                        getSupport(a, acc, dir);
                        getSupport(b, ref, dir);
                        const climbDot = dot(a, dir);
                        const bruteDot = dot(b, dir);
                        expect(climbDot).toBeGreaterThanOrEqual(bruteDot - tol);
                        if (strictlyConvex) {
                            expect(a[0]).toBeCloseTo(b[0], 12);
                            expect(a[1]).toBeCloseTo(b[1], 12);
                            expect(a[2]).toBeCloseTo(b[2], 12);
                        }
                    }
                });

                test(`${name} · ${scaleName} · ${modeName}: warm path (rotating directions) stays equivalent`, () => {
                    const dirs = rotatingDirections(120);

                    // acc keeps its warm-start hint across the whole sequence (one pair-fill)
                    const acc = createSupport();
                    setHullSupport(acc, shape, mode, scale);
                    const ref = createSupport();
                    setHullSupport(ref, shape, mode, scale);
                    ref.hull.neighborsStart = [];

                    const tol = 1e-12 * maxAbs(acc);
                    const a = vec3.create();
                    const b = vec3.create();

                    for (const dir of dirs) {
                        getSupport(a, acc, dir);
                        getSupport(b, ref, dir);
                        const climbDot = dot(a, dir);
                        const bruteDot = dot(b, dir);
                        expect(climbDot).toBeGreaterThanOrEqual(bruteDot - tol);
                        if (strictlyConvex) {
                            expect(a[0]).toBeCloseTo(b[0], 12);
                            expect(a[1]).toBeCloseTo(b[1], 12);
                            expect(a[2]).toBeCloseTo(b[2], 12);
                        }
                    }
                });
            }
        }
    }

    test('sub-threshold hull (no bake) still returns correct support via brute path', () => {
        const box = convexHull.create({ positions: BOX_POINTS, convexRadius: 0 });
        expect(box.pointNeighbors.length).toBe(0);
        const s = createSupport();
        setHullSupport(s, box, INCLUDE_CONVEX_RADIUS, vec3.fromValues(1, 1, 1));
        const out = vec3.create();
        getSupport(out, s, vec3.fromValues(1, 1, 1));
        expect(out[0]).toBeCloseTo(1, 12);
        expect(out[1]).toBeCloseTo(1, 12);
        expect(out[2]).toBeCloseTo(1, 12);
    });
});
