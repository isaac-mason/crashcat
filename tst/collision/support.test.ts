import { mat4, quat, type Vec3, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, capsule, convexHull, cylinder, sphere } from '../../src';
import {
    boxSupport,
    capsuleSupport,
    createSupport,
    cylinderSupport,
    hullSupport,
    pointSupport,
    type Support,
    type SupportFunction,
    SupportFunctionMode,
    setPointSupport,
    setPolygonSupport,
    setTriangleSupport,
    sphereSupport,
    triangleSupport,
} from '../../src/collision/support';
import { setShapeSupport } from '../../src/shapes/shapes';

const { INCLUDE_CONVEX_RADIUS, EXCLUDE_CONVEX_RADIUS, DEFAULT } = SupportFunctionMode;
const ONE: Vec3 = vec3.fromValues(1, 1, 1);

/** run getSupport into a fresh vector */
function at(support: Support, dir: Vec3): Vec3 {
    const out = vec3.create();
    support.getSupport(out, support, dir);
    return out;
}

function expectVec(actual: Vec3, expected: readonly number[], precision = 10): void {
    expect(actual[0]).toBeCloseTo(expected[0], precision);
    expect(actual[1]).toBeCloseTo(expected[1], precision);
    expect(actual[2]).toBeCloseTo(expected[2], precision);
}

/** brute-force max-dot vertex over a flat [x,y,z,...] buffer — reference for HULL/POLYGON */
function bruteForceMaxDot(positions: number[], count: number, dir: Vec3): Vec3 {
    let best = -Infinity;
    let bx = 0;
    let by = 0;
    let bz = 0;
    for (let i = 0; i < count * 3; i += 3) {
        const x = positions[i];
        const y = positions[i + 1];
        const z = positions[i + 2];
        const dot = x * dir[0] + y * dir[1] + z * dir[2];
        if (dot > best) {
            best = dot;
            bx = x;
            by = y;
            bz = z;
        }
    }
    return vec3.fromValues(bx, by, bz);
}

const AXES: Vec3[] = [
    vec3.fromValues(1, 0, 0),
    vec3.fromValues(-1, 0, 0),
    vec3.fromValues(0, 1, 0),
    vec3.fromValues(0, -1, 0),
    vec3.fromValues(0, 0, 1),
    vec3.fromValues(0, 0, -1),
    vec3.fromValues(1, 1, 1),
    vec3.fromValues(-1, 2, -3),
];

describe('getSupport — box', () => {
    const shape = box.create({ halfExtents: vec3.fromValues(2, 3, 4), convexRadius: 0.05 });

    test('INCLUDE returns the full box; sign-selects half-extents per axis', () => {
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        expect(s.getSupport).toBe(boxSupport);
        expect(s.convexRadius).toBe(0);
        expectVec(at(s, vec3.fromValues(1, 1, 1)), [2, 3, 4]);
        expectVec(at(s, vec3.fromValues(-1, -1, -1)), [-2, -3, -4]);
        expectVec(at(s, vec3.fromValues(1, -1, 1)), [2, -3, 4]);
    });

    test('EXCLUDE shrinks by the (clamped) convex radius and reports it', () => {
        const s = createSupport();
        setShapeSupport(s, shape, EXCLUDE_CONVEX_RADIUS, ONE);
        expect(s.convexRadius).toBeCloseTo(0.05, 10);
        expectVec(at(s, vec3.fromValues(1, 1, 1)), [2 - 0.05, 3 - 0.05, 4 - 0.05]);
    });

    test('scale is applied componentwise', () => {
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, vec3.fromValues(2, 0.5, 1.5));
        expectVec(at(s, vec3.fromValues(1, 1, 1)), [4, 1.5, 6]);
    });
});

describe('getSupport — sphere', () => {
    const shape = sphere.create({ radius: 2 });

    test('INCLUDE returns radius·dir̂; convexRadius is 0', () => {
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        expect(s.getSupport).toBe(sphereSupport);
        expect(s.convexRadius).toBe(0);
        expectVec(at(s, vec3.fromValues(1, 0, 0)), [2, 0, 0]);
        const r3 = 2 / Math.sqrt(3);
        expectVec(at(s, vec3.fromValues(1, 1, 1)), [r3, r3, r3]);
    });

    test('EXCLUDE returns the origin core and reports radius', () => {
        const s = createSupport();
        setShapeSupport(s, shape, EXCLUDE_CONVEX_RADIUS, ONE);
        expect(s.convexRadius).toBeCloseTo(2, 10);
        for (const dir of AXES) {
            expectVec(at(s, dir), [0, 0, 0]);
        }
    });

    test('uniform scale scales the radius', () => {
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, vec3.fromValues(3, 3, 3));
        expectVec(at(s, vec3.fromValues(1, 0, 0)), [6, 0, 0]);
    });
});

describe('getSupport — capsule', () => {
    const shape = capsule.create({ radius: 0.5, halfHeightOfCylinder: 1 });

    test('INCLUDE returns segment endpoint + radius·dir̂', () => {
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        expect(s.getSupport).toBe(capsuleSupport);
        expect(s.convexRadius).toBe(0);
        expectVec(at(s, vec3.fromValues(0, 1, 0)), [0, 1 + 0.5, 0]);
        expectVec(at(s, vec3.fromValues(0, -1, 0)), [0, -(1 + 0.5), 0]);
    });

    test('EXCLUDE returns the segment only and reports radius', () => {
        const s = createSupport();
        setShapeSupport(s, shape, EXCLUDE_CONVEX_RADIUS, ONE);
        expect(s.convexRadius).toBeCloseTo(0.5, 10);
        expectVec(at(s, vec3.fromValues(0, 1, 0)), [0, 1, 0]);
        expectVec(at(s, vec3.fromValues(0, -1, 0)), [0, -1, 0]);
    });
});

describe('getSupport — cylinder', () => {
    const shape = cylinder.create({ radius: 1, halfHeight: 2, convexRadius: 0.05 });

    test('INCLUDE combines the radial + axial extremes', () => {
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        expect(s.getSupport).toBe(cylinderSupport);
        expect(s.convexRadius).toBe(0);
        expectVec(at(s, vec3.fromValues(1, 0, 0)), [1, 2, 0]); // dir.y === 0 → +halfHeight
        expectVec(at(s, vec3.fromValues(0, 1, 0)), [0, 2, 0]); // no radial component
        expectVec(at(s, vec3.fromValues(0, -1, 0)), [0, -2, 0]);
    });

    test('EXCLUDE pre-shrinks radius + halfHeight and reports the radius', () => {
        const s = createSupport();
        setShapeSupport(s, shape, EXCLUDE_CONVEX_RADIUS, ONE);
        expect(s.convexRadius).toBeCloseTo(0.05, 10);
        expectVec(at(s, vec3.fromValues(1, 0, 0)), [1 - 0.05, 2 - 0.05, 0]);
    });

    test('DEFAULT behaves like INCLUDE for the cylinder', () => {
        const inc = createSupport();
        const def = createSupport();
        setShapeSupport(inc, shape, INCLUDE_CONVEX_RADIUS, ONE);
        setShapeSupport(def, shape, DEFAULT, ONE);
        for (const dir of AXES) {
            expectVec(at(def, dir), Array.from(at(inc, dir)));
        }
    });
});

describe('getSupport — convex hull', () => {
    // unit cube corners (plus centre, which must not survive the hull)
    const positions = [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 0, 0, 0];
    const shape = convexHull.create({ positions, convexRadius: 0 });

    test('INCLUDE returns the max-dot hull vertex for every direction', () => {
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        expect(s.getSupport).toBe(hullSupport);
        for (const dir of AXES) {
            const expected = bruteForceMaxDot(shape.pointPositions, shape.numPoints, dir);
            expectVec(at(s, dir), Array.from(expected));
        }
    });

    test('scaled INCLUDE scans the scaled vertices', () => {
        const s = createSupport();
        const scale = vec3.fromValues(2, 0.5, 3);
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, scale);
        const scaled: number[] = [];
        for (let i = 0; i < shape.numPoints * 3; i += 3) {
            scaled.push(shape.pointPositions[i] * 2, shape.pointPositions[i + 1] * 0.5, shape.pointPositions[i + 2] * 3);
        }
        for (const dir of AXES) {
            const expected = bruteForceMaxDot(scaled, shape.numPoints, dir);
            expectVec(at(s, dir), Array.from(expected));
        }
    });
});

describe('getSupport — convex hull scaling', () => {
    // ~n points spread on a unit sphere (each is a hull vertex, all with a nonzero convex radius)
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

    function randomDirections(count: number): Vec3[] {
        const rand = mulberry32(0x9e3779b9);
        const dirs: Vec3[] = [];
        while (dirs.length < count) {
            const d = vec3.fromValues(rand() * 2 - 1, rand() * 2 - 1, rand() * 2 - 1);
            if (vec3.length(d) > 1e-3) dirs.push(d);
        }
        return dirs;
    }

    const shape = convexHull.create({ positions: fibonacciSphere(100), convexRadius: 0.05 });
    const dirs = randomDirections(50);

    test('uniform-scale fast path matches explicitly scaling the raw / shrunk vertices', () => {
        for (const s of [0.5, 1, 2.37]) {
            const scale = vec3.fromValues(s, s, s);

            // INCLUDE → raw vertices scaled by s
            const inc = createSupport();
            setShapeSupport(inc, shape, INCLUDE_CONVEX_RADIUS, scale);
            expect(inc.convexRadius).toBe(0);
            const scaledRaw: number[] = [];
            for (let i = 0; i < shape.numPoints * 3; i++) scaledRaw.push(shape.pointPositions[i] * s);
            for (const dir of dirs) {
                expectVec(at(inc, dir), Array.from(bruteForceMaxDot(scaledRaw, shape.numPoints, dir)), 9);
            }

            // EXCLUDE → shrunk vertices scaled by s, radius reported as r·s
            const exc = createSupport();
            setShapeSupport(exc, shape, EXCLUDE_CONVEX_RADIUS, scale);
            expect(exc.convexRadius).toBeCloseTo(shape.convexRadius * s, 9);
            const scaledShrunk: number[] = [];
            for (let i = 0; i < shape.numPoints * 3; i++) scaledShrunk.push(shape.shrunkPointPositions[i] * s);
            for (const dir of dirs) {
                expectVec(at(exc, dir), Array.from(bruteForceMaxDot(scaledShrunk, shape.numPoints, dir)), 9);
            }
        }
    });

    test('non-uniform EXCLUDE bakes scaled shrunk vertices (analytic box reference, outputScale stays 1)', () => {
        // an axis-aligned box hull: the scaled shrunk corner is the scaled corner pushed inward by the
        // scaled radius (r·minScale) along each of its three perpendicular face normals
        const cube = convexHull.create({
            positions: [-1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1],
            convexRadius: 0.05,
        });
        const scale = vec3.fromValues(1, 2, 0.5);
        const rScaled = cube.convexRadius * 0.5; // minScale = 0.5

        const s = createSupport();
        setShapeSupport(s, cube, EXCLUDE_CONVEX_RADIUS, scale);
        expect(s.convexRadius).toBeCloseTo(rScaled, 9);
        expect(s.hull.outputScale).toBe(1); // slow path: vertices are pre-baked, not output-scaled

        const ref: number[] = [];
        for (let i = 0; i < cube.numPoints * 3; i += 3) {
            const cx = cube.pointPositions[i];
            const cy = cube.pointPositions[i + 1];
            const cz = cube.pointPositions[i + 2];
            ref.push(
                cx * scale[0] - Math.sign(cx) * rScaled,
                cy * scale[1] - Math.sign(cy) * rScaled,
                cz * scale[2] - Math.sign(cz) * rScaled,
            );
        }
        for (const dir of dirs) {
            expectVec(at(s, dir), Array.from(bruteForceMaxDot(ref, cube.numPoints, dir)), 9);
        }
    });
});

describe('getSupport — triangle / point / polygon primitives', () => {
    test('triangle returns the max-dot of its three vertices', () => {
        const a = vec3.fromValues(0, 0, 0);
        const b = vec3.fromValues(1, 0, 0);
        const c = vec3.fromValues(0, 1, 0);
        const s = createSupport();
        setTriangleSupport(s, a, b, c);
        expect(s.getSupport).toBe(triangleSupport);
        expect(s.triangle.vertices).toEqual([0, 0, 0, 1, 0, 0, 0, 1, 0]);
        expectVec(at(s, vec3.fromValues(1, 0, 0)), [1, 0, 0]);
        expectVec(at(s, vec3.fromValues(0, 1, 0)), [0, 1, 0]);
        expectVec(at(s, vec3.fromValues(-1, -1, 0)), [0, 0, 0]);
    });

    test('point always returns the point', () => {
        const p = vec3.fromValues(3, -4, 5);
        const s = createSupport();
        setPointSupport(s, p);
        expect(s.getSupport).toBe(pointSupport);
        for (const dir of AXES) {
            expectVec(at(s, dir), [3, -4, 5]);
        }
    });

    test('point with addRadius and a transform is the transformed point plus radius along the world direction', () => {
        const s = createSupport();
        setPointSupport(s, vec3.fromValues(1, 2, 3));
        s.addRadius = 0.5;
        s.hasTransform = true;
        mat4.fromRotationTranslation(
            s.transform,
            quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2),
            vec3.fromValues(10, 0, 0),
        );
        // rotating (1,2,3) by 90° about z gives (-2,1,3); translated → (8,1,3). the radius is added
        // along the direction, which the rotation round-trips exactly
        for (const dir of AXES) {
            const unit = vec3.normalize(vec3.create(), dir);
            expectVec(at(s, dir), [8 + 0.5 * unit[0], 1 + 0.5 * unit[1], 3 + 0.5 * unit[2]], 6);
        }
    });

    test('polygon (borrowed face verts) returns the max-dot vertex', () => {
        const verts = [-1, 0, -1, 1, 0, -1, 1, 0, 1, -1, 0, 1];
        const s = createSupport();
        setPolygonSupport(s, verts, 4);
        expect(s.getSupport).toBe(hullSupport);
        for (const dir of AXES) {
            const expected = bruteForceMaxDot(verts, 4, dir);
            expectVec(at(s, dir), Array.from(expected));
        }
    });
});

describe('getSupport — folded addRadius (was AddConvexRadiusSupport)', () => {
    test('adds addRadius·dir̂ to the core support point', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(2, 3, 4), convexRadius: 0.05 });
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        s.addRadius = 0.1;
        expectVec(at(s, vec3.fromValues(1, 0, 0)), [2 + 0.1, 3, 4]);
        const inv = 0.1 / Math.sqrt(3);
        expectVec(at(s, vec3.fromValues(1, 1, 1)), [2 + inv, 3 + inv, 4 + inv]);
    });
});

describe('getSupport — folded transform (was TransformedSupport)', () => {
    test('pure translation offsets the world support point', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(2, 3, 4), convexRadius: 0 });
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        s.hasTransform = true;
        mat4.fromTranslation(s.transform, vec3.fromValues(10, 20, 30));
        expectVec(at(s, vec3.fromValues(1, 1, 1)), [2 + 10, 3 + 20, 4 + 30]);
    });

    test('rotation+translation: a sphere support is r·dir̂ + translation, rotation-invariant', () => {
        const shape = sphere.create({ radius: 2 });
        const s = createSupport();
        setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
        s.hasTransform = true;
        const axis = vec3.normalize(vec3.create(), vec3.fromValues(1, 2, -3));
        const rot = quat.setAxisAngle(quat.create(), axis, 0.9);
        mat4.fromRotationTranslation(s.transform, rot, vec3.fromValues(5, -6, 7));
        for (const dir of AXES) {
            const expected = vec3.scale(vec3.create(), vec3.normalize(vec3.create(), dir), 2);
            vec3.add(expected, expected, vec3.fromValues(5, -6, 7));
            expectVec(at(s, dir), Array.from(expected));
        }
    });
});

describe('getSupport — dispatch + degenerate directions', () => {
    test('setShapeSupport installs the correct evaluator per shape', () => {
        const cases: Array<[ReturnType<typeof box.create> | any, SupportFunction]> = [
            [box.create({ halfExtents: ONE }), boxSupport],
            [sphere.create({ radius: 1 }), sphereSupport],
            [capsule.create({ radius: 0.5, halfHeightOfCylinder: 1 }), capsuleSupport],
            [cylinder.create({ radius: 1, halfHeight: 1 }), cylinderSupport],
            [convexHull.create({ positions: [-1, -1, -1, 1, -1, -1, 0, 1, -1, 0, 0, 1] }), hullSupport],
        ];
        const s = createSupport();
        for (const [shape, evaluator] of cases) {
            setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
            expect(s.getSupport).toBe(evaluator);
        }
    });

    test('a zero-length direction never produces NaN', () => {
        const zero = vec3.fromValues(0, 0, 0);
        const shapes = [
            box.create({ halfExtents: ONE }),
            sphere.create({ radius: 1 }),
            capsule.create({ radius: 0.5, halfHeightOfCylinder: 1 }),
            cylinder.create({ radius: 1, halfHeight: 1 }),
        ];
        const s = createSupport();
        for (const shape of shapes) {
            setShapeSupport(s, shape, INCLUDE_CONVEX_RADIUS, ONE);
            const out = at(s, zero);
            expect(Number.isFinite(out[0])).toBe(true);
            expect(Number.isFinite(out[1])).toBe(true);
            expect(Number.isFinite(out[2])).toBe(true);
        }
    });
});
