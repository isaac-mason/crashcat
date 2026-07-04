import { describe, expect, test } from 'vitest';
import { convexHull } from '../../src';
import { SUPPORT_HILL_CLIMB_MIN_POINTS } from '../../src/shapes/convex-hull';

/** ~n points on a unit sphere; every point is a hull vertex, strictly convex */
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

/** a coplanar-heavy hull: a thin slab with a grid of points on each large face (many coplanar clusters) */
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

/** collect the set of undirected polygon edges (as "min,max" keys) walked over the hull faces */
function faceEdgeKeys(shape: ReturnType<typeof convexHull.create>): Set<string> {
    const keys = new Set<string>();
    for (const face of shape.faces) {
        for (let v = 0; v < face.numVertices; v++) {
            const a = shape.vertexIndices[face.firstVertex + v];
            const b = shape.vertexIndices[face.firstVertex + ((v + 1) % face.numVertices)];
            keys.add(a < b ? `${a},${b}` : `${b},${a}`);
        }
    }
    return keys;
}

function neighborsOf(shape: ReturnType<typeof convexHull.create>, p: number): number[] {
    return shape.pointNeighbors.slice(shape.pointNeighborsStart[p], shape.pointNeighborsStart[p + 1]);
}

describe('convex hull — support adjacency (CSR 1-ring)', () => {
    describe('flag / threshold behavior', () => {
        test('box (8 pts) is not baked by default', () => {
            const box = convexHull.create({ positions: BOX_POINTS });
            expect(box.numPoints).toBe(8);
            expect(box.numPoints).toBeLessThanOrEqual(SUPPORT_HILL_CLIMB_MIN_POINTS);
            expect(box.pointNeighborsStart).toEqual([]);
            expect(box.pointNeighbors).toEqual([]);
        });

        test('box is baked when bakeSupportAdjacency: true', () => {
            const box = convexHull.create({ positions: BOX_POINTS, bakeSupportAdjacency: true });
            expect(box.pointNeighborsStart.length).toBe(box.numPoints + 1);
            expect(box.pointNeighbors.length).toBeGreaterThan(0);
        });

        test('bakeSupportAdjacency: false never bakes even above threshold', () => {
            const hull = convexHull.create({ positions: fibonacciSphere(100), bakeSupportAdjacency: false });
            expect(hull.numPoints).toBeGreaterThan(SUPPORT_HILL_CLIMB_MIN_POINTS);
            expect(hull.pointNeighborsStart).toEqual([]);
            expect(hull.pointNeighbors).toEqual([]);
        });

        test('fibonacci 60 / 100 / 256 auto-bake (numPoints > 32)', () => {
            for (const n of [60, 100, 256]) {
                const hull = convexHull.create({ positions: fibonacciSphere(n) });
                expect(hull.numPoints).toBeGreaterThan(SUPPORT_HILL_CLIMB_MIN_POINTS);
                expect(hull.pointNeighborsStart.length).toBe(hull.numPoints + 1);
                expect(hull.pointNeighbors.length).toBeGreaterThan(0);
            }
        });
    });

    // property tests run on baked hulls (force the box + slab with the flag)
    const cases: Array<[string, ReturnType<typeof convexHull.create>]> = [
        ['box (forced)', convexHull.create({ positions: BOX_POINTS, bakeSupportAdjacency: true })],
        ['fibonacci 60', convexHull.create({ positions: fibonacciSphere(60) })],
        ['fibonacci 100', convexHull.create({ positions: fibonacciSphere(100) })],
        ['fibonacci 256', convexHull.create({ positions: fibonacciSphere(256) })],
        ['slab grid (forced)', convexHull.create({ positions: slabGrid(), bakeSupportAdjacency: true })],
    ];

    describe.each(cases)('%s', (_name, shape) => {
        test('CSR shape: start length numPoints+1, start[0]=0, monotone, last=neighbors.length', () => {
            expect(shape.pointNeighborsStart.length).toBe(shape.numPoints + 1);
            expect(shape.pointNeighborsStart[0]).toBe(0);
            for (let p = 0; p < shape.numPoints; p++) {
                expect(shape.pointNeighborsStart[p + 1]).toBeGreaterThanOrEqual(shape.pointNeighborsStart[p]);
            }
            expect(shape.pointNeighborsStart[shape.numPoints]).toBe(shape.pointNeighbors.length);
        });

        test('symmetry: a ∈ N(b) ⇔ b ∈ N(a)', () => {
            for (let p = 0; p < shape.numPoints; p++) {
                for (const n of neighborsOf(shape, p)) {
                    expect(neighborsOf(shape, n)).toContain(p);
                }
            }
        });

        test('no self-loops, no duplicate neighbors', () => {
            for (let p = 0; p < shape.numPoints; p++) {
                const ns = neighborsOf(shape, p);
                expect(ns).not.toContain(p);
                expect(new Set(ns).size).toBe(ns.length);
            }
        });

        test('every adjacency pair shares a face edge, and every face edge is an adjacency pair', () => {
            const edgeKeys = faceEdgeKeys(shape);
            const adjKeys = new Set<string>();
            for (let p = 0; p < shape.numPoints; p++) {
                for (const n of neighborsOf(shape, p)) {
                    adjKeys.add(p < n ? `${p},${n}` : `${n},${p}`);
                }
            }
            for (const k of adjKeys) expect(edgeKeys).toContain(k);
            for (const k of edgeKeys) expect(adjKeys).toContain(k);
        });

        test('Σ valencies === Σ face vertex counts (handshaking / 2·edgeCount)', () => {
            let faceVertexCount = 0;
            for (const face of shape.faces) faceVertexCount += face.numVertices;
            expect(shape.pointNeighbors.length).toBe(faceVertexCount);
        });

        test('spans all vertices (every vertex has ≥1 neighbor)', () => {
            for (let p = 0; p < shape.numPoints; p++) {
                expect(shape.pointNeighborsStart[p + 1] - shape.pointNeighborsStart[p]).toBeGreaterThan(0);
            }
        });
    });
});
