import { type Mat4, mat4, quat, type Vec3, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { convexHull } from '../../src';
import { createSupportingFaceResult, type SupportingFaceResult } from '../../src/shapes/shapes';
import { isScaleInsideOut, transformFaceWithMat4Scale } from '../../src/utils/face';

const MAX_FACE_VERTICES = 32;

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

const _invScale = vec3.create();
const _planeNormal = vec3.create();

/** verbatim reimplementation of the pre-change getSupportingFace selection (sqrt per plane per call) */
function refGetSupportingFace(
    ioResult: SupportingFaceResult,
    direction: Vec3,
    shape: ReturnType<typeof convexHull.create>,
): void {
    const face = ioResult.face;
    const scale = ioResult.scale;
    const transform = ioResult.transform;

    vec3.set(_invScale, 1 / scale[0], 1 / scale[1], 1 / scale[2]);
    vec3.multiply(_planeNormal, _invScale, shape.planes[0].normal);
    let bestDot = vec3.dot(_planeNormal, direction) / vec3.length(_planeNormal);
    let bestFaceIdx = 0;
    for (let i = 1; i < shape.planes.length; i++) {
        vec3.multiply(_planeNormal, _invScale, shape.planes[i].normal);
        const dot = vec3.dot(_planeNormal, direction) / vec3.length(_planeNormal);
        if (dot < bestDot) {
            bestDot = dot;
            bestFaceIdx = i;
        }
    }

    const bestFace = shape.faces[bestFaceIdx];
    const firstVtxIdx = bestFace.firstVertex;
    const numVertices = bestFace.numVertices;
    const maxVerticesToReturn = Math.floor(MAX_FACE_VERTICES / 2);
    const deltaVtx = Math.floor((numVertices + maxVerticesToReturn - 1) / maxVerticesToReturn);
    const insideOut = isScaleInsideOut(scale);

    face.numVertices = 0;
    if (insideOut) {
        for (let i = numVertices - 1; i >= 0; i -= deltaVtx) {
            const vtxIdx = shape.vertexIndices[firstVtxIdx + i];
            const pbase = vtxIdx * 3;
            const base = face.numVertices * 3;
            face.vertices[base] = shape.pointPositions[pbase];
            face.vertices[base + 1] = shape.pointPositions[pbase + 1];
            face.vertices[base + 2] = shape.pointPositions[pbase + 2];
            face.numVertices++;
        }
    } else {
        for (let i = 0; i < numVertices; i += deltaVtx) {
            const vtxIdx = shape.vertexIndices[firstVtxIdx + i];
            const pbase = vtxIdx * 3;
            const base = face.numVertices * 3;
            face.vertices[base] = shape.pointPositions[pbase];
            face.vertices[base + 1] = shape.pointPositions[pbase + 1];
            face.vertices[base + 2] = shape.pointPositions[pbase + 2];
            face.numVertices++;
        }
    }

    transformFaceWithMat4Scale(face, transform, scale);
}

// a fixed non-trivial transform (shared by ref + actual, so it cancels in the comparison)
const TRANSFORM: Mat4 = mat4.fromRotationTranslation(
    mat4.create(),
    quat.setAxisAngle(quat.create(), vec3.normalize(vec3.create(), vec3.fromValues(1, 2, -3)), 0.7),
    vec3.fromValues(5, -6, 7),
);

const UNIFORM_SCALES: Vec3[] = [
    vec3.fromValues(0.5, 0.5, 0.5),
    vec3.fromValues(1, 1, 1),
    vec3.fromValues(2.37, 2.37, 2.37),
    vec3.fromValues(-1.5, -1.5, -1.5), // mirrored uniform
];

const NON_UNIFORM_SCALES: Vec3[] = [vec3.fromValues(2, 0.5, 3), vec3.fromValues(-2, 1, 0.5)];

const HULLS: Array<[string, ReturnType<typeof convexHull.create>]> = [
    ['box', convexHull.create({ positions: BOX_POINTS, convexRadius: 0.05 })],
    ['fibonacci 100', convexHull.create({ positions: fibonacciSphere(100), convexRadius: 0.05 })],
    ['slab grid', convexHull.create({ positions: slabGrid(), convexRadius: 0.02, bakeSupportAdjacency: true })],
];

function expectSameFace(shape: ReturnType<typeof convexHull.create>, scale: Vec3, dirs: Vec3[]): void {
    const actual = createSupportingFaceResult();
    const ref = createSupportingFaceResult();
    mat4.copy(actual.transform, TRANSFORM);
    mat4.copy(ref.transform, TRANSFORM);
    vec3.copy(actual.scale, scale);
    vec3.copy(ref.scale, scale);

    for (const dir of dirs) {
        convexHull.def.getSupportingFace(actual, dir, shape, 0);
        refGetSupportingFace(ref, dir, shape);
        expect(actual.face.numVertices).toBe(ref.face.numVertices);
        for (let i = 0; i < ref.face.numVertices * 3; i++) {
            expect(actual.face.vertices[i]).toBeCloseTo(ref.face.vertices[i], 10);
        }
    }
}

describe('convex hull getSupportingFace — uniform fast path equivalence', () => {
    const dirs = [
        ...randomDirections(80, 0xabcdef),
        vec3.fromValues(1, 0, 0),
        vec3.fromValues(0, -1, 0),
        vec3.fromValues(0, 0, 1),
    ];

    for (const [name, shape] of HULLS) {
        for (const scale of UNIFORM_SCALES) {
            test(`${name} · uniform scale [${scale[0]}] matches the sqrt reference`, () => {
                expectSameFace(shape, scale, dirs);
            });
        }

        // the non-uniform path is unchanged; assert it still matches the reference (regression guard)
        for (const scale of NON_UNIFORM_SCALES) {
            test(`${name} · non-uniform scale [${scale[0]},${scale[1]},${scale[2]}] matches the reference`, () => {
                expectSameFace(shape, scale, dirs);
            });
        }
    }
});
