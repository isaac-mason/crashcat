import { mat4, quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    castRayVsShape,
    compound,
    createAllCastRayCollector,
    createDefaultCastRaySettings,
    createFace,
    EMPTY_SUB_SHAPE_ID,
    getShapeSupportingFace,
    getShapeSurfaceNormal,
    rigidBody,
    type Shape,
    ShapeType,
    sphere,
    subShape,
} from '../../src';

describe('Compound shape AABB caching', () => {
    test('compound shape should have correct AABB', () => {
        const s = sphere.create({ radius: 1.0 });
        const b = box.create({ halfExtents: vec3.fromValues(2, 2, 2) });
        const cs = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s },
                { position: vec3.create(), quaternion: quat.create(), shape: b },
            ],
        });
        expect(cs.aabb).toBeDefined();
        // Union of sphere [-1,-1,-1] to [1,1,1] and box [-2,-2,-2] to [2,2,2]
        // Result: [-2,-2,-2] to [2,2,2]
        expect(cs.aabb[0]).toBe(-2);
        expect(cs.aabb[1]).toBe(-2);
        expect(cs.aabb[2]).toBe(-2);
        expect(cs.aabb[3]).toBe(2);
        expect(cs.aabb[4]).toBe(2);
        expect(cs.aabb[5]).toBe(2);
    });
});

describe('Compound shape creation', () => {
    test('should create a compound shape', () => {
        const s = sphere.create({ radius: 1.0 });
        const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const cs = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s },
                { position: vec3.create(), quaternion: quat.create(), shape: b },
            ],
        });

        expect(cs.type).toBe(ShapeType.COMPOUND);
        expect(cs.children.length).toBe(2);
        expect(cs.children[0].shape).toBe(s);
        expect(cs.children[1].shape).toBe(b);
    });
});

describe('Compound shape center of mass', () => {
    test('compound shape should have mass-weighted center of mass', () => {
        const s = sphere.create({ radius: 1.0 });
        const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const cs = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s },
                { position: vec3.create(), quaternion: quat.create(), shape: b },
            ],
        });

        // Both are at origin, so COM should be origin
        expect(cs.centerOfMass[0]).toBe(0);
        expect(cs.centerOfMass[1]).toBe(0);
        expect(cs.centerOfMass[2]).toBe(0);
    });

    test('compound shape with offset children should have mass-weighted COM', () => {
        // Create two spheres with same radius (equal mass)
        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });

        const cs = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(4, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        // Equal mass spheres: mass-weighted average of [0,0,0] and [4,0,0] is [2,0,0]
        expect(cs.centerOfMass[0]).toBeCloseTo(2, 5);
        expect(cs.centerOfMass[1]).toBeCloseTo(0, 5);
        expect(cs.centerOfMass[2]).toBeCloseTo(0, 5);
    });
});

describe('Compound shape sub shape ids', () => {
    // the last child of a power of two count pushed onto an empty id is all ones, the same bits as the empty id
    const makePair = (second: Shape) =>
        compound.create({
            children: [
                { position: vec3.fromValues(-2, 0, 0), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
                { position: vec3.fromValues(2, 0, 0), quaternion: quat.create(), shape: second },
            ],
        });

    function rayHitSubShapeId(shape: Shape, originX: number): number {
        const collector = createAllCastRayCollector();
        castRayVsShape(
            collector,
            createDefaultCastRaySettings(),
            originX,
            10,
            0,
            0,
            -1,
            0,
            20,
            shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
        );
        expect(collector.hits.length).toBe(1);
        return collector.hits[0].subShapeId;
    }

    test("the second of two children resolves through the ray hit's id, the leaf and the transformed shape lookups", () => {
        const leaf = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const shape = makePair(leaf);

        const id = rayHitSubShapeId(shape, 2);
        expect(id).toBe(0xffffffff);

        const leafResult = rigidBody.createGetLeafShapeResult();
        rigidBody.getLeafShape(leafResult, shape, id);
        expect(leafResult.shape).toBe(leaf);

        const transformedResult = rigidBody.createGetSubShapeTransformedShapeResult();
        rigidBody.getSubShapeTransformedShape(
            transformedResult,
            shape,
            id,
            vec3.fromValues(0, 0, 0),
            quat.create(),
            vec3.fromValues(1, 1, 1),
        );
        expect(transformedResult.shape).toBe(leaf);
        expect(transformedResult.position[0]).toBe(2);

        const normal = getShapeSurfaceNormal(vec3.create(), shape, vec3.fromValues(2, 1, 0), id);
        expect(normal[1]).toBeCloseTo(1, 6);
    });

    test('the path 1, 1 through two compounds of two resolves to the inner second child', () => {
        const leaf = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const inner = makePair(leaf);
        const outer = makePair(inner);

        // inner child 1 sits at 2 + 2 along x
        const id = rayHitSubShapeId(outer, 4);
        expect(id).toBe(0xffffffff);

        const leafResult = rigidBody.createGetLeafShapeResult();
        rigidBody.getLeafShape(leafResult, outer, id);
        expect(leafResult.shape).toBe(leaf);

        const transformedResult = rigidBody.createGetSubShapeTransformedShapeResult();
        rigidBody.getSubShapeTransformedShape(
            transformedResult,
            outer,
            id,
            vec3.fromValues(0, 0, 0),
            quat.create(),
            vec3.fromValues(1, 1, 1),
        );
        expect(transformedResult.shape).toBe(leaf);
        expect(transformedResult.position[0]).toBe(4);
    });

    test('a one child compound consumes no bits and keeps the rest of the path', () => {
        const first = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const pair = compound.create({
            children: [
                { position: vec3.fromValues(-2, 0, 0), quaternion: quat.create(), shape: first },
                { position: vec3.fromValues(2, 0, 0), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
            ],
        });
        const wrapper = compound.create({ children: [{ position: vec3.create(), quaternion: quat.create(), shape: pair }] });

        const id = rayHitSubShapeId(wrapper, -2);
        expect(id).toBe(0xfffffffe);

        const leafResult = rigidBody.createGetLeafShapeResult();
        rigidBody.getLeafShape(leafResult, wrapper, id);
        expect(leafResult.shape).toBe(first);

        const popped = subShape.popResult();
        subShape.pop(popped, id, 0);
        expect(popped.value).toBe(0);
        expect(popped.remainder).toBe(id);
    });

    test('the leaf lookup under a scaled compound places the child at the scaled position', () => {
        const leaf = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const shape = makePair(leaf);
        const result = rigidBody.createGetSubShapeTransformedShapeResult();
        rigidBody.getSubShapeTransformedShape(
            result,
            shape,
            0xffffffff,
            vec3.fromValues(0, 0, 0),
            quat.create(),
            vec3.fromValues(3, 1, 1),
        );
        expect(result.shape).toBe(leaf);
        expect(result.position[0]).toBe(6);
    });
});

describe('Compound shape surface normal', () => {
    test('a compound inside a rotated compound returns the world normal', () => {
        // outer turns its child a quarter about z, inner turns its box a quarter about x
        const quarterZ = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);
        const quarterX = quat.setAxisAngle(quat.create(), vec3.fromValues(1, 0, 0), Math.PI / 2);
        const leaf = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        const inner = compound.create({ children: [{ position: vec3.create(), quaternion: quarterX, shape: leaf }] });
        const outer = compound.create({ children: [{ position: vec3.create(), quaternion: quarterZ, shape: inner }] });

        // the box's +y face centre (0, 2, 0) goes to (0, 0, 2) under x, which z leaves alone
        const normal = getShapeSurfaceNormal(vec3.create(), outer, vec3.fromValues(0, 0, 2), EMPTY_SUB_SHAPE_ID);
        expect(normal[0]).toBeCloseTo(0, 6);
        expect(normal[1]).toBeCloseTo(0, 6);
        expect(normal[2]).toBeCloseTo(1, 6);
    });
});

describe('Compound shape supporting face', () => {
    test('a scaled compound of two boxes returns the face at the scaled position', () => {
        const shape = compound.create({
            children: [
                {
                    position: vec3.fromValues(-2, 0, 0),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                },
                {
                    position: vec3.fromValues(2, 0, 0),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                },
            ],
        });

        // the second box under scale (2, 1, 1): centre x = 4, half extent 2, so its +x face is at x = 6
        const face = createFace();
        getShapeSupportingFace(face, shape, 0xffffffff, vec3.fromValues(-1, 0, 0), mat4.create(), vec3.fromValues(2, 1, 1));
        expect(face.numVertices).toBe(4);
        for (let i = 0; i < 4; i++) {
            expect(face.vertices[i * 3]).toBeCloseTo(6, 6);
        }
    });
});
