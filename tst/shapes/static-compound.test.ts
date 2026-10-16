import { mat4, quat, vec3 } from 'math';
import { box3 } from 'math/shapes';
import { describe, expect, test } from 'vitest';
import {
    box,
    bvh,
    CastRayStatus,
    CastShapeStatus,
    castRayVsShape,
    castShapeVsShape,
    collidePointVsShape,
    collideShapeVsShape,
    createAllCastRayCollector,
    createAllCastShapeCollector,
    createAllCollidePointCollector,
    createAllCollideShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    createFace,
    EMPTY_SUB_SHAPE_ID,
    getShapeSupportingFace,
    getShapeSurfaceNormal,
    rigidBody,
    type Shape,
    ShapeType,
    sphere,
    staticCompound,
    staticCompoundBvh,
} from '../../src';

function createRayParams(origin: [number, number, number], direction: [number, number, number], length = 1000) {
    const dir = vec3.fromValues(...direction);
    vec3.normalize(dir, dir);
    return {
        originX: origin[0],
        originY: origin[1],
        originZ: origin[2],
        directionX: dir[0],
        directionY: dir[1],
        directionZ: dir[2],
        length,
    };
}

const defaultCastRaySettings = createDefaultCastRaySettings();
const defaultCollidePointSettings = createDefaultCollidePointSettings();

describe('StaticCompoundShape creation', () => {
    test('should create empty static compound shape', () => {
        const shape = staticCompound.create({ children: [] });

        expect(shape.type).toBe(ShapeType.STATIC_COMPOUND);
        expect(shape.children.length).toBe(0);
        expect(shape.bvh.buffer.length).toBe(0);
        expect(shape.volume).toBe(0);
    });

    test('should create static compound shape with single child', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.create(), quaternion: quat.create(), shape: s }],
        });

        expect(shape.type).toBe(ShapeType.STATIC_COMPOUND);
        expect(shape.children.length).toBe(1);
        expect(shape.bvh.buffer.length).toBeGreaterThan(0);
        expect(shape.volume).toBeCloseTo((4 / 3) * Math.PI, 4);
    });

    test('should create static compound shape with multiple children', () => {
        const s = sphere.create({ radius: 1.0 });
        const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const shape = staticCompound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s },
                { position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: b },
            ],
        });

        expect(shape.type).toBe(ShapeType.STATIC_COMPOUND);
        expect(shape.children.length).toBe(2);
        expect(shape.bvh.buffer.length).toBeGreaterThan(0);
    });
});

describe('StaticCompoundShape AABB', () => {
    test('should have correct AABB for single child at origin', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.create(), quaternion: quat.create(), shape: s }],
        });

        expect(shape.aabb[0]).toBeCloseTo(-1, 5);
        expect(shape.aabb[1]).toBeCloseTo(-1, 5);
        expect(shape.aabb[2]).toBeCloseTo(-1, 5);
        expect(shape.aabb[3]).toBeCloseTo(1, 5);
        expect(shape.aabb[4]).toBeCloseTo(1, 5);
        expect(shape.aabb[5]).toBeCloseTo(1, 5);
    });

    test('should have correct AABB for offset children', () => {
        const s = sphere.create({ radius: 1.0 });
        const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const shape = staticCompound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s },
                { position: vec3.fromValues(10, 0, 0), quaternion: quat.create(), shape: b },
            ],
        });

        // sphere at origin: [-1, -1, -1] to [1, 1, 1]
        // box at (10, 0, 0): [9, -1, -1] to [11, 1, 1]
        // union: [-1, -1, -1] to [11, 1, 1]
        expect(shape.aabb[0]).toBeCloseTo(-1, 5);
        expect(shape.aabb[1]).toBeCloseTo(-1, 5);
        expect(shape.aabb[2]).toBeCloseTo(-1, 5);
        expect(shape.aabb[3]).toBeCloseTo(11, 5);
        expect(shape.aabb[4]).toBeCloseTo(1, 5);
        expect(shape.aabb[5]).toBeCloseTo(1, 5);
    });
});

describe('StaticCompoundShape center of mass', () => {
    test('should have center of mass at origin for single centered child', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.create(), quaternion: quat.create(), shape: s }],
        });

        expect(shape.centerOfMass[0]).toBeCloseTo(0, 5);
        expect(shape.centerOfMass[1]).toBeCloseTo(0, 5);
        expect(shape.centerOfMass[2]).toBeCloseTo(0, 5);
    });

    test('should have mass-weighted center of mass for offset children', () => {
        // two equal-mass spheres
        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(4, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        // mass-weighted average of [0,0,0] and [4,0,0] is [2,0,0]
        expect(shape.centerOfMass[0]).toBeCloseTo(2, 5);
        expect(shape.centerOfMass[1]).toBeCloseTo(0, 5);
        expect(shape.centerOfMass[2]).toBeCloseTo(0, 5);
    });
});

describe('StaticCompoundShape BVH', () => {
    test('should build BVH with correct structure', () => {
        const shapes: ReturnType<typeof sphere.create>[] = [];
        for (let i = 0; i < 10; i++) {
            shapes.push(sphere.create({ radius: 0.5 }));
        }

        const shape = staticCompound.create({
            children: shapes.map((s, i) => ({
                position: vec3.fromValues(i * 2, 0, 0),
                quaternion: quat.create(),
                shape: s,
            })),
        });

        expect(shape.bvh.buffer.length).toBeGreaterThan(0);

        const s = staticCompoundBvh.stats(shape.bvh);
        expect(s.totalNodes).toBeGreaterThan(0);
        expect(s.leafNodes).toBeGreaterThan(0);
        expect(s.maxDepth).toBeGreaterThan(0);
        expect(s.totalChildren).toBe(10);
    });
});

describe('StaticCompoundShape update', () => {
    test('should rebuild BVH on update', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.create(), quaternion: quat.create(), shape: s }],
        });

        const originalBufferLength = shape.bvh.buffer.length;

        // add more children manually
        const s2 = sphere.create({ radius: 1.0 });
        shape.children.push({ position: vec3.fromValues(10, 0, 0), quaternion: quat.create(), shape: s2 });

        // call update
        staticCompound.update(shape);

        // BVH should be rebuilt
        expect(shape.bvh.buffer.length).toBeGreaterThanOrEqual(originalBufferLength);
        expect(shape.aabb[3]).toBeCloseTo(11, 5);
    });
});

describe('StaticCompoundShape castRay', () => {
    test('should hit child sphere with ray', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: s }],
        });

        const ray = createRayParams([0, 0, 0], [1, 0, 0]);
        const collector = createAllCastRayCollector();

        castRayVsShape(
            collector,
            defaultCastRaySettings,
            ray.originX,
            ray.originY,
            ray.originZ,
            ray.directionX,
            ray.directionY,
            ray.directionZ,
            ray.length,
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
        expect(collector.hits[0].status).toBe(CastRayStatus.COLLIDING);
        // hit should be at distance ~4 (sphere center at 5, radius 1)
        expect(collector.hits[0].fraction).toBeGreaterThan(0);
    });

    test('should miss child sphere when ray points away', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: s }],
        });

        const ray = createRayParams([0, 0, 0], [-1, 0, 0]);
        const collector = createAllCastRayCollector();

        castRayVsShape(
            collector,
            defaultCastRaySettings,
            ray.originX,
            ray.originY,
            ray.originZ,
            ray.directionX,
            ray.directionY,
            ray.directionZ,
            ray.length,
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

        expect(collector.hits.length).toBe(0);
    });

    test('should hit multiple children with ray', () => {
        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [
                { position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(10, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        const ray = createRayParams([0, 0, 0], [1, 0, 0]);
        const collector = createAllCastRayCollector();

        castRayVsShape(
            collector,
            defaultCastRaySettings,
            ray.originX,
            ray.originY,
            ray.originZ,
            ray.directionX,
            ray.directionY,
            ray.directionZ,
            ray.length,
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

        expect(collector.hits.length).toBe(2);
    });

    test('should hit static compound with BVH culling (many children)', () => {
        // create a grid of spheres along x-axis
        const children: {
            position: ReturnType<typeof vec3.create>;
            quaternion: ReturnType<typeof quat.create>;
            shape: ReturnType<typeof sphere.create>;
        }[] = [];
        for (let i = 0; i < 50; i++) {
            children.push({
                position: vec3.fromValues(i * 3, 0, 0),
                quaternion: quat.create(),
                shape: sphere.create({ radius: 1.0 }),
            });
        }

        const shape = staticCompound.create({ children });

        // ray along x-axis should hit all spheres
        const ray = createRayParams([0, 0, 0], [1, 0, 0], 1000);
        const collector = createAllCastRayCollector();

        castRayVsShape(
            collector,
            defaultCastRaySettings,
            ray.originX,
            ray.originY,
            ray.originZ,
            ray.directionX,
            ray.directionY,
            ray.directionZ,
            ray.length,
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

        expect(collector.hits.length).toBe(50);
    });
});

describe('StaticCompoundShape collidePoint', () => {
    test('should detect point inside child sphere', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: s }],
        });

        const collector = createAllCollidePointCollector();

        collidePointVsShape(
            collector,
            defaultCollidePointSettings,
            5,
            0,
            0,
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
    });

    test('should not detect point outside all children', () => {
        const s = sphere.create({ radius: 1.0 });
        const shape = staticCompound.create({
            children: [{ position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: s }],
        });

        const collector = createAllCollidePointCollector();

        collidePointVsShape(
            collector,
            defaultCollidePointSettings,
            0,
            0,
            0,
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

        expect(collector.hits.length).toBe(0);
    });

    test('should detect point inside overlapping children', () => {
        const s1 = sphere.create({ radius: 2.0 });
        const s2 = sphere.create({ radius: 2.0 });
        const shape = staticCompound.create({
            children: [
                { position: vec3.fromValues(0, 0, 0), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(1, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        const collector = createAllCollidePointCollector();

        // point at x=0.5 is inside both spheres
        collidePointVsShape(
            collector,
            defaultCollidePointSettings,
            0.5,
            0,
            0,
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

        expect(collector.hits.length).toBe(2);
    });
});

describe('StaticCompoundShape BVH efficiency', () => {
    test('BVH should correctly bound all children', () => {
        const children: {
            position: ReturnType<typeof vec3.create>;
            quaternion: ReturnType<typeof quat.create>;
            shape: ReturnType<typeof sphere.create>;
        }[] = [];
        for (let x = 0; x < 5; x++) {
            for (let y = 0; y < 5; y++) {
                for (let z = 0; z < 5; z++) {
                    children.push({
                        position: vec3.fromValues(x * 3, y * 3, z * 3),
                        quaternion: quat.create(),
                        shape: sphere.create({ radius: 1.0 }),
                    });
                }
            }
        }

        const shape = staticCompound.create({ children });
        const s = staticCompoundBvh.stats(shape.bvh);

        expect(s.totalChildren).toBe(125);
        // binary BVH should have good depth for 125 children
        // log2(125) ≈ 7, so max depth should be reasonable
        expect(s.maxDepth).toBeGreaterThanOrEqual(3);
        expect(s.maxDepth).toBeLessThanOrEqual(20);
    });
});

describe('StaticCompoundShape vs CompoundShape parity', () => {
    test('should produce same AABB as CompoundShape', () => {
        const s = sphere.create({ radius: 1.0 });
        const b = box.create({ halfExtents: vec3.fromValues(2, 2, 2) });

        const children = [
            { position: vec3.create(), quaternion: quat.create(), shape: s },
            { position: vec3.fromValues(5, 0, 0), quaternion: quat.create(), shape: b },
        ];

        const staticShape = staticCompound.create({ children: [...children] });

        // AABB should be union of:
        // sphere at origin: [-1,-1,-1] to [1,1,1]
        // box at (5,0,0): [3,-2,-2] to [7,2,2]
        // result: [-1,-2,-2] to [7,2,2]
        expect(staticShape.aabb[0]).toBeCloseTo(-1, 5);
        expect(staticShape.aabb[1]).toBeCloseTo(-2, 5);
        expect(staticShape.aabb[2]).toBeCloseTo(-2, 5);
        expect(staticShape.aabb[3]).toBeCloseTo(7, 5);
        expect(staticShape.aabb[4]).toBeCloseTo(2, 5);
        expect(staticShape.aabb[5]).toBeCloseTo(2, 5);
    });
});

describe('StaticCompoundShape sub shape ids', () => {
    // the last child of a power of two count pushed onto an empty id is all ones, the same bits as the empty id
    const makePair = (second: Shape) =>
        staticCompound.create({
            children: [
                { position: vec3.fromValues(-2, 0, 0), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
                { position: vec3.fromValues(2, 0, 0), quaternion: quat.create(), shape: second },
            ],
        });

    function rayHitSubShapeId(shape: Shape, originX: number): number {
        const collector = createAllCastRayCollector();
        castRayVsShape(
            collector,
            defaultCastRaySettings,
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
        expect(shape.children[1].shape).toBe(leaf);

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

    test('the path 1, 1 through two static compounds of two resolves to the inner second child', () => {
        const leaf = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const inner = makePair(leaf);
        const outer = makePair(inner);

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

    test('the leaf lookup under a scaled static compound places the child at the scaled position', () => {
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

describe('StaticCompoundShape surface normal', () => {
    test('a static compound inside a rotated static compound returns the world normal', () => {
        const quarterZ = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);
        const quarterX = quat.setAxisAngle(quat.create(), vec3.fromValues(1, 0, 0), Math.PI / 2);
        const leaf = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        const inner = staticCompound.create({ children: [{ position: vec3.create(), quaternion: quarterX, shape: leaf }] });
        const outer = staticCompound.create({ children: [{ position: vec3.create(), quaternion: quarterZ, shape: inner }] });

        // the box's +y face centre (0, 2, 0) goes to (0, 0, 2) under x, which z leaves alone
        const normal = getShapeSurfaceNormal(vec3.create(), outer, vec3.fromValues(0, 0, 2), EMPTY_SUB_SHAPE_ID);
        expect(normal[0]).toBeCloseTo(0, 6);
        expect(normal[1]).toBeCloseTo(0, 6);
        expect(normal[2]).toBeCloseTo(1, 6);
    });
});

describe('StaticCompoundShape supporting face', () => {
    test('a scaled static compound of two boxes returns the face at the scaled position', () => {
        const shape = staticCompound.create({
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
        expect(shape.children[1].position[0]).toBe(2);

        // the second box under scale (2, 1, 1): centre x = 4, half extent 2, so its +x face is at x = 6
        const face = createFace();
        getShapeSupportingFace(face, shape, 0xffffffff, vec3.fromValues(-1, 0, 0), mat4.create(), vec3.fromValues(2, 1, 1));
        expect(face.numVertices).toBe(4);
        for (let i = 0; i < 4; i++) {
            expect(face.vertices[i * 3]).toBeCloseTo(6, 6);
        }
    });
});

describe('StaticCompoundShape as A culls with the scale of B', () => {
    const identity = quat.create();
    const makeCompound = () =>
        staticCompound.create({
            children: [
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                },
            ],
        });

    test('collide: a unit sphere scaled by 3 at x = 3.5 reaches x = 0.5, inside the box', () => {
        // unscaled, the sphere's bounds start at x = 2.5 and miss the box's [-1, 1]
        const collector = createAllCollideShapeCollector();
        collideShapeVsShape(
            collector,
            createDefaultCollideShapeSettings(),
            makeCompound(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...identity,
            1,
            1,
            1,
            sphere.create({ radius: 1 }),
            EMPTY_SUB_SHAPE_ID,
            0,
            3.5,
            0,
            0,
            ...identity,
            3,
            3,
            3,
        );
        expect(collector.hits.length).toBe(1);
        expect(collector.hits[0].penetration).toBeCloseTo(0.5, 4);
    });

    test('cast: the compound moving 4 along x meets a sphere scaled by 3 at x = 7 three quarters of the way', () => {
        // the box's +x face travels 1 -> 5 and meets the sphere's near side at x = 4. unscaled, the
        // sphere's swept bounds [2, 8] miss the box
        const collector = createAllCastShapeCollector();
        castShapeVsShape(
            collector,
            createDefaultCastShapeSettings(),
            makeCompound(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            ...identity,
            1,
            1,
            1,
            4,
            0,
            0,
            sphere.create({ radius: 1 }),
            EMPTY_SUB_SHAPE_ID,
            0,
            7,
            0,
            0,
            ...identity,
            3,
            3,
            3,
        );
        expect(collector.hits.length).toBe(1);
        expect(collector.hits[0].status).toBe(CastShapeStatus.COLLIDING);
        expect(collector.hits[0].fraction).toBeCloseTo(0.75, 3);
    });
});

describe('StaticCompoundShape BVH node bounds', () => {
    test('every node contains its children exactly, for coordinates with no exact binary form', () => {
        const children = [];
        for (let i = 0; i < 64; i++) {
            children.push({
                position: vec3.fromValues(i * 0.1 + 0.37, (i % 7) * 0.13 - 0.3, (i % 5) * 0.29 + 0.01),
                quaternion: quat.create(),
                shape: box.create({ halfExtents: vec3.fromValues(0.21, 0.07, 0.13), convexRadius: 0 }),
            });
        }
        const shape = staticCompound.create({ children, bvhMaxLeafChildren: 2 });
        const buffer = shape.bvh.buffer;

        const childBounds = box3.create();
        const childTransform = mat4.create();
        const contains = (offset: number, b: ArrayLike<number>) =>
            buffer[offset + bvh.NODE_MIN_X] <= b[0] &&
            buffer[offset + bvh.NODE_MIN_Y] <= b[1] &&
            buffer[offset + bvh.NODE_MIN_Z] <= b[2] &&
            buffer[offset + bvh.NODE_MAX_X] >= b[3] &&
            buffer[offset + bvh.NODE_MAX_Y] >= b[4] &&
            buffer[offset + bvh.NODE_MAX_Z] >= b[5];
        const nodeBounds = (offset: number) => buffer.slice(offset + bvh.NODE_MIN_X, offset + bvh.NODE_MAX_Z + 1);

        let checked = 0;
        const stack = [0];
        while (stack.length > 0) {
            const offset = stack.pop()!;
            if (bvh.nodeIsLeaf(buffer, offset)) {
                const start = staticCompoundBvh.nodeChildStart(buffer, offset);
                const count = staticCompoundBvh.nodeChildCount(buffer, offset);
                for (let i = start; i < start + count; i++) {
                    const child = shape.children[i];
                    mat4.fromRotationTranslation(childTransform, child.quaternion, child.position);
                    box3.transformMat4(childBounds, child.shape.aabb, childTransform);
                    expect(contains(offset, childBounds)).toBe(true);
                    checked++;
                }
            } else {
                const left = bvh.nodeLeft(offset);
                const right = bvh.nodeRight(buffer, offset);
                expect(contains(offset, nodeBounds(left))).toBe(true);
                expect(contains(offset, nodeBounds(right))).toBe(true);
                stack.push(left, right);
            }
        }
        expect(checked).toBe(64);
    });
});
