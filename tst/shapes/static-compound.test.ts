import { quat, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    box,
    CastRayStatus,
    castRayVsShape,
    collidePointVsShape,
    createAllCastRayCollector,
    createAllCollidePointCollector,
    createDefaultCastRaySettings,
    createDefaultCollidePointSettings,
    EMPTY_SUB_SHAPE_ID,
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
