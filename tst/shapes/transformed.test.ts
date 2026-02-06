import { describe, expect, test } from 'vitest';
import { quat, vec3 } from 'mathcat';
import { ShapeType, sphere, transformed } from '../../src';

describe('Transformed shape AABB caching', () => {
    test('transformed shape should have correct AABB (no rotation)', () => {
        const s = sphere.create({ radius: 1.0 });
        const ts = transformed.create({
            shape: s,
            position: vec3.fromValues(5, 3, 2),
            quaternion: quat.create(),
        });
        expect(ts.aabb).toBeDefined();
        // Transformed sphere with translation [5,3,2] should have AABB offset by that amount
        // Original sphere AABB: [-1,-1,-1] to [1,1,1]
        // After translation: [4,2,1] to [6,4,3]
        expect(ts.aabb[0][0]).toBe(4);
        expect(ts.aabb[0][1]).toBe(2);
        expect(ts.aabb[0][2]).toBe(1);
        expect(ts.aabb[1][0]).toBe(6);
        expect(ts.aabb[1][1]).toBe(4);
        expect(ts.aabb[1][2]).toBe(3);
    });
});

describe('Transformed shape creation', () => {
    test('should create a transformed shape', () => {
        const s = sphere.create({ radius: 1.0 });
        const trans = vec3.fromValues(5, 3, 2);
        const rot = quat.create();
        const ts = transformed.create({ shape: s, position: trans, quaternion: rot });

        expect(ts.type).toBe(ShapeType.TRANSFORMED);
        expect(ts.shape).toBe(s);
        expect(ts.position[0]).toBe(5);
        expect(ts.quaternion[3]).toBe(1); // quat.create() has w=1
    });
});

describe('Transformed shape center of mass', () => {
    test('transformed sphere should have center of mass at translation offset', () => {
        const s = sphere.create({ radius: 1.0 });
        const position = vec3.fromValues(5, 3, 2);
        const ts = transformed.create({
            shape: s,
            position,
            quaternion: quat.create(), // Identity rotation
        });
        // COM should be offset by translation
        expect(ts.centerOfMass[0]).toBeCloseTo(5, 5);
        expect(ts.centerOfMass[1]).toBeCloseTo(3, 5);
        expect(ts.centerOfMass[2]).toBeCloseTo(2, 5);
    });

    test('transformed sphere with rotation should have center of mass transformed', () => {
        const s = sphere.create({ radius: 1.0 });
        const position = vec3.fromValues(1, 0, 0);

        // Rotate 90° around Z axis
        const quaternion = quat.create();
        quat.setAxisAngle(quaternion, [0, 0, 1], Math.PI / 2);
        const ts = transformed.create({
            shape: s,
            position,
            quaternion,
        });

        // COM at origin rotated by 90° around Z = still origin, plus translation [1, 0, 0]
        expect(ts.centerOfMass[0]).toBeCloseTo(1, 5);
        expect(ts.centerOfMass[1]).toBeCloseTo(0, 5);
        expect(ts.centerOfMass[2]).toBeCloseTo(0, 5);
    });
});
