import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, offsetCenterOfMass, ShapeType, scaled, sphere, transformed } from '../../src';

describe('Scaled shape AABB caching', () => {
    test('scaled shape should have correct AABB', () => {
        const s = sphere.create({ radius: 1.0 });
        const ss = scaled.create({ shape: s, scale: vec3.fromValues(2, 3, 4) });
        expect(ss.aabb).toBeDefined();
        // Scaled sphere: [-2, -3, -4] to [2, 3, 4]
        expect(ss.aabb[0]).toBe(-2);
        expect(ss.aabb[1]).toBe(-3);
        expect(ss.aabb[2]).toBe(-4);
        expect(ss.aabb[3]).toBe(2);
        expect(ss.aabb[4]).toBe(3);
        expect(ss.aabb[5]).toBe(4);
    });
});

describe('Scaled shape mirrored bounds', () => {
    test('a negative scale mirrors the bounds of an off-origin inner shape', () => {
        const inner = transformed.create({
            shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1), convexRadius: 0 }),
            position: vec3.fromValues(3, 0, 0),
            quaternion: quat.create(),
        });
        const ss = scaled.create({ shape: inner, scale: vec3.fromValues(-2, 1, 1) });

        // inner spans x in [2, 4], mirrored and doubled to [-8, -4]
        expect(ss.aabb[0]).toBe(-8);
        expect(ss.aabb[3]).toBe(-4);
        expect(ss.aabb[1]).toBe(-1);
        expect(ss.aabb[4]).toBe(1);
    });
});

describe('Scaled shape creation', () => {
    test('should create a scaled shape', () => {
        const s = sphere.create({ radius: 1.0 });
        const scale = vec3.fromValues(2, 3, 4);
        const ss = scaled.create({ shape: s, scale });

        expect(ss.type).toBe(ShapeType.SCALED);
        expect(ss.shape).toBe(s);
        expect(ss.scale[0]).toBe(2);
        expect(ss.scale[1]).toBe(3);
        expect(ss.scale[2]).toBe(4);
    });
});

describe('Scaled shape center of mass', () => {
    test('scaled sphere should have center of mass at origin', () => {
        const s = sphere.create({ radius: 1.0 });
        const ss = scaled.create({ shape: s, scale: vec3.fromValues(2, 3, 4) });
        // Scaling doesn't change center of mass of a symmetric shape
        expect(ss.centerOfMass[0]).toBe(0);
        expect(ss.centerOfMass[1]).toBe(0);
        expect(ss.centerOfMass[2]).toBe(0);
    });

    test('scales the inner shape\'s center of mass', () => {
        const inner = offsetCenterOfMass.create({ shape: sphere.create({ radius: 1 }), offset: vec3.fromValues(1, 2, 3) });
        const ss = scaled.create({ shape: inner, scale: vec3.fromValues(2, -3, 4) });
        expect(ss.centerOfMass[0]).toBe(2);
        expect(ss.centerOfMass[1]).toBe(-6);
        expect(ss.centerOfMass[2]).toBe(12);
    });
});
