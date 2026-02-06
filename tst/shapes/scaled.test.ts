import { describe, expect, test } from 'vitest';
import { vec3 } from 'mathcat';
import { ShapeType, scaled, sphere } from '../../src';

describe('Scaled shape AABB caching', () => {
    test('scaled shape should have correct AABB', () => {
        const s = sphere.create({ radius: 1.0 });
        const ss = scaled.create({ shape: s, scale: vec3.fromValues(2, 3, 4) });
        expect(ss.aabb).toBeDefined();
        // Scaled sphere: [-2, -3, -4] to [2, 3, 4]
        expect(ss.aabb[0][0]).toBe(-2);
        expect(ss.aabb[0][1]).toBe(-3);
        expect(ss.aabb[0][2]).toBe(-4);
        expect(ss.aabb[1][0]).toBe(2);
        expect(ss.aabb[1][1]).toBe(3);
        expect(ss.aabb[1][2]).toBe(4);
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
});
