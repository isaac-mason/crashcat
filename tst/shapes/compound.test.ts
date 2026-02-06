import { describe, expect, test } from 'vitest';
import { quat, vec3 } from 'mathcat';
import { ShapeType, box, compound, sphere } from '../../src';

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
        expect(cs.aabb[0][0]).toBe(-2);
        expect(cs.aabb[0][1]).toBe(-2);
        expect(cs.aabb[0][2]).toBe(-2);
        expect(cs.aabb[1][0]).toBe(2);
        expect(cs.aabb[1][1]).toBe(2);
        expect(cs.aabb[1][2]).toBe(2);
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
