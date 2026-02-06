import { describe, expect, test } from 'vitest';
import { vec3 } from 'mathcat';
import { box, computeMassProperties, ShapeType } from '../../src';
import * as massProperties from '../../src/body/mass-properties';

describe('Box AABB caching', () => {
    test('box should have correct AABB', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        expect(b.aabb).toBeDefined();
        // AABB should be [-he[i], -he[i], -he[i]] to [he[i], he[i], he[i]]
        expect(b.aabb[0][0]).toBe(-1);
        expect(b.aabb[0][1]).toBe(-2);
        expect(b.aabb[0][2]).toBe(-3);
        expect(b.aabb[1][0]).toBe(1);
        expect(b.aabb[1][1]).toBe(2);
        expect(b.aabb[1][2]).toBe(3);
    });
});

describe('Box creation and mass properties', () => {
    test('should create a box with default density', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        expect(b.type).toBe(ShapeType.BOX);
        expect(b.halfExtents[0]).toBe(1);
        expect(b.halfExtents[1]).toBe(2);
        expect(b.halfExtents[2]).toBe(3);
        expect(b.density).toBe(1000);
    });

    test('should compute box mass properties', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1), density: 1000 });
        const props = massProperties.create();
        computeMassProperties(props, b);

        // For a box with halfExtents [1,1,1]: volume = 8, mass = 8*density
        expect(props.mass).toBeCloseTo(8000, 0);
    });
});

describe('Box center of mass', () => {
    test('box should have center of mass at origin', () => {
        const b = box.create({ halfExtents: vec3.fromValues(1, 2, 3) });
        expect(b.centerOfMass[0]).toBe(0);
        expect(b.centerOfMass[1]).toBe(0);
        expect(b.centerOfMass[2]).toBe(0);
    });
});
