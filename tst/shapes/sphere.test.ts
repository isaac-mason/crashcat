import { describe, expect, test } from 'vitest';
import { computeMassProperties, sphere, ShapeType } from '../../src';
import * as massProperties from '../../src/body/mass-properties';

describe('Sphere AABB caching', () => {
    test('sphere should have correct AABB', () => {
        const s = sphere.create({ radius: 2.0 });
        expect(s.aabb).toBeDefined();
        // AABB should be [-r, -r, -r] to [r, r, r]
        expect(s.aabb[0][0]).toBe(-2.0);
        expect(s.aabb[0][1]).toBe(-2.0);
        expect(s.aabb[0][2]).toBe(-2.0);
        expect(s.aabb[1][0]).toBe(2.0);
        expect(s.aabb[1][1]).toBe(2.0);
        expect(s.aabb[1][2]).toBe(2.0);
    });
});

describe('Sphere creation and mass properties', () => {
    test('should create a sphere with default density', () => {
        const s = sphere.create({ radius: 1.0 });
        expect(s.type).toBe(ShapeType.SPHERE);
        expect(s.radius).toBe(1.0);
        expect(s.density).toBe(1000); // DEFAULT_SHAPE_DENSITY
    });

    test('should create a sphere with custom density', () => {
        const s = sphere.create({ radius: 1.0, density: 500 });
        expect(s.radius).toBe(1.0);
        expect(s.density).toBe(500);
    });

    test('should compute sphere mass properties', () => {
        const s = sphere.create({ radius: 1.0 });
        const props = massProperties.create();
        computeMassProperties(props, s);

        // For a unit sphere with density 1000: m = (4/3)π*density
        const expected = (4.0 / 3.0) * Math.PI * 1000;
        expect(props.mass).toBeCloseTo(expected, 1);
    });
});

describe('Sphere center of mass', () => {
    test('sphere should have center of mass at origin', () => {
        const s = sphere.create({ radius: 1.0 });
        expect(s.centerOfMass[0]).toBe(0);
        expect(s.centerOfMass[1]).toBe(0);
        expect(s.centerOfMass[2]).toBe(0);
    });
});
