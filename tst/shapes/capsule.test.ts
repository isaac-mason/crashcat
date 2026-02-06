import { describe, expect, test } from 'vitest';
import * as massProperties from '../../src/body/mass-properties';
import { capsule, computeMassProperties } from '../../src';

describe('Capsule Shape', () => {
    describe('Factory Function', () => {
        test('should create a capsule with valid parameters', () => {
            const c = capsule.create({ halfHeightOfCylinder: 1.0, radius: 0.5 });
            expect(c.halfHeightOfCylinder).toBe(1.0);
            expect(c.radius).toBe(0.5);
            expect(c.density).toBe(1000);
        });

        test('should create a capsule with custom density', () => {
            const c = capsule.create({ halfHeightOfCylinder: 2.0, radius: 1.0, density: 500 });
            expect(c.density).toBe(500);
        });

        test('should throw when radius is zero', () => {
            expect(() => capsule.create({ halfHeightOfCylinder: 1.0, radius: 0 })).toThrow('Invalid radius');
        });

        test('should throw when radius is negative', () => {
            expect(() => capsule.create({ halfHeightOfCylinder: 1.0, radius: -1.0 })).toThrow('Invalid radius');
        });

        test('should throw when halfHeightOfCylinder is zero', () => {
            expect(() => capsule.create({ halfHeightOfCylinder: 0, radius: 1.0 })).toThrow('Invalid height');
        });

        test('should throw when halfHeightOfCylinder is negative', () => {
            expect(() => capsule.create({ halfHeightOfCylinder: -1.0, radius: 1.0 })).toThrow('Invalid height');
        });
    });

    describe('Geometry', () => {
        test('should compute correct volume for unit capsule', () => {
            const c = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1.0 });
            // V = π*r²*2h + (4/3)*π*r³
            // V = π*1²*2*1 + (4/3)*π*1³ = 2π + (4/3)π ≈ 10.472
            const expectedVolume = Math.PI * 2 + (4 / 3) * Math.PI;
            expect(c.volume).toBeCloseTo(expectedVolume);
        });

        test('should compute correct volume for capsule with different dimensions', () => {
            const c = capsule.create({ halfHeightOfCylinder: 2.0, radius: 0.5 });
            // V = π*0.5²*2*2 + (4/3)*π*0.5³
            const cylinderVolume = Math.PI * 0.25 * 4;
            const sphereVolume = (4 / 3) * Math.PI * 0.125;
            const expectedVolume = cylinderVolume + sphereVolume;
            expect(c.volume).toBeCloseTo(expectedVolume);
        });

        test('should have correct AABB bounds', () => {
            const c = capsule.create({ halfHeightOfCylinder: 2.0, radius: 0.5 });
            // AABB should be [-r, -(h+r), -r] to [r, h+r, r]
            // Box3 is [Vec3, Vec3] where [0] = min, [1] = max
            expect(c.aabb[0][0]).toBeCloseTo(-0.5); // min X
            expect(c.aabb[0][1]).toBeCloseTo(-2.5); // min Y
            expect(c.aabb[0][2]).toBeCloseTo(-0.5); // min Z
            expect(c.aabb[1][0]).toBeCloseTo(0.5); // max X
            expect(c.aabb[1][1]).toBeCloseTo(2.5); // max Y
            expect(c.aabb[1][2]).toBeCloseTo(0.5); // max Z
        });

        test('should have center of mass at origin', () => {
            const c = capsule.create({ halfHeightOfCylinder: 1.5, radius: 0.75 });
            expect(c.centerOfMass[0]).toBeCloseTo(0);
            expect(c.centerOfMass[1]).toBeCloseTo(0);
            expect(c.centerOfMass[2]).toBeCloseTo(0);
        });
    });

    describe('Mass Properties', () => {
        test('should compute mass from density and volume', () => {
            const c = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1.0, density: 1000 });
            const massProps = massProperties.create();
            computeMassProperties(massProps, c);

            const expectedMass = 1000 * c.volume;
            expect(massProps.mass).toBeCloseTo(expectedMass);
        });

        test('should compute inertia tensor for unit capsule', () => {
            const c = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1.0, density: 1000 });
            const massProps = massProperties.create();
            computeMassProperties(massProps, c);

            // Inertia tensor should be diagonal matrix (capsule aligned along Y-axis)
            // I_xx and I_zz should be equal (symmetry around Y-axis)
            // I_yy should be smaller (rotation around Y-axis)

            // Get diagonal elements from column-major Mat4
            const Ixx = massProps.inertia[0]; // [0,0]
            const Iyy = massProps.inertia[5]; // [1,1]
            const Izz = massProps.inertia[10]; // [2,2]

            expect(Ixx).toBeGreaterThan(0);
            expect(Iyy).toBeGreaterThan(0);
            expect(Izz).toBeGreaterThan(0);

            // Symmetry: Ixx should equal Izz
            expect(Ixx).toBeCloseTo(Izz);

            // Iyy (around longitudinal axis) should be less than Ixx/Izz
            expect(Iyy).toBeLessThan(Ixx);
        });

        test('should have zero off-diagonal inertia elements', () => {
            const c = capsule.create({ halfHeightOfCylinder: 2.0, radius: 0.5, density: 500 });
            const massProps = massProperties.create();
            computeMassProperties(massProps, c);

            // Off-diagonal elements should be zero (column-major indexing)
            expect(massProps.inertia[1]).toBeCloseTo(0); // [1,0]
            expect(massProps.inertia[2]).toBeCloseTo(0); // [2,0]
            expect(massProps.inertia[4]).toBeCloseTo(0); // [0,1]
            expect(massProps.inertia[6]).toBeCloseTo(0); // [2,1]
            expect(massProps.inertia[8]).toBeCloseTo(0); // [0,2]
            expect(massProps.inertia[9]).toBeCloseTo(0); // [1,2]
        });

        test('should scale inertia with mass for different densities', () => {
            const c1 = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1.0, density: 1000 });
            const c2 = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1.0, density: 2000 });

            const massProps1 = massProperties.create();
            const massProps2 = massProperties.create();

            computeMassProperties(massProps1, c1);
            computeMassProperties(massProps2, c2);

            // Mass should double
            expect(massProps2.mass).toBeCloseTo(massProps1.mass * 2);

            // Inertia should also double
            expect(massProps2.inertia[0]).toBeCloseTo(massProps1.inertia[0] * 2);
            expect(massProps2.inertia[5]).toBeCloseTo(massProps1.inertia[5] * 2);
            expect(massProps2.inertia[10]).toBeCloseTo(massProps1.inertia[10] * 2);
        });

        test('should compute correct inertia for very long capsule', () => {
            // Very long capsule should have much higher Ixx and Izz compared to Iyy
            const c = capsule.create({ halfHeightOfCylinder: 10.0, radius: 0.5, density: 1000 });
            const massProps = massProperties.create();
            computeMassProperties(massProps, c);

            const Ixx = massProps.inertia[0];
            const Iyy = massProps.inertia[5];
            const Izz = massProps.inertia[10];

            expect(Ixx).toBeCloseTo(Izz);
            expect(Iyy).toBeLessThan(Ixx * 0.1); // Iyy should be much smaller
        });

        test('should compute correct inertia for wide capsule (large radius, small height)', () => {
            // Wide capsule: closer to sphere
            const c = capsule.create({ halfHeightOfCylinder: 0.1, radius: 2.0, density: 1000 });
            const massProps = massProperties.create();
            computeMassProperties(massProps, c);

            const Ixx = massProps.inertia[0];
            const Iyy = massProps.inertia[5];
            const Izz = massProps.inertia[10];

            expect(Ixx).toBeCloseTo(Izz);
            // For near-sphere, all inertias should be close
            expect(Iyy / Ixx).toBeGreaterThan(0.5);
        });
    });
});
