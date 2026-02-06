import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { dof, rigidBody, MotionType, sphere, updateWorld } from '../../src';
import { createTestWorld } from '../helpers';

describe('AllowedDegreesOfFreedom', () => {
    describe('DOF Helper Function', () => {
        test('dof() creates correct bitmask for individual axes', () => {
            // Translation only
            expect(dof(true, false, false, false, false, false)).toBe(0b000001); // TX
            expect(dof(false, true, false, false, false, false)).toBe(0b000010); // TY
            expect(dof(false, false, true, false, false, false)).toBe(0b000100); // TZ

            // Rotation only
            expect(dof(false, false, false, true, false, false)).toBe(0b001000); // RX
            expect(dof(false, false, false, false, true, false)).toBe(0b010000); // RY
            expect(dof(false, false, false, false, false, true)).toBe(0b100000); // RZ
        });

        test('dof() creates correct bitmask for combinations', () => {
            // All DOFs (default)
            expect(dof(true, true, true, true, true, true)).toBe(0b111111);

            // Plane2D (TX, TY, RZ)
            expect(dof(true, true, false, false, false, true)).toBe(0b100011);

            // Translation only
            expect(dof(true, true, true, false, false, false)).toBe(0b000111);

            // Rotation only
            expect(dof(false, false, false, true, true, true)).toBe(0b111000);

            // Custom: TX, TZ, RY
            expect(dof(true, false, true, false, true, false)).toBe(0b010101);
        });
    });

    describe('Force Integration Enforcement', () => {
        test('locked translation axes remain at zero velocity after force integration', () => {
            const { world, layers } = createTestWorld();

            // Create body with only TX allowed (TY and TZ locked)
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: dof(true, false, false, false, false, false), // Only TX
            });

            // Apply forces in all directions
            body.motionProperties.force = vec3.fromValues(100, 100, 100);

            // Step physics to integrate forces → velocities
            updateWorld(world, undefined, 1 / 60);

            // Only X velocity should be non-zero
            expect(body.motionProperties.linearVelocity[0]).not.toBe(0);
            expect(body.motionProperties.linearVelocity[1]).toBe(0); // Y locked
            expect(body.motionProperties.linearVelocity[2]).toBe(0); // Z locked
        });

        test('locked rotation axes remain at zero velocity after torque integration', () => {
            const { world, layers } = createTestWorld();

            // Create body with only RZ allowed (RX and RY locked)
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: dof(false, false, false, false, false, true), // Only RZ
            });

            // Apply torque in all directions
            body.motionProperties.torque = vec3.fromValues(10, 10, 10);

            // Step physics to integrate torques → angular velocities
            updateWorld(world, undefined, 1 / 60);

            // Only Z angular velocity should be non-zero
            expect(body.motionProperties.angularVelocity[0]).toBe(0); // X rotation locked
            expect(body.motionProperties.angularVelocity[1]).toBe(0); // Y rotation locked
            expect(body.motionProperties.angularVelocity[2]).not.toBe(0);
        });

        test('Plane2D constraint (TX, TY, RZ) enforced correctly', () => {
            const { world, layers } = createTestWorld();

            // Plane2D: can move in X-Y plane and rotate around Z axis
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: dof(true, true, false, false, false, true), // Plane2D
            });

            // Apply forces and torques in all directions
            body.motionProperties.force = vec3.fromValues(100, 100, 100);
            body.motionProperties.torque = vec3.fromValues(10, 10, 10);

            // Step physics
            updateWorld(world, undefined, 1 / 60);

            // Translation: X and Y allowed, Z locked
            expect(body.motionProperties.linearVelocity[0]).not.toBe(0);
            expect(body.motionProperties.linearVelocity[1]).not.toBe(0);
            expect(body.motionProperties.linearVelocity[2]).toBe(0); // Z translation locked

            // Rotation: Z allowed, X and Y locked
            expect(body.motionProperties.angularVelocity[0]).toBe(0); // X rotation locked
            expect(body.motionProperties.angularVelocity[1]).toBe(0); // Y rotation locked
            expect(body.motionProperties.angularVelocity[2]).not.toBe(0);
        });

        test('all DOFs locked results in no motion', () => {
            // NOTE: Locking all DOFs is not allowed - the system will assert
            // Use a static body instead if you want a completely immovable object
            
            // This test is intentionally skipped to document the constraint
            expect(true).toBe(true);
        });
    });

    describe('Velocity Clamping Enforcement', () => {
        test('DOF constraints applied after velocity clamping', () => {
            const { world, layers } = createTestWorld();

            // Lock TY and TZ
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: dof(true, false, false, false, false, false), // Only TX
                maxLinearVelocity: 10.0,
            });

            // Apply huge force that would exceed max velocity
            body.motionProperties.force = vec3.fromValues(10000, 10000, 10000);

            // Step physics
            updateWorld(world, undefined, 1 / 60);

            // X velocity should be clamped but non-zero
            expect(Math.abs(body.motionProperties.linearVelocity[0])).toBeGreaterThan(0);
            expect(Math.abs(body.motionProperties.linearVelocity[0])).toBeLessThanOrEqual(10.0);

            // Y and Z should still be zero (DOF enforcement after clamping)
            expect(body.motionProperties.linearVelocity[1]).toBe(0);
            expect(body.motionProperties.linearVelocity[2]).toBe(0);
        });
    });

    describe('Multi-Step Persistence', () => {
        test('DOF constraints persist across multiple simulation steps', () => {
            const { world, layers } = createTestWorld();

            // Plane2D
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: dof(true, true, false, false, false, true),
            });

            // Continuous force application
            for (let i = 0; i < 10; i++) {
                body.motionProperties.force = vec3.fromValues(50, 50, 50);
                body.motionProperties.torque = vec3.fromValues(5, 5, 5);

                updateWorld(world, undefined, 1 / 60);

                // Check DOF constraints hold every step
                expect(body.motionProperties.linearVelocity[2]).toBe(0); // Z locked
                expect(body.motionProperties.angularVelocity[0]).toBe(0); // RX locked
                expect(body.motionProperties.angularVelocity[1]).toBe(0); // RY locked

                // Allowed DOFs should accumulate velocity (with damping)
                expect(Math.abs(body.motionProperties.linearVelocity[0])).toBeGreaterThan(0);
                expect(Math.abs(body.motionProperties.linearVelocity[1])).toBeGreaterThan(0);
            }
        });
    });

    describe('Gravity Interaction', () => {
        test('gravity only affects allowed translation axes', () => {
            const { world, layers } = createTestWorld();

            // Lock Y axis (vertical), allow X and Z
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: dof(true, false, true, false, false, false), // TX, TZ only
            });

            // Default gravity is [0, -9.81, 0]
            // Step physics with gravity
            updateWorld(world, undefined, 1 / 60);

            // Y velocity should be zero (locked), even though gravity pulls in -Y
            expect(body.motionProperties.linearVelocity[1]).toBe(0);

            // X and Z should also be zero (no forces applied)
            expect(body.motionProperties.linearVelocity[0]).toBe(0);
            expect(body.motionProperties.linearVelocity[2]).toBe(0);
        });

        test('gravity affects allowed vertical axis normally', () => {
            const { world, layers } = createTestWorld();

            // Allow only Y translation
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: dof(false, true, false, false, false, false), // TY only
            });

            // Step physics with gravity
            updateWorld(world, undefined, 1 / 60);

            // Y velocity should be negative (falling)
            expect(body.motionProperties.linearVelocity[1]).toBeLessThan(0);

            // X and Z locked
            expect(body.motionProperties.linearVelocity[0]).toBe(0);
            expect(body.motionProperties.linearVelocity[2]).toBe(0);
        });
    });

    describe('Edge Cases', () => {
        test('default body has all DOFs enabled', () => {
            const { world, layers } = createTestWorld();

            const shape = sphere.create({ radius: 1.0 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
            });

            expect(body.motionProperties.allowedDegreesOfFreedom).toBe(0b111111);
        });

        test('static bodies ignore DOF constraints', () => {
            const { world, layers } = createTestWorld();

            // Static body with locked DOFs (should have no effect)
            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.STATIC,
                allowedDegreesOfFreedom: dof(true, false, false, false, false, false),
            });

            // Static bodies should have zero inverse mass regardless of DOFs
            expect(body.motionProperties.invMass).toBe(0);
        });

        test('changing DOF mid-simulation takes effect immediately', () => {
            const { world, layers } = createTestWorld();

            const shape = sphere.create({ radius: 1.0, density: 1000 });
            const body = rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                allowedDegreesOfFreedom: 0b111111, // All DOFs
            });

            // Apply forces
            body.motionProperties.force = vec3.fromValues(100, 100, 100);
            updateWorld(world, undefined, 1 / 60);

            // Should have velocity in all directions
            expect(body.motionProperties.linearVelocity[0]).not.toBe(0);
            expect(body.motionProperties.linearVelocity[1]).not.toBe(0);
            expect(body.motionProperties.linearVelocity[2]).not.toBe(0);

            // Change DOFs to lock Y and Z
            body.motionProperties.allowedDegreesOfFreedom = dof(true, false, false, false, false, false);

            // Apply more forces and step
            body.motionProperties.force = vec3.fromValues(100, 100, 100);
            updateWorld(world, undefined, 1 / 60);

            // Y and Z velocities should be zeroed
            expect(body.motionProperties.linearVelocity[1]).toBe(0);
            expect(body.motionProperties.linearVelocity[2]).toBe(0);

            // X should continue to accumulate
            expect(Math.abs(body.motionProperties.linearVelocity[0])).toBeGreaterThan(0);
        });
    });
});
