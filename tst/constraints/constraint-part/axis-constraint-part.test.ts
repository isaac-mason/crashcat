import { mat4, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, MotionType, rigidBody } from '../../../src';
import { getWorldInverseInertia } from '../../../src/body/motion-properties';
import * as axisConstraintPart from '../../../src/constraints/constraint-part/axis-constraint-part';
import { createTestWorld } from '../../helpers';

describe('axisConstraintPart', () => {
    test('a spinning kinematic body hit off-centre pushes a resting dynamic body with its surface velocity', () => {
        const { world, layers } = createTestWorld();
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

        // kinematic a spins about z; the contact is at its top right corner, r1 = (1, 1, 0)
        const kinematic = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.KINEMATIC,
            position: vec3.fromValues(0, 0, 0),
        });
        const spin = 2;
        vec3.set(kinematic.motionProperties.angularVelocity, 0, 0, spin);

        // dynamic b rests on that corner, hit straight below its centre of mass, r2 = (0, -1, 0)
        const dynamic = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(1, 2, 0),
        });

        const r1 = vec3.fromValues(1, 1, 0);
        const r2 = vec3.fromValues(0, -1, 0);
        const axis = vec3.fromValues(0, 1, 0);

        const invMassB = dynamic.motionProperties.invMass;
        const invInertiaA = mat4.create();
        const invInertiaB = getWorldInverseInertia(mat4.create(), dynamic.motionProperties, dynamic.quaternion, -1);

        const part = axisConstraintPart.create();
        axisConstraintPart.calculateConstraintProperties(
            part,
            kinematic,
            dynamic,
            0,
            invMassB,
            invInertiaA,
            invInertiaB,
            r1,
            r2,
            axis,
            0,
        );

        // the corner's velocity is w x r1 = (0, 0, 2) x (1, 1, 0) = (-2, 2, 0): it closes on b at 2 along the axis.
        // r2 x axis = 0, so the effective mass is b's mass and the impulse brings b's normal speed up to 2
        const closingSpeed = spin;
        const expectedLambda = closingSpeed / invMassB;

        axisConstraintPart.solveVelocityConstraint(part, kinematic, dynamic, axis, 0, Number.POSITIVE_INFINITY);

        expect(part.totalLambda).toBeCloseTo(expectedLambda, 5);
        expect(dynamic.motionProperties.linearVelocity[0]).toBeCloseTo(0, 6);
        expect(dynamic.motionProperties.linearVelocity[1]).toBeCloseTo(closingSpeed, 5);
        expect(dynamic.motionProperties.linearVelocity[2]).toBeCloseTo(0, 6);
        expect(vec3.length(dynamic.motionProperties.angularVelocity)).toBeCloseTo(0, 6);

        // the kinematic body is untouched
        expect(kinematic.motionProperties.angularVelocity[2]).toBe(spin);
    });

    // jolt clamps as min(max(lambda, min), max), so inverted bounds (the ccd friction limit of a contact whose normal
    // lambda came out negative) give the max bound
    test('inverted lambda bounds clamp to the max bound, as jolt does', () => {
        const { world, layers } = createTestWorld();
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const ground = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });
        const dynamic = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 2, 0),
        });
        // b falls onto a at 3, so the unclamped lambda is positive (3 x b's mass)
        vec3.set(dynamic.motionProperties.linearVelocity, 0, -3, 0);

        const invMassB = dynamic.motionProperties.invMass;
        const invInertiaB = getWorldInverseInertia(mat4.create(), dynamic.motionProperties, dynamic.quaternion, -1);
        const axis = vec3.fromValues(0, 1, 0);

        const part = axisConstraintPart.create();
        axisConstraintPart.calculateConstraintProperties(
            part,
            ground,
            dynamic,
            0,
            invMassB,
            mat4.create(),
            invInertiaB,
            vec3.fromValues(0, 1, 0),
            vec3.fromValues(0, -1, 0),
            axis,
            0,
        );

        axisConstraintPart.solveVelocityConstraint(part, ground, dynamic, axis, 1, -1);

        expect(part.totalLambda).toBe(-1);
        expect(dynamic.motionProperties.linearVelocity[1]).toBeCloseTo(-3 - invMassB, 6);
    });
});
