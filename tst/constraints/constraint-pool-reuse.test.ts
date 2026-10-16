import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    hingeConstraint,
    MotionType,
    MotorState,
    rigidBody,
    SixDOFAxis,
    SpringMode,
    type SpringSettings,
    sixDOFConstraint,
    sliderConstraint,
    sphere,
} from '../../src';
import { createTestWorld } from '../helpers';

function createTwoBodies() {
    const { world, layers } = createTestWorld();
    const bodyA = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: layers.OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 0, 0),
    });
    const bodyB = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: layers.OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(2, 0, 0),
    });
    return { world, bodyA, bodyB };
}

const softSpring = (): SpringSettings => ({ mode: SpringMode.STIFFNESS_AND_DAMPING, frequencyOrStiffness: 500, damping: 3 });

function expectDefaultSpring(spring: SpringSettings) {
    expect(spring.mode).toBe(SpringMode.FREQUENCY_AND_DAMPING);
    expect(spring.frequencyOrStiffness).toBe(0);
    expect(spring.damping).toBe(0);
}

describe('a reused pooled constraint keeps nothing of the previous one', () => {
    test('hinge limits spring', () => {
        const { world, bodyA, bodyB } = createTwoBodies();
        const settings = {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(1, 0, 0),
            pointB: vec3.fromValues(1, 0, 0),
            hingeAxisA: vec3.fromValues(0, 1, 0),
            hingeAxisB: vec3.fromValues(0, 1, 0),
            normalAxisA: vec3.fromValues(1, 0, 0),
            normalAxisB: vec3.fromValues(1, 0, 0),
        };

        const first = hingeConstraint.create(world, { ...settings, limitsSpringSettings: softSpring() });
        hingeConstraint.remove(world, first);
        const second = hingeConstraint.create(world, settings);

        expect(second).toBe(first);
        expectDefaultSpring(second.limitsSpringSettings);
    });

    test('slider limits spring', () => {
        const { world, bodyA, bodyB } = createTwoBodies();
        const settings = {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(1, 0, 0),
            pointB: vec3.fromValues(1, 0, 0),
            sliderAxisA: vec3.fromValues(1, 0, 0),
            sliderAxisB: vec3.fromValues(1, 0, 0),
            normalAxisA: vec3.fromValues(0, 1, 0),
            normalAxisB: vec3.fromValues(0, 1, 0),
        };

        const first = sliderConstraint.create(world, { ...settings, limitsSpringSettings: softSpring() });
        sliderConstraint.remove(world, first);
        const second = sliderConstraint.create(world, settings);

        expect(second).toBe(first);
        expectDefaultSpring(second.limitsSpringSettings);
    });

    test('six dof limits, friction, limit springs, motor force limits and motor states', () => {
        const { world, bodyA, bodyB } = createTwoBodies();
        const settings = {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            position1: vec3.fromValues(1, 0, 0),
            position2: vec3.fromValues(1, 0, 0),
            axisX1: vec3.fromValues(1, 0, 0),
            axisY1: vec3.fromValues(0, 1, 0),
            axisX2: vec3.fromValues(1, 0, 0),
            axisY2: vec3.fromValues(0, 1, 0),
        };

        const first = sixDOFConstraint.create(world, {
            ...settings,
            limitMin: [-1, -1, -1, -0.5, -0.5, -0.5],
            limitMax: [1, 1, 1, 0.5, 0.5, 0.5],
            maxFriction: [2, 2, 2, 2, 2, 2],
            limitsSpringSettings: [softSpring(), softSpring(), softSpring()],
            motorMinForceLimit: [-10, -10, -10, -10, -10, -10],
            motorMaxForceLimit: [10, 10, 10, 10, 10, 10],
        });
        sixDOFConstraint.setMotorState(first, SixDOFAxis.TRANSLATION_Y, MotorState.VELOCITY);
        sixDOFConstraint.remove(world, first);
        const second = sixDOFConstraint.create(world, settings);

        expect(second).toBe(first);
        expect(second.limitMin).toEqual([-Infinity, -Infinity, -Infinity, -Math.PI, -Math.PI, -Math.PI]);
        expect(second.limitMax).toEqual([Infinity, Infinity, Infinity, Math.PI, Math.PI, Math.PI]);
        expect(second.maxFriction).toEqual([0, 0, 0, 0, 0, 0]);
        for (const spring of second.limitsSpringSettings) expectDefaultSpring(spring);
        expect(second.motorMinForceLimit).toEqual([-Infinity, -Infinity, -Infinity, -Infinity, -Infinity, -Infinity]);
        expect(second.motorMaxForceLimit).toEqual([Infinity, Infinity, Infinity, Infinity, Infinity, Infinity]);
        expect(second.motorState.every((state) => state === MotorState.OFF)).toBe(true);
        expect(second.translationMotorActive).toBe(false);
    });
});
