import type { Vec3 } from 'math';
import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    MotionType,
    MotorState,
    motorSettings,
    type RigidBody,
    rigidBody,
    SpringMode,
    sliderConstraint,
    sphere,
} from '../../src';
import type { World } from '../../src/world';
import { createTestWorld } from '../helpers';

const DT = 1 / 60;

function createBody(world: World, layers: ReturnType<typeof createTestWorld>['layers'], motionType: MotionType, position: Vec3) {
    return rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: motionType === MotionType.STATIC ? layers.OBJECT_LAYER_NOT_MOVING : layers.OBJECT_LAYER_MOVING,
        motionType,
        position,
        linearDamping: 0,
        angularDamping: 0,
    });
}

function createSlider(
    world: World,
    bodyA: RigidBody,
    bodyB: RigidBody,
    settings: Partial<sliderConstraint.SliderConstraintSettings> = {},
) {
    return sliderConstraint.create(world, {
        bodyIdA: bodyA.id,
        bodyIdB: bodyB.id,
        sliderAxisA: [1, 0, 0],
        sliderAxisB: [1, 0, 0],
        normalAxisA: [0, 1, 0],
        normalAxisB: [0, 1, 0],
        ...settings,
    });
}

function createAutoDetected(motionTypeA: MotionType, motionTypeB: MotionType) {
    const { world, layers } = createTestWorld();
    const bodyA = createBody(world, layers, motionTypeA, [0, 0, 0]);
    const bodyB = createBody(world, layers, motionTypeB, [4, 0, 0]);
    return createSlider(world, bodyA, bodyB, { autoDetectPoint: true });
}

describe('sliderConstraint auto-detected anchor', () => {
    test('two static bodies anchor on body b, as body a is static', () => {
        const constraint = createAutoDetected(MotionType.STATIC, MotionType.STATIC);
        expect(constraint.localSpacePositionA).toEqual([4, 0, 0]);
        expect(constraint.localSpacePositionB).toEqual([0, 0, 0]);
    });

    test('a static body a and a kinematic body b anchor on body b', () => {
        const constraint = createAutoDetected(MotionType.STATIC, MotionType.KINEMATIC);
        expect(constraint.localSpacePositionA).toEqual([4, 0, 0]);
        expect(constraint.localSpacePositionB).toEqual([0, 0, 0]);
    });

    test('a kinematic body a and a static body b anchor on body a', () => {
        const constraint = createAutoDetected(MotionType.KINEMATIC, MotionType.STATIC);
        expect(constraint.localSpacePositionA).toEqual([0, 0, 0]);
        expect(constraint.localSpacePositionB).toEqual([-4, 0, 0]);
    });

    test('a body made static anchors on the other body, whatever inverse mass it kept', () => {
        const { world, layers } = createTestWorld();
        const bodyA = createBody(world, layers, MotionType.DYNAMIC, [0, 0, 0]);
        const bodyB = createBody(world, layers, MotionType.DYNAMIC, [4, 0, 0]);
        rigidBody.setMotionType(world, bodyA, MotionType.STATIC, false);

        const constraint = createSlider(world, bodyA, bodyB, { autoDetectPoint: true });
        expect(constraint.localSpacePositionA).toEqual([4, 0, 0]);
        expect(constraint.localSpacePositionB).toEqual([0, 0, 0]);
    });

    test('two equal dynamic bodies anchor half way', () => {
        const constraint = createAutoDetected(MotionType.DYNAMIC, MotionType.DYNAMIC);
        expect(constraint.localSpacePositionA[0]).toBeCloseTo(2, 6);
        expect(constraint.localSpacePositionB[0]).toBeCloseTo(-2, 6);
    });
});

describe('sliderConstraint limits', () => {
    test('setLimits asserts min <= 0 <= max instead of clamping', () => {
        const { world, layers } = createTestWorld();
        const constraint = createSlider(
            world,
            createBody(world, layers, MotionType.STATIC, [0, 0, 0]),
            createBody(world, layers, MotionType.DYNAMIC, [0, 0, 0]),
        );
        expect(() => sliderConstraint.setLimits(constraint, 0.5, 1)).toThrow();
        expect(() => sliderConstraint.setLimits(constraint, -1, -0.5)).toThrow();

        sliderConstraint.setLimits(constraint, -2, 3);
        expect(constraint.limitsMin).toBe(-2);
        expect(constraint.limitsMax).toBe(3);
        expect(constraint.hasLimits).toBe(true);
    });
});

describe('sliderConstraint position and velocity motor', () => {
    function createMotorSlider(settings: Partial<sliderConstraint.SliderConstraintSettings> = {}) {
        const { world, layers } = createTestWorld();
        const base = createBody(world, layers, MotionType.STATIC, [0, 0, 0]);
        const block = createBody(world, layers, MotionType.DYNAMIC, [0, 0, 0]);
        const constraint = createSlider(world, base, block, settings);
        return { world, block, constraint };
    }

    function solveVelocities(world: World, constraint: sliderConstraint.SliderConstraint): void {
        sliderConstraint.def.setupVelocity(constraint, world.bodies, DT);
        sliderConstraint.def.warmStartVelocity(constraint, world.bodies, 1);
        for (let i = 0; i < 10; i++) sliderConstraint.def.solveVelocity(constraint, world.bodies, DT);
    }

    test('drives the velocity with the spring at the target position', () => {
        const { world, block, constraint } = createMotorSlider();
        sliderConstraint.setMotorState(constraint, MotorState.POSITION_AND_VELOCITY);
        sliderConstraint.setTargetVelocity(constraint, 2);
        sliderConstraint.setTargetPosition(constraint, 0);

        solveVelocities(world, constraint);
        expect(block.motionProperties.linearVelocity[0]).toBeGreaterThan(0);
    });

    test('a damping only spring is a soft velocity drive: v = v_target * dt c / m / (dt c / m + 1)', () => {
        const mass = 1 / createMotorSlider().block.motionProperties.invMass;
        const settings = motorSettings.create();
        settings.springSettings.mode = SpringMode.STIFFNESS_AND_DAMPING;
        settings.springSettings.frequencyOrStiffness = 0;
        // dt c / m = 1 halves the target
        settings.springSettings.damping = mass / DT;

        const { world, block, constraint } = createMotorSlider({ motorSettings: settings });
        sliderConstraint.setMotorState(constraint, MotorState.POSITION_AND_VELOCITY);
        sliderConstraint.setTargetVelocity(constraint, 2);

        solveVelocities(world, constraint);
        expect(block.motionProperties.linearVelocity[0]).toBeCloseTo(1, 6);
    });

    test('a damping only spring cannot drive a position motor, which needs stiffness', () => {
        const settings = motorSettings.create();
        settings.springSettings.mode = SpringMode.STIFFNESS_AND_DAMPING;
        settings.springSettings.frequencyOrStiffness = 0;
        settings.springSettings.damping = 100;

        const { world, block, constraint } = createMotorSlider({ motorSettings: settings });
        sliderConstraint.setMotorState(constraint, MotorState.POSITION);
        sliderConstraint.setTargetPosition(constraint, 1);

        solveVelocities(world, constraint);
        expect(block.motionProperties.linearVelocity[0]).toBe(0);
    });
});

describe('sliderConstraint initial orientation', () => {
    test('axes a hair apart are not taken as identical', () => {
        const angle = 9e-7;
        const { world, layers } = createTestWorld();
        const constraint = createSlider(
            world,
            createBody(world, layers, MotionType.STATIC, [0, 0, 0]),
            createBody(world, layers, MotionType.DYNAMIC, [0, 0, 0]),
            {
                normalAxisB: vec3.fromValues(0, Math.cos(angle), Math.sin(angle)),
            },
        );

        // r0^-1 = c2 c1^-1 turns by the hair about the slider axis
        expect(constraint.invInitialOrientation[0]).toBeCloseTo(Math.sin(angle / 2), 12);
    });
});
