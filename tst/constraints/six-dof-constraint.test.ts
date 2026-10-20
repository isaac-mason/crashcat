import type { Quat, Vec3 } from 'math';
import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    MotionType,
    MotorState,
    motorSettings,
    type RigidBody,
    rigidBody,
    SixDOFAxis,
    SpringMode,
    SwingType,
    sixDOFConstraint,
    sphere,
} from '../../src';
import type { World } from '../../src/world';
import { createTestWorld } from '../helpers';

const DT = 1 / 60;
const FREE_MIN = [-Infinity, -Infinity, -Infinity, -Math.PI, -Math.PI, -Math.PI];
const FREE_MAX = [Infinity, Infinity, Infinity, Math.PI, Math.PI, Math.PI];
const FIXED = [0, 0, 0, 0, 0, 0];

function axisAngle(axis: Vec3, angle: number): Quat {
    const unit = vec3.normalize(vec3.create(), axis);
    return quat.setAxisAngle(quat.create(), unit, angle);
}

/** a static base at the origin and a dynamic ball, joined at the origin with constraint space along the world axes */
function createScene(settings: Partial<sixDOFConstraint.SixDOFConstraintSettings> = {}, ballPosition: Vec3 = [0, 0, 0]) {
    const { world, layers } = createTestWorld();
    const base = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
    });
    const ball = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: layers.OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: ballPosition,
        linearDamping: 0,
        angularDamping: 0,
    });
    const constraint = sixDOFConstraint.create(world, {
        bodyIdA: base.id,
        bodyIdB: ball.id,
        position1: [0, 0, 0],
        position2: [0, 0, 0],
        axisX1: [1, 0, 0],
        axisY1: [0, 1, 0],
        axisX2: [1, 0, 0],
        axisY2: [0, 1, 0],
        ...settings,
    });
    return { world, ball, constraint };
}

function solveVelocities(world: World, constraint: sixDOFConstraint.SixDOFConstraint): void {
    sixDOFConstraint.def.setupVelocity(constraint, world.bodies, DT);
    sixDOFConstraint.def.warmStartVelocity(constraint, world.bodies, 1);
    for (let i = 0; i < 10; i++) sixDOFConstraint.def.solveVelocity(constraint, world.bodies, DT);
}

/** the constraint point on the ball in world space */
function ballPoint(ball: RigidBody, constraint: sixDOFConstraint.SixDOFConstraint): Vec3 {
    const out = vec3.transformQuat(vec3.create(), constraint.localSpacePosition2, ball.quaternion);
    return vec3.add(out, out, ball.centerOfMassPosition);
}

describe('sixDOFConstraint velocity solve', () => {
    test('reports whether any part applied an impulse', () => {
        const fixed = createScene({ limitMin: FIXED, limitMax: FIXED });
        vec3.set(fixed.ball.motionProperties.linearVelocity, 1, 0, 0);
        sixDOFConstraint.def.setupVelocity(fixed.constraint, fixed.world.bodies, DT);
        expect(sixDOFConstraint.def.solveVelocity(fixed.constraint, fixed.world.bodies, DT)).toBe(true);
        // the velocity is now consistent, nothing more to apply
        expect(sixDOFConstraint.def.solveVelocity(fixed.constraint, fixed.world.bodies, DT)).toBe(false);

        const free = createScene();
        vec3.set(free.ball.motionProperties.linearVelocity, 1, 0, 0);
        sixDOFConstraint.def.setupVelocity(free.constraint, free.world.bodies, DT);
        expect(sixDOFConstraint.def.solveVelocity(free.constraint, free.world.bodies, DT)).toBe(false);
    });
});

describe('sixDOFConstraint fixed and free axis changes', () => {
    test('a change to the fixed or free sets clears the warm start, a change within them keeps it', () => {
        const { world, ball, constraint } = createScene({ limitMin: FIXED, limitMax: FIXED });
        vec3.set(ball.motionProperties.linearVelocity, 0, -1, 0);
        solveVelocities(world, constraint);
        expect(constraint.pointConstraintPart.totalLambda[1]).not.toBe(0);

        // still fixed, at another offset: the sets are unchanged
        sixDOFConstraint.setLimitedAxis(constraint, SixDOFAxis.TRANSLATION_Z, 0.5, 0.5);
        expect(constraint.pointConstraintPart.totalLambda[1]).not.toBe(0);

        // translation x becomes limited: the point part no longer applies
        sixDOFConstraint.setLimitedAxis(constraint, SixDOFAxis.TRANSLATION_X, -1, 1);
        expect(constraint.pointConstraintPart.totalLambda).toEqual([0, 0, 0]);
        expect(constraint.rotationConstraintPart.totalLambda).toEqual([0, 0, 0]);
    });

    test('a translation axis made fixed at runtime is fixed at 0, and the position solve pulls it there', () => {
        const { world, ball, constraint } = createScene();
        ball.centerOfMassPosition[0] = 2;
        sixDOFConstraint.makeFixedAxis(constraint, SixDOFAxis.TRANSLATION_X);
        expect(constraint.limitMin[SixDOFAxis.TRANSLATION_X]).toBe(0);
        expect(constraint.limitMax[SixDOFAxis.TRANSLATION_X]).toBe(0);

        for (let i = 0; i < 4; i++) sixDOFConstraint.def.solvePosition(constraint, world.bodies, DT, 1);
        expect(ball.centerOfMassPosition[0]).toBeCloseTo(0, 6);
    });

    test('a rotation axis made fixed or free at runtime reaches the swing twist part', () => {
        const limitMin = [...FREE_MIN];
        const limitMax = [...FREE_MAX];
        limitMin[SixDOFAxis.ROTATION_X] = -0.5;
        limitMax[SixDOFAxis.ROTATION_X] = 0.5;
        const { constraint } = createScene({ limitMin, limitMax });
        const part = constraint.swingTwistConstraintPart;
        expect(part.sinTwistHalfMaxAngle).toBeCloseTo(Math.sin(0.25), 12);

        // free: the twist limit is at its +-1 sentinel
        sixDOFConstraint.makeFreeAxis(constraint, SixDOFAxis.ROTATION_X);
        expect(constraint.limitMin[SixDOFAxis.ROTATION_X]).toBe(-Math.PI);
        expect(constraint.limitMax[SixDOFAxis.ROTATION_X]).toBe(Math.PI);
        expect(part.sinTwistHalfMinAngle).toBe(-1);
        expect(part.sinTwistHalfMaxAngle).toBe(1);

        // fixed: locked at 0
        sixDOFConstraint.makeFixedAxis(constraint, SixDOFAxis.ROTATION_X);
        expect(constraint.limitMin[SixDOFAxis.ROTATION_X]).toBe(0);
        expect(constraint.limitMax[SixDOFAxis.ROTATION_X]).toBe(0);
        expect(part.sinTwistHalfMinAngle).toBe(0);
        expect(part.sinTwistHalfMaxAngle).toBe(0);
    });
});

describe('sixDOFConstraint position solve', () => {
    test('the point part sees the rotations the rotation solve left', () => {
        // the ball's center is a unit from the joint; turn it 0.3 about z around the joint: no point error, a rotation error
        const { world, ball, constraint } = createScene({ limitMin: FIXED, limitMax: FIXED }, [1, 0, 0]);
        const turn = axisAngle([0, 0, 1], 0.3);
        quat.copy(ball.quaternion, turn);
        vec3.transformQuat(ball.centerOfMassPosition, [1, 0, 0], turn);
        expect(vec3.length(ballPoint(ball, constraint))).toBeLessThan(1e-12);

        sixDOFConstraint.def.solvePosition(constraint, world.bodies, DT, 1);

        // turning the ball back swings the joint point 0.2955 away. the point solve, with its moment arm taken from the
        // turned back ball, splits the correction by 1/m : r^2/I = 1 : 10, so it turns the ball by about
        // 10/11 * 0.2955 = 0.269 and leaves the second order residual of that turn, 0.269^2 / 2 = 0.036
        const error = vec3.length(ballPoint(ball, constraint));
        expect(error).toBeGreaterThan(0.03);
        expect(error).toBeLessThan(0.04);
    });

    test('fixed rotation offsets compose as z * y * x euler angles', () => {
        const limits = [0, 0, 0, 0.4, 0.3, 0];
        const rest = quat.multiply(quat.create(), axisAngle([0, 1, 0], 0.3), axisAngle([1, 0, 0], 0.4));
        const { world, ball, constraint } = createScene({ swingType: SwingType.PYRAMID, limitMin: limits, limitMax: limits });
        quat.copy(ball.quaternion, rest);

        sixDOFConstraint.def.solvePosition(constraint, world.bodies, DT, 1);
        expect(quat.getAngle(ball.quaternion, rest)).toBeLessThan(1e-6);
    });
});

describe('sixDOFConstraint motors', () => {
    test('translation motor springs default to 2 hz, as every axis', () => {
        const { constraint } = createScene();
        for (const settings of constraint.motorSettings) {
            expect(settings.springSettings.mode).toBe(SpringMode.FREQUENCY_AND_DAMPING);
            expect(settings.springSettings.frequencyOrStiffness).toBe(2);
            expect(settings.springSettings.damping).toBe(1);
        }
    });

    test('rotation motors are limited by the torque limits, translation motors by the force limits', () => {
        const settings = Array.from({ length: 6 }, () => motorSettings.create());
        motorSettings.setTorqueLimit(settings[SixDOFAxis.ROTATION_X], 3);
        motorSettings.setForceLimit(settings[SixDOFAxis.TRANSLATION_X], 5);
        const { world, ball, constraint } = createScene({ motorSettings: settings });
        sixDOFConstraint.setMotorState(constraint, SixDOFAxis.ROTATION_X, MotorState.VELOCITY);
        sixDOFConstraint.setMotorState(constraint, SixDOFAxis.TRANSLATION_X, MotorState.VELOCITY);
        sixDOFConstraint.setTargetAngularVelocityCS(constraint, [100, 0, 0]);
        sixDOFConstraint.setTargetVelocityCS(constraint, [100, 0, 0]);

        solveVelocities(world, constraint);
        const invInertia = ball.motionProperties.invInertiaDiagonal[0];
        expect(ball.motionProperties.angularVelocity[0]).toBeCloseTo(3 * DT * invInertia, 9);
        expect(ball.motionProperties.linearVelocity[0]).toBeCloseTo(5 * DT * ball.motionProperties.invMass, 9);
    });

    test('a target orientation inside the limits is kept as given', () => {
        const { constraint } = createScene();
        // w < 0, the same rotation as its negation
        const orientation = quat.scale(quat.create(), axisAngle([1, 2, 3], 0.2), -1);
        sixDOFConstraint.setTargetOrientationCS(constraint, orientation);
        expect(constraint.targetOrientation).toEqual(orientation);
    });

    test('a position motor on x keeps the twist about x of a near half turn about y', () => {
        const { world, ball, constraint } = createScene();
        sixDOFConstraint.setMotorState(constraint, SixDOFAxis.ROTATION_X, MotorState.POSITION);
        // the twist about x is a quarter turn, though its unnormalized length is tiny: 2e-12
        quat.normalize(constraint.targetOrientation, quat.fromValues(1e-6, 1, 0, 1e-6));

        solveVelocities(world, constraint);
        expect(ball.motionProperties.angularVelocity[0]).toBeGreaterThan(0);
    });

    test('position and velocity motors drive the velocity, a damping only spring softly', () => {
        const probe = createScene().ball.motionProperties;
        const settings = Array.from({ length: 6 }, () => motorSettings.create());
        const damped = settings[SixDOFAxis.TRANSLATION_X].springSettings;
        damped.mode = SpringMode.STIFFNESS_AND_DAMPING;
        damped.frequencyOrStiffness = 0;
        // dt c / m = 1 halves the target
        damped.damping = 1 / probe.invMass / DT;

        const { world, ball, constraint } = createScene({ motorSettings: settings });
        sixDOFConstraint.setMotorState(constraint, SixDOFAxis.TRANSLATION_X, MotorState.POSITION_AND_VELOCITY);
        sixDOFConstraint.setMotorState(constraint, SixDOFAxis.ROTATION_Z, MotorState.POSITION_AND_VELOCITY);
        sixDOFConstraint.setTargetVelocityCS(constraint, [2, 0, 0]);
        sixDOFConstraint.setTargetAngularVelocityCS(constraint, [0, 0, 2]);

        solveVelocities(world, constraint);
        expect(ball.motionProperties.linearVelocity[0]).toBeCloseTo(1, 6);
        expect(ball.motionProperties.angularVelocity[2]).toBeGreaterThan(0);
    });
});
