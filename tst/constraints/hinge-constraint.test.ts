import type { Quat, Vec3 } from 'math';
import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    ConstraintSpace,
    hingeConstraint,
    MotionType,
    MotorState,
    motorSettings,
    type RigidBody,
    rigidBody,
    SpringMode,
    sphere,
} from '../../src';
import type { World } from '../../src/world';
import { createTestWorld } from '../helpers';

const DT = 1 / 60;

function axisAngle(axis: Vec3, angle: number): Quat {
    const unit = vec3.normalize(vec3.create(), axis);
    return quat.setAxisAngle(quat.create(), unit, angle);
}

function createBody(world: World, objectLayer: number, motionType: MotionType, position: Vec3, quaternion: Quat): RigidBody {
    return rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer,
        motionType,
        position,
        quaternion,
        linearDamping: 0,
        angularDamping: 0,
    });
}

/** setup, warm start and ten velocity iterations, as one solver step runs them */
function solveVelocities(world: World, constraint: hingeConstraint.HingeConstraint): void {
    hingeConstraint.def.setupVelocity(constraint, world.bodies, DT);
    hingeConstraint.def.warmStartVelocity(constraint, world.bodies, 1);
    for (let i = 0; i < 10; i++) hingeConstraint.def.solveVelocity(constraint, world.bodies, DT);
}

/** a ball on a static base, both centered on the pivot, hinged about world z */
function createSpinner(settings: Partial<hingeConstraint.HingeConstraintSettings> = {}) {
    const { world, layers } = createTestWorld();
    const base = createBody(world, layers.OBJECT_LAYER_NOT_MOVING, MotionType.STATIC, [0, 0, 0], quat.create());
    const ball = createBody(world, layers.OBJECT_LAYER_MOVING, MotionType.DYNAMIC, [0, 0, 0], quat.create());
    const constraint = hingeConstraint.create(world, {
        bodyIdA: base.id,
        bodyIdB: ball.id,
        pointA: [0, 0, 0],
        pointB: [0, 0, 0],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        ...settings,
    });
    return { world, ball, constraint };
}

describe('hingeConstraint rest orientation', () => {
    const cases: { name: string; rotationA: Quat; rotationB: Quat }[] = [
        {
            name: 'body a unrotated, body b a quarter turn about the hinge',
            rotationA: quat.create(),
            rotationB: axisAngle([0, 0, 1], -Math.PI / 2),
        },
        {
            name: 'both bodies rotated about other axes',
            rotationA: axisAngle([1, 1, 0], 0.7),
            rotationB: axisAngle([0, 1, 2], -1.3),
        },
    ];

    for (const { name, rotationA, rotationB } of cases) {
        test(`local space axes that line up in world space read angle 0: ${name}`, () => {
            const { world, layers } = createTestWorld();
            const bodyA = createBody(world, layers.OBJECT_LAYER_MOVING, MotionType.DYNAMIC, [0, 0, 0], rotationA);
            const bodyB = createBody(world, layers.OBJECT_LAYER_MOVING, MotionType.DYNAMIC, [0, 2, 0], rotationB);

            // the hinge frame in world space is normal +x, hinge +z; each body sees it through its own rotation
            const worldNormal = vec3.fromValues(1, 0, 0);
            const worldHinge = vec3.fromValues(0, 0, 1);
            const invA = quat.conjugate(quat.create(), rotationA);
            const invB = quat.conjugate(quat.create(), rotationB);

            const constraint = hingeConstraint.create(world, {
                bodyIdA: bodyA.id,
                bodyIdB: bodyB.id,
                space: ConstraintSpace.LOCAL,
                pointA: [0, 1, 0],
                pointB: [0, -1, 0],
                hingeAxisA: vec3.transformQuat(vec3.create(), worldHinge, invA),
                hingeAxisB: vec3.transformQuat(vec3.create(), worldHinge, invB),
                normalAxisA: vec3.transformQuat(vec3.create(), worldNormal, invA),
                normalAxisB: vec3.transformQuat(vec3.create(), worldNormal, invB),
                maxFrictionTorque: 1,
            });

            hingeConstraint.def.setupVelocity(constraint, world.bodies, DT);
            expect(hingeConstraint.getCurrentAngle(constraint)).toBeCloseTo(0, 9);

            // turning body b about the world hinge axis turns the angle by as much
            quat.multiply(bodyB.quaternion, axisAngle(worldHinge, 0.3), bodyB.quaternion);
            hingeConstraint.def.setupVelocity(constraint, world.bodies, DT);
            expect(hingeConstraint.getCurrentAngle(constraint)).toBeCloseTo(0.3, 9);
        });
    }

    test('axes a hair apart are not taken as identical', () => {
        const angle = 9e-7;
        const { constraint } = createSpinner({
            space: ConstraintSpace.LOCAL,
            normalAxisB: [Math.cos(angle), Math.sin(angle), 0],
        });

        // r0^-1 = c2 c1^-1 turns by the hair about the hinge axis
        expect(constraint.invInitialOrientation[2]).toBeCloseTo(Math.sin(angle / 2), 12);
        expect(constraint.invInitialOrientation[3]).toBeCloseTo(Math.cos(angle / 2), 12);
    });
});

describe('hingeConstraint limits', () => {
    test('setLimits asserts min in [-pi, 0] and max in [0, pi] instead of clamping', () => {
        const { constraint } = createSpinner();
        expect(() => hingeConstraint.setLimits(constraint, 0.1, 1)).toThrow();
        expect(() => hingeConstraint.setLimits(constraint, -1, -0.1)).toThrow();
        expect(() => hingeConstraint.setLimits(constraint, -4, 1)).toThrow();
        expect(() => hingeConstraint.setLimits(constraint, -1, 4)).toThrow();

        hingeConstraint.setLimits(constraint, -0.5, 0.25);
        expect(constraint.limitsMin).toBe(-0.5);
        expect(constraint.limitsMax).toBe(0.25);
        expect(constraint.hasLimits).toBe(true);

        hingeConstraint.setLimits(constraint, -Math.PI, Math.PI);
        expect(constraint.hasLimits).toBe(false);
    });

    test('create asserts on limits outside [-pi, pi]', () => {
        expect(() => createSpinner({ limitsMin: -4, limitsMax: 1 })).toThrow();
    });

    test('setLimits leaves the target angle alone; only setTargetAngle clamps', () => {
        const { constraint } = createSpinner();
        hingeConstraint.setTargetAngle(constraint, 1);
        hingeConstraint.setLimits(constraint, -0.5, 0.5);
        expect(constraint.targetAngle).toBe(1);

        hingeConstraint.setTargetAngle(constraint, 1);
        expect(constraint.targetAngle).toBe(0.5);
    });
});

describe('hingeConstraint position and velocity motor', () => {
    test('drives the angular velocity with the spring at the target angle', () => {
        const { world, ball, constraint } = createSpinner();
        hingeConstraint.setMotorState(constraint, MotorState.POSITION_AND_VELOCITY);
        hingeConstraint.setTargetAngularVelocity(constraint, 2);
        hingeConstraint.setTargetAngle(constraint, 0);

        solveVelocities(world, constraint);
        expect(ball.motionProperties.angularVelocity[2]).toBeGreaterThan(0);
    });

    test('a damping only spring is a soft velocity drive: w = w_target * dt c / I / (dt c / I + 1)', () => {
        const inertia = 1 / createSpinner().ball.motionProperties.invInertiaDiagonal[2];
        const settings = motorSettings.create();
        settings.springSettings.mode = SpringMode.STIFFNESS_AND_DAMPING;
        settings.springSettings.frequencyOrStiffness = 0;
        // dt c / I = 1 halves the target
        settings.springSettings.damping = inertia / DT;

        const { world, ball, constraint } = createSpinner({ motorSettings: settings });
        hingeConstraint.setMotorState(constraint, MotorState.POSITION_AND_VELOCITY);
        hingeConstraint.setTargetAngularVelocity(constraint, 2);

        solveVelocities(world, constraint);
        expect(ball.motionProperties.angularVelocity[2]).toBeCloseTo(1, 6);
    });
});
