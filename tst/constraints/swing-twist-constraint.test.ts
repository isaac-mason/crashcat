import { describe, expect, test } from 'vitest';
import { MotionType, MotorState, motorSettings, rigidBody, SpringMode, sphere, swingTwistConstraint } from '../../src';
import { createTestWorld } from '../helpers';

const DT = 1 / 60;

/** a static base and a dynamic ball joined at the origin, twist about x, every rotation within the limits */
function createScene(settings: Partial<swingTwistConstraint.SwingTwistConstraintSettings> = {}) {
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
        position: [0, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    const constraint = swingTwistConstraint.create(world, {
        bodyIdA: base.id,
        bodyIdB: ball.id,
        position1: [0, 0, 0],
        position2: [0, 0, 0],
        twistAxis1: [1, 0, 0],
        planeAxis1: [0, 1, 0],
        twistAxis2: [1, 0, 0],
        planeAxis2: [0, 1, 0],
        normalHalfConeAngle: Math.PI / 2,
        planeHalfConeAngle: Math.PI / 2,
        twistMinAngle: -Math.PI,
        twistMaxAngle: Math.PI,
        ...settings,
    });
    return { world, ball, constraint };
}

function solveVelocities(world: ReturnType<typeof createScene>['world'], constraint: swingTwistConstraint.SwingTwistConstraint) {
    swingTwistConstraint.def.setupVelocity(constraint, world.bodies, DT);
    swingTwistConstraint.def.warmStartVelocity(constraint, world.bodies, 1);
    for (let i = 0; i < 10; i++) swingTwistConstraint.def.solveVelocity(constraint, world.bodies, DT);
}

describe('swingTwistConstraint position and velocity motors', () => {
    test('a damping only twist spring is a soft velocity drive, the swing spring drives the swing velocity', () => {
        const inertia = 1 / createScene().ball.motionProperties.invInertiaDiagonal[0];
        const twistMotorSettings = motorSettings.create();
        twistMotorSettings.springSettings.mode = SpringMode.STIFFNESS_AND_DAMPING;
        twistMotorSettings.springSettings.frequencyOrStiffness = 0;
        // dt c / I = 1 halves the target
        twistMotorSettings.springSettings.damping = inertia / DT;

        const { world, ball, constraint } = createScene({ twistMotorSettings });
        swingTwistConstraint.setTwistMotorState(constraint, MotorState.POSITION_AND_VELOCITY);
        swingTwistConstraint.setSwingMotorState(constraint, MotorState.POSITION_AND_VELOCITY);
        swingTwistConstraint.setTargetAngularVelocityCS(constraint, [2, 0, 2]);

        solveVelocities(world, constraint);
        expect(ball.motionProperties.angularVelocity[0]).toBeCloseTo(1, 6);
        expect(ball.motionProperties.angularVelocity[2]).toBeGreaterThan(0);
    });
});
