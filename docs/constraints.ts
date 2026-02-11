/* eslint-disable @typescript-eslint/no-unused-vars */
import {
    box,
    ConstraintSpace,
    createWorld,
    createWorldSettings,
    distanceConstraint,
    fixedConstraint,
    hingeConstraint,
    MotionType,
    motorSettings,
    MotorState,
    pointConstraint,
    rigidBody,
    sliderConstraint,
    sphere,
} from 'crashcat';
import { vec3 } from 'mathcat';

/* SNIPPET_START: basic */
const world = createWorld(createWorldSettings());

// create two bodies
const bodyA = rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
    objectLayer: 0,
    motionType: MotionType.STATIC,
    position: [0, 5, 0],
});

const bodyB = rigidBody.create(world, {
    shape: sphere.create({ radius: 0.5 }),
    objectLayer: 1,
    motionType: MotionType.DYNAMIC,
    position: [0, 3, 0],
});

// create a point constraint connecting them
const constraint = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 4.5, 0], // point on bodyA in world space
    pointB: [0, 3.5, 0], // point on bodyB in world space
    space: ConstraintSpace.WORLD,
});

// constraints can be enabled/disabled
constraint.enabled = false;

// remove constraint when done
pointConstraint.remove(world, constraint);
/* SNIPPET_END: basic */

/* SNIPPET_START: types */
// point constraint - connects two bodies at a point (removes 3 DOF)
const point = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 1, 0],
    pointB: [0, 0, 0],
});

// distance constraint - maintains distance between two points (removes 1 DOF)
const distance = distanceConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 1, 0],
    pointB: [0, 0, 0],
    minDistance: 1,
    maxDistance: 2,
});

// hinge constraint - allows rotation around an axis (removes 5 DOF)
const hinge = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
});

// slider constraint - allows movement along an axis (removes 5 DOF)
const slider = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
    limitsMin: -2,
    limitsMax: 2,
});

// fixed constraint - completely locks two bodies together (removes 6 DOF)
const fixed = fixedConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    point1: [0, 0, 0],
    point2: [0, 0, 0],
    axisX1: [1, 0, 0],
    axisY1: [0, 1, 0],
    axisX2: [1, 0, 0],
    axisY2: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
});
/* SNIPPET_END: types */

/* SNIPPET_START: motors-velocity */
// create a hinge with a velocity motor
const hingeWithMotor = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
});

// configure motor for velocity control
motorSettings.setTorqueLimit(hingeWithMotor.motorSettings, 100); // max torque in N*m
hingeConstraint.setMotorState(hingeWithMotor, MotorState.VELOCITY);
hingeConstraint.setTargetAngularVelocity(hingeWithMotor, 2 * Math.PI); // rad/s

// for slider constraints, use force limits instead
const sliderWithMotor = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
});

motorSettings.setForceLimit(sliderWithMotor.motorSettings, 500); // max force in N
sliderConstraint.setMotorState(sliderWithMotor, MotorState.VELOCITY);
sliderConstraint.setTargetVelocity(sliderWithMotor, 2); // m/s
/* SNIPPET_END: motors-velocity */

/* SNIPPET_START: motors-position */
// position motors drive to a target angle/position using a spring
const positionHinge = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
});

// configure spring settings for position motor
// frequency: how fast it reaches target (Hz). higher = stiffer spring
// damping: prevents overshoot. 0 = oscillates forever, 1 = critical damping, >1 = overdamped
positionHinge.motorSettings.springSettings.frequencyOrStiffness = 2; // 2 Hz - soft spring
positionHinge.motorSettings.springSettings.damping = 1; // critical damping, no overshoot

// drive to target angle
hingeConstraint.setMotorState(positionHinge, MotorState.POSITION);
hingeConstraint.setTargetAngle(positionHinge, Math.PI / 2); // 90 degrees

// same for slider - drives to target position
const positionSlider = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
});

positionSlider.motorSettings.springSettings.frequencyOrStiffness = 5; // 5 Hz - stiffer
positionSlider.motorSettings.springSettings.damping = 0.5; // some oscillation
sliderConstraint.setMotorState(positionSlider, MotorState.POSITION);
sliderConstraint.setTargetPosition(positionSlider, 1.5); // meters
/* SNIPPET_END: motors-position */

/* SNIPPET_START: limits */
// hinge constraints can have angle limits
const limitedHinge = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
    limitsMin: -Math.PI / 4, // -45 degrees
    limitsMax: Math.PI / 4, // +45 degrees
});

// slider constraints can have position limits
const limitedSlider = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
    limitsMin: -1, // -1 meter
    limitsMax: 2, // +2 meters
});

// limits can use soft springs instead of hard stops
limitedSlider.limitsSpringSettings.frequencyOrStiffness = 10; // Hz
limitedSlider.limitsSpringSettings.damping = 0.7;
/* SNIPPET_END: limits */

/* SNIPPET_START: local-vs-world */
// world space - specify attachment points in world coordinates
const worldConstraint = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 5, 0], // absolute world position
    pointB: [0, 3, 0], // absolute world position
    space: ConstraintSpace.WORLD, // default
});

// local space - specify attachment points relative to body center of mass
const localConstraint = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0], // offset from bodyA's center
    pointB: [0, -0.5, 0], // offset from bodyB's center
    space: ConstraintSpace.LOCAL,
});
/* SNIPPET_END: local-vs-world */
