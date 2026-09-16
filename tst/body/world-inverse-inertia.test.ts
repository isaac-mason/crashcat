import { mat4, type Quat, quat, vec3 } from 'math';
import { describe, expect, it } from 'vitest';
import { box, MotionType, rigidBody, sphere, updateWorld } from '../../src';
import { DOF_ALL } from '../../src/body/dof';
import * as motionProperties from '../../src/body/motion-properties';
import { createTestWorld } from '../helpers';

const AXIS = vec3.normalize(vec3.create(), vec3.fromValues(0.3, 0.8, -0.5));

function makeMotionProperties(): motionProperties.MotionProperties {
    const mp = motionProperties.create();
    mp.invMass = 1;
    mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
    quat.setAxisAngle(mp.inertiaRotation, vec3.normalize(vec3.create(), vec3.fromValues(1, 2, 3)), 0.7);
    mp.allowedDegreesOfFreedom = DOF_ALL;
    return mp;
}

function freshWorldInverseInertia(mp: motionProperties.MotionProperties, q: Quat) {
    const rotation = mat4.fromQuat(mat4.create(), q);
    return motionProperties.getInverseInertiaForRotation(mat4.create(), mp, rotation);
}

describe('getWorldInverseInertia', () => {
    it('computes once per stamp and returns the memo for repeated calls', () => {
        const mp = makeMotionProperties();
        const q = quat.setAxisAngle(quat.create(), AXIS, 1.1);
        const out = mat4.create();

        const first = motionProperties.getWorldInverseInertia(out, mp, q, 1);
        expect(first).toBe(mp.worldInverseInertia);
        expect(mp.worldInverseInertiaStamp).toBe(1);
        expect(Array.from(first)).toEqual(Array.from(freshWorldInverseInertia(mp, q)));

        // same stamp: the rotation is ignored, the memo is returned untouched
        const rotated = quat.setAxisAngle(quat.create(), AXIS, 2.0);
        const again = motionProperties.getWorldInverseInertia(out, mp, rotated, 1);
        expect(again).toBe(mp.worldInverseInertia);
        expect(Array.from(again)).toEqual(Array.from(freshWorldInverseInertia(mp, q)));

        // new stamp: recomputed for the rotation passed now
        const next = motionProperties.getWorldInverseInertia(out, mp, rotated, 2);
        expect(mp.worldInverseInertiaStamp).toBe(2);
        expect(Array.from(next)).toEqual(Array.from(freshWorldInverseInertia(mp, rotated)));
    });

    it('computes fresh into out for STEP_STAMP_NONE without touching the memo', () => {
        const mp = makeMotionProperties();
        const q = quat.setAxisAngle(quat.create(), AXIS, 1.1);
        motionProperties.getWorldInverseInertia(mat4.create(), mp, q, 5);
        const memo = Array.from(mp.worldInverseInertia);

        const rotated = quat.setAxisAngle(quat.create(), AXIS, 2.0);
        const out = mat4.create();
        const result = motionProperties.getWorldInverseInertia(out, mp, rotated, motionProperties.STEP_STAMP_NONE);
        expect(result).toBe(out);
        expect(Array.from(result)).toEqual(Array.from(freshWorldInverseInertia(mp, rotated)));

        expect(mp.worldInverseInertiaStamp).toBe(5);
        expect(Array.from(mp.worldInverseInertia)).toEqual(memo);
    });

    it('is invalidated when the local inertia changes', () => {
        const mp = makeMotionProperties();
        motionProperties.getWorldInverseInertia(mat4.create(), mp, quat.create(), 3);
        expect(mp.worldInverseInertiaStamp).toBe(3);

        const massProperties = { mass: 2, inertia: mat4.identity(mat4.create()) };
        motionProperties.setMassProperties(mp, DOF_ALL, massProperties);
        expect(mp.worldInverseInertiaStamp).toBe(motionProperties.STEP_STAMP_NONE);
    });
});

describe('world inverse inertia in the step', () => {
    it('integrates torque as I^-1 τ dt for a rotated body with a locked axis', () => {
        const { world, layers } = createTestWorld();
        world.settings.gravityEnabled = false;

        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 1, 2) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 10, 0),
            quaternion: quat.setAxisAngle(quat.create(), AXIS, 0.9),
            allowedDegreesOfFreedom: DOF_ALL & ~(1 << 4), // rotation about y locked
            angularDamping: 0,
        });

        const torque = vec3.fromValues(3, -2, 5);
        rigidBody.addTorque(world, body, torque, true);

        const expected = mat4.multiply3x3Vec(vec3.create(), rigidBody.getInverseInertia(mat4.create(), body), torque);
        vec3.scale(expected, expected, 1 / 60);

        updateWorld(world, undefined, 1 / 60);

        const omega = body.motionProperties.angularVelocity;
        expect(omega[0]).toBeCloseTo(expected[0], 10);
        expect(omega[1]).toBe(0);
        expect(omega[2]).toBeCloseTo(expected[2], 10);
    });

    it('leaves angular velocity alone for a body without torque', () => {
        const { world, layers } = createTestWorld();
        world.settings.gravityEnabled = false;

        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 1, 2) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 10, 0),
            angularDamping: 0,
        });
        rigidBody.setAngularVelocity(world, body, vec3.fromValues(0.1, 0.2, 0.3));

        updateWorld(world, undefined, 1 / 60);

        expect(Array.from(body.motionProperties.angularVelocity)).toEqual([0.1, 0.2, 0.3]);
    });

    it('rebuilds the memo for a body rotated between steps and for a body woken by a contact', () => {
        const { world, layers } = createTestWorld();

        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(10, 0.5, 10) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        const resting = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0),
        });
        const mp = resting.motionProperties;

        updateWorld(world, undefined, 1 / 60);

        // the contact setup consumed the memo this step
        expect(mp.worldInverseInertiaStamp).toBe(world.bodies.stepStamp);

        // rotate the box a quarter turn about y between steps: still resting flat, different inertia frame
        const turned = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), Math.PI / 2);
        rigidBody.setQuaternion(world, resting, turned, true);
        expect(mp.worldInverseInertiaStamp).toBe(motionProperties.STEP_STAMP_NONE);

        updateWorld(world, undefined, 1 / 60);

        expect(mp.worldInverseInertiaStamp).toBe(world.bodies.stepStamp);
        const fresh = freshWorldInverseInertia(mp, resting.quaternion);
        for (let i = 0; i < 16; i++) {
            expect(mp.worldInverseInertia[i]).toBeCloseTo(fresh[i], 4);
        }

        // let the box fall asleep, then wake it by dropping a ball on it
        for (let i = 0; i < 240 && !resting.sleeping; i++) {
            updateWorld(world, undefined, 1 / 60);
        }
        expect(resting.sleeping).toBe(true);
        const stampWhenAsleep = mp.worldInverseInertiaStamp;

        // a sleeping body sets up no constraints, so its memo stays behind the step stamp
        for (let i = 0; i < 5; i++) {
            updateWorld(world, undefined, 1 / 60);
        }
        expect(resting.sleeping).toBe(true);
        expect(mp.worldInverseInertiaStamp).toBe(stampWhenAsleep);
        expect(stampWhenAsleep).toBeLessThan(world.bodies.stepStamp);

        const ball = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.25 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1.3, 0),
        });
        rigidBody.setLinearVelocity(world, ball, vec3.fromValues(0, -5, 0));

        for (let i = 0; i < 30 && resting.sleeping; i++) {
            updateWorld(world, undefined, 1 / 60);
        }
        expect(resting.sleeping).toBe(false);

        // woken by narrowphase mid-step: the memo is from this step, not from when it fell asleep
        expect(mp.worldInverseInertiaStamp).toBe(world.bodies.stepStamp);
        expect(mp.worldInverseInertiaStamp).toBeGreaterThan(stampWhenAsleep);
    });
});
