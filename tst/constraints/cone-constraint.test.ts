import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { coneConstraint, MotionType, rigidBody, sphere } from '../../src';
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
        position: vec3.fromValues(1, 0, 0),
    });
    return { world, bodyA, bodyB };
}

describe('coneConstraint', () => {
    test('the initial rotation axis is the twist axis perpendicular jolt picks', () => {
        const { world, bodyA, bodyB } = createTwoBodies();

        // jolt: |x| <= |y| picks (0, z, -y), so +z gives +y
        const constraint = coneConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0.5, 0, 0),
            pointB: vec3.fromValues(0.5, 0, 0),
            twistAxisA: vec3.fromValues(0, 0, 1),
            twistAxisB: vec3.fromValues(0, 0, 1),
        });

        expect(constraint.worldSpaceRotationAxis[0]).toBeCloseTo(0, 6);
        expect(constraint.worldSpaceRotationAxis[1]).toBeCloseTo(1, 6);
        expect(constraint.worldSpaceRotationAxis[2]).toBeCloseTo(0, 6);
    });

    test('a tiny swing past the cone still updates the rotation axis', () => {
        const { world, bodyA, bodyB } = createTwoBodies();

        const constraint = coneConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0.5, 0, 0),
            pointB: vec3.fromValues(0.5, 0, 0),
            twistAxisA: vec3.fromValues(1, 0, 0),
            twistAxisB: vec3.fromValues(1, 0, 0),
            halfConeAngle: 0,
        });
        // the initial axis is the perpendicular of +x, which is -z
        expect(constraint.worldSpaceRotationAxis[2]).toBeCloseTo(-1, 6);

        // turn body b 5e-7 rad about +y: twist2 x twist1 is -y with length 5e-7
        quat.setAxisAngle(bodyB.quaternion, [0, 1, 0], 5e-7);
        coneConstraint.def.setupVelocity(constraint, world.bodies, 1 / 60);

        expect(constraint.cosTheta).toBeLessThan(constraint.cosHalfConeAngle);
        expect(constraint.worldSpaceRotationAxis[0]).toBeCloseTo(0, 6);
        expect(constraint.worldSpaceRotationAxis[1]).toBeCloseTo(-1, 6);
        expect(constraint.worldSpaceRotationAxis[2]).toBeCloseTo(0, 6);
    });

    test('the half cone angle must be in [0, pi]', () => {
        const { world, bodyA, bodyB } = createTwoBodies();
        const settings = {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0.5, 0, 0),
            pointB: vec3.fromValues(0.5, 0, 0),
            twistAxisA: vec3.fromValues(1, 0, 0),
            twistAxisB: vec3.fromValues(1, 0, 0),
        };

        expect(() => coneConstraint.create(world, { ...settings, halfConeAngle: -0.1 })).toThrow();
        expect(() => coneConstraint.create(world, { ...settings, halfConeAngle: Math.PI + 0.1 })).toThrow();

        const constraint = coneConstraint.create(world, { ...settings, halfConeAngle: Math.PI });
        expect(constraint.cosHalfConeAngle).toBeCloseTo(-1, 6);
        expect(() => coneConstraint.setHalfConeAngle(constraint, 4)).toThrow();
    });
});
