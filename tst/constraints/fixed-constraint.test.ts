import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { fixedConstraint, MotionType, rigidBody, sphere } from '../../src';
import { createTestWorld } from '../helpers';

function createAutoDetected(motionTypeA: MotionType, motionTypeB: MotionType) {
    const { world, layers } = createTestWorld();
    const bodyA = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: motionTypeA === MotionType.STATIC ? layers.OBJECT_LAYER_NOT_MOVING : layers.OBJECT_LAYER_MOVING,
        motionType: motionTypeA,
        position: vec3.fromValues(0, 0, 0),
    });
    const bodyB = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: motionTypeB === MotionType.STATIC ? layers.OBJECT_LAYER_NOT_MOVING : layers.OBJECT_LAYER_MOVING,
        motionType: motionTypeB,
        position: vec3.fromValues(4, 0, 0),
    });
    const constraint = fixedConstraint.create(world, { bodyIdA: bodyA.id, bodyIdB: bodyB.id, autoDetectPoint: true });
    return constraint;
}

describe('fixedConstraint auto-detected anchor', () => {
    test('two static bodies anchor on body b, as body a is static', () => {
        const constraint = createAutoDetected(MotionType.STATIC, MotionType.STATIC);
        expect(constraint.localSpacePosition1).toEqual([4, 0, 0]);
        expect(constraint.localSpacePosition2).toEqual([0, 0, 0]);
    });

    test('a static body a and a kinematic body b anchor on body b', () => {
        const constraint = createAutoDetected(MotionType.STATIC, MotionType.KINEMATIC);
        expect(constraint.localSpacePosition1).toEqual([4, 0, 0]);
        expect(constraint.localSpacePosition2).toEqual([0, 0, 0]);
    });

    test('a kinematic body a and a static body b anchor on body a', () => {
        const constraint = createAutoDetected(MotionType.KINEMATIC, MotionType.STATIC);
        expect(constraint.localSpacePosition1).toEqual([0, 0, 0]);
        expect(constraint.localSpacePosition2).toEqual([-4, 0, 0]);
    });

    test('a body made static anchors on the other body, whatever inverse mass it kept', () => {
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
            position: vec3.fromValues(4, 0, 0),
        });
        rigidBody.setMotionType(world, bodyA, MotionType.STATIC, false);

        const constraint = fixedConstraint.create(world, { bodyIdA: bodyA.id, bodyIdB: bodyB.id, autoDetectPoint: true });
        expect(constraint.localSpacePosition1).toEqual([4, 0, 0]);
        expect(constraint.localSpacePosition2).toEqual([0, 0, 0]);
    });

    test('two equal dynamic bodies anchor half way', () => {
        const constraint = createAutoDetected(MotionType.DYNAMIC, MotionType.DYNAMIC);
        expect(constraint.localSpacePosition1[0]).toBeCloseTo(2, 6);
        expect(constraint.localSpacePosition2[0]).toBeCloseTo(-2, 6);
    });
});
