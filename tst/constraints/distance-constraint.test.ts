import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { ConstraintSpace, distanceConstraint, MotionType, rigidBody, SpringMode, sphere, updateWorld } from '../../src';
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
        position: vec3.fromValues(5, 0, 0),
    });
    return { world, bodyA, bodyB };
}

describe('distanceConstraint', () => {
    test('a reused pooled constraint created without spring settings gets the default spring', () => {
        const { world, bodyA, bodyB } = createTwoBodies();

        const first = distanceConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0, 0, 0),
            pointB: vec3.fromValues(5, 0, 0),
            springSettings: { mode: SpringMode.STIFFNESS_AND_DAMPING, frequencyOrStiffness: 500, damping: 3 },
        });
        expect(first.limitsSpringSettings.frequencyOrStiffness).toBe(500);
        distanceConstraint.remove(world, first);

        const second = distanceConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0, 0, 0),
            pointB: vec3.fromValues(5, 0, 0),
        });
        expect(second).toBe(first);
        expect(second.limitsSpringSettings.mode).toBe(SpringMode.FREQUENCY_AND_DAMPING);
        expect(second.limitsSpringSettings.frequencyOrStiffness).toBe(0);
        expect(second.limitsSpringSettings.damping).toBe(0);
    });

    test('a reused pooled constraint starts with the +y fallback normal', () => {
        const { world, bodyA, bodyB } = createTwoBodies();

        // the first constraint's setup points its normal along +x, from a to b
        const first = distanceConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0, 0, 0),
            pointB: vec3.fromValues(5, 0, 0),
        });
        distanceConstraint.def.setupVelocity(first, world.bodies, 1 / 60);
        expect(first.worldSpaceNormal).toEqual([1, 0, 0]);
        distanceConstraint.remove(world, first);

        // coincident points have no direction of their own, so the normal is the fallback
        const second = distanceConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(2, 0, 0),
            pointB: vec3.fromValues(2, 0, 0),
        });
        expect(second).toBe(first);
        expect(second.worldSpaceNormal).toEqual([0, 1, 0]);
    });

    test('setDistance sets both limits as given', () => {
        const { world, bodyA, bodyB } = createTwoBodies();
        const constraint = distanceConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0, 0, 0),
            pointB: vec3.fromValues(5, 0, 0),
            space: ConstraintSpace.WORLD,
        });
        expect(constraint.minDistance).toBeCloseTo(5, 6);
        expect(constraint.maxDistance).toBeCloseTo(5, 6);

        distanceConstraint.setDistance(constraint, 2, 7);
        expect(constraint.minDistance).toBe(2);
        expect(constraint.maxDistance).toBe(7);

        // a negative is a value like any other, not "keep the current limit"
        distanceConstraint.setDistance(constraint, -1, 4);
        expect(constraint.minDistance).toBe(-1);
        expect(constraint.maxDistance).toBe(4);
    });

    test('setDistance asserts min <= max', () => {
        const { world, bodyA, bodyB } = createTwoBodies();
        const constraint = distanceConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0, 0, 0),
            pointB: vec3.fromValues(5, 0, 0),
        });

        expect(() => distanceConstraint.setDistance(constraint, 3, 2)).toThrow();
    });

    test('create asserts min <= max when both are given', () => {
        const { world, bodyA, bodyB } = createTwoBodies();

        expect(() =>
            distanceConstraint.create(world, {
                bodyIdA: bodyA.id,
                bodyIdB: bodyB.id,
                pointA: vec3.fromValues(0, 0, 0),
                pointB: vec3.fromValues(5, 0, 0),
                minDistance: 4,
                maxDistance: 3,
            }),
        ).toThrow();
    });
});

describe('distanceConstraint: sleeping', () => {
    test('a weight hanging still from a static anchor sleeps and stays asleep', () => {
        const { world, layers } = createTestWorld();
        const anchor = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.1 }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 10, 0),
        });
        const weight = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.5 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 7, 0),
        });
        distanceConstraint.create(world, {
            bodyIdA: weight.id,
            bodyIdB: anchor.id,
            pointA: vec3.fromValues(0, 7, 0),
            pointB: vec3.fromValues(0, 10, 0),
            space: ConstraintSpace.WORLD,
        });

        let slept = -1;
        for (let i = 0; i < 300 && slept < 0; i++) {
            updateWorld(world, undefined, 1 / 60);
            if (weight.sleeping) slept = i;
        }
        expect(slept).toBeGreaterThanOrEqual(0);

        // the static anchor is never active, so nothing wakes the weight again
        for (let i = 0; i < 120; i++) updateWorld(world, undefined, 1 / 60);
        expect(weight.sleeping).toBe(true);
        expect(Array.from(weight.motionProperties.linearVelocity)).toEqual([0, 0, 0]);
    });
});
