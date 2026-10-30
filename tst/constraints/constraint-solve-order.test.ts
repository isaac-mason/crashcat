import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { distanceConstraint, hingeConstraint, MotionType, pointConstraint, rigidBody, sphere, updateWorld } from '../../src';
import { createTestWorld } from '../helpers';

function createBodies(count: number) {
    const { world, layers } = createTestWorld();
    const bodies = [];
    for (let i = 0; i < count; i++) {
        bodies.push(
            rigidBody.create(world, {
                shape: sphere.create({ radius: 0.25 }),
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(i, 0, 0),
            }),
        );
    }
    return { world, bodies };
}

describe('constraint solve order', () => {
    // at equal priority, whatever their types, as jolt's constraint index
    test('an island solves its constraints in creation order across types', () => {
        const { world, bodies } = createBodies(3);
        const [a, b, c] = bodies;
        const hinge = hingeConstraint.create(world, {
            bodyIdA: a.id,
            bodyIdB: b.id,
            pointA: vec3.fromValues(0.5, 0, 0),
            pointB: vec3.fromValues(0.5, 0, 0),
            hingeAxisA: vec3.fromValues(0, 1, 0),
            hingeAxisB: vec3.fromValues(0, 1, 0),
            normalAxisA: vec3.fromValues(1, 0, 0),
            normalAxisB: vec3.fromValues(1, 0, 0),
        });
        const point = pointConstraint.create(world, {
            bodyIdA: b.id,
            bodyIdB: c.id,
            pointA: vec3.fromValues(1.5, 0, 0),
            pointB: vec3.fromValues(1.5, 0, 0),
        });
        const distance = distanceConstraint.create(world, {
            bodyIdA: a.id,
            bodyIdB: c.id,
            pointA: vec3.fromValues(0, 0, 0),
            pointB: vec3.fromValues(2, 0, 0),
        });
        updateWorld(world, undefined, 1 / 60);

        expect(world.islands.islands.length).toBe(1);
        expect(world.islands.islands[0].constraintIds).toEqual([hinge.id, point.id, distance.id]);
    });

    test('a removal moves the last constraint into the gap, and a body removal takes its constraints out', () => {
        const { world, bodies } = createBodies(2);
        const [a, b] = bodies;
        const settings = { bodyIdA: a.id, bodyIdB: b.id, pointA: vec3.fromValues(0.5, 0, 0), pointB: vec3.fromValues(0.5, 0, 0) };
        const first = pointConstraint.create(world, settings);
        const second = distanceConstraint.create(world, settings);
        const third = pointConstraint.create(world, settings);
        expect(world.constraints.list).toEqual([first.id, second.id, third.id]);
        expect(third.listIndex).toBe(2);

        pointConstraint.remove(world, first);
        expect(world.constraints.list).toEqual([third.id, second.id]);
        expect(third.listIndex).toBe(0);

        rigidBody.remove(world, a);
        expect(world.constraints.list).toEqual([]);
    });

    // jolt's priority is a uint32, which a negative or fractional one would not be
    test('a priority is an integer in [0, 2^32)', () => {
        const { world, bodies } = createBodies(2);
        const [a, b] = bodies;
        const settings = { bodyIdA: a.id, bodyIdB: b.id, pointA: vec3.fromValues(0.5, 0, 0), pointB: vec3.fromValues(0.5, 0, 0) };
        expect(() => pointConstraint.create(world, { ...settings, constraintPriority: -1 })).toThrow();
        expect(() => distanceConstraint.create(world, { ...settings, constraintPriority: 1.5 })).toThrow();
        expect(() => pointConstraint.create(world, { ...settings, constraintPriority: 2 ** 32 })).toThrow();
        expect(pointConstraint.create(world, { ...settings, constraintPriority: 0xffffffff }).constraintPriority).toBe(
            0xffffffff,
        );
    });
});
