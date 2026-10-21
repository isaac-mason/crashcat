import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, type Listener, MotionType, rigidBody, sphere, updateWorld } from '../src';
import { getSleepTestPoints } from '../src/body/sleep';
import { createTestWorld } from './helpers';

describe('end of step bookkeeping', () => {
    test('position and world aabb follow the centre of mass after a step', () => {
        const { world, layers } = createTestWorld();
        const body = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.5 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(1, 10, 2),
        });
        updateWorld(world, undefined, 1 / 60);
        expect(body.centerOfMassPosition[1]).toBeLessThan(10);
        expect(body.position[1]).toBeCloseTo(body.centerOfMassPosition[1], 12);
        expect(body.aabb[1]).toBeCloseTo(body.position[1] - 0.5, 12);
        expect(body.aabb[4]).toBeCloseTo(body.position[1] + 0.5, 12);
    });

    test('forces are consumed by a step and left by a zero step, as jolt', () => {
        const { world, layers } = createTestWorld();
        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 10, 0),
        });
        rigidBody.addForce(world, body, vec3.fromValues(3, 0, 0), true);
        updateWorld(world, undefined, 1 / 60);
        expect(Array.from(body.motionProperties.force)).toEqual([0, 0, 0]);
        expect(body.motionProperties.linearVelocity[0]).toBeGreaterThan(0);

        // a step of no time moves nothing and leaves the force for the next step
        rigidBody.addForce(world, body, vec3.fromValues(3, 0, 0), true);
        const position = vec3.clone(body.position);
        const velocity = body.motionProperties.linearVelocity[0];
        updateWorld(world, undefined, 0);
        expect(Array.from(body.motionProperties.force)).toEqual([3, 0, 0]);
        expect(Array.from(body.position)).toEqual(Array.from(position));
        updateWorld(world, undefined, 1 / 60);
        expect(Array.from(body.motionProperties.force)).toEqual([0, 0, 0]);
        expect(body.motionProperties.linearVelocity[0]).toBeGreaterThan(velocity);
    });

    test('a zero step reports no contacts and keeps them', () => {
        const { world, layers } = createTestWorld();
        world.settings.sleeping.allowSleeping = false;
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(10, 0.5, 10) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0),
        });
        const counts = { added: 0, persisted: 0, removed: 0 };
        const listener: Listener = {
            onContactAdded: () => void counts.added++,
            onContactPersisted: () => void counts.persisted++,
            onContactRemoved: () => void counts.removed++,
        };
        for (let i = 0; i < 10; i++) updateWorld(world, listener, 1 / 60);

        // no collision runs, so nothing is reported, and the contact is still there for the next step
        Object.assign(counts, { added: 0, persisted: 0, removed: 0 });
        updateWorld(world, listener, 0);
        expect(counts).toEqual({ added: 0, persisted: 0, removed: 0 });
        updateWorld(world, listener, 1 / 60);
        expect(counts).toEqual({ added: 0, persisted: 1, removed: 0 });
    });

    // the two largest extents pick which rotated axes are tested, so all three smallest-extent
    // branches build a different pair of rotation matrix columns from the quaternion
    test.each([
        ['x smallest', [0.4, 1.2, 0.8], 1, 2],
        ['y smallest', [1.2, 0.4, 0.8], 0, 2],
        ['z smallest', [1.2, 0.8, 0.4], 0, 1],
    ] as const)('sleep test points match the rotation matrix columns (%s)', (_name, halfExtents, axis1, axis2) => {
        const { world, layers } = createTestWorld();
        const axis = vec3.normalize(vec3.create(), vec3.fromValues(0.3, -0.7, 0.5));
        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(halfExtents[0], halfExtents[1], halfExtents[2]) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(1, 2, 3),
            quaternion: quat.setAxisAngle(quat.create(), axis, 1.3),
        });
        const points: [ReturnType<typeof vec3.create>, ReturnType<typeof vec3.create>, ReturnType<typeof vec3.create>] = [
            vec3.create(),
            vec3.create(),
            vec3.create(),
        ];
        getSleepTestPoints(body, points);

        const unitAxis = (i: number) => vec3.fromValues(i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0);
        const world1 = vec3.transformQuat(vec3.create(), unitAxis(axis1), body.quaternion);
        const world2 = vec3.transformQuat(vec3.create(), unitAxis(axis2), body.quaternion);
        for (let k = 0; k < 3; k++) {
            expect(points[0][k]).toBeCloseTo(body.centerOfMassPosition[k], 12);
            expect(points[1][k]).toBeCloseTo(body.centerOfMassPosition[k] + world1[k] * halfExtents[axis1], 10);
            expect(points[2][k]).toBeCloseTo(body.centerOfMassPosition[k] + world2[k] * halfExtents[axis2], 10);
        }
    });
});
