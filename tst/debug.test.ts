import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, compound, debug, hingeConstraint, MotionType, registerAll, rigidBody, scaled, sphere, transformed } from '../src';
import { createTestWorld } from './helpers';

registerAll();

function lineBoundsSize(result: debug.DebugRenderResult): [number, number, number] {
    const min = [Infinity, Infinity, Infinity];
    const max = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < result.vertices.length; i += 3) {
        for (let axis = 0; axis < 3; axis++) {
            min[axis] = Math.min(min[axis], result.vertices[i + axis]);
            max[axis] = Math.max(max[axis], result.vertices[i + axis]);
        }
    }
    return [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
}

/** the end minus the start of every line drawn in the given colour */
function lineDirectionsWithColor(result: debug.DebugRenderResult, r: number, g: number, b: number): number[][] {
    const directions: number[][] = [];
    for (let line = 0; line < result.numLines; line++) {
        const offset = line * 6;
        const colors = result.colors;
        if (colors[offset] !== r || colors[offset + 1] !== g || colors[offset + 2] !== b) continue;
        if (colors[offset + 3] !== r || colors[offset + 4] !== g || colors[offset + 5] !== b) continue;
        const vertices = result.vertices;
        directions.push([
            vertices[offset + 3] - vertices[offset],
            vertices[offset + 4] - vertices[offset + 1],
            vertices[offset + 5] - vertices[offset + 2],
        ]);
    }
    return directions;
}

describe('debug shape scale', () => {
    // a 2 x 1 x 1 box quarter-turned about z is 1 x 2 x 1 in its parent, which the parent's (2, 1, 1) scale makes 2 x 2 x 1
    const quarterTurnZ = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);
    const SCALE = vec3.fromValues(2, 1, 1);

    test('a quarter-turned compound child under a non-uniform scale draws its collision box', () => {
        const shape = scaled.create({
            shape: compound.create({
                children: [
                    {
                        shape: box.create({ halfExtents: vec3.fromValues(1, 0.5, 0.5), convexRadius: 0 }),
                        position: vec3.create(),
                        quaternion: quarterTurnZ,
                    },
                ],
            }),
            scale: SCALE,
        });

        const size = lineBoundsSize(debug.shape(shape, [0, 0, 0], [0, 0, 0, 1]));
        expect(size[0]).toBeCloseTo(2, 5);
        expect(size[1]).toBeCloseTo(2, 5);
        expect(size[2]).toBeCloseTo(1, 5);
    });

    test('a quarter-turned transformed shape under a non-uniform scale draws its collision box', () => {
        const shape = scaled.create({
            shape: transformed.create({
                shape: box.create({ halfExtents: vec3.fromValues(1, 0.5, 0.5), convexRadius: 0 }),
                position: vec3.create(),
                quaternion: quarterTurnZ,
            }),
            scale: SCALE,
        });

        const size = lineBoundsSize(debug.shape(shape, [0, 0, 0], [0, 0, 0, 1]));
        expect(size[0]).toBeCloseTo(2, 5);
        expect(size[1]).toBeCloseTo(2, 5);
        expect(size[2]).toBeCloseTo(1, 5);
    });
});

describe('debug hinge', () => {
    // a hinge about z through b's centre of mass, normal axes along x. jolt draws b's hinge axis in green and b's
    // normal axis in white, both under b's transform
    test("b's hinge axis is drawn and b's normal line turns with b", () => {
        const { world, layers } = createTestWorld();
        const bodyA = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.25 }),
            position: vec3.fromValues(0, 1, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        const bodyB = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.25 }),
            position: vec3.fromValues(0, 0, 0),
            motionType: MotionType.DYNAMIC,
            objectLayer: layers.OBJECT_LAYER_MOVING,
        });
        hingeConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            pointA: vec3.fromValues(0, 0, 0),
            pointB: vec3.fromValues(0, 0, 0),
            hingeAxisA: vec3.fromValues(0, 0, 1),
            hingeAxisB: vec3.fromValues(0, 0, 1),
            normalAxisA: vec3.fromValues(1, 0, 0),
            normalAxisB: vec3.fromValues(1, 0, 0),
        });

        const size = 0.5;
        const before = debug.joints(world, { size });
        const greenBefore = lineDirectionsWithColor(before, 0, 1, 0);
        expect(greenBefore.some(([x, y, z]) => Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6 && Math.abs(z - size) < 1e-6)).toBe(true);
        const whiteBefore = lineDirectionsWithColor(before, 1, 1, 1);
        expect(whiteBefore.length).toBe(1);
        expect(whiteBefore[0][0]).toBeCloseTo(size, 5);
        expect(whiteBefore[0][1]).toBeCloseTo(0, 5);

        // a quarter turn of b about the hinge axis turns its normal from x to y
        rigidBody.setQuaternion(world, bodyB, quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2), false);

        const whiteAfter = lineDirectionsWithColor(debug.joints(world, { size }), 1, 1, 1);
        expect(whiteAfter.length).toBe(1);
        expect(whiteAfter[0][0]).toBeCloseTo(0, 5);
        expect(whiteAfter[0][1]).toBeCloseTo(size, 5);
    });
});
