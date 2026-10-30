import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    type ContactManifold,
    capsule,
    type Listener,
    MotionType,
    type RigidBody,
    rigidBody,
    triangleMesh,
    updateWorld,
} from '../../src';
import { getWorldSpaceContactPointOnA, getWorldSpaceContactPointOnB } from '../../src/manifold/manifold';
import { createTestWorld } from '../helpers';

describe('narrowphase manifolds', () => {
    test('a capsule lying on a box gets two contact points, the ends of its resting line', () => {
        const { world, layers } = createTestWorld();

        // a 2 m static box with its top at y = 1, and a capsule (radius 0.5, cylinder half height 0.5) turned to lie
        // along x, its bottom line 1 cm into the box's top
        const boxBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
        });
        rigidBody.create(world, {
            shape: capsule.create({ radius: 0.5, halfHeightOfCylinder: 0.5 }),
            position: vec3.fromValues(0, 1.49, 0),
            quaternion: quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const onBox: number[][] = [];
        const onCapsule: number[][] = [];
        const listener: Listener = {
            onContactAdded: (bodyA: RigidBody, _bodyB: RigidBody, manifold: ContactManifold) => {
                for (let i = 0; i < manifold.numContactPoints; i++) {
                    const a = getWorldSpaceContactPointOnA(vec3.create(), manifold, i);
                    const b = getWorldSpaceContactPointOnB(vec3.create(), manifold, i);
                    onBox.push(bodyA === boxBody ? [...a] : [...b]);
                    onCapsule.push(bodyA === boxBody ? [...b] : [...a]);
                }
            },
        };
        updateWorld(world, listener, 1 / 60);

        // the cylinder part's bottom line ends, x = -0.5 and 0.5, on the box's top face and on the capsule below it
        expect(onBox.length).toBe(2);
        const xs = onBox.map((p) => p[0]).sort((a, b) => a - b);
        expect(xs[0]).toBeCloseTo(-0.5, 3);
        expect(xs[1]).toBeCloseTo(0.5, 3);
        for (let i = 0; i < 2; i++) {
            expect(onBox[i][1]).toBeCloseTo(1, 3);
            expect(onBox[i][2]).toBeCloseTo(0, 3);
            expect(onCapsule[i][0]).toBeCloseTo(onBox[i][0], 3);
            expect(onCapsule[i][1]).toBeLessThan(1);
            expect(onCapsule[i][1]).toBeGreaterThan(0.98);
        }
    });

    test('a box resting along a capsule gets two contact points, the ends of the capsule top line', () => {
        const { world, layers } = createTestWorld();

        // a static capsule (radius 0.5, cylinder half height 0.5) lying along x with its top line at y = 0.5, and a
        // dynamic box over it whose bottom face sits 1 cm into that line. the dynamic body is shape 1, so its face is
        // the polygon and the capsule's line is face 2
        const capsuleBody = rigidBody.create(world, {
            shape: capsule.create({ radius: 0.5, halfHeightOfCylinder: 0.5 }),
            quaternion: quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
        });
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(1, 0.5, 0.5) }),
            position: vec3.fromValues(0, 0.99, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        const onCapsule: number[][] = [];
        const onBox: number[][] = [];
        const listener: Listener = {
            onContactAdded: (bodyA: RigidBody, _bodyB: RigidBody, manifold: ContactManifold) => {
                for (let i = 0; i < manifold.numContactPoints; i++) {
                    const a = getWorldSpaceContactPointOnA(vec3.create(), manifold, i);
                    const b = getWorldSpaceContactPointOnB(vec3.create(), manifold, i);
                    onCapsule.push(bodyA === capsuleBody ? [...a] : [...b]);
                    onBox.push(bodyA === capsuleBody ? [...b] : [...a]);
                }
            },
        };
        updateWorld(world, listener, 1 / 60);

        // the top line's ends, x = -0.5 and 0.5, on the capsule and on the box's bottom face just below them
        expect(onCapsule.length).toBe(2);
        const xs = onCapsule.map((p) => p[0]).sort((a, b) => a - b);
        expect(xs[0]).toBeCloseTo(-0.5, 3);
        expect(xs[1]).toBeCloseTo(0.5, 3);
        for (let i = 0; i < 2; i++) {
            expect(onCapsule[i][1]).toBeCloseTo(0.5, 3);
            expect(onCapsule[i][2]).toBeCloseTo(0, 3);
            expect(onBox[i][0]).toBeCloseTo(onCapsule[i][0], 3);
            expect(onBox[i][1]).toBeLessThan(0.5);
            expect(onBox[i][1]).toBeGreaterThan(0.48);
        }
    });

    test('a box resting on a finely tessellated mesh prunes along a unit normal', () => {
        const { world, layers } = createTestWorld();

        // a 2 m square of 0.1 m cells, 800 triangles: a 1 m box over it touches many of them, each adding its clipped
        // points to one manifold, which is pruned past 32 points before the normals are averaged
        const cells = 20;
        const cellSize = 0.1;
        const positions: number[] = [];
        const indices: number[] = [];
        for (let z = 0; z <= cells; z++) {
            for (let x = 0; x <= cells; x++) positions.push(-1 + x * cellSize, 0, -1 + z * cellSize);
        }
        for (let z = 0; z < cells; z++) {
            for (let x = 0; x < cells; x++) {
                const i = z * (cells + 1) + x;
                indices.push(i, i + cells + 1, i + 1, i + 1, i + cells + 1, i + cells + 2);
            }
        }
        rigidBody.create(world, {
            shape: triangleMesh.create({ positions, indices }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
        });
        const boxBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            position: vec3.fromValues(0, 0.49, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        let maxPoints = 0;
        const listener: Listener = {
            onContactAdded: (_bodyA, _bodyB, manifold) => {
                maxPoints = Math.max(maxPoints, manifold.numContactPoints);
            },
        };
        for (let i = 0; i < 30; i++) updateWorld(world, listener, 1 / 60);

        // pruned to a quad, and the box stays resting on the mesh
        expect(maxPoints).toBe(4);
        expect(boxBody.position[1]).toBeCloseTo(0.5, 1);
        expect(Math.abs(boxBody.position[0])).toBeLessThan(0.01);
        expect(Math.abs(boxBody.position[2])).toBeLessThan(0.01);
    });
});
