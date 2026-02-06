import { type Box3, mat4, quat, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
    box,
    compound,
    MotionType,
    massProperties,
    motionProperties,
    rigidBody,
    type Shape,
    ShapeType,
    sphere,
    subShape,
    triangleMesh,
} from '../../src';
import { createTestWorld } from '../helpers';

describe('Body - Motion Properties', () => {
    test('should initialize with zero velocities', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        expect(vec3.length(body.motionProperties.linearVelocity)).toBe(0);
        expect(vec3.length(body.motionProperties.angularVelocity)).toBe(0);
    });

    test('should initialize with zero forces', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        expect(vec3.length(body.motionProperties.force)).toBe(0);
        expect(vec3.length(body.motionProperties.torque)).toBe(0);
    });

    test('should have valid inverse mass for dynamic body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        expect(body.motionProperties.invMass).toBeGreaterThan(0);
    });

    test('should have zero inverse mass for static body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.STATIC });

        expect(body.motionProperties.invMass).toBe(0);
    });
});

describe('Body - Force Application', () => {
    test('addForce should accumulate forces on dynamic body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        const force = vec3.fromValues(10, 0, 0);
        rigidBody.addForce(world, body, force, true);

        expect(body.motionProperties.force[0]).toBeCloseTo(10);
        expect(body.motionProperties.force[1]).toBeCloseTo(0);
        expect(body.motionProperties.force[2]).toBeCloseTo(0);
    });

    test('addForce should accumulate multiple forces', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        rigidBody.addForce(world, body, vec3.fromValues(5, 0, 0), true);
        rigidBody.addForce(world, body, vec3.fromValues(3, 0, 0), true);

        expect(body.motionProperties.force[0]).toBeCloseTo(8);
    });

    test('addForce should be ignored on static body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.STATIC });

        const force = vec3.fromValues(10, 0, 0);
        rigidBody.addForce(world, body, force, true);

        expect(vec3.length(body.motionProperties.force)).toBe(0);
    });

    test('addForce should be ignored on kinematic body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.KINEMATIC,
        });

        const force = vec3.fromValues(10, 0, 0);
        rigidBody.addForce(world, body, force, true);

        expect(vec3.length(body.motionProperties.force)).toBe(0);
    });
});

describe('Body - Impulse Application', () => {
    test('addImpulse should change linear velocity', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        const impulse = vec3.fromValues(1, 0, 0);
        rigidBody.addImpulse(world, body, impulse);

        // Δv = impulse * invMass
        expect(body.motionProperties.linearVelocity[0]).toBeGreaterThan(0);
        expect(body.motionProperties.linearVelocity[1]).toBeCloseTo(0);
        expect(body.motionProperties.linearVelocity[2]).toBeCloseTo(0);
    });

    test('addImpulse should be ignored on static body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.STATIC });

        const impulse = vec3.fromValues(1, 0, 0);
        rigidBody.addImpulse(world, body, impulse);

        expect(vec3.length(body.motionProperties.linearVelocity)).toBe(0);
    });

    test('addAngularImpulse should change angular velocity', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        const angularImpulse = vec3.fromValues(0.1, 0, 0);
        rigidBody.addAngularImpulse(world, body, angularImpulse);

        expect(vec3.length(body.motionProperties.angularVelocity)).toBeGreaterThan(0);
    });

    test('addAngularImpulse should be ignored on static body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.STATIC });

        const angularImpulse = vec3.fromValues(0.1, 0, 0);
        rigidBody.addAngularImpulse(world, body, angularImpulse);

        expect(vec3.length(body.motionProperties.angularVelocity)).toBe(0);
    });

    test('addImpulseAtPosition should apply both linear and angular impulse', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        const impulse = vec3.fromValues(1, 0, 0);
        const position = vec3.fromValues(0, 1, 0); // Above center of mass

        rigidBody.addImpulseAtPosition(world, body, impulse, position);

        // Should have linear velocity
        expect(body.motionProperties.linearVelocity[0]).toBeGreaterThan(0);

        // Should have angular velocity (torque = r × impulse)
        expect(vec3.length(body.motionProperties.angularVelocity)).toBeGreaterThan(0);
    });
});

describe('Body - Velocity Clamping', () => {
    test('should clamp linear velocity to max', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            maxLinearVelocity: 10.0,
        });

        // Apply a very large impulse
        const largeImpulse = vec3.fromValues(1000, 0, 0);
        rigidBody.addImpulse(world, body, largeImpulse);

        // Velocity should not exceed maxLinearVelocity
        const speed = vec3.length(body.motionProperties.linearVelocity);
        expect(speed).toBeLessThanOrEqual(body.motionProperties.maxLinearVelocity + 1e-6);
    });

    test('should clamp angular velocity to max', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            maxAngularVelocity: 1.0,
        });

        // Apply a very large angular impulse
        const largeImpulse = vec3.fromValues(100, 0, 0);
        rigidBody.addAngularImpulse(world, body, largeImpulse);

        // Angular velocity should not exceed maxAngularVelocity
        const angularSpeed = vec3.length(body.motionProperties.angularVelocity);
        expect(angularSpeed).toBeLessThanOrEqual(body.motionProperties.maxAngularVelocity + 1e-6);
    });
});

describe('Body - Sleep Management', () => {
    test('should sleep and wake body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        expect(body.sleeping).toBe(false);

        rigidBody.sleep(world, body);
        expect(body.sleeping).toBe(true);

        rigidBody.wake(world, body);
        expect(body.sleeping).toBe(false);
    });

    test('should not sleep static body', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.STATIC });

        rigidBody.sleep(world, body);
        expect(body.sleeping).toBe(false);
    });
});

describe('Body - Mass Properties Override', () => {
    test('should use computed mass properties when no override provided', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        // Sphere mass = (4/3) * π * r^3 * density = (4/3) * π * 1 * 1000
        const expectedMass = (4.0 / 3.0) * Math.PI * 1000;
        const actualMass = 1 / body.motionProperties.invMass;
        expect(actualMass).toBeCloseTo(expectedMass, 1);
    });

    test('should use override mass properties when provided', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0, density: 1000 });
        const override = massProperties.create();
        override.mass = 500;
        mat4.identity(override.inertia);
        mat4.scale(override.inertia, override.inertia, vec3.fromValues(100, 100, 100));

        const body = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            massPropertiesOverride: override,
        });

        // Should use override mass instead of computed mass
        const actualMass = 1 / body.motionProperties.invMass;
        expect(actualMass).toBeCloseTo(500, 5);
    });

    test('should apply override to triangle mesh (static by default)', () => {
        const { world, layers } = createTestWorld();
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        const body = rigidBody.create(world, {
            shape: mesh,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        // Triangle meshes default to static (zero mass)
        expect(body.motionProperties.invMass).toBe(0);
    });

    test('should allow dynamic triangle mesh with custom mass override', () => {
        const { world, layers } = createTestWorld();
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        const override = massProperties.create();
        override.mass = 200;
        mat4.identity(override.inertia);

        const body = rigidBody.create(world, {
            shape: mesh,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            massPropertiesOverride: override,
        });

        expect(body.motionProperties.invMass).toBeCloseTo(1 / 200, 5);
    });

    // TODO: test expectations right?
    // test('should preserve override inertia tensor', () => {
    //     const shape = sphere.create({ radius: 1.0 });
    //     const override = massProperties.create();
    //     override.mass = 100;

    //     // Set custom inertia (diagonal matrix)
    //     mat4.identity(override.inertia);
    //     override.inertia[0] = 50; // Ixx
    //     override.inertia[5] = 60; // Iyy
    //     override.inertia[10] = 70; // Izz

    //     const body = rigidBody.create({
    //         shape,
    //         objectLayer: layers.OBJECT_LAYER_MOVING,
    //         motionType: MotionType.DYNAMIC,
    //         massPropertiesOverride: override,
    //     });

    //     // Mass should be preserved
    //     expect(body.motionProperties.invMass).toBeCloseTo(1 / 100, 5);
    //     // Inertia values should be preserved in motion properties
    //     expect(body.motionProperties.invInertiaDiagonal[0]).toBeCloseTo(1 / 50, 5);
    //     expect(body.motionProperties.invInertiaDiagonal[1]).toBeCloseTo(1 / 60, 5);
    //     expect(body.motionProperties.invInertiaDiagonal[2]).toBeCloseTo(1 / 70, 5);
    // });
});

describe('Body - World Space Surface Normal', () => {
    test('should compute sphere surface normal at various world positions', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        const worldPos = vec3.fromValues(1, 0, 0); // Point on sphere surface
        const normal = vec3.create();
        rigidBody.getSurfaceNormal(normal, body, worldPos, subShape.EMPTY_SUB_SHAPE_ID);

        // For a sphere, normal at surface point should point outward from center
        // At (1, 0, 0), normal should be (1, 0, 0)
        expect(normal[0]).toBeCloseTo(1, 5);
        expect(normal[1]).toBeCloseTo(0, 5);
        expect(normal[2]).toBeCloseTo(0, 5);
        expect(vec3.length(normal)).toBeCloseTo(1, 5);
    });

    test('should compute sphere normal with offset body position', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(5, 10, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        // World position: body center + (1, 0, 0) in local space
        const worldPos = vec3.fromValues(6, 10, 0);
        const normal = vec3.create();
        rigidBody.getSurfaceNormal(normal, body, worldPos, subShape.EMPTY_SUB_SHAPE_ID);

        // Normal should point along (1, 0, 0) in world space
        expect(normal[0]).toBeCloseTo(1, 5);
        expect(normal[1]).toBeCloseTo(0, 5);
        expect(normal[2]).toBeCloseTo(0, 5);
    });

    test('should apply body rotation to normal', () => {
        const { world, layers } = createTestWorld();
        const shape = sphere.create({ radius: 1.0 });
        // Rotate 90 degrees around Z axis (0, 0, 1)
        const rotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);

        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            quaternion: rotation,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        // World position: in local space (1, 0, 0), but rotated 90 degrees
        // After rotation: (0, 1, 0) in world space
        const worldPos = vec3.fromValues(0, 1, 0);
        const normal = vec3.create();
        rigidBody.getSurfaceNormal(normal, body, worldPos, subShape.EMPTY_SUB_SHAPE_ID);

        // Normal should be (0, 1, 0) in world space after rotation
        expect(normal[0]).toBeCloseTo(0, 5);
        expect(normal[1]).toBeCloseTo(1, 5);
        expect(normal[2]).toBeCloseTo(0, 5);
    });

    test('should compute box surface normal', () => {
        const { world, layers } = createTestWorld();
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        // Point on top face of box
        const worldPos = vec3.fromValues(0, 1, 0);
        const normal = vec3.create();
        rigidBody.getSurfaceNormal(normal, body, worldPos, subShape.EMPTY_SUB_SHAPE_ID);

        // Normal on top face should point up (0, 1, 0)
        expect(normal[1]).toBeCloseTo(1, 5);
        expect(vec3.length(normal)).toBeCloseTo(1, 5);
    });

    test('should compute triangle mesh surface normal with subShapeId', () => {
        const { world, layers } = createTestWorld();

        const mesh = triangleMesh.create({
            positions: [
                0,
                0,
                0, // v0: origin
                1,
                0,
                0, // v1: X axis
                0,
                1,
                0, // v2: Y axis (shared)
                0,
                0,
                1, // v3: Z axis
            ],
            indices: [
                0,
                1,
                2, // Triangle 0: XY plane, normal +Z (cross product of edge1 and edge2)
                0,
                2,
                3, // Triangle 1: YZ plane, normal +X (cross product of edge1 and edge2)
            ],
        });

        const body = rigidBody.create(world, {
            shape: mesh,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        // Test triangle 0 (XY plane) - vertices [0,1,2] = [(0,0,0), (1,0,0), (0,1,0)], normal +Z
        const pos0 = vec3.fromValues(0.25, 0.25, 0);
        const normal0 = vec3.create();
        rigidBody.getSurfaceNormal(normal0, body, pos0, 0);

        expect(normal0[0]).toBeCloseTo(0, 5);
        expect(normal0[1]).toBeCloseTo(0, 5);
        expect(normal0[2]).toBeCloseTo(1, 5);

        // Test triangle 1 (YZ plane) - vertices [0,2,3] = [(0,0,0), (0,1,0), (0,0,1)], normal +X
        const pos1 = vec3.fromValues(0, 0.25, 0.25);
        const normal1 = vec3.create();
        rigidBody.getSurfaceNormal(normal1, body, pos1, 1);

        expect(normal1[0]).toBeCloseTo(1, 5);
        expect(normal1[1]).toBeCloseTo(0, 5);
        expect(normal1[2]).toBeCloseTo(0, 5);
    });

    test('should apply both position and rotation transforms', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        // Translate to (10, 20, 0) and rotate 90 degrees around Z
        const rotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);

        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(10, 20, 0),
            quaternion: rotation,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        // World position: (10, 21, 0) = body center + rotated(1, 0, 0)
        const worldPos = vec3.fromValues(10, 21, 0);
        const normal = vec3.create();
        rigidBody.getSurfaceNormal(normal, body, worldPos, subShape.EMPTY_SUB_SHAPE_ID);

        // Normal should be (0, 1, 0) after rotation
        expect(normal[0]).toBeCloseTo(0, 5);
        expect(normal[1]).toBeCloseTo(1, 5);
        expect(normal[2]).toBeCloseTo(0, 5);
    });

    test('should return normalized normal vector', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 2.0 });
        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        const worldPos = vec3.fromValues(2, 0, 0);
        const normal = vec3.create();
        rigidBody.getSurfaceNormal(normal, body, worldPos, subShape.EMPTY_SUB_SHAPE_ID);

        // Normal should always be normalized (length = 1)
        expect(vec3.length(normal)).toBeCloseTo(1, 5);
    });

    test('should match the return value from getBodySurfaceNormal', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
        });

        const worldPos = vec3.fromValues(1, 0, 0);
        const normal = vec3.create();
        const returned = rigidBody.getSurfaceNormal(normal, body, worldPos, subShape.EMPTY_SUB_SHAPE_ID);

        // Function should return the same vector as the output
        expect(returned).toBe(normal);
    });
});

describe('Body - Center of Mass Position', () => {
    test('should initialize centerOfMassPosition for sphere at body position', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const bodyPos = vec3.fromValues(5, 10, 15);
        const body = rigidBody.create(world, {
            shape,
            position: bodyPos,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Sphere is centered, so COM should equal position
        expect(body.centerOfMassPosition[0]).toBeCloseTo(5, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(10, 5);
        expect(body.centerOfMassPosition[2]).toBeCloseTo(15, 5);
    });

    test('should initialize centerOfMassPosition for box at body position', () => {
        const { world, layers } = createTestWorld();

        const shape = box.create({ halfExtents: vec3.fromValues(1, 2, 1) });
        const bodyPos = vec3.fromValues(0, 5, 0);
        const body = rigidBody.create(world, {
            shape,
            position: bodyPos,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Box is centered, so COM should equal position
        expect(body.centerOfMassPosition[0]).toBeCloseTo(0, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(5, 5);
        expect(body.centerOfMassPosition[2]).toBeCloseTo(0, 5);
    });

    test('should compute centerOfMassPosition with compound shape offset COM', () => {
        const { world, layers } = createTestWorld();

        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });
        const shape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(4, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        const bodyPos = vec3.fromValues(10, 0, 0);
        const body = rigidBody.create(world, {
            shape,
            position: bodyPos,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Compound COM is at (2, 0, 0) in shape-local space
        // World COM should be bodyPos + shapeLocalCOM = (10, 0, 0) + (2, 0, 0) = (12, 0, 0)
        expect(body.centerOfMassPosition[0]).toBeCloseTo(12, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(0, 5);
        expect(body.centerOfMassPosition[2]).toBeCloseTo(0, 5);
    });

    test('should apply rotation to centerOfMassPosition', () => {
        const { world, layers } = createTestWorld();

        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });
        const shape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(2, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        // Rotate 90 degrees around Z axis
        const rotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);

        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            quaternion: rotation,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Shape-local COM is (1, 0, 0)
        // After 90-degree rotation around Z: (0, 1, 0)
        // World COM = (0, 0, 0) + (0, 1, 0) = (0, 1, 0)
        expect(body.centerOfMassPosition[0]).toBeCloseTo(0, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(1, 5);
        expect(body.centerOfMassPosition[2]).toBeCloseTo(0, 5);
    });

    test('should compute centerOfMassPosition with both position and rotation', () => {
        const { world, layers } = createTestWorld();

        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });
        const shape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(2, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        const rotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);

        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(5, 10, 0),
            quaternion: rotation,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Shape-local COM is (1, 0, 0)
        // After 90-degree rotation: (0, 1, 0)
        // World COM = (5, 10, 0) + (0, 1, 0) = (5, 11, 0)
        expect(body.centerOfMassPosition[0]).toBeCloseTo(5, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(11, 5);
        expect(body.centerOfMassPosition[2]).toBeCloseTo(0, 5);
    });

    test('should use centerOfMassPosition for torque calculation in addForceAtPosition', () => {
        const { world, layers } = createTestWorld();

        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });
        const shape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(0, 4, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Compound shape has COM at (0, 2, 0)
        expect(body.centerOfMassPosition[1]).toBeCloseTo(2, 5);

        // Apply force at shape origin (0, 0, 0)
        // Moment arm = (0, 0, 0) - (0, 2, 0) = (0, -2, 0)
        // Force = (1, 0, 0)
        // Torque = (0, -2, 0) × (1, 0, 0) = (0, 0, 2)
        const force = vec3.fromValues(1, 0, 0);
        const forcePosition = vec3.fromValues(0, 0, 0);
        rigidBody.addForceAtPosition(world, body, force, forcePosition, true);

        expect(body.motionProperties.torque[0]).toBeCloseTo(0, 5);
        expect(body.motionProperties.torque[1]).toBeCloseTo(0, 5);
        expect(body.motionProperties.torque[2]).toBeCloseTo(2, 5);
    });

    test('should use centerOfMassPosition for torque calculation in addImpulseAtPosition', () => {
        const { world, layers } = createTestWorld();

        const s1 = sphere.create({ radius: 1.0, density: 1000 });
        const s2 = sphere.create({ radius: 1.0, density: 1000 });
        const shape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(0, 4, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Clear any accumulated motion from initialization
        vec3.zero(body.motionProperties.linearVelocity);
        vec3.zero(body.motionProperties.angularVelocity);

        // Compound shape has COM at (0, 2, 0)
        // Apply impulse at shape origin (0, 0, 0)
        const impulse = vec3.fromValues(1, 0, 0);
        const impulsePosition = vec3.fromValues(0, 0, 0);
        rigidBody.addImpulseAtPosition(world, body, impulse, impulsePosition);

        // Should have linear velocity from impulse
        expect(body.motionProperties.linearVelocity[0]).toBeGreaterThan(0);

        // Should have angular velocity from moment arm
        expect(vec3.length(body.motionProperties.angularVelocity)).toBeGreaterThan(0);
    });

    test('should update centerOfMassPosition when called explicitly', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Initial COM
        const initialCOM = vec3.clone(body.centerOfMassPosition);
        expect(initialCOM[0]).toBeCloseTo(0, 5);

        // Manually update position
        vec3.set(body.position, 5, 10, 15);

        // Call updateCenterOfMassPosition
        rigidBody.updateCenterOfMassPosition(body);

        // COM should match new position
        expect(body.centerOfMassPosition[0]).toBeCloseTo(5, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(10, 5);
        expect(body.centerOfMassPosition[2]).toBeCloseTo(15, 5);
    });

    test('should update centerOfMassPosition when rotation changes', () => {
        const { world, layers } = createTestWorld();

        const s1 = sphere.create({ radius: 1.0 });
        const s2 = sphere.create({ radius: 1.0 });
        const shape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: s1 },
                { position: vec3.fromValues(2, 0, 0), quaternion: quat.create(), shape: s2 },
            ],
        });

        const body = rigidBody.create(world, {
            shape,
            position: vec3.fromValues(0, 0, 0),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
        });

        // Initial COM at (1, 0, 0)
        expect(body.centerOfMassPosition[0]).toBeCloseTo(1, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(0, 5);

        // Rotate 90 degrees around Z
        const rotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);
        quat.copy(body.quaternion, rotation);

        // Update COM
        rigidBody.updateCenterOfMassPosition(body);

        // After rotation, COM should be at (0, 1, 0)
        expect(body.centerOfMassPosition[0]).toBeCloseTo(0, 5);
        expect(body.centerOfMassPosition[1]).toBeCloseTo(1, 5);
        expect(body.centerOfMassPosition[2]).toBeCloseTo(0, 5);
    });
});

describe('Body Shape Navigation', () => {
    test('should navigate to leaf shapes correctly', () => {
        const leafShape = sphere.create({ radius: 1 });

        const result = { shape: null as Shape | null, remainder: subShape.EMPTY_SUB_SHAPE_ID };
        rigidBody.getLeafShape(result, leafShape, subShape.EMPTY_SUB_SHAPE_ID);

        expect(result.shape).toBe(leafShape);
        expect(result.remainder).toBe(subShape.EMPTY_SUB_SHAPE_ID);
    });

    test('should navigate through nested compound shapes to find leaf', () => {
        // Create inner compound with 2 shapes (needs 1 bit)
        const innerCompound = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 0.5 }) },
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
                },
            ],
        });

        // Create outer compound with 3 shapes including the inner compound (needs 2 bits)
        const outerCompound = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
                { position: vec3.create(), quaternion: quat.create(), shape: innerCompound },
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                },
            ],
        });

        // Build SubShapeID: outer index 1 (inner compound), inner index 0 (sphere)
        const builder1 = subShape.builder();
        const builder2 = subShape.builder();

        // Push outer compound child index 1 (the inner compound)
        subShape.pushIndex(builder1, builder2, 1, outerCompound.children.length);
        // Push inner compound child index 0 (the sphere)
        subShape.pushIndex(builder2, builder1, 0, innerCompound.children.length);

        // Navigate to the nested leaf shape
        const leafResult = rigidBody.createGetLeafShapeResult();
        rigidBody.getLeafShape(leafResult, outerCompound, builder2.value);

        // Should find the sphere inside the inner compound
        expect(leafResult.shape?.type).toBe(ShapeType.SPHERE);
        expect((leafResult.shape as any)?.radius).toBe(0.5);
    });

    test('should handle empty SubShapeID for compound shapes', () => {
        const compoundShape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
                {
                    position: vec3.create(),
                    quaternion: quat.create(),
                    shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                },
            ],
        });

        const result = rigidBody.createGetLeafShapeResult();
        rigidBody.getLeafShape(result, compoundShape, subShape.EMPTY_SUB_SHAPE_ID);

        // Should return null for compound with empty SubShapeID
        expect(result.shape).toBe(null);
        expect(result.remainder).toBe(subShape.EMPTY_SUB_SHAPE_ID);
    });
});

describe('Body - Velocity Queries', () => {
    test('getPointVelocityCOM should return zero for static body', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.STATIC });

        const point = vec3.fromValues(1, 0, 0);
        const velocity = vec3.create();
        rigidBody.getVelocityAtPointCOM(velocity, body, point);

        expect(vec3.length(velocity)).toBe(0);
    });

    test('getPointVelocityCOM should return linear velocity for point at center of mass', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        // Set linear velocity
        const linearVel = vec3.fromValues(10, 0, 0);
        motionProperties.setLinearVelocity(body.motionProperties, linearVel);

        // Point at center of mass (r = 0)
        const point = vec3.fromValues(0, 0, 0);
        const velocity = vec3.create();
        rigidBody.getVelocityAtPointCOM(velocity, body, point);

        // Should equal linear velocity (no angular contribution)
        expect(velocity[0]).toBeCloseTo(10);
        expect(velocity[1]).toBeCloseTo(0);
        expect(velocity[2]).toBeCloseTo(0);
    });

    test('getPointVelocityCOM should include angular velocity contribution', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        // Set angular velocity (rotating around Z axis)
        const angularVel = vec3.fromValues(0, 0, 1);
        motionProperties.setAngularVelocity(body.motionProperties, angularVel);

        // Point at (1, 0, 0) relative to COM
        const point = vec3.fromValues(1, 0, 0);
        const velocity = vec3.create();
        rigidBody.getVelocityAtPointCOM(velocity, body, point);

        // ω × r = (0, 0, 1) × (1, 0, 0) = (0, 1, 0)
        expect(velocity[0]).toBeCloseTo(0);
        expect(velocity[1]).toBeCloseTo(1);
        expect(velocity[2]).toBeCloseTo(0);
    });

    test('getPointVelocityCOM should combine linear and angular velocities', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, { shape, objectLayer: layers.OBJECT_LAYER_MOVING, motionType: MotionType.DYNAMIC });

        // Set both linear and angular velocity
        const linearVel = vec3.fromValues(5, 0, 0);
        const angularVel = vec3.fromValues(0, 0, 2);
        motionProperties.setLinearVelocity(body.motionProperties, linearVel);
        motionProperties.setAngularVelocity(body.motionProperties, angularVel);

        // Point at (0, 1, 0) relative to COM
        const point = vec3.fromValues(0, 1, 0);
        const velocity = vec3.create();
        rigidBody.getVelocityAtPointCOM(velocity, body, point);

        // v = v_linear + ω × r
        // v = (5, 0, 0) + (0, 0, 2) × (0, 1, 0)
        // ω × r = (-2, 0, 0)
        // v = (5, 0, 0) + (-2, 0, 0) = (3, 0, 0)
        expect(velocity[0]).toBeCloseTo(3);
        expect(velocity[1]).toBeCloseTo(0);
        expect(velocity[2]).toBeCloseTo(0);
    });

    test('getPointVelocity should convert world space to COM-relative', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(5, 0, 0),
        });

        // Update center of mass position
        rigidBody.updateCenterOfMassPosition(body);

        // Set angular velocity
        const angularVel = vec3.fromValues(0, 0, 1);
        motionProperties.setAngularVelocity(body.motionProperties, angularVel);

        // World point at (6, 0, 0) -> relative to COM at (5, 0, 0) = (1, 0, 0)
        const worldPoint = vec3.fromValues(6, 0, 0);
        const velocity = vec3.create();
        rigidBody.getVelocityAtPoint(velocity, body, worldPoint);

        // ω × r = (0, 0, 1) × (1, 0, 0) = (0, 1, 0)
        expect(velocity[0]).toBeCloseTo(0);
        expect(velocity[1]).toBeCloseTo(1);
        expect(velocity[2]).toBeCloseTo(0);
    });

    test('getPointVelocity should return zero for static body regardless of world position', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(10, 5, -3),
        });

        const worldPoint = vec3.fromValues(15, 10, 2);
        const velocity = vec3.create();
        rigidBody.getVelocityAtPoint(velocity, body, worldPoint);

        expect(vec3.length(velocity)).toBe(0);
    });

    test('getPointVelocity should work with offset center of mass', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 1.0 });
        const body = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // Manually set center of mass offset (normally computed from shape)
        vec3.set(body.centerOfMassPosition, 2, 0, 0);

        // Set linear velocity
        const linearVel = vec3.fromValues(0, 10, 0);
        motionProperties.setLinearVelocity(body.motionProperties, linearVel);

        // World point at (2, 0, 0) is exactly at COM
        const worldPoint = vec3.fromValues(2, 0, 0);
        const velocity = vec3.create();
        rigidBody.getVelocityAtPoint(velocity, body, worldPoint);

        // Should get pure linear velocity (no angular contribution at COM)
        expect(velocity[0]).toBeCloseTo(0);
        expect(velocity[1]).toBeCloseTo(10);
        expect(velocity[2]).toBeCloseTo(0);
    });
});

describe('Body - wakeInAABB', () => {
    test('should wake sleeping bodies within AABB', () => {
        const { world, layers } = createTestWorld();

        // create three dynamic bodies
        const body1 = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const body2 = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(5, 0, 0),
        });

        const body3 = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(10, 0, 0),
        });

        // put all bodies to sleep
        rigidBody.sleep(world, body1);
        rigidBody.sleep(world, body2);
        rigidBody.sleep(world, body3);

        expect(body1.sleeping).toBe(true);
        expect(body2.sleeping).toBe(true);
        expect(body3.sleeping).toBe(true);

        // define AABB that covers body1 and body2 but not body3
        const aabb: Box3 = [[-2, -2, -2], [7, 2, 2]];

        // wake bodies in AABB
        rigidBody.wakeInAABB(world, aabb);

        // body1 and body2 should be awake, body3 should still be asleep
        expect(body1.sleeping).toBe(false);
        expect(body2.sleeping).toBe(false);
        expect(body3.sleeping).toBe(true);
    });

    test('should not wake static bodies', () => {
        const { world, layers } = createTestWorld();

        const staticBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const dynamicBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(2, 0, 0),
        });

        rigidBody.sleep(world, dynamicBody);

        const aabb: Box3 = [[-2, -2, -2], [4, 2, 2]];

        rigidBody.wakeInAABB(world, aabb);

        // static body should not be affected (sleeping remains false, static bodies don't sleep)
        expect(staticBody.sleeping).toBe(false);
        // dynamic body should wake up
        expect(dynamicBody.sleeping).toBe(false);
    });

    test('should wake kinematic bodies', () => {
        const { world, layers } = createTestWorld();

        const kinematicBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.KINEMATIC,
            position: vec3.fromValues(0, 0, 0),
        });

        rigidBody.sleep(world, kinematicBody);
        expect(kinematicBody.sleeping).toBe(true);

        const aabb: Box3 = [[-2, -2, -2], [2, 2, 2]];

        rigidBody.wakeInAABB(world, aabb);

        expect(kinematicBody.sleeping).toBe(false);
    });



    test('should handle empty AABB region', () => {
        const { world, layers } = createTestWorld();

        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        rigidBody.sleep(world, body);

        // AABB that doesn't contain any bodies
        const aabb: Box3 = [[100, 100, 100], [110, 110, 110]];

        rigidBody.wakeInAABB(world, aabb);

        // body should remain asleep
        expect(body.sleeping).toBe(true);
    });

    test('should handle bodies that are already awake', () => {
        const { world, layers } = createTestWorld();

        const sleepingBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        const awakeBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(2, 0, 0),
        });

        rigidBody.sleep(world, sleepingBody);
        // awakeBody is already awake (default state)

        const aabb: Box3 = [[-2, -2, -2], [4, 2, 2]];

        rigidBody.wakeInAABB(world, aabb);

        // both should be awake
        expect(sleepingBody.sleeping).toBe(false);
        expect(awakeBody.sleeping).toBe(false);
    });
});
