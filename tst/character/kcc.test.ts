import { quat, vec3, vec4 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { box, capsule, filter, kcc, MotionType, registerAll, rigidBody, updateWorld } from '../../src';
import { createTestWorld } from '../helpers';

registerAll();

const DELTA_TIME = 1 / 60;

describe('KCC steep slope escape', () => {
    /**
     * reproduces the scenario from jolt PR #1887:
     * character starts inside two steep slopes (steeper than maxSlopeAngle).
     * with the bug, penetration recovery velocity leaked into vertical wall constraints,
     * trapping the character. with the fix, the character escapes upward.
     *
     * matches jolt's TestInitiallyIntersecting2:
     * - floor at y=-0.5
     * - box at (-0.5, 0.5, 0) half extents (0.5, 0.5, 0.5)
     * - two slopes: boxes with half extents (1, 0.1, 1) rotated ±75° around Z at origin
     * - capsule character (half height 0.5, radius 0.3) at origin
     */
    test('character should escape upward when surrounded by steep slopes', () => {
        const { world, layers } = createTestWorld();

        // create a floor
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) }),
            position: vec3.fromValues(0, -0.5, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });

        // create a box that is intersecting with the character (matches jolt test)
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            position: vec3.fromValues(-0.5, 0.5, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });

        // create two very steep slopes (75 degrees) that initially intersect with the character
        // this matches the jolt test: two boxes rotated ±75° around Z axis
        const slopeShape = box.create({ halfExtents: vec3.fromValues(1, 0.1, 1) });
        const slopeAngle = (75 * Math.PI) / 180;

        rigidBody.create(world, {
            shape: slopeShape,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), slopeAngle),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });

        rigidBody.create(world, {
            shape: slopeShape,
            position: vec3.fromValues(0, 0, 0),
            quaternion: quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), -slopeAngle),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });

        // step the world once so broadphase is ready
        updateWorld(world, undefined, DELTA_TIME);

        // create character at origin (initially intersecting with the slopes)
        const characterShape = capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
        const character = kcc.create(
            {
                shape: characterShape,
                mass: 70,
                maxSlopeAngle: (50 * Math.PI) / 180, // 50 degrees — slopes at 75° are too steep
                penetrationRecoverySpeed: 1.0,
                up: vec3.fromValues(0, 1, 0),
                supportingVolumePlane: vec4.fromValues(0, 1, 0, -1e10),
            },
            vec3.fromValues(0, 0, 0),
            quat.create(),
        );

        const characterFilter = filter.create(world.settings.layers);
        const updateSettings = kcc.createDefaultUpdateSettings();
        // disable stair walking and stick to floor for this test
        vec3.zero(updateSettings.stickToFloorStepDown);
        vec3.zero(updateSettings.walkStairsStepUp);

        const gravity = vec3.fromValues(0, -10, 0);

        // step the character several times and verify it moves upward (not stuck)
        let y = character.position[1];
        for (let step = 0; step < 10; step++) {
            // apply gravity to character velocity (like jolt's test harness)
            vec3.scaleAndAdd(character.linearVelocity, character.linearVelocity, gravity, DELTA_TIME);

            kcc.update(world, character, DELTA_TIME, gravity, updateSettings, undefined, characterFilter);
            updateWorld(world, undefined, DELTA_TIME);

            y = character.position[1];
        }

        // after 10 steps, the character should have escaped upward and not be stuck at y=0
        // this is the core assertion for PR #1887: without the fix, y stays at 0
        expect(y).toBeGreaterThan(0.05);
        // character should stay roughly in the vicinity (not launched to infinity)
        // note: the asymmetric box at (-0.5, 0.5, 0) causes some lateral drift, which is expected
        expect(Math.abs(character.position[0])).toBeLessThan(2.0);
        expect(Math.abs(character.position[2])).toBeLessThan(2.0);
    });
});
