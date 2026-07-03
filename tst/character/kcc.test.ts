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

describe('KCC conflicting contacts', () => {
    /**
     * a character wedged between two DIFFERENT static bodies with opposing contact
     * normals must keep both contacts. removeConflictingContacts only discards
     * opposing-normal pairs on the SAME body — that algorithm targets one body's
     * internal-edge double hits, not a real two-body wedge.
     */
    test('character wedged between two bodies keeps both contacts', () => {
        const { world, layers } = createTestWorld();

        // floor
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) }),
            position: vec3.fromValues(0, -0.5, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });

        // two walls forming a corridor slightly narrower than the capsule:
        // capsule radius 0.3, wall inner faces at x = ±0.25 → 0.05 penetration per
        // side, beyond the 1.25 * characterPadding (0.02) conflict threshold
        const wallShape = box.create({ halfExtents: vec3.fromValues(0.5, 1, 1) });
        const wallA = rigidBody.create(world, {
            shape: wallShape,
            position: vec3.fromValues(-0.75, 0.5, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        const wallB = rigidBody.create(world, {
            shape: wallShape,
            position: vec3.fromValues(0.75, 0.5, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });

        updateWorld(world, undefined, DELTA_TIME);

        const character = kcc.create(
            {
                shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
                mass: 70,
                up: vec3.fromValues(0, 1, 0),
                supportingVolumePlane: vec4.fromValues(0, 1, 0, -1e10),
            },
            vec3.fromValues(0, 0, 0),
            quat.create(),
        );

        const characterFilter = filter.create(world.settings.layers);
        const updateSettings = kcc.createDefaultUpdateSettings();
        vec3.zero(updateSettings.stickToFloorStepDown);
        vec3.zero(updateSettings.walkStairsStepUp);

        const gravity = vec3.fromValues(0, -10, 0);

        // assert after the FIRST update, while still deeply penetrating both walls:
        // the conflict-discard logic only runs for penetration beyond
        // 1.25 * characterPadding, which recovery resolves within a few steps
        vec3.scaleAndAdd(character.linearVelocity, character.linearVelocity, gravity, DELTA_TIME);
        kcc.update(world, character, DELTA_TIME, gravity, updateSettings, undefined, characterFilter);
        updateWorld(world, undefined, DELTA_TIME);

        // both wall contacts must be live — discarding one lets penetration
        // recovery push the character one-sided through the other wall
        expect(kcc.hasCollidedWith(character, wallA.id)).toBe(true);
        expect(kcc.hasCollidedWith(character, wallB.id)).toBe(true);

        // settle and verify the character stays centered in the corridor
        for (let step = 0; step < 5; step++) {
            vec3.scaleAndAdd(character.linearVelocity, character.linearVelocity, gravity, DELTA_TIME);
            kcc.update(world, character, DELTA_TIME, gravity, updateSettings, undefined, characterFilter);
            updateWorld(world, undefined, DELTA_TIME);
        }
        expect(Math.abs(character.position[0])).toBeLessThan(0.1);
    });
});

describe('KCC listener contact identity', () => {
    /**
     * listener contact tracking must key on the FULL body id (52-bit index+sequence).
     * a destroyed body whose pool slot is recycled produces a new id with the same
     * index but a bumped sequence — under the old 16/16-bit packed key both ids
     * collided, so the recycled body fired onContactPersisted instead of
     * onContactAdded.
     */
    test('recycled body slot fires onContactAdded, not onContactPersisted', () => {
        const { world, layers } = createTestWorld();

        const floorSpec = {
            shape: box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) }),
            position: vec3.fromValues(0, -0.5, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        };
        const floorA = rigidBody.create(world, floorSpec);

        updateWorld(world, undefined, DELTA_TIME);

        const character = kcc.create(
            {
                shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
                mass: 70,
                up: vec3.fromValues(0, 1, 0),
                supportingVolumePlane: vec4.fromValues(0, 1, 0, -1e10),
            },
            vec3.fromValues(0, 0.01, 0),
            quat.create(),
        );

        const characterFilter = filter.create(world.settings.layers);
        const updateSettings = kcc.createDefaultUpdateSettings();
        const gravity = vec3.fromValues(0, -10, 0);

        const added: number[] = [];
        const persisted: number[] = [];
        const listener = {
            onContactAdded: (_c: unknown, body: { id: number }) => {
                added.push(body.id);
            },
            onContactPersisted: (_c: unknown, body: { id: number }) => {
                persisted.push(body.id);
            },
        };

        const step = () => {
            vec3.scaleAndAdd(character.linearVelocity, character.linearVelocity, gravity, DELTA_TIME);
            kcc.update(world, character, DELTA_TIME, gravity, updateSettings, listener, characterFilter);
            updateWorld(world, undefined, DELTA_TIME);
        };

        // settle onto floor A: contact added once, then persisted
        for (let i = 0; i < 5; i++) step();
        expect(added).toContain(floorA.id);

        // destroy floor A and create floor B — the pool slot recycles: same index,
        // bumped sequence, so the ids collide under a 16-bit packed key
        const floorAId = floorA.id;
        rigidBody.remove(world, floorA);
        const floorB = rigidBody.create(world, floorSpec);
        expect(floorB.id).not.toBe(floorAId);

        added.length = 0;
        persisted.length = 0;
        for (let i = 0; i < 3; i++) step();

        // the recycled body is a NEW contact: it must fire onContactAdded.
        // under the old packed key it matched floor A's stale tracking entry and
        // fired onContactPersisted on first touch instead.
        expect(added).toContain(floorB.id);
        expect(persisted).not.toContain(floorAId);
    });
});
