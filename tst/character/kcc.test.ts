import { quat, type Vec3, vec3, vec4 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    capsule,
    compound,
    filter,
    kcc,
    MotionType,
    type RigidBody,
    registerAll,
    rigidBody,
    type Shape,
    scaled,
    sphere,
    triangleMesh,
    updateWorld,
    type World,
} from '../../src';
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

describe('KCC contact removed for destroyed body', () => {
    /**
     * onContactRemoved receives the body ID (not the body) so removal fires even when
     * the contacted body has been destroyed — a destroyed body has no RigidBody to pass.
     */
    test('destroying the ground fires onContactRemoved with its id', () => {
        const { world, layers } = createTestWorld();

        const floor = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) }),
            position: vec3.fromValues(0, -0.5, 0),
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
            vec3.fromValues(0, 0.01, 0),
            quat.create(),
        );

        const characterFilter = filter.create(world.settings.layers);
        const updateSettings = kcc.createDefaultUpdateSettings();
        const gravity = vec3.fromValues(0, -10, 0);

        const removed: number[] = [];
        const listener = {
            onContactRemoved: (_c: unknown, bodyId: number) => {
                removed.push(bodyId);
            },
        };

        const step = () => {
            vec3.scaleAndAdd(character.linearVelocity, character.linearVelocity, gravity, DELTA_TIME);
            kcc.update(world, character, DELTA_TIME, gravity, updateSettings, listener, characterFilter);
            updateWorld(world, listener, DELTA_TIME);
        };

        // settle onto the floor to establish the tracked contact
        for (let i = 0; i < 5; i++) step();
        expect(removed.length).toBe(0);

        // destroy the floor, then step: the tracked contact disappears and the removal
        // must fire with the destroyed body's id
        const floorId = floor.id;
        rigidBody.remove(world, floor);
        step();
        expect(removed).toContain(floorId);
    });
});

describe('KCC back face mode', () => {
    // a capsule resting 0.05 into a quad whose front faces down, so the character touches its back face
    const contactsWithDownFacingQuad = (backFaceMode: kcc.BackFaceMode) => {
        const { world, layers } = createTestWorld();

        const quad = rigidBody.create(world, {
            shape: triangleMesh.create({
                positions: [-2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2],
                indices: [0, 1, 2, 0, 2, 3],
            }),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });

        updateWorld(world, undefined, DELTA_TIME);

        const character = kcc.create(
            {
                shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 }),
                mass: 70,
                up: vec3.fromValues(0, 1, 0),
                backFaceMode,
            },
            vec3.fromValues(0, 0.75, 0),
            quat.create(),
        );

        kcc.refreshContacts(world, character, filter.create(world.settings.layers));
        return character.contacts.filter((contact) => contact.bodyId === quad.id).length;
    };

    test('COLLIDE finds the back face contact', () => {
        expect(contactsWithDownFacingQuad(kcc.BackFaceMode.COLLIDE)).toBeGreaterThan(0);
    });

    test('IGNORE does not', () => {
        expect(contactsWithDownFacingQuad(kcc.BackFaceMode.IGNORE)).toBe(0);
    });
});

/** a 10 x 1 x 10 static floor whose top face is at y = 0 */
function createFloor(
    world: World,
    objectLayer: number,
    rotation = quat.create(),
    halfExtents = vec3.fromValues(5, 0.5, 5),
): RigidBody {
    return rigidBody.create(world, {
        shape: box.create({ halfExtents }),
        position: vec3.fromValues(0, -0.5, 0),
        quaternion: rotation,
        motionType: MotionType.STATIC,
        objectLayer,
    });
}

/** a character whose shape sits on its position (plus the padding) */
function createCharacter(shape: Shape, position: Vec3, characterPadding: number) {
    return kcc.create({ shape, characterPadding, up: vec3.fromValues(0, 1, 0) }, position, quat.create());
}

describe('KCC sweep padding correction', () => {
    const PADDING = 0.1;

    // the shape is cast from position + padding * up, so a shape whose lowest point is `bottom` below its origin rests
    // one padding above the floor when position.y = bottom
    test('a sweep onto a floor stops one padding from its face', () => {
        const { world, layers } = createTestWorld();
        createFloor(world, layers.OBJECT_LAYER_NOT_MOVING);
        updateWorld(world, undefined, DELTA_TIME);

        const character = createCharacter(sphere.create({ radius: 0.5 }), vec3.fromValues(0, 1, 0), PADDING);
        expect(
            kcc.stickToFloor(world, character, vec3.fromValues(0, -2, 0), filter.create(world.settings.layers), undefined),
        ).toBe(true);

        expect(character.position[1]).toBeCloseTo(0.5, 4);
    });

    // an oblique sweep only lands one padding above the face (rather than one padding back along the sweep) when the
    // correction casts against the right face
    test('the supporting face of a rotated hit body is found in its own space', () => {
        const { world, layers } = createTestWorld();
        // a quarter turn about z makes the local x faces the floor's top and bottom
        const quarterTurnZ = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);
        createFloor(world, layers.OBJECT_LAYER_NOT_MOVING, quarterTurnZ, vec3.fromValues(0.5, 5, 5));
        updateWorld(world, undefined, DELTA_TIME);

        const character = createCharacter(sphere.create({ radius: 0.5 }), vec3.fromValues(0, 1, 0), PADDING);
        expect(
            kcc.stickToFloor(world, character, vec3.fromValues(2, -2, 0), filter.create(world.settings.layers), undefined),
        ).toBe(true);

        expect(character.position[1]).toBeCloseTo(0.5, 4);
        expect(character.position[0]).toBeCloseTo(0.5, 4);
    });

    test('a scaled capsule character is corrected through its scale', () => {
        const { world, layers } = createTestWorld();
        createFloor(world, layers.OBJECT_LAYER_NOT_MOVING);
        updateWorld(world, undefined, DELTA_TIME);

        // radius 0.5, half height 1 once scaled, so the lowest point is 1.5 below the origin
        const shape = scaled.create({
            shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.25 }),
            scale: vec3.fromValues(2, 2, 2),
        });
        const character = createCharacter(shape, vec3.fromValues(0, 2, 0), PADDING);
        expect(
            kcc.stickToFloor(world, character, vec3.fromValues(2, -2, 0), filter.create(world.settings.layers), undefined),
        ).toBe(true);

        expect(character.position[1]).toBeCloseTo(1.5, 3);
    });

    test('a compound character is corrected through its children', () => {
        const { world, layers } = createTestWorld();
        createFloor(world, layers.OBJECT_LAYER_NOT_MOVING);
        updateWorld(world, undefined, DELTA_TIME);

        // two spheres stacked, the lower one's lowest point 1 below the origin
        const shape = compound.create({
            children: [
                { shape: sphere.create({ radius: 0.5 }), position: vec3.fromValues(0, -0.5, 0), quaternion: quat.create() },
                { shape: sphere.create({ radius: 0.5 }), position: vec3.fromValues(0, 0.5, 0), quaternion: quat.create() },
            ],
        });
        const character = createCharacter(shape, vec3.fromValues(0, 2, 0), PADDING);
        expect(
            kcc.stickToFloor(world, character, vec3.fromValues(2, -2, 0), filter.create(world.settings.layers), undefined),
        ).toBe(true);

        expect(character.position[1]).toBeCloseTo(1, 3);
    });
});

describe('KCC sweep contact acceptance', () => {
    // a platform between the character and the floor that validate refuses. the floor's bounds reach above the
    // platform (a tall triangle off to the side), so the cast visits the floor first and accepts it, then reaches the
    // closer platform hit and rejects it: the sweep still lands on the floor and the contact is the floor's
    test('a rejected closer hit does not replace the accepted one', () => {
        const { world, layers } = createTestWorld();
        const floor = rigidBody.create(world, {
            shape: triangleMesh.create({
                positions: [-4, 0, -2, 0, 0, 4, 4, 0, -2, 20, 0, 0, 20, 5, 0, 21, 0, 0],
                indices: [0, 1, 2, 3, 4, 5],
            }),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        const platform = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(1, 0.25, 1) }),
            position: vec3.fromValues(0, 0.25, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        updateWorld(world, undefined, DELTA_TIME);

        const listener: kcc.CharacterListener = {
            onContactValidate: (_character, body) => body.id !== platform.id,
        };

        const character = createCharacter(sphere.create({ radius: 0.5 }), vec3.fromValues(0, 2, 0), 0.02);
        expect(
            kcc.stickToFloor(world, character, vec3.fromValues(0, -3, 0), filter.create(world.settings.layers), listener),
        ).toBe(true);

        expect(character.position[1]).toBeCloseTo(0.5, 3);
        expect(character.ground.bodyId).toBe(floor.id);
    });
});

describe('KCC move to contact', () => {
    // with no room for collide hits the contact moved to is not found again, so the swept contact is stored as is
    test('a contact not found at the new position keeps every field of the swept contact', () => {
        const { world, layers } = createTestWorld();
        const floor = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(5, 0.5, 5), materialId: 7 }),
            position: vec3.fromValues(0, -0.5, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        updateWorld(world, undefined, DELTA_TIME);

        const character = kcc.create(
            { shape: sphere.create({ radius: 0.5 }), maxNumHits: 0, up: vec3.fromValues(0, 1, 0) },
            vec3.fromValues(0, 1, 0),
            quat.create(),
        );
        expect(
            kcc.stickToFloor(world, character, vec3.fromValues(0, -2, 0), filter.create(world.settings.layers), undefined),
        ).toBe(true);

        expect(character.contacts.length).toBe(1);
        const contact = character.contacts[0];
        expect(contact.bodyId).toBe(floor.id);
        expect(contact.materialId).toBe(7);
        expect(contact.isSensor).toBe(false);
        expect(contact.canReceiveImpulses).toBe(true);
        expect(contact.hadCollision).toBe(true);
    });
});

describe('KCC hit reduction', () => {
    // two triangles at y = 0 and one at y = 0.05, all under the sphere and all facing up. with room for two hits the
    // full collide finds three, and reduction (same body, same normal, different triangles) keeps the deepest
    test('at max hits, contacts on one body with matching normals reduce to the deepest', () => {
        const { world, layers } = createTestWorld();
        const mesh = rigidBody.create(world, {
            shape: triangleMesh.create({
                positions: [-4, 0, -2, 0, 0, 4, 4, 0, -2, -3, 0, -3, 0, 0, 5, 3, 0, -3, -4, 0.05, -2, 0, 0.05, 4, 4, 0.05, -2],
                indices: [0, 1, 2, 3, 4, 5, 6, 7, 8],
            }),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        updateWorld(world, undefined, DELTA_TIME);

        // sphere bottom at 0.03: 0.02 into the upper triangle, 0.03 above the lower two
        const character = kcc.create(
            { shape: sphere.create({ radius: 0.5 }), maxNumHits: 2, up: vec3.fromValues(0, 1, 0) },
            vec3.fromValues(0, 0.51, 0),
            quat.create(),
        );
        kcc.refreshContacts(world, character, filter.create(world.settings.layers));

        expect(character.contacts.length).toBe(2);
        let deepest = Number.POSITIVE_INFINITY;
        for (const contact of character.contacts) {
            expect(contact.bodyId).toBe(mesh.id);
            deepest = Math.min(deepest, contact.distance);
        }
        // penetration 0.02, less the 0.02 padding
        expect(deepest).toBeCloseTo(-0.04, 4);
    });

    // four coplanar triangles with room for three: the fourth hit runs one full reduction pass, which merges all three
    // stored contacts into one before the fourth is added
    test('reduction makes a full pass rather than stopping once a slot frees', () => {
        const { world, layers } = createTestWorld();
        rigidBody.create(world, {
            shape: triangleMesh.create({
                positions: [
                    -4, 0, -2, 0, 0, 4, 4, 0, -2, -3, 0, -3, 0, 0, 5, 3, 0, -3, -5, 0, -2, 0, 0, 6, 5, 0, -2, -4, 0, -4, 0, 0, 3,
                    4, 0, -4,
                ],
                indices: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
            }),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        updateWorld(world, undefined, DELTA_TIME);

        const character = kcc.create(
            { shape: sphere.create({ radius: 0.5 }), maxNumHits: 3, up: vec3.fromValues(0, 1, 0) },
            vec3.fromValues(0, 0.47, 0),
            quat.create(),
        );
        kcc.refreshContacts(world, character, filter.create(world.settings.layers));

        expect(character.contacts.length).toBe(2);
    });

    // two triangles of one mesh fill both slots, then a box deeper under the sphere is reached: reduction frees a slot
    // by merging the two triangle contacts, so the query must not stop at max hits before it gets there
    test('reaching max hits does not stop the query while reduction can still free a slot', () => {
        const { world, layers } = createTestWorld();
        const mesh = rigidBody.create(world, {
            shape: triangleMesh.create({
                positions: [-4, 0, -2, 0, 0, 4, 4, 0, -2, -3, 0, -3, 0, 0, 5, 3, 0, -3],
                indices: [0, 1, 2, 3, 4, 5],
            }),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        const block = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(1, 0.5, 1) }),
            position: vec3.fromValues(0, -0.45, 0),
            motionType: MotionType.STATIC,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
        });
        updateWorld(world, undefined, DELTA_TIME);

        // sphere bottom at 0.03: 0.02 into the box, 0.03 above the triangles
        const character = kcc.create(
            { shape: sphere.create({ radius: 0.5 }), maxNumHits: 2, up: vec3.fromValues(0, 1, 0) },
            vec3.fromValues(0, 0.51, 0),
            quat.create(),
        );
        kcc.refreshContacts(world, character, filter.create(world.settings.layers));

        expect(character.contacts.length).toBe(2);
        expect(character.contacts.filter((contact) => contact.bodyId === mesh.id).length).toBe(1);
        const blockContact = character.contacts.find((contact) => contact.bodyId === block.id);
        expect(blockContact?.distance).toBeCloseTo(-0.04, 4);
    });
});
