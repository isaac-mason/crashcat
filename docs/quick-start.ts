import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    type CollideShapeHit,
    type ContactManifold,
    type ContactSettings,
    ContactValidateResult,
    createWorld,
    createWorldSettings,
    enableCollision,
    type Listener,
    MotionType,
    type RigidBody,
    registerAll,
    rigidBody,
    updateWorld,
} from 'crashcat';
import type { Vec3 } from 'mathcat';

// we can use registerShapes and registerConstraints to granularly declare
// which shapes and constraints we want to use for the best tree shaking.
// but early in development, it's easier to just register everything.
registerAll();

// this is a container for all settings related to world simulation.
// in a real project, you'd put this in a e.g. "physics-world-settings.ts" file seperate from the physics world
// creation, and import it and below constants where needed.
const worldSettings = createWorldSettings();

// we're first up going to define "broadphase layers".
// for simple projects, a "moving" and "not moving" broadphase layer is a good start.
// a "broadphase layer" has it's own broadphase tree, so defining layers is a way to split up the broadphase tree for better performance.
export const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
export const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

// next, we'll define some "object layers".
// an "object layer" is a grouping of rigid bodies, that belongs to a single "broadphase layer".
// we can set up rules for which "object layers" can collide with each other.
export const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
export const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

// here we declare that "moving" objects should collide with "not moving" objects, and with other "moving" objects.
// if we had more "object layers", e.g. "player", "debris", "terrain",we could set up more specific collision rules,
// e.g. "debris" collides with "terrain" but not with "player".
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

// time to create the physics world from the settings we've defined
const world = createWorld(worldSettings);

// now we can start adding bodies and constraints to the world, and simulating it!

// create a static ground
rigidBody.create(world, {
    motionType: MotionType.STATIC,
    shape: box.create({ halfExtents: [10, 1, 10] }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
});

// create a stack of dynamic boxes
for (let i = 0; i < 5; i++) {
    rigidBody.create(world, {
        motionType: MotionType.DYNAMIC,
        shape: box.create({ halfExtents: [1, 1, 1] }),
        objectLayer: OBJECT_LAYER_MOVING,
        position: [0, 2 + i * 2, 0],
    });
}

// update the world
// typically you will do this in a loop, e.g. requestAnimationFrame or setInterval, but here we'll just do a single step for demonstration
updateWorld(world, undefined, 1 / 60);

// if we want to listen to and modify physics events, we can pass a "listener" as the second argument
const listener: Listener = {
    onBodyPairValidate: (bodyA: RigidBody, bodyB: RigidBody) => {
        return true;
    },
    onContactValidate: (bodyA: RigidBody, bodyB: RigidBody, baseOffset: Vec3, hit: CollideShapeHit) => {
        return ContactValidateResult.ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR;
    },
    onContactAdded: (bodyA: RigidBody, bodyB: RigidBody, manifold: ContactManifold, settings: ContactSettings) => {
        // ...
    },
    onContactPersisted: (bodyA: RigidBody, bodyB: RigidBody, manifold: ContactManifold, settings: ContactSettings) => {
        // ...
    },
    onContactRemoved: (bodyIdA: number, bodyIdB: number, subShapeIdA: number, subShapeIdB: number) => {
        // ...
    },
};

// update the world with the listener
updateWorld(world, listener, 1 / 60);
