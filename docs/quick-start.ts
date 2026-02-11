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

// register all shapes
registerAll();

// create a simple world
const worldSettings = createWorldSettings();

export const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
export const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

export const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
export const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

worldSettings.gravity = [0, -9.81, 0];

const world = createWorld(worldSettings);


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

// simulate 10 seconds
for (let i = 0; i < 60 * 10; i++) {
  // typically you will do this in a loop, e.g. requestAnimationFrame or setInterval
  updateWorld(world, listener, 1 / 60);
}
