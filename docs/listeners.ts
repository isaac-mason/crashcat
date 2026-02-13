import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    ContactValidateResult,
    createWorld,
    createWorldSettings,
    enableCollision,
    getWorldSpaceContactPointOnA, getWorldSpaceContactPointOnB,
    type Listener,
    MotionType,
    removeBody,
    type RigidBody,
    rigidBody,
    updateWorld
} from 'crashcat';
import { vec3 } from 'mathcat';

const worldSettings = createWorldSettings();
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
const world = createWorld(worldSettings);

/* SNIPPET_START: basic */
// create a listener to react to physics events
const listener: Listener = {
    onContactAdded: (bodyA, bodyB, manifold, settings) => {
        // called when a new contact is detected
        console.log('contact added between', bodyA.id, 'and', bodyB.id);
    },
    onContactPersisted: (bodyA, bodyB, manifold, settings) => {
        // called when a contact from last frame is still active
    },
    onContactRemoved: (bodyIdA, bodyIdB, subShapeIdA, subShapeIdB) => {
        // called when a contact is no longer active
        // WARNING: bodies may be destroyed, only body IDs are safe to use
    },
};

// pass listener to updateWorld
updateWorld(world, listener, 1 / 60);
/* SNIPPET_END: basic */

/* SNIPPET_START: deferred-removal */
// WARNING: do NOT remove bodies inside listener callbacks!
// the physics system is in the middle of processing contacts and removing bodies
// will corrupt internal state. instead, store the body IDs and remove them after
// updateWorld completes:

const bodiesToRemove: number[] = [];

const deferredRemovalListener: Listener = {
    onContactAdded: (bodyA, bodyB) => {
        // example: destroy bodies that touch lava
        if (bodyA.userData === 'lava') {
            bodiesToRemove.push(bodyB.id);
        }
        if (bodyB.userData === 'lava') {
            bodiesToRemove.push(bodyA.id);
        }
    },
};

updateWorld(world, deferredRemovalListener, 1 / 60);

// now it's safe to remove bodies
for (const id of bodiesToRemove) {
    removeBody(world, id);
}
bodiesToRemove.length = 0;
/* SNIPPET_END: deferred-removal */

/* SNIPPET_START: body-pair-validate */
// most efficient place to filter collisions - before narrowphase runs
const validateListener: Listener = {
    onBodyPairValidate: (bodyA, bodyB) => {
        // custom filtering logic
        // e.g., prevent ragdoll self-collision, faction systems, etc.

        // example: prevent bodies with same userData from colliding
        if (bodyA.userData === bodyB.userData) {
            return false; // skip collision detection
        }

        return true; // allow collision
    },
};
/* SNIPPET_END: body-pair-validate */

/* SNIPPET_START: contact-validate */
// called after collision detection, before adding contact constraint
const contactValidateListener: Listener = {
    onContactValidate: (bodyA, bodyB, baseOffset, hit) => {
        // expensive to reject here - narrowphase already ran
        // prefer onBodyPairValidate or object layer filtering

        // example: one-way platform - only collide if falling down
        const relativeVelocity = bodyA.motionProperties.linearVelocity[1] - bodyB.motionProperties.linearVelocity[1];

        if (relativeVelocity > 0) {
            // moving up through platform
            return ContactValidateResult.REJECT_CONTACT;
        }

        return ContactValidateResult.ACCEPT_CONTACT;
    },
};
/* SNIPPET_END: contact-validate */

/* SNIPPET_START: modify-contact */
// modify contact behavior by changing settings
const modifyContactListener: Listener = {
    onContactAdded: (bodyA, bodyB, manifold, settings) => {
        // increase friction for this specific contact
        settings.combinedFriction = 2.0;

        // disable restitution (no bounce)
        settings.combinedRestitution = 0.0;

        // collision normal (points from bodyA to bodyB)
        const normal = manifold.worldSpaceNormal;

        // access contact points in world space
        const worldPointA = vec3.create();
        const worldPointB = vec3.create();

        for (let i = 0; i < manifold.numContactPoints; i++) {
            // get world-space positions of contact points
            getWorldSpaceContactPointOnA(worldPointA, manifold, i);
            getWorldSpaceContactPointOnB(worldPointB, manifold, i);

            // example: spawn particle effect at contact point
            // spawnContactEffect(worldPointA);

            // example: apply damage based on penetration depth
            // const damage = manifold.penetrationDepth * 10;
        }
    },
};
/* SNIPPET_END: modify-contact */
