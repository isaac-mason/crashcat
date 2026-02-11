import {
    addBroadphaseLayer,
    addObjectLayer,
    bitmask,
    box,
    createWorld,
    createWorldSettings,
    dof,
    enableCollision,
    MaterialCombineMode,
    massProperties,
    MotionQuality,
    MotionType,
    type RigidBody,
    rigidBody,
    sphere,
    type Listener,
} from 'crashcat';
import { Box3, box3, quat, vec3 } from 'mathcat';

const worldSettings = createWorldSettings();
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
const world = createWorld(worldSettings);

/* SNIPPET_START: creation */
// create a dynamic box
const dynamicBox = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    position: [0, 5, 0],
});

// create a static ground
const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [10, 0.5, 10] }),
    motionType: MotionType.STATIC,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    position: [0, -0.5, 0],
});

// remove a body from the world
rigidBody.remove(world, dynamicBox);
/* SNIPPET_END: creation */

/* SNIPPET_START: object-pooling */
// ❌ BAD: storing direct reference
type MyEntity = {
    body: RigidBody;
};

// ✅ GOOD: store body.id instead
type MyEntityGood = {
    bodyId: number;
};

// look up body by id when needed
const storedId = dynamicBox.id;
const bodyById = rigidBody.get(world, storedId);

if (bodyById) {
    rigidBody.addForce(world, bodyById, [0, 10, 0], true);
}
/* SNIPPET_END: object-pooling */

/* SNIPPET_START: motion-types */
// static: cannot move, infinite mass, not affected by forces
const staticBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [5, 0.5, 5] }),
    motionType: MotionType.STATIC,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
});

// dynamic: fully simulated, affected by forces, gravity, and contacts
const dynamicBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});

// kinematic: user-controlled velocity, pushes dynamic bodies
const kinematicBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [2, 0.2, 2] }),
    motionType: MotionType.KINEMATIC,
    objectLayer: OBJECT_LAYER_MOVING,
});
/* SNIPPET_END: motion-types */

/* SNIPPET_START: transform */
// read position and quaternion
const position = dynamicBody.position; // [x, y, z]
const quaternion = dynamicBody.quaternion; // [x, y, z, w]

// set position and quaternion together
const newPosition = vec3.fromValues(0, 10, 0);
const newQuaternion = quat.create();
rigidBody.setTransform(world, kinematicBody, newPosition, newQuaternion, false);

// set position and quaternion separately
rigidBody.setPosition(world, kinematicBody, newPosition, false);
rigidBody.setQuaternion(world, kinematicBody, newQuaternion, false);
/* SNIPPET_END: transform */

/* SNIPPET_START: velocity */
// read velocities
const linearVelocity = dynamicBody.motionProperties.linearVelocity; // [x, y, z] in m/s
const angularVelocity = dynamicBody.motionProperties.angularVelocity; // [x, y, z] in rad/s

// set linear velocity
rigidBody.setLinearVelocity(world, dynamicBody, [5, 0, 0]); // shoot to the right

// set angular velocity
rigidBody.setAngularVelocity(world, dynamicBody, [0, 1, 0]); // spin around y-axis
/* SNIPPET_END: velocity */

/* SNIPPET_START: forces */
// add force at center of mass (accumulates until next physics step)
const force = vec3.fromValues(0, 100, 0); // upward force
rigidBody.addForce(world, dynamicBody, force, true); // last arg: wake if sleeping

// add force at specific position (generates torque)
const worldPosition = vec3.fromValues(1, 0, 0); // apply force at right edge
rigidBody.addForceAtPosition(world, dynamicBody, force, worldPosition, true);

// add impulse (instant velocity change at center of mass)
const impulse = vec3.fromValues(0, 10, 0);
rigidBody.addImpulse(world, dynamicBody, impulse);

// add impulse at position (instant velocity + angular velocity change)
rigidBody.addImpulseAtPosition(world, dynamicBody, impulse, worldPosition);
/* SNIPPET_END: forces */

/* SNIPPET_START: mass */
// mass properties are computed automatically from shape and density
const body = rigidBody.create(world, {
    shape: sphere.create({ radius: 1, density: 1000 }), // density in kg/m³
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});

// override just the mass (inertia is scaled automatically)
const customMassBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    mass: 50, // kg
});

// completely override mass properties (useful for triangle meshes)
const triangleMeshMassProperties = massProperties.create();
massProperties.setMassAndInertiaOfSolidBox(triangleMeshMassProperties, [2, 2, 2], 1000); // boxSize, density

const dynamicTriangleMeshBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    massPropertiesOverride: triangleMeshMassProperties,
});

// read mass properties
const mass = 1 / body.motionProperties.invMass; // mass in kg
const invMass = body.motionProperties.invMass; // 1/mass, used internally
/* SNIPPET_END: mass */

/* SNIPPET_START: damping */
// linear damping reduces linear velocity over time (0-1, higher = more drag)
const dampedBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    linearDamping: 0.5, // default is 0.05
    angularDamping: 0.8, // default is 0.05
});

// damping can also be modified after creation
dampedBody.motionProperties.linearDamping = 0.2;
dampedBody.motionProperties.angularDamping = 0.2;
/* SNIPPET_END: damping */

/* SNIPPET_START: max-velocities */
// clamp maximum velocities to prevent instability
const fastBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    maxLinearVelocity: 100, // m/s, default is 500
    maxAngularVelocity: 10, // rad/s, default is ~47 (0.25 * PI * 60)
});
/* SNIPPET_END: max-velocities */

/* SNIPPET_START: dof */
// degrees of freedom control which axes a body can move/rotate on
// useful for 2d games or constrained movement

// only allow movement in x and z (2d platformer on xz plane)
const platformerBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 1, 0.5] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    allowedDegreesOfFreedom: dof(true, false, true, false, true, false), // tx, tz, ry
});

// dof args: translateX, translateY, translateZ, rotateX, rotateY, rotateZ
/* SNIPPET_END: dof */

/* SNIPPET_START: sleeping */
const sleepyBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    allowSleeping: true, // default
});

// check if sleeping
const isSleeping = sleepyBody.sleeping;

// manually control sleep state
rigidBody.sleep(world, sleepyBody);
rigidBody.wake(world, sleepyBody);
/* SNIPPET_END: sleeping */

/* SNIPPET_START: wake-in-aabb */
// wake all sleeping bodies within a region
// useful after explosions, level loading, or regional activation
rigidBody.wakeInAABB(world, [[-10, 0, -10], [10, 20, 10]]);
/* SNIPPET_END: wake-in-aabb */

/* SNIPPET_START: gravity-factor */
// no gravity
const floatingBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    gravityFactor: 0,
});

// double gravity
const heavyBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    gravityFactor: 2,
});

// modify after creation
heavyBody.motionProperties.gravityFactor = 0.5;
/* SNIPPET_END: gravity-factor */

/* SNIPPET_START: move-kinematic */
const platform = rigidBody.create(world, {
    shape: box.create({ halfExtents: [2, 0.2, 2] }),
    motionType: MotionType.KINEMATIC,
    objectLayer: OBJECT_LAYER_MOVING,
    position: [0, 2, 0],
});

// move each frame by computing velocities
const deltaTime = 1 / 60;
const targetPosition = vec3.fromValues(5, 2, 0);
const targetQuaternion = quat.create();
quat.setAxisAngle(targetQuaternion, vec3.fromValues(0, 1, 0), Math.PI / 4);

rigidBody.moveKinematic(platform, targetPosition, targetQuaternion, deltaTime);
/* SNIPPET_END: move-kinematic */

/* SNIPPET_START: ccd */
const bullet = rigidBody.create(world, {
    shape: sphere.create({ radius: 0.1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    motionQuality: MotionQuality.LINEAR_CAST, // enables ccd
});

// configure threshold in world settings (default 0.05 = 5%)
// worldSettings.ccd.linearCastThreshold = 0.05;
/* SNIPPET_END: ccd */

/* SNIPPET_START: userdata */
const player = rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 1, 0.5] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    userData: 123, // store entity id
});

const entityId = player.userData as number;
/* SNIPPET_END: userdata */

/* SNIPPET_START: collision-groups */
const GROUPS = bitmask.createFlags(['player', 'enemy', 'debris', 'projectile'] as const);

// player collides with enemies and projectiles, but not debris
const playerBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    collisionGroups: GROUPS.player,
    collisionMask: GROUPS.enemy | GROUPS.projectile,
});

// enemy collides with player and debris, but not other enemies
const enemyBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    collisionGroups: GROUPS.enemy,
    collisionMask: GROUPS.player | GROUPS.debris,
});

// debris collides with everything
const debrisBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    collisionGroups: GROUPS.debris,
    collisionMask: GROUPS.player | GROUPS.enemy | GROUPS.debris | GROUPS.projectile,
});
/* SNIPPET_END: collision-groups */

/* SNIPPET_START: material */
// low friction, high restitution
const bouncyBall = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    friction: 0.1,
    restitution: 0.9,
});

// high friction, no bounce
const stickyBox = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    friction: 1.0,
    restitution: 0.0,
});

// custom combine modes
const customCombine = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    friction: 0.5,
    restitution: 0.5,
    frictionCombineMode: MaterialCombineMode.MIN,
    restitutionCombineMode: MaterialCombineMode.MAX,
});
/* SNIPPET_END: material */

/* SNIPPET_START: sensor */
const triggerZone = rigidBody.create(world, {
    shape: box.create({ halfExtents: [5, 5, 5] }),
    motionType: MotionType.STATIC,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    sensor: true,
});

// detect when bodies enter/exit sensor
const listener: Listener = {
    onContactAdded: (bodyA, bodyB, manifold, settings) => {
        if (bodyA.id === triggerZone.id || bodyB.id === triggerZone.id) {
            // otherBody entered trigger zone
            const otherBody = bodyA.id === triggerZone.id ? bodyB : bodyA;
        }
    },
};
/* SNIPPET_END: sensor */
/* SNIPPET_START: update-shape */
// change a body's shape after creation
const changingBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});

// later: change to a sphere
changingBody.shape = sphere.create({ radius: 1.5 });
rigidBody.updateShape(world, changingBody); // recalculates mass, inertia, aabb
/* SNIPPET_END: update-shape */

/* SNIPPET_START: set-object-layer */
// move a body to a different object layer
const movableBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});

// change to a different layer (e.g. when picked up by player)
rigidBody.setObjectLayer(world, movableBody, OBJECT_LAYER_NOT_MOVING);
/* SNIPPET_END: set-object-layer */

/* SNIPPET_START: set-motion-type */
// change a body's motion type at runtime
const switchableBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});

// make it kinematic (e.g. player grabs it)
rigidBody.setMotionType(world, switchableBody, MotionType.KINEMATIC, true);

// make it dynamic again (e.g. player drops it)
rigidBody.setMotionType(world, switchableBody, MotionType.DYNAMIC, true);

// make it static (e.g. permanently attach to world)
rigidBody.setMotionType(world, switchableBody, MotionType.STATIC, false);
/* SNIPPET_END: set-motion-type */