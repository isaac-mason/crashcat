![cover](./docs/cover.png)

[![Version](https://img.shields.io/npm/v/crashcat?style=for-the-badge)](https://www.npmjs.com/package/crashcat)
![GitHub Workflow Status (with event)](https://img.shields.io/github/actions/workflow/status/isaac-mason/crashcat/main.yml?style=for-the-badge)
[![Downloads](https://img.shields.io/npm/dt/crashcat.svg?style=for-the-badge)](https://www.npmjs.com/package/crashcat)

```bash
> npm install crashcat
```

# crashcat

crashcat is physics engine for javascript, built for games, simulations, and creative websites.

**Features**

- 🎯 rigid body simulation
- 📦 support for various convex shapes, triangle mesh shapes, custom shapes
- 🔗 constraints with motors and springs (hinge, slider, distance, point, fixed, cone, swing-twist, six-dof)
- ⚡ continuous collision detection (ccd) for fast-moving objects
- 🎚️ flexible collision filtering
- 🔧 hooks for listening to and modifying physics events
- 🌳 broadphase spatial acceleration with dynamic bvh
- 😴 sleeping/activation system for performance
- 👻 sensor bodies for trigger volumes
- 🌲 pure javascript, written to be highly tree-shakeable
- 🔌 works with any javascript engine/library - babylon.js, playcanvas, three.js, or your own engine

**API Documentation**

This readme provides curated explanations, guides, and examples to help you get started with crashcat.

Auto-generated API documentation can be found at [crashcat.dev/docs](https://crashcat.dev/docs).

**Changelog**

See the [CHANGELOG.md](./CHANGELOG.md) for a detailed list of changes in each version.

**Examples**

<table>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-shapes">
        <img src="./examples/public/screenshots/example-shapes.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Shapes
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-triangle-mesh">
        <img src="./examples/public/screenshots/example-triangle-mesh.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Triangle Mesh
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-ragdoll">
        <img src="./examples/public/screenshots/example-ragdoll.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Ragdoll
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-kcc">
        <img src="./examples/public/screenshots/example-kcc.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Kinematic Character Controller
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-floating-character-controller">
        <img src="./examples/public/screenshots/example-floating-character-controller.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Floating Character Controller
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-pointer-ray">
        <img src="./examples/public/screenshots/example-pointer-ray.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Pointer Raycast
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-cluster">
        <img src="./examples/public/screenshots/example-cluster.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Cluster
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-cube-heap">
        <img src="./examples/public/screenshots/example-cube-heap.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Cube Heap
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-ccd">
        <img src="./examples/public/screenshots/example-ccd.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Continuous Collision Detection
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-kinematic">
        <img src="./examples/public/screenshots/example-kinematic.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Kinematic Body
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-constraints">
        <img src="./examples/public/screenshots/example-constraints.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Constraints
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-constraints-motors">
        <img src="./examples/public/screenshots/example-constraints-motors.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Constraint Motors
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-hinge-motor">
        <img src="./examples/public/screenshots/example-hinge-motor.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Hinge Motor
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-voxel-triangle-mesh">
        <img src="./examples/public/screenshots/example-voxel-triangle-mesh.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Voxel Triangle Mesh
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-voxel-custom-shape">
        <img src="./examples/public/screenshots/example-voxel-custom-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Voxel Custom Shape
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-restitution">
        <img src="./examples/public/screenshots/example-restitution.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Restitution
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-friction">
        <img src="./examples/public/screenshots/example-friction.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Friction
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-conveyor-belt">
        <img src="./examples/public/screenshots/example-conveyor-belt.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Conveyor Belt
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-linear-damping">
        <img src="./examples/public/screenshots/example-linear-damping.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Linear Damping
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-angular-damping">
        <img src="./examples/public/screenshots/example-angular-damping.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Angular Damping
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-gravity-factor">
        <img src="./examples/public/screenshots/example-gravity-factor.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Gravity Factor
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-mass-properties">
        <img src="./examples/public/screenshots/example-mass-properties.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Mass Properties
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-add-impulse-at-position">
        <img src="./examples/public/screenshots/example-add-impulse-at-position.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Add Impulse at Position
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-cast-ray">
        <img src="./examples/public/screenshots/example-cast-ray.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Cast Ray
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-cast-shape">
        <img src="./examples/public/screenshots/example-cast-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Cast Shape
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-collide-point">
        <img src="./examples/public/screenshots/example-collide-point.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Collide Point
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-collide-shape">
        <img src="./examples/public/screenshots/example-collide-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Collide Shape
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-sensor">
        <img src="./examples/public/screenshots/example-sensor.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Sensor
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-object-layer-filtering">
        <img src="./examples/public/screenshots/example-object-layer-filtering.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Object Layer Filtering
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-collision-filtering">
        <img src="./examples/public/screenshots/example-collision-filtering.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Collision Filtering
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-collide-shape-vs-shape">
        <img src="./examples/public/screenshots/example-collide-shape-vs-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Collide Shape vs Shape
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-collide-point-vs-shape">
        <img src="./examples/public/screenshots/example-collide-point-vs-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Collide Point vs Shape
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-cast-ray-vs-shape">
        <img src="./examples/public/screenshots/example-cast-ray-vs-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Cast Ray vs Shape
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-cast-shape-vs-shape">
        <img src="./examples/public/screenshots/example-cast-shape-vs-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Cast Shape vs Shape
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-gjk">
        <img src="./examples/public/screenshots/example-gjk.png" width="180" height="120" style="object-fit:cover;"/><br/>
        GJK Collision Detection
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-get-supporting-face">
        <img src="./examples/public/screenshots/example-get-supporting-face.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Get Supporting Face
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-convex-hull-builder">
        <img src="./examples/public/screenshots/example-convex-hull-builder.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Convex Hull Builder
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-test-scenarios">
        <img src="./examples/public/screenshots/example-test-scenarios.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Test Scenarios
      </a>
    </td>
  </tr>
</table>


## Table of Contents

- [Quick Start](#quick-start)
- [Physics World](#physics-world)
- [Rigid Bodies](#rigid-bodies)
- [Shapes](#shapes)
- [Listener](#listener)
- [Queries](#queries)
- [Constraints](#constraints)
- [Character Controllers](#character-controllers)
- [Multiple Physics Worlds](#multiple-physics-worlds)
- [World State Serialization](#world-state-serialization)
- [Tree Shaking](#tree-shaking)
- [Common Mistakes](#common-mistakes)
- [Optimization Tips](#optimization-tips)
- [Determinism](#determinism)
- [Custom Shapes](#custom-shapes)
- [Library Integrations](#library-integrations)
- [FAQ](#faq)
- [Community](#community)
- [Acknowledgements](#acknowledgements)

## Quick Start

Below is a minimal example of creating a physics world with a static ground and some dynamic boxes:

```ts
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

// simulate 10 seconds
for (let i = 0; i < 60 * 10; i++) {
  // typically you will do this in a loop, e.g. requestAnimationFrame or setInterval
  // pass 'undefined' for no physics listener
  updateWorld(world, undefined, 1 / 60);
}
```

## Physics World

A physics world contains all bodies, constraints, and simulation state. It manages collision detection, constraint solving, and integration.

crashcat uses a two-tier layer system for finding potential collisions efficiently:

**Broadphase Layers** partition space for performance. Each broadphase layer has its own spatial acceleration structure (dynamic bvh tree). Bodies in different broadphase layers use separate trees, improving query performance.

- simple approach: two layers - "moving" and "not moving"
- advanced: separate by update frequency - static terrain, kinematic platforms, dynamic debris
- each body belongs to exactly one broadphase layer

**Object Layers** control collision filtering. They define which bodies can collide with each other. Each object layer belongs to a single broadphase layer.

- examples: "player", "enemy", "terrain", "projectile", "debris"
- you define collision rules between object layers (e.g., "projectiles hit enemies but not other projectiles")
- this is the primary mechanism for controlling what collides with what

```ts
import { addBroadphaseLayer, addObjectLayer, createWorld, createWorldSettings, enableCollision, registerAll } from 'crashcat';

// we can use registerShapes and registerConstraints to granularly declare
// which shapes and constraints we want to use for the best tree shaking.
// but early in development, it's easier to just register everything.
registerAll();

// this is a container for all settings related to world simulation.
// in a real project, you'd put this in a e.g. "physics-world-settings.ts" file seperate from the physics world
// creation, and import it and below constants where needed.
const worldSettings = createWorldSettings();

// earth gravity
worldSettings.gravity = [0, -9.81, 0];

// we're first up going to define "broadphase layers".
// for simple projects, a "moving" and "not moving" broadphase layer is a good start.
export const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
export const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

// next, we'll define some "object layers".
export const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
export const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

// here we declare that "moving" objects should collide with "not moving" objects, and with other "moving" objects.
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

// time to create the physics world from the settings we've defined
const world = createWorld(worldSettings);
```

See the `WorldSettings` type for all available settings and their documentation: https://crashcat.dev/docs/types/crashcat.WorldSettings.html

### Stepping the Simulation

After creating a world, you advance the simulation by calling `updateWorld(world, listener, deltaTime)` in your game loop.

The `deltaTime` parameter is the time in seconds to advance the simulation. For a 60 FPS game loop, this is typically `1/60` (≈0.0167 seconds).

You can pass a listener to `updateWorld` to listen to and modify physics events, see the [Listener](#listener) section.

For simple use cases, you can use the frame delta time directly to do variable time stepping:

```ts
let lastTime = performance.now();
const maxDelta = 1 / 30;

function gameLoopVariableTimestep() {
    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    updateWorld(world, undefined, delta);

    // ... render ...

    requestAnimationFrame(gameLoopVariableTimestep);
}
```

For maximum stability and determinism, use a fixed physics timestep with an accumulator:

```ts
const PHYSICS_DT = 1 / 60;
let accumulator = 0;
let lastTimeFixed = performance.now();

function gameLoopFixedTimestep() {
    const currentTime = performance.now();
    const frameTime = Math.min((currentTime - lastTimeFixed) / 1000, 0.25);
    lastTimeFixed = currentTime;

    accumulator += frameTime;

    // step physics at fixed rate
    while (accumulator >= PHYSICS_DT) {
        updateWorld(world, undefined, PHYSICS_DT);
        accumulator -= PHYSICS_DT;
    }

    // ... render with interpolation ...
    // const alpha = accumulator / PHYSICS_DT;
    // interpolate body positions using alpha for smooth rendering

    requestAnimationFrame(gameLoopFixedTimestep);
}
```

This decouples physics simulation rate from render frame rate. Physics always steps at exactly 60 Hz regardless of how fast rendering runs. For smooth visuals, interpolate body positions between physics steps using the `alpha` value.

Read more on this here: https://gafferongames.com/post/fix_your_timestep/

### Units and Scale

crashcat uses SI units and OpenGL conventions:

- **Length**: meters (m)
- **Mass**: kilograms (kg)
- **Time**: seconds (s)
- **Force**: newtons (N)
- **Gravity**: -9.81 m/s² (earth gravity)
- **Coordinate System**: positive y is "up" by default, OpenGL right-handed system
- **Triangle Winding**: counter-clockwise (CCW) is front face

**Scale matters**: a box with `halfExtents: [100, 100, 100]` is a 100-meter cube (skyscraper-sized), which will appear to fall slowly relative to its size.

If your renderer uses a different coordinate system (e.g., z-up, left-handed), transform coordinates when transferring data between crashcat and your renderer.

## Rigid Bodies

Rigid bodies are the fundamental simulation objects in crashcat. They have attached shapes that define their collision geometry, and properties that control their simulation behavior.

### Creation and Removal

```ts
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
```

**⚠️ Storing references to bodies**

Bodies are pooled internally for performance. As such, be careful with storing long-lived references to body objects! Store `body.id` instead and use `rigidBody.get(world, id)` to look up bodies when needed.

Body ids contain an index and sequence number. When bodies are removed, their ids are invalidated and indices can be reused.

```ts
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
```

### Motion Types

Bodies can be static, dynamic, or kinematic. Choose the type based on how the object should behave:

**Static**: immovable objects
- **Use for**: terrain, walls, buildings, fixed obstacles
- **Properties**: infinite mass, never moves, collides only with dynamic bodies
- **Examples**: ground, level geometry, walls, buildings

**Dynamic**: physical objects
- **Use for**: objects that should respond naturally to physics forces and collisions
- **Properties**: has mass, affected by forces/gravity, collides with all body types
- **Examples**: falling boxes, bouncing balls, physics props, debris, projectiles

**Kinematic**: nonphysical moving objects
- **Use for**: objects that move via script/animation but should push dynamic bodies
- **Properties**: user-controlled velocity, pushes dynamic bodies but isn't pushed back
- **Examples**: moving platforms, elevators, automated doors, scripted animations

```ts
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
```

### Position and Rotation

The position of a rigid body represents its location (translation) in 3d world-space. The quaternion represents its orientation (rotation).

Position and rotation can be set when creating a body, or modified after creation using the APIs below.

```ts
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
```

**position vs centerOfMassPosition:**

- `position`: Location of the shape's origin (where the collision shape is)
- `centerOfMassPosition`: Location of the body's center of mass in world-space

For simple shapes (sphere, box, capsule), these are the same. For compound shapes or shapes with offset center of mass, they differ. The physics engine uses `centerOfMassPosition` internally for simulation.

### Velocity

Linear and angular velocity can be read directly from rigid body objects. The `rigidBody` namespace provides APIs for modifying velocities.

```ts
// read velocities
const linearVelocity = dynamicBody.motionProperties.linearVelocity; // [x, y, z] in m/s
const angularVelocity = dynamicBody.motionProperties.angularVelocity; // [x, y, z] in rad/s

// set linear velocity
rigidBody.setLinearVelocity(world, dynamicBody, [5, 0, 0]); // shoot to the right

// set angular velocity
rigidBody.setAngularVelocity(world, dynamicBody, [0, 1, 0]); // spin around y-axis
```

### Forces and Impulses

Forces accumulate until the next physics step, then get cleared. Impulses apply instant velocity changes. Use `addForceAtPosition` or `addImpulseAtPosition` to generate rotation.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-add-impulse-at-position">
      <img src="./examples/public/screenshots/example-add-impulse-at-position.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Add Impulse at Position</strong>
    </a>
  </td>
  </tr>
</table>

### Mass Properties

For most shapes, mass properties are computed automatically. For triangle meshes you need to provide them explicitly to use them with kinematic or static bodies.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-mass-properties">
      <img src="./examples/public/screenshots/example-mass-properties.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Mass Properties</strong>
    </a>
  </td>
  </tr>
</table>

### Damping

Damping simulates air resistance or drag. Higher values make objects slow down faster.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-linear-damping">
      <img src="./examples/public/screenshots/example-linear-damping.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Linear Damping</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-angular-damping">
      <img src="./examples/public/screenshots/example-angular-damping.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Angular Damping</strong>
    </a>
  </td>
  </tr>
</table>

### Maximum Velocities

Clamping velocities prevents instability and tunneling from extreme speeds.

```ts
// clamp maximum velocities to prevent instability
const fastBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    maxLinearVelocity: 100, // m/s, default is 500
    maxAngularVelocity: 10, // rad/s, default is ~47 (0.25 * PI * 60)
});
```

### Degrees of Freedom

Restricting degrees of freedom is useful for 2D games or objects that should only move on specific axes.

```ts
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
```

### Sleeping

Sleeping improves performance by skipping simulation for bodies at rest.

```ts
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
```

You can also wake all sleeping bodies within a specific region:

```ts
// wake all sleeping bodies within a region
// useful after explosions, level loading, or regional activation
rigidBody.wakeInAABB(world, [[-10, 0, -10], [10, 20, 10]]);
```

### Gravity Factor

Gravity factor multiplies the world gravity for a specific body. Set to 0 for floating objects, less than 1 for lighter-than-normal gravity, or greater than 1 for heavier gravity.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-gravity-factor">
      <img src="./examples/public/screenshots/example-gravity-factor.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Gravity Factor</strong>
    </a>
  </td>
  </tr>
</table>

### Moving Kinematic Bodies

`moveKinematic` takes a target position and quaternion, and computes the velocities needed to reach them, ensuring physical interactions with dynamic bodies rather than a direct teleportation.

Prefer using `moveKinematic` over `setTransform` for kinematic bodies such as moving platforms.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-kinematic">
      <img src="./examples/public/screenshots/example-kinematic.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Kinematic Body</strong>
    </a>
  </td>
  </tr>
</table>

### Continuous Collision Detection

Use CCD for fast-moving objects like bullets or vehicles to prevent tunneling through thin walls.

```ts
const bullet = rigidBody.create(world, {
    shape: sphere.create({ radius: 0.1 }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    motionQuality: MotionQuality.LINEAR_CAST, // enables ccd
});

// configure threshold in world settings (default 0.05 = 5%)
// worldSettings.ccd.linearCastThreshold = 0.05;
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-ccd">
      <img src="./examples/public/screenshots/example-ccd.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Continuous Collision Detection</strong>
    </a>
  </td>
  </tr>
</table>

### User Data

User data lets you attach game-specific data to bodies for easy lookup during collision callbacks.

```ts
const player = rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 1, 0.5] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    userData: 123, // store entity id
});

const entityId = player.userData as number;
```

### Collision Groups and Masks

Collision groups and masks provide fine-grained collision filtering using 32-bit bitmasks. This works alongside object layer filtering - both must pass for bodies to collide.

A collision occurs when `(groupA & maskB) != 0 AND (groupB & maskA) != 0`. Use the `bitmask` helper to define named groups.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-collision-filtering">
      <img src="./examples/public/screenshots/example-collision-filtering.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Collision Filtering</strong>
    </a>
  </td>
  </tr>
</table>

### Material Properties

Friction and restitution control surface interaction. Combine modes determine how material properties mix when two bodies collide.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-friction">
      <img src="./examples/public/screenshots/example-friction.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Friction</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-restitution">
      <img src="./examples/public/screenshots/example-restitution.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Restitution</strong>
    </a>
  </td>
  </tr>
</table>

### Sensors

Sensor bodies detect collisions without applying physical forces. Use them for trigger zones, pickups, or detection areas.

```ts
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
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-sensor">
      <img src="./examples/public/screenshots/example-sensor.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Sensor</strong>
    </a>
  </td>
  </tr>
</table>

### Updating Shape

You can change a body's shape after creation. This recalculates mass properties, inertia, and the axis-aligned bounding box.

```ts
// change a body's shape after creation
const changingBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});

// later: change to a sphere
changingBody.shape = sphere.create({ radius: 1.5 });
rigidBody.updateShape(world, changingBody); // recalculates mass, inertia, aabb
```

### Changing Object Layer

Object layers control which bodies can collide. You can change a body's layer at runtime to modify collision behavior.

```ts
// move a body to a different object layer
const movableBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1, 1, 1] }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});

// change to a different layer (e.g. when picked up by player)
rigidBody.setObjectLayer(world, movableBody, OBJECT_LAYER_NOT_MOVING);
```

### Changing Motion Type

You can change a body's motion type at runtime to switch between static, kinematic, and dynamic behavior.

```ts
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
```

## Shapes

Shapes determine how rigid bodies collide with each other. crashcat provides primitive shapes, complex shapes like triangle meshes, and decorator shapes for advanced use cases.

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-shapes">
      <img src="./examples/public/screenshots/example-shapes.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Shapes</strong>
    </a>
  </td>
  </tr>
</table>

### Convex Shapes

A convex shape is one where, if you pick any two points inside the shape, the line segment between them is also inside the shape. This property enables fast collision detection with the GJK/EPA algorithms.

To speed up collision detection, all convex shapes use a convex radius. The shape is first shrunk by the convex radius, then inflated again by the same amount, resulting in a rounded shape.

This rounding improves performance and contact manifold quality, but makes geometry slightly less accurate. Adjust the radius to balance speed vs precision.

#### Sphere

The simplest and fastest convex shape.

```ts
// simplest and fastest convex shape
sphere.create({ radius: 1 });

// with density for mass calculation
sphere.create({ radius: 1, density: 1000 }); // kg/m³
```

#### Box

Defined by half extents from the center.

```ts
// defined by half extents from center
box.create({ halfExtents: [1, 2, 0.5] });

// with density
box.create({ halfExtents: [1, 1, 1], density: 500 });
```

#### Capsule

A cylinder with hemispherical caps on each end.

```ts
// cylinder with hemispherical caps
capsule.create({
    halfHeightOfCylinder: 1, // half height of cylindrical section
    radius: 0.5,
});

// with density
capsule.create({
    halfHeightOfCylinder: 1,
    radius: 0.5,
    density: 800,
});
```

#### Cylinder

Defined by half height and radius.

```ts
// defined by half height and radius
cylinder.create({
    halfHeight: 1,
    radius: 0.5,
});

// with density
cylinder.create({
    halfHeight: 1,
    radius: 0.5,
    density: 1200,
});
```

#### Convex Hull

The convex hull of a set of points.

```ts
// convex envelope of a set of points
convexHull.create({
    positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], // flat array [x,y,z, x,y,z, ...]
});

// with density
convexHull.create({
    positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
    density: 900,
});
```

### Triangle Mesh Shape

Triangle meshes represent complex geometry using triangles. Typically used for static terrain and level geometry.

```ts
// triangle mesh: for complex static geometry like terrain
const meshShape = triangleMesh.create({
    positions: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10],
    indices: [0, 1, 2, 0, 2, 3],
});

// triangle meshes typically used with static bodies
rigidBody.create(world, {
    shape: meshShape,
    motionType: MotionType.STATIC,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
});

// for dynamic triangle meshes, provide mass properties explicitly
const dynamicMeshProps = massProperties.create();
massProperties.setMassAndInertiaOfSolidBox(dynamicMeshProps, [2, 2, 2], 1000);

rigidBody.create(world, {
    shape: meshShape,
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    massPropertiesOverride: dynamicMeshProps,
});
```

**⚠️ Dynamic triangle meshes**

Avoid using triangle meshes for dynamic bodies. Performance is poor (collision detection against triangle meshes is usually more expensive), and fast-moving meshes can tunnel through other objects easily. Use convex hulls or compound shapes instead for dynamic objects.

### Compound Shape

Compound shapes combine multiple child shapes into a single shape. Useful for complex objects like vehicles or characters.

```ts
// compound: combine multiple shapes into one
compound.create({
    children: [
        {
            position: [0, 0, 0],
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [2, 0.5, 1] }), // main body
        },
        {
            position: [0, 1, 0],
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }), // turret
        },
    ],
});

// useful for complex objects like vehicles, characters, furniture
rigidBody.create(world, {
    shape: compound.create({
        children: [
            {
                position: [0, 0, 0],
                quaternion: quat.create(),
                shape: box.create({ halfExtents: [1, 1, 1] }),
            },
        ],
    }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});
```

### Decorator Shapes

Decorator shapes modify other shapes without changing their collision shape.

#### Scaled

Apply non-uniform scaling to any shape.

```ts
// non-uniform scaling of any shape
scaled.create({
    shape: box.create({ halfExtents: [1, 1, 1] }),
    scale: [2, 0.5, 1], // stretch in x, squash in y
});

// works with any shape
scaled.create({
    shape: sphere.create({ radius: 1 }),
    scale: [1, 2, 1], // creates an ellipsoid
});
```

#### Offset Center of Mass

Shift the center of mass without changing collision shape. Useful for improving stability of tall objects.

```ts
// shift center of mass without changing collision geometry
// useful for stability (e.g., lowering COM on tall objects)
const stableShape = offsetCenterOfMass.create({
    shape: box.create({ halfExtents: [0.5, 2, 0.5] }), // tall box
    offset: [0, -1, 0], // lower center of mass
});

rigidBody.create(world, {
    shape: stableShape,
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});
```

### Reusing Shapes

Shapes can be created once and reused across multiple bodies. This saves memory and improves performance.

```ts
// create a shape once
const sharedBoxShape = box.create({ halfExtents: [1, 1, 1] });

// reuse it for multiple bodies
rigidBody.create(world, {
    shape: sharedBoxShape,
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    position: [0, 5, 0],
});

rigidBody.create(world, {
    shape: sharedBoxShape, // same shape instance
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    position: [5, 5, 0],
});
```

### Offline Shape Generation

Shapes are JSON-serializable objects. You can generate complex shapes offline (especially triangle meshes, which perform sanitization, active edge computation, and BVH construction) and load the JSON at runtime.

```ts
// offline (node.js, build step, bundler macro, etc)
const pregeneratedTriangleMeshShape = triangleMesh.create({
    positions: [/* large terrain data */],
    indices: [/* large index data */],
});

// serialize to JSON
const triangleMeshShapeJson = JSON.stringify(pregeneratedTriangleMeshShape);
// save to file or bundle with app

// runtime (browser):
const triangleMeshShape = JSON.parse(triangleMeshShapeJson);

// use the pre-processed shape directly
rigidBody.create(world, {
    shape: triangleMeshShape,
    motionType: MotionType.STATIC,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
});
```

## Listener

The listener lets you react to and modify physics events during world updates. Pass a listener to `updateWorld()` to receive callbacks for collision events.

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-sensor">
      <img src="./examples/public/screenshots/example-sensor.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Sensor</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-conveyor-belt">
      <img src="./examples/public/screenshots/example-conveyor-belt.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Conveyor Belt</strong>
    </a>
  </td>
  </tr>
</table>

### Basic Usage

```ts
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

// WARNING: do NOT remove bodies inside listener callbacks!
// the physics system is in the middle of processing contacts and removing bodies
// will corrupt internal state. instead, store the body IDs and remove them after
// updateWorld completes:
//
// const bodiesToRemove: number[] = [];
// const listener: Listener = {
//     onContactAdded: (bodyA, bodyB) => {
//         if (shouldDestroy(bodyA)) {
//             bodiesToRemove.push(bodyA.id);
//         }
//     }
// };
// updateWorld(world, listener, 1 / 60);
// for (const id of bodiesToRemove) {
//     removeBody(world, id);
// }
// bodiesToRemove.length = 0;
```

### Body Pair Validation

Runs before expensive narrowphase collision detection. Use this when filtering logic is too complex for object layers or collision groups/masks (which are faster). Prefer those simpler mechanisms when possible.

```ts
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
```

### Contact Validation

Called after collision detection but before adding the contact constraint. Rejecting contacts here is expensive since narrowphase has already run - prefer `onBodyPairValidate` or object layer filtering where possible. Use this for special cases where you use the contact information (contact point, normal, etc) to make a decision, such as one-way platforms or material-based effects.

```ts
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
```

### Modifying Contact Behavior

Adjust friction, restitution, and other properties for specific contacts.

```ts
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
```

## Queries

Queries let you ask questions about the physics world without running a full simulation step. Use them for raycasts, shape sweeps, overlap tests, and more.

### Cast Ray

Cast a ray through the world to find bodies along a line. Useful for line-of-sight checks, projectile trajectories, and mouse picking.

```ts
// cast a ray through the world to find bodies
const rayOrigin = vec3.fromValues(0, 5, 0);
const rayDirection = vec3.fromValues(0, -1, 0);
const rayLength = 100;

// create a filter to control what the ray can hit
const queryFilter = filter.create(world.settings.layers);

// closest: finds the nearest hit along the ray
const closestCollector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();
castRay(world, closestCollector, raySettings, rayOrigin, rayDirection, rayLength, queryFilter);

if (closestCollector.hit.status === CastRayStatus.COLLIDING) {
    const hitDistance = closestCollector.hit.fraction * rayLength;
    const hitPoint = vec3.scaleAndAdd(vec3.create(), rayOrigin, rayDirection, hitDistance);
    const hitBody = rigidBody.get(world, closestCollector.hit.bodyIdB)!;
    const surfaceNormal = rigidBody.getSurfaceNormal(vec3.create(), hitBody, hitPoint, closestCollector.hit.subShapeId);
    console.log('closest hit at', hitPoint);
    console.log('surface normal:', surfaceNormal);
}

// any: finds the first hit (fast early-out, useful for line-of-sight checks)
const anyCollector = createAnyCastRayCollector();
anyCollector.reset();
castRay(world, anyCollector, raySettings, rayOrigin, rayDirection, rayLength, queryFilter);

if (anyCollector.hit.status === CastRayStatus.COLLIDING) {
    console.log('ray hit something');
}

// all: collects every hit along the ray
const allCollector = createAllCastRayCollector();
allCollector.reset();
castRay(world, allCollector, raySettings, rayOrigin, rayDirection, rayLength, queryFilter);

for (const hit of allCollector.hits) {
    if (hit.status === CastRayStatus.COLLIDING) {
        console.log('hit at fraction', hit.fraction);
    }
}
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-cast-ray">
      <img src="./examples/public/screenshots/example-cast-ray.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Cast Ray</strong>
    </a>
  </td>
  </tr>
</table>

### Cast Shape

Sweep a shape through the world to find what it would hit. Essential for character movement, projectile prediction, and object placement.

```ts
// sweep a shape through the world (useful for character movement, projectiles)
const castPosition = vec3.fromValues(0, 5, 0);
const castQuaternion = quat.create();
const castScale = vec3.fromValues(1, 1, 1);
const castDisplacement = vec3.fromValues(0, -10, 0);

const sweepShape = sphere.create({ radius: 0.5 });

// closest: finds the nearest hit along the sweep
const closestShapeCollector = createClosestCastShapeCollector();
const shapeSettings = createDefaultCastShapeSettings();
castShape(
    world,
    closestShapeCollector,
    shapeSettings,
    sweepShape,
    castPosition,
    castQuaternion,
    castScale,
    castDisplacement,
    queryFilter,
);

if (closestShapeCollector.hit.status === CastShapeStatus.COLLIDING) {
    const hitFraction = closestShapeCollector.hit.fraction;
    const hitPosition = vec3.create();
    vec3.scaleAndAdd(hitPosition, castPosition, castDisplacement, hitFraction);
    console.log('shape hit at', hitPosition);
}

// any: finds the first hit (fast early-out)
const anyShapeCollector = createAnyCastShapeCollector();
anyShapeCollector.reset();
castShape(
    world,
    anyShapeCollector,
    shapeSettings,
    sweepShape,
    castPosition,
    castQuaternion,
    castScale,
    castDisplacement,
    queryFilter,
);

if (anyShapeCollector.hit.status === CastShapeStatus.COLLIDING) {
    console.log('shape hit something');
}

// all: collects every hit along the sweep
const allShapeCollector = createAllCastShapeCollector();
allShapeCollector.reset();
castShape(
    world,
    allShapeCollector,
    shapeSettings,
    sweepShape,
    castPosition,
    castQuaternion,
    castScale,
    castDisplacement,
    queryFilter,
);

for (const hit of allShapeCollector.hits) {
    if (hit.status === CastShapeStatus.COLLIDING) {
        console.log('shape hit at fraction', hit.fraction);
    }
}
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-cast-shape">
      <img src="./examples/public/screenshots/example-cast-shape.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Cast Shape</strong>
    </a>
  </td>
  </tr>
</table>

### Collide Point

Test if a point is inside any bodies. Useful for trigger zones, item pickups, and spatial checks.

```ts
// test if a point is inside any bodies (useful for triggers, pickups)
const point = vec3.fromValues(0, 2, 0);

// any: checks if the point is inside any body (fast early-out)
const anyPointCollector = createAnyCollidePointCollector();
const pointSettings = createDefaultCollidePointSettings();
collidePoint(world, anyPointCollector, pointSettings, point, queryFilter);

if (anyPointCollector.hit !== null) {
    console.log('point is inside body', anyPointCollector.hit.bodyIdB);
}

// all: finds every body containing the point
const allPointCollector = createAllCollidePointCollector();
allPointCollector.reset();
collidePoint(world, allPointCollector, pointSettings, point, queryFilter);

console.log('point is inside', allPointCollector.hits.length, 'bodies');
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-collide-point">
      <img src="./examples/public/screenshots/example-collide-point.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Collide Point</strong>
    </a>
  </td>
  </tr>
</table>

### Collide Shape

Test if a shape overlaps any bodies. Perfect for area triggers, placement validation, and explosion radius checks.

```ts
// test if a shape overlaps any bodies (useful for area triggers, placement validation)
const queryShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
const queryPosition = vec3.fromValues(0, 2, 0);
const queryQuaternion = quat.create();
const queryScale = vec3.fromValues(1, 1, 1);

// any: checks if the shape overlaps any body (fast early-out)
const anyShapeOverlapCollector = createAnyCollideShapeCollector();
const shapeOverlapSettings = createDefaultCollideShapeSettings();
collideShape(
    world,
    anyShapeOverlapCollector,
    shapeOverlapSettings,
    queryShape,
    queryPosition,
    queryQuaternion,
    queryScale,
    queryFilter,
);

if (anyShapeOverlapCollector.hit !== null) {
    console.log('shape overlaps body', anyShapeOverlapCollector.hit.bodyIdB);
}

// all: finds every body overlapping the shape
const allShapeOverlapCollector = createAllCollideShapeCollector();
allShapeOverlapCollector.reset();
collideShape(
    world,
    allShapeOverlapCollector,
    shapeOverlapSettings,
    queryShape,
    queryPosition,
    queryQuaternion,
    queryScale,
    queryFilter,
);

console.log('shape overlaps', allShapeOverlapCollector.hits.length, 'bodies');
```

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-collide-shape">
      <img src="./examples/public/screenshots/example-collide-shape.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Collide Shape</strong>
    </a>
  </td>
  </tr>
</table>

### Broadphase Queries

For advanced scenarios, you can query the broadphase spatial acceleration structure directly. This is faster than narrowphase queries but less precise - it only tests axis-aligned bounding boxes (AABBs), not exact shapes.

```ts
// for advanced scenarios: query the broadphase spatial acceleration structure directly
// useful when you need custom traversal logic or want to avoid narrowphase overhead

// intersectAABB: find all bodies whose AABBs overlap a box
const queryAABB: Box3 = [
    [-5, -5, -5],
    [5, 5, 5],
];

const aabbVisitor: BodyVisitor = {
    shouldExit: false,
    visit(body: RigidBody) {
        console.log('body AABB overlaps query AABB:', body.id);
        // set shouldExit = true to stop traversal early
    },
};

broadphase.intersectAABB(world, queryAABB, queryFilter, aabbVisitor);

// intersectPoint: find all bodies whose AABBs contain a point
const queryPoint = vec3.fromValues(0, 5, 0);

const pointVisitor: BodyVisitor = {
    shouldExit: false,
    visit(body: RigidBody) {
        console.log('body AABB contains point:', body.id);
    },
};

broadphase.intersectPoint(world, queryPoint, queryFilter, pointVisitor);
```

### Shape vs Shape

For advanced scenarios, you can query shape-vs-shape directly.

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-collide-shape-vs-shape">
      <img src="./examples/public/screenshots/example-collide-shape-vs-shape.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Collide Shape vs Shape</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-collide-point-vs-shape">
      <img src="./examples/public/screenshots/example-collide-point-vs-shape.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Collide Point vs Shape</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-cast-ray-vs-shape">
      <img src="./examples/public/screenshots/example-cast-ray-vs-shape.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Cast Ray vs Shape</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-cast-shape-vs-shape">
      <img src="./examples/public/screenshots/example-cast-shape-vs-shape.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Cast Shape vs Shape</strong>
    </a>
  </td>
  </tr>
</table>

### Query Filters

Filters control what queries can hit using object layers, broadphase layers, collision groups/masks, and custom callbacks.

Filters apply three levels of filtering in order:

1. **Object/Broadphase Layers**: Fast spatial partitioning (configured in world settings)
2. **Collision Groups/Masks**: Bitwise filtering using 32-bit masks (checks against `rigidBody.collisionGroups` and `rigidBody.collisionMasks`)
3. **Body Filter Callback**: Custom logic for complex filtering (slowest, use sparingly)

All three must pass for a query to pass for a body.

```ts
// filters control what queries can hit, using object layers and collision groups/masks

// basic: create a filter with all layers enabled
const worldQueryFilter = filter.create(world.settings.layers);

// filter specific object layers
filter.disableObjectLayer(worldQueryFilter, world.settings.layers, OBJECT_LAYER_DEBRIS);
filter.enableObjectLayer(worldQueryFilter, world.settings.layers, OBJECT_LAYER_MOVING);

// filter specific broadphase layers
filter.disableBroadphaseLayer(worldQueryFilter, world.settings.layers, BROADPHASE_LAYER_MOVING);
filter.enableBroadphaseLayer(worldQueryFilter, world.settings.layers, BROADPHASE_LAYER_NOT_MOVING);

// filter everything, then selectively enable
filter.disableAllLayers(worldQueryFilter, world.settings.layers);
filter.enableObjectLayer(worldQueryFilter, world.settings.layers, OBJECT_LAYER_MOVING);

// collision groups and masks (works alongside layer filtering)
worldQueryFilter.collisionGroups = 0b0001; // query belongs to group 1
worldQueryFilter.collisionMask = 0b0010 | 0b0100; // query hits groups 2 and 4

// custom body filter callback
worldQueryFilter.bodyFilter = (body: RigidBody) => {
    // custom logic - exclude specific bodies
    if (body.userData === 'player') return false;

    // only hit dynamic bodies
    if (body.motionType !== MotionType.DYNAMIC) return false;

    return true;
};

// setFromBody: configure filter to match what a body can collide with
const playerBody = rigidBody.get(world, playerId)!;
filter.setFromBody(worldQueryFilter, world.settings.layers, playerBody);

// copy filter settings
const rayFilter = filter.create(world.settings.layers);
filter.copy(rayFilter, worldQueryFilter);

// use filter in queries
castRay(world, closestCollector, raySettings, rayOrigin, rayDirection, rayLength, worldQueryFilter);
```

```ts
export type Filter = {
    /** enabled object layers (1 = enabled, 0 = disabled) */
    enabledObjectLayers: number[];
    /** enabled broadphase layers (1 = enabled, 0 = disabled) */
    enabledBroadphaseLayers: number[];
    /** collision mask */
    collisionMask: number;
    /** collision group */
    collisionGroups: number;
    /** body filter callback */
    bodyFilter: ((body: RigidBody) => boolean) | undefined;
};
```

## Constraints

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-constraints">
      <img src="./examples/public/screenshots/example-constraints.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Constraints</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-constraints-motors">
      <img src="./examples/public/screenshots/example-constraints-motors.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Constraint Motors</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-hinge-motor">
      <img src="./examples/public/screenshots/example-hinge-motor.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Hinge Motor</strong>
    </a>
  </td>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-ragdoll">
      <img src="./examples/public/screenshots/example-ragdoll.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Ragdoll</strong>
    </a>
  </td>
  </tr>
</table>

Constraints connect bodies together to create complex mechanical systems like ragdolls, vehicles, and articulated structures. crashcat supports 8 constraint types ranging from simple connections to fully configurable constraints.

### Creating and Removing Constraints

```ts
const world = createWorld(createWorldSettings());

// create two bodies
const bodyA = rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
    objectLayer: 0,
    motionType: MotionType.STATIC,
    position: [0, 5, 0],
});

const bodyB = rigidBody.create(world, {
    shape: sphere.create({ radius: 0.5 }),
    objectLayer: 1,
    motionType: MotionType.DYNAMIC,
    position: [0, 3, 0],
});

// create a point constraint connecting them
const constraint = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 4.5, 0], // point on bodyA in world space
    pointB: [0, 3.5, 0], // point on bodyB in world space
    space: ConstraintSpace.WORLD,
});

// constraints can be enabled/disabled
constraint.enabled = false;

// remove constraint when done
pointConstraint.remove(world, constraint);
```

### Constraint Types

crashcat supports the following constraint types:

**Available Constraints**:

- **PointConstraint**: Connects two bodies at a point (removes 3 DOF). Like a ball-and-socket.
- **DistanceConstraint**: Maintains distance between two points (removes 1 DOF). Like a rope or stick.
- **HingeConstraint**: Allows rotation around an axis (removes 5 DOF). Like a door hinge or wheel axle.
- **SliderConstraint**: Allows movement along an axis (removes 5 DOF). Like a piston or rail.
- **FixedConstraint**: Completely locks two bodies together (removes 6 DOF). Like welding.
- **ConeConstraint**: Limits rotation within a cone (removes 3 DOF). Like a shoulder.
- **SwingTwistConstraint**: Approximates shoulder-like movement with swing and twist limits.
- **SixDOFConstraint**: Most configurable - specify limits per translation/rotation axis.

```ts
// point constraint - connects two bodies at a point (removes 3 DOF)
const point = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 1, 0],
    pointB: [0, 0, 0],
});

// distance constraint - maintains distance between two points (removes 1 DOF)
const distance = distanceConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 1, 0],
    pointB: [0, 0, 0],
    minDistance: 1,
    maxDistance: 2,
});

// hinge constraint - allows rotation around an axis (removes 5 DOF)
const hinge = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
});

// slider constraint - allows movement along an axis (removes 5 DOF)
const slider = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
    limitsMin: -2,
    limitsMax: 2,
});

// fixed constraint - completely locks two bodies together (removes 6 DOF)
const fixed = fixedConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    point1: [0, 0, 0],
    point2: [0, 0, 0],
    axisX1: [1, 0, 0],
    axisY1: [0, 1, 0],
    axisX2: [1, 0, 0],
    axisY2: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
});
```

### Constraint Motors

Some constraints support motors that apply forces/torques to drive bodies to a target velocity or position. There are two motor types:

**Velocity Motors**

Drive bodies to a constant relative velocity. For hinges this is angular velocity (rad/s), for sliders it's linear velocity (m/s).

```ts
// create a hinge with a velocity motor
const hingeWithMotor = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
});

// configure motor for velocity control
motorSettings.setTorqueLimit(hingeWithMotor.motorSettings, 100); // max torque in N*m
hingeConstraint.setMotorState(hingeWithMotor, MotorState.VELOCITY);
hingeConstraint.setTargetAngularVelocity(hingeWithMotor, 2 * Math.PI); // rad/s

// for slider constraints, use force limits instead
const sliderWithMotor = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
});

motorSettings.setForceLimit(sliderWithMotor.motorSettings, 500); // max force in N
sliderConstraint.setMotorState(sliderWithMotor, MotorState.VELOCITY);
sliderConstraint.setTargetVelocity(sliderWithMotor, 2); // m/s
```

**Position Motors**

Drive bodies to a target angle (hinges) or position (sliders) using a spring. The spring has two parameters:

- **Frequency**: How fast it reaches the target (Hz). Higher = stiffer spring. Valid range: (0, 0.5 × simulation frequency]. For 60 Hz physics, 20 Hz is stiff, 2 Hz is soft.
- **Damping**: Prevents overshoot. 0 = oscillates forever, 1 = critical damping (no overshoot), >1 = overdamped (slower).

```ts
// position motors drive to a target angle/position using a spring
const positionHinge = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
});

// configure spring settings for position motor
// frequency: how fast it reaches target (Hz). higher = stiffer spring
// damping: prevents overshoot. 0 = oscillates forever, 1 = critical damping, >1 = overdamped
positionHinge.motorSettings.springSettings.frequencyOrStiffness = 2; // 2 Hz - soft spring
positionHinge.motorSettings.springSettings.damping = 1; // critical damping, no overshoot

// drive to target angle
hingeConstraint.setMotorState(positionHinge, MotorState.POSITION);
hingeConstraint.setTargetAngle(positionHinge, Math.PI / 2); // 90 degrees

// same for slider - drives to target position
const positionSlider = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
});

positionSlider.motorSettings.springSettings.frequencyOrStiffness = 5; // 5 Hz - stiffer
positionSlider.motorSettings.springSettings.damping = 0.5; // some oscillation
sliderConstraint.setMotorState(positionSlider, MotorState.POSITION);
sliderConstraint.setTargetPosition(positionSlider, 1.5); // meters
```

**Motor States**:

- `MotorState.OFF`: Motor is inactive
- `MotorState.VELOCITY`: Drives to target velocity
- `MotorState.POSITION`: Drives to target position using spring

**Force/Torque Limits**:

- For angular motors (hinges): Use `setTorqueLimit()`. Units are Newton-meters (N·m). Formula: Torque = Inertia × Angular Acceleration.
- For linear motors (sliders): Use `setForceLimit()`. Units are Newtons (N). Formula: Force = Mass × Acceleration.
- Common pattern: Set limits to large values (e.g., 1e6) or `[-Infinity, Infinity]` to let the motor achieve its target as fast as possible.

### Constraint Limits

Hinges can limit rotation angle, sliders can limit position. Limits can use hard stops or soft springs.

```ts
// hinge constraints can have angle limits
const limitedHinge = hingeConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0],
    pointB: [0, -0.5, 0],
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    space: ConstraintSpace.LOCAL,
    limitsMin: -Math.PI / 4, // -45 degrees
    limitsMax: Math.PI / 4, // +45 degrees
});

// slider constraints can have position limits
const limitedSlider = sliderConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0, 0],
    pointB: [0, 0, 0],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
    space: ConstraintSpace.LOCAL,
    limitsMin: -1, // -1 meter
    limitsMax: 2, // +2 meters
});

// limits can use soft springs instead of hard stops
limitedSlider.limitsSpringSettings.frequencyOrStiffness = 10; // Hz
limitedSlider.limitsSpringSettings.damping = 0.7;
```

### Local vs World Space

Constraint attachment points can be specified in world space or local space:

```ts
// world space - specify attachment points in world coordinates
const worldConstraint = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 5, 0], // absolute world position
    pointB: [0, 3, 0], // absolute world position
    space: ConstraintSpace.WORLD, // default
});

// local space - specify attachment points relative to body center of mass
const localConstraint = pointConstraint.create(world, {
    bodyIdA: bodyA.id,
    bodyIdB: bodyB.id,
    pointA: [0, 0.5, 0], // offset from bodyA's center
    pointB: [0, -0.5, 0], // offset from bodyB's center
    space: ConstraintSpace.LOCAL,
});
```

## Character Controllers

### Kinematic Character Controllers (KCC)

crashcat has a built-in `kcc` API that provides kinematic character controller functionality ideal for player characters that need precise movement.

Features include:

- Sliding along walls and obstacles
- Stair stepping and slope handling
- Ground and ceiling detection
- Interaction with moving platforms and elevators
- Enhanced steep slope detection (prevents getting stuck in corners)
- Stick-to-ground behavior when walking down slopes

A kinematic character controller by default is not visible to raycasts or collision queries by default since it's not a rigid body. If you need it to be detectable / have a physical presence in the world (e.g., for sensors or AI line-of-sight, or dynamic body collisions), the `kcc` API supports creating an inner rigid body that follows the KCC's movement.

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-kcc">
      <img src="./examples/public/screenshots/example-kcc.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Kinematic Character Controller</strong>
    </a>
  </td>
  </tr>
</table>

### Dynamic Character Controllers

For characters that should behave like physics objects (ragdolls, physics-based characters, simple AI), you can build a controller using a regular dynamic rigid body with constraints on rotation to keep it upright.

This approach can be cheaper to simulate, but has tradeoffs in precise control over movement behavior. It can work well for AI characters or situations where you want realistic physical reactions.

The below example shows how you can create a floating capsule character controller in user-land. This can be copy/pasted into your project as a starting place and customized as needed.

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-floating-character-controller">
      <img src="./examples/public/screenshots/example-floating-character-controller.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Floating Character Controller</strong>
    </a>
  </td>
  </tr>
</table>

## Multiple Physics Worlds

The shape and constraints registry is global in crashcat, but you can create as many independent physics worlds as you need.

This can be useful in some advanced scenarios, e.g. for space games, where a spaceship and it's movement might be simulated in one world, but the movement of characters inside the ship is simulated in another world with different gravity and scale.

## World State Serialization

A physics world in crashcat is a simple JSON-serializable object. If need be, you can JSON.stringify and JSON.parse with the entire world state, including bodies, shapes, constraints, and settings. This can be useful for saving/loading game state for debugging, or more advanced use cases. Note that object references for e.g. sharing shapes across bodies will of course not survive serialization.

## Tree Shaking

crashcat is built to be highly tree-shakeable. By selectively registering only the shapes and constraints you need, modern bundlers can eliminate unused code and significantly reduce your bundle size.

### Using `registerAll`

The simplest approach is to use `registerAll()`, which registers all built-in shapes and constraints. This is convenient but includes everything in your bundle:

```ts
import { registerAll } from 'crashcat';

// register all built-in shapes and constraints
// this is simple but includes everything in the bundle
registerAll();
```

### Using `registerShapes` and `registerConstraints`

During development, it can be easier to use `registerAll()` while exploring what shapes and constraints you need. Once your usage is more finalized, you can switch to selective registration reduce bundle size significantly.

You must call `registerShapes()` or `registerConstraints()` before creating any bodies or constraints using those types. If you try to create a body with an unregistered shape, the shapes will effectively be "empty" shapes that have no collision

```ts
import { registerShapes, registerConstraints } from 'crashcat';
import { sphere, box, capsule } from 'crashcat';
import { hingeConstraint, distanceConstraint } from 'crashcat';

// only register the shapes you need
registerShapes([sphere.def, box.def, capsule.def]);

// only register the constraints you need
registerConstraints([hingeConstraint.def, distanceConstraint.def]);
```

With this approach, only the sphere, box, and capsule shapes will be included in your bundle. All other shapes (cylinder, convexHull, plane, triangleMesh, etc.) will be tree-shaken away.

## Common Mistakes

### Shapes have no collision or constraints don't work

If you create bodies but they fall through each other, or constraints don't connect properly, you may have forgotten to register shapes and constraints.

**Before shapes or constraints are simulated**, you must call either:

- `registerAll()` - registers all built-in shapes and constraints (simplest)
- `registerShapes([...])` and `registerConstraints([...])` - selective registration (smaller bundle size)

```typescript
import { registerAll } from "crashcat";

// MUST call this before updateWorld, or doing world queries
registerAll();
```

Without registration, shapes will behave as "empty" shapes with no collision, and constraints won't function properly.

See [Tree Shaking](#tree-shaking) for details on selective registration to reduce bundle size.

### Body isn't affected by gravity

If you expect your body to fall but it doesn't, check the following:

- **Gravity is enabled**: World gravity is non-zero (`worldSettings.gravity` and `worldSettings.gravityEnabled`)
- **Motion type is dynamic**: Static and kinematic bodies ignore gravity
- **Translations aren't locked**: Check `allowedDOFs` - if you locked translation axes, the body can't move in those directions
- **Body has mass**: Bodies with zero mass won't respond to forces

**Mass calculation**:

- If you don't explicitly set mass properties, crashcat calculates mass from the shape's volume and density
- Most shapes default to `density: 1000` (kg/m³), which gives reasonable mass values
- Triangle mesh shapes don't calculate mass automatically - you must provide mass properties explicitly for dynamic/kinematic triangle mesh bodies
- If mass seems wrong, check your shape's `density` parameter or set mass properties explicitly using `massProperties.create()`

### Applying forces or impulses doesn't work

If forces/impulses have no effect, verify:

- **Motion type is dynamic**: Only dynamic bodies respond to forces
- **Body has mass**: Zero mass means infinite inertia - forces have no effect
- **Force magnitude is sufficient**: Try a very large force (e.g., `1e5`) to rule out magnitude issues. Remember Force = Mass × Acceleration, so heavier bodies need stronger forces
- **Angular inertia for torques**: For rotational forces (`addTorque`, `addImpulseAtPosition`), the body needs non-zero angular inertia

### Everything moves in slow motion

A common mistake is using pixels or other non-SI units as the physics length unit.

crashcat uses **SI units** (meters, kilograms, seconds). The default gravity is `-9.81 m/s²` (earth gravity). If you create a box with `halfExtents: [100, 100, 100]`, you've made a **100-meter cube** (the size of a large building), which will fall very slowly relative to its size.

```typescript
// physics: 1 meter = human-scale
const body = rigidBody.create(world, {
  shape: capsule.create({ radius: 0.3, halfHeightOfCylinder: 0.7 }), // ~1.4m tall
  position: [0, 10, 0], // 10 meters up
  // ...
});

// rendering: scale physics position to pixels
const renderPosition = [
  body.position[0] * 50, // 50 pixels per meter
  body.position[1] * 50,
  body.position[2] * 50,
];
```

## Optimization Tips

### Use simple shapes

Use the simplest shapes that work for your use case. Collision detection cost roughly follows:
- Sphere (fastest) → Box → Capsule → Cylinder → Convex Hull → Triangle Mesh (slowest)

For complex objects, prefer compound shapes made of simple convex shapes over triangle meshes when possible and practical.

### Reuse shapes

Create complex shapes (especially triangle meshes and convex hulls) once and reuse them across multiple bodies. Shape creation can be expensive, but using the same shape instance for many bodies is cheap.

```typescript
// good: create once, reuse many times
const terrainShape = triangleMesh.create({ positions, indices });
for (let i = 0; i < 100; i++) {
    rigidBody.create(world, { shape: terrainShape, /* ... */ });
}

// bad: creating new triangle mesh for each body
for (let i = 0; i < 100; i++) {
    rigidBody.create(world, { 
        shape: triangleMesh.create({ positions, indices }), 
        /* ... */ 
    });
}
```

### Sleep bodies on creation

If you're spawning many bodies that start at rest (e.g., a pile of objects), create them with `allowSleeping: true` and they'll enter sleep state quickly, skipping simulation until disturbed. This is especially useful for scenes with many pre-placed objects.

### Try reducing solver iterations

The default solver iteration counts balance accuracy and performance. If you can accept less accurate physics, reduce `numVelocitySteps` and `numPositionSteps` in world settings. This directly affects simulation time.

### Consider moving the simulation to a Web Worker

For heavy simulations, consider running physics in a Web Worker to keep the main thread responsive.

This can be especially appropriate for creative website use cases where you want to push the limits of physics complexity for an interesting interactive experience, but don't want to compromise the responsiveness of the UI.

This is not always trivial for more complicated game developement scenarios, and so is more of a situational decision to make with respect to your wider engine architecture.

## Determinism

The world simulation update has implementation considerations for determinism (such as contact sorting), but this is not deeply tested yet.

**For deterministic results, you must ensure:**

- **Exact same initial state**: World settings, bodies, constraints, shapes - everything must match exactly
- **Exact same order of operations**: Adding and removing bodies, constraints - all operations must happen in the same order every time
- **Same update sequence**: Call `updateWorld` with identical delta times and in the same order relative to other game logic

**Easy ways to break determinism:**

- Using non-deterministic data structures or iteration orders (e.g., `Set`, `Map` without careful handling)
- Creating bodies in different orders
- Using varying delta time steps
- Using javascript functions like `Math.sin` or `Math.cos` that may have environment-dependent implementations

## Custom Shapes

crashcat has experimental functionality for defining custom shapes.

This is a lot of work to set up, but can be situationally very worthwhile when assumptions about a world can be more effectively communicated with a custom shape, e.g. voxel or voxel-like worlds, destructable terrain, or other complex procedurally-generated geometry.

This is demonstrated in the below example, which creates a custom "voxel world" shape that efficiently represents voxel volume, supporting modification at runtime without any rebuild cost.

<table>
  <tr>
  <td align="center">
    <a href="https://crashcat.dev/examples#example-voxel-custom-shape">
      <img src="./examples/public/screenshots/example-voxel-custom-shape.png" width="200" height="133" style="object-fit:cover;"/><br/>
      <strong>Voxel Custom Shape</strong>
    </a>
  </td>
  </tr>
</table>

## Library Integrations

crashcat is agnostic of rendering or game engine library, so it will work well with any other javascript libraries - three.js, babylon.js, playcanvas, or your own engine.

The examples use threejs for rendering, but the core crashcat apis are completely agnostic of any rendering or game engine libraries.

### Three.js Debug Renderer

crashcat provides a debug renderer for three.js via the `crashcat/three` package export. This is useful for visualizing physics simulation state during development.

The debug renderer uses batched rendering for efficiency, but visualizing many bodies, contacts, or constraints can still impact performance.

**Usage**

```ts
import type { World } from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import type * as THREE from 'three';

declare const scene: THREE.Scene;
declare const world: World;

// create debug renderer with default options
const options = debugRenderer.createDefaultOptions();
const state = debugRenderer.init(options);

// add to scene
scene.add(state.object3d);

// update each frame after physics step
function animate() {
    // ... update physics ...

    debugRenderer.update(state, world);

    // ... render scene ...
}

// customize what to visualize
const customOptions = debugRenderer.createDefaultOptions();

/* body visualization options */
customOptions.bodies.enabled = true;
customOptions.bodies.wireframe = false;
customOptions.bodies.showLinearVelocity = false;
customOptions.bodies.showAngularVelocity = false;

// unique color per body instance
customOptions.bodies.color = debugRenderer.BodyColorMode.INSTANCE;
// color by motion type (static, dynamic, kinematic)
customOptions.bodies.color = debugRenderer.BodyColorMode.MOTION_TYPE;
// color by sleeping state
customOptions.bodies.color = debugRenderer.BodyColorMode.SLEEPING;
// color by simulation island
customOptions.bodies.color = debugRenderer.BodyColorMode.ISLAND;

/* contact points options */
customOptions.contacts.enabled = true;

/* contact constraints options */
customOptions.contactConstraints.enabled = true;

/* constraints options (hinges, sliders, etc.) */
customOptions.constraints.enabled = true;
customOptions.constraints.drawLimits = true;
customOptions.constraints.size = 0.5;

/* broadphase options */
customOptions.broadphaseDbvt.enabled = false;
customOptions.broadphaseDbvt.showLeafNodes = true;
customOptions.broadphaseDbvt.showNonLeafNodes = true;

/* triangle mesh bvh options */
customOptions.triangleMeshBvh.enabled = false;
customOptions.triangleMeshBvh.showLeafNodes = true;
customOptions.triangleMeshBvh.showNonLeafNodes = true;
```

## FAQ

### When should I use crashcat over a WASM physics library?

crashcat is a good choice when:

- **Bundle size matters**: crashcat is pure JavaScript and highly tree-shakeable. WASM physics engines like Rapier or JoltPhysics.js add megabytes to your bundle, and WASM initialization can take tens of milliseconds.

- **Your simulation isn't extremely complex**: while crashcat cannot compete with optimized WASM engines for very large simulations, or engines that have multithreading capabilities, it can perform well for many scenarios, e.g. it can easily simulate hundreds of dynamic bodies at 60 Hz on a typical desktop browser, which is more than sufficient for many games and interactive experiences.

- **You need frequent JavaScript callbacks**: If your game logic heavily uses physics events (collision callbacks, contact modification, custom character behavior), crashcat is much faster. WASM → JavaScript boundary crossings are expensive, and libraries like Rapier can suffer significant performance hits when making many callbacks per frame.

- **You want simplicity**: crashcat is pure JavaScript - no manual memory management or awkward wasm APIs. Everything is just JavaScript objects you can inspect, debug, and serialize naturally.

**When to choose WASM instead**: If you need absolute maximum performance, and don't need to interact with / customize the simulation deeply, WASM engines can be a better choice. Although at a certain point, the stronger architecture would be to write an engine that can live entirely in WASM, rather than just having the physics in WASM and all other state in javascript.

### Can I use crashcat with [my favorite framework]?

Yes! crashcat is library-agnostic and will work well with three.js, babylon.js, playcanvas, or any other javascript library.

## Community

**webgamedev discord**

Join the webgamedev discord to discuss crashcat with other users and contributors, ask questions, and share your projects!

https://www.webgamedev.com/discord

## Acknowledgements

crashcat stands on the shoulders of giants! Many ideas and implementations were drawn from existing work:

- JoltPhysics: https://github.com/jrouwe/joltphysics (narrowphase, constraint solver design)
- Box2D: https://github.com/erincatto/box2d (contact management and solver design)
- Bullet Physics: https://github.com/bulletphysics/bullet3 (broadphase, dbvt)
- Bounce: https://codeberg.org/perplexdotgg/bounce (influenced crashcat's narrowphase design)
- three-mesh-bvh: https://github.com/gkjohnson/three-mesh-bvh (referenced for triangle mesh bvh building and traversal)
