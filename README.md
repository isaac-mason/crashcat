![cover](./docs/cover.png)

[![Version](https://img.shields.io/npm/v/crashcat?style=for-the-badge)](https://www.npmjs.com/package/crashcat)
![GitHub Workflow Status (with event)](https://img.shields.io/github/actions/workflow/status/isaac-mason/crashcat/main.yml?style=for-the-badge)
[![Downloads](https://img.shields.io/npm/dt/crashcat.svg?style=for-the-badge)](https://www.npmjs.com/package/crashcat)

```bash
> npm install crashcat
```

# crashcat

crashcat is a high-performance 3D physics engine for javascript, built for games and simulations.

crashcat is ideal for use in games, simulations, and creative websites that require realistic physics simulation in complex 3D environments.

**Features**

- 🎯 rigid body dynamics with multiple motion types (dynamic, kinematic, static)
- 💥 comprehensive collision detection and response
- 📦 rich set of primitive and complex shapes (box, sphere, capsule, cylinder, convex hull, triangle mesh, compound shapes)
- 🔗 constraint system with motors and springs (hinge, slider, distance, point, fixed, cone, swing-twist, six-dof)
- ⚡ continuous collision detection (ccd) for fast-moving objects
- 🌳 broadphase spatial acceleration with dynamic bvh
- 🎚️ collision filtering with object layers and broadphase layers
- 😴 sleeping/activation system for performance
- 👻 sensor bodies for trigger volumes
- 🌲 pure javascript, written to be highly tree-shakeable
- 🔌 works with any javascript engine/library - babylon.js, playcanvas, three.js, or your own engine

**Documentation**

this readme provides curated explanations, guides, and examples to help you get started with crashcat.

api documentation can be found at [crashcat.dev/docs](https://crashcat.dev/docs).

**Changelog**

see the [changelog.md](./changelog.md) for a detailed list of changes in each version.

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
      <a href="https://crashcat.dev/examples#example-pointer-ray">
        <img src="./examples/public/screenshots/example-pointer-ray.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Pointer Raycast
      </a>
    </td>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-cluster">
        <img src="./examples/public/screenshots/example-cluster.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Cluster
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-ccd">
        <img src="./examples/public/screenshots/example-ccd.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Continuous Collision Detection
      </a>
    </td>
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
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-constraints-motors">
        <img src="./examples/public/screenshots/example-constraints-motors.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Constraint Motors
      </a>
    </td>
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
  </tr>
  <tr>
    <td align="center">
      <a href="https://crashcat.dev/examples#example-voxel-custom-shape">
        <img src="./examples/public/screenshots/example-voxel-custom-shape.png" width="180" height="120" style="object-fit:cover;"/><br/>
        Voxel Custom Shape
      </a>
    </td>
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

- [Can crashcat be integrated with my engine/library?](#can-crashcat-be-integrated-with-my-enginelibrary)
- [Quick Start](#quick-start)
- [Community](#community)
- [Acknowledgements](#acknowledgements)

## Can crashcat be integrated with my engine/library?

crashcat is agnostic of rendering or game engine library, so it will work well with any other javascript libraries - three., babylon.js, playcanvas, or your own engine.

crashcat uses standard OpenGL conventions:
- uses the right-handed coordinate system (y-up)
- vectors are represented as `[x, y, z]` array tuples

if your environment uses a different coordinate system, you will need to transform coordinates going into and out of crashcat.

the examples use threejs for rendering, but the core crashcat apis are completely agnostic of any rendering or game engine libraries.

## Quick Start

below is a minimal example of creating a physics world with a static ground and some dynamic boxes:

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
// a "broadphase layer" has it's own broadphase tree, so defining layers can be a good way to split up the broadphase tree for better performance.
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
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    shape: box.create({ halfExtents: [10, 1, 10] }),
});

// create a stack of dynamic boxes
for (let i = 0; i < 5; i++) {
    rigidBody.create(world, {
        motionType: MotionType.DYNAMIC,
        objectLayer: OBJECT_LAYER_MOVING,
        shape: box.create({ halfExtents: [1, 1, 1] }),
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
```

## Community

**webgamedev discord**

Join the webgamedev discord to discuss crashcat with other users and contributors, ask questions, and share your projects!

https://www.webgamedev.com/discord

## Acknowledgements

This library stands on the shoulders of giants! Many ideas and inspirations were drawn from existing work, particularly:
- JoltPhysics: https://github.com/jrouwe/joltphysics (narrowphase, constraint solver design)
- Box2D: https://github.com/erincatto/box2d (contact management and solver design)
- Bullet Physics: https://github.com/bulletphysics/bullet3 (broadphase, dbvt)
- Bounce: https://codeberg.org/perplexdotgg/bounce (influenced crashcat's narrowphase design)
