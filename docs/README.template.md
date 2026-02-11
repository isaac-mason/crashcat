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

<Examples />

## Table of Contents

<TOC />

## Quick Start

Below is a minimal example of creating a physics world with a static ground and some dynamic boxes:

<Snippet source="./quick-start.ts" />

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

<Snippet source="./world-settings.ts" />

See the `WorldSettings` type for all available settings and their documentation: https://crashcat.dev/docs/types/crashcat.WorldSettings.html

### Stepping the Simulation

After creating a world, you advance the simulation by calling `updateWorld(world, listener, deltaTime)` in your game loop.

The `deltaTime` parameter is the time in seconds to advance the simulation. For a 60 FPS game loop, this is typically `1/60` (≈0.0167 seconds).

You can pass a listener to `updateWorld` to listen to and modify physics events, see the [Listener](#listener) section.

For simple use cases, you can use the frame delta time directly to do variable time stepping:

<Snippet source="./stepping.ts" select="variable-timestep" />

For maximum stability and determinism, use a fixed physics timestep with an accumulator:

<Snippet source="./stepping.ts" select="fixed-timestep" />

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

<Snippet source="./rigid-bodies.ts" select="creation" />

**⚠️ Storing references to bodies**

Bodies are pooled internally for performance. As such, be careful with storing long-lived references to body objects! Store `body.id` instead and use `rigidBody.get(world, id)` to look up bodies when needed.

Body ids contain an index and sequence number. When bodies are removed, their ids are invalidated and indices can be reused.

<Snippet source="./rigid-bodies.ts" select="object-pooling" />

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

<Snippet source="./rigid-bodies.ts" select="motion-types" />

### Position and Rotation

The position of a rigid body represents its location (translation) in 3d world-space. The quaternion represents its orientation (rotation).

Position and rotation can be set when creating a body, or modified after creation using the APIs below.

<Snippet source="./rigid-bodies.ts" select="transform" />

**position vs centerOfMassPosition:**

- `position`: Location of the shape's origin (where the collision shape is)
- `centerOfMassPosition`: Location of the body's center of mass in world-space

For simple shapes (sphere, box, capsule), these are the same. For compound shapes or shapes with offset center of mass, they differ. The physics engine uses `centerOfMassPosition` internally for simulation.

### Velocity

Linear and angular velocity can be read directly from rigid body objects. The `rigidBody` namespace provides APIs for modifying velocities.

<Snippet source="./rigid-bodies.ts" select="velocity" />

### Forces and Impulses

Forces accumulate until the next physics step, then get cleared. Impulses apply instant velocity changes. Use `addForceAtPosition` or `addImpulseAtPosition` to generate rotation.

<Snippet source="./rigid-bodies.ts" select="forces" />

<ExamplesTable ids="example-add-impulse-at-position" />

### Mass Properties

For most shapes, mass properties are computed automatically. For triangle meshes you need to provide them explicitly to use them with kinematic or static bodies.

<Snippet source="./rigid-bodies.ts" select="mass" />

<ExamplesTable ids="example-mass-properties" />

### Damping

Damping simulates air resistance or drag. Higher values make objects slow down faster.

<Snippet source="./rigid-bodies.ts" select="damping" />

<ExamplesTable ids="example-linear-damping,example-angular-damping" />

### Maximum Velocities

Clamping velocities prevents instability and tunneling from extreme speeds.

<Snippet source="./rigid-bodies.ts" select="max-velocities" />

### Degrees of Freedom

Restricting degrees of freedom is useful for 2D games or objects that should only move on specific axes.

<Snippet source="./rigid-bodies.ts" select="dof" />

### Sleeping

Sleeping improves performance by skipping simulation for bodies at rest.

<Snippet source="./rigid-bodies.ts" select="sleeping" />

You can also wake all sleeping bodies within a specific region:

<Snippet source="./rigid-bodies.ts" select="wake-in-aabb" />

### Gravity Factor

Gravity factor multiplies the world gravity for a specific body. Set to 0 for floating objects, less than 1 for lighter-than-normal gravity, or greater than 1 for heavier gravity.

<Snippet source="./rigid-bodies.ts" select="gravity-factor" />

<ExamplesTable ids="example-gravity-factor" />

### Moving Kinematic Bodies

`moveKinematic` takes a target position and quaternion, and computes the velocities needed to reach them, ensuring physical interactions with dynamic bodies rather than a direct teleportation.

Prefer using `moveKinematic` over `setTransform` for kinematic bodies such as moving platforms.

<Snippet source="./rigid-bodies.ts" select="move-kinematic" />

<ExamplesTable ids="example-kinematic" />

### Continuous Collision Detection

Use CCD for fast-moving objects like bullets or vehicles to prevent tunneling through thin walls.

<Snippet source="./rigid-bodies.ts" select="ccd" />

<ExamplesTable ids="example-ccd" />

### User Data

User data lets you attach game-specific data to bodies for easy lookup during collision callbacks.

<Snippet source="./rigid-bodies.ts" select="userdata" />

### Collision Groups and Masks

Collision groups and masks provide fine-grained collision filtering using 32-bit bitmasks. This works alongside object layer filtering - both must pass for bodies to collide.

A collision occurs when `(groupA & maskB) != 0 AND (groupB & maskA) != 0`. Use the `bitmask` helper to define named groups.

<Snippet source="./rigid-bodies.ts" select="collision-groups" />

<ExamplesTable ids="example-collision-filtering" />

### Material Properties

Friction and restitution control surface interaction. Combine modes determine how material properties mix when two bodies collide.

<Snippet source="./rigid-bodies.ts" select="material" />

<ExamplesTable ids="example-friction,example-restitution" />

### Sensors

Sensor bodies detect collisions without applying physical forces. Use them for trigger zones, pickups, or detection areas.

<Snippet source="./rigid-bodies.ts" select="sensor" />

<ExamplesTable ids="example-sensor" />

### Updating Shape

You can change a body's shape after creation. This recalculates mass properties, inertia, and the axis-aligned bounding box.

<Snippet source="./rigid-bodies.ts" select="update-shape" />

### Changing Object Layer

Object layers control which bodies can collide. You can change a body's layer at runtime to modify collision behavior.

<Snippet source="./rigid-bodies.ts" select="set-object-layer" />

### Changing Motion Type

You can change a body's motion type at runtime to switch between static, kinematic, and dynamic behavior.

<Snippet source="./rigid-bodies.ts" select="set-motion-type" />

## Shapes

Shapes determine how rigid bodies collide with each other. crashcat provides primitive shapes, complex shapes like triangle meshes, and decorator shapes for advanced use cases.

<ExamplesTable ids="example-shapes" />

### Convex Shapes

A convex shape is one where, if you pick any two points inside the shape, the line segment between them is also inside the shape. This property enables fast collision detection with the GJK/EPA algorithms.

To speed up collision detection, all convex shapes use a convex radius. The shape is first shrunk by the convex radius, then inflated again by the same amount, resulting in a rounded shape.

This rounding improves performance and contact manifold quality, but makes geometry slightly less accurate. Adjust the radius to balance speed vs precision.

#### Sphere

The simplest and fastest convex shape.

<Snippet source="./shapes.ts" select="sphere" />

#### Box

Defined by half extents from the center.

<Snippet source="./shapes.ts" select="box" />

#### Capsule

A cylinder with hemispherical caps on each end.

<Snippet source="./shapes.ts" select="capsule" />

#### Cylinder

Defined by half height and radius.

<Snippet source="./shapes.ts" select="cylinder" />

#### Convex Hull

The convex hull of a set of points.

<Snippet source="./shapes.ts" select="convex-hull" />

### Triangle Mesh Shape

Triangle meshes represent complex geometry using triangles. Typically used for static terrain and level geometry.

<Snippet source="./shapes.ts" select="triangle-mesh" />

**⚠️ Dynamic triangle meshes**

Avoid using triangle meshes for dynamic bodies. Performance is poor (collision detection against triangle meshes is usually more expensive), and fast-moving meshes can tunnel through other objects easily. Use convex hulls or compound shapes instead for dynamic objects.

### Compound Shape

Compound shapes combine multiple child shapes into a single shape. Useful for complex objects like vehicles or characters.

<Snippet source="./shapes.ts" select="compound" />

### Decorator Shapes

Decorator shapes modify other shapes without changing their collision shape.

#### Scaled

Apply non-uniform scaling to any shape.

<Snippet source="./shapes.ts" select="scaled" />

#### Offset Center of Mass

Shift the center of mass without changing collision shape. Useful for improving stability of tall objects.

<Snippet source="./shapes.ts" select="offset-center-of-mass" />

### Reusing Shapes

Shapes can be created once and reused across multiple bodies. This saves memory and improves performance.

<Snippet source="./shapes.ts" select="reuse" />

### Offline Shape Generation

Shapes are JSON-serializable objects. You can generate complex shapes offline (especially triangle meshes, which perform sanitization, active edge computation, and BVH construction) and load the JSON at runtime.

<Snippet source="./shapes.ts" select="offline" />

## Listener

The listener lets you react to and modify physics events during world updates. Pass a listener to `updateWorld()` to receive callbacks for collision events.

<ExamplesTable ids="example-sensor,example-conveyor-belt" />

### Basic Usage

<Snippet source="./listeners.ts" select="basic" />

### Body Pair Validation

Runs before expensive narrowphase collision detection. Use this when filtering logic is too complex for object layers or collision groups/masks (which are faster). Prefer those simpler mechanisms when possible.

<Snippet source="./listeners.ts" select="body-pair-validate" />

### Contact Validation

Called after collision detection but before adding the contact constraint. Rejecting contacts here is expensive since narrowphase has already run - prefer `onBodyPairValidate` or object layer filtering where possible. Use this for special cases where you use the contact information (contact point, normal, etc) to make a decision, such as one-way platforms or material-based effects.

<Snippet source="./listeners.ts" select="contact-validate" />

### Modifying Contact Behavior

Adjust friction, restitution, and other properties for specific contacts.

<Snippet source="./listeners.ts" select="modify-contact" />

## Queries

Queries let you ask questions about the physics world without running a full simulation step. Use them for raycasts, shape sweeps, overlap tests, and more.

### Cast Ray

Cast a ray through the world to find bodies along a line. Useful for line-of-sight checks, projectile trajectories, and mouse picking.

<Snippet source="./queries.ts" select="cast-ray" />

<ExamplesTable ids="example-cast-ray" />

### Cast Shape

Sweep a shape through the world to find what it would hit. Essential for character movement, projectile prediction, and object placement.

<Snippet source="./queries.ts" select="cast-shape" />

<ExamplesTable ids="example-cast-shape" />

### Collide Point

Test if a point is inside any bodies. Useful for trigger zones, item pickups, and spatial checks.

<Snippet source="./queries.ts" select="collide-point" />

<ExamplesTable ids="example-collide-point" />

### Collide Shape

Test if a shape overlaps any bodies. Perfect for area triggers, placement validation, and explosion radius checks.

<Snippet source="./queries.ts" select="collide-shape" />

<ExamplesTable ids="example-collide-shape" />

### Broadphase Queries

For advanced scenarios, you can query the broadphase spatial acceleration structure directly. This is faster than narrowphase queries but less precise - it only tests axis-aligned bounding boxes (AABBs), not exact shapes.

<Snippet source="./queries.ts" select="broadphase-query" />

### Shape vs Shape

For advanced scenarios, you can query shape-vs-shape directly.

<ExamplesTable ids="example-collide-shape-vs-shape,example-collide-point-vs-shape,example-cast-ray-vs-shape,example-cast-shape-vs-shape" />

### Query Filters

Filters control what queries can hit using object layers, broadphase layers, collision groups/masks, and custom callbacks.

Filters apply three levels of filtering in order:

1. **Object/Broadphase Layers**: Fast spatial partitioning (configured in world settings)
2. **Collision Groups/Masks**: Bitwise filtering using 32-bit masks (checks against `rigidBody.collisionGroups` and `rigidBody.collisionMasks`)
3. **Body Filter Callback**: Custom logic for complex filtering (slowest, use sparingly)

All three must pass for a query to pass for a body.

<Snippet source="./queries.ts" select="filter" />

<RenderType type="import('crashcat').Filter" />

## Constraints

<ExamplesTable ids="example-constraints,example-constraints-motors,example-hinge-motor,example-ragdoll" />

Constraints connect bodies together to create complex mechanical systems like ragdolls, vehicles, and articulated structures. crashcat supports 8 constraint types ranging from simple connections to fully configurable constraints.

### Creating and Removing Constraints

<Snippet source="./constraints.ts" select="basic" />

### Constraint Types

crashcat supports the following constraint types:

<Snippet source="./constraints.ts" select="types" />

**Available Constraints**:

- **PointConstraint**: Connects two bodies at a point (removes 3 DOF). Like a ball-and-socket.
- **DistanceConstraint**: Maintains distance between two points (removes 1 DOF). Like a rope or stick.
- **HingeConstraint**: Allows rotation around an axis (removes 5 DOF). Like a door hinge or wheel axle.
- **SliderConstraint**: Allows movement along an axis (removes 5 DOF). Like a piston or rail.
- **FixedConstraint**: Completely locks two bodies together (removes 6 DOF). Like welding.
- **ConeConstraint**: Limits rotation within a cone (removes 3 DOF). Like a shoulder.
- **SwingTwistConstraint**: Approximates shoulder-like movement with swing and twist limits.
- **SixDOFConstraint**: Most configurable - specify limits per translation/rotation axis.

### Constraint Motors

Some constraints support motors that apply forces/torques to drive bodies to a target velocity or position. There are two motor types:

**Velocity Motors**

Drive bodies to a constant relative velocity. For hinges this is angular velocity (rad/s), for sliders it's linear velocity (m/s).

<Snippet source="./constraints.ts" select="motors-velocity" />

**Position Motors**

Drive bodies to a target angle (hinges) or position (sliders) using a spring. The spring has two parameters:

- **Frequency**: How fast it reaches the target (Hz). Higher = stiffer spring. Valid range: (0, 0.5 × simulation frequency]. For 60 Hz physics, 20 Hz is stiff, 2 Hz is soft.
- **Damping**: Prevents overshoot. 0 = oscillates forever, 1 = critical damping (no overshoot), >1 = overdamped (slower).

<Snippet source="./constraints.ts" select="motors-position" />

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

<Snippet source="./constraints.ts" select="limits" />

### Local vs World Space

Constraint attachment points can be specified in world space or local space:

<Snippet source="./constraints.ts" select="local-vs-world" />

## Character Controllers

Character controllers handle player and NPC movement with features like ground detection, slope handling, and stair stepping. crashcat provides two approaches:

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

<ExamplesTable ids="example-kcc" />

### Dynamic Character Controllers

For characters that should behave like physics objects (ragdolls, physics-based characters, simple AI), you can build a controller using a regular dynamic rigid body with constraints on rotation to keep it upright.

This approach can be cheaper to simulate, but has tradeoffs in precise control over movement behavior. It can work well for AI characters or situations where you want realistic physical reactions.

The below example shows how you can create a floating capsule character controller in user-land. This can be copy/pasted into your project as a starting place and customized as needed.

<ExamplesTable ids="example-floating-character-controller" />

## Multiple Physics Worlds

The shape and constraints registry is global in crashcat, but you can create as many independent physics worlds as you need.

This can be useful in some advanced scenarios, e.g. for space games, where a spaceship and it's movement might be simulated in one world, but the movement of characters inside the ship is simulated in another world with different gravity and scale.

## World State Serialization

A physics world in crashcat is a simple JSON-serializable object. If need be, you can JSON.stringify and JSON.parse with the entire world state, including bodies, shapes, constraints, and settings. This can be useful for saving/loading game state for debugging, or more advanced use cases. Note that object references for e.g. sharing shapes across bodies will of course not survive serialization.

## Tree Shaking

crashcat is built to be highly tree-shakeable. By selectively registering only the shapes and constraints you need, modern bundlers can eliminate unused code and significantly reduce your bundle size.

### Using `registerAll`

The simplest approach is to use `registerAll()`, which registers all built-in shapes and constraints. This is convenient but includes everything in your bundle:

<Snippet source="./tree-shaking.ts" select="register-all" />

### Using `registerShapes` and `registerConstraints`

During development, it can be easier to use `registerAll()` while exploring what shapes and constraints you need. Once your usage is more finalized, you can switch to selective registration reduce bundle size significantly.

You must call `registerShapes()` or `registerConstraints()` before creating any bodies or constraints using those types. If you try to create a body with an unregistered shape, the shapes will effectively be "empty" shapes that have no collision

<Snippet source="./tree-shaking.ts" select="register-selective" />

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

**Solution**: Use a scaling factor between physics and rendering:

- Physics: Use meters (e.g., player capsule with `radius: 0.5`, `halfHeightOfCylinder: 1` = 3m tall human)
- Rendering: Scale up for pixels (multiply physics positions by scale factor, e.g., `position * 50` for 50 pixels per meter)

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

<ExamplesTable ids="example-voxel-custom-shape" />

## Library Integrations

crashcat is agnostic of rendering or game engine library, so it will work well with any other javascript libraries - three.js, babylon.js, playcanvas, or your own engine.

The examples use threejs for rendering, but the core crashcat apis are completely agnostic of any rendering or game engine libraries.

### Three.js Debug Renderer

crashcat provides a debug renderer for three.js via the `crashcat/three` package export. This is useful for visualizing physics simulation state during development.

**Basic Usage**

<Snippet source="./debug-renderer.ts" select="basic-usage" />

**Options**

The debug renderer supports visualizing various aspects of the physics simulation:

<Snippet source="./debug-renderer.ts" select="options" />

**Body Color Modes**

Different color modes help visualize different aspects of the simulation:

<Snippet source="./debug-renderer.ts" select="color-modes" />

**Runtime Updates**

Debug renderer options can be modified at runtime to toggle different visualizations on and off:

<Snippet source="./debug-renderer.ts" select="runtime-updates" />

**Performance Considerations**

The debug renderer uses batched rendering for efficiency, but visualizing many bodies, contacts, or constraints can still impact performance.

For production builds, consider conditionally excluding the debug renderer from your bundle using tree-shaking.

## FAQ

### When should I use crashcat over a WASM physics library?

crashcat is a good choice when:

- **Bundle size matters**: crashcat is pure JavaScript and highly tree-shakeable. WASM physics engines like Rapier or JoltPhysics.js add megabytes to your bundle, and WASM initialization can take tens of milliseconds.

- **Your simulation isn't extremely complex**: while crashcat cannot compete with optimized WASM engines for very large simulations, or engines that have multithreading capabilities, it can perform well for many scenarios, e.g. it can easily simulate hundreds of dynamic bodies at 60 Hz on a typical desktop browser, which is more than sufficient for many games and interactive experiences.

- **You need frequent JavaScript callbacks**: If your game logic heavily uses physics events (collision callbacks, contact modification, custom character behavior), crashcat is much faster. WASM → JavaScript boundary crossings are expensive, and libraries like Rapier can suffer significant performance hits when making many callbacks per frame.

- **You want simplicity**: crashcat is pure JavaScript - no manual memory management or awkward wasm APIs. Everything is just JavaScript objects you can inspect, debug, and serialize naturally.

**When to choose WASM instead**: If you need absolute maximum performance for very large simulations (10k+ bodies), and don't need to interact with / customize the simulation deeply, WASM engines can be a better choice. Although at a certain point, the stronger architecture would be to write an engine that can live entirely in WASM, rather than just having the physics in WASM and all other state in javascript.

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
