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

<Examples />

## Table of Contents

<TOC />

## Can crashcat be integrated with my engine/library?

crashcat is agnostic of rendering or game engine library, so it will work well with any other javascript libraries - three., babylon.js, playcanvas, or your own engine.

crashcat uses standard OpenGL conventions:
- uses the right-handed coordinate system (y-up)
- vectors are represented as `[x, y, z]` array tuples

if your environment uses a different coordinate system, you will need to transform coordinates going into and out of crashcat.

the examples use threejs for rendering, but the core crashcat apis are completely agnostic of any rendering or game engine libraries.

## Quick Start

below is a minimal example of creating a physics world with a static ground and some dynamic boxes:

<Snippet source="./quick-start.ts" />

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
