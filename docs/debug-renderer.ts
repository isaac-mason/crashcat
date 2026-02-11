/* SNIPPET_START: imports */

import type { World } from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import type * as THREE from 'three';

/* SNIPPET_END: imports */

declare const scene: THREE.Scene;
declare const world: World;

/* SNIPPET_START: basic-usage */
// create debug renderer with default options
const options = debugRenderer.createDefaultOptions();
const state = debugRenderer.init(options);

// add to scene
scene.add(state.object3d);

// update each frame after physics step
function _animate() {
    // ... update physics ...

    debugRenderer.update(state, world);

    // ... render scene ...
}
/* SNIPPET_END: basic-usage */

/* SNIPPET_START: options */
// customize what to visualize
const customOptions = debugRenderer.createDefaultOptions();

// body visualization
customOptions.bodies.enabled = true;
customOptions.bodies.wireframe = false;
customOptions.bodies.color = debugRenderer.BodyColorMode.MOTION_TYPE;
customOptions.bodies.showLinearVelocity = false;
customOptions.bodies.showAngularVelocity = false;

// contact points
customOptions.contacts.enabled = true;

// contact constraints
customOptions.contactConstraints.enabled = true;

// constraints (hinges, sliders, etc.)
customOptions.constraints.enabled = true;
customOptions.constraints.drawLimits = true;
customOptions.constraints.size = 0.5;

// broadphase debug visualization
customOptions.broadphaseDbvt.enabled = false;
customOptions.broadphaseDbvt.showLeafNodes = true;
customOptions.broadphaseDbvt.showNonLeafNodes = true;

// triangle mesh bvh visualization
customOptions.triangleMeshBvh.enabled = false;
customOptions.triangleMeshBvh.showLeafNodes = true;
customOptions.triangleMeshBvh.showNonLeafNodes = true;
/* SNIPPET_END: options */

/* SNIPPET_START: color-modes */
// different body color modes for debugging

// unique color per body instance
customOptions.bodies.color = debugRenderer.BodyColorMode.INSTANCE;

// color by motion type (static, dynamic, kinematic)
customOptions.bodies.color = debugRenderer.BodyColorMode.MOTION_TYPE;

// color by sleeping state
customOptions.bodies.color = debugRenderer.BodyColorMode.SLEEPING;

// color by simulation island
customOptions.bodies.color = debugRenderer.BodyColorMode.ISLAND;
/* SNIPPET_END: color-modes */

/* SNIPPET_START: runtime-updates */
// options can be modified at runtime
state.options.bodies.wireframe = true;
state.options.contacts.enabled = true;
state.options.constraints.enabled = false;
/* SNIPPET_END: runtime-updates */
