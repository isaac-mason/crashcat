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
