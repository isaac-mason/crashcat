import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    compound,
    rigidBody,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    updateWorld,
    registerAllShapes,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 5, 15);
camera.lookAt(0, 2, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const onResize = () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
};

window.addEventListener('resize', onResize);
onResize();

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;
orbitControls.target.set(0, 2, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* physics world */

registerAllShapes();

const worldSettings = createWorldSettings();

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

// Create static ground
const groundShape = box.create({ halfExtents: [30, 0.5, 30] });
rigidBody.create(world, {
    shape: groundShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0,
    friction: 0.5,
});

/* create bodies demonstrating different mass properties */

const SPACING = 3;

// Body A: Simple box standing upright - stable
const boxShape = box.create({ halfExtents: [0.5, 1.5, 0.5], convexRadius: 0.05 });
rigidBody.create(world, {
    shape: boxShape,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-SPACING, 1.5, 0),
    quaternion: quat.create(),
    restitution: 0,
    friction: 0.5,
});

// Body B: Compound upside down L shape - will fall over due to high center of mass
// This demonstrates how inertia and center of mass affect stability
const lShapeCompound = compound.create({
    children: [
        // Vertical part (stem of the L)
        {
            position: vec3.fromValues(0, 0.75, 0),
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.25, 0.75, 0.25], convexRadius: 0.05 }),
        },
        // Horizontal part (top of the L) - offset to create asymmetry
        {
            position: vec3.fromValues(0.75, 1.25, 0),
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.75, 0.25, 0.25], convexRadius: 0.05 }),
        },
    ],
});

rigidBody.create(world, {
    shape: lShapeCompound,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(0, 1.5, 0),
    quaternion: quat.create(),
    restitution: 0,
    friction: 0.5,
});

// Body C: Double L shape (balanced) - two L shapes stacked to create symmetry
// This demonstrates how balanced mass distribution creates stability
const doubleLShapeCompound = compound.create({
    children: [
        // Bottom L - vertical stem
        {
            position: vec3.fromValues(0, 0.75, 0),
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.25, 0.75, 0.25], convexRadius: 0.05 }),
        },
        // Bottom L - horizontal top (right side)
        {
            position: vec3.fromValues(0.75, 1.25, 0),
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.75, 0.25, 0.25], convexRadius: 0.05 }),
        },
        // Top L (inverted) - vertical stem
        {
            position: vec3.fromValues(0, 2.25, 0),
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.25, 0.75, 0.25], convexRadius: 0.05 }),
        },
        // Top L (inverted) - horizontal bottom (left side) - mirrors bottom L
        {
            position: vec3.fromValues(-0.75, 1.75, 0),
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.75, 0.25, 0.25], convexRadius: 0.05 }),
        },
    ],
});

rigidBody.create(world, {
    shape: doubleLShapeCompound,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(SPACING, 1.5, 0),
    quaternion: quat.create(),
    restitution: 0,
    friction: 0.5,
});

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // Update physics with delta time
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    // Update orbit controls
    orbitControls.update();

    // Render
    renderer.render(scene, camera);
}

animate();
