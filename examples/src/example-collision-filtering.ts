import { vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    bitmask,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAllShapes,
    rigidBody,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(10, 5, 10);
camera.lookAt(0, 0, 0);

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
orbitControls.target.set(0, 0, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* physics world */

registerAllShapes();

const worldSettings = createWorldSettings();
worldSettings.gravity = vec3.fromValues(0, -9.81, 0);

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

(globalThis as any).world = world; // for debugging

/* collision groups */

const COLLISION_GROUPS = bitmask.createFlags(['a', 'b'] as const);

// group a: cubes that fall onto lower platform
const GROUP_A = COLLISION_GROUPS.a;
const MASK_A = COLLISION_GROUPS.a;

// group b: cubes that fall onto upper platform
const GROUP_B = COLLISION_GROUPS.b;
const MASK_B = COLLISION_GROUPS.b;

/* create ground */

rigidBody.create(world, {
    shape: box.create({ halfExtents: [5.0, 0.1, 5.0] }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    friction: 0.5,
});

/* create floating platforms */

// platform 1 (lower) - only collides with group a
rigidBody.create(world, {
    shape: box.create({ halfExtents: [1.0, 0.1, 1.0] }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 1.0, 0),
    collisionGroup: GROUP_A,
    collisionMask: MASK_A,
    friction: 0.5,
});

// platform 2 (upper) - only collides with group b
rigidBody.create(world, {
    shape: box.create({ halfExtents: [1.0, 0.1, 1.0] }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 2.0, 0),
    collisionGroup: GROUP_B,
    collisionMask: MASK_B,
    friction: 0.5,
});

/* create dynamic cubes */

const num = 4;
const rad = 0.1;
const shift = rad * 2.0;
const centerx = shift * (num / 2);
const centery = 2.5;
const centerz = shift * (num / 2);

for (let j = 0; j < 4; j++) {
    for (let i = 0; i < num; i++) {
        for (let k = 0; k < num; k++) {
            const x = i * shift - centerx;
            const y = j * shift + centery;
            const z = k * shift - centerz;

            // Alternate between group a and group b based on z position
            const isGroupA = k % 2 === 0;
            const collisionGroup = isGroupA ? GROUP_A : GROUP_B;
            const collisionMask = isGroupA ? MASK_A : MASK_B;

            rigidBody.create(world, {
                shape: box.create({ halfExtents: [rad, rad, rad] }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(x, y, z),
                collisionGroup,
                collisionMask,
                friction: 1,
                restitution: 0.0,
            });
        }
    }
}

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

/* simulation */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    debugRenderer.update(debugRendererState, world);

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
