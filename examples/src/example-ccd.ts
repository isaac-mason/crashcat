import { vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { World } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionQuality,
    MotionType,
    registerAllShapes,
    rigidBody,
    sphere,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(-31.96, 19.73, -27.86);
camera.lookAt(-0.0505, -0.4126, -0.0229);

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
orbitControls.target.set(-0.0505, -0.4126, -0.0229);

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

/* wall creation helper */

function createWall(world: World, offset: { x: number; y: number; z: number }, stackHeight: number, objectLayer: number): void {
    const shiftY = 1.0;
    const shiftZ = 2.0;

    for (let i = 0; i < stackHeight; i++) {
        for (let j = i; j < stackHeight; j++) {
            const x = offset.x;
            const y = i * shiftY + offset.y;
            const z = (i * shiftZ) / 2.0 + (j - i) * shiftZ + offset.z - stackHeight;

            // Create dynamic cube
            const shape = box.create({ halfExtents: [0.5, 0.5, 1.0] });
            rigidBody.create(world, {
                shape,
                objectLayer,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(x, y, z),
                restitution: 0,
                friction: 0.5,
            });
        }
    }
}

/* scene setup */

// create ground
const groundHeight = 0.1;
const groundShape = box.create({ halfExtents: [30.0, 0.1, 30.0] });
rigidBody.create(world, {
    shape: groundShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    restitution: 0,
    friction: 0.5,
});

// create walls
const numX = 5;
const numZ = 8;
const shiftY = groundHeight + 0.5;

for (let i = 0; i < numX; i++) {
    const x = i * 6.0;
    createWall(world, { x, y: shiftY, z: 0.0 }, numZ, OBJECT_LAYER_MOVING);
}

const fastBall = rigidBody.create(world, {
    shape: sphere.create({ radius: 1.0, density: 10000.0 }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-20.0, shiftY + 2.0, 0.0),
    restitution: 0,
    friction: 0.5,
    motionQuality: MotionQuality.LINEAR_CAST,
});

rigidBody.addImpulse(world, fastBall, [150_000_000, 0, 0]);

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

    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    debugRenderer.update(debugRendererState, world);

    orbitControls.update();

    renderer.render(scene, camera);
}

animate();
