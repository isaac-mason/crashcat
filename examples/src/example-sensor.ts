import { vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    type Listener,
    rigidBody,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    updateWorld,
    registerAll,
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

registerAll();

const worldSettings = createWorldSettings();
worldSettings.gravity = vec3.fromValues(0, 0, 0);

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

(globalThis as any).world = world; // for debugging

// create kinematic sensor box that will side to side
const sensorBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [1.5, 1.0, 1.5] }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.KINEMATIC,
    position: vec3.fromValues(-8, 1, 0),
    sensor: true,
});

// create one dynamic box
rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-2, 1, 0),
    restitution: 0.3,
    friction: 0.5,
});

// create one static box
rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(2, 1, 0),
    restitution: 0.3,
    friction: 0.5,
});

// create label sprite above sensor
const canvas = document.createElement('canvas');
const context = canvas.getContext('2d')!;
canvas.width = 256;
canvas.height = 64;

const texture = new THREE.CanvasTexture(canvas);

function updateLabel(text: string, color: string) {
    context.fillStyle = '#1a1a1a';
    context.fillRect(0, 0, canvas.width, canvas.height);
    context.fillStyle = color;
    context.font = 'bold 32px monospace';
    context.textAlign = 'center';
    context.fillText(text, 128, 42);
    texture.needsUpdate = true;
}

updateLabel('CLEAR', '#4CAF50');

const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
const sprite = new THREE.Sprite(spriteMaterial);
sprite.scale.set(4, 1, 1);
scene.add(sprite);

/* contact listener */

let isIntersecting = false;

const listener: Listener = {
    onContactAdded: (bodyA, bodyB) => {
        if (bodyA.id === sensorBody.id || bodyB.id === sensorBody.id) {
            isIntersecting = true;
        }
    },
    onContactPersisted: (bodyA, bodyB) => {
        if (bodyA.id === sensorBody.id || bodyB.id === sensorBody.id) {
            isIntersecting = true;
        }
    },
};

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
ui.gui.add(sensorBody, 'collideKinematicVsNonDynamic').name('Collide with Non-Dynamic');
scene.add(debugRendererState.object3d);

/* simulation */

const maxDelta = 1 / 30;
let lastTime = performance.now();
let time = 0;

const sensorPosition = vec3.create();
const amplitude = 6.0;
const frequency = 1.0;

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    time += delta;

    // move sensor side to side
    const xPos = Math.sin(time * frequency) * amplitude;
    vec3.set(sensorPosition, xPos, 1, 0);
    rigidBody.setPosition(world, sensorBody, sensorPosition, false);

    // reset intersection flag before world update
    isIntersecting = false;

    debugUI.beginPerf(ui);
    updateWorld(world, listener, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // update label based on intersection state
    if (isIntersecting) {
        updateLabel('hit', '#4CAF50');
    } else {
        updateLabel('no hit', '#F44336');
    }

    // update sprite position to follow sensor
    sprite.position.set(xPos, 3.5, 0);

    debugRenderer.update(debugRendererState, world);

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
