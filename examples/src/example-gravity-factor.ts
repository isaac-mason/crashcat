import { vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAll,
    rigidBody,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 8, 15);
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

// set gravity to a standard value
vec3.set(worldSettings.gravity, 0, -9.81, 0);

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

(globalThis as any).world = world; // for debugging

// create static floor
const floorShape = box.create({ halfExtents: [20, 0.5, 10] });
rigidBody.create(world, {
    shape: floorShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0.3,
    friction: 0.5,
});

// create boxes with different gravity factors
// gravity factor scales the effect of world gravity on the body
// 0.0 = no gravity (floats), 1.0 = normal gravity, 2.0 = double gravity
const numBoxes = 11;
const gravityFactors = Array.from({ length: numBoxes }, (_, i) => i * 0.2); // 0.0 to 2.0
const spacing = 2.0;
const startX = -10.0;
const startHeight = 8.0;

type BoxData = {
    body: ReturnType<typeof rigidBody.create>;
    gravityFactor: number;
    initialPosition: [number, number, number];
    sprite: THREE.Sprite;
};

const boxes: BoxData[] = [];

for (let i = 0; i < numBoxes; i++) {
    const gravityFactor = gravityFactors[i];
    const xPos = startX + spacing * i;

    // create physics body
    const boxShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
    const boxBody = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(xPos, startHeight, 0),
        restitution: 0.3,
        friction: 0.5,
    });

    // set gravity factor
    boxBody.motionProperties.gravityFactor = gravityFactor;

    // add label above box showing gravity factor
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 24px monospace';
    context.textAlign = 'center';
    context.fillText(`g×${gravityFactor.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(xPos, startHeight + 1.5, 0);
    scene.add(sprite);

    boxes.push({
        body: boxBody,
        gravityFactor,
        initialPosition: [xPos, startHeight, 0],
        sprite,
    });
}

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

/* simulation */

const maxDelta = 1 / 30;
let lastTime = performance.now();
let resetTimer = 0;

function resetBoxes() {
    for (const box of boxes) {
        vec3.copy(box.body.position, box.initialPosition);
        vec3.set(box.body.motionProperties.linearVelocity, 0, 0, 0);
        vec3.set(box.body.motionProperties.angularVelocity, 0, 0, 0);
        box.sprite.position.set(box.initialPosition[0], box.initialPosition[1] + 1.5, box.initialPosition[2]);
    }
    resetTimer = 0;
}

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

    // update sprite positions to follow boxes
    for (const box of boxes) {
        box.sprite.position.set(box.body.position[0], box.body.position[1] + 1.5, box.body.position[2]);
    }

    // check if all boxes have settled (either stopped or fell off)
    const allSettled = boxes.every((box) => {
        const vy = Math.abs(box.body.motionProperties.linearVelocity[1]);
        const isOnFloor = box.body.position[1] < 1.5;
        const fellOff = box.body.position[1] < -5;
        return (isOnFloor && vy < 0.1) || fellOff;
    });

    if (allSettled) {
        resetTimer += delta;
        if (resetTimer > 2) {
            resetBoxes();
        }
    }

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
