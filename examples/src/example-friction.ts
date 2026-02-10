import { euler, quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    sphere,
    createWorld,
    createWorldSettings,
    dof,
    enableCollision,
    MotionType,
    rigidBody,
    updateWorld,
    registerAll,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 8, 25);
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

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

(globalThis as any).world = world; // for debugging

// Create static floor with high friction
const floorShape = box.create({ halfExtents: [50, 0.5, 300] });
rigidBody.create(world, {
    shape: floorShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0,
    friction: 1.0,
});

// Create boxes with different friction values
const numBoxes = 10;
const frictionValues = Array.from({ length: numBoxes }, (_, i) => i / (numBoxes - 1));
const spacing = 2.0;
const rowWidth = (numBoxes - 1) * spacing; // width of one row of shapes
const startHeight = 0.5;

type BodyData = {
    body: ReturnType<typeof rigidBody.create>;
    friction: number;
    initialPosition: [number, number, number];
};

const bodies: BodyData[] = [];

// Row 1: Spheres (no rotation restriction) - left of boxes
for (let i = 0; i < numBoxes; i++) {
    const friction = frictionValues[i];
    const xPos = -rowWidth / 2 - spacing - rowWidth + spacing * i; // left of boxes with gap
    const zPos = 0;

    const body = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(xPos, startHeight, zPos),
        restitution: 0,
        friction,
    });

    vec3.set(body.motionProperties.linearVelocity, 0, 0, 10);

    // add label above sphere
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 28px monospace';
    context.textAlign = 'center';
    context.fillText(`μ=${friction.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(xPos, startHeight + 1.5, zPos);
    scene.add(sprite);

    bodies.push({
        body,
        friction,
        initialPosition: [xPos, startHeight, zPos],
    });
}

// Row 2: Boxes with reduced degrees of freedom (no rotation) - centered
for (let i = 0; i < numBoxes; i++) {
    const friction = frictionValues[i];
    const xPos = -rowWidth / 2 + spacing * i; // centered at x=0
    const zPos = 0;

    const body = rigidBody.create(world, {
        shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(xPos, startHeight, zPos),
        restitution: 0,
        friction,
        allowedDegreesOfFreedom: dof(true, true, true, false, false, false),
    });

    vec3.set(body.motionProperties.linearVelocity, 0, 0, 10);

    // add label above box
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 28px monospace';
    context.textAlign = 'center';
    context.fillText(`μ=${friction.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(xPos, startHeight + 1.5, zPos);
    scene.add(sprite);

    bodies.push({
        body,
        friction,
        initialPosition: [xPos, startHeight, zPos],
    });
}

// Row 3: Capsules (rotated to lie on side so they roll) - right of boxes
const capsuleRotation = quat.fromEuler(quat.create(), euler.fromValues(0, 0, Math.PI / 2, 'zyx')); // rotate 90 degrees around Z
for (let i = 0; i < numBoxes; i++) {
    const friction = frictionValues[i];
    const xPos = rowWidth / 2 + spacing + spacing * i; // right of boxes with gap
    const zPos = 0;

    const body = rigidBody.create(world, {
        shape: capsule.create({ halfHeightOfCylinder: 0.3, radius: 0.5 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(xPos, startHeight, zPos),
        quaternion: capsuleRotation,
        restitution: 0,
        friction,
    });

    vec3.set(body.motionProperties.linearVelocity, 0, 0, 10);

    // add label above capsule
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 28px monospace';
    context.textAlign = 'center';
    context.fillText(`μ=${friction.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(xPos, startHeight + 1.5, zPos);
    scene.add(sprite);

    bodies.push({
        body,
        friction,
        initialPosition: [xPos, startHeight, zPos],
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
let frame = 0;

function resetBodies() {
    for (const data of bodies) {
        vec3.copy(data.body.position, data.initialPosition);
        vec3.set(data.body.motionProperties.linearVelocity, 0, 0, 10);
        vec3.set(data.body.motionProperties.angularVelocity, 0, 0, 0);
    }
}

resetBodies();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);
    frame++;

    debugRenderer.update(debugRendererState, world);

    const allStopped = bodies.every((data) => {
        const vz = Math.abs(data.body.motionProperties.linearVelocity[2]);
        return vz < 0.01 || data.body.position[2] > 15;
    });

    if (allStopped && frame > 60) {
        setTimeout(resetBodies, 1000);
    }

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
