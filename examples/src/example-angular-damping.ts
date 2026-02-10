import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
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
camera.position.set(0, 0, 25);
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

registerAll();

const worldSettings = createWorldSettings();

worldSettings.gravity = vec3.fromValues(0, 0, 0);

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

const world = createWorld(worldSettings);

(globalThis as any).world = world; // for debugging

// Create boxes with different angular damping values
const numBoxes = 10;
const dampingValues = Array.from({ length: numBoxes }, (_, i) => i * 0.1); // 0.0 to 0.9
const spacing = 3.0;
const startX = -13.5;
const startY = 0;

type BoxData = {
    body: ReturnType<typeof rigidBody.create>;
    damping: number;
    initialPosition: [number, number, number];
};

const boxes: BoxData[] = [];

for (let i = 0; i < numBoxes; i++) {
    const damping = dampingValues[i];
    const xPos = startX + spacing * i;

    // Create physics body
    const boxShape = box.create({ halfExtents: [0.75, 0.75, 0.75] });
    const boxBody = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(xPos, startY, 0),
        angularDamping: damping,
    });

    // Apply initial angular velocity to make it spin
    vec3.set(boxBody.motionProperties.angularVelocity, 3, 5, 2);

    // Add label above box
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 28px monospace';
    context.textAlign = 'center';
    context.fillText(`ω=${damping.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(xPos, startY + 2, 0);
    scene.add(sprite);

    boxes.push({
        body: boxBody,
        damping,
        initialPosition: [xPos, startY, 0],
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

function resetBoxes() {
    for (const box of boxes) {
        vec3.copy(box.body.position, box.initialPosition);
        vec3.set(box.body.motionProperties.linearVelocity, 0, 0, 0);
        vec3.set(box.body.motionProperties.angularVelocity, 3, 5, 2);
        quat.set(box.body.quaternion, 0, 0, 0, 1);
    }
}

resetBoxes();

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

    // Check if all boxes have stopped spinning
    const allStopped = boxes.every((box) => {
        const angularVel = box.body.motionProperties.angularVelocity;
        const speed = Math.sqrt(angularVel[0] ** 2 + angularVel[1] ** 2 + angularVel[2] ** 2);
        return speed < 0.05;
    });

    if (allStopped && frame > 60) {
        setTimeout(resetBoxes, 1000);
    }

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
