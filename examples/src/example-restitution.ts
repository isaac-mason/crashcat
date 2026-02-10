import { vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    rigidBody,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    sphere,
    updateWorld,
    registerAll,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 8, 20);
camera.lookAt(0, 3, 0);

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
orbitControls.target.set(0, 3, 0);

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

// create static floor
const floorShape = box.create({ halfExtents: [15, 1, 10] });
rigidBody.create(world, {
    shape: floorShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    restitution: 0,
    friction: 0.5,
});

// create spheres with different restitution values
const restitutionValues = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
const spacing = 3;
const dropHeight = 5.0;

type BallData = {
    body: ReturnType<typeof rigidBody.create>;
    restitution: number;
};

const balls: BallData[] = [];

for (let i = 0; i < restitutionValues.length; i++) {
    const restitution = restitutionValues[i];
    const xPos = (i - (restitutionValues.length - 1) / 2) * spacing;

    // Create physics body
    const ballShape = sphere.create({ radius: 0.5 });
    const ball = rigidBody.create(world, {
        shape: ballShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(xPos, dropHeight, 0),
        restitution,
        friction: 0.5,
    });

    // Add label above sphere
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 32px monospace';
    context.textAlign = 'center';
    context.fillText(`e=${restitution.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(xPos, dropHeight + 1.5, 0);
    scene.add(sprite);

    balls.push({ body: ball, restitution });
}

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

/* simulation */

function resetBalls() {
    for (let i = 0; i < balls.length; i++) {
        const xPos = (i - (balls.length - 1) / 2) * spacing;
        balls[i].body.position = vec3.fromValues(xPos, dropHeight, 0);
        balls[i].body.motionProperties.linearVelocity = vec3.fromValues(0, 0, 0);
        balls[i].body.motionProperties.angularVelocity = vec3.fromValues(0, 0, 0);
    }
}

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

window.addEventListener('keydown', (e) => {
    if (e.key === 'r' || e.key === 'R') {
        resetBalls();
    }
});

animate();
