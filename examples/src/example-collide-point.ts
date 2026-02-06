import { vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    collidePoint,
    createAllCollidePointCollector,
    createDefaultCollidePointSettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
    MotionType,
    registerAllShapes,
    rigidBody,
} from 'crashcat';
import * as debugUI from './debug/debug-ui';
import { debugRenderer } from 'crashcat/three';

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
worldSettings.gravity = vec3.fromValues(0, 0, 0);

const BROADPHASE_LAYER = addBroadphaseLayer(worldSettings);
const OBJECT_LAYER = addObjectLayer(worldSettings, BROADPHASE_LAYER);

enableCollision(worldSettings, OBJECT_LAYER, OBJECT_LAYER);

const world = createWorld(worldSettings);

(globalThis as any).world = world; // for debugging

/* create static box */

rigidBody.create(world, {
    shape: box.create({ halfExtents: [2.0, 2.0, 2.0] }),
    objectLayer: OBJECT_LAYER,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 2, 0),
});

/* create point visualization */

const pointGeometry = new THREE.SphereGeometry(0.2, 16, 16);
const pointMaterial = new THREE.MeshStandardMaterial({ color: 0xff00ff });
const pointMesh = new THREE.Mesh(pointGeometry, pointMaterial);
scene.add(pointMesh);

/* create label sprite */

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

/* collide point query */

const queryPoint = vec3.create();
const queryFilter = filter.create(worldSettings.layers);
const collector = createAllCollidePointCollector();

const settings = createDefaultCollidePointSettings();

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

/* simulation */

const maxDelta = 1 / 30;
let lastTime = performance.now();
let time = 0;

const amplitude = 6.0;
const frequency = 1.0;

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    time += delta;

    // move point side to side
    const xPos = Math.sin(time * frequency) * amplitude;
    vec3.set(queryPoint, xPos, 2, 0);

    // update point mesh position
    pointMesh.position.set(xPos, 2, 0);

    // reset collector before query
    collector.reset();

    // query point collision
    debugUI.beginPerf(ui);
    collidePoint(world, collector, settings, queryPoint, queryFilter);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // update label based on collision state
    const isColliding = collector.hits.length > 0;
    if (isColliding) {
        updateLabel('hit', '#4CAF50');
        pointMaterial.color.setHex(0x00ff00);
    } else {
        updateLabel('no hit', '#F44336');
        pointMaterial.color.setHex(0xff00ff);
    }

    // update sprite position to follow point
    sprite.position.set(xPos, 4.5, 0);

    debugRenderer.update(debugRendererState, world);

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
