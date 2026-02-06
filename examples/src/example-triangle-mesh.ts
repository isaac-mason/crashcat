import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import type { RigidBody, Shape } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAllShapes,
    rigidBody,
    sphere,
    triangleMesh,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(20, 15, -20);
camera.lookAt(0, 5, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
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
orbitControls.target.set(0, 5, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(20, 30, 15);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 2048;
directionalLight.shadow.mapSize.height = 2048;
directionalLight.shadow.camera.left = -50;
directionalLight.shadow.camera.right = 50;
directionalLight.shadow.camera.top = 50;
directionalLight.shadow.camera.bottom = -50;
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

/* load jungle scene as triangle mesh collider */

const gltfLoader = new GLTFLoader();
let jungleSceneMesh: THREE.Group;

async function loadJungleScene() {
    const gltf = await gltfLoader.loadAsync('./models/low_poly_environment_jungle_scene.glb');
    jungleSceneMesh = gltf.scene;
    jungleSceneMesh.position.set(0, 0, 0);

    // scene.add(jungleSceneMesh);

    // extract geometry for physics collider
    const allPositions: number[] = [];
    const allIndices: number[] = [];
    let indexOffset = 0;

    jungleSceneMesh.traverse((child) => {
        if (child instanceof THREE.Mesh) {
            const geometry = child.geometry;

            const positions = geometry.getAttribute('position');
            if (!positions) return;

            const worldMatrix = child.matrixWorld;

            const vertex = new THREE.Vector3();
            for (let i = 0; i < positions.count; i++) {
                vertex.fromBufferAttribute(positions, i);
                vertex.applyMatrix4(worldMatrix);
                allPositions.push(vertex.x, vertex.y, vertex.z);
            }

            const indices = geometry.getIndex();
            if (indices) {
                for (let i = 0; i < indices.count; i++) {
                    allIndices.push(indices.getX(i) + indexOffset);
                }
            } else {
                for (let i = 0; i < positions.count; i++) {
                    allIndices.push(i + indexOffset);
                }
            }

            indexOffset += positions.count;
        }
    });

    // create triangle mesh collider
    const jungleSceneShape = triangleMesh.create({
        positions: allPositions,
        indices: allIndices,
    });

    // create static physics body for the jungle scene
    rigidBody.create(world, {
        shape: jungleSceneShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 0, 0),
        restitution: 0.2,
        friction: 0.7,
    });
}

/* shape spawning system */

type ShapeConfig = {
    name: string;
    createShape: () => Shape;
    color: number;
};

const shapeConfigs: ShapeConfig[] = [
    {
        name: 'sphere',
        createShape: () => sphere.create({ radius: 0.5 }),
        color: 0xff6b6b,
    },
    {
        name: 'box',
        createShape: () => box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 }),
        color: 0x4ecdc4,
    },
];

type DynamicShape = {
    body: RigidBody;
    config: ShapeConfig;
};

const dynamicShapes: DynamicShape[] = [];

const settings = {
    numberOfBodies: 30,
    respawnIntervalMs: 500,
};

const SPAWN_HEIGHT = 5;
const SPAWN_AREA = 5; // spawn within +/- 15 units on x and z

function randomInRange(min: number, max: number): number {
    return Math.random() * (max - min) + min;
}

function spawnShape(config: ShapeConfig): void {
    const shape = config.createShape();

    // random position above the ground
    const x = randomInRange(-SPAWN_AREA, SPAWN_AREA);
    const z = randomInRange(-SPAWN_AREA, SPAWN_AREA);
    const position = vec3.fromValues(x, SPAWN_HEIGHT, z);

    // random rotation
    const axis = vec3.fromValues(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
    vec3.normalize(axis, axis);
    const angle = Math.random() * Math.PI * 2;
    const rotation = quat.create();
    quat.setAxisAngle(rotation, axis, angle);

    const body = rigidBody.create(world, {
        shape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position,
        quaternion: rotation,
        restitution: 0.4,
        friction: 0.6,
    });

    dynamicShapes.push({
        body,
        config,
    });
}

function updateBodyCount() {
    const targetCount = settings.numberOfBodies;
    const currentCount = dynamicShapes.length;

    if (currentCount < targetCount) {
        const toSpawn = targetCount - currentCount;
        for (let i = 0; i < toSpawn; i++) {
            const config = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
            spawnShape(config);
        }
    } else if (currentCount > targetCount) {
        const toRemove = currentCount - targetCount;
        for (let i = 0; i < toRemove; i++) {
            const shape = dynamicShapes.pop();
            if (shape) {
                rigidBody.remove(world, shape.body);
            }
        }
    }
}

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

ui.gui
    .add(settings, 'numberOfBodies', 0, 200, 1)
    .name('Number of Bodies')
    .onChange(() => {
        updateBodyCount();
    });
ui.gui.add(settings, 'respawnIntervalMs', 100, 5000, 100).name('Respawn Interval (ms)');

const maxDelta = 1 / 30;
let lastTime = performance.now();
let lastRespawnTime = performance.now();
let respawnIndex = 0;

async function init() {
    // load the jungle scene first
    await loadJungleScene();

    // initial spawn
    updateBodyCount();

    // start animation loop
    animate();
}

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // update physics with delta time
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // round-robin respawning based on interval
    if (dynamicShapes.length > 0 && currentTime - lastRespawnTime >= settings.respawnIntervalMs) {
        lastRespawnTime = currentTime;

        // get the next shape to respawn in round-robin fashion
        const shapeToRespawn = dynamicShapes[respawnIndex % dynamicShapes.length];

        // remove old body properly
        rigidBody.remove(world, shapeToRespawn.body);

        // create new body with random shape type
        const newConfig = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
        const newShape = newConfig.createShape();

        const x = randomInRange(-SPAWN_AREA, SPAWN_AREA);
        const z = randomInRange(-SPAWN_AREA, SPAWN_AREA);
        const position = vec3.fromValues(x, SPAWN_HEIGHT, z);

        const axis = vec3.fromValues(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
        vec3.normalize(axis, axis);
        const angle = Math.random() * Math.PI * 2;
        const rotation = quat.create();
        quat.setAxisAngle(rotation, axis, angle);

        const newBody = rigidBody.create(world, {
            shape: newShape,
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position,
            quaternion: rotation,
            restitution: 0.4,
            friction: 0.6,
        });

        // replace the body in the array
        shapeToRespawn.body = newBody;
        shapeToRespawn.config = newConfig;

        respawnIndex++;
    }

    // maintain target body count
    while (dynamicShapes.length < settings.numberOfBodies) {
        const config = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
        spawnShape(config);
    }

    // update debug renderer
    debugRenderer.update(debugRendererState, world);

    // update orbit controls
    orbitControls.update();

    // render
    renderer.render(scene, camera);
}

// start the application
init();
