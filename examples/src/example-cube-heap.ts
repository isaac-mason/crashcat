import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { RigidBody, Shape } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    plane,
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
camera.position.set(0, 10, 10);
camera.lookAt(0, 5, 0);

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
orbitControls.target.set(0, 5, 0);

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

// increase gravity for more dynamic motion
worldSettings.gravity = [0, -50, 0];

const world = createWorld(worldSettings);

// create static ground plane
rigidBody.create(world, {
    shape: plane.create({
        plane: { normal: [0, 1, 0], constant: 0 },
        halfExtent: 50,
    }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    restitution: 0,
    friction: 0.5,
});

/* cube heap */

const cubes: RigidBody[] = [];
const cubeShape: Shape = box.create({ halfExtents: [0.25, 0.25, 0.25], convexRadius: 0.05 });

const settings = {
    numberOfCubes: 200,
};

const SPAWN_HEIGHT = 10;
const SPAWN_AREA = 2.5; // spawn within +/- 2.5 units on x and z

function randomInRange(min: number, max: number): number {
    return Math.random() * (max - min) + min;
}

function spawnCube(): void {
    // random position above the ground
    const x = randomInRange(-SPAWN_AREA, SPAWN_AREA);
    const y = randomInRange(0, SPAWN_HEIGHT);
    const z = randomInRange(-SPAWN_AREA, SPAWN_AREA);
    const position = vec3.fromValues(x, y, z);

    const body = rigidBody.create(world, {
        shape: cubeShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position,
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
        mass: 1,
    });

    cubes.push(body);
}

function updateCubeCount() {
    const targetCount = settings.numberOfCubes;
    const currentCount = cubes.length;

    if (currentCount < targetCount) {
        // spawn more cubes
        const toSpawn = targetCount - currentCount;
        for (let i = 0; i < toSpawn; i++) {
            spawnCube();
        }
    } else if (currentCount > targetCount) {
        // remove excess cubes
        const toRemove = currentCount - targetCount;
        for (let i = 0; i < toRemove; i++) {
            const cube = cubes.pop();
            if (cube) {
                rigidBody.remove(world, cube);
            }
        }
    }
}

// initial spawn
updateCubeCount();

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

ui.gui
    .add(settings, 'numberOfCubes', 0, 1000, 1)
    .name('Number of Cubes')
    .onChange(() => {
        updateCubeCount();
    });

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // randomly reposition one cube per frame to keep things dynamic
    if (cubes.length > 0) {
        const randomIndex = Math.floor(Math.random() * cubes.length);
        const cube = cubes[randomIndex];

        // reset position to random location
        rigidBody.setPosition(world, cube, [0, randomInRange(0, SPAWN_HEIGHT), 0], true);
        rigidBody.setLinearVelocity(world, cube, vec3.fromValues(0, 0, 0));
        rigidBody.setAngularVelocity(world, cube, vec3.fromValues(0, 0, 0));
    }

    // update physics with delta time
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // update debug renderer
    debugRenderer.update(debugRendererState, world);

    // update orbit controls
    orbitControls.update();

    // render
    renderer.render(scene, camera);
}

animate();
