import type { RigidBody } from 'crashcat';
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
    sphere,
    staticCompound,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import { euler, quat, vec3 } from 'math';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 22, 30);
camera.lookAt(0, 8, 0);

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
orbitControls.target.set(0, 8, 0);

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

/* static compound bridge */

const identity: [number, number, number, number] = [0, 0, 0, 1];
const children: {
    shape: ReturnType<typeof box.create>;
    position: [number, number, number];
    quaternion: [number, number, number, number];
}[] = [];

const bridgeLength = 36;
const bridgeWidth = 3;
const bridgeHeight = 8;
const beamThickness = 0.12;
const deckThickness = 0.15;

// main deck planks
const numPlanks = 12;
for (let i = 0; i < numPlanks; i++) {
    const xPos = -bridgeLength / 2 + (i / (numPlanks - 1)) * bridgeLength;
    children.push({
        shape: box.create({
            halfExtents: [bridgeLength / numPlanks / 2 + 0.05, deckThickness / 2, bridgeWidth / 2],
            convexRadius: 0.01,
        }),
        position: [xPos, bridgeHeight, 0],
        quaternion: identity,
    });
}

// side rails
for (let side = 0; side < 2; side++) {
    const zPos = (side === 0 ? -1 : 1) * (bridgeWidth / 2 + 0.2);

    // bottom rail
    children.push({
        shape: box.create({ halfExtents: [bridgeLength / 2, beamThickness, beamThickness], convexRadius: 0.01 }),
        position: [0, bridgeHeight + 0.3, zPos],
        quaternion: identity,
    });

    // top rail
    children.push({
        shape: box.create({ halfExtents: [bridgeLength / 2, beamThickness, beamThickness], convexRadius: 0.01 }),
        position: [0, bridgeHeight + 1.2, zPos],
        quaternion: identity,
    });

    // vertical posts every 3 units
    for (let i = 0; i < 9; i++) {
        const xPos = -bridgeLength / 2 + (i / 8) * bridgeLength;
        children.push({
            shape: box.create({ halfExtents: [beamThickness, 0.6, beamThickness], convexRadius: 0.01 }),
            position: [xPos, bridgeHeight + 0.75, zPos],
            quaternion: identity,
        });
    }
}

// support trusses underneath
const numTrusses = 8;
for (let i = 0; i < numTrusses; i++) {
    const xPos = -bridgeLength / 2 + (i / (numTrusses - 1)) * bridgeLength;

    // vertical supports from bottom to deck
    for (let side = 0; side < 2; side++) {
        const zPos = (side === 0 ? -1 : 1) * (bridgeWidth / 2 - 0.3);
        const supportHeight = 3.5;

        children.push({
            shape: box.create({ halfExtents: [beamThickness * 1.5, supportHeight / 2, beamThickness * 1.5], convexRadius: 0.01 }),
            position: [xPos, bridgeHeight - supportHeight / 2, zPos],
            quaternion: identity,
        });
    }

    // diagonal cross-bracing between vertical supports
    if (i < numTrusses - 1) {
        const nextXPos = -bridgeLength / 2 + ((i + 1) / (numTrusses - 1)) * bridgeLength;
        const crossLength = Math.sqrt((nextXPos - xPos) ** 2 + bridgeWidth ** 2 + 1.5 ** 2);

        // x-shaped bracing
        for (let cross = 0; cross < 2; cross++) {
            const angle = cross === 0 ? Math.atan2(bridgeWidth, nextXPos - xPos) : -Math.atan2(bridgeWidth, nextXPos - xPos);
            const tiltAngle = Math.atan2(1.5, nextXPos - xPos);

            const crossQuat = quat.create();
            quat.fromEuler(crossQuat, euler.fromValues(tiltAngle, angle, 0, 'xyz'));

            children.push({
                shape: box.create({
                    halfExtents: [crossLength / 2, beamThickness * 0.8, beamThickness * 0.8],
                    convexRadius: 0.01,
                }),
                position: [(xPos + nextXPos) / 2, bridgeHeight - 1.5, 0],
                quaternion: crossQuat,
            });
        }
    }
}

// bottom structural beams running the length
for (let side = 0; side < 2; side++) {
    const zPos = (side === 0 ? -1 : 1) * (bridgeWidth / 2 - 0.3);
    children.push({
        shape: box.create({ halfExtents: [bridgeLength / 2, beamThickness * 1.5, beamThickness * 1.5], convexRadius: 0.01 }),
        position: [0, bridgeHeight - 3.5, zPos],
        quaternion: identity,
    });
}

// cross beams connecting the two bottom structural beams
for (let i = 0; i < 9; i++) {
    const xPos = -bridgeLength / 2 + (i / 8) * bridgeLength;
    children.push({
        shape: box.create({ halfExtents: [beamThickness, beamThickness * 1.5, bridgeWidth / 2 - 0.3], convexRadius: 0.01 }),
        position: [xPos, bridgeHeight - 3.5, 0],
        quaternion: identity,
    });
}

const bridgeStructure = staticCompound.create({ children });

rigidBody.create(world, {
    shape: bridgeStructure,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: [0, 0, 0],
    quaternion: identity,
    restitution: 0.3,
    friction: 0.5,
});

/* sphere spawning */

/* hollow box made of 8 spheres at corners */

function createSphereBox(
    boxHalfExtents: [number, number, number],
    sphereRadius: number,
): ReturnType<typeof staticCompound.create> {
    const [xHalf, yHalf, zHalf] = boxHalfExtents;
    const r = sphereRadius;

    const children: {
        shape: ReturnType<typeof sphere.create>;
        position: [number, number, number];
        quaternion: [number, number, number, number];
    }[] = [];
    const identity: [number, number, number, number] = [0, 0, 0, 1];

    // 8 corners
    children.push({ shape: sphere.create({ radius: r }), position: [xHalf, yHalf, zHalf], quaternion: identity });
    children.push({ shape: sphere.create({ radius: r }), position: [xHalf, yHalf, -zHalf], quaternion: identity });
    children.push({ shape: sphere.create({ radius: r }), position: [xHalf, -yHalf, zHalf], quaternion: identity });
    children.push({ shape: sphere.create({ radius: r }), position: [xHalf, -yHalf, -zHalf], quaternion: identity });
    children.push({ shape: sphere.create({ radius: r }), position: [-xHalf, yHalf, zHalf], quaternion: identity });
    children.push({ shape: sphere.create({ radius: r }), position: [-xHalf, yHalf, -zHalf], quaternion: identity });
    children.push({ shape: sphere.create({ radius: r }), position: [-xHalf, -yHalf, zHalf], quaternion: identity });
    children.push({ shape: sphere.create({ radius: r }), position: [-xHalf, -yHalf, -zHalf], quaternion: identity });

    return staticCompound.create({ children });
}

const BOX_HALF_EXTS: [number, number, number] = [0.25, 0.25, 0.25];
const CORNER_SPHERE_RADIUS = 0.12;
const sphereBoxShape = createSphereBox(BOX_HALF_EXTS, CORNER_SPHERE_RADIUS);

const sphereBodies: RigidBody[] = [];

const settings = {
    numberOfBoxes: 100,
    respawnIntervalMs: 200,
};

function spawnBox(): RigidBody {
    const x = (Math.random() - 0.5) * (bridgeLength - 4);
    const y = bridgeHeight + 5 + Math.random() * 3;
    const z = (Math.random() - 0.5) * (bridgeWidth - 0.5);

    return rigidBody.create(world, {
        shape: sphereBoxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(x, y, z),
        restitution: 0.3,
        friction: 0.5,
    });
}

function updateBoxCount() {
    const target = settings.numberOfBoxes;

    while (sphereBodies.length < target) {
        sphereBodies.push(spawnBox());
    }

    while (sphereBodies.length > target) {
        const body = sphereBodies.pop()!;
        rigidBody.remove(world, body);
    }
}

updateBoxCount();

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

ui.gui
    .add(settings, 'numberOfBoxes', 0, 500, 1)
    .name('Number of Boxes')
    .onChange(() => {
        updateBoxCount();
    });
ui.gui.add(settings, 'respawnIntervalMs', 50, 2000, 50).name('Respawn Interval (ms)');

const maxDelta = 1 / 30;
let lastTime = performance.now();
let lastRespawnTime = performance.now();
let respawnIndex = 0;

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // round-robin respawn so boxes keep falling onto the bridge
    if (sphereBodies.length > 0 && currentTime - lastRespawnTime >= settings.respawnIntervalMs) {
        lastRespawnTime = currentTime;

        const idx = respawnIndex % sphereBodies.length;
        rigidBody.remove(world, sphereBodies[idx]);
        sphereBodies[idx] = spawnBox();
        respawnIndex++;
    }

    // maintain target count in case it was increased via the slider
    while (sphereBodies.length < settings.numberOfBoxes) {
        sphereBodies.push(spawnBox());
    }

    debugRenderer.update(debugRendererState, world);

    orbitControls.update();

    renderer.render(scene, camera);
}

animate();
