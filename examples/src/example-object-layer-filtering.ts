import { vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    enableCollision,
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
camera.position.set(5, 5, 10);
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
worldSettings.gravity = vec3.fromValues(0, -9.81, 0);

// broadphase layers
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

// object layers
const OBJECT_LAYER_ENVIRONMENT = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);
const OBJECT_LAYER_CHARACTERS = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_DEBRIS = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

// collision matrix - debris does NOT collide with characters
enableCollision(worldSettings, OBJECT_LAYER_ENVIRONMENT, OBJECT_LAYER_ENVIRONMENT);
enableCollision(worldSettings, OBJECT_LAYER_ENVIRONMENT, OBJECT_LAYER_CHARACTERS);
enableCollision(worldSettings, OBJECT_LAYER_ENVIRONMENT, OBJECT_LAYER_DEBRIS);
enableCollision(worldSettings, OBJECT_LAYER_CHARACTERS, OBJECT_LAYER_CHARACTERS);
// enableCollision(worldSettings, OBJECT_LAYER_CHARACTERS, OBJECT_LAYER_DEBRIS); // DISABLED - debris passes through characters
enableCollision(worldSettings, OBJECT_LAYER_DEBRIS, OBJECT_LAYER_DEBRIS);

const world = createWorld(worldSettings);

(globalThis as any).world = world; // for debugging

/* create environment - static floor */

rigidBody.create(world, {
    shape: box.create({ halfExtents: [5.0, 0.1, 5.0] }),
    objectLayer: OBJECT_LAYER_ENVIRONMENT,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    friction: 0.5,
});

/* create character - kinematic capsule */

rigidBody.create(world, {
    shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.5 }),
    objectLayer: OBJECT_LAYER_CHARACTERS,
    motionType: MotionType.KINEMATIC,
    position: vec3.fromValues(0, 1.3, 0),
    friction: 0.5,
});

/* create debris spawner */

let debrisSpawnTimer = 0;
const DEBRIS_SPAWN_INTERVAL = 0.1; // spawn every 100ms
const DEBRIS_LIFETIME = 5.0; // despawn after 5 seconds
const DEBRIS_RADIUS = 0.05;
const SPAWN_HEIGHT = 5.0;
const SPAWN_RADIUS = 1.5;

// translation only DOF: allow movement in x, y, z but no rotation
const TRANSLATION_ONLY_DOF = 0b000111;

// track debris bodies with their spawn time
const debrisBodies = new Map<number, number>();

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

/* simulation */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // spawn debris at intervals
    debrisSpawnTimer += delta;
    if (debrisSpawnTimer >= DEBRIS_SPAWN_INTERVAL) {
        debrisSpawnTimer = 0;

        // random position in circle above character
        const angle = Math.random() * Math.PI * 2;
        const radius = Math.random() * SPAWN_RADIUS;
        const x = Math.cos(angle) * radius;
        const z = Math.sin(angle) * radius;

        const debrisBody = rigidBody.create(world, {
            shape: sphere.create({ radius: DEBRIS_RADIUS }),
            objectLayer: OBJECT_LAYER_DEBRIS,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(x, SPAWN_HEIGHT, z),
            friction: 0.3,
            restitution: 0.2,
            allowedDegreesOfFreedom: TRANSLATION_ONLY_DOF,
        });

        // track spawn time
        debrisBodies.set(debrisBody.id, currentTime / 1000);
    }

    // remove debris older than DEBRIS_LIFETIME
    const currentTimeSeconds = currentTime / 1000;
    for (const [bodyId, spawnTime] of debrisBodies.entries()) {
        if (currentTimeSeconds - spawnTime > DEBRIS_LIFETIME) {
            const body = world.bodies.pool.find((b) => b.id === bodyId);
            if (body) {
                rigidBody.remove(world, body);
            }
            debrisBodies.delete(bodyId);
        }
    }

    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    debugRenderer.update(debugRendererState, world);

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
