import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { RigidBody, Shape } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    compound,
    convexHull,
    createWorld,
    createWorldSettings,
    cylinder,
    enableCollision,
    MotionType,
    plane,
    registerAll,
    rigidBody,
    sphere,
    transformed,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 10, 30);
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

const world = createWorld(worldSettings);

// create static ground plane
const groundShape = plane.create({ 
    plane: { normal: [0, 1, 0], constant: 0 },
    halfExtent: 50
});
rigidBody.create(world, {
    shape: groundShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    restitution: 0,
    friction: 0.5,
});

/* shape spawning system */

type ShapeConfig = {
    name: string;
    createShape: () => Shape
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
    {
        name: 'capsule',
        createShape: () => capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 }),
        color: 0xdfe6e9,
    },
    {
        name: 'cylinder',
        createShape: () => cylinder.create({ halfHeight: 0.75, radius: 0.5 }),
        color: 0x74b9ff,
    },
    {
        name: 'dumbbell',
        createShape: () => {
            const dumbbell = compound.create({
                children: [
                    // Left sphere
                    {
                        position: vec3.fromValues(-1.1, 0, 0),
                        quaternion: quat.create(),
                        shape: sphere.create({ radius: 0.5 }),
                    },
                    // Right sphere
                    {
                        position: vec3.fromValues(1.1, 0, 0),
                        quaternion: quat.create(),
                        shape: sphere.create({ radius: 0.5 }),
                    },
                    // Connecting capsule (rotated 90° around Z axis)
                    {
                        position: vec3.fromValues(0, 0, 0),
                        quaternion: quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI * 0.5),
                        shape: capsule.create({ halfHeightOfCylinder: 0.8, radius: 0.3 }),
                    },
                ],
            });
            return dumbbell;
        },
        color: 0x95e1d3,
    },
    {
        name: 'hammer',
        createShape: () => {
            const hammerHead = transformed.create({
                shape: box.create({
                    halfExtents: [0.8, 0.3, 0.3],
                    convexRadius: 0.05,
                    density: 10000,
                }),
                position: vec3.fromValues(0, 1.5, 0), // Position at top of handle
                quaternion: quat.create(), // No rotation for simplicity
            });

            const hammer = compound.create({
                children: [
                    // Long handle - positioned at origin, using compound position/quaternion (both identity)
                    {
                        position: vec3.fromValues(0, 0, 0),
                        quaternion: quat.create(),
                        shape: box.create({ halfExtents: [0.2, 1.5, 0.2], convexRadius: 0.05 }),
                    },
                    // Hammer head - using nested transformed shape instead of compound's transform
                    {
                        position: vec3.fromValues(0, 0, 0), // Intentionally identity
                        quaternion: quat.create(), // Intentionally identity
                        shape: hammerHead, // Transform is inside the shape itself
                    },
                ],
            });
            return hammer;
        },
        color: 0xf39c12,
    },
    {
        name: 'pyramid',
        createShape: () => {
            // Create a simple pyramid shape using convex hull
            // Base is a square, apex is centered above
            return convexHull.create({
                positions: [
                    // Base vertices (square at y=0)
                    -0.5, 0, -0.5,  // bottom-left
                     0.5, 0, -0.5,  // bottom-right
                     0.5, 0,  0.5,  // top-right
                    -0.5, 0,  0.5,  // top-left
                    // Apex (centered above at y=1)
                     0, 1, 0,
                ],
                convexRadius: 0.05,
            });
        },
        color: 0x9b59b6,
    },
];

type DynamicShape = {
    body: RigidBody;
    config: ShapeConfig;
};

const dynamicShapes: DynamicShape[] = [];

const settings = {
    numberOfBodies: 50,
    respawnIntervalMs: 500, // respawn one body every 500ms
};

const SPAWN_HEIGHT = 15;
const SPAWN_AREA = 10; // spawn within +/- 10 units on x and z

function randomInRange(min: number, max: number): number {
    return Math.random() * (max - min) + min;
}

function spawnShape(config: ShapeConfig): void {
    const shape = config.createShape();

    // Random position above the ground
    const x = randomInRange(-SPAWN_AREA, SPAWN_AREA);
    const z = randomInRange(-SPAWN_AREA, SPAWN_AREA);
    const position = vec3.fromValues(x, SPAWN_HEIGHT, z);

    // Random rotation
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
        restitution: 0,
        friction: 0.5,
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
        // Spawn more bodies
        const toSpawn = targetCount - currentCount;
        for (let i = 0; i < toSpawn; i++) {
            const config = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
            spawnShape(config);
        }
    } else if (currentCount > targetCount) {
        // Remove excess bodies
        const toRemove = currentCount - targetCount;
        for (let i = 0; i < toRemove; i++) {
            const shape = dynamicShapes.pop();
            if (shape) {
                rigidBody.remove(world, shape.body);
            }
        }
    }
}

// Initial spawn
updateBodyCount();

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

ui.gui.add(settings, 'numberOfBodies', 0, 1000, 1).name('Number of Bodies').onChange(() => {
    updateBodyCount();
});
ui.gui.add(settings, 'respawnIntervalMs', 100, 5000, 100).name('Respawn Interval (ms)');

const maxDelta = 1 / 30;
let lastTime = performance.now();
let lastRespawnTime = performance.now();
let respawnIndex = 0;

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // Update physics with delta time
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Round-robin respawning based on interval
    if (dynamicShapes.length > 0 && currentTime - lastRespawnTime >= settings.respawnIntervalMs) {
        lastRespawnTime = currentTime;
        
        // Get the next shape to respawn in round-robin fashion
        const shapeToRespawn = dynamicShapes[respawnIndex % dynamicShapes.length];
        
        // Remove old body properly
        rigidBody.remove(world, shapeToRespawn.body);
        
        // Create new body with random shape type
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
            restitution: 0.3,
            friction: 0.5,
        });
        
        // Replace the body in the array
        shapeToRespawn.body = newBody;
        shapeToRespawn.config = newConfig;
        
        respawnIndex++;
    }

    // Maintain target body count
    while (dynamicShapes.length < settings.numberOfBodies) {
        const config = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
        spawnShape(config);
    }

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    // Update orbit controls
    orbitControls.update();

    // Render
    renderer.render(scene, camera);
}

animate();
