import type GUI from 'lil-gui';
import { euler, quat, vec3 } from 'mathcat';
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
    type World,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

registerAllShapes();

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 5, 15);
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

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

/* scene management */

type SceneSetup = {
    name: string;
    setup: () => World;
};

const scenes: SceneSetup[] = [];
let currentSceneIndex = 0;
let currentWorld: World | null = null;

function addScene(name: string, setup: () => World) {
    scenes.push({ name, setup });
}

function loadScene(index: number) {
    if (index < 0 || index >= scenes.length) return;

    currentSceneIndex = index;
    const sceneSetup = scenes[currentSceneIndex];

    console.log(`Loading scene: ${sceneSetup.name}`);

    // Clear debug visuals before recreating world
    debugRenderer.clear(debugRendererState);

    // Setup new world
    currentWorld = sceneSetup.setup();
    (globalThis as any).world = currentWorld;

    // Update UI
    updateSceneInfo();
}

function nextScene() {
    loadScene((currentSceneIndex + 1) % scenes.length);
}

function previousScene() {
    loadScene((currentSceneIndex - 1 + scenes.length) % scenes.length);
}

/* UI */

let sceneButtonsFolder: GUI;
const sceneControllers: any[] = [];

const guiConfig = {
    restart: () => loadScene(currentSceneIndex),
    previous: previousScene,
    next: nextScene,
};

function setupGui() {
    sceneButtonsFolder = ui.gui.addFolder('Scenes');
    scenes.forEach((scene, i) => {
        const controller = sceneButtonsFolder
            .add(
                {
                    select: () => loadScene(i),
                },
                'select',
            )
            .name(scene.name);
        sceneControllers.push(controller);
    });
    sceneButtonsFolder.open();
    ui.gui.add(guiConfig, 'previous').name('← Previous (←)');
    ui.gui.add(guiConfig, 'next').name('Next → (→)');
    ui.gui.add(guiConfig, 'restart').name('↻ Restart (Space)');
}

function updateSceneInfo() {
    sceneControllers.forEach((controller, i) => {
        const isActive = i === currentSceneIndex;
        const sceneName = scenes[i].name;
        controller.name(isActive ? `► ${sceneName}` : sceneName);
    });
}

// Keyboard controls
document.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowRight') {
        nextScene();
    } else if (e.key === 'ArrowLeft') {
        previousScene();
    } else if (e.key === ' ') {
        e.preventDefault();
        loadScene(currentSceneIndex);
    }
});

/* Scene definitions */

addScene('Sphere x Sphere', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const sphereShape = sphere.create({ radius: 1.0 });

    // Sphere 1 (moving right)
    const body1 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere 2 (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Sphere x Box Side', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Box (moving right)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Sphere x Box Edge', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Box (moving right, rotated 45° around Y)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    const q1 = quat.create();
    quat.fromEuler(q1, euler.fromValues(0, Math.PI * 0.25, 0, 'xyz'));
    rigidBody.setTransform(world, body1, body1.position, q1, false);
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Sphere x Box Corner', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Box (moving right, rotated 45° around Y and Z)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    const q1 = quat.create();
    const q2 = quat.create();
    const q3 = quat.create();
    quat.fromEuler(q1, euler.fromValues(0, Math.PI * 0.25, 0, 'xyz'));
    quat.fromEuler(q2, euler.fromValues(0, 0, Math.PI * 0.25, 'xyz'));
    quat.multiply(q3, q1, q2);
    rigidBody.setTransform(world, body1, body1.position, q3, false);
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Box x Box Face', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

    // Box 1 (moving right)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Box 2 (moving left)
    const body2 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Box x Box Edge', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

    // Box 1 (moving right, rotated)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    const q1 = quat.create();
    quat.fromEuler(q1, euler.fromValues(0, Math.PI * 0.25, 0, 'xyz'));
    rigidBody.setTransform(world, body1, body1.position, q1, false);
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Box 2 (moving left)
    const body2 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Box x Box Corner', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

    // Box 1 (moving right, rotated 45° Y and Z)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    const q1 = quat.create();
    const q2 = quat.create();
    const q3 = quat.create();
    quat.fromEuler(q1, euler.fromValues(0, Math.PI * 0.25, 0, 'xyz'));
    quat.fromEuler(q2, euler.fromValues(0, 0, Math.PI * 0.25, 'xyz'));
    quat.multiply(q3, q1, q2);
    rigidBody.setTransform(world, body1, body1.position, q3, false);
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Box 2 (moving left, rotated opposite)
    const body2 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    const q4 = quat.create();
    const q5 = quat.create();
    const q6 = quat.create();
    quat.fromEuler(q4, euler.fromValues(0, -Math.PI * 0.25, 0, 'xyz'));
    quat.fromEuler(q5, euler.fromValues(0, 0, -Math.PI * 0.25, 'xyz'));
    quat.multiply(q6, q4, q5);
    rigidBody.setTransform(world, body2, body2.position, q6, false);
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Capsule x Sphere', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Capsule (moving right)
    const body1 = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Capsule x Box', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

    // Capsule (moving right)
    const body1 = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Box (moving left)
    const body2 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Capsule Top x Box', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

    // Box (static, on ground)
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 0, 0),
    });

    // Capsule (dynamic, above box, vertical)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 4, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    // Give it a small downward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, 0, -2, 0);

    return world;
});

addScene('Capsule Top x Sphere', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Sphere (static, on ground)
    rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 0, 0),
    });

    // Capsule (dynamic, above sphere, vertical)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 4, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    // Give it a small downward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, 0, -2, 0);

    return world;
});

addScene('Capsule Edge x Sphere', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Sphere (static, at origin)
    rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 0, 0),
    });

    // Capsule (dynamic, edge-on, moving right to left)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(5, 0, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    // Rotate capsule to be horizontal (along X axis)
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 2, 'xyz'));
    rigidBody.setTransform(world, capsuleBody, capsuleBody.position, q, false);
    // Give it a leftward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, -5, 0, 0);

    return world;
});

addScene('Capsule Top x Box Edge', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

    // Box (static, rotated 45° around Y, edge up, positioned right and raised)
    const boxBody = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(3, 1, 0), // right and raised
    });
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, Math.PI * 0.25, 0, 'xyz'));
    rigidBody.setTransform(world, boxBody, boxBody.position, q, false);

    // Capsule (dynamic, left, vertical, moving right)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-3, 2, 0), // left, slightly above box
        linearDamping: 0,
        angularDamping: 0,
    });
    // Give it a rightward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, 4, 0, 0);

    return world;
});

addScene('Tilted Capsule x Static Box Floor', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, -10, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const boxShape = box.create({ halfExtents: vec3.fromValues(5, 0.5, 5) });

    // Static box floor
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.5, 0),
    });

    // Tilted capsule above floor
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 3, 0),
        quaternion: quat.fromDegrees(quat.create(), 0, 0, 45, 'xyz'),
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the capsule 45° around Z
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 4, 'xyz'));
    rigidBody.setTransform(world, capsuleBody, capsuleBody.position, q, false);

    return world;
});

addScene('Tilted Box x Static Box Floor', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, -10, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    const dynamicBoxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
    const floorBoxShape = box.create({ halfExtents: vec3.fromValues(5, 0.5, 5) });

    // Static box floor
    rigidBody.create(world, {
        shape: floorBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.5, 0),
    });

    // Tilted box above floor
    const boxBody = rigidBody.create(world, {
        shape: dynamicBoxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 3, 0),
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the box
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 3, 'xyz'));
    rigidBody.setTransform(world, boxBody, boxBody.position, q, false);

    return world;
});

addScene('Box Stack', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, -10, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
    const floorBoxShape = box.create({ halfExtents: vec3.fromValues(5, 0.5, 5) });

    // Static box floor
    rigidBody.create(world, {
        shape: floorBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.5, 0),
    });

    // Stack of 5 boxes with gaps
    const gap = 0;
    const boxHeight = 2.0;
    const spacing = boxHeight + gap;
    const startHeight = boxHeight / 2 + 0.5;

    for (let i = 0; i < 5; i++) {
        rigidBody.create(world, {
            shape: boxShape,
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, startHeight + i * spacing, 0),
            linearDamping: 0,
            angularDamping: 0,
        });
    }

    return world;
});

addScene('Tilted Box Falls onto Dynamic Box', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, -10, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
    const floorBoxShape = box.create({ halfExtents: vec3.fromValues(5, 0.5, 5) });

    // Static box floor
    rigidBody.create(world, {
        shape: floorBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.5, 0),
    });

    // Box on ground
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 1, 0),
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilted box above
    const tiltedBox = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 5, 0),
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the falling box
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 3, 'xyz'));
    rigidBody.setTransform(world, tiltedBox, tiltedBox.position, q, false);

    return world;
});

addScene('Tilted Box Falls onto Static Box', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, -10, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

    // Box on ground
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 1, 0),
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilted box above
    const tiltedBox = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 5, 0),
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the falling box
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 3, 'xyz'));
    rigidBody.setTransform(world, tiltedBox, tiltedBox.position, q, false);

    return world;
});

addScene('Jenga', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, -5, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static ground
    const groundShape = box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) });
    rigidBody.create(world, {
        shape: groundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.5, 0),
    });

    const size = 0.5;
    const gap = 0.02;

    // Build Jenga tower layers
    for (let i = 0; i < 10; i++) {
        for (let j = 0; j < 3; j++) {
            let halfExtents: [number, number, number];
            let dx: number;
            let dz: number;

            if (i % 2 === 0) {
                // Horizontal layer (along X axis)
                halfExtents = [size, size, size * 3];
                dx = 1;
                dz = 0;
            } else {
                // Horizontal layer (along Z axis)
                halfExtents = [size * 3, size, size];
                dx = 0;
                dz = 1;
            }

            const blockShape = box.create({ halfExtents: vec3.fromValues(...halfExtents) });

            rigidBody.create(world, {
                shape: blockShape,
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(
                    2 * (size + gap) * (j - 1) * dx,
                    2 * (size + gap) * (i + 1),
                    2 * (size + gap) * (j - 1) * dz,
                ),
            });
        }
    }

    return world;
});

addScene('Pyramid Stress Test', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Create static floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: vec3.fromValues(20, 0.5, 20) }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.5, 0),
        restitution: 0.3,
    });

    // Create pyramid
    const towerHeight = 5;
    const boxSize = 1.0;
    const boxHalfExtents = vec3.fromValues(boxSize * 0.5, boxSize * 0.5, boxSize * 0.5);
    const boxShape = box.create({ halfExtents: boxHalfExtents });

    for (let y = 0; y < towerHeight; y++) {
        const baseOffset = towerHeight - 1 - y;

        for (let x = -baseOffset; x <= baseOffset; x++) {
            for (let z = -baseOffset; z <= baseOffset; z++) {
                rigidBody.create(world, {
                    shape: boxShape,
                    objectLayer: OBJECT_LAYER_MOVING,
                    motionType: MotionType.DYNAMIC,
                    position: vec3.fromValues(x * boxSize, y * boxSize + boxSize * 0.5, z * boxSize),
                    restitution: 0.3,
                });
            }
        }
    }

    // Create block to fall on pyramid
    rigidBody.create(world, {
        shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 20, 0),
        restitution: 0.3,
    });

    return world;
});

addScene('Sleep', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static ground
    rigidBody.create(world, {
        shape: box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.5, 0),
    });

    // Create sphere that will fall and sleep
    const size = 1;
    rigidBody.create(world, {
        shape: sphere.create({ radius: size }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, size * 6, 0),
    });

    // The body will sleep after coming to rest
    // Watch the console for when it goes to sleep (timeBeforeSleep = 0.5s by default)
    console.log('Sphere will sleep after 0.5s of being at rest');

    return world;
});

addScene('Wake up when hit', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0]; // No gravity so sleeping body stays in place

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const size = 2;
    const sphereShape = sphere.create({ radius: size });

    // Create sphere and force it to sleep
    const sphereBody1 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, size, 0),
    });

    // Force it to sleep
    rigidBody.sleep(world, sphereBody1);
    console.log('Sphere 1 is sleeping, waiting to be hit...');

    // Create sphere that will wake up the first one
    const sphereBody2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-size * 10, size, 0),
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(sphereBody2.motionProperties.linearVelocity, 10, 0, 0);

    // The sleeping body will wake up when it gets hit
    console.log('Sphere 2 moving to hit sleeping sphere 1');

    return world;
});

addScene('Wake up with impulse', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0]; // No gravity

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const size = 2;
    const sphereBody = rigidBody.create(world, {
        shape: sphere.create({ radius: size }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, size, 0),
    });

    // Force it to sleep
    rigidBody.sleep(world, sphereBody);
    console.log('Sphere is sleeping...');

    // Apply an impulse after 1 second to wake it up
    setTimeout(() => {
        console.log('Applying impulse to wake up the sphere!');
        rigidBody.addImpulse(world, sphereBody, [5, 0, 0]);
    }, 1000);

    return world;
});

/* simulation */

setupGui();

// load first scene
loadScene(0);

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    if (currentWorld) {
        debugUI.beginPerf(ui);
        updateWorld(currentWorld, undefined, delta);
        debugUI.endPerf(ui);
        debugUI.updateStats(ui, currentWorld);
        debugRenderer.update(debugRendererState, currentWorld);
    }

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
