import type GUI from 'lil-gui';
import { euler, quat, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type RigidBody,
    box,
    ConstraintSpace,
    capsule,
    coneConstraint,
    createWorld,
    createWorldSettings,
    cylinder,
    distanceConstraint,
    enableCollision,
    fixedConstraint,
    hingeConstraint,
    type Listener,
    MotionQuality,
    MotionType,
    motionProperties,
    offsetCenterOfMass,
    plane,
    pointConstraint,
    rigidBody,
    SpringMode,
    sixDOFConstraint,
    sliderConstraint,
    sphere,
    swingTwistConstraint,
    triangleMesh,
    updateWorld,
    type World,
    registerAll,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';
import { loadGLTF } from './utils/gltf';

registerAll();

/* constraint-connected body pair filtering */

/** creates a listener that filters out collisions between constraint-connected bodies */
function createConstraintFilterListener(): Listener {
    return {
        onBodyPairValidate: (bodyA: RigidBody, bodyB: RigidBody): boolean => {
            // skip collision if bodies are connected by a constraint
            return !rigidBody.bodiesShareConstraint(bodyA, bodyB);
        },
    };
}

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

type SceneResult = { world: World; listener?: Listener };

type SceneSetup = {
    name: string;
    setup: () => SceneResult | Promise<SceneResult>;
};

const scenes: SceneSetup[] = [];
let currentSceneIndex = 0;
let currentWorld: World | null = null;
let currentListener: Listener | undefined;

function addScene(name: string, setup: () => SceneResult | Promise<SceneResult>) {
    scenes.push({ name, setup });
}

async function loadScene(index: number) {
    if (index < 0 || index >= scenes.length) return;

    currentSceneIndex = index;
    const sceneSetup = scenes[currentSceneIndex];

    console.log(`Loading scene: ${sceneSetup.name}`);

    // Clear debug visuals before recreating world
    debugRenderer.clear(debugRendererState);

    // Setup new world (handle both sync and async)
    const result = await sceneSetup.setup();
    currentWorld = result.world;
    (globalThis as any).world = currentWorld;

    // Use provided contact listener if any
    currentListener = result.listener;

    // Update UI
    updateSceneInfo();

    // Update URL hash
    updateUrlHash();
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

function getSceneSlug(sceneName: string): string {
    return sceneName.toLowerCase().replace(/\s+/g, '-');
}

function updateUrlHash() {
    const sceneSlug = getSceneSlug(scenes[currentSceneIndex].name);
    window.location.hash = sceneSlug;
}

function loadSceneFromHash() {
    const hash = window.location.hash.slice(1); // Remove the '#'
    if (!hash) return 0;

    const sceneIndex = scenes.findIndex((scene) => getSceneSlug(scene.name) === hash);
    return sceneIndex >= 0 ? sceneIndex : 0;
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
        position: [-5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere 2 (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Plane x Dynamic Bodies', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // create a plane as floor with normal (0.1, 1.0, 0.0).normalized() and constant 1.0
    const planeNormal: Vec3 = [0.1, 1.0, 0.0];
    vec3.normalize(planeNormal, planeNormal);
    const planeShape = plane.create({
        plane: { normal: planeNormal, constant: 1.0 },
        halfExtent: 100,
    });

    rigidBody.create(world, {
        shape: planeShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
    });

    // add a sphere at (0, 1, 0)
    const sphereShape = sphere.create({ radius: 0.5 });
    rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 1, 0],
    });

    // add a box at (2, 1, 0)
    const boxShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [2, 1, 0],
    });

    return { world };
});

addScene('Box x Triangle Mesh Floor', () => {
    const worldSettings = createWorldSettings();
    // worldSettings.narrowphase.collideOnlyWithActiveEdges = false
    // worldSettings.narrowphase.collideWithBackfaces = true;

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Create simple quad floor as triangle mesh
    // Two triangles forming a 20x20 quad in the XZ plane
    const size = 10;

    // biome-ignore format: readability
    const floorMesh = triangleMesh.create({
        positions: [
            -size, 0, -size,  // v0: bottom-left
             size, 0, -size,  // v1: bottom-right
             size, 0,  size,  // v2: top-right
            -size, 0,  size,  // v3: top-left
        ],
        indices: [
            0, 2, 1,  // first triangle: v0 → v2 → v1 (CCW when viewed from above, normal points up)
            0, 3, 2,  // second triangle: v0 → v3 → v2 (CCW when viewed from above, normal points up)
        ],
    });

    // Static floor
    rigidBody.create(world, {
        shape: floorMesh,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        restitution: 0, //.3,
        friction: 0.5,
    });

    // Dynamic box
    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 5, 0],
        quaternion: quat.fromDegrees(quat.create(), 0, 0, 0, 'yxz'),
        restitution: 0, //.3,
        friction: 0.5,
        motionQuality: MotionQuality.LINEAR_CAST,
    });

    worldSettings.ccd.linearCastThreshold = 0;

    return { world };
});

addScene('Sphere x Box Side', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Box (moving right)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Sphere x Box Edge', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Box (moving right, rotated 45° around Y)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
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
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Sphere x Box Corner', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
    const sphereShape = sphere.create({ radius: 1.0 });

    // Box (moving right, rotated 45° around Y and Z)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
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
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Box x Box Face', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });

    // Box 1 (moving right)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Box 2 (moving left)
    const body2 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Box x Box Edge', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });

    // Box 1 (moving right, rotated)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
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
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Box x Box Corner', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });

    // Box 1 (moving right, rotated 45° Y and Z)
    const body1 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
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
        position: [5, 0, 0],
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

    return { world };
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
        position: [-5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Sphere (moving left)
    const body2 = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Capsule x Box', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });

    // Capsule (moving right)
    const body1 = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body1.motionProperties.linearVelocity, 5, 0, 0);

    // Box (moving left)
    const body2 = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(body2.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Capsule Top x Box', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });

    // Box (static, on ground)
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
    });

    // Capsule (dynamic, above box, vertical)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 4, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    // Give it a small downward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, 0, -2, 0);

    return { world };
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
        position: [0, 0, 0],
    });

    // Capsule (dynamic, above sphere, vertical)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 4, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    // Give it a small downward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, 0, -2, 0);

    return { world };
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
        position: [0, 0, 0],
    });

    // Capsule (dynamic, edge-on, moving right to left)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    // Rotate capsule to be horizontal (along X axis)
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 2, 'xyz'));
    rigidBody.setTransform(world, capsuleBody, capsuleBody.position, q, false);
    // Give it a leftward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, -5, 0, 0);

    return { world };
});

addScene('Capsule Top x Box Edge', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });

    // Box (static, rotated 45° around Y, edge up, positioned right and raised)
    const boxBody = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: [3, 1, 0], // right and raised
    });
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, Math.PI * 0.25, 0, 'xyz'));
    rigidBody.setTransform(world, boxBody, boxBody.position, q, false);

    // Capsule (dynamic, left, vertical, moving right)
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-3, 2, 0], // left, slightly above box
        linearDamping: 0,
        angularDamping: 0,
    });
    // Give it a rightward velocity
    vec3.set(capsuleBody.motionProperties.linearVelocity, 4, 0, 0);

    return { world };
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
    const boxShape = box.create({ halfExtents: [5, 0.5, 5], convexRadius: 0.05 });

    // Static box floor
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Tilted capsule above floor
    const capsuleBody = rigidBody.create(world, {
        shape: capsuleShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0],
        quaternion: quat.fromDegrees(quat.create(), 0, 0, 45, 'xyz'),
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the capsule 45° around Z
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 4, 'xyz'));
    rigidBody.setTransform(world, capsuleBody, capsuleBody.position, q, false);

    return { world };
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

    const dynamicBoxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
    const floorBoxShape = box.create({ halfExtents: [5, 0.5, 5], convexRadius: 0.05 });

    // Static box floor
    rigidBody.create(world, {
        shape: floorBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Tilted box above floor
    const boxBody = rigidBody.create(world, {
        shape: dynamicBoxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0],
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the box
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 3, 'xyz'));
    rigidBody.setTransform(world, boxBody, boxBody.position, q, false);

    return { world };
});

addScene('Box with Convex Radius x Static Box Floor', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, -10, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Box with convex radius - should show visible gap when resting on floor
    const dynamicBoxShape = box.create({
        halfExtents: [1, 1, 1],
        convexRadius: 0.1,
    });

    const floorBoxShape = box.create({
        halfExtents: [5, 0.5, 5],
        convexRadius: 0.05,
    });

    // Static box floor with convex radius
    rigidBody.create(world, {
        shape: floorBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Box with convex radius above floor
    rigidBody.create(world, {
        shape: dynamicBoxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0],
        linearDamping: 0.05,
        angularDamping: 0.05,
    });

    return { world };
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

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
    const floorBoxShape = box.create({ halfExtents: [5, 0.5, 5], convexRadius: 0.05 });

    // Static box floor
    rigidBody.create(world, {
        shape: floorBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
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
            position: [0, startHeight + i * spacing, 0],
            linearDamping: 0,
            angularDamping: 0,
        });
    }

    return { world };
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

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
    const floorBoxShape = box.create({ halfExtents: [5, 0.5, 5], convexRadius: 0.05 });

    // Static box floor
    rigidBody.create(world, {
        shape: floorBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Box on ground
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 1, 0],
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilted box above
    const tiltedBox = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 5, 0],
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the falling box
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 3, 'xyz'));
    rigidBody.setTransform(world, tiltedBox, tiltedBox.position, q, false);

    return { world };
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

    const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });

    // Box on ground
    rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 1, 0],
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilted box above
    const tiltedBox = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 5, 0],
        linearDamping: 0,
        angularDamping: 0,
    });

    // Tilt the falling box
    const q = quat.create();
    quat.fromEuler(q, euler.fromValues(0, 0, Math.PI / 3, 'xyz'));
    rigidBody.setTransform(world, tiltedBox, tiltedBox.position, q, false);

    return { world };
});

/* Cylinder test scenarios */

addScene('Cylinder x Sphere', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Cylinder (moving right)
    const cylinderBody = rigidBody.create(world, {
        shape: cylinder.create({ halfHeight: 1.5, radius: 0.8 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(cylinderBody.motionProperties.linearVelocity, 3, 0, 0);

    // Sphere (moving left)
    const sphereBody = rigidBody.create(world, {
        shape: sphere.create({ radius: 1.0 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [5, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(sphereBody.motionProperties.linearVelocity, -3, 0, 0);

    return { world };
});

addScene('Cylinder x Box Edge', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Cylinder falling onto box edge
    const cylinderBody = rigidBody.create(world, {
        shape: cylinder.create({ halfHeight: 1.0, radius: 0.5 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0],
        quaternion: quat.setAxisAngle(quat.create(), [0, 0, 1], Math.PI / 2), // Horizontal
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(cylinderBody.motionProperties.linearVelocity, 0, -2, 0);

    // Static box positioned as edge obstacle
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [2, 0.2, 0.2] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
    });

    return { world };
});

addScene('Rolling Cylinder x Static Floor', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [10, 0.5, 5] }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
        friction: 0.8,
    });

    // Horizontal cylinder with initial angular velocity (rolling)
    const cylinderBody = rigidBody.create(world, {
        shape: cylinder.create({ halfHeight: 1.0, radius: 0.5 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-5, 2, 0],
        quaternion: quat.setAxisAngle(quat.create(), [0, 0, 1], Math.PI / 2), // Horizontal
        friction: 0.8,
    });
    // Give it angular velocity to make it roll
    cylinderBody.motionProperties.angularVelocity[2] = 5;
    cylinderBody.motionProperties.linearVelocity[0] = 2.5; // Match rolling motion

    return { world };
});

addScene('Cylinder Stack', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [10, 0.5, 5] }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    const cylinderShape = cylinder.create({ halfHeight: 0.5, radius: 0.4 });

    // Stack of upright cylinders
    const stackHeight = 6;
    for (let i = 0; i < stackHeight; i++) {
        rigidBody.create(world, {
            shape: cylinderShape,
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [0, i * 1.0 + 0.5, 0],
            friction: 0.5,
        });
    }

    return { world };
});

addScene('Tilted Cylinder x Static Floor', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [10, 0.5, 5] }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Tilted cylinder falling onto floor
    const rotation = quat.create();
    quat.setAxisAngle(rotation, [0, 0, 1], Math.PI / 6); // 30 degrees tilt

    rigidBody.create(world, {
        shape: cylinder.create({ halfHeight: 1.5, radius: 0.5 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0],
        quaternion: rotation,
    });

    return { world };
});

addScene('Cylinder x Cylinder T-Bone', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Horizontal cylinder (moving down)
    const horizontalCylinder = rigidBody.create(world, {
        shape: cylinder.create({ halfHeight: 1.5, radius: 0.4 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 4, 0],
        quaternion: quat.setAxisAngle(quat.create(), [0, 0, 1], Math.PI / 2), // Horizontal
        linearDamping: 0,
        angularDamping: 0,
    });
    vec3.set(horizontalCylinder.motionProperties.linearVelocity, 0, -3, 0);

    // Vertical cylinder (stationary)
    rigidBody.create(world, {
        shape: cylinder.create({ halfHeight: 1.5, radius: 0.4 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 0, 0],
        linearDamping: 0,
        angularDamping: 0,
    });

    return { world };
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
    const groundShape = box.create({ halfExtents: [50, 0.5, 50], convexRadius: 0.05 });
    rigidBody.create(world, {
        shape: groundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
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

            const blockShape = box.create({ halfExtents: [...halfExtents], convexRadius: 0.05 });

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

    return { world };
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
        shape: box.create({ halfExtents: [20, 0.5, 20], convexRadius: 0.05 }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
        restitution: 0.3,
    });

    // Create pyramid
    const towerHeight = 6;
    const boxSize = 1.0;
    const boxHalfExtents: Vec3 = [boxSize * 0.5, boxSize * 0.5, boxSize * 0.5];
    const boxShape = box.create({ halfExtents: boxHalfExtents, convexRadius: 0.2 });

    for (let y = 0; y < towerHeight; y++) {
        const baseOffset = towerHeight - 1 - y;

        for (let x = -baseOffset; x <= baseOffset; x++) {
            for (let z = -baseOffset; z <= baseOffset; z++) {
                rigidBody.create(world, {
                    shape: boxShape,
                    objectLayer: OBJECT_LAYER_MOVING,
                    motionType: MotionType.DYNAMIC,
                    position: [x * boxSize, y * boxSize + boxSize * 0.5, z * boxSize],
                    restitution: 0,
                    friction: 0.5,
                });
            }
        }
    }

    // Create block to fall on pyramid
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 20, 0],
        restitution: 0.3,
    });

    return { world };
});

addScene('Box Stack Pyramid', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Create static box floor (big box)
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [20, 0.5, 20], convexRadius: 0.05 }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
        friction: 1.0,
    });

    // Create pyramid stack
    const size = 10;
    const boxShape = box.create({ halfExtents: [1.0, 1.0, 1.0], convexRadius: 0.05 });

    for (let i = 0; i < size; i++) {
        for (let j = 0; j < size - i; j++) {
            for (let k = 0; k < size - i; k++) {
                rigidBody.create(world, {
                    shape: boxShape,
                    objectLayer: OBJECT_LAYER_MOVING,
                    motionType: MotionType.DYNAMIC,
                    position: [2.0 * j * 1.3 - size + i * 1.2, i * 2.2 + 1.0, 2.0 * k * 1.3 - size + i * 1.2],
                    friction: 1, // higher friction for stability
                });
            }
        }
    }

    return { world };
});

addScene('100 Spheres x Suzanne', async () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    rigidBody.create(world, {
        shape: box.create({ halfExtents: [20, 0.5, 20], convexRadius: 0.05 }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
        restitution: 0.3,
        friction: 0.5,
    });

    // Load Suzanne mesh
    const gltf = await loadGLTF('./models/low-poly-suzi.glb');
    const gltfMesh = gltf.scene.children[0] as THREE.Mesh;
    const suzanneGeometry = gltfMesh.geometry as THREE.BufferGeometry;

    const positions = Array.from(suzanneGeometry.getAttribute('position').array!);
    const indices = Array.from(suzanneGeometry.getIndex()!.array);

    // scale positions by 3x
    for (let i = 0; i < positions.length; i++) {
        positions[i] *= 3;
    }

    const suzanneShape = triangleMesh.create({
        positions: positions,
        indices: indices,
    });

    // create static Suzanne just above the floor
    rigidBody.create(world, {
        shape: suzanneShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 3, 0],
        restitution: 0.3,
        friction: 0.5,
    });

    // Create 100 falling spheres in a grid
    const sphereShape = sphere.create({ radius: 0.1 });

    for (let i = 0; i < 100; i++) {
        const x = 0.4 + Math.random() * 0.3;
        const y = 10 + Math.random();
        const z = 0.4 + Math.random() * 0.3;

        rigidBody.create(world, {
            shape: sphereShape,
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [x * 10 - 5, y, z * 10 - 5],
            restitution: 0.3,
            friction: 0.5,
        });
    }

    return { world };
});

addScene('CCD Test', () => {
    const worldSettings = createWorldSettings();
    worldSettings.gravity = [0, 0, 0];

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    const sphereShape = sphere.create({ radius: 0.5 });
    const wallShape = box.create({ halfExtents: [0.5, 5, 5], convexRadius: 0.05 });

    // Static wall
    rigidBody.create(world, {
        shape: wallShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [10, 0, 0],
        restitution: 0,
    });

    // Fast sphere WITHOUT CCD (will tunnel through wall)
    const sphereNoCCD = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-10, 2, 0],
        linearDamping: 0,
        angularDamping: 0,
        motionQuality: motionProperties.MotionQuality.DISCRETE,
    });
    vec3.set(sphereNoCCD.motionProperties.linearVelocity, 300, 0, 0);

    // Fast sphere WITH CCD (should collide with wall)
    const sphereWithCCD = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-10, -2, 0],
        linearDamping: 0,
        angularDamping: 0,
        motionQuality: motionProperties.MotionQuality.LINEAR_CAST,
    });
    vec3.set(sphereWithCCD.motionProperties.linearVelocity, 300, 0, 0);

    return { world };
});

addScene('Point Constraint - Chain', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static ground
    const groundShape = box.create({ halfExtents: [15, 0.5, 15] });
    rigidBody.create(world, {
        shape: groundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
        restitution: 0.3,
        friction: 0.5,
    });

    // Chain parameters
    const chainLength = 5;
    const linkSpacing = 1.5;
    const chainBodies: { id: number }[] = [];

    // Static anchor for chain
    const chainAnchor = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.2 }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 12, 0],
    });
    chainBodies.push(chainAnchor);

    // Create chain links
    for (let i = 0; i < chainLength; i++) {
        const linkBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.3, 0.3, 0.3], convexRadius: 0.05 }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [0, 12 - (i + 1) * linkSpacing, 0],
            restitution: 0.1,
            friction: 0.5,
        });
        chainBodies.push(linkBody);

        // Connect to previous body
        const prevBody = chainBodies[i];
        const connectionY = 12 - (i + 0.5) * linkSpacing;

        pointConstraint.create(world, {
            bodyIdA: prevBody.id,
            bodyIdB: linkBody.id,
            space: ConstraintSpace.WORLD,
            pointA: [0, connectionY, 0],
            pointB: [0, connectionY, 0],
        });

        linkBody.motionProperties.linearVelocity[0] = 1; // initial push to start swinging
    }

    return { world, listener: createConstraintFilterListener() };
});

addScene('Distance Constraint - Chain', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static ground
    const groundShape = box.create({ halfExtents: [15, 0.5, 15] });
    rigidBody.create(world, {
        shape: groundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
        restitution: 0.3,
        friction: 0.5,
    });

    // Chain parameters
    const chainLength = 8;
    const linkSpacing = 1.0;
    const chainBodies: RigidBody[] = [];

    // Static anchor for chain (left side)
    const chainAnchor = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.15 }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [-4, 10, 0],
    });
    chainBodies.push(chainAnchor);

    // Create chain links
    for (let i = 0; i < chainLength; i++) {
        const linkBody = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.25 }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [-4 + (i + 1) * linkSpacing, 10, 0],
            restitution: 0.1,
            friction: 0.5,
        });
        chainBodies.push(linkBody);

        // Connect to previous body with distance constraint
        const prevBody = chainBodies[i];

        distanceConstraint.create(world, {
            bodyIdA: prevBody.id,
            bodyIdB: linkBody.id,
            space: ConstraintSpace.WORLD,
            pointA: prevBody.centerOfMassPosition,
            pointB: linkBody.centerOfMassPosition,
            // minDistance/maxDistance default to initial distance
        });
    }

    return { world, listener: createConstraintFilterListener() };
});

addScene('Distance Constraint - Rope Bridge', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // Static ground
    const groundShape = box.create({ halfExtents: [15, 0.5, 15] });
    rigidBody.create(world, {
        shape: groundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
        restitution: 0.3,
        friction: 0.5,
    });

    // Bridge parameters
    const bridgeLength = 12;
    const linkSpacing = 0.8;
    const bridgeY = 8;
    const bridgeStartX = -5;

    // Left anchor
    const leftAnchor = rigidBody.create(world, {
        shape: box.create({ halfExtents: [0.3, 0.3, 0.5] }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [bridgeStartX, bridgeY, 0],
    });

    // Right anchor
    const rightAnchor = rigidBody.create(world, {
        shape: box.create({ halfExtents: [0.3, 0.3, 0.5] }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [bridgeStartX + (bridgeLength + 1) * linkSpacing, bridgeY, 0],
    });

    // Create bridge planks
    const bridgeBodies: RigidBody[] = [leftAnchor];

    for (let i = 0; i < bridgeLength; i++) {
        const plank = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.3, 0.1, 0.4], convexRadius: 0.02 }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [bridgeStartX + (i + 1) * linkSpacing, bridgeY, 0],
            restitution: 0.1,
            friction: 0.8,
        });
        bridgeBodies.push(plank);

        // Connect to previous body
        const prevBody = bridgeBodies[i];
        distanceConstraint.create(world, {
            bodyIdA: prevBody.id,
            bodyIdB: plank.id,
            space: ConstraintSpace.WORLD,
            pointA: prevBody.centerOfMassPosition,
            pointB: plank.centerOfMassPosition,
        });
    }

    // Connect last plank to right anchor
    const lastPlank = bridgeBodies[bridgeBodies.length - 1];
    distanceConstraint.create(world, {
        bodyIdA: lastPlank.id,
        bodyIdB: rightAnchor.id,
        space: ConstraintSpace.WORLD,
        pointA: lastPlank.centerOfMassPosition,
        pointB: rightAnchor.centerOfMassPosition,
    });

    // Drop a ball onto the bridge
    rigidBody.create(world, {
        shape: sphere.create({ radius: 0.5 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 12, 0],
        restitution: 0.3,
        friction: 0.5,
    });

    return { world, listener: createConstraintFilterListener() };
});

addScene('Distance Constraint - Spring', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Top fixed bar (centered at x=15 to cover -25 to 55)
    const barY = 25;
    const topBar = rigidBody.create(world, {
        shape: box.create({ halfExtents: [50, 0.5, 0.5] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [15, barY, 0],
    });

    // Group 1: Same frequency (0.33 Hz), no damping, different spring lengths
    // Left side: x = -25 to -5
    for (let i = 0; i < 5; i++) {
        const attachmentX = -25 + i * 5;
        const springLength = 8 + i * 2; // 8, 10, 12, 14, 16
        const bodyY = barY - springLength;

        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [attachmentX, bodyY, 0],
            linearDamping: 0,
            angularDamping: 0,
        });

        // Create spring constraint
        distanceConstraint.create(world, {
            bodyIdA: topBar.id,
            bodyIdB: body.id,
            space: ConstraintSpace.WORLD,
            pointA: [attachmentX, barY, 0],
            pointB: [attachmentX, bodyY, 0],
            springSettings: {
                mode: SpringMode.FREQUENCY_AND_DAMPING,
                frequencyOrStiffness: 0.33,
                damping: 0,
            },
        });

        // Move body up to start oscillating
        rigidBody.setTransform(world, body, [attachmentX, barY - 4, 0], body.quaternion, false);
    }

    // Group 2: Different frequencies (0.2 to 1.0 Hz), no damping
    // Center: x = 5 to 25
    for (let i = 0; i < 5; i++) {
        const attachmentX = 5 + i * 5;
        const springLength = 12;
        const bodyY = barY - springLength;
        const frequency = 0.2 + i * 0.2; // 0.2, 0.4, 0.6, 0.8, 1.0 Hz

        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [attachmentX, bodyY, 0],
            linearDamping: 0,
            angularDamping: 0,
        });

        distanceConstraint.create(world, {
            bodyIdA: topBar.id,
            bodyIdB: body.id,
            space: ConstraintSpace.WORLD,
            pointA: [attachmentX, barY, 0],
            pointB: [attachmentX, bodyY, 0],
            springSettings: {
                mode: SpringMode.FREQUENCY_AND_DAMPING,
                frequencyOrStiffness: frequency,
                damping: 0,
            },
        });

        // Move body up to start oscillating
        rigidBody.setTransform(world, body, [attachmentX, barY - 4, 0], body.quaternion, false);
    }

    // Group 3: Same frequency (0.5 Hz), different damping (0 to 1.0)
    // Right side: x = 35 to 55
    for (let i = 0; i < 5; i++) {
        const attachmentX = 35 + i * 5;
        const springLength = 12;
        const bodyY = barY - springLength;
        const dampingRatio = i * 0.25; // 0, 0.25, 0.5, 0.75, 1.0 (critical damping)

        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [attachmentX, bodyY, 0],
            linearDamping: 0,
            angularDamping: 0,
        });

        distanceConstraint.create(world, {
            bodyIdA: topBar.id,
            bodyIdB: body.id,
            space: ConstraintSpace.WORLD,
            pointA: [attachmentX, barY, 0],
            pointB: [attachmentX, bodyY, 0],
            springSettings: {
                mode: SpringMode.FREQUENCY_AND_DAMPING,
                frequencyOrStiffness: 0.5,
                damping: dampingRatio,
            },
        });

        // Move body up to start oscillating
        rigidBody.setTransform(world, body, [attachmentX, barY - 4, 0], body.quaternion, false);
    }

    return { world, listener: createConstraintFilterListener() };
});

addScene('Hinge Constraint', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);

    const world = createWorld(worldSettings);

    // Static anchor box
    const anchor = rigidBody.create(world, {
        shape: box.create({ halfExtents: [1, 1, 1] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 10, 0],
    });

    // Dynamic box connected by hinge
    const pendulum = rigidBody.create(world, {
        shape: box.create({ halfExtents: [1, 1, 1] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [3, 10, 0],
    });

    // Hinge constraint - rotates around Z axis (swings in XY plane)
    hingeConstraint.create(world, {
        bodyIdA: anchor.id,
        bodyIdB: pendulum.id,
        space: ConstraintSpace.WORLD,
        pointA: [1, 10, 0],
        pointB: [2, 10, 0],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
    });

    return { world, listener: createConstraintFilterListener() };
});

addScene('Fixed Constraint', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [50, 0.5, 50] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Two cubes welded together with a fixed constraint
    const cubeA = rigidBody.create(world, {
        shape: box.create({ halfExtents: [1, 1, 1] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-1.5, 10, 0],
    });

    const cubeB = rigidBody.create(world, {
        shape: box.create({ halfExtents: [1, 1, 1] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [1.5, 11, 0],
    });

    // Fix the two cubes together - they will fall and tumble as one rigid body
    fixedConstraint.create(world, {
        bodyIdA: cubeA.id,
        bodyIdB: cubeB.id,
        autoDetectPoint: true,
    });

    return { world, listener: createConstraintFilterListener() };
});

addScene('SwingTwist Constraint Chain', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);
    // Note: We don't enable collision between moving objects to avoid adjacent capsule collisions

    const world = createWorld(worldSettings);

    // Floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [50, 0.5, 50] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    const halfCapsuleHeight = 1.5;
    const capsuleRadius = 0.5;
    const chainLength = 10;

    // Swing-twist limits (in radians)
    const normalHalfConeAngle = Math.PI / 4; // 45 degrees - swing around Z axis
    const planeHalfConeAngle = Math.PI / 6; // 30 degrees - swing around Y axis
    const twistMinAngle = -Math.PI / 4; // -45 degrees
    const twistMaxAngle = Math.PI / 4; // 45 degrees

    let prevBody: RigidBody | null = null;
    const startPosition: Vec3 = [0, 25, 0];
    const position = vec3.clone(startPosition);

    // Create chain of capsules connected by swing-twist constraints
    for (let i = 0; i < chainLength; i++) {
        // Move position for this segment (capsules are oriented along X axis)
        position[0] += 2.0 * halfCapsuleHeight;

        // Rotation: rotate capsule to lie along X axis, plus incremental twist
        const baseRotation = quat.create();
        quat.rotateZ(baseRotation, baseRotation, Math.PI / 2); // Capsule along X
        const incrementalTwist = quat.create();
        quat.rotateX(incrementalTwist, incrementalTwist, 0.25 * Math.PI * i); // Twist around X
        const rotation = quat.create();
        quat.multiply(rotation, incrementalTwist, baseRotation);

        const body = rigidBody.create(world, {
            shape: capsule.create({ halfHeightOfCylinder: halfCapsuleHeight, radius: capsuleRadius }),
            objectLayer: i === 0 ? OBJECT_LAYER_NON_MOVING : OBJECT_LAYER_MOVING,
            motionType: i === 0 ? MotionType.STATIC : MotionType.DYNAMIC,
            position: vec3.clone(position),
            quaternion: rotation,
        });

        // Don't allow sleeping so the chain stays active
        if (i !== 0) {
            body.motionProperties.allowSleeping = false;
        }

        // Create swing-twist constraint between adjacent bodies
        if (prevBody !== null) {
            // Constraint position is at the junction between capsules
            const constraintPos = vec3.fromValues(position[0] - halfCapsuleHeight, position[1], position[2]);

            swingTwistConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                space: ConstraintSpace.WORLD,
                position1: constraintPos,
                position2: constraintPos,
                // Twist axis is along X (the capsule's length axis)
                twistAxis1: [1, 0, 0],
                twistAxis2: [1, 0, 0],
                // Plane axis is along Y
                planeAxis1: [0, 1, 0],
                planeAxis2: [0, 1, 0],
                // Swing limits
                normalHalfConeAngle: normalHalfConeAngle,
                planeHalfConeAngle: planeHalfConeAngle,
                // Twist limits
                twistMinAngle: twistMinAngle,
                twistMaxAngle: twistMaxAngle,
                // Optional: add some friction
                maxFrictionTorque: 0.5,
            });
        }

        prevBody = body;
    }

    return { world, listener: createConstraintFilterListener() };
});

addScene('SwingTwist Ragdoll Shoulder', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);

    const world = createWorld(worldSettings);

    // Floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [50, 0.5, 50] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Static "torso" (the shoulder attachment point)
    const torso = rigidBody.create(world, {
        shape: box.create({ halfExtents: [2, 1.5, 1] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 10, 0],
    });

    // Dynamic "upper arm" connected to torso with swing-twist constraint
    const upperArm = rigidBody.create(world, {
        shape: capsule.create({ halfHeightOfCylinder: 1.5, radius: 0.3 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [4, 10, 0],
    });
    // Capsule oriented along X axis (rotate 90° around Z)
    const armRotation = quat.create();
    quat.fromEuler(armRotation, euler.fromValues(0, 0, Math.PI / 2, 'xyz'));
    rigidBody.setTransform(world, upperArm, upperArm.position, armRotation, false);

    // Shoulder joint with realistic limits
    // Twist axis points outward (along the arm)
    // Allows the arm to swing forward/backward and up/down, plus rotate around its axis
    swingTwistConstraint.create(world, {
        bodyIdA: torso.id,
        bodyIdB: upperArm.id,
        space: ConstraintSpace.WORLD,
        position1: [2, 10, 0],
        position2: [2.5, 10, 0],
        // Twist axis is along X (arm's length axis)
        twistAxis1: [1, 0, 0],
        twistAxis2: [1, 0, 0],
        // Plane axis is along Z (forward/backward swing plane)
        planeAxis1: [0, 0, 1],
        planeAxis2: [0, 0, 1],
        // Shoulder has wide range of motion
        normalHalfConeAngle: Math.PI / 2, // 90 degrees up/down
        planeHalfConeAngle: Math.PI / 3, // 60 degrees forward/backward
        // Twist (arm rotation) has limited range
        twistMinAngle: -Math.PI / 3, // -60 degrees
        twistMaxAngle: Math.PI / 2, // 90 degrees
        // Add some friction to simulate muscle resistance
        maxFrictionTorque: 1.0,
    });

    return { world, listener: createConstraintFilterListener() };
});

addScene('Slider Constraint', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [50, 0.5, 50] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Static rail (the slider anchor)
    const rail = rigidBody.create(world, {
        shape: box.create({ halfExtents: [5, 0.2, 0.2] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 5, 0],
    });

    // Dynamic slider box
    const slider = rigidBody.create(world, {
        shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-3, 5, 0],
    });

    slider.motionProperties.linearVelocity[0] = 10; // initial velocity along the rail

    // Create slider constraint - box can only slide along the rail (X axis)
    sliderConstraint.create(world, {
        bodyIdA: rail.id,
        bodyIdB: slider.id,
        space: ConstraintSpace.WORLD,
        pointA: [0, 5, 0],
        pointB: [-3, 5, 0],
        sliderAxisA: [1, 0, 0],
        sliderAxisB: [1, 0, 0],
        normalAxisA: [0, 1, 0],
        normalAxisB: [0, 1, 0],
        limitsMin: -4,
        limitsMax: 4,
    });

    return { world, listener: createConstraintFilterListener() };
});

addScene('Cone Constraint', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [50, 0.5, 50] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    // Static anchor point
    const anchor = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.2 }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 5, 0],
    });

    // Dynamic pendulum arm
    const arm = rigidBody.create(world, {
        shape: capsule.create({ halfHeightOfCylinder: 1, radius: 0.5 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0], // hanging below anchor
        quaternion: quat.fromDegrees(quat.create(), 0, 0, 0, 'xyz'), // aligned with Y axis
    });

    arm.motionProperties.linearVelocity[0] = 10;

    // Create cone constraint - arm can swing within a 45° cone
    // Twist axis points down (-Y) for both bodies
    coneConstraint.create(world, {
        bodyIdA: anchor.id,
        bodyIdB: arm.id,
        space: ConstraintSpace.WORLD,
        pointA: [0, 5, 0],
        pointB: [0, 4.5, 0], // top of capsule
        twistAxisA: [0, -1, 0], // pointing down
        twistAxisB: [0, -1, 0],
        halfConeAngle: Math.PI / 4, // 45 degrees
    });

    // Give it some initial angular velocity to swing
    arm.motionProperties.angularVelocity[0] = 2;
    arm.motionProperties.angularVelocity[2] = 1;

    return { world, listener: createConstraintFilterListener() };
});

addScene('SixDOF Constraint', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // Floor
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [50, 0.5, 50] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.5, 0],
    });

    const spacing = 4;
    // limitMin/limitMax arrays: [TranslationX, TranslationY, TranslationZ, RotationX, RotationY, RotationZ]

    // ============ Example 1: Fixed All Axes (like a weld/fixed constraint) ============
    const anchor1 = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.3 }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [-2 * spacing, 5, 0],
    });

    const body1 = rigidBody.create(world, {
        shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-2 * spacing, 4, 0],
    });

    // All axes fixed - acts like a fixed constraint (limitMin >= limitMax means fixed)
    sixDOFConstraint.create(world, {
        bodyIdA: anchor1.id,
        bodyIdB: body1.id,
        space: ConstraintSpace.WORLD,
        position1: [-2 * spacing, 4.5, 0],
        position2: [-2 * spacing, 4.5, 0],
        axisX1: [1, 0, 0],
        axisY1: [0, 1, 0],
        axisX2: [1, 0, 0],
        axisY2: [0, 1, 0],
        // [TransX, TransY, TransZ, RotX, RotY, RotZ] - all fixed (min >= max)
        limitMin: [0, 0, 0, 0, 0, 0],
        limitMax: [0, 0, 0, 0, 0, 0],
    });

    // ============ Example 2: Slider (only X translation free with limits) ============
    const anchor2 = rigidBody.create(world, {
        shape: box.create({ halfExtents: [3, 0.1, 0.1] }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [-1 * spacing, 5, 0],
    });

    const body2 = rigidBody.create(world, {
        shape: box.create({ halfExtents: [0.4, 0.4, 0.4] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [-1 * spacing - 1, 5, 0],
    });

    body2.motionProperties.linearVelocity[0] = 5; // push it along X

    // X translation limited [-2.5, 2.5], everything else fixed
    sixDOFConstraint.create(world, {
        bodyIdA: anchor2.id,
        bodyIdB: body2.id,
        space: ConstraintSpace.WORLD,
        position1: [-1 * spacing, 5, 0],
        position2: [-1 * spacing - 1, 5, 0],
        axisX1: [1, 0, 0],
        axisY1: [0, 1, 0],
        axisX2: [1, 0, 0],
        axisY2: [0, 1, 0],
        // X translation limited, Y/Z translation fixed, all rotations fixed
        limitMin: [-2.5, 0, 0, 0, 0, 0],
        limitMax: [2.5, 0, 0, 0, 0, 0],
    });

    // ============ Example 3: Hinge-like (only Z rotation free) ============
    const anchor3 = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.2 }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [0, 5, 0],
    });

    const body3 = rigidBody.create(world, {
        shape: capsule.create({ halfHeightOfCylinder: 1, radius: 0.2 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [0, 3.5, 0],
        quaternion: quat.fromDegrees(quat.create(), 0, 0, 90, 'xyz'), // horizontal
    });

    // Z rotation free (like a hinge), everything else fixed
    sixDOFConstraint.create(world, {
        bodyIdA: anchor3.id,
        bodyIdB: body3.id,
        space: ConstraintSpace.WORLD,
        position1: [0, 5, 0],
        position2: [-1.2, 3.5, 0],
        axisX1: [1, 0, 0],
        axisY1: [0, 1, 0],
        axisX2: [0, 1, 0], // rotated 90 deg
        axisY2: [-1, 0, 0],
        // All translations fixed, RX/RY fixed, RZ free
        limitMin: [0, 0, 0, 0, 0, -Infinity],
        limitMax: [0, 0, 0, 0, 0, Infinity],
    });

    // ============ Example 4: Ball and socket (all rotations free) ============
    const anchor4 = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.3 }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [1 * spacing, 5, 0],
    });

    const body4 = rigidBody.create(world, {
        shape: capsule.create({ halfHeightOfCylinder: 1, radius: 0.3 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [1 * spacing, 3, 0],
    });

    body4.motionProperties.angularVelocity[0] = 3;
    body4.motionProperties.angularVelocity[2] = 2;

    // All rotations free (ball socket), translations fixed
    sixDOFConstraint.create(world, {
        bodyIdA: anchor4.id,
        bodyIdB: body4.id,
        space: ConstraintSpace.WORLD,
        position1: [1 * spacing, 5, 0],
        position2: [1 * spacing, 4.3, 0],
        axisX1: [1, 0, 0],
        axisY1: [0, 1, 0],
        axisX2: [1, 0, 0],
        axisY2: [0, 1, 0],
        // All translations fixed, all rotations free
        limitMin: [0, 0, 0, -Infinity, -Infinity, -Infinity],
        limitMax: [0, 0, 0, Infinity, Infinity, Infinity],
    });

    // ============ Example 5: Limited swing (like a joystick) ============
    const anchor5 = rigidBody.create(world, {
        shape: sphere.create({ radius: 0.3 }),
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        position: [2 * spacing, 5, 0],
    });

    const body5 = rigidBody.create(world, {
        shape: capsule.create({ halfHeightOfCylinder: 1, radius: 0.2 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [2 * spacing, 3.5, 0],
    });

    body5.motionProperties.linearVelocity[0] = 5;

    // Limited swing/tilt in X and Z rotation, Y twist locked
    const maxSwing = Math.PI / 4; // 45 degrees
    sixDOFConstraint.create(world, {
        bodyIdA: anchor5.id,
        bodyIdB: body5.id,
        space: ConstraintSpace.WORLD,
        position1: [2 * spacing, 5, 0],
        position2: [2 * spacing, 4.5, 0],
        axisX1: [1, 0, 0],
        axisY1: [0, 1, 0],
        axisX2: [1, 0, 0],
        axisY2: [0, 1, 0],
        // All translations fixed, limited swing X/Z, no twist (Y)
        limitMin: [0, 0, 0, -maxSwing, 0, -maxSwing],
        limitMax: [0, 0, 0, maxSwing, 0, maxSwing],
    });

    return { world, listener: createConstraintFilterListener() };
});

addScene('Offset Center of Mass', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

    const world = createWorld(worldSettings);

    // floor
    const floorShape = box.create({ halfExtents: [50, 0.5, 50] });
    rigidBody.create(world, {
        shape: floorShape,
        position: [0, -0.5, 0],
        objectLayer: OBJECT_LAYER_NON_MOVING,
        motionType: MotionType.STATIC,
        friction: 1.0,
    });

    // sphere with center of mass moved to the left side
    const leftOffsetShape = offsetCenterOfMass.create({
        shape: sphere.create({ radius: 1.0 }),
        offset: [-1, 0, 0],
    });
    rigidBody.create(world, {
        shape: leftOffsetShape,
        position: [-5, 5, 0],
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        friction: 1.0,
    });

    // sphere with center of mass centered
    rigidBody.create(world, {
        shape: sphere.create({ radius: 1.0 }),
        position: [0, 5, 0],
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        friction: 1.0,
    });

    // sphere with center of mass moved to the right side
    const rightOffsetShape = offsetCenterOfMass.create({
        shape: sphere.create({ radius: 1.0 }),
        offset: [1, 0, 0],
    });
    rigidBody.create(world, {
        shape: rightOffsetShape,
        position: [5, 5, 0],
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        friction: 1.0,
    });

    // create body and apply a large angular impulse to see that it spins around the COM
    const rotatingOffsetShape1 = offsetCenterOfMass.create({
        shape: sphere.create({ radius: 1.0 }),
        offset: [-3, 0, 0],
    });
    const bodyRotating1 = rigidBody.create(world, {
        shape: rotatingOffsetShape1,
        position: [-5, 5, 10],
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        linearDamping: 0.0,
        angularDamping: 0.0,
    });
    bodyRotating1.motionProperties.gravityFactor = 0.0;
    rigidBody.addAngularImpulse(world, bodyRotating1, [0, 1.0e6, 0]);

    // create the same body but this time apply a torque
    const rotatingOffsetShape2 = offsetCenterOfMass.create({
        shape: sphere.create({ radius: 1.0 }),
        offset: [-3, 0, 0],
    });
    const bodyRotating2 = rigidBody.create(world, {
        shape: rotatingOffsetShape2,
        position: [5, 5, 10],
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        linearDamping: 0.0,
        angularDamping: 0.0,
    });
    bodyRotating2.motionProperties.gravityFactor = 0.0;
    // assuming physics sim is at 60Hz here, otherwise the bodies won't rotate with the same speed
    rigidBody.addTorque(world, bodyRotating2, [0, 1.0e6 * 60.0, 0], true);

    return { world };
});

addScene('Enhanced Internal Edge Removal', () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    // scene 1: dense grid of boxes with sliding objects
    {
        const boxShape = box.create({ halfExtents: [1, 1, 1] });
        const gridSize = 2;

        // create 20x20 grid of boxes
        for (let x = -10; x < 10; x++) {
            for (let z = -10; z < 10; z++) {
                rigidBody.create(world, {
                    shape: boxShape,
                    objectLayer: OBJECT_LAYER_NOT_MOVING,
                    motionType: MotionType.STATIC,
                    position: [gridSize * x, -1, gridSize * z - 40],
                });
            }
        }

        // sliding objects - normal vs enhanced
        const startPos: Vec3 = [-18, 1.9, -40];

        for (let enhanced = 0; enhanced < 2; enhanced++) {
            const pos: Vec3 = [...startPos];
            pos[2] -= 12;

            // box
            const slidingBox = rigidBody.create(world, {
                shape: box.create({ halfExtents: [2, 2, 2] }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: pos,
                enhancedInternalEdgeRemoval: enhanced === 1,
            });
            vec3.set(slidingBox.motionProperties.linearVelocity, 20, 0, 0);
            pos[2] += 5;

            // sphere
            const slidingSphere = rigidBody.create(world, {
                shape: sphere.create({ radius: 2 }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: pos,
                enhancedInternalEdgeRemoval: enhanced === 1,
            });
            vec3.set(slidingSphere.motionProperties.linearVelocity, 20, 0, 0);
            pos[2] += 5;

            // compound (simulate with offset boxes)
            const smallBox = box.create({ halfExtents: [0.1, 0.1, 0.1] });
            for (let ox = 0; ox < 2; ox++) {
                for (let oy = 0; oy < 2; oy++) {
                    for (let oz = 0; oz < 2; oz++) {
                        const compoundBody = rigidBody.create(world, {
                            shape: smallBox,
                            objectLayer: OBJECT_LAYER_MOVING,
                            motionType: MotionType.DYNAMIC,
                            position: [
                                pos[0] + (ox === 0 ? -1.9 : 1.9),
                                pos[1] + (oy === 0 ? -1.9 : 1.9),
                                pos[2] + (oz === 0 ? -1.9 : 1.9),
                            ],
                            enhancedInternalEdgeRemoval: enhanced === 1,
                        });
                        vec3.set(compoundBody.motionProperties.linearVelocity, 20, 0, 0);
                    }
                }
            }
            pos[2] += 7;

            startPos[2] += 5;
        }
    }

    // scene 2: dense triangle mesh with sliding objects
    {
        const gridSize = 2;
        const positions: number[] = [];
        const indices: number[] = [];
        let vertexIndex = 0;

        // create 20x20 grid of triangles
        for (let x = -10; x < 10; x++) {
            for (let z = -10; z < 10; z++) {
                const x1 = gridSize * x;
                const z1 = gridSize * z;
                const x2 = x1 + gridSize;
                const z2 = z1 + gridSize;

                // add 4 vertices for this quad
                positions.push(x1, 0, z1, x1, 0, z2, x2, 0, z2, x2, 0, z1);

                // triangle 1: v0, v1, v2
                indices.push(vertexIndex, vertexIndex + 1, vertexIndex + 2);
                // triangle 2: v0, v2, v3
                indices.push(vertexIndex, vertexIndex + 2, vertexIndex + 3);

                vertexIndex += 4;
            }
        }

        const meshShape = triangleMesh.create({
            positions,
            indices,
            activeEdgeCosThresholdAngle: -1.0, // disable active edge determination, rely only on enhanced internal edge removal
        });

        rigidBody.create(world, {
            shape: meshShape,
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: [0, 0, 0],
        });

        // sliding objects - normal vs enhanced
        const startPos: Vec3 = [-18, 1.9, 0];

        for (let enhanced = 0; enhanced < 2; enhanced++) {
            const pos: Vec3 = [...startPos];
            pos[2] -= 12;

            // box
            const slidingBox = rigidBody.create(world, {
                shape: box.create({ halfExtents: [2, 2, 2] }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: pos,
                enhancedInternalEdgeRemoval: enhanced === 1,
            });
            vec3.set(slidingBox.motionProperties.linearVelocity, 20, 0, 0);
            pos[2] += 5;

            // sphere
            const slidingSphere = rigidBody.create(world, {
                shape: sphere.create({ radius: 2 }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: pos,
                enhancedInternalEdgeRemoval: enhanced === 1,
            });
            vec3.set(slidingSphere.motionProperties.linearVelocity, 20, 0, 0);
            pos[2] += 5;
        }
    }

    // scene 3: L-shaped mesh - test external edge preservation
    {
        const height = 0.5;
        const halfWidth = 5;
        const halfLength = 2;

        const positions = [
            // horizontal part vertices
            -halfLength,
            0,
            halfWidth,
            halfLength,
            0,
            -halfWidth,
            -halfLength,
            0,
            -halfWidth,
            halfLength,
            0,
            halfWidth,
            // vertical part vertices
            halfLength,
            height,
            halfWidth,
            halfLength,
            height,
            -halfWidth,
            halfLength,
            0,
            halfWidth,
            halfLength,
            0,
            -halfWidth,
        ];

        const indices = [
            // horizontal triangles
            0, 1, 2, 0, 3, 1,
            // vertical triangles
            4, 5, 6, 6, 5, 7,
        ];

        const lShapeMesh = triangleMesh.create({ positions, indices });

        rigidBody.create(world, {
            shape: lShapeMesh,
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: [0, 0, 30],
        });

        // roll spheres towards the edge
        let z = 28;
        for (let enhanced = 0; enhanced < 2; enhanced++) {
            const rollingSphere = rigidBody.create(world, {
                shape: sphere.create({ radius: 1 }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [0, 1, z],
                enhancedInternalEdgeRemoval: enhanced === 1,
            });
            vec3.set(rollingSphere.motionProperties.linearVelocity, 20, 0, 0);
            z += 4;
        }
    }

    // scene 4: fast rolling spheres on flat and sloped planes
    {
        // flat plane
        const flatMesh = triangleMesh.create({
            positions: [-10, 0, -10, -10, 0, 10, 10, 0, 10, 10, 0, -10],
            indices: [0, 1, 2, 0, 2, 3],
        });

        rigidBody.create(world, {
            shape: flatMesh,
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: [-10, 0, 50],
            friction: 1,
        });

        // rolling ball on flat plane
        rigidBody.create(world, {
            shape: sphere.create({ radius: 0.5 }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [-10, 1, 41],
            enhancedInternalEdgeRemoval: true,
            friction: 1,
        });

        // sloped plane (45 degrees)
        const slopeQuat = quat.create();
        quat.fromEuler(slopeQuat, euler.fromValues(Math.PI / 4, 0, 0, 'xyz'));

        rigidBody.create(world, {
            shape: flatMesh,
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: [10, 0, 50],
            quaternion: slopeQuat,
            friction: 1,
        });

        // rolling ball on slope
        rigidBody.create(world, {
            shape: sphere.create({ radius: 0.5 }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [10, 8, 44],
            enhancedInternalEdgeRemoval: true,
            friction: 1,
        });
    }

    // scene 5: compound shape falling through box bug test
    {
        // static box
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [2.5, 2.5, 2.5] }),
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: [0, 0, 70],
        });

        // compound shape (simulated with two separate boxes for now)
        // big horizontal box
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [2.5, 0.1, 0.1] }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [2 - 2.5, 5, 70],
            enhancedInternalEdgeRemoval: true,
        });

        // small vertical box
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [0.1, 1, 1] }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [2 + 0.1, 5, 70],
            enhancedInternalEdgeRemoval: true,
        });
    }

    // scene 6: super dense grid of triangles
    {
        const gridSize = 0.25;
        const positions: number[] = [];
        const indices: number[] = [];
        let vertexIndex = 0;

        // create 200x10 grid of small triangles
        for (let x = -100; x < 100; x++) {
            for (let z = -5; z < 5; z++) {
                const x1 = gridSize * x;
                const z1 = gridSize * z;
                const x2 = x1 + gridSize;
                const z2 = z1 + gridSize;

                // add 4 vertices for this quad
                positions.push(x1, 0, z1, x1, 0, z2, x2, 0, z2, x2, 0, z1);

                // triangle 1: v0, v1, v2
                indices.push(vertexIndex, vertexIndex + 1, vertexIndex + 2);
                // triangle 2: v0, v2, v3
                indices.push(vertexIndex, vertexIndex + 2, vertexIndex + 3);

                vertexIndex += 4;
            }
        }

        const denseMesh = triangleMesh.create({
            positions,
            indices,
            activeEdgeCosThresholdAngle: -1.0, // disable active edge determination, rely only on enhanced internal edge removal
        });

        rigidBody.create(world, {
            shape: denseMesh,
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: [0, 0, 80],
        });

        // fast sliding box with enhanced edge removal
        const fastBox = rigidBody.create(world, {
            shape: box.create({ halfExtents: [1, 1, 1] }),
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [-24, 0.9, 80],
            enhancedInternalEdgeRemoval: true,
        });
        vec3.set(fastBox.motionProperties.linearVelocity, 20, 0, 0);
    }

    return { world };
});

setupGui();

// Load scene from URL hash or default to first scene
const initialSceneIndex = loadSceneFromHash();
loadScene(initialSceneIndex);

// Listen for hash changes (back/forward navigation)
window.addEventListener('hashchange', () => {
    const sceneIndex = loadSceneFromHash();
    if (sceneIndex !== currentSceneIndex) {
        loadScene(sceneIndex);
    }
});

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    if (currentWorld) {
        debugUI.beginPerf(ui);
        updateWorld(currentWorld, currentListener ?? undefined, delta);
        debugUI.endPerf(ui);
        debugUI.updateStats(ui, currentWorld);
        debugRenderer.update(debugRendererState, currentWorld);
    }

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
