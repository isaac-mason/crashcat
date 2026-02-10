import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import type { RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    convexHull,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAll,
    rigidBody,
    sphere,
    updateWorld,
} from 'crashcat';
import { createShapeHelper, debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

type PhysicsObject = {
    object3d: THREE.Object3D;
    body: RigidBody;
};

/* rendering */

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 5, 10);
camera.lookAt(0, 3, 0);

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
orbitControls.target.set(0, 3, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 2048;
directionalLight.shadow.mapSize.height = 2048;
directionalLight.shadow.camera.left = -20;
directionalLight.shadow.camera.right = 20;
directionalLight.shadow.camera.top = 20;
directionalLight.shadow.camera.bottom = -20;
directionalLight.shadow.camera.near = 0.5;
directionalLight.shadow.camera.far = 50;
scene.add(directionalLight);

// Add rim light from behind-right of camera
const rimLight = new THREE.DirectionalLight(0xffffff, 0.4);
rimLight.position.set(3, 6, 18);
scene.add(rimLight);

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

// create static ground box body
rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0.2,
    friction: 0.5,
});

// create ground mesh
const groundGeometry = new THREE.CircleGeometry(25, 64);
groundGeometry.rotateX(-Math.PI / 2); // rotate to lie flat on XZ plane
const groundMaterial = new THREE.MeshStandardMaterial({ color: '#666' });
const groundMesh = new THREE.Mesh(groundGeometry, groundMaterial);
groundMesh.position.set(0, 0, 0);
groundMesh.receiveShadow = true;
scene.add(groundMesh);

/* kinematic body setup */

// load maxwell model
const gltfLoader = new GLTFLoader();
const gltf = await gltfLoader.loadAsync('./models/maxwell.glb');

let maxwellMesh: THREE.Mesh = undefined!;
gltf.scene.traverse((object) => {
    if (!maxwellMesh && object instanceof THREE.Mesh) {
        maxwellMesh = object;
    }
});

const maxwellScale = 0.1;
maxwellMesh!.scale.set(maxwellScale, maxwellScale, maxwellScale);

const geometry = maxwellMesh!.geometry as THREE.BufferGeometry;
const positions = geometry.getAttribute('position');

// extract vertex positions into flat array (scaled down 10x)
const maxwellMeshPositions: number[] = [];
for (let i = 0; i < positions.count; i++) {
    maxwellMeshPositions.push(
        positions.getX(i) * maxwellScale,
        positions.getY(i) * maxwellScale,
        positions.getZ(i) * maxwellScale,
    );
}

// build convex hull shape
const maxwellConvexHullShape = convexHull.create({
    positions: maxwellMeshPositions,
});

// offset maxwell mesh to align with convex hull's center of mass
// (hull vertices are stored relative to COM, but maxwell mesh vertices are not)
const shapeCOM = maxwellConvexHullShape.centerOfMass;
maxwellMesh!.position.set(-shapeCOM[0], -shapeCOM[1], -shapeCOM[2]);

// movement parameters
const kinematicPosition = vec3.fromValues(0, 3, 0);
const kinematicRotation = quat.create();
const MOVEMENT_RANGE = 2.5; // Movement range on x-axis: -2.5 to +2.5
const MOVEMENT_SPEED = 2; // Units per second
const ROTATION_SPEED = Math.PI * 1.5; // Radians per second

let kinematicTime = 0;

// create kinematic body
const kinematicBody = rigidBody.create(world, {
    shape: maxwellConvexHullShape,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.KINEMATIC,
    position: kinematicPosition,
    quaternion: kinematicRotation,
    restitution: 0.5,
    friction: 0.3,
});

// add convex hull shape helper with transparent material
const hullMaterial = new THREE.MeshStandardMaterial({
    color: 0x66ccff,
    transparent: true,
    opacity: 0.3,
    side: THREE.DoubleSide,
});
const hullHelper = createShapeHelper(maxwellConvexHullShape, { material: hullMaterial });
// scale slightly larger to prevent z-fighting
hullHelper.object.scale.set(1.01, 1.01, 1.01);
// Enable shadows on maxwell mesh
maxwellMesh!.castShadow = true;
maxwellMesh!.receiveShadow = true;
// create parent group to contain both maxwell and hull helper
const maxwellGroup = new THREE.Group();
maxwellGroup.add(maxwellMesh);
maxwellGroup.add(hullHelper.object);
scene.add(maxwellGroup);

/* physics objects tracking */

const physicsObjects: PhysicsObject[] = [
    {
        object3d: maxwellGroup,
        body: kinematicBody,
    },
];
const settings = {
    spawnInterval: 0.05, // seconds between spawns
    maxBalls: 100,
};

let timeSinceLastSpawn = 0;

function resetBallPosition(body: RigidBody): void {
    // spawn along the x-axis movement range, high above
    const x = (Math.random() - 0.5) * MOVEMENT_RANGE * 2;
    const y = 12;
    const z = 0;
    rigidBody.setPosition(world, body, vec3.fromValues(x, y, z), true);
    vec3.zero(body.motionProperties!.linearVelocity);
    vec3.zero(body.motionProperties!.angularVelocity);
}

function spawnBall() {
    if (physicsObjects.length - 1 >= settings.maxBalls) {
        // remove oldest ball (skip maxwell at index 0)
        const oldBall = physicsObjects.splice(1, 1)[0];
        if (oldBall) {
            rigidBody.remove(world, oldBall.body);
            scene.remove(oldBall.object3d);
            if (oldBall.object3d instanceof THREE.Mesh) {
                oldBall.object3d.geometry.dispose();
                (oldBall.object3d.material as THREE.Material).dispose();
            }
        }
    }

    const ballShape = sphere.create({ radius: 0.15 });

    const ballBody = rigidBody.create(world, {
        shape: ballShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 12, 0), // Will be randomized by resetBallPosition
        quaternion: quat.create(),
        restitution: 0.6,
        friction: 0.4,
    });

    // Set random spawn position
    resetBallPosition(ballBody);

    // create ball mesh
    const ballGeometry = new THREE.SphereGeometry(0.15, 16, 16);
    // random vibrant colors
    const randomColor = new THREE.Color().setHSL(Math.random(), 0.65 + Math.random() * 0.25, 0.55 + Math.random() * 0.15);
    const ballMaterial = new THREE.MeshStandardMaterial({ color: randomColor });
    const ballMesh = new THREE.Mesh(ballGeometry, ballMaterial);
    ballMesh.castShadow = true;
    ballMesh.receiveShadow = true;
    scene.add(ballMesh);

    physicsObjects.push({
        object3d: ballMesh,
        body: ballBody,
    });
}

/* helper to update mesh from physics body */

function updateMeshFromBody(physicsObject: PhysicsObject) {
    physicsObject.object3d.position.set(
        physicsObject.body.position[0],
        physicsObject.body.position[1],
        physicsObject.body.position[2],
    );
    physicsObject.object3d.quaternion.set(
        physicsObject.body.quaternion[0],
        physicsObject.body.quaternion[1],
        physicsObject.body.quaternion[2],
        physicsObject.body.quaternion[3],
    );
}

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
debugRendererState.options.bodies.enabled = false;
scene.add(debugRendererState.object3d);

/* simulation loop */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // update kinematic body position and rotation
    kinematicTime += delta;

    // move side to side along x-axis using sine wave
    const targetX = Math.sin(kinematicTime * MOVEMENT_SPEED) * MOVEMENT_RANGE;
    vec3.set(kinematicPosition, targetX, 3, 0);

    // rotate continuously around y-axis
    const rotationAngle = kinematicTime * ROTATION_SPEED;
    quat.setAxisAngle(kinematicRotation, vec3.fromValues(0, 1, 0), rotationAngle);

    // call moveKinematicBody with the target position and rotation
    rigidBody.moveKinematic(kinematicBody, kinematicPosition, kinematicRotation, delta);

    // spawn balls at regular intervals
    timeSinceLastSpawn += delta;
    if (timeSinceLastSpawn >= settings.spawnInterval) {
        timeSinceLastSpawn = 0;
        spawnBall();
    }

    // update physics with delta time
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    // update all physics objects and respawn balls that fall off
    for (let i = 1; i < physicsObjects.length; i++) {
        // Skip maxwell at index 0
        const physicsObject = physicsObjects[i];
        updateMeshFromBody(physicsObject);

        // Check if ball is outside radius of 15
        const pos = physicsObject.body.position;
        const distSq = pos[0] * pos[0] + pos[2] * pos[2]; // XZ plane distance
        if (distSq > 225) {
            // 15 * 15 = 225
            resetBallPosition(physicsObject.body);
        }
    }

    // Update maxwell
    updateMeshFromBody(physicsObjects[0]);

    // update orbit controls
    orbitControls.update();

    // render
    renderer.render(scene, camera);
}

animate();
