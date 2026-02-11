import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import type { RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAll,
    rigidBody,
    sphere,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* constants */

const N = 100;
const SPHERE_RADIUS = 1;
const COLORS = ['orange', 'hotpink', '#ffffff'];
const LINEAR_DAMPING = 2;
const ANGULAR_DAMPING = 4;

/* state */

const pointOfGravity = vec3.fromValues(0, 0, 0);
let pointOfGravityIntensity = 0.1;
let pointerDown = false;

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 0, 20);
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

const ambientLight = new THREE.AmbientLight(0xffffff, 1.2);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
directionalLight.position.set(5, 5, 5);
scene.add(directionalLight);

/* physics world */

registerAll();

const worldSettings = createWorldSettings();
worldSettings.gravity = [0, 0, 0];

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

const world = createWorld(worldSettings);

/* instanced mesh setup */

const sphereGeometry = new THREE.SphereGeometry(SPHERE_RADIUS, 32, 16);
const sphereMaterial = new THREE.MeshPhysicalMaterial({ roughness: 0.4 });
const instancedMesh = new THREE.InstancedMesh(sphereGeometry, sphereMaterial, N);
instancedMesh.frustumCulled = false;
scene.add(instancedMesh);

// set instance colors
const color = new THREE.Color();
for (let i = 0; i < N; i++) {
    color.set(COLORS[i % COLORS.length]);
    instancedMesh.setColorAt(i, color);
}
instancedMesh.instanceColor!.needsUpdate = true;

/* create physics bodies */

const bodies: RigidBody[] = [];
const matrix = new THREE.Matrix4();
const position = new THREE.Vector3();
const quaternion = new THREE.Quaternion();
const scale = new THREE.Vector3(1, 1, 1);

const sphereShape = sphere.create({ radius: SPHERE_RADIUS });

function randomBetween(a: number, b: number): number {
    const min = Math.min(a, b);
    const max = Math.max(a, b);
    return Math.random() * (max - min) + min;
}

for (let i = 0; i < N; i++) {
    const side = Math.random() > 0.5 ? 1 : -1;
    const x = side * randomBetween(50, 100);
    const y = randomBetween(-50, 50);
    const z = randomBetween(-10, 10);

    const body = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(x, y, z),
        quaternion: quat.create(),
        linearDamping: LINEAR_DAMPING,
        angularDamping: ANGULAR_DAMPING,
        friction: 0.1,
    });

    bodies.push(body);

    // set initial instance matrix
    position.set(x, y, z);
    quaternion.set(0, 0, 0, 1);
    matrix.compose(position, quaternion, scale);
    instancedMesh.setMatrixAt(i, matrix);
}
instancedMesh.instanceMatrix.needsUpdate = true;

/* kinematic pointer body */

const pointerBody = rigidBody.create(world, {
    shape: sphere.create({ radius: 5 }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.KINEMATIC,
    position: vec3.fromValues(0, -100, 0),
    restitution: 0.8,
    friction: 0.5,
});

const pointerLight = new THREE.PointLight(0xffffff, 3, 100, 1);
pointerLight.position.set(0, 0, 0);
scene.add(pointerLight);

/* mouse/pointer handling */

const mouse = new THREE.Vector2();
const pointerPosition = vec3.create();
const pointerQuaternion = quat.create();

window.addEventListener('pointermove', (event) => {
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
});

window.addEventListener('pointerdown', () => {
    pointerDown = true;
});

window.addEventListener('pointerup', () => {
    pointerDown = false;
});

/* impulse vector for gravity attraction */

const impulse = vec3.create();
const bodyPos = vec3.create();

function applyGravityTowardPoint(body: RigidBody) {
    vec3.set(bodyPos, body.position[0], body.position[1], body.position[2]);
    vec3.subtract(impulse, pointOfGravity, bodyPos);
    vec3.scale(impulse, impulse, pointOfGravityIntensity);

    // apply impulse to linear velocity
    const mp = body.motionProperties;
    mp.linearVelocity[0] += impulse[0];
    mp.linearVelocity[1] += impulse[1];
    mp.linearVelocity[2] += impulse[2];
}

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
options.bodies.enabled = false;
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

const maxDelta = 1 / 30;
let lastTime = performance.now();

function loop() {
    requestAnimationFrame(loop);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // update pointer world position based on mouse
    // project mouse to world space at z=0
    const viewportWidth = 2 * Math.tan((camera.fov * Math.PI) / 360) * camera.position.z;
    const viewportHeight = viewportWidth / camera.aspect;

    const pointerWorldX = (mouse.x * viewportWidth) / 2;
    const pointerWorldY = (mouse.y * viewportHeight) / 2;

    vec3.set(pointerPosition, pointerWorldX, pointerWorldY, 0);
    rigidBody.moveKinematic(pointerBody, pointerPosition, pointerQuaternion, delta);

    // update pointer light position
    pointerLight.position.set(pointerWorldX, pointerWorldY, 0);

    // update point of gravity
    if (pointerDown) {
        vec3.set(pointOfGravity, pointerWorldX, pointerWorldY, 0);
        pointOfGravityIntensity = 0.3;
    } else {
        vec3.set(pointOfGravity, 0, 0, 0);
        pointOfGravityIntensity = 0.2;
    }

    // apply gravity toward point for all bodies
    for (let i = 0; i < bodies.length; i++) {
        applyGravityTowardPoint(bodies[i]);
    }

    // update physics
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // update debug renderer
    debugRenderer.update(debugRendererState, world);

    // update instanced mesh matrices from physics bodies
    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        position.set(body.position[0], body.position[1], body.position[2]);
        quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
        matrix.compose(position, quaternion, scale);
        instancedMesh.setMatrixAt(i, matrix);
    }
    instancedMesh.instanceMatrix.needsUpdate = true;

    // render
    renderer.render(scene, camera);
}

loop();
