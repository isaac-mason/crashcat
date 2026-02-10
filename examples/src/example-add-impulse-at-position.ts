import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    CastRayStatus,
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
    registerAll,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 10, 15);
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

registerAll();

const worldSettings = createWorldSettings();

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

/* static ground */

const groundShape = box.create({ halfExtents: [30, 0.5, 30] });
rigidBody.create(world, {
    shape: groundShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0.3,
    friction: 0.5,
});

/* dynamic bodies */

const dynamicBodies: RigidBody[] = [];

// dynamic box
const boxShape = box.create({ halfExtents: [1, 1, 1], convexRadius: 0.05 });
const boxBody = rigidBody.create(world, {
    shape: boxShape,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-3, 2, 0),
    quaternion: quat.create(),
    restitution: 0.3,
    friction: 0.5,
    mass: 10,
});
dynamicBodies.push(boxBody);

// dynamic sphere
const sphereShape = sphere.create({ radius: 1 });
const sphereBody = rigidBody.create(world, {
    shape: sphereShape,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(3, 2, 0),
    quaternion: quat.create(),
    restitution: 0.3,
    friction: 0.5,
    mass: 10,
});
dynamicBodies.push(sphereBody);

/* raycasting setup */

const rayCollector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();
const queryFilter = filter.create(worldSettings.layers);

/* hit point visualization */

const hitPointGeometry = new THREE.SphereGeometry(0.1, 16, 16);
const hitPointMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const hitPointMesh = new THREE.Mesh(hitPointGeometry, hitPointMaterial);
hitPointMesh.visible = false;
scene.add(hitPointMesh);

/* impulse arrow visualization */

const arrowHelper = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0, 0),
    1,
    0x00ff00,
    0.2,
    0.1,
);
arrowHelper.visible = false;
scene.add(arrowHelper);

/* mouse/pointer handling */

const mouse = new THREE.Vector2();
const raycaster = new THREE.Raycaster();

window.addEventListener('pointermove', (event) => {
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
});

const settings = {
    impulseStrength: 20,
};

window.addEventListener('click', () => {
    // get ray from camera through mouse position
    raycaster.setFromCamera(mouse, camera);
    const rayOrigin = raycaster.ray.origin;
    const rayDirection = raycaster.ray.direction;

    const origin = vec3.fromValues(rayOrigin.x, rayOrigin.y, rayOrigin.z);
    const direction = vec3.fromValues(rayDirection.x, rayDirection.y, rayDirection.z);

    // perform raycast
    const maxDistance = 100;
    rayCollector.reset();
    castRay(world, rayCollector, raySettings, origin, direction, maxDistance, queryFilter);

    if (rayCollector.hit.status === CastRayStatus.COLLIDING) {
        const hitBody = rigidBody.get(world, rayCollector.hit.bodyIdB);
        if (!hitBody || hitBody.motionType !== MotionType.DYNAMIC) {
            return;
        }

        // calculate hit position in world space
        const hitDistance = rayCollector.hit.fraction * maxDistance;
        const hitPosition = vec3.fromValues(
            rayOrigin.x + rayDirection.x * hitDistance,
            rayOrigin.y + rayDirection.y * hitDistance,
            rayOrigin.z + rayDirection.z * hitDistance,
        );

        // calculate impulse direction (along the ray direction)
        const impulse = vec3.create();
        vec3.scale(impulse, direction, settings.impulseStrength);

        // apply impulse at the hit position
        rigidBody.addImpulseAtPosition(world, hitBody, impulse, hitPosition);

        // visualize the hit point and impulse direction
        hitPointMesh.position.set(hitPosition[0], hitPosition[1], hitPosition[2]);
        hitPointMesh.visible = true;

        arrowHelper.position.set(hitPosition[0], hitPosition[1], hitPosition[2]);
        arrowHelper.setDirection(new THREE.Vector3(direction[0], direction[1], direction[2]));
        arrowHelper.setLength(2);
        arrowHelper.visible = true;

        // hide visualization after a short delay
        setTimeout(() => {
            hitPointMesh.visible = false;
            arrowHelper.visible = false;
        }, 200);
    }
});

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
debugRendererState.options.bodies.enabled = true;
scene.add(debugRendererState.object3d);

ui.gui.title('Add Impulse at Position');
ui.gui.add(settings, 'impulseStrength', 1, 100, 1).name('Impulse Strength');

/* simulation loop */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // update physics
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
