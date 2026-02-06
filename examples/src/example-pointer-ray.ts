import { quat, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { RigidBody, Shape } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    castRay,
    CastRayStatus,
    convexHull,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
    MotionType,
    registerAllShapes,
    rigidBody,
    sphere,
    updateWorld,
} from 'crashcat';
import { createShapeHelper, debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

type PhysicsObject = {
    batchedMesh: THREE.BatchedMesh;
    batchedMeshId: number;
    body: RigidBody;
    radialDirection: Vec3;
    minRadius: number;
    maxRadius: number;
    radialSpeed: number;
    radialPhase: number;
    angularVelocity: Vec3;
};

/* scratch variables for Three.js math */

const _matrix4 = new THREE.Matrix4();
const _vector3 = new THREE.Vector3();
const _quaternion = new THREE.Quaternion();
const _scale = new THREE.Vector3(1, 1, 1);
const _color = new THREE.Color();

/* rendering */

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 5, 40);
camera.lookAt(0, 0, 0);

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
orbitControls.target.set(0, 0, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 1);
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

const rimLight = new THREE.DirectionalLight(0xffffff, 0.4);
rimLight.position.set(3, 6, 18);
scene.add(rimLight);

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
const queryFilter = filter.create(worldSettings.layers);

/* reusable shapes */

// create shapes once to reduce work
const boxShape = box.create({ halfExtents: vec3.fromValues(0.8, 0.8, 0.8) });
const sphereShape = sphere.create({ radius: 0.8 });

// biome-ignore format: readability
const pyramidPositions = [
    // base
    -0.8, 0, -0.8, // v0
    0.8, 0, -0.8,  // v1
    0.8, 0, 0.8,   // v2
    -0.8, 0, 0.8,  // v3
    // apex
    0, 1.2, 0,     // v4
];

const convexHullShape = convexHull.create({
    positions: pyramidPositions,
});

/* batched mesh setup */

const colors = [
    0xff6b9d, // pink
    0x4ecdc4, // teal
    0xffe66d, // yellow
    0x95e1d3, // mint
    0xff6b6b, // red
    0x6c5ce7, // purple
    0x00b894, // green
    0xfdcb6e, // gold
];

const boxHelper = createShapeHelper(boxShape);
const sphereHelper = createShapeHelper(sphereShape);
const pyramidHelper = createShapeHelper(convexHullShape);

const boxGeometry = (boxHelper.object as THREE.Mesh).geometry;
const sphereGeometry = (sphereHelper.object as THREE.Mesh).geometry;
const pyramidGeometry = (pyramidHelper.object as THREE.Mesh).geometry;

// Ensure all geometries have UVs (required for BatchedMesh)
if (!boxGeometry.attributes.uv) {
    const vertexCount = boxGeometry.attributes.position.count;
    boxGeometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(vertexCount * 2), 2));
}
if (!sphereGeometry.attributes.uv) {
    const vertexCount = sphereGeometry.attributes.position.count;
    sphereGeometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(vertexCount * 2), 2));
}
if (!pyramidGeometry.attributes.uv) {
    const vertexCount = pyramidGeometry.attributes.position.count;
    pyramidGeometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(vertexCount * 2), 2));
}

const N = 1000;

// calculate total buffer requirements
const boxVertices = boxGeometry.attributes.position.count;
const boxIndices = boxGeometry.index?.count || 0;
const sphereVertices = sphereGeometry.attributes.position.count;
const sphereIndices = sphereGeometry.index?.count || 0;
const pyramidVertices = pyramidGeometry.attributes.position.count;
const pyramidIndices = pyramidGeometry.index?.count || 0;

const numPerShape = Math.ceil(N / 3);
const totalVertices = (boxVertices + sphereVertices + pyramidVertices) * numPerShape;
const totalIndices = (boxIndices + sphereIndices + pyramidIndices) * numPerShape;

// create single batched mesh for all shapes
const batchedMesh = new THREE.BatchedMesh(
    N,
    totalVertices,
    totalIndices,
    new THREE.MeshStandardMaterial({
        roughness: 0.7,
        metalness: 0.3,
    }),
);
batchedMesh.castShadow = true;
batchedMesh.receiveShadow = true;
scene.add(batchedMesh);

// add geometries once
const boxGeometryId = batchedMesh.addGeometry(boxGeometry);
const sphereGeometryId = batchedMesh.addGeometry(sphereGeometry);
const pyramidGeometryId = batchedMesh.addGeometry(pyramidGeometry);

/* physics objects tracking */

const physicsObjects: PhysicsObject[] = [];

// create scattered kinematic bodies
const shapes: Shape[] = [boxShape, sphereShape, convexHullShape];

for (let i = 0; i < N; i++) {
    // pick a shape (reuse shapes!)
    const shapeIndex = i % shapes.length;
    const shape = shapes[shapeIndex];

    // fibonacci sphere distribution for even spacing
    const goldenRatio = (1 + Math.sqrt(5)) / 2;
    const theta = (2 * Math.PI * i) / goldenRatio;
    const phi = Math.acos(1 - (2 * (i + 0.5)) / N);

    // vary the radius slightly for depth variation
    const radius = 10 + (i % 5) * 5;

    const x = radius * Math.sin(phi) * Math.cos(theta);
    const y = radius * Math.sin(phi) * Math.sin(theta);
    const z = radius * Math.cos(phi);

    const position = vec3.fromValues(x, y, z);

    // random initial rotation
    const rotation = quat.create();
    const randomX = Math.random() * Math.PI * 2;
    const randomY = Math.random() * Math.PI * 2;
    const randomZ = Math.random() * Math.PI * 2;
    quat.fromEuler(rotation, [randomX, randomY, randomZ, 'xyz']);

    // random angular velocity (slow rotation) - applied manually each frame
    const angularVelocity = vec3.fromValues(
        (Math.random() - 0.5) * 0.5,
        (Math.random() - 0.5) * 0.5,
        (Math.random() - 0.5) * 0.5,
    );

    // create static body - we'll transform it directly each frame
    const body = rigidBody.create(world, {
        shape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.STATIC,
        position,
        quaternion: rotation,
        restitution: 0.5,
        friction: 0.3,
    });

    // add instance to batched mesh
    _color.set(colors[i % colors.length]);
    let geometryId: number;

    if (shapeIndex === 0) {
        geometryId = boxGeometryId;
    } else if (shapeIndex === 1) {
        geometryId = sphereGeometryId;
    } else {
        geometryId = pyramidGeometryId;
    }

    const batchedMeshId = batchedMesh.addInstance(geometryId);

    // set initial transform
    _vector3.set(position[0], position[1], position[2]);
    _quaternion.set(rotation[0], rotation[1], rotation[2], rotation[3]);
    _matrix4.compose(_vector3, _quaternion, _scale);
    batchedMesh.setMatrixAt(batchedMeshId, _matrix4);
    batchedMesh.setColorAt(batchedMeshId, _color);

    // radial movement parameters - objects pulse in/out along their radial direction
    const radialDirection = vec3.create();
    vec3.normalize(radialDirection, position);
    const currentRadius = vec3.length(position);
    const minRadius = currentRadius - 3;
    const maxRadius = currentRadius + 3;
    const radialSpeed = 0.2 + Math.random() * 0.3; // vary speeds
    const radialPhase = Math.random() * Math.PI * 2; // random starting phase

    physicsObjects.push({
        batchedMesh,
        batchedMeshId,
        body,
        radialDirection,
        minRadius,
        maxRadius,
        radialSpeed,
        radialPhase,
        angularVelocity,
    });
}

// Dispose temporary helpers
boxHelper.dispose();
sphereHelper.dispose();
pyramidHelper.dispose();

/* raycasting setup */

const rayCollector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();

// Hit point visualization
const hitPointGeometry = new THREE.SphereGeometry(0.3, 16, 16);
const hitPointMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const hitPoint = new THREE.Mesh(hitPointGeometry, hitPointMaterial);
hitPoint.visible = false;
scene.add(hitPoint);

// Mouse tracking
const mouse = new THREE.Vector2();
const raycaster = new THREE.Raycaster();

window.addEventListener('pointermove', (event) => {
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
});

/* helper to update batched mesh from physics body */

function updateBatchedMeshFromBody(physicsObject: PhysicsObject) {
    const body = physicsObject.body;
    _vector3.set(body.position[0], body.position[1], body.position[2]);
    _quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
    _matrix4.compose(_vector3, _quaternion, _scale);
    physicsObject.batchedMesh.setMatrixAt(physicsObject.batchedMeshId, _matrix4);
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

    debugUI.beginPerf(ui);

    // update object positions with radial pulsing and manual rotation
    const time = currentTime / 1000;
    const _tempPosition = vec3.create();
    const _tempQuat = quat.create();
    const _deltaRotation = quat.create();

    for (const physicsObject of physicsObjects) {
        const body = physicsObject.body;

        // calculate radius using sine wave (smoothly oscillates between min and max)
        const t = time * physicsObject.radialSpeed + physicsObject.radialPhase;
        const normalizedRadius = (Math.sin(t) + 1) / 2; // maps sine to 0-1 range
        const radius = physicsObject.minRadius + normalizedRadius * (physicsObject.maxRadius - physicsObject.minRadius);

        // set position along radial direction
        vec3.scale(_tempPosition, physicsObject.radialDirection, radius);

        // apply angular velocity manually: q' = q + 0.5 * dt * w * q
        const av = physicsObject.angularVelocity;
        const len = vec3.length(av);
        if (len > 1e-6) {
            quat.setAxisAngle(_deltaRotation, [av[0] / len, av[1] / len, av[2] / len], len * delta);
            quat.multiply(_tempQuat, _deltaRotation, body.quaternion);
            quat.normalize(_tempQuat, _tempQuat);
        } else {
            quat.copy(_tempQuat, body.quaternion);
        }

        // directly set transform on static body
        rigidBody.setTransform(world, body, _tempPosition, _tempQuat, false);

        // update batched mesh
        updateBatchedMeshFromBody(physicsObject);
    }

    // update physics world
    updateWorld(world, undefined, delta);

    // get ray origin and direction
    raycaster.setFromCamera(mouse, camera);
    const rayOrigin = raycaster.ray.origin;
    const rayDirection = raycaster.ray.direction;

    const origin = vec3.fromValues(rayOrigin.x, rayOrigin.y, rayOrigin.z);
    const direction = vec3.fromValues(rayDirection.x, rayDirection.y, rayDirection.z);

    // perform raycast
    const maxDistance = 100;

    rayCollector.reset();
    castRay(world, rayCollector, raySettings, origin, direction, maxDistance, queryFilter);

    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // update hit point visualization
    if (rayCollector.hit.status === CastRayStatus.COLLIDING) {
        const hitDistance = rayCollector.hit.fraction * maxDistance;
        hitPoint.position.set(
            rayOrigin.x + rayDirection.x * hitDistance,
            rayOrigin.y + rayDirection.y * hitDistance,
            rayOrigin.z + rayDirection.z * hitDistance,
        );
        hitPoint.visible = true;
    } else {
        hitPoint.visible = false;
    }

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    // update orbit controls
    orbitControls.update();

    // render
    renderer.render(scene, camera);
}

animate();
