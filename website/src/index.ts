import type { RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAllShapes,
    rigidBody,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import { quat, vec3 } from 'mathcat';
import * as THREE from 'three/webgpu';

const CAR_SPEED = 40;
const CAR_HALF_EXTENTS = vec3.fromValues(0.9, 0.65, 2.2);
const CAR_X_RANGE = 10; // max lateral offset from center
const CAR_X_LERP = 3; // how fast car moves toward target x
const TILE_DEPTH = 200;
const TILES_AHEAD = 3;
const TILES_BEHIND = 1;
const ROAD_WIDTH = 14;
const GROUND_WIDTH = 400;

/* rendering */

const container = document.getElementById('root')!;

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x000000, 80, 120);
scene.background = new THREE.Color(0x000000);

const STAR_COUNT = 1200;
const STAR_RADIUS = 200;
const starGeometry = new THREE.SphereGeometry(0.15, 4, 4);
const starMaterial = new THREE.MeshBasicNodeMaterial({ color: 0xffffff });
const stars = new THREE.InstancedMesh(starGeometry, starMaterial, STAR_COUNT);
stars.frustumCulled = false;
(starMaterial as any).fog = false;
const _starMatrix = new THREE.Matrix4();
for (let i = 0; i < STAR_COUNT; i++) {
    const theta = Math.random() * Math.PI * 2;
    const elevation = 0.08 + Math.random() * Math.random() * (Math.PI / 2);
    const x = Math.cos(elevation) * Math.cos(theta) * STAR_RADIUS;
    const y = Math.sin(elevation) * STAR_RADIUS;
    const z = Math.cos(elevation) * Math.sin(theta) * STAR_RADIUS;
    const s = 0.3 + Math.random() * 1.2;
    _starMatrix.makeScale(s, s, s);
    _starMatrix.setPosition(x, y, z);
    stars.setMatrixAt(i, _starMatrix);
}
scene.add(stars);

const camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 2000);

const renderer = new THREE.WebGPURenderer({ antialias: true, alpha: true });
await renderer.init();
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

const onResize = () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
};
window.addEventListener('resize', onResize);

const ambientLight = new THREE.AmbientLight("#fff", 1.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight("#fff", 1.5);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 2048;
directionalLight.shadow.mapSize.height = 2048;
directionalLight.shadow.camera.near = 0.5;
directionalLight.shadow.camera.far = 100;
directionalLight.shadow.camera.left = -50;
directionalLight.shadow.camera.right = 50;
directionalLight.shadow.camera.top = 50;
directionalLight.shadow.camera.bottom = -50;
directionalLight.shadow.bias = -0.001;
scene.add(directionalLight);

/* physics world */

registerAllShapes();

const worldSettings = createWorldSettings();
vec3.set(worldSettings.gravity, 0, -20, 0);

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

/* debug renderer */

const debugRendererState = debugRenderer.init();
debugRendererState.options.bodies.wireframe = true;
scene.add(debugRendererState.object3d);

const toggleDebugButton = document.getElementById('debug-toggle')!;
const toggleDebug = () => {
    const isVisible = !debugRendererState.options.bodies.enabled;
    debugRendererState.options.bodies.enabled = isVisible;
    toggleDebugButton.textContent = isVisible ? 'hide debug view [d]' : 'show debug view [d]';
};
toggleDebugButton.addEventListener('click', toggleDebug);
window.addEventListener('keydown', (e) => {
    if (e.key === 'd' || e.key === 'D') toggleDebug();
});

/* car */

const carShape = box.create({ halfExtents: CAR_HALF_EXTENTS });
const carBody = rigidBody.create(world, {
    shape: carShape,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.KINEMATIC,
    position: vec3.fromValues(0, CAR_HALF_EXTENTS[1], 0),
});

const carMesh = new THREE.Mesh(
    new THREE.BoxGeometry(CAR_HALF_EXTENTS[0] * 2, CAR_HALF_EXTENTS[1] * 2, CAR_HALF_EXTENTS[2] * 2),
    new THREE.MeshStandardNodeMaterial({ color: 0xee3333 }),
);
carMesh.castShadow = true;
scene.add(carMesh);

/* pointer input */

let pointerNormX = 0; // -1 to 1
let carX = 0;

window.addEventListener('pointermove', (e) => {
    pointerNormX = (e.clientX / window.innerWidth) * 2 - 1;
});
window.addEventListener('touchmove', (e) => {
    if (e.touches.length > 0) {
        pointerNormX = (e.touches[0].clientX / window.innerWidth) * 2 - 1;
    }
}, { passive: true });

/* tile system */

const DASH_GAP = 10;
const DASHES_PER_TILE = Math.floor(TILE_DEPTH / DASH_GAP);

const MAX_TILES = TILES_AHEAD + TILES_BEHIND + 1; // 5

const OBSTACLE_CUBE_HALF = 0.4;
const OBSTACLE_STACKS_PER_TILE = 3;
const OBSTACLE_CHANCE = 0.6;

/* ── geometries ── */

const groundGeometry = new THREE.BoxGeometry(GROUND_WIDTH, 0.1, TILE_DEPTH);
const roadGeometry = new THREE.BoxGeometry(ROAD_WIDTH, 0.12, TILE_DEPTH);
const dashGeometry = new THREE.BoxGeometry(0.3, 0.02, 3);
const obstacleGeometry = new THREE.BoxGeometry(OBSTACLE_CUBE_HALF * 2, OBSTACLE_CUBE_HALF * 2, OBSTACLE_CUBE_HALF * 2);

/* ── single BatchedMesh for all tile visuals ── */

const MAX_OBSTACLE_INSTANCES = 512;
const MAX_INSTANCES = MAX_TILES * 2 + MAX_TILES * DASHES_PER_TILE + MAX_OBSTACLE_INSTANCES;
const MAX_VERTICES = groundGeometry.getAttribute('position').count
    + roadGeometry.getAttribute('position').count
    + dashGeometry.getAttribute('position').count
    + obstacleGeometry.getAttribute('position').count;
const MAX_INDICES = groundGeometry.index!.count
    + roadGeometry.index!.count
    + dashGeometry.index!.count
    + obstacleGeometry.index!.count;

const tileBatch = new THREE.BatchedMesh(MAX_INSTANCES, MAX_VERTICES, MAX_INDICES, new THREE.MeshStandardNodeMaterial());
tileBatch.castShadow = true;
tileBatch.receiveShadow = true;
tileBatch.sortObjects = false;
tileBatch.perObjectFrustumCulled = false;
tileBatch.frustumCulled = false;
scene.add(tileBatch);

const geoGround = tileBatch.addGeometry(groundGeometry);
const geoRoad = tileBatch.addGeometry(roadGeometry);
const geoDash = tileBatch.addGeometry(dashGeometry);
const geoObstacle = tileBatch.addGeometry(obstacleGeometry);

const GROUND_COLOR = new THREE.Color(0x333333);
const ROAD_COLOR = new THREE.Color(0x222222);
const DASH_COLOR = new THREE.Color(0xaaaaaa);
const OBSTACLE_COLOR = new THREE.Color(0xffffff);

const _m = new THREE.Matrix4();
const _pos = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _s = new THREE.Vector3(1, 1, 1);

function batchAdd(geoId: number, color: THREE.Color, x: number, y: number, z: number): number {
    const id = tileBatch.addInstance(geoId);
    _pos.set(x, y, z);
    _q.identity();
    _s.set(1, 1, 1);
    _m.compose(_pos, _q, _s);
    tileBatch.setMatrixAt(id, _m);
    tileBatch.setColorAt(id, color);
    return id;
}

function batchRemove(id: number): void {
    tileBatch.deleteInstance(id);
}

/* ── physics shapes ── */

const groundShape = box.create({ halfExtents: vec3.fromValues(GROUND_WIDTH / 2, 0.05, TILE_DEPTH / 2) });

const obstacleShape = box.create({ halfExtents: vec3.fromValues(OBSTACLE_CUBE_HALF, OBSTACLE_CUBE_HALF, OBSTACLE_CUBE_HALF) });

type ObstacleEntry = { body: RigidBody; instId: number };

/* ── tile type ── */

type Tile = {
    index: number;
    groundBody: RigidBody;
    groundId: number;
    roadId: number;
    dashIds: number[];
    obstacles: ObstacleEntry[];
};

const activeTiles = new Map<number, Tile>();

function spawnTile(index: number): Tile {
    const z = index * TILE_DEPTH;

    const groundBody = rigidBody.create(world, {
        shape: groundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, -0.05, z),
        friction: 0.5,
    });

    const groundId = batchAdd(geoGround, GROUND_COLOR, 0, -0.05, z);
    const roadId = batchAdd(geoRoad, ROAD_COLOR, 0, 0.01, z);

    // center dashes
    const dashIds: number[] = [];
    const tileStart = z - TILE_DEPTH / 2;
    for (let d = 0; d < DASHES_PER_TILE; d++) {
        const dz = tileStart + d * DASH_GAP + DASH_GAP / 2;
        dashIds.push(batchAdd(geoDash, DASH_COLOR, 0, 0.08, dz));
    }

    // obstacle stacks
    const obstacles: ObstacleEntry[] = [];
    for (let s = 0; s < OBSTACLE_STACKS_PER_TILE; s++) {
        if (Math.random() < OBSTACLE_CHANCE) {
            const halfRoad = ROAD_WIDTH / 2;
            const stackX = (Math.random() - 0.5) * halfRoad * 1.2;
            const stackZ = z + (-0.4 + (s / Math.max(1, OBSTACLE_STACKS_PER_TILE - 1)) * 0.8) * TILE_DEPTH;
            const cols = 2 + Math.floor(Math.random() * 3);
            const rows = 2 + Math.floor(Math.random() * 3);
            const size = OBSTACLE_CUBE_HALF * 2;
            for (let r = 0; r < rows; r++) {
                for (let c = 0; c < cols; c++) {
                    const bx = stackX + (c - (cols - 1) / 2) * size;
                    const by = OBSTACLE_CUBE_HALF + r * size;
                    const body = rigidBody.create(world, {
                        shape: obstacleShape,
                        objectLayer: OBJECT_LAYER_MOVING,
                        motionType: MotionType.DYNAMIC,
                        position: vec3.fromValues(bx, by, stackZ),
                        restitution: 0.1,
                        friction: 0.5,
                    });
                    const instId = batchAdd(geoObstacle, OBSTACLE_COLOR, bx, by, stackZ);
                    obstacles.push({ body, instId });
                }
            }
        }
    }

    return { index, groundBody, groundId, roadId, dashIds, obstacles };
}

function despawnTile(tile: Tile) {
    rigidBody.remove(world, tile.groundBody);
    batchRemove(tile.groundId);
    batchRemove(tile.roadId);
    for (const id of tile.dashIds) {
        batchRemove(id);
    }
    for (const obs of tile.obstacles) {
        rigidBody.remove(world, obs.body);
        batchRemove(obs.instId);
    }
}

function updateTiles(carZ: number) {
    const current = Math.floor(carZ / TILE_DEPTH);
    const lo = current - TILES_AHEAD;
    const hi = current + TILES_BEHIND;

    // despawn first to free pool slots before spawning new tiles
    for (const [index, tile] of activeTiles) {
        if (index < lo || index > hi) {
            despawnTile(tile);
            activeTiles.delete(index);
        }
    }

    for (let i = lo; i <= hi; i++) {
        if (!activeTiles.has(i)) {
            activeTiles.set(i, spawnTile(i));
        }
    }
}

/* simulation */

const _targetPos = vec3.create();
const _targetQuat = quat.create();
const _yAxis = vec3.fromValues(0, 1, 0);

let prevTime = performance.now();
let carZ = 0;

function update() {
    const now = performance.now();
    const dt = Math.min((now - prevTime) / 1000, 1 / 30);
    prevTime = now;

    carZ -= CAR_SPEED * dt;

    const targetX = pointerNormX * CAR_X_RANGE;
    const prevX = carX;
    carX += (targetX - carX) * Math.min(1, CAR_X_LERP * dt);

    // subtle yaw from lateral movement
    const lateralV = (carX - prevX) / dt;
    const yaw = Math.atan2(-lateralV, CAR_SPEED) * 0.5;

    vec3.set(_targetPos, carX, CAR_HALF_EXTENTS[1], carZ);
    quat.setAxisAngle(_targetQuat, _yAxis, yaw);
    rigidBody.moveKinematic(carBody, _targetPos, _targetQuat, dt);

    updateWorld(world, undefined, dt);

    // sync car mesh
    carMesh.position.set(carBody.position[0], carBody.position[1], carBody.position[2]);
    carMesh.quaternion.set(carBody.quaternion[0], carBody.quaternion[1], carBody.quaternion[2], carBody.quaternion[3]);

    // tiles
    updateTiles(carZ);

    // sync obstacle instances from physics
    for (const [, tile] of activeTiles) {
        for (const obs of tile.obstacles) {
            const b = obs.body;
            _q.set(b.quaternion[0], b.quaternion[1], b.quaternion[2], b.quaternion[3]);
            _pos.set(b.position[0], b.position[1], b.position[2]);
            _s.set(1, 1, 1);
            _m.compose(_pos, _q, _s);
            tileBatch.setMatrixAt(obs.instId, _m);
        }
    }

    // camera
    const camX = carBody.position[0] * 0.5;
    const camZ = carZ + 8;
    camera.position.x += (camX - camera.position.x) * Math.min(1, 3 * dt);
    camera.position.y = 5;
    camera.position.z += (camZ - camera.position.z) * Math.min(1, 3 * dt);
    camera.lookAt(carBody.position[0], 1, carZ - 10);

    // keep directional light near the car
    directionalLight.position.set(carBody.position[0] - 5, 10, carZ - 10);
    directionalLight.target.position.set(carBody.position[0], 0, carZ);
    directionalLight.target.updateMatrixWorld();

    stars.position.z = carZ;

    debugRenderer.update(debugRendererState, world);
    renderer.render(scene, camera);
    requestAnimationFrame(update);
}

document.querySelector('#loading')!.remove();
update();
