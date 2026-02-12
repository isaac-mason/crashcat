import type { DistanceConstraint, Listener, RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    ConstraintSpace,
    compound,
    createWorld,
    createWorldSettings,
    cylinder,
    distanceConstraint,
    enableCollision,
    MotionType,
    registerAll,
    rigidBody,
    sphere,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import { quat, vec3 } from 'mathcat';
import { GLTFLoader } from 'three/examples/jsm/Addons.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { cameraPosition, Discard, Fn, float, If, positionWorld, screenCoordinate, vec3 as tslVec3 } from 'three/tsl';
import * as THREE from 'three/webgpu';

/* resources */

const gltfLoader = new GLTFLoader();
const assetsGLTF = await gltfLoader.loadAsync('/assets.glb');

/* physics world */

registerAll();

const worldSettings = createWorldSettings();
vec3.set(worldSettings.gravity, 0, -20, 0);

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

// obstacle box dimensions
const CRATE_HALF_EXTENTS = vec3.fromValues(0.5, 0.5, 0.5);
const CRATE_SIZE = 1.0; // full width/height of a crate

// cone dimensions
const CONE_BASE_SIZE = 0.9; // width/depth of base
const CONE_BASE_HEIGHT = 0.12;
const CONE_BODY_BOTTOM_RADIUS = 0.3; // bottom of truncated cone
const CONE_BODY_TOP_RADIUS = 0.08; // flat top
const CONE_BODY_HEIGHT = 0.9;

// barrel dimensions
const BARREL_RADIUS = 0.45;
const BARREL_HALF_HEIGHT = 0.6;
const ROAD_WIDTH = 14;
const GROUND_WIDTH = 400;

const CAR_SPEED = 40;
const CAR_BOOST_MULTIPLIER = 2.0; // boost speed multiplier
const CAR_HALF_EXTENTS = vec3.fromValues(0.9, 0.65, 2.2);
const CAR_X_RANGE = ROAD_WIDTH / 2 - CAR_HALF_EXTENTS[0]; // stay on road
const CAR_X_LERP = 3;

// camera constants
const CAMERA_FOV_NORMAL = 60;
const CAMERA_FOV_BOOST = 75;
const CAMERA_FOV_LERP = 8;

// boost constants
const CAR_SPEED_LERP = 2.5; // how fast speed transitions

// jump animation constants
const JUMP_MIN_DURATION = 0.35; // seconds (for min height jump)
const JUMP_MAX_DURATION = 0.65; // seconds (for max height jump)
const JUMP_MIN_HEIGHT = 0.5; // units (quick tap)
const JUMP_MAX_HEIGHT = 2.5; // units (full charge)
const JUMP_MAX_CHARGE_TIME = 0.3; // seconds to reach max height
const JUMP_COOLDOWN = 0; // seconds
const JUMP_CHARGE_TILT = 0.05; // radians

// suspension constants
const SUSPENSION_SPRING_K = 12;
const SUSPENSION_DAMPING = 0.65;
const SUSPENSION_COMPRESS_START = -0.15;
const SUSPENSION_EXTEND_LAUNCH = 0.1;
const SUSPENSION_COMPRESS_LAND = -0.25;

const TILE_DEPTH = 200;
const TILES_AHEAD = 3;
const TILES_BEHIND = 2;
const OBSTACLE_BEHIND_CAMERA_DESPAWN_DIST = 20;

/* utility functions */

// exponential decay for hit reactions
function calculateDecay(strength: number, decayRate: number, elapsed: number): number {
    return strength * Math.exp(-decayRate * elapsed);
}

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
    // heavily bias toward the horizon — cube the random to cluster near 0
    const r = Math.random();
    const elevation = 0.01 + r * r * r * (Math.PI / 2.2);
    const x = Math.cos(elevation) * Math.cos(theta) * STAR_RADIUS;
    const y = Math.sin(elevation) * STAR_RADIUS;
    const z = Math.cos(elevation) * Math.sin(theta) * STAR_RADIUS;
    // normalized elevation: 0 at horizon, 1 at zenith
    const t = elevation / (Math.PI / 2);
    // small at horizon, larger upwards
    const s = 0.3 + t * 2.5;
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

const ambientLight = new THREE.AmbientLight('#fff', 1.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight('#fff', 1.5);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 1024;
directionalLight.shadow.mapSize.height = 1024;
directionalLight.shadow.camera.near = 1;
directionalLight.shadow.camera.far = 200;
directionalLight.shadow.camera.left = -20;
directionalLight.shadow.camera.right = 12;
directionalLight.shadow.camera.top = 10;
directionalLight.shadow.camera.bottom = -130;
directionalLight.shadow.bias = -0.002;

scene.add(directionalLight);

/* debug renderer */

const debugRendererState = debugRenderer.init();
debugRendererState.options.bodies.enabled = false;
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

const carObject = assetsGLTF.scene.getObjectByName('car_group') as THREE.Object3D;

scene.add(carObject);

carObject.traverse((child) => {
    if (child instanceof THREE.Mesh) {
        child.castShadow = true;
        child.receiveShadow = true;
    }
});

const catObject = carObject.getObjectByName('cat')!;
const catBasePosition = catObject.position.clone();
const catBaseRotation = catObject.rotation.clone();

// cat sway from turning — tilt head away from movement
const CAT_SWAY_ROLL = 2.25; // max roll in radians when turning hard
const CAT_SWAY_YAW = 1.0; // head turns away from movement direction
const CAT_SWAY_PITCH = 0.5; // forward pitch when accelerating laterally
const CAT_SWAY_LERP = 5; // how fast the sway catches up
let catSwayRoll = 0;
let catSwayYaw = 0;
let catSwayPitch = 0;

// cat hit reaction — separate from chassis, slower wobble, more exaggerated
let catHitTime = -Infinity;
let catHitStrength = 0;
let catHitNormalX = 0;
let catHitNormalZ = 0;
const CAT_HIT_DECAY = 3; // slower decay than chassis
const CAT_HIT_FREQ = 14; // lower frequency wobble
const CAT_HIT_Y_AMPLITUDE = 0.18; // vertical bounce
const CAT_HIT_ROLL_AMPLITUDE = 1.05; // exaggerated roll
const CAT_HIT_PITCH_AMPLITUDE = 0.6; // exaggerated pitch
const CAT_MAX_ROT = Math.PI / 9; // 20 deg clamp

// jump state
type JumpState = 'grounded' | 'jumping';
let jumpState: JumpState = 'grounded';
let jumpTime = 0;
let jumpCooldownTimer = 0;
let landingTime = 0; // track when we landed
let isChargingJump = false;
let jumpChargeStartTime = 0;
let currentJumpHeight = JUMP_MIN_HEIGHT;
let currentJumpDuration = JUMP_MIN_DURATION;

// boost state
let isBoosting = false;
let currentSpeed = CAR_SPEED;
let targetSpeed = CAR_SPEED;
let currentFov = CAMERA_FOV_NORMAL;

// suspension state
let suspensionOffset = 0;
let suspensionVelocity = 0;
let targetSuspensionOffset = 0;

/* pointer input */

let pointerNormX = 0; // -1 to 1
let carX = 0;

window.addEventListener('pointermove', (e) => {
    pointerNormX = (e.clientX / window.innerWidth) * 2 - 1;
});

window.addEventListener(
    'touchmove',
    (e) => {
        if (e.touches.length > 0) {
            pointerNormX = (e.touches[0].clientX / window.innerWidth) * 2 - 1;
        }
    },
    { passive: true },
);

/* jump input */

function calculateJumpHeight(chargeDuration: number): number {
    const chargeT = Math.min(chargeDuration / JUMP_MAX_CHARGE_TIME, 1);
    // ease-out curve for diminishing returns at high charge
    const eased = 1 - (1 - chargeT) ** 2;
    return JUMP_MIN_HEIGHT + (JUMP_MAX_HEIGHT - JUMP_MIN_HEIGHT) * eased;
}

window.addEventListener('keydown', (e) => {
    if ((e.key === ' ' || e.key === 'Spacebar') && jumpState === 'grounded' && jumpCooldownTimer <= 0 && !isChargingJump) {
        isChargingJump = true;
        jumpChargeStartTime = performance.now() / 1000;
        e.preventDefault();
    }
    if (e.key === 'Shift') {
        isBoosting = true;
        e.preventDefault();
    }
});

window.addEventListener('keyup', (e) => {
    if ((e.key === ' ' || e.key === 'Spacebar') && isChargingJump) {
        const now = performance.now() / 1000;
        const chargeDuration = now - jumpChargeStartTime;
        currentJumpHeight = calculateJumpHeight(chargeDuration);
        // scale duration with sqrt of height for realistic physics
        const heightT = (currentJumpHeight - JUMP_MIN_HEIGHT) / (JUMP_MAX_HEIGHT - JUMP_MIN_HEIGHT);
        currentJumpDuration = JUMP_MIN_DURATION + (JUMP_MAX_DURATION - JUMP_MIN_DURATION) * Math.sqrt(heightT);
        jumpState = 'jumping';
        jumpTime = 0;
        isChargingJump = false;
        e.preventDefault();
    }
    if (e.key === 'Shift') {
        isBoosting = false;
        e.preventDefault();
    }
});

/* tile system */

const DASH_GAP = 10;
const DASHES_PER_TILE = Math.floor(TILE_DEPTH / DASH_GAP);

const CRATE_STACKS_PER_TILE = 4;
const CONE_COLUMNS_PER_TILE = 3;
const BARREL_STACKS_PER_TILE = 3;
const WRECKING_BALLS_PER_TILE = 1;

// wrecking ball dimensions
const WRECKING_BALL_RADIUS = 2;
const ROPE_SEGMENT_LENGTH = 1.5;
const ROPE_SEGMENT_RADIUS = 0.08;
const ROPE_SEGMENTS = 6; // more segments = longer rope = ball hangs higher at rest
const WRECKING_BALL_HEIGHT = 14; // anchor height above ground
const WRECKING_BALL_START_ANGLE = Math.PI / 3; // start swinging from ~60 degrees

/* geometries */

const groundGeometry = new THREE.BoxGeometry(GROUND_WIDTH, 0.1, TILE_DEPTH);
const roadGeometry = new THREE.BoxGeometry(ROAD_WIDTH, 0.12, TILE_DEPTH);
const dashGeometry = new THREE.BoxGeometry(0.3, 0.02, 3);
const crateGeometry = new THREE.BoxGeometry(CRATE_SIZE, CRATE_SIZE, CRATE_SIZE);

// cone merged geometry: base box + truncated cone body (flat top)
const coneBaseGeo = new THREE.BoxGeometry(CONE_BASE_SIZE, CONE_BASE_HEIGHT, CONE_BASE_SIZE);
coneBaseGeo.translate(0, CONE_BASE_HEIGHT / 2, 0);
const coneBodyGeo = new THREE.CylinderGeometry(CONE_BODY_TOP_RADIUS, CONE_BODY_BOTTOM_RADIUS, CONE_BODY_HEIGHT, 8);
coneBodyGeo.translate(0, CONE_BASE_HEIGHT + CONE_BODY_HEIGHT / 2, 0);
const coneGeometry = mergeGeometries([coneBaseGeo, coneBodyGeo])!;

// barrel geometry
const barrelGeometry = new THREE.CylinderGeometry(BARREL_RADIUS, BARREL_RADIUS, BARREL_HALF_HEIGHT * 2, 12);

// wrecking ball geometries
const wreckingBallGeometry = new THREE.SphereGeometry(WRECKING_BALL_RADIUS, 16, 12);
const ropeSegmentGeometry = new THREE.CylinderGeometry(ROPE_SEGMENT_RADIUS, ROPE_SEGMENT_RADIUS, ROPE_SEGMENT_LENGTH, 6);

/* tile visuals batched mesh */

const MAX_INSTANCES = 100_000;
const MAX_VERTICES = 100_000;
const MAX_INDICES = 100_000;

// white phong material for obstacles with dithering based on camera proximity
const obstacleMaterial = new THREE.MeshPhongNodeMaterial({ color: '#ccc', shininess: 10 });

// dithering thresholds
const ditherStartDistance = 3;
const ditherEndDistance = 0.2;

obstacleMaterial.colorNode = Fn(() => {
    // calculate distance from world position to camera
    const worldPos = positionWorld;
    const camPos = cameraPosition;
    const dx = worldPos.x.sub(camPos.x);
    const dy = worldPos.y.sub(camPos.y);
    const dz = worldPos.z.sub(camPos.z);
    const distance = dx.mul(dx).add(dy.mul(dy)).add(dz.mul(dz)).sqrt();

    // calculate dither amount (0 = fully visible, 1 = fully transparent)
    const ditherAmount = distance
        .sub(ditherEndDistance)
        .div(ditherStartDistance - ditherEndDistance)
        .oneMinus()
        .clamp();

    If(ditherAmount.greaterThan(0.0), () => {
        // 4x4 bayer matrix for dithering pattern
        const screen = screenCoordinate.xy;
        const x = screen.x.mod(4.0).floor();
        const y = screen.y.mod(4.0).floor();

        // bayer matrix thresholds (0-15) / 16
        const bayerIndex = y.mul(4.0).add(x).floor();

        // bayer 4x4 matrix values
        const bayerMatrix = float(0.0).toVar();
        If(bayerIndex.equal(0.0), () => bayerMatrix.assign(0.0 / 16.0));
        If(bayerIndex.equal(1.0), () => bayerMatrix.assign(8.0 / 16.0));
        If(bayerIndex.equal(2.0), () => bayerMatrix.assign(2.0 / 16.0));
        If(bayerIndex.equal(3.0), () => bayerMatrix.assign(10.0 / 16.0));
        If(bayerIndex.equal(4.0), () => bayerMatrix.assign(12.0 / 16.0));
        If(bayerIndex.equal(5.0), () => bayerMatrix.assign(4.0 / 16.0));
        If(bayerIndex.equal(6.0), () => bayerMatrix.assign(14.0 / 16.0));
        If(bayerIndex.equal(7.0), () => bayerMatrix.assign(6.0 / 16.0));
        If(bayerIndex.equal(8.0), () => bayerMatrix.assign(3.0 / 16.0));
        If(bayerIndex.equal(9.0), () => bayerMatrix.assign(11.0 / 16.0));
        If(bayerIndex.equal(10.0), () => bayerMatrix.assign(1.0 / 16.0));
        If(bayerIndex.equal(11.0), () => bayerMatrix.assign(9.0 / 16.0));
        If(bayerIndex.equal(12.0), () => bayerMatrix.assign(15.0 / 16.0));
        If(bayerIndex.equal(13.0), () => bayerMatrix.assign(7.0 / 16.0));
        If(bayerIndex.equal(14.0), () => bayerMatrix.assign(13.0 / 16.0));
        If(bayerIndex.equal(15.0), () => bayerMatrix.assign(5.0 / 16.0));

        If(ditherAmount.greaterThan(bayerMatrix), () => {
            Discard();
        });
    });

    // return base color
    return tslVec3(0.8, 0.8, 0.8);
})();

obstacleMaterial.needsUpdate = true;

const tileBatch = new THREE.BatchedMesh(MAX_INSTANCES, MAX_VERTICES, MAX_INDICES, new THREE.MeshStandardNodeMaterial());
tileBatch.castShadow = true;
tileBatch.receiveShadow = true;
tileBatch.sortObjects = false;
tileBatch.perObjectFrustumCulled = false;
tileBatch.frustumCulled = false;
scene.add(tileBatch);

// separate batched mesh for obstacles (crates) with white phong material
const obstacleBatch = new THREE.BatchedMesh(MAX_INSTANCES, MAX_VERTICES, MAX_INDICES, obstacleMaterial);
obstacleBatch.castShadow = true;
obstacleBatch.receiveShadow = true;
obstacleBatch.sortObjects = false;
obstacleBatch.perObjectFrustumCulled = false;
obstacleBatch.frustumCulled = false;
scene.add(obstacleBatch);

const groundGeometryId = tileBatch.addGeometry(groundGeometry);
const roadGeometryId = tileBatch.addGeometry(roadGeometry);
const dashGeometryId = tileBatch.addGeometry(dashGeometry);

const crateGeometryId = obstacleBatch.addGeometry(crateGeometry);
const coneGeometryId = obstacleBatch.addGeometry(coneGeometry);
const barrelGeometryId = obstacleBatch.addGeometry(barrelGeometry);
const wreckingBallGeometryId = obstacleBatch.addGeometry(wreckingBallGeometry);
const ropeSegmentGeometryId = obstacleBatch.addGeometry(ropeSegmentGeometry);

const GROUND_COLOR = new THREE.Color(0x333333);
const ROAD_COLOR = new THREE.Color(0x222222);
const DASH_COLOR = new THREE.Color(0xaaaaaa);

const OBSTACLE_COLOR = new THREE.Color(0xffffff);

const _batch_matrix = new THREE.Matrix4();
const _batch_pos = new THREE.Vector3();
const _batch_quat = new THREE.Quaternion();
const _batch_scale = new THREE.Vector3(1, 1, 1);

function batchAdd(geoId: number, color: THREE.Color, x: number, y: number, z: number): number {
    const id = tileBatch.addInstance(geoId);
    _batch_pos.set(x, y, z);
    _batch_quat.identity();
    _batch_scale.set(1, 1, 1);
    _batch_matrix.compose(_batch_pos, _batch_quat, _batch_scale);
    tileBatch.setMatrixAt(id, _batch_matrix);
    tileBatch.setColorAt(id, color);
    return id;
}

function batchRemove(id: number): void {
    tileBatch.deleteInstance(id);
}

function obstacleBatchAdd(geoId: number, x: number, y: number, z: number): number {
    const id = obstacleBatch.addInstance(geoId);
    _batch_pos.set(x, y, z);
    _batch_quat.identity();
    _batch_scale.set(1, 1, 1);
    _batch_matrix.compose(_batch_pos, _batch_quat, _batch_scale);
    obstacleBatch.setMatrixAt(id, _batch_matrix);
    obstacleBatch.setColorAt(id, OBSTACLE_COLOR);
    return id;
}

function obstacleBatchRemove(id: number): void {
    obstacleBatch.deleteInstance(id);
}

function obstacleBatchUpdate(id: number, x: number, y: number, z: number, qx: number, qy: number, qz: number, qw: number): void {
    _batch_pos.set(x, y, z);
    _batch_quat.set(qx, qy, qz, qw);
    _batch_scale.set(1, 1, 1);
    _batch_matrix.compose(_batch_pos, _batch_quat, _batch_scale);
    obstacleBatch.setMatrixAt(id, _batch_matrix);
}

const crateShape = box.create({ halfExtents: CRATE_HALF_EXTENTS });

// cone compound shape: thin base box + taller box for cone body
const coneShape = compound.create({
    children: [
        // base
        {
            position: vec3.fromValues(0, CONE_BASE_HEIGHT / 2, 0),
            quaternion: quat.identity(quat.create()),
            shape: box.create({ halfExtents: vec3.fromValues(CONE_BASE_SIZE / 2, CONE_BASE_HEIGHT / 2, CONE_BASE_SIZE / 2) }),
        },
        // cone body approximated as box (use average of top/bottom radius)
        {
            position: vec3.fromValues(0, CONE_BASE_HEIGHT + CONE_BODY_HEIGHT / 2, 0),
            quaternion: quat.identity(quat.create()),
            shape: box.create({
                halfExtents: vec3.fromValues(CONE_BODY_BOTTOM_RADIUS, CONE_BODY_HEIGHT / 2, CONE_BODY_BOTTOM_RADIUS),
            }),
        },
    ],
});

const barrelShape = cylinder.create({ halfHeight: BARREL_HALF_HEIGHT, radius: BARREL_RADIUS });

const groundShape = box.create({ halfExtents: vec3.fromValues(GROUND_WIDTH / 2, 0.05, TILE_DEPTH / 2) });

// wrecking ball shapes
const wreckingBallShape = sphere.create({ radius: WRECKING_BALL_RADIUS });
const ropeSegmentShape = box.create({
    halfExtents: vec3.fromValues(ROPE_SEGMENT_RADIUS, ROPE_SEGMENT_LENGTH / 2, ROPE_SEGMENT_RADIUS),
});

type ObstacleEntry = { body: RigidBody; instanceId: number };

// wrecking ball entry - includes anchor, rope segments, ball, and constraints
type WreckingBallEntry = {
    anchorBody: RigidBody;
    ropeSegments: { body: RigidBody; instanceId: number }[];
    ballBody: RigidBody;
    ballInstanceId: number;
    constraints: DistanceConstraint[];
};

type Tile = {
    index: number;
    groundBody: RigidBody;
    groundId: number;
    roadId: number;
    dashIds: number[];
    obstacles: ObstacleEntry[];
    wreckingBalls: WreckingBallEntry[];
};

const activeTiles = new Map<number, Tile>();

function removeObstacle(obs: ObstacleEntry) {
    rigidBody.remove(world, obs.body);
    obstacleBatchRemove(obs.instanceId);
}

function removeWreckingBall(wb: WreckingBallEntry) {
    for (const c of wb.constraints) {
        distanceConstraint.remove(world, c);
    }
    rigidBody.remove(world, wb.anchorBody);
    for (const seg of wb.ropeSegments) {
        rigidBody.remove(world, seg.body);
        obstacleBatchRemove(seg.instanceId);
    }
    rigidBody.remove(world, wb.ballBody);
    obstacleBatchRemove(wb.ballInstanceId);
}

function spawnTile(index: number): Tile {
    const z = index * TILE_DEPTH;

    const groundBody = rigidBody.create(world, {
        shape: groundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: [0, -0.05, z],
        friction: 0.8,
    });

    const groundId = batchAdd(groundGeometryId, GROUND_COLOR, 0, -0.1, z);
    const roadId = batchAdd(roadGeometryId, ROAD_COLOR, 0, -0.06, z);

    // center dashes
    const dashIds: number[] = [];
    const tileStart = z - TILE_DEPTH / 2;
    for (let d = 0; d < DASHES_PER_TILE; d++) {
        const dz = tileStart + d * DASH_GAP + DASH_GAP / 2;
        dashIds.push(batchAdd(dashGeometryId, DASH_COLOR, 0, 0.08, dz));
    }

    // obstacles using batched mesh
    const obstacles: ObstacleEntry[] = [];

    // symmetrical pyramid crate stacks - centered, wider at bottom
    const CRATE_GAP = 0.05;
    const CRATE_SPACING = CRATE_SIZE + CRATE_GAP;
    for (let s = 0; s < CRATE_STACKS_PER_TILE; s++) {
        const halfRoad = ROAD_WIDTH / 2;
        const stackX = (Math.random() - 0.5) * halfRoad * 1.6;
        const stackZ = z + (-0.45 + (s / Math.max(1, CRATE_STACKS_PER_TILE - 1)) * 0.9) * TILE_DEPTH;
        const baseWidth = 3 + Math.floor(Math.random() * 3); // 3-5 crates at base
        const rows = Math.min(baseWidth, 2 + Math.floor(Math.random() * 3)); // cap rows to base width
        for (let r = 0; r < rows; r++) {
            const colsInRow = Math.max(1, baseWidth - r); // pyramid: fewer crates each row
            for (let c = 0; c < colsInRow; c++) {
                // centered - no offset, just center each row
                const bx = stackX + (c - (colsInRow - 1) / 2) * CRATE_SPACING;
                const by = CRATE_HALF_EXTENTS[1] + r * CRATE_SPACING;
                const body = rigidBody.create(world, {
                    shape: crateShape,
                    objectLayer: OBJECT_LAYER_MOVING,
                    motionType: MotionType.DYNAMIC,
                    position: [bx, by, stackZ],
                    restitution: 0.3,
                    friction: 0.5,
                    mass: 1,
                });
                const instanceId = obstacleBatchAdd(crateGeometryId, bx, by, stackZ);
                obstacles.push({ body, instanceId });
            }
        }
    }

    // cone columns
    for (let col = 0; col < CONE_COLUMNS_PER_TILE; col++) {
        const halfRoad = ROAD_WIDTH / 2;
        const colX = (Math.random() - 0.5) * halfRoad * 1.5;
        const colZ = z + (-0.4 + (col / Math.max(1, CONE_COLUMNS_PER_TILE - 1)) * 0.8) * TILE_DEPTH;
        const coneCount = 3 + Math.floor(Math.random() * 4); // 3-6 cones per column
        const spacing = CONE_BASE_SIZE * 2.5;
        for (let i = 0; i < coneCount; i++) {
            const cz = colZ + (i - (coneCount - 1) / 2) * spacing;
            // cone shape center of mass
            const shapeCOM = coneShape.centerOfMass;
            const by = shapeCOM[1];
            const body = rigidBody.create(world, {
                shape: coneShape,
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [colX, by, cz],
                restitution: 0.3,
                friction: 0.6,
                mass: 0.5,
            });
            // visual: merged cone geometry, offset so origin aligns with physics COM
            const instanceId = obstacleBatchAdd(coneGeometryId, colX, by - shapeCOM[1], cz);
            obstacles.push({ body, instanceId });
        }
    }

    // barrel stacks - pyramid style like crates
    const BARREL_GAP = 0.05;
    const BARREL_SPACING_H = BARREL_RADIUS * 2 + BARREL_GAP;
    const BARREL_SPACING_V = BARREL_HALF_HEIGHT * 2 + BARREL_GAP;
    for (let s = 0; s < BARREL_STACKS_PER_TILE; s++) {
        const halfRoad = ROAD_WIDTH / 2;
        const stackX = (Math.random() - 0.5) * halfRoad * 1.6;
        const stackZ = z + (-0.35 + (s / Math.max(1, BARREL_STACKS_PER_TILE - 1)) * 0.7) * TILE_DEPTH;
        const baseWidth = 2 + Math.floor(Math.random() * 3); // 2-4 barrels at base
        const rows = Math.min(baseWidth, 2 + Math.floor(Math.random() * 2)); // 2-3 rows
        for (let r = 0; r < rows; r++) {
            const colsInRow = Math.max(1, baseWidth - r);
            for (let c = 0; c < colsInRow; c++) {
                const bx = stackX + (c - (colsInRow - 1) / 2) * BARREL_SPACING_H;
                const by = BARREL_HALF_HEIGHT + r * BARREL_SPACING_V;
                const body = rigidBody.create(world, {
                    shape: barrelShape,
                    objectLayer: OBJECT_LAYER_MOVING,
                    motionType: MotionType.DYNAMIC,
                    position: [bx, by, stackZ],
                    restitution: 0.3,
                    friction: 0.5,
                    mass: 2,
                });
                const instanceId = obstacleBatchAdd(barrelGeometryId, bx, by, stackZ);
                obstacles.push({ body, instanceId });
            }
        }
    }

    // wrecking balls
    const wreckingBalls: WreckingBallEntry[] = [];
    for (let w = 0; w < WRECKING_BALLS_PER_TILE; w++) {
        const ballZ = z + (Math.random() - 0.5) * TILE_DEPTH * 0.6;
        // pick a side (-1 or 1) for swing direction, anchor in center
        const side = Math.random() < 0.5 ? -1 : 1;
        const anchorX = 0; // center of road
        const anchorY = WRECKING_BALL_HEIGHT;

        // create static anchor
        const anchorBody = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.2, 0.2, 0.2) }),
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: [anchorX, anchorY, ballZ],
        });

        const constraints: DistanceConstraint[] = [];
        const ropeSegments: { body: RigidBody; instanceId: number }[] = [];

        // calculate starting position for pendulum swing
        // swing starts to the side (away from center of road)
        const totalRopeLength = ROPE_SEGMENTS * ROPE_SEGMENT_LENGTH;
        const swingOffsetX = Math.sin(WRECKING_BALL_START_ANGLE) * totalRopeLength * side;
        const swingOffsetY = -Math.cos(WRECKING_BALL_START_ANGLE) * totalRopeLength;

        let prevBody = anchorBody;

        // create rope segments
        for (let seg = 0; seg < ROPE_SEGMENTS; seg++) {
            const t = (seg + 0.5) / ROPE_SEGMENTS;
            const segX = anchorX + swingOffsetX * t;
            const segY = anchorY + swingOffsetY * t;

            const segBody = rigidBody.create(world, {
                shape: ropeSegmentShape,
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [segX, segY, ballZ],
                friction: 0.3,
            });

            const instanceId = obstacleBatchAdd(ropeSegmentGeometryId, segX, segY, ballZ);
            ropeSegments.push({ body: segBody, instanceId });

            // connect to previous body with distance constraint using LOCAL space
            const constraint = distanceConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: segBody.id,
                // pointA: bottom of previous body (local coords)
                pointA:
                    seg === 0
                        ? vec3.fromValues(0, -0.2, 0) // bottom of anchor
                        : vec3.fromValues(0, -ROPE_SEGMENT_LENGTH / 2, 0), // bottom of prev segment
                // pointB: top of this segment (local coords)
                pointB: vec3.fromValues(0, ROPE_SEGMENT_LENGTH / 2, 0),
                minDistance: 0,
                maxDistance: 0.05,
                space: ConstraintSpace.LOCAL,
            });
            constraints.push(constraint);

            prevBody = segBody;
        }

        // create wrecking ball at end
        const ballX = anchorX + swingOffsetX;
        const ballY = anchorY + swingOffsetY - WRECKING_BALL_RADIUS;

        const ballBody = rigidBody.create(world, {
            shape: wreckingBallShape,
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [ballX, ballY, ballZ],
            mass: 40,
            friction: 0.5,
            restitution: 0.3,
        });

        const ballInstanceId = obstacleBatchAdd(wreckingBallGeometryId, ballX, ballY, ballZ);

        // connect last rope segment to ball using LOCAL space
        const ballConstraint = distanceConstraint.create(world, {
            bodyIdA: prevBody.id,
            bodyIdB: ballBody.id,
            // bottom of last segment
            pointA: vec3.fromValues(0, -ROPE_SEGMENT_LENGTH / 2, 0),
            // top of ball
            pointB: vec3.fromValues(0, WRECKING_BALL_RADIUS, 0),
            minDistance: 0,
            maxDistance: 0.05,
            space: ConstraintSpace.LOCAL,
        });
        constraints.push(ballConstraint);

        wreckingBalls.push({
            anchorBody,
            ropeSegments,
            ballBody,
            ballInstanceId,
            constraints,
        });

        // give the ball an initial impulse to start swinging in a random direction
        const impulseMagnitude = 100 + Math.random() * 50;
        const impulseAngle = Math.random() * Math.PI * 2;
        const impulseX = Math.cos(impulseAngle) * impulseMagnitude;
        const impulseZ = Math.sin(impulseAngle) * impulseMagnitude;
        rigidBody.addImpulse(world, ballBody, vec3.fromValues(impulseX, 0, impulseZ));
    }

    return {
        index,
        groundBody,
        groundId,
        roadId,
        dashIds,
        obstacles,
        wreckingBalls,
    };
}

function despawnTile(tile: Tile) {
    rigidBody.remove(world, tile.groundBody);
    batchRemove(tile.groundId);
    batchRemove(tile.roadId);
    for (const id of tile.dashIds) {
        batchRemove(id);
    }
    for (const obs of tile.obstacles) {
        removeObstacle(obs);
    }
    for (const wb of tile.wreckingBalls) {
        removeWreckingBall(wb);
    }
}

// eagerly remove individual obstacles that are behind camera (with shadow buffer)
function cleanupObstaclesBehindCamera(tile: Tile, cameraZ: number) {
    const cleanupThreshold = cameraZ + OBSTACLE_BEHIND_CAMERA_DESPAWN_DIST;
    for (let i = tile.obstacles.length - 1; i >= 0; i--) {
        const obs = tile.obstacles[i];
        if (obs.body.position[2] > cleanupThreshold) {
            removeObstacle(obs);
            tile.obstacles.splice(i, 1);
        }
    }

    for (let i = tile.wreckingBalls.length - 1; i >= 0; i--) {
        const wb = tile.wreckingBalls[i];
        if (wb.ballBody.position[2] > cleanupThreshold) {
            removeWreckingBall(wb);
            tile.wreckingBalls.splice(i, 1);
        }
    }
}

function updateTiles(carZ: number) {
    const current = Math.floor(carZ / TILE_DEPTH);
    const lo = current - TILES_AHEAD;
    const hi = current + TILES_BEHIND;

    for (const [index, tile] of activeTiles) {
        if (index < lo || index > hi) {
            despawnTile(tile);
            activeTiles.delete(index);
        } else {
            cleanupObstaclesBehindCamera(tile, camera.position.z);
        }
    }

    for (let i = lo; i <= hi; i++) {
        if (!activeTiles.has(i)) {
            activeTiles.set(i, spawnTile(i));
        }
    }
}

/* simulation */

// scratch variables for updateCarMovement
const _updateCarMovement_targetPos = vec3.create();
const _updateCarMovement_yAxis = vec3.fromValues(0, 1, 0);
const _updateCarMovement_xAxis = vec3.fromValues(1, 0, 0);
const _updateCarMovement_rotQuat = quat.create();
const _updateCarMovement_pitchQuat = quat.create();

let prevTime = performance.now();
let carZ = 0;

// chassis hit reaction state
let hitTime = -Infinity;
let hitStrength = 0;
let hitNormalX = 0;
let hitNormalZ = 0;
const HIT_DECAY = 5; // how fast the shake fades (higher = faster)
const HIT_FREQ = 27; // oscillation frequency
const HIT_Y_AMPLITUDE = 0.1; // vertical bounce amplitude
const HIT_ROLL_AMPLITUDE = 0.17; // roll amplitude in radians
const HIT_PITCH_AMPLITUDE = 0.08; // pitch amplitude in radians

const listener: Listener = {
    onContactAdded: (bodyA, bodyB, manifold) => {
        const isCarA = bodyA === carBody;
        const isCarB = bodyB === carBody;
        if (!isCarA && !isCarB) return;

        // skip ground contacts — only react to obstacle hits
        const other = isCarA ? bodyB : bodyA;
        if (other.motionType === MotionType.STATIC) return;

        // estimate impact intensity from penetration depth
        const depth = Math.abs(manifold.penetrationDepth);
        const strength = Math.min(depth * 8, 1);

        // only override if this hit is stronger than the current decaying one
        const now = performance.now() / 1000;
        const remaining = calculateDecay(hitStrength, HIT_DECAY, now - hitTime);
        const sign = isCarA ? 1 : -1;
        if (strength > remaining) {
            hitTime = now;
            hitStrength = strength;
            // normal points A -> B; flip if car is B so it always points away from car
            hitNormalX = manifold.worldSpaceNormal[0] * sign;
            hitNormalZ = -manifold.worldSpaceNormal[2] * sign;
        }

        // cat has its own hit state — more dramatic, slower decay
        const catRemaining = calculateDecay(catHitStrength, CAT_HIT_DECAY, now - catHitTime);
        if (strength > catRemaining) {
            catHitTime = now;
            catHitStrength = strength;
            catHitNormalX = manifold.worldSpaceNormal[0] * sign;
            catHitNormalZ = -manifold.worldSpaceNormal[2] * sign;
        }
    },
};

function updateCarMovement(dt: number) {
    // only update target speed when grounded (maintain speed in air)
    if (jumpState === 'grounded') {
        targetSpeed = isBoosting ? CAR_SPEED * CAR_BOOST_MULTIPLIER : CAR_SPEED;
    }
    
    currentSpeed += (targetSpeed - currentSpeed) * Math.min(1, CAR_SPEED_LERP * dt);
    
    // map current speed to FOV (lerp based on actual speed, not button state)
    const speedT = (currentSpeed - CAR_SPEED) / (CAR_SPEED * CAR_BOOST_MULTIPLIER - CAR_SPEED);
    const targetFov = CAMERA_FOV_NORMAL + (CAMERA_FOV_BOOST - CAMERA_FOV_NORMAL) * Math.max(0, Math.min(1, speedT));
    currentFov += (targetFov - currentFov) * Math.min(1, CAMERA_FOV_LERP * dt);
    camera.fov = currentFov;
    camera.updateProjectionMatrix();

    carZ -= currentSpeed * dt;

    const targetX = pointerNormX * CAR_X_RANGE;
    const prevX = carX;
    carX += (targetX - carX) * Math.min(1, CAR_X_LERP * dt);

    // update jump cooldown
    if (jumpCooldownTimer > 0) {
        jumpCooldownTimer -= dt;
    }

    // handle jump charging
    if (isChargingJump) {
        const now = performance.now() / 1000;
        const chargeDuration = now - jumpChargeStartTime;
        const chargeT = Math.min(chargeDuration / JUMP_MAX_CHARGE_TIME, 1);

        // progressive suspension compression during charge
        targetSuspensionOffset = SUSPENSION_COMPRESS_START * (0.5 + 0.5 * chargeT);
    }

    // update jump state
    const baseCarY = CAR_HALF_EXTENTS[1];
    let carY: number;
    let pitchRotation = 0;

    if (jumpState === 'jumping') {
        jumpTime += dt;

        if (jumpTime >= currentJumpDuration) {
            // landing
            jumpState = 'grounded';
            landingTime = performance.now() / 1000;
            jumpTime = 0;
            jumpCooldownTimer = JUMP_COOLDOWN;
            targetSuspensionOffset = SUSPENSION_COMPRESS_LAND;
            carY = baseCarY;
        } else {
            // animate jump
            const t = Math.min(jumpTime / currentJumpDuration, 1.0);
            const heightProgress = 4 * t * (1 - t);
            carY = baseCarY + currentJumpHeight * heightProgress;

            // suspension animation during jump
            if (t < 0.05) {
                targetSuspensionOffset = SUSPENSION_COMPRESS_START;
            } else if (t < 0.15) {
                targetSuspensionOffset = SUSPENSION_EXTEND_LAUNCH;
            } else if (t < 0.7) {
                targetSuspensionOffset = 0;
            } else {
                targetSuspensionOffset = -0.05;
            }
        }
    } else {
        // grounded
        carY = baseCarY;
        
        const now = performance.now() / 1000;
        const timeSinceLanding = now - landingTime;

        if (isChargingJump) {
            const chargeDuration = now - jumpChargeStartTime;
            const chargeT = Math.min(chargeDuration / JUMP_MAX_CHARGE_TIME, 1);
            pitchRotation = JUMP_CHARGE_TILT * chargeT;
        } else if (timeSinceLanding >= 0.3) {
            // after 0.3s grounded, reset suspension to neutral
            targetSuspensionOffset = 0;
        }
    }

    // guard against division by very small dt to prevent extreme lateral velocity values
    const lateralV = dt > 0.001 ? (carX - prevX) / dt : 0;
    const yaw = Math.atan2(-lateralV, CAR_SPEED) * 0.5;

    // combine yaw with pitch from jump
    quat.setAxisAngle(_updateCarMovement_rotQuat, _updateCarMovement_yAxis, yaw);
    if (pitchRotation !== 0) {
        quat.setAxisAngle(_updateCarMovement_pitchQuat, _updateCarMovement_xAxis, pitchRotation);
        quat.multiply(_updateCarMovement_rotQuat, _updateCarMovement_rotQuat, _updateCarMovement_pitchQuat);
    }

    vec3.set(_updateCarMovement_targetPos, carX, carY, carZ);
    rigidBody.moveKinematic(carBody, _updateCarMovement_targetPos, _updateCarMovement_rotQuat, dt);

    return lateralV;
}

function updateCarMesh(now: number, dt: number) {
    // update suspension spring
    suspensionVelocity += (targetSuspensionOffset - suspensionOffset) * SUSPENSION_SPRING_K * dt;
    suspensionVelocity *= SUSPENSION_DAMPING;
    suspensionOffset += suspensionVelocity * dt;

    carObject.position.set(carBody.position[0], carBody.position[1] + 0.1 + suspensionOffset, carBody.position[2]);
    carObject.quaternion.set(carBody.quaternion[0], carBody.quaternion[1], carBody.quaternion[2], carBody.quaternion[3]);

    // chassis hit reaction — decaying oscillation
    const timeSinceHit = now / 1000 - hitTime;
    const envelope = calculateDecay(hitStrength, HIT_DECAY, timeSinceHit);
    if (envelope > 0.001) {
        const osc = Math.sin(HIT_FREQ * timeSinceHit);
        carObject.position.y += osc * envelope * HIT_Y_AMPLITUDE;
        carObject.rotation.z += osc * envelope * HIT_ROLL_AMPLITUDE * hitNormalX;
        carObject.rotation.x += osc * envelope * HIT_PITCH_AMPLITUDE * hitNormalZ;
    }
}

function updateCatMesh(now: number, lateralV: number, dt: number) {
    // clamp normalized lateral to prevent extreme values from affecting cat animation
    const normalizedLateral = Math.max(-2, Math.min(2, lateralV / (CAR_SPEED * 0.5)));
    const targetRoll = -normalizedLateral * CAT_SWAY_ROLL;
    const targetYaw = -normalizedLateral * CAT_SWAY_YAW;
    const targetPitch = Math.abs(normalizedLateral) * CAT_SWAY_PITCH;
    catSwayRoll += (targetRoll - catSwayRoll) * Math.min(1, CAT_SWAY_LERP * dt);
    catSwayYaw += (targetYaw - catSwayYaw) * Math.min(1, CAT_SWAY_LERP * dt);
    catSwayPitch += (targetPitch - catSwayPitch) * Math.min(1, CAT_SWAY_LERP * dt);

    catObject.position.copy(catBasePosition);
    catObject.rotation.copy(catBaseRotation);

    catObject.rotation.z += catSwayRoll;
    catObject.rotation.y += catSwayYaw;
    catObject.rotation.x += catSwayPitch;

    // cat hit reaction
    const catTimeSinceHit = now / 1000 - catHitTime;
    const catEnvelope = calculateDecay(catHitStrength, CAT_HIT_DECAY, catTimeSinceHit);
    if (catEnvelope > 0.001) {
        const catOsc = Math.sin(CAT_HIT_FREQ * catTimeSinceHit);
        catObject.position.y += catOsc * catEnvelope * CAT_HIT_Y_AMPLITUDE;
        catObject.rotation.z += catOsc * catEnvelope * CAT_HIT_ROLL_AMPLITUDE * catHitNormalX;
        catObject.rotation.x += catOsc * catEnvelope * CAT_HIT_PITCH_AMPLITUDE * catHitNormalZ;
    }

    // clamp cat rotation
    catObject.rotation.x = Math.max(
        catBaseRotation.x - CAT_MAX_ROT,
        Math.min(catBaseRotation.x + CAT_MAX_ROT, catObject.rotation.x),
    );
    catObject.rotation.y = Math.max(
        catBaseRotation.y - CAT_MAX_ROT,
        Math.min(catBaseRotation.y + CAT_MAX_ROT, catObject.rotation.y),
    );
    catObject.rotation.z = Math.max(
        catBaseRotation.z - CAT_MAX_ROT,
        Math.min(catBaseRotation.z + CAT_MAX_ROT, catObject.rotation.z),
    );
}

function syncObstacleMeshes() {
    for (const [, tile] of activeTiles) {
        for (const obs of tile.obstacles) {
            const b = obs.body;
            obstacleBatchUpdate(
                obs.instanceId,
                b.position[0],
                b.position[1],
                b.position[2],
                b.quaternion[0],
                b.quaternion[1],
                b.quaternion[2],
                b.quaternion[3],
            );
        }
        for (const wb of tile.wreckingBalls) {
            for (const seg of wb.ropeSegments) {
                const b = seg.body;
                obstacleBatchUpdate(
                    seg.instanceId,
                    b.position[0],
                    b.position[1],
                    b.position[2],
                    b.quaternion[0],
                    b.quaternion[1],
                    b.quaternion[2],
                    b.quaternion[3],
                );
            }
            const ball = wb.ballBody;
            obstacleBatchUpdate(
                wb.ballInstanceId,
                ball.position[0],
                ball.position[1],
                ball.position[2],
                ball.quaternion[0],
                ball.quaternion[1],
                ball.quaternion[2],
                ball.quaternion[3],
            );
        }
    }
}

function updateCamera() {
    camera.position.x = 0;
    camera.position.y = 3;
    camera.position.z = carBody.position[2] + 8;
    camera.lookAt(carBody.position[0], 1, carZ - 10);

    directionalLight.position.set(carBody.position[0] + 10, 40, carZ + 5);
    directionalLight.target.position.set(carBody.position[0], 0, carZ - 50);
    directionalLight.target.updateMatrixWorld();

    stars.position.z = carBody.position[2];
}

function update() {
    const now = performance.now();
    const dt = Math.min((now - prevTime) / 1000, 1 / 30);
    prevTime = now;

    const lateralV = updateCarMovement(dt);
    updateWorld(world, listener, dt);
    updateCarMesh(now, dt);
    updateCatMesh(now, lateralV, dt);
    updateTiles(carZ);
    syncObstacleMeshes();
    updateCamera();

    debugRenderer.update(debugRendererState, world);
    renderer.render(scene, camera);
    requestAnimationFrame(update);
}

document.querySelector('#loading')!.remove();
update();
