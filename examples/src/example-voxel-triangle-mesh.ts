import { createSimplex2D, quat, vec3 } from 'mathcat';
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
    enableCollision,
    MotionType,
    registerAllShapes,
    rigidBody,
    sphere,
    transformed,
    triangleMesh,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

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

/* voxels */

const CHUNK_BITS = 4;
const CHUNK_SIZE = 1 << CHUNK_BITS;
const CHUNK_MASK = CHUNK_SIZE - 1;

type VoxelChunk = {
    cx: number;
    cy: number;
    cz: number;
    solid: Uint8Array;
    body: RigidBody | null;
};

type Voxels = {
    chunks: Map<string, VoxelChunk>;
};

function createVoxels(): Voxels {
    return {
        chunks: new Map<string, VoxelChunk>(),
    };
}

function setVoxel(voxels: Voxels, x: number, y: number, z: number, value: boolean): void {
    const cx = x >> CHUNK_BITS;
    const cy = y >> CHUNK_BITS;
    const cz = z >> CHUNK_BITS;
    const chunkKey = `${cx},${cy},${cz}`;
    let chunk = voxels.chunks.get(chunkKey);
    if (!chunk) {
        chunk = {
            cx,
            cy,
            cz,
            solid: new Uint8Array(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE),
            body: null,
        };
        voxels.chunks.set(chunkKey, chunk);
    }
    const lx = x & CHUNK_MASK;
    const ly = y & CHUNK_MASK;
    const lz = z & CHUNK_MASK;
    const index = lx + ly * CHUNK_SIZE + lz * CHUNK_SIZE * CHUNK_SIZE;
    chunk.solid[index] = value ? 1 : 0;
}

function getVoxel(voxels: Voxels, x: number, y: number, z: number): boolean {
    const cx = x >> CHUNK_BITS;
    const cy = y >> CHUNK_BITS;
    const cz = z >> CHUNK_BITS;
    const chunkKey = `${cx},${cy},${cz}`;
    const chunk = voxels.chunks.get(chunkKey);
    if (!chunk) {
        return false;
    }
    const lx = x & CHUNK_MASK;
    const ly = y & CHUNK_MASK;
    const lz = z & CHUNK_MASK;
    const index = lx + ly * CHUNK_SIZE + lz * CHUNK_SIZE * CHUNK_SIZE;
    return chunk.solid[index] !== 0;
}

type VoxelChunkMesh = {
    positions: number[];
    indices: number[];
    normals: number[];
};

// Direction vectors for computing quad vertices
// DIRECTION_VECTORS[axis][0] gives the "u" direction, [axis][1] gives the "v" direction
const DIRECTION_VECTORS: number[][][] = [];
for (let i = 0; i < 3; ++i) {
    DIRECTION_VECTORS[i] = [
        [0, 0, 0],
        [0, 0, 0],
    ];
    DIRECTION_VECTORS[i][0][(i + 1) % 3] = 1;
    DIRECTION_VECTORS[i][1][(i + 2) % 3] = 1;
}

// Face normals for each face direction
const FACE_NORMALS: [number, number, number][] = [
    [1, 0, 0], // +X
    [-1, 0, 0], // -X
    [0, 1, 0], // +Y
    [0, -1, 0], // -Y
    [0, 0, 1], // +Z
    [0, 0, -1], // -Z
];

// Maps (axis, side) to face index
// side 0 = current block is solid (positive face)
// side 1 = neighbor block is solid (negative face)
const AXIS_SIDE_TO_FACE: number[][] = [
    [0, 1], // X axis: side 0 -> +X, side 1 -> -X
    [2, 3], // Y axis: side 0 -> +Y, side 1 -> -Y
    [4, 5], // Z axis: side 0 -> +Z, side 1 -> -Z
];

function createVoxelChunkMesh(voxels: Voxels, cx: number, cy: number, cz: number): VoxelChunkMesh {
    const positions: number[] = [];
    const indices: number[] = [];
    const normals: number[] = [];

    // World offset for this chunk
    const worldOffsetX = cx * CHUNK_SIZE;
    const worldOffsetY = cy * CHUNK_SIZE;
    const worldOffsetZ = cz * CHUNK_SIZE;

    // March directions: +X, +Y, +Z
    const marchDirs = [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ];

    // March from -1 to CHUNK_SIZE-1 so we can compare with neighbors at CHUNK_SIZE
    for (let x = -1; x < CHUNK_SIZE; x++) {
        for (let y = -1; y < CHUNK_SIZE; y++) {
            for (let z = -1; z < CHUNK_SIZE; z++) {
                const worldX = worldOffsetX + x;
                const worldY = worldOffsetY + y;
                const worldZ = worldOffsetZ + z;

                const currentSolid = getVoxel(voxels, worldX, worldY, worldZ);

                // Check each march direction (X, Y, Z)
                for (let axis = 0; axis < 3; axis++) {
                    const [dx, dy, dz] = marchDirs[axis];
                    const neighborX = worldX + dx;
                    const neighborY = worldY + dy;
                    const neighborZ = worldZ + dz;

                    const neighborSolid = getVoxel(voxels, neighborX, neighborY, neighborZ);

                    // If both are solid or both are air, no face needed
                    if (currentSolid === neighborSolid) continue;

                    // side: 0 if current is solid (face belongs to current), 1 if neighbor is solid
                    const side = currentSolid ? 0 : 1;

                    // Determine the block position that owns this face
                    const blockX = x + (side === 1 ? dx : 0);
                    const blockY = y + (side === 1 ? dy : 0);
                    const blockZ = z + (side === 1 ? dz : 0);

                    // Only create faces for blocks inside this chunk
                    if (
                        blockX < 0 ||
                        blockX >= CHUNK_SIZE ||
                        blockY < 0 ||
                        blockY >= CHUNK_SIZE ||
                        blockZ < 0 ||
                        blockZ >= CHUNK_SIZE
                    ) {
                        continue;
                    }

                    // Get face index and normal
                    const faceIndex = AXIS_SIDE_TO_FACE[axis][side];
                    const [nx, ny, nz] = FACE_NORMALS[faceIndex];

                    // Get u and v direction vectors for this face
                    const [ux, uy, uz] = DIRECTION_VECTORS[axis][side];
                    const [vx, vy, vz] = DIRECTION_VECTORS[axis][side ^ 1];

                    // The face starts at the neighbor position (on the boundary)
                    const faceX = worldOffsetX + x + dx;
                    const faceY = worldOffsetY + y + dy;
                    const faceZ = worldOffsetZ + z + dz;

                    // Create 4 vertices for the quad
                    // Vertex 0: base
                    positions.push(faceX, faceY, faceZ);
                    // Vertex 1: base + u
                    positions.push(faceX + ux, faceY + uy, faceZ + uz);
                    // Vertex 2: base + u + v
                    positions.push(faceX + ux + vx, faceY + uy + vy, faceZ + uz + vz);
                    // Vertex 3: base + v
                    positions.push(faceX + vx, faceY + vy, faceZ + vz);

                    // Normals (same for all 4 vertices)
                    normals.push(nx, ny, nz);
                    normals.push(nx, ny, nz);
                    normals.push(nx, ny, nz);
                    normals.push(nx, ny, nz);

                    // Indices for two triangles
                    // Quad vertices: a=0, b=1, c=2, d=3
                    const baseIndex = positions.length / 3 - 4;
                    const a = baseIndex;
                    const b = baseIndex + 1;
                    const c = baseIndex + 2;
                    const d = baseIndex + 3;

                    // Triangle 1: a, b, d
                    // Triangle 2: b, c, d
                    indices.push(a, b, d);
                    indices.push(b, c, d);
                }
            }
        }
    }

    return {
        positions,
        indices,
        normals,
    };
}

/**
 * Build a chunk's physics body.
 */
function buildChunk(chunk: VoxelChunk, voxels: Voxels, world: ReturnType<typeof createWorld>, objectLayer: number): void {
    // Generate mesh data
    const meshData = createVoxelChunkMesh(voxels, chunk.cx, chunk.cy, chunk.cz);

    // If the chunk has no geometry, we're done
    if (meshData.positions.length === 0) {
        return;
    }

    // Create physics body with triangle mesh shape
    const shape = triangleMesh.create({
        positions: meshData.positions,
        indices: meshData.indices,
        bvhSplitStrategy: triangleMesh.BvhSplitStrategy.CENTER,
    });

    const body = rigidBody.create(world, {
        shape,
        objectLayer,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 0, 0),
        restitution: 0.2,
        friction: 0.7,
    });

    chunk.body = body;
}

/**
 * Build all chunks.
 */
function buildAllChunks(voxels: Voxels, world: ReturnType<typeof createWorld>, objectLayer: number): void {
    for (const chunk of voxels.chunks.values()) {
        buildChunk(chunk, voxels, world, objectLayer);
    }
}

/* voxel terrain */

const voxels = createVoxels();

// Create terrain using layered simplex noise
const TERRAIN_SIZE = 32;
const TERRAIN_HEIGHT = 16;
const BASE_HEIGHT = 4;

const simplexNoise = createSimplex2D(42);

for (let x = -TERRAIN_SIZE; x < TERRAIN_SIZE; x++) {
    for (let z = -TERRAIN_SIZE; z < TERRAIN_SIZE; z++) {
        // Layer 1: Large scale hills
        const noise1 = simplexNoise(x * 0.02, z * 0.02) * 0.5 + 0.5; // 0-1
        // Layer 2: Medium details
        const noise2 = simplexNoise(x * 0.08, z * 0.08) * 0.5 + 0.5; // 0-1
        // Layer 3: Small details
        const noise3 = simplexNoise(x * 0.2, z * 0.2) * 0.5 + 0.5; // 0-1

        // Combine layers with different weights
        const combinedNoise = noise1 * 0.6 + noise2 * 0.3 + noise3 * 0.1;
        const height = Math.floor(BASE_HEIGHT + combinedNoise * TERRAIN_HEIGHT);

        for (let y = 0; y < height; y++) {
            setVoxel(voxels, x, y, z, true);
        }
    }
}

// Build all chunk meshes once
buildAllChunks(voxels, world, OBJECT_LAYER_NOT_MOVING);

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

/* shape spawning system */

type ShapeConfig = {
    name: string;
    createShape: () => Shape;
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
                    -0.5,
                    0,
                    -0.5, // bottom-left
                    0.5,
                    0,
                    -0.5, // bottom-right
                    0.5,
                    0,
                    0.5, // top-right
                    -0.5,
                    0,
                    0.5, // top-left
                    // Apex (centered above at y=1)
                    0,
                    1,
                    0,
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
    respawnIntervalMs: 500,
};

const SPAWN_HEIGHT = 25;
const SPAWN_AREA = 10; // +/- units on x and z

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

ui.gui
    .add(settings, 'numberOfBodies', 0, 1000, 1)
    .name('Number of Bodies')
    .onChange(() => {
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
