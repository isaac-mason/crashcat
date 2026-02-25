import { quat, vec3, vec4 } from 'mathcat';
import type { Vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
    type KCC,
    kcc,
    MotionType,
    rigidBody,
    transformed,
    triangleMesh,
    updateWorld,
    registerAll,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* physics world */

registerAll();

const worldSettings = createWorldSettings();
worldSettings.gravity = vec3.fromValues(0, -25, 0);

const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);

const LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
const LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

enableCollision(worldSettings, LAYER_NON_MOVING, LAYER_MOVING);
enableCollision(worldSettings, LAYER_MOVING, LAYER_MOVING);

const world = createWorld(worldSettings);

/* scene */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x333333);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 40, 70);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(10, 20, 10);
scene.add(directionalLight);

/* debugging */

const options = debugRenderer.createDefaultOptions();
const debugState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugState.object3d);

/* terrain — 32×32 grid, 64×64 meters */

{
    const gridSize = 32;
    const cellSize = 2.0;
    const heightScale = 2;

    const heightAt = (x: number, z: number): number => {
        const freq1 = 0.3;
        const freq2 = 0.7;
        const freq3 = 0.15;
        return (
            Math.sin(x * freq1) * Math.cos(z * freq1) * heightScale * 0.5 +
            Math.sin(x * freq2 + 1.0) * Math.sin(z * freq2) * heightScale * 0.3 +
            Math.cos((x + z) * freq3) * heightScale * 0.2
        );
    };

    const positions: number[] = [];
    for (let iz = 0; iz <= gridSize; iz++) {
        for (let ix = 0; ix <= gridSize; ix++) {
            const x = (ix - gridSize / 2) * cellSize;
            const z = (iz - gridSize / 2) * cellSize;
            const y = heightAt(x, z);
            positions.push(x, y, z);
        }
    }

    const indices: number[] = [];
    for (let iz = 0; iz < gridSize; iz++) {
        for (let ix = 0; ix < gridSize; ix++) {
            const row = gridSize + 1;
            const bl = iz * row + ix;
            const br = bl + 1;
            const tl = bl + row;
            const tr = tl + 1;
            indices.push(bl, tl, br);
            indices.push(br, tl, tr);
        }
    }

    const terrainShape = triangleMesh.create({ positions, indices });
    rigidBody.create(world, {
        shape: terrainShape,
        position: vec3.fromValues(0, 0, 0),
        motionType: MotionType.STATIC,
        objectLayer: LAYER_NON_MOVING,
    });
}

// boundary walls to keep agents on the terrain (terrain spans ±32m in X and Z)
const terrainBound = 31; // slightly inside mesh edge
const wallHeight = 6;
const wallThickness = 1;
for (const [wx, wz, hx, hz] of [
    [0, terrainBound, terrainBound + wallThickness, wallThickness],
    [0, -terrainBound, terrainBound + wallThickness, wallThickness],
    [terrainBound, 0, wallThickness, terrainBound + wallThickness],
    [-terrainBound, 0, wallThickness, terrainBound + wallThickness],
] as [number, number, number, number][]) {
    rigidBody.create(world, {
        shape: box.create({ halfExtents: vec3.fromValues(hx, wallHeight, hz) }),
        position: vec3.fromValues(wx, wallHeight, wz),
        motionType: MotionType.STATIC,
        objectLayer: LAYER_NON_MOVING,
    });
}

/* settings */

const settings = {
    showDebug: true,
    agentCount: 50,
};

/* agents */

const AGENT_SPEED = 4.0;
const AGENT_RADIUS = 0.4;
const AGENT_HALF_HEIGHT = 0.5;
const SPAWN_Y = 10;
const FALL_THRESHOLD = -4;

type AgentState = {
    character: KCC;
    targetAngle: number;
    turnTimer: number;
    spawnPosition: Vec3;
};

const characterFilter = filter.create(world.settings.layers);
const updateSettings = kcc.createDefaultUpdateSettings();

function createAgent(spawnX: number, spawnZ: number): AgentState {
    const position = vec3.fromValues(spawnX, SPAWN_Y, spawnZ);

    const agentShape = transformed.create({
        shape: capsule.create({
            halfHeightOfCylinder: AGENT_HALF_HEIGHT,
            radius: AGENT_RADIUS,
        }),
        position: vec3.fromValues(0, AGENT_HALF_HEIGHT + AGENT_RADIUS, 0),
        quaternion: quat.create(),
    });

    const character = kcc.create(
        {
            ...kcc.DEFAULT_KCC_SETTINGS,
            shape: agentShape,
            mass: 70,
            maxSlopeAngle: (45.0 * Math.PI) / 180,
            characterPadding: 0.02,
            innerRigidBody: {
                shape: agentShape,
                objectLayer: LAYER_MOVING,
            },
            supportingVolumePlane: vec4.fromValues(0, 1, 0, -AGENT_RADIUS),
        },
        position,
        quat.create(),
    );

    kcc.add(world, character);

    return {
        character,
        targetAngle: Math.random() * Math.PI * 2,
        turnTimer: 1 + Math.random() * 2,
        spawnPosition: vec3.clone(position),
    };
}

const agents: AgentState[] = [];
// safe spawn area: stay well inside the ±31m boundary walls
const SPAWN_BOUND = 24;

function spawnAgent(index: number): void {
    const target = settings.agentCount;
    const cols = Math.ceil(Math.sqrt(target));
    const rows = Math.ceil(target / cols);
    const col = index % cols;
    const row = Math.floor(index / cols);
    // evenly distribute across the safe area
    const x = cols > 1 ? (col / (cols - 1) - 0.5) * 2 * SPAWN_BOUND : 0;
    const z = rows > 1 ? (row / (rows - 1) - 0.5) * 2 * SPAWN_BOUND : 0;
    agents.push(createAgent(x, z));
}

function updateAgentCount(): void {
    // remove all existing agents and respawn to recompute grid layout
    while (agents.length > 0) {
        kcc.remove(world, agents.pop()!.character);
    }
    for (let i = 0; i < settings.agentCount; i++) {
        spawnAgent(i);
    }
}

// initial spawn
updateAgentCount();

/* agent AI update */

function updateAgent(agent: AgentState, deltaTime: number): void {
    // respawn if fallen through floor
    if (agent.character.position[1] < FALL_THRESHOLD) {
        kcc.setPosition(world, agent.character, agent.spawnPosition);
        vec3.zero(agent.character.linearVelocity);
        return;
    }

    // turn timer — pick a new random heading when it expires
    agent.turnTimer -= deltaTime;
    if (agent.turnTimer <= 0) {
        agent.targetAngle += (Math.random() - 0.5) * Math.PI; // ±90°
        agent.turnTimer = 1 + Math.random() * 2;
    }

    // update ground velocity (needed for ground tracking)
    kcc.updateGroundVelocity(world, agent.character);

    const characterUp = agent.character.up;
    const linearVelocity = agent.character.linearVelocity;
    const currentVerticalVelocity = vec3.scale(vec3.create(), characterUp, vec3.dot(linearVelocity, characterUp));
    const groundVelocity = agent.character.ground.velocity;
    const gravity = worldSettings.gravity;

    const newVelocity = vec3.create();

    if (agent.character.ground.state === kcc.GroundState.ON_GROUND) {
        // match ground velocity when standing
        vec3.copy(newVelocity, groundVelocity);
    } else {
        // carry vertical velocity when airborne
        vec3.copy(newVelocity, currentVerticalVelocity);
    }

    // apply gravity
    vec3.scaleAndAdd(newVelocity, newVelocity, gravity, deltaTime);

    // add horizontal movement
    const dx = Math.cos(agent.targetAngle) * AGENT_SPEED;
    const dz = Math.sin(agent.targetAngle) * AGENT_SPEED;
    newVelocity[0] += dx;
    newVelocity[2] += dz;

    vec3.copy(agent.character.linearVelocity, newVelocity);
}

/* GUI */

ui.gui.add(settings, 'showDebug').name('Show Debug');
ui.gui.add(settings, 'agentCount', 0, 100, 1).name('Agent Count').onChange(() => {
    updateAgentCount();
});

/* animation loop */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const deltaTime = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    debugUI.beginPerf(ui);

    for (const agent of agents) {
        updateAgent(agent, deltaTime);
        kcc.update(world, agent.character, deltaTime, worldSettings.gravity, updateSettings, undefined, characterFilter);
    }

    updateWorld(world, undefined, deltaTime);

    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    if (settings.showDebug) {
        debugRenderer.update(debugState, world);
    } else {
        debugRenderer.clear(debugState);
    }

    controls.update();
    renderer.render(scene, camera);
}

animate();

/* window resize */

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});
