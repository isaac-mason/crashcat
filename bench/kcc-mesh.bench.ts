// Macro perf benchmark — kcc-mesh.
//
// "Characters running around on a triangle mesh." A procedural sine/cos
// heightfield terrain (64×64 quads over ~50×50m), 16 kinematic character
// controllers roaming with seeded per-character headings that drift over time
// (so they climb slopes and occasionally bump each other), plus 20 dynamic
// boxes they can knock around. This exercises the KCC shape-cast /
// collide-shape path against a triangle mesh BVH — the dominant cost in a real
// third-person / top-down game frame — rather than the contact solver.
//
// Deterministic mulberry32 RNG. The measured op re-seeds with a FIXED seed each
// iteration so the workload is identical (no cube-heap seedNonce noise). Warmup
// uses a different fixed seed. `runForProfiling` is the entry the cpu-prof rig
// invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    enableCollision,
    type Filter,
    filter,
    type KCC,
    kcc,
    MotionType,
    registerAll,
    rigidBody,
    type Shape,
    transformed,
    triangleMesh,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';
import { quat, type Vec3, vec3, vec4 } from 'mathcat';

registerAll();

const NUMBER_OF_CHARACTERS = 16;
const NUMBER_OF_PROPS = 20;
const STEPS_PER_OP = 300; // 5s of sim at 60Hz
const TIME_STEP = 1 / 60;
const GRAVITY: Vec3 = [0, -20, 0];

// terrain: 64×64 quads over ~50×50m, ±2m of smooth height variation
const TERRAIN_QUADS = 64;
const TERRAIN_EXTENT = 50;
const TERRAIN_CELL = TERRAIN_EXTENT / TERRAIN_QUADS;
const TERRAIN_HALF = TERRAIN_EXTENT / 2;

const AGENT_RADIUS = 0.4;
const AGENT_HALF_HEIGHT = 0.5;
const AGENT_SPEED = 3;
const ROAM_HALF = 18; // keep characters within the interesting middle of the terrain

const RNG_SEED = 0xc0ffee;
const OP_SEED = 0x5eed11fe;
const WARMUP_SEED = RNG_SEED ^ 0xdeadbeef;
const STEADY_WARMUP_STEPS = 240;

function makeRng(seed: number): () => number {
    let s = seed >>> 0;
    return () => {
        s = (s + 0x6d2b79f5) >>> 0;
        let t = s;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

// smooth ±2m heightfield — amplitudes sum to 2
function terrainHeight(x: number, z: number): number {
    return (
        Math.sin(x * 0.18) * Math.cos(z * 0.18) * 1.2 +
        Math.sin(x * 0.42 + 1.0) * Math.sin(z * 0.37) * 0.5 +
        Math.cos((x + z) * 0.11) * 0.3
    );
}

function makeWorldSettings(): WorldSettings {
    const s = createWorldSettings();
    const BPH_STATIC = addBroadphaseLayer(s);
    const BPH_MOVING = addBroadphaseLayer(s);
    const OL_STATIC = addObjectLayer(s, BPH_STATIC);
    const OL_MOVING = addObjectLayer(s, BPH_MOVING);
    enableCollision(s, OL_MOVING, OL_MOVING);
    enableCollision(s, OL_MOVING, OL_STATIC);
    s.gravity = GRAVITY;
    return s;
}

const OL_STATIC = 0;
const OL_MOVING = 1;

// character capsule offset so the character origin sits at the base of the shape
const agentShape: Shape = transformed.create({
    shape: capsule.create({ halfHeightOfCylinder: AGENT_HALF_HEIGHT, radius: AGENT_RADIUS }),
    position: [0, AGENT_HALF_HEIGHT + AGENT_RADIUS, 0],
    quaternion: quat.create(),
});

const propShape: Shape = box.create({ halfExtents: [0.3, 0.3, 0.3], convexRadius: 0.04 });

function makeTerrainShape(): Shape {
    const positions: number[] = [];
    for (let iz = 0; iz <= TERRAIN_QUADS; iz++) {
        for (let ix = 0; ix <= TERRAIN_QUADS; ix++) {
            const x = ix * TERRAIN_CELL - TERRAIN_HALF;
            const z = iz * TERRAIN_CELL - TERRAIN_HALF;
            positions.push(x, terrainHeight(x, z), z);
        }
    }
    const indices: number[] = [];
    const row = TERRAIN_QUADS + 1;
    for (let iz = 0; iz < TERRAIN_QUADS; iz++) {
        for (let ix = 0; ix < TERRAIN_QUADS; ix++) {
            const bl = iz * row + ix;
            const br = bl + 1;
            const tl = bl + row;
            const tr = tl + 1;
            indices.push(bl, tl, br);
            indices.push(br, tl, tr);
        }
    }
    return triangleMesh.create({ positions, indices });
}

const terrainShape: Shape = makeTerrainShape();

type Agent = {
    character: KCC;
    targetAngle: number;
    turnTimer: number;
};

type State = {
    world: World;
    agents: Agent[];
    props: RigidBodyHandle[];
    charFilter: Filter;
    updateSettings: ReturnType<typeof kcc.createDefaultUpdateSettings>;
};

type RigidBodyHandle = ReturnType<typeof rigidBody.create>;

function populate(world: World, rng: () => number): State {
    // static terrain
    rigidBody.create(world, {
        shape: terrainShape,
        objectLayer: OL_STATIC,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        friction: 0.6,
        restitution: 0,
    });

    const charFilter = filter.create(world.settings.layers);
    const updateSettings = kcc.createDefaultUpdateSettings();

    const agents: Agent[] = [];
    for (let i = 0; i < NUMBER_OF_CHARACTERS; i++) {
        const x = (rng() * 2 - 1) * ROAM_HALF;
        const z = (rng() * 2 - 1) * ROAM_HALF;
        const y = terrainHeight(x, z) + 0.5;
        const character = kcc.create(
            {
                ...kcc.DEFAULT_KCC_SETTINGS,
                shape: agentShape,
                mass: 70,
                maxSlopeAngle: (45 * Math.PI) / 180,
                characterPadding: 0.02,
                innerRigidBody: { shape: agentShape, objectLayer: OL_MOVING },
                supportingVolumePlane: vec4.fromValues(0, 1, 0, -AGENT_RADIUS),
            },
            [x, y, z],
            quat.create(),
        );
        kcc.add(world, character);
        agents.push({ character, targetAngle: rng() * Math.PI * 2, turnTimer: rng() * 2 });
    }

    const props: RigidBodyHandle[] = [];
    for (let i = 0; i < NUMBER_OF_PROPS; i++) {
        const x = (rng() * 2 - 1) * ROAM_HALF;
        const z = (rng() * 2 - 1) * ROAM_HALF;
        const y = terrainHeight(x, z) + 1 + rng() * 2;
        props.push(
            rigidBody.create(world, {
                shape: propShape,
                objectLayer: OL_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [x, y, z],
                friction: 0.5,
                restitution: 0,
                mass: 2,
            }),
        );
    }

    return { world, agents, props, charFilter, updateSettings };
}

const _vertical = vec3.create();
const _horizontal = vec3.create();

function steerAgent(world: World, agent: Agent, rng: () => number): void {
    const character = agent.character;

    // pick a new heading occasionally; steer back toward the center if wandering out
    agent.turnTimer -= TIME_STEP;
    if (agent.turnTimer <= 0) {
        agent.targetAngle += (rng() - 0.5) * Math.PI;
        agent.turnTimer = 1 + rng() * 2;
    }
    const pos = character.position;
    if (Math.abs(pos[0]) > ROAM_HALF || Math.abs(pos[2]) > ROAM_HALF) {
        agent.targetAngle = Math.atan2(-pos[2], -pos[0]);
    }

    kcc.updateGroundVelocity(world, character);

    const up = character.up;
    const lv = character.linearVelocity;
    // keep only the vertical component of current velocity when airborne
    vec3.scale(_vertical, up, vec3.dot(lv, up));

    if (character.ground.state === kcc.GroundState.ON_GROUND) {
        vec3.copy(_horizontal, character.ground.velocity);
    } else {
        vec3.copy(_horizontal, _vertical);
    }
    // apply gravity
    vec3.scaleAndAdd(_horizontal, _horizontal, GRAVITY, TIME_STEP);
    // desired horizontal locomotion
    _horizontal[0] += Math.cos(agent.targetAngle) * AGENT_SPEED;
    _horizontal[2] += Math.sin(agent.targetAngle) * AGENT_SPEED;
    vec3.copy(lv, _horizontal);
}

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` steers +
 * updates every character (the KCC shape-cast work) then advances the sim.
 * Warmup uses WARMUP_SEED; the op rng is re-seeded with the FIXED OP_SEED at
 * each op boundary (labs and profiling both use OP_SEED) so repeated ops replay
 * the identical steering. World-state drift is fine.
 */
function build(): { scenario: Scenario; agents: Agent[] } {
    const state = populate(createWorld(makeWorldSettings()), makeRng(RNG_SEED));
    const { world, agents, updateSettings, charFilter } = state;
    const warmupRng = makeRng(WARMUP_SEED);
    let opRng = makeRng(OP_SEED);

    const stepAll = (rng: () => number): void => {
        for (let a = 0; a < agents.length; a++) {
            steerAgent(world, agents[a], rng);
            kcc.update(world, agents[a].character, TIME_STEP, GRAVITY, updateSettings, undefined, charFilter);
        }
        updateWorld(world, undefined, TIME_STEP);
    };

    const scenario: Scenario = {
        world,
        warmupSteps: STEADY_WARMUP_STEPS,
        stepOnce(stepIndex: number): void {
            if (stepIndex < STEADY_WARMUP_STEPS) {
                stepAll(warmupRng);
            } else {
                const g = stepIndex - STEADY_WARMUP_STEPS;
                if (g % STEPS_PER_OP === 0) opRng = makeRng(OP_SEED);
                stepAll(opRng);
            }
        },
    };
    return { scenario, agents };
}

export function createScenario(): Scenario {
    return build().scenario;
}

group('kcc-mesh', () => {
    bench('kcc-mesh', function* () {
        const s = createScenario();
        for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
        yield () => {
            for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
        };
    }).gc('inner');
});

export function runForProfiling(): void {
    const s = createScenario();
    for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
    for (let i = 0; i < 5 * STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
}

// TEMP sanity
export function __sanity(): void {
    const { scenario: s, agents } = build();
    const t0 = performance.now();
    for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
    const t1 = performance.now();
    for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
    const t2 = performance.now();
    let onGround = 0;
    let minAbove = Infinity;
    let fellThrough = 0;
    for (const a of agents) {
        const p = a.character.position;
        if (kcc.isSupported(a.character)) onGround++;
        const above = p[1] - terrainHeight(p[0], p[2]);
        minAbove = Math.min(minAbove, above);
        if (p[1] < -10) fellThrough++;
    }
    console.error(
        `[kcc-mesh] warmup ${(t1 - t0).toFixed(0)}ms  op ${(t2 - t1).toFixed(0)}ms  ` +
            `onGround ${onGround}/${agents.length}  minAboveTerrain ${minAbove.toFixed(2)}m  fellThrough ${fellThrough}`,
    );
}
