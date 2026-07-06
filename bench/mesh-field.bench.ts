// Macro perf benchmark — mesh-field.
//
// "Lots of isolated dynamic bodies spread across a big triangle mesh." A large
// sine/cos heightfield (128×128 quads over ~200×200m) with 200 dynamic bodies
// of MIXED shape types (boxes, spheres, capsules, small convex hulls) scattered
// widely, so each body mostly collides with the MESH rather than its neighbours.
// A gentle round-robin churn re-drops a few bodies per second at random spots so
// a realistic fraction is asleep at steady state — this keeps the sleeping
// system and broadphase coverage (not just the contact solver) in the profile.
//
// Deterministic mulberry32 RNG. The measured op re-seeds with a FIXED seed each
// iteration and restarts the churn cursor so the workload is identical (no
// cube-heap seedNonce noise). `runForProfiling` is the entry the cpu-prof rig
// invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';

import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    convexHull,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    type RigidBody,
    registerAll,
    rigidBody,
    type Shape,
    sphere,
    triangleMesh,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const NUMBER_OF_BODIES = 200;
const STEPS_PER_OP = 300; // 5s of sim at 60Hz
const TIME_STEP = 1 / 60;

// gentle churn: re-drop one body every N steps, round-robin. At N=3 this is
// ~20 re-drops/second, so most of the field settles and sleeps while a rolling
// handful stays awake — tuned so ~30-70% of the field is asleep at steady
// state (sleeping + broadphase coverage is what this scenario measures). Any
// body that has slid off an edge is teleported back as the cursor reaches it.
const CHURN_INTERVAL = 3; // re-drop one body every N steps (~20/sec)
const SPAWN_HEIGHT = 4; // metres above the terrain a re-dropped body appears

// terrain: 128×128 quads over ~200×200m, ±3m of smooth height variation
const TERRAIN_QUADS = 128;
const TERRAIN_EXTENT = 200;
const TERRAIN_CELL = TERRAIN_EXTENT / TERRAIN_QUADS;
const TERRAIN_HALF = TERRAIN_EXTENT / 2;
const SCATTER_HALF = 70; // bodies scattered within ±70m so they stay well clear of edges

const RNG_SEED = 0xc0ffee;
const OP_SEED = 0x5eed11fe;
const WARMUP_SEED = RNG_SEED ^ 0xdeadbeef;
const STEADY_WARMUP_STEPS = 600;

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

// smooth ±3m heightfield — amplitudes sum to 3
function terrainHeight(x: number, z: number): number {
    return (
        Math.sin(x * 0.08) * Math.cos(z * 0.08) * 1.8 +
        Math.sin(x * 0.19 + 1.0) * Math.sin(z * 0.17) * 0.8 +
        Math.cos((x + z) * 0.05) * 0.4
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
    s.gravity = [0, -20, 0];
    return s;
}

const OL_STATIC = 0;
const OL_MOVING = 1;

// small convex hull (icosahedron-ish, ~12 verts) shared across hull bodies
function hullPositions(radius: number): number[] {
    const t = (1 + Math.sqrt(5)) / 2;
    const raw = [
        [-1, t, 0],
        [1, t, 0],
        [-1, -t, 0],
        [1, -t, 0],
        [0, -1, t],
        [0, 1, t],
        [0, -1, -t],
        [0, 1, -t],
        [t, 0, -1],
        [t, 0, 1],
        [-t, 0, -1],
        [-t, 0, 1],
    ];
    const norm = Math.sqrt(1 + t * t);
    const out: number[] = [];
    for (const [x, y, z] of raw) out.push((x / norm) * radius, (y / norm) * radius, (z / norm) * radius);
    return out;
}

// four mixed shape types
const shapes: Shape[] = [
    box.create({ halfExtents: [0.35, 0.35, 0.35], convexRadius: 0.04 }),
    sphere.create({ radius: 0.4 }),
    capsule.create({ halfHeightOfCylinder: 0.3, radius: 0.25 }),
    convexHull.create({ positions: hullPositions(0.4), convexRadius: 0.04 }),
];

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

type BodyInfo = {
    body: RigidBody;
    x: number;
    z: number;
};

function populate(world: World, rng: () => number): BodyInfo[] {
    rigidBody.create(world, {
        shape: terrainShape,
        objectLayer: OL_STATIC,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        friction: 0.6,
        restitution: 0.1,
    });

    const bodies: BodyInfo[] = [];
    for (let i = 0; i < NUMBER_OF_BODIES; i++) {
        const x = (rng() * 2 - 1) * SCATTER_HALF;
        const z = (rng() * 2 - 1) * SCATTER_HALF;
        const y = terrainHeight(x, z) + SPAWN_HEIGHT + rng() * 2;
        const body = rigidBody.create(world, {
            shape: shapes[i % shapes.length],
            objectLayer: OL_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [x, y, z],
            friction: 0.5,
            restitution: 0.1,
            mass: 1,
        });
        bodies.push({ body, x, z });
    }
    return bodies;
}

const zero: [number, number, number] = [0, 0, 0];

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` re-drops
 * one body every CHURN_INTERVAL steps (round-robin, seeded scatter) then advances
 * the sim. Warmup uses WARMUP_SEED; the op rng is re-seeded with the FIXED
 * OP_SEED at each op boundary (labs and profiling both use OP_SEED), and the
 * round-robin cursor resets at each phase boundary so repeated ops replay the
 * identical churn. World-state drift is fine.
 */
function build(): { scenario: Scenario; bodies: BodyInfo[] } {
    const world = createWorld(makeWorldSettings());
    const bodies = populate(world, makeRng(RNG_SEED));
    const warmupRng = makeRng(WARMUP_SEED);
    let opRng = makeRng(OP_SEED);
    let cursor = 0;

    const churnStep = (rng: () => number, localIndex: number): void => {
        if (localIndex % CHURN_INTERVAL === 0) {
            const info = bodies[cursor % bodies.length];
            cursor++;
            // re-drop at a fresh random scatter position, waking the body
            const x = (rng() * 2 - 1) * SCATTER_HALF;
            const z = (rng() * 2 - 1) * SCATTER_HALF;
            const y = terrainHeight(x, z) + SPAWN_HEIGHT + rng() * 2;
            rigidBody.setLinearVelocity(world, info.body, zero);
            rigidBody.setAngularVelocity(world, info.body, zero);
            rigidBody.setPosition(world, info.body, [x, y, z], true);
        }
        updateWorld(world, undefined, TIME_STEP);
    };

    const scenario: Scenario = {
        world,
        warmupSteps: STEADY_WARMUP_STEPS,
        stepOnce(stepIndex: number): void {
            if (stepIndex < STEADY_WARMUP_STEPS) {
                if (stepIndex === 0) cursor = 0;
                churnStep(warmupRng, stepIndex);
            } else {
                const k = (stepIndex - STEADY_WARMUP_STEPS) % STEPS_PER_OP;
                if (k === 0) {
                    opRng = makeRng(OP_SEED);
                    cursor = 0;
                }
                churnStep(opRng, k);
            }
        },
    };
    return { scenario, bodies };
}

export function createScenario(): Scenario {
    return build().scenario;
}

group('mesh-field', () => {
    bench('mesh-field', function* () {
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
    const { scenario: s, bodies } = build();
    const t0 = performance.now();
    for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
    const t1 = performance.now();
    for (let op = 0; op < 6; op++) {
        const a = performance.now();
        for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + op * STEPS_PER_OP + i);
        const opMs = performance.now() - a;
        let asleep = 0;
        let below = 0;
        for (const { body } of bodies) {
            if (body.sleeping) asleep++;
            if (body.position[1] < -20) below++;
        }
        console.error(
            `[mesh-field] op${op} ${opMs.toFixed(0)}ms  ` +
                `asleep ${asleep}/${bodies.length} (${((100 * asleep) / bodies.length).toFixed(0)}%)  belowVoid ${below}`,
        );
    }
    console.error(`[mesh-field] warmup ${(t1 - t0).toFixed(0)}ms`);
}
