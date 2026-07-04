// Macro perf benchmark — raycasts.
//
// The one scenario shaped like query-heavy game code: a sensor rig fires ~192 closest-hit
// raycasts per step (AI line-of-sight / wheel rays / lasers) against a 64×64-quad terrain
// mesh while ~40 dynamic spheres bounce around on it. Exercises the broadphase castRay
// traversal and castRayVsTriangleMesh — paths no other scenario covers.
// `runForProfiling` is the entry the cpu-prof rig invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';

import {
    addBroadphaseLayer,
    addObjectLayer,
    CastRayStatus,
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
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

const NUMBER_OF_SPHERES = 40;
const RAYS_PER_STEP = 192;
const RAY_LENGTH = 40;
const STEPS_PER_OP = 300; // 5s of sim at 60Hz
const TIME_STEP = 1 / 60;
const RNG_SEED = 0xc0ffee;
const STEADY_WARMUP_STEPS = 300;

const TERRAIN_QUADS = 64;
const TERRAIN_EXTENT = 50;
const TERRAIN_CELL = TERRAIN_EXTENT / TERRAIN_QUADS;
const TERRAIN_HALF = TERRAIN_EXTENT / 2;

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
    s.gravity = [0, -20, 0];
    return s;
}

const OL_STATIC = 0;
const OL_MOVING = 1;

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
const sphereShape: Shape = sphere.create({ radius: 0.3 });

function populate(world: World, rng: () => number): RigidBody[] {
    rigidBody.create(world, {
        shape: terrainShape,
        objectLayer: OL_STATIC,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        friction: 0.6,
    });

    const spheres: RigidBody[] = [];
    for (let i = 0; i < NUMBER_OF_SPHERES; i++) {
        const x = (rng() * 2 - 1) * (TERRAIN_HALF - 4);
        const z = (rng() * 2 - 1) * (TERRAIN_HALF - 4);
        spheres.push(
            rigidBody.create(world, {
                shape: sphereShape,
                objectLayer: OL_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [x, 6, z],
                restitution: 0.7,
                friction: 0.3,
                mass: 1,
            }),
        );
    }
    return spheres;
}

// hit counter exposed so runs can sanity-check that rays actually hit things
let _hitCount = 0;

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` fires the
 * sensor rig's fan of raycasts, runs the once-a-second sphere churn, then
 * advances the sim. The sensor rig's position is driven by the per-phase local
 * step index (so warmup and each op replay the same circling motion). Seeding
 * matches cube-heap: warmup uses a distinct seed; the op rng is re-seeded with
 * `RNG_SEED + op` at each op boundary, so a labs op (op = 0) runs the fixed-seed
 * workload while the profiling run's ops rotate. World-state drift is fine.
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());
    const spheres = populate(world, makeRng(RNG_SEED));
    const warmupRng = makeRng(RNG_SEED ^ 0xdeadbeef);
    let opRng = makeRng(RNG_SEED);

    const queryFilter = filter.create(world.settings.layers);
    const collector = createClosestCastRayCollector();
    const settings = createDefaultCastRaySettings();
    const origin: [number, number, number] = [0, 0, 0];
    const direction: [number, number, number] = [0, 0, 0];

    const stepRays = (rng: () => number, localIndex: number): void => {
        // sensor rig circles above the terrain
        const t = localIndex * TIME_STEP;
        const rigX = Math.cos(t * 0.7) * 12;
        const rigZ = Math.sin(t * 0.7) * 12;
        const rigY = terrainHeight(rigX, rigZ) + 6;

        for (let r = 0; r < RAYS_PER_STEP; r++) {
            // fan of rays: mostly downward-forward at varied azimuth, some flat LOS rays
            const azimuth = (r / RAYS_PER_STEP) * Math.PI * 2 + t * 0.3;
            const downward = 0.25 + (0.7 * ((r * 2654435761) % 97)) / 97;
            origin[0] = rigX;
            origin[1] = rigY;
            origin[2] = rigZ;
            direction[0] = Math.cos(azimuth) * (1 - downward);
            direction[1] = -downward;
            direction[2] = Math.sin(azimuth) * (1 - downward);
            const invLen = 1 / Math.hypot(direction[0], direction[1], direction[2]);
            direction[0] *= invLen;
            direction[1] *= invLen;
            direction[2] *= invLen;

            collector.reset();
            castRay(world, collector, settings, origin, direction, RAY_LENGTH, queryFilter);
            if (collector.hit.status === CastRayStatus.COLLIDING) _hitCount++;
        }

        // churn: relaunch one sphere per second to keep dynamics awake
        if (localIndex % 60 === 0) {
            const s = spheres[(rng() * spheres.length) | 0];
            rigidBody.setPosition(world, s, [(rng() * 2 - 1) * 10, 6, (rng() * 2 - 1) * 10], true);
            rigidBody.setLinearVelocity(world, s, [(rng() * 2 - 1) * 5, 0, (rng() * 2 - 1) * 5]);
        }

        updateWorld(world, undefined, TIME_STEP);
    };

    return {
        world,
        warmupSteps: STEADY_WARMUP_STEPS,
        stepOnce(stepIndex: number): void {
            if (stepIndex < STEADY_WARMUP_STEPS) {
                stepRays(warmupRng, stepIndex);
            } else {
                const g = stepIndex - STEADY_WARMUP_STEPS;
                const k = g % STEPS_PER_OP;
                if (k === 0) opRng = makeRng(RNG_SEED + Math.floor(g / STEPS_PER_OP));
                stepRays(opRng, k);
            }
        },
    };
}

group('raycasts', () => {
    bench('raycasts', function* () {
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
    _hitCount = 0;
    for (let i = 0; i < 5 * STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
}

/** sanity helper: fraction of rays that hit something during a probe run (inert unless called) */
export function __sanity(): { hitFraction: number } {
    const s = createScenario();
    _hitCount = 0;
    // 120 op-phase steps from a fresh world (op 0 → RNG_SEED, no warmup)
    for (let i = 0; i < 120; i++) s.stepOnce(s.warmupSteps + i);
    return { hitFraction: _hitCount / (120 * RAYS_PER_STEP) };
}
