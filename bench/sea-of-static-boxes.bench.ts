// Macro perf benchmark — sea of static boxes.
//
// Port of PEEL's "SeaOfStaticBoxes" (CATEGORY_STATIC_SCENE): a 64×64 field of
// randomly-sized STATIC boxes with a 48×48 grid of downward closest-hit rays cast
// over it every step (the grid drifts in a slow circle so the query set keeps
// changing). Exercises the broadphase castRay tree traversal at scale — a 4096-leaf
// tree, the workload no other bench covers (raycasts.bench uses a ~41-leaf tree, so
// its cost is castRayVsTriangleMesh, not the dbvt walk). This is the scenario where
// crashcat's incremental-only btDbvt goes degenerate (height ~127) vs a balanced
// rebuild (height ~16).
// `runForProfiling` is the entry the cpu-prof rig invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';

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
    registerAll,
    rigidBody,
    type Shape,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const GRID = 64; // boxes per side → 4096 static boxes
const RAYS = 48; // rays per side → 2304 closest-hit rays / step
const AMPLITUDE = 40;
const RAY_TOP_Y = 25;
const RAY_MAX_DIST = 32;
const STEPS_PER_OP = 100;
const TIME_STEP = 1 / 60;
const STEADY_WARMUP_STEPS = 100;

// Deterministic pseudo-random in [0, 1] — matches the crashcat-benchmarks scenario
// so the box field (and therefore the profile) reproduces exactly.
function rand01(seed: number): number {
    const s = Math.sin(seed * 91.7) * 43758.5453;
    return s - Math.floor(s);
}

function makeWorldSettings(): WorldSettings {
    const s = createWorldSettings();
    const BPH_MOVING = addBroadphaseLayer(s);
    const BPH_NOT_MOVING = addBroadphaseLayer(s);
    const OL_MOVING = addObjectLayer(s, BPH_MOVING);
    const OL_NOT_MOVING = addObjectLayer(s, BPH_NOT_MOVING);
    enableCollision(s, OL_MOVING, OL_MOVING);
    enableCollision(s, OL_MOVING, OL_NOT_MOVING);
    s.gravity = [0, -9.81, 0];
    return s;
}

// The crashcat-benchmarks adapter places every body (including static) in the moving
// object layer; replicate that so this bench matches the captured sea-crashcat profile.
const OL_MOVING = 0;

function buildField(world: World): void {
    let seed = 1;
    for (let y = 0; y < GRID; y++) {
        const coeffY = 2 * (y / (GRID - 1) - 0.5);
        for (let x = 0; x < GRID; x++) {
            const coeffX = 2 * (x / (GRID - 1) - 0.5);
            const hx = 1 + rand01(seed++);
            const hy = 1 + rand01(seed++);
            const hz = 1 + rand01(seed++);
            const shape: Shape = box.create({ halfExtents: [hx, hy, hz], convexRadius: 0.05 });
            rigidBody.create(world, {
                shape,
                objectLayer: OL_MOVING,
                motionType: MotionType.STATIC,
                position: [coeffX * AMPLITUDE + rand01(seed++) * 2 - 1, 0, coeffY * AMPLITUDE + rand01(seed++) * 2 - 1],
                friction: 0.5,
            });
        }
    }
}

let _hitCount = 0;

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` casts the
 * drifting 48×48 downward ray grid over the field, then advances the sim (which is
 * inert — all bodies are static — but keeps the broadphase `optimize` path on the
 * hot loop exactly as the real scenario runs it). `phase` drives the slow circular
 * drift of the grid from the local step index so warmup and each op replay the same
 * query motion.
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());
    buildField(world);

    const queryFilter = filter.create(world.settings.layers);
    const collector = createClosestCastRayCollector();
    const settings = createDefaultCastRaySettings();
    const origin: [number, number, number] = [0, RAY_TOP_Y, 0];
    const direction: [number, number, number] = [0, -1, 0];

    const stepRays = (localIndex: number): void => {
        const phase = localIndex * TIME_STEP * 0.3;
        const span = AMPLITUDE * 2;
        const ox = Math.cos(phase) * 4;
        const oz = Math.sin(phase) * 4;
        for (let j = 0; j < RAYS; j++) {
            const z = (RAYS === 1 ? 0 : (j / (RAYS - 1) - 0.5) * span) + oz;
            for (let i = 0; i < RAYS; i++) {
                const x = (RAYS === 1 ? 0 : (i / (RAYS - 1) - 0.5) * span) + ox;
                origin[0] = x;
                origin[2] = z;
                collector.reset();
                castRay(world, collector, settings, origin, direction, RAY_MAX_DIST, queryFilter);
                if (collector.hit.status === CastRayStatus.COLLIDING) _hitCount++;
            }
        }
        updateWorld(world, undefined, TIME_STEP);
    };

    return {
        world,
        warmupSteps: STEADY_WARMUP_STEPS,
        stepOnce(stepIndex: number): void {
            stepRays(stepIndex);
        },
    };
}

group('sea-of-static-boxes', () => {
    bench('sea-of-static-boxes', function* () {
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
    for (let i = 0; i < 60; i++) s.stepOnce(i);
    return { hitFraction: _hitCount / (60 * RAYS * RAYS) };
}
