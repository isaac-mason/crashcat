// Macro perf benchmark — cube-heap.
//
// 200 dynamic boxes drop into a static plane, round-robin churn re-spawns one
// box per frame to keep contacts hot. Deterministic mulberry32 RNG so labs's
// adaptive sampling (and the cpu-prof rig in profile.mjs) sees a stationary
// signal. `runForProfiling` is the entry the cpu-prof rig invokes via
// `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';

import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    plane,
    type RigidBody,
    registerAll,
    rigidBody,
    type Shape,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const NUMBER_OF_CUBES = 200;
const SPAWN_HEIGHT = 10;
const SPAWN_AREA = 2.5;
const STEPS_PER_OP = 300; // 5s of sim at 60Hz
const TIME_STEP = 1 / 60;
const RNG_SEED = 0xc0ffee;
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

function makeWorldSettings(): WorldSettings {
    const s = createWorldSettings();
    const BPH_MOVING = addBroadphaseLayer(s);
    const BPH_STATIC = addBroadphaseLayer(s);
    const OL_MOVING = addObjectLayer(s, BPH_MOVING);
    const OL_STATIC = addObjectLayer(s, BPH_STATIC);
    enableCollision(s, OL_MOVING, OL_MOVING);
    enableCollision(s, OL_MOVING, OL_STATIC);
    s.gravity = [0, -50, 0];
    return s;
}

const cubeShape: Shape = box.create({ halfExtents: [0.25, 0.25, 0.25], convexRadius: 0.05 });

function populate(world: World, rng: () => number): RigidBody[] {
    rigidBody.create(world, {
        shape: plane.create({ plane: { normal: [0, 1, 0], constant: 0 }, halfExtent: 50 }),
        objectLayer: 1, // OL_STATIC
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        restitution: 0,
        friction: 0.5,
    });

    const cubes: RigidBody[] = [];
    for (let i = 0; i < NUMBER_OF_CUBES; i++) {
        const x = (rng() * 2 - 1) * SPAWN_AREA;
        const y = rng() * SPAWN_HEIGHT;
        const z = (rng() * 2 - 1) * SPAWN_AREA;
        cubes.push(
            rigidBody.create(world, {
                shape: cubeShape,
                objectLayer: 0, // OL_MOVING
                motionType: MotionType.DYNAMIC,
                position: [x, y, z],
                restitution: 0,
                friction: 0.5,
                mass: 1,
            }),
        );
    }
    return cubes;
}

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` re-spawns
 * one cube per frame (the churn) then advances the sim. Warmup uses a distinct
 * seed; the op rng is re-seeded at each op boundary with `RNG_SEED + op`. A labs
 * op always replays the same window [warmupSteps, warmupSteps+STEPS_PER_OP), so
 * op = 0 → the fixed `RNG_SEED` workload (a rotating seed made contact-count
 * variance masquerade as timing noise); the profiling run advances continuously,
 * so its ops rotate `RNG_SEED + i`. World-state drift between ops is fine.
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());
    const cubes = populate(world, makeRng(RNG_SEED));
    const warmupRng = makeRng(RNG_SEED ^ 0xdeadbeef);
    let opRng = makeRng(RNG_SEED);
    const zero: [number, number, number] = [0, 0, 0];

    const churn = (rng: () => number): void => {
        const cube = cubes[(rng() * cubes.length) | 0];
        rigidBody.setPosition(world, cube, [0, rng() * SPAWN_HEIGHT, 0], true);
        rigidBody.setLinearVelocity(world, cube, zero);
        rigidBody.setAngularVelocity(world, cube, zero);
        updateWorld(world, undefined, TIME_STEP);
    };

    return {
        world,
        warmupSteps: STEADY_WARMUP_STEPS,
        stepOnce(stepIndex: number): void {
            if (stepIndex < STEADY_WARMUP_STEPS) {
                churn(warmupRng);
            } else {
                const g = stepIndex - STEADY_WARMUP_STEPS;
                if (g % STEPS_PER_OP === 0) opRng = makeRng(RNG_SEED + Math.floor(g / STEPS_PER_OP));
                churn(opRng);
            }
        },
    };
}

group('cube-heap', () => {
    bench('cube-heap', function* () {
        const s = createScenario();
        for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
        yield () => {
            for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
        };
    }).gc('inner');
});

/**
 * Canonical run for cpu-prof attribution (invoked by `run-scenario.ts`).
 * Pre-settle, then run 5×STEPS_PER_OP steady-state steps so the sampler
 * collects ~5s of in-scenario CPU.
 */
export function runForProfiling(): void {
    const s = createScenario();
    for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
    for (let i = 0; i < 5 * STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
}
