// Macro perf benchmark — body-churn.
//
// Spawn/despawn lifecycle. A static ground; every step a seeded emitter spawns
// 2-3 small dynamic boxes (deterministic positions/velocities) and every body
// older than LIFETIME steps is removed via rigidBody.remove — holding a steady
// population of ~180 boxes in flight, bouncing on the ground. Warmup runs until
// the population is steady; the measured op is 120 steps. Unlike the other
// scenarios (which keep a fixed body set), this stresses body create/remove,
// the pair-purge cascades a removal triggers, freelist reuse, and the discovery
// pressure of fresh bodies entering the broadphase every step.
//
// Deterministic mulberry32 RNG. The measured op re-seeds with a FIXED seed so
// every op runs the identical per-step spawn workload (world-state drift between
// ops is fine — see the cube-heap note). Spawn *count* is derived from an
// internal monotonic tick (not the rng) so create/remove stay balanced and the
// population is stable regardless of the reseed.
//
// `runForProfiling` is the entry the cpu-prof rig invokes via `run-scenario.ts`.
// See settle-sleep.bench.ts for the `createScenario` per-step convention.

import { bench, group } from '@pmndrs/labs';

import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    type RigidBody,
    registerAll,
    rigidBody,
    type Shape,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const LIFETIME = 75; // steps a body lives before removal
const STEPS_PER_OP = 120; // 2s of sim at 60Hz
const TIME_STEP = 1 / 60;
// spawn 2 or 3 per step (avg 2.5) → steady population ≈ 2.5 * LIFETIME ≈ 187
const SPAWN_HEIGHT = 8;
const EMIT_AREA = 1.5;

const RNG_SEED = 0xc0ffee;
const OP_SEED = 0x5eed11fe;
const WARMUP_SEED = RNG_SEED ^ 0xdeadbeef;
// LIFETIME steps to fill the pipeline + margin for the field to reach a bouncing
// steady state before timing starts.
const STEADY_WARMUP_STEPS = 150;

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
    s.gravity = [0, -30, 0];
    return s;
}

const OL_MOVING = 0;
const OL_STATIC = 1;

const boxShape: Shape = box.create({ halfExtents: [0.2, 0.2, 0.2], convexRadius: 0.02 });

type Live = { body: RigidBody; birth: number };

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` runs one
 * step: retire aged bodies (rigidBody.remove), then emit fresh ones. `stepIndex`
 * is the global step counter; steps below warmupSteps are warmup. The op rng is
 * re-seeded at each op boundary so repeated labs ops are identical; body age is
 * tracked with an internal monotonic tick so removal is independent of the
 * reseed and the population stays steady.
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());
    rigidBody.create(world, {
        motionType: MotionType.STATIC,
        objectLayer: OL_STATIC,
        shape: box.create({ halfExtents: [30, 1, 30] }),
        position: [0, -1, 0],
        friction: 0.5,
        restitution: 0.2,
    });

    const live: Live[] = [];
    let tick = 0;
    const warmupRng = makeRng(WARMUP_SEED);
    let opRng = makeRng(OP_SEED);

    const spawn = (rng: () => number): void => {
        const x = (rng() * 2 - 1) * EMIT_AREA;
        const z = (rng() * 2 - 1) * EMIT_AREA;
        const body = rigidBody.create(world, {
            motionType: MotionType.DYNAMIC,
            objectLayer: OL_MOVING,
            shape: boxShape,
            position: [x, SPAWN_HEIGHT, z],
            mass: 1,
            friction: 0.4,
            restitution: 0.4,
        });
        rigidBody.setLinearVelocity(world, body, [(rng() * 2 - 1) * 4, -rng() * 2, (rng() * 2 - 1) * 4]);
        live.push({ body, birth: tick });
    };

    const churn = (rng: () => number): void => {
        // retire aged bodies (swap-remove from the live list)
        for (let i = live.length - 1; i >= 0; i--) {
            if (tick - live[i].birth >= LIFETIME) {
                rigidBody.remove(world, live[i].body);
                live[i] = live[live.length - 1];
                live.pop();
            }
        }
        // emit 2 or 3 (avg 2.5) — count from tick so it's independent of the rng
        const count = 2 + (tick & 1);
        for (let n = 0; n < count; n++) spawn(rng);
        updateWorld(world, undefined, TIME_STEP);
        tick++;
    };

    return {
        world,
        warmupSteps: STEADY_WARMUP_STEPS,
        stepOnce(stepIndex: number): void {
            if (stepIndex < STEADY_WARMUP_STEPS) {
                churn(warmupRng);
            } else {
                const k = (stepIndex - STEADY_WARMUP_STEPS) % STEPS_PER_OP;
                if (k === 0) opRng = makeRng(OP_SEED);
                churn(opRng);
            }
        },
    };
}

group('body-churn', () => {
    bench('body-churn', function* () {
        const s = createScenario();
        for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
        yield () => {
            for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
        };
    }).gc('inner');
});

/**
 * Canonical run for cpu-prof attribution (invoked by `run-scenario.ts`).
 * Pre-warm to steady population, then run 5×STEPS_PER_OP steady-state steps.
 */
export function runForProfiling(): void {
    const s = createScenario();
    for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
    for (let i = 0; i < 5 * STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
}

// TEMP sanity
export function __sanity(): void {
    const s = createScenario();
    const { world } = s;
    const liveContacts = () => world.contacts.contacts.length - world.contacts.contactsFreeIndices.length;
    const bodyCount = () => world.bodies.pool.length - world.bodies.freeIndices.length;
    const a = performance.now();
    for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
    const w = performance.now();
    for (let op = 0; op < 5; op++) {
        const b = performance.now();
        for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + op * STEPS_PER_OP + i);
        console.error(
            `[body-churn] op${op} ${(performance.now() - b).toFixed(0)}ms  ` +
                `active ${world.bodies.activeBodyCount}  liveBodies ${bodyCount()}  liveContacts ${liveContacts()}  ` +
                `freeIdx ${world.bodies.freeIndices.length}`,
        );
    }
    console.error(`[body-churn] warmup ${(w - a).toFixed(0)}ms`);
}
