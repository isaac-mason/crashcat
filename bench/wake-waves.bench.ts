// Macro perf benchmark — wake-waves.
//
// Periodic sleep/wake at steady state. A settle-sleep-style grid of small box
// stacks settles and goes to sleep during warmup, then a KINEMATIC sphere sweeps
// through the grid lane by lane at constant velocity (a kinematic body with
// nonzero velocity never sleeps). It knocks each lane's stacks awake as it
// arrives; they scatter and re-sleep behind it once it has passed. When the
// sweeper exits the far side of the grid it is teleported (rigidBody.setPosition)
// back to the start of the next lane, and the cycle repeats. Where settle-sleep
// measures the per-step cost of a world *at rest*, this measures the sleep-
// transition machinery itself — the contact-chain destroy/re-add, island
// rebuild, and broadphase discovery paid every time a resting stack is woken and
// then put back to sleep — exercised repeatedly at steady state. Op = 240 steps.
// Fully deterministic (no rng).
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
    sphere,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const GRID = 8; // GRID×GRID stacks
const STACK_HEIGHT = 4;
const BOX_SIZE = 0.6;
// wide spacing (settle-sleep style) so neighbouring stacks never interact at
// rest: a stack the sweeper topples stays in its own cell and re-sleeps behind
// the sweeper instead of avalanching into the next lane.
const STACK_SPACING = 3.5;
const STEPS_PER_OP = 240; // 4s of sim at 60Hz
const TIME_STEP = 1 / 60;
// 10s: settle + timeBeforeSleep with margin so timing starts fully asleep.
const STEADY_WARMUP_STEPS = 600;

const SWEEP_SPEED = 7; // m/s — gentle enough that toppled boxes resettle near their lane and re-sleep
const SWEEP_RADIUS = 1.1;
const SWEEP_Y = 1.1; // sphere centre height — clips the stacks and topples them without launching

function makeWorldSettings(): WorldSettings {
    const s = createWorldSettings();
    const BPH_MOVING = addBroadphaseLayer(s);
    const BPH_STATIC = addBroadphaseLayer(s);
    const OL_MOVING = addObjectLayer(s, BPH_MOVING);
    const OL_STATIC = addObjectLayer(s, BPH_STATIC);
    enableCollision(s, OL_MOVING, OL_MOVING);
    enableCollision(s, OL_MOVING, OL_STATIC);
    return s;
}

const OL_MOVING = 0;
const OL_STATIC = 1;

const half = BOX_SIZE * 0.5;
const cubeShape = box.create({ halfExtents: [half, half, half], convexRadius: 0.02 });
const sweepShape = sphere.create({ radius: SWEEP_RADIUS });

const ORIGIN = -((GRID - 1) * STACK_SPACING) / 2;
const X_START = ORIGIN - STACK_SPACING; // just outside the grid on the -x side
const X_END = -ORIGIN + STACK_SPACING; // just outside on the +x side

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. During warmup the
 * sweeper is parked high above the grid so the stacks settle and sleep. On the
 * first post-warmup step it is set into motion at the start of lane 0, and from
 * then on it advances continuously through the lanes (wrapping z), teleporting
 * back to the -x side on exit. The sweep state persists across ops so the world
 * reaches a steady wake/sleep turnover (world-state drift between ops is fine —
 * see the cube-heap note); no rng, so the whole run is deterministic.
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());

    const groundExtent = GRID * STACK_SPACING;
    rigidBody.create(world, {
        motionType: MotionType.STATIC,
        objectLayer: OL_STATIC,
        shape: box.create({ halfExtents: [groundExtent, 1, groundExtent] }),
        position: [0, -1, 0],
        friction: 0.5,
        restitution: 0,
    });

    for (let i = 0; i < GRID; i++) {
        for (let j = 0; j < GRID; j++) {
            for (let k = 0; k < STACK_HEIGHT; k++) {
                rigidBody.create(world, {
                    motionType: MotionType.DYNAMIC,
                    objectLayer: OL_MOVING,
                    shape: cubeShape,
                    position: [ORIGIN + i * STACK_SPACING, half + 0.05 + k * (BOX_SIZE + 0.05), ORIGIN + j * STACK_SPACING],
                    mass: 1,
                    friction: 0.5,
                    restitution: 0,
                });
            }
        }
    }

    // kinematic sweeper — parked high above the grid during warmup
    const sweeper: RigidBody = rigidBody.create(world, {
        motionType: MotionType.KINEMATIC,
        objectLayer: OL_MOVING,
        shape: sweepShape,
        position: [0, 40, 0],
    });

    let lane = 0;
    let started = false;

    const launchLane = (): void => {
        rigidBody.setPosition(world, sweeper, [X_START, SWEEP_Y, ORIGIN + lane * STACK_SPACING], true);
        rigidBody.setLinearVelocity(world, sweeper, [SWEEP_SPEED, 0, 0]);
    };

    return {
        world,
        warmupSteps: STEADY_WARMUP_STEPS,
        stepOnce(stepIndex: number): void {
            if (stepIndex >= STEADY_WARMUP_STEPS) {
                if (!started) {
                    started = true;
                    launchLane();
                } else if (sweeper.position[0] > X_END) {
                    lane = (lane + 1) % GRID;
                    launchLane();
                }
            }
            updateWorld(world, undefined, TIME_STEP);
        },
    };
}

group('wake-waves', () => {
    bench('wake-waves', function* () {
        const s = createScenario();
        for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
        yield () => {
            for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
        };
    }).gc('inner');
});

/**
 * Canonical run for cpu-prof attribution (invoked by `run-scenario.ts`).
 * Pre-settle, then run 5×STEPS_PER_OP steady-state sweep steps.
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
    const a = performance.now();
    for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
    const w = performance.now();
    console.error(`[wake-waves] warmup ${(w - a).toFixed(0)}ms  activeAfterWarmup ${world.bodies.activeBodyCount}`);
    // continuous ops: sample activeBodyCount min/max within each op to see the
    // repeating wave and confirm it reaches a steady turnover (not full-grid wake)
    for (let op = 0; op < 8; op++) {
        const b = performance.now();
        let lo = Infinity;
        let hi = 0;
        for (let i = 0; i < STEPS_PER_OP; i++) {
            s.stepOnce(s.warmupSteps + op * STEPS_PER_OP + i);
            const ac = world.bodies.activeBodyCount;
            if (ac < lo) lo = ac;
            if (ac > hi) hi = ac;
        }
        console.error(
            `[wake-waves] op${op} ${(performance.now() - b).toFixed(0)}ms  active ${lo}..${hi}  ` +
                `endContacts ${liveContacts()}`,
        );
    }
}
