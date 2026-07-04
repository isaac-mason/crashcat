// Macro perf benchmark — settle-sleep.
//
// 864 dynamic boxes in a 12×12 grid of 6-high stacks settle onto a static
// plane and go to sleep during warmup, plus one never-sleeping sphere resting
// on the plane away from the stacks so every step still runs a real (tiny)
// active workload. Steady state is a mostly-sleeping world — the shape of a
// game scene at rest — so this measures the per-step overhead the engine pays
// for bodies and contacts that are doing nothing: broadphase pair sweep,
// contact bookkeeping sweeps, and any other O(world) per-frame work.
//
// `runForProfiling` is the entry the cpu-prof rig invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';

import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAll,
    rigidBody,
    sphere,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const GRID = 12; // 12×12 stacks
const STACK_HEIGHT = 6;
const BOX_SIZE = 1.0;
const STACK_SPACING = 4.0;
const STEPS_PER_OP = 240; // 4s of sim at 60Hz — sleeping steps are cheap
const TIME_STEP = 1 / 60;
// 10s: settle (~1s of falling/compression) + timeBeforeSleep (0.5s) with margin,
// so timing starts with every stack asleep and only the sphere active
const STEADY_WARMUP_STEPS = 600;

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

const half = BOX_SIZE * 0.5;
const cubeShape = box.create({ halfExtents: [half, half, half], convexRadius: 0.02 });

function populate(world: World): void {
    const groundExtent = GRID * STACK_SPACING;
    rigidBody.create(world, {
        motionType: MotionType.STATIC,
        objectLayer: 1, // OL_STATIC
        shape: box.create({ halfExtents: [groundExtent, 1, groundExtent] }),
        position: [0, -1, 0],
    });

    // grid of stacks, spaced so neighbouring stacks never interact
    const origin = -((GRID - 1) * STACK_SPACING) / 2;
    for (let i = 0; i < GRID; i++) {
        for (let j = 0; j < GRID; j++) {
            for (let k = 0; k < STACK_HEIGHT; k++) {
                rigidBody.create(world, {
                    motionType: MotionType.DYNAMIC,
                    objectLayer: 0, // OL_MOVING
                    shape: cubeShape,
                    position: [
                        origin + i * STACK_SPACING,
                        half + 0.1 + k * (BOX_SIZE + 0.1),
                        origin + j * STACK_SPACING,
                    ],
                    mass: 1,
                });
            }
        }
    }

    // the one persistently active body: never sleeps, rests on the plane
    // outside the stack grid with a single persistent contact
    rigidBody.create(world, {
        motionType: MotionType.DYNAMIC,
        objectLayer: 0, // OL_MOVING
        shape: sphere.create({ radius: 0.5 }),
        position: [origin - STACK_SPACING, 0.6, origin - STACK_SPACING],
        mass: 1,
        allowSleeping: false,
    });
}

function step(world: World, steps: number): void {
    for (let i = 0; i < steps; i++) {
        updateWorld(world, undefined, TIME_STEP);
    }
}

group('settle-sleep', () => {
    bench('settle-sleep', function* () {
        const world = createWorld(makeWorldSettings());
        populate(world);
        step(world, STEADY_WARMUP_STEPS);
        yield () => {
            step(world, STEPS_PER_OP);
        };
    }).gc('inner');
});

/**
 * Canonical run for cpu-prof attribution (invoked by `run-scenario.ts`).
 * Pre-settle, then run 5×STEPS_PER_OP steady-state steps so the sampler
 * collects in-scenario CPU.
 */
export function runForProfiling(): void {
    const world = createWorld(makeWorldSettings());
    populate(world);
    step(world, STEADY_WARMUP_STEPS);
    for (let i = 0; i < 5; i++) {
        step(world, STEPS_PER_OP);
    }
}
