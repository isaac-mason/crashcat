// Macro perf benchmark — pyramid.
//
// 385 dynamic boxes stacked into a height-10 pyramid on a static plane.
// Sleeping is disabled so every step does a full contact solve even after
// the stack settles — that gives labs's adaptive sampling a stationary
// signal without any churn. Complements cube-heap: where cube-heap measures
// broadphase under motion, pyramid measures the solver + contact-cache hot
// path under dense persistent stacking.
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
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const PYRAMID_HEIGHT = 10;
const BOX_SIZE = 2.0;
const BOX_SEP = 0.5;
const STEPS_PER_OP = 120; // 2s of sim at 60Hz — pyramid is heavier than cube-heap
const TIME_STEP = 1 / 60;
const STEADY_WARMUP_STEPS = 300; // 5s — lets the stack fully compress before timing

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

const half = BOX_SIZE * 0.5;
const cubeShape = box.create({ halfExtents: [half, half, half], convexRadius: 0 });

function populate(world: World): void {
    rigidBody.create(world, {
        motionType: MotionType.STATIC,
        objectLayer: 1, // OL_STATIC
        shape: box.create({ halfExtents: [50, 1, 50] }),
        position: [0, -1, 0],
    });

    for (let i = 0; i < PYRAMID_HEIGHT; i++) {
        for (let j = Math.floor(i / 2); j < PYRAMID_HEIGHT - Math.ceil(i / 2); j++) {
            for (let k = Math.floor(i / 2); k < PYRAMID_HEIGHT - Math.ceil(i / 2); k++) {
                const x = -PYRAMID_HEIGHT + BOX_SIZE * j + (i & 1 ? half : 0);
                const y = 1.0 + (BOX_SIZE + BOX_SEP) * i;
                const z = -PYRAMID_HEIGHT + BOX_SIZE * k + (i & 1 ? half : 0);
                rigidBody.create(world, {
                    motionType: MotionType.DYNAMIC,
                    objectLayer: 0, // OL_MOVING
                    shape: cubeShape,
                    position: [x, y, z],
                    mass: 1,
                    allowSleeping: false,
                });
            }
        }
    }
}

function step(world: World, steps: number): void {
    for (let i = 0; i < steps; i++) {
        updateWorld(world, undefined, TIME_STEP);
    }
}

group('pyramid', () => {
    bench('pyramid', function* () {
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
 * collects ~10s of in-scenario CPU.
 */
export function runForProfiling(): void {
    const world = createWorld(makeWorldSettings());
    populate(world);
    step(world, STEADY_WARMUP_STEPS);
    for (let i = 0; i < 5; i++) {
        step(world, STEPS_PER_OP);
    }
}
