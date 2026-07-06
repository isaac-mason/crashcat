// Macro perf benchmark — collapse.
//
// Transient structure collapse. A wall of 160 dynamic boxes (10 wide × 8 high ×
// 2 deep) stands on a static ground, and a heavy dense sphere is launched into
// the wall's base at t=0. There is NO settling warmup: every op builds a FRESH
// world and runs 360 steps (6s @ 60Hz) through the whole transient — topple,
// scatter, and most bodies falling asleep by the end. This measures contact
// churn + island merging + mass sleep transitions during a transient, unlike
// every existing steady-state scenario. Fully deterministic (fixed geometry +
// fixed launch velocity, no rng).
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
    registerAll,
    rigidBody,
    sphere,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const WALL_WIDTH = 10;
const WALL_HEIGHT = 6;
const WALL_DEPTH = 2;
const BOX_SIZE = 1.0;
const STEPS_PER_OP = 360; // 6s of sim at 60Hz — long enough for the topple to settle
const TIME_STEP = 1 / 60;

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

const half = BOX_SIZE * 0.5;
const cubeShape = box.create({ halfExtents: [half, half, half], convexRadius: 0.02 });
const ballShape = sphere.create({ radius: 1.5 });

const OL_MOVING = 0;
const OL_STATIC = 1;

const GAP = 0.01;
const STEP_X = BOX_SIZE + GAP;
const STEP_Y = BOX_SIZE + GAP * 0.5;

function populate(world: World): void {
    // static ground
    rigidBody.create(world, {
        motionType: MotionType.STATIC,
        objectLayer: OL_STATIC,
        shape: box.create({ halfExtents: [50, 1, 50] }),
        position: [0, -1, 0],
        friction: 0.6,
        restitution: 0,
    });

    // dynamic wall
    const originX = -((WALL_WIDTH - 1) * STEP_X) / 2;
    const originZ = -((WALL_DEPTH - 1) * STEP_X) / 2;
    for (let w = 0; w < WALL_WIDTH; w++) {
        for (let h = 0; h < WALL_HEIGHT; h++) {
            for (let d = 0; d < WALL_DEPTH; d++) {
                rigidBody.create(world, {
                    motionType: MotionType.DYNAMIC,
                    objectLayer: OL_MOVING,
                    shape: cubeShape,
                    position: [originX + w * STEP_X, half + h * STEP_Y, originZ + d * STEP_X],
                    mass: 1,
                    friction: 0.5,
                    restitution: 0,
                });
            }
        }
    }

    // heavy dense sphere launched into the wall's base
    const ball = rigidBody.create(world, {
        motionType: MotionType.DYNAMIC,
        objectLayer: OL_MOVING,
        shape: ballShape,
        position: [originX - 3.0, 1.5, 0],
        mass: 300,
        friction: 0.4,
        restitution: 0.1,
    });
    rigidBody.setLinearVelocity(world, ball, [35, 0, 0]);
}

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` performs
 * exactly one simulated step. collapse is a transient with NO warmup
 * (warmupSteps = 0) and no per-step scenario work beyond advancing the sim, so
 * every op re-creates a fresh scenario (see the labs bench below).
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());
    populate(world);
    return {
        world,
        warmupSteps: 0,
        stepOnce(): void {
            updateWorld(world, undefined, TIME_STEP);
        },
    };
}

group('collapse', () => {
    bench('collapse', function* () {
        // fresh transient world per op — no shared warmup
        yield () => {
            const s = createScenario();
            for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(i);
        };
    }).gc('inner');
});

/**
 * Canonical run for cpu-prof attribution (invoked by `run-scenario.ts`).
 * Five fresh transients so the sampler collects the full collapse each time.
 */
export function runForProfiling(): void {
    for (let op = 0; op < 5; op++) {
        const s = createScenario();
        for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(i);
    }
}

// TEMP sanity
export function __sanity(): void {
    const s = createScenario();
    const { world } = s;
    let dynamicTotal = 0;
    for (const b of rigidBody.iterate(world)) if (b.motionType === MotionType.DYNAMIC) dynamicTotal++;
    const liveContacts = () => world.contacts.contacts.length - world.contacts.contactsFreeIndices.length;
    const bodyCount = () => world.bodies.pool.length - world.bodies.freeIndices.length;
    const a = performance.now();
    for (let i = 0; i < STEPS_PER_OP; i++) {
        s.stepOnce(i);
        if ((i + 1) % 60 === 0) {
            let asleep = 0;
            for (const b of rigidBody.iterate(world)) if (b.motionType === MotionType.DYNAMIC && b.sleeping) asleep++;
            console.error(
                `[collapse] step ${i + 1}  active ${world.bodies.activeBodyCount}  ` +
                    `asleep ${asleep}/${dynamicTotal}  liveContacts ${liveContacts()}  bodies ${bodyCount()}`,
            );
        }
    }
    const opMs = performance.now() - a;
    let asleep = 0;
    for (const b of rigidBody.iterate(world)) if (b.motionType === MotionType.DYNAMIC && b.sleeping) asleep++;
    console.error(
        `[collapse] op ${opMs.toFixed(0)}ms  final asleep ${asleep}/${dynamicTotal} ` +
            `(${((100 * asleep) / dynamicTotal).toFixed(0)}%)`,
    );
}
