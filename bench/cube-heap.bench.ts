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
    registerAll,
    rigidBody,
    type RigidBody,
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

function runSim(world: World, cubes: RigidBody[], rng: () => number, steps: number): void {
    const zero: [number, number, number] = [0, 0, 0];
    for (let i = 0; i < steps; i++) {
        const idx = (rng() * cubes.length) | 0;
        const cube = cubes[idx];
        rigidBody.setPosition(world, cube, [0, rng() * SPAWN_HEIGHT, 0], true);
        rigidBody.setLinearVelocity(world, cube, zero);
        rigidBody.setAngularVelocity(world, cube, zero);
        updateWorld(world, undefined, TIME_STEP);
    }
}

group('cube-heap', () => {
    bench('cube-heap', function* () {
        const settings = makeWorldSettings();
        const world = createWorld(settings);
        const cubes = populate(world, makeRng(RNG_SEED));
        runSim(world, cubes, makeRng(RNG_SEED ^ 0xdeadbeef), STEADY_WARMUP_STEPS);
        let seedNonce = 0;
        yield () => {
            runSim(world, cubes, makeRng(RNG_SEED + seedNonce++), STEPS_PER_OP);
        };
    }).gc('inner');
});

/**
 * Canonical run for cpu-prof attribution (invoked by `run-scenario.ts`).
 * Pre-settle, then run 5×STEPS_PER_OP steady-state steps so the sampler
 * collects ~5s of in-scenario CPU.
 */
export function runForProfiling(): void {
    const settings = makeWorldSettings();
    const world = createWorld(settings);
    const cubes = populate(world, makeRng(RNG_SEED));
    runSim(world, cubes, makeRng(RNG_SEED ^ 0xdeadbeef), STEADY_WARMUP_STEPS);
    for (let i = 0; i < 5; i++) {
        runSim(world, cubes, makeRng(RNG_SEED + i), STEPS_PER_OP);
    }
}
