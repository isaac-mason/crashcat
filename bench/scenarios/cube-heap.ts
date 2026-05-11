// Cube-heap scenario — shared between the labs bench (`cube-heap.bench.ts`)
// and the cpu-prof attribution runner (`run-scenario.ts`).
//
// Mirrors examples/example-cube-heap.ts: 200 dynamic boxes drop into a static
// plane, round-robin churn re-spawns one box per frame to keep contacts hot.
// Uses a deterministic mulberry32 RNG so runs are reproducible across both
// the bench harness and the profiler.

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
} from '../../src';

registerAll();

export const NUMBER_OF_CUBES = 200;
export const SPAWN_HEIGHT = 10;
export const SPAWN_AREA = 2.5;
export const STEPS_PER_OP = 300; // 5s of sim at 60Hz
export const TIME_STEP = 1 / 60;
export const RNG_SEED = 0xc0ffee;
export const STEADY_WARMUP_STEPS = 600;

export function makeRng(seed: number): () => number {
    let s = seed >>> 0;
    return () => {
        s = (s + 0x6d2b79f5) >>> 0;
        let t = s;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

export function makeWorldSettings(): WorldSettings {
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

export function populate(world: World, rng: () => number): RigidBody[] {
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

export function runSim(world: World, cubes: RigidBody[], rng: () => number, steps: number): void {
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

/**
 * Canonical run for attribution profiling: pre-settle, then run several
 * steady-state ops back-to-back so the cpu-prof sampler collects enough
 * samples (~5s of CPU). What we profile is the running cost of a stable
 * heap, which is the cost that dominates a real game's frame budget.
 */
export function runForProfiling(): void {
    const settings = makeWorldSettings();
    const world = createWorld(settings);
    const cubes = populate(world, makeRng(RNG_SEED));
    runSim(world, cubes, makeRng(RNG_SEED ^ 0xdeadbeef), STEADY_WARMUP_STEPS);
    // 5 × 300 frames = 1500 measured steps; ~5-6s of CPU on M-series.
    for (let i = 0; i < 5; i++) {
        runSim(world, cubes, makeRng(RNG_SEED + i), STEPS_PER_OP);
    }
}
