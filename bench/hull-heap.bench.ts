// Macro perf benchmark — hull-heap.
//
// Same harness as cube-heap, but the 200 dynamic bodies are a shared 100-vertex
// convex hull instead of a box. This puts real weight on the support machinery:
// the O(numPoints) `getSupport` vertex scan and the per-pair convex-radius shrink
// build. `runForProfiling` is the entry the cpu-prof rig invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';

import {
    addBroadphaseLayer,
    addObjectLayer,
    convexHull,
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

const NUMBER_OF_BODIES = 200;
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

// 100 points on a fibonacci sphere — every point is a hull vertex, so numPoints ≈ 100
function hullPositions(n: number, radius: number): number[] {
    const positions: number[] = [];
    for (let i = 0; i < n; i++) {
        const y = 1 - (i / (n - 1)) * 2;
        const r = Math.sqrt(Math.max(0, 1 - y * y));
        const th = i * 2.399963;
        positions.push(Math.cos(th) * r * radius, y * radius, Math.sin(th) * r * radius);
    }
    return positions;
}

const hullShape: Shape = convexHull.create({ positions: hullPositions(100, 0.28), convexRadius: 0.05 });

function populate(world: World, rng: () => number): RigidBody[] {
    rigidBody.create(world, {
        shape: plane.create({ plane: { normal: [0, 1, 0], constant: 0 }, halfExtent: 50 }),
        objectLayer: 1, // OL_STATIC
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        restitution: 0,
        friction: 0.5,
    });

    const bodies: RigidBody[] = [];
    for (let i = 0; i < NUMBER_OF_BODIES; i++) {
        const x = (rng() * 2 - 1) * SPAWN_AREA;
        const y = rng() * SPAWN_HEIGHT;
        const z = (rng() * 2 - 1) * SPAWN_AREA;
        bodies.push(
            rigidBody.create(world, {
                shape: hullShape,
                objectLayer: 0, // OL_MOVING
                motionType: MotionType.DYNAMIC,
                position: [x, y, z],
                restitution: 0,
                friction: 0.5,
                mass: 1,
            }),
        );
    }
    return bodies;
}

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` re-spawns
 * one hull body per frame (the churn) then advances the sim. Seeding matches
 * cube-heap: warmup uses a distinct seed; the op rng is re-seeded at each op
 * boundary with `RNG_SEED + op`, so a labs op (op = 0) runs the fixed-seed
 * workload while the profiling run's ops rotate. World-state drift is fine.
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());
    const bodies = populate(world, makeRng(RNG_SEED));
    const warmupRng = makeRng(RNG_SEED ^ 0xdeadbeef);
    let opRng = makeRng(RNG_SEED);
    const zero: [number, number, number] = [0, 0, 0];

    const churn = (rng: () => number): void => {
        const body = bodies[(rng() * bodies.length) | 0];
        rigidBody.setPosition(world, body, [0, rng() * SPAWN_HEIGHT, 0], true);
        rigidBody.setLinearVelocity(world, body, zero);
        rigidBody.setAngularVelocity(world, body, zero);
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

group('hull-heap', () => {
    bench('hull-heap', function* () {
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
    for (let i = 0; i < 5 * STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
}
