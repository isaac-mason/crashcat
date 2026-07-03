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
    registerAll,
    rigidBody,
    type RigidBody,
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

function runSim(world: World, bodies: RigidBody[], rng: () => number, steps: number): void {
    const zero: [number, number, number] = [0, 0, 0];
    for (let i = 0; i < steps; i++) {
        const idx = (rng() * bodies.length) | 0;
        const body = bodies[idx];
        rigidBody.setPosition(world, body, [0, rng() * SPAWN_HEIGHT, 0], true);
        rigidBody.setLinearVelocity(world, body, zero);
        rigidBody.setAngularVelocity(world, body, zero);
        updateWorld(world, undefined, TIME_STEP);
    }
}

group('hull-heap', () => {
    bench('hull-heap', function* () {
        const settings = makeWorldSettings();
        const world = createWorld(settings);
        const bodies = populate(world, makeRng(RNG_SEED));
        runSim(world, bodies, makeRng(RNG_SEED ^ 0xdeadbeef), STEADY_WARMUP_STEPS);
        // fixed per-op seed: identical workload every iteration (see cube-heap note)
        yield () => {
            runSim(world, bodies, makeRng(RNG_SEED), STEPS_PER_OP);
        };
    }).gc('inner');
});

export function runForProfiling(): void {
    const settings = makeWorldSettings();
    const world = createWorld(settings);
    const bodies = populate(world, makeRng(RNG_SEED));
    runSim(world, bodies, makeRng(RNG_SEED ^ 0xdeadbeef), STEADY_WARMUP_STEPS);
    for (let i = 0; i < 5; i++) {
        runSim(world, bodies, makeRng(RNG_SEED + i), STEPS_PER_OP);
    }
}
