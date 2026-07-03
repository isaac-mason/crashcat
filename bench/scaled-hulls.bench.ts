// Macro perf benchmark — scaled-hulls.
//
// "One convex-hull asset instanced everywhere at many uniform scales." A single
// shared ~40-vertex convex hull is wrapped in `scaled` shapes at 10 distinct
// UNIFORM scales (0.4x .. 2.5x) and instanced 150 times, then heaped onto a
// static plane exactly like cube-heap / hull-heap (round-robin churn re-spawns
// one body per frame to keep contacts hot). This isolates the cost of the
// scaled-convex-hull support path: every narrowphase query on these bodies goes
// scaled -> convex-hull getSupport with a per-call scale multiply. A uniform-
// scale fast path is being implemented in parallel, and this scenario is its
// before/after oracle.
//
// Deterministic mulberry32 RNG with a FIXED per-op seed so every measured op is
// the identical 300-step workload (see the cube-heap note). `runForProfiling`
// is the entry the cpu-prof rig invokes via `run-scenario.ts`.

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
    scaled,
    type Shape,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const NUMBER_OF_BODIES = 150;
const NUMBER_OF_SCALES = 10;
const SCALE_MIN = 0.4;
const SCALE_MAX = 2.5;
const HULL_VERTS = 40;
const HULL_RADIUS = 0.3;
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

// ~40 points on a fibonacci sphere — every point is a hull vertex, so numPoints ≈ 40
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

// one shared hull asset, wrapped in NUMBER_OF_SCALES distinct uniform-scale shapes
const hullShape: Shape = convexHull.create({ positions: hullPositions(HULL_VERTS, HULL_RADIUS), convexRadius: 0.05 });

const scaledShapes: Shape[] = [];
for (let k = 0; k < NUMBER_OF_SCALES; k++) {
    const s = SCALE_MIN + ((SCALE_MAX - SCALE_MIN) * k) / (NUMBER_OF_SCALES - 1);
    scaledShapes.push(scaled.create({ shape: hullShape, scale: [s, s, s] }));
}

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
                shape: scaledShapes[i % NUMBER_OF_SCALES],
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

group('scaled-hulls', () => {
    bench('scaled-hulls', function* () {
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

// TEMP sanity
export function __sanity(): void {
    const settings = makeWorldSettings();
    const world = createWorld(settings);
    const bodies = populate(world, makeRng(RNG_SEED));
    const t0 = performance.now();
    runSim(world, bodies, makeRng(RNG_SEED ^ 0xdeadbeef), STEADY_WARMUP_STEPS);
    const t1 = performance.now();
    runSim(world, bodies, makeRng(RNG_SEED), STEPS_PER_OP);
    const t2 = performance.now();
    let minY = Infinity;
    let below = 0;
    for (const b of bodies) {
        minY = Math.min(minY, b.position[1]);
        if (b.position[1] < -1) below++;
    }
    console.error(
        `[scaled-hulls] warmup ${(t1 - t0).toFixed(0)}ms  op ${(t2 - t1).toFixed(0)}ms  ` +
            `bodies ${bodies.length} scales ${NUMBER_OF_SCALES} (${SCALE_MIN}-${SCALE_MAX}x)  ` +
            `hullVerts ${HULL_VERTS}  minY ${minY.toFixed(2)}m  belowPlane ${below}`,
    );
}
