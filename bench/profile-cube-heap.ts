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
} from '../src';


/**
 * Deterministic port of examples/src/example-cube-heap.ts:
 * - 200 dynamic cubes (0.25 half-extent, convexRadius 0.05), gravity -50, friction 0.5, restitution 0
 * - cubes spawn at random positions in a +/-2.5 x/z column, height 0..10
 * - each frame, one random cube gets repositioned and zero-velocity'd (round robin churn)
 * - static plane ground
 *
 * uses a seeded mulberry32 RNG so runs are reproducible.
 */

registerAll();

// --- deterministic RNG (mulberry32) ---
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

const NUMBER_OF_CUBES = 200;
const SPAWN_HEIGHT = 10;
const SPAWN_AREA = 2.5;
const SECONDS = 10;
const TIME_STEP = 1 / 60;
const STEPS = (SECONDS / TIME_STEP) | 0;
const ITERS = 10;
const RNG_SEED = 0xc0ffee;

const worldSettings = createWorldSettings();
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
worldSettings.gravity = [0, -50, 0];

const cubeShape: Shape = box.create({ halfExtents: [0.25, 0.25, 0.25], convexRadius: 0.05 });

function populate(world: World, rng: () => number): RigidBody[] {
    rigidBody.create(world, {
        shape: plane.create({ plane: { normal: [0, 1, 0], constant: 0 }, halfExtent: 50 }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
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
                objectLayer: OBJECT_LAYER_MOVING,
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

function runSim(world: World, cubes: RigidBody[], rng: () => number): void {
    const zero: [number, number, number] = [0, 0, 0];
    for (let i = 0; i < STEPS; i++) {
        // round-robin churn: reposition one random cube per step
        const idx = (rng() * cubes.length) | 0;
        const cube = cubes[idx];
        rigidBody.setPosition(world, cube, [0, rng() * SPAWN_HEIGHT, 0], true);
        rigidBody.setLinearVelocity(world, cube, zero);
        rigidBody.setAngularVelocity(world, cube, zero);

        updateWorld(world, undefined, TIME_STEP);
    }
}

// warmup pass — separate world, separate RNG stream, lets V8 reach steady-state IC + tier
{
    const warmupWorld = createWorld(worldSettings);
    const warmupCubes = populate(warmupWorld, makeRng(RNG_SEED ^ 0xdeadbeef));
    runSim(warmupWorld, warmupCubes, makeRng(RNG_SEED ^ 0xdeadbeef));
}

// timed measurement
const measuredWorld = createWorld(worldSettings);
const measuredCubes = populate(measuredWorld, makeRng(RNG_SEED));

const t0 = performance.now();
for (let it = 0; it < ITERS; it++) {
    // each iter restarts the RNG so successive iters do the same churn sequence
    runSim(measuredWorld, measuredCubes, makeRng(RNG_SEED + it));
}
const t1 = performance.now();
const total = t1 - t0;
console.log(`avg: ${(total / ITERS).toFixed(2)} ms/iter (${ITERS} iters of ${STEPS} steps each)`);
