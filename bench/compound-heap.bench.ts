// Macro perf benchmark — compound-heap.
//
// A heap of compound-shaped bodies. 120 dynamic bodies whose shapes are
// COMPOUNDS of 3-5 child boxes/spheres (three templates — hammer, L-bracket,
// small table) are dropped into a static box container, cube-heap style, with a
// round-robin churn re-spawning one body per frame (seeded jitter) to keep
// contacts hot. Where cube-heap/hull-heap put single convex leaves against each
// other, this exercises the compound sub-shape narrowphase (each body-pair
// expands into a child × child collide loop) and the multi-contact pair chains
// that result (several simultaneous contacts per body pair feeding the manifold
// + solver).
//
// Deterministic mulberry32 RNG with a FIXED per-op seed so every measured op is
// the identical 300-step workload (see the cube-heap note). `runForProfiling`
// is the entry the cpu-prof rig invokes via `run-scenario.ts`. See
// settle-sleep.bench.ts for the `createScenario` per-step convention.

import { bench, group } from '@pmndrs/labs';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    compound,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    type RigidBody,
    registerAll,
    rigidBody,
    type Shape,
    sphere,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';
import { quat, type Vec3 } from 'mathcat';

registerAll();

const NUMBER_OF_BODIES = 60;
const SPAWN_HEIGHT = 8;
const SPAWN_AREA = 3.0;
const STEPS_PER_OP = 240; // 4s of sim at 60Hz
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
    s.gravity = [0, -30, 0];
    return s;
}

const OL_MOVING = 0;
const OL_STATIC = 1;

const IDENTITY = quat.create();
const child = (shape: Shape, position: Vec3): { shape: Shape; position: Vec3; quaternion: typeof IDENTITY } => ({
    shape,
    position,
    quaternion: IDENTITY,
});

// three compound templates (3-5 children each), a mix of boxes and spheres
const hammer: Shape = compound.create({
    children: [
        child(box.create({ halfExtents: [0.07, 0.34, 0.07], convexRadius: 0.02 }), [0, 0, 0]), // handle
        child(box.create({ halfExtents: [0.12, 0.12, 0.28], convexRadius: 0.02 }), [0, 0.38, 0]), // head
        child(box.create({ halfExtents: [0.1, 0.06, 0.1], convexRadius: 0.02 }), [0, 0.14, 0]), // collar
    ],
});
const lBracket: Shape = compound.create({
    children: [
        child(box.create({ halfExtents: [0.34, 0.08, 0.1], convexRadius: 0.02 }), [0, 0, 0]), // arm A
        child(box.create({ halfExtents: [0.08, 0.3, 0.1], convexRadius: 0.02 }), [-0.26, 0.22, 0]), // arm B
        child(sphere.create({ radius: 0.11 }), [-0.2, 0.05, 0]), // gusset
    ],
});
const table: Shape = compound.create({
    children: [
        child(box.create({ halfExtents: [0.34, 0.05, 0.34], convexRadius: 0.02 }), [0, 0.3, 0]), // top
        child(box.create({ halfExtents: [0.05, 0.28, 0.05], convexRadius: 0.02 }), [0.27, 0, 0.27]),
        child(box.create({ halfExtents: [0.05, 0.28, 0.05], convexRadius: 0.02 }), [-0.27, 0, 0.27]),
        child(box.create({ halfExtents: [0.05, 0.28, 0.05], convexRadius: 0.02 }), [0.27, 0, -0.27]),
        child(box.create({ halfExtents: [0.05, 0.28, 0.05], convexRadius: 0.02 }), [-0.27, 0, -0.27]),
    ],
});
const templates: Shape[] = [hammer, lBracket, table];

function makeContainer(world: World): void {
    const EXTENT = 6;
    const WALL_H = 5;
    const T = 0.5;
    // floor + four walls forming an open box the compounds heap up inside
    const parts: { he: Vec3; pos: Vec3 }[] = [
        { he: [EXTENT, T, EXTENT], pos: [0, -T, 0] },
        { he: [T, WALL_H, EXTENT], pos: [EXTENT + T, WALL_H, 0] },
        { he: [T, WALL_H, EXTENT], pos: [-(EXTENT + T), WALL_H, 0] },
        { he: [EXTENT, WALL_H, T], pos: [0, WALL_H, EXTENT + T] },
        { he: [EXTENT, WALL_H, T], pos: [0, WALL_H, -(EXTENT + T)] },
    ];
    for (const p of parts) {
        rigidBody.create(world, {
            motionType: MotionType.STATIC,
            objectLayer: OL_STATIC,
            shape: box.create({ halfExtents: p.he }),
            position: p.pos,
            friction: 0.5,
            restitution: 0,
        });
    }
}

export type Scenario = {
    world: World;
    warmupSteps: number;
    stepOnce(stepIndex: number): void;
};

/**
 * Single source of truth for construction + per-step work. `stepOnce` re-spawns
 * one body per frame (cube-heap-style churn) then advances the sim. The op rng
 * is re-seeded at each op boundary — with `RNG_SEED + op` — so repeated labs ops
 * (op 0 → RNG_SEED) and the rotating-seed profiling ops both fall out of the one
 * source (world-state drift between ops is fine — see the cube-heap note).
 */
export function createScenario(): Scenario {
    const world = createWorld(makeWorldSettings());
    makeContainer(world);

    const warmupRng = makeRng(RNG_SEED ^ 0xdeadbeef);
    let opRng = makeRng(RNG_SEED);

    const bodies: RigidBody[] = [];
    // populate uses a fresh RNG_SEED stream (matches cube-heap's populate)
    const popRng = makeRng(RNG_SEED);
    for (let i = 0; i < NUMBER_OF_BODIES; i++) {
        const x = (popRng() * 2 - 1) * SPAWN_AREA;
        const y = 1 + popRng() * SPAWN_HEIGHT;
        const z = (popRng() * 2 - 1) * SPAWN_AREA;
        bodies.push(
            rigidBody.create(world, {
                motionType: MotionType.DYNAMIC,
                objectLayer: OL_MOVING,
                shape: templates[i % templates.length],
                position: [x, y, z],
                mass: 1,
                friction: 0.5,
                restitution: 0,
            }),
        );
    }

    const zero: Vec3 = [0, 0, 0];
    const churn = (rng: () => number): void => {
        const b = bodies[(rng() * bodies.length) | 0];
        const x = (rng() * 2 - 1) * SPAWN_AREA;
        const z = (rng() * 2 - 1) * SPAWN_AREA;
        rigidBody.setPosition(world, b, [x, 1 + rng() * SPAWN_HEIGHT, z], true);
        rigidBody.setLinearVelocity(world, b, zero);
        rigidBody.setAngularVelocity(world, b, zero);
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

group('compound-heap', () => {
    bench('compound-heap', function* () {
        const s = createScenario();
        for (let i = 0; i < s.warmupSteps; i++) s.stepOnce(i);
        yield () => {
            for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + i);
        };
    }).gc('inner');
});

/**
 * Canonical run for cpu-prof attribution (invoked by `run-scenario.ts`).
 * Pre-settle, then run 5×STEPS_PER_OP steady-state steps (rotating op seeds).
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
    for (let op = 0; op < 5; op++) {
        const b = performance.now();
        for (let i = 0; i < STEPS_PER_OP; i++) s.stepOnce(s.warmupSteps + op * STEPS_PER_OP + i);
        let asleep = 0;
        let minY = Infinity;
        let below = 0;
        for (const body of rigidBody.iterate(world)) {
            if (body.motionType !== MotionType.DYNAMIC) continue;
            if (body.sleeping) asleep++;
            minY = Math.min(minY, body.position[1]);
            if (body.position[1] < -3) below++;
        }
        console.error(
            `[compound-heap] op${op} ${(performance.now() - b).toFixed(0)}ms  ` +
                `active ${world.bodies.activeBodyCount}  asleep ${asleep}/${NUMBER_OF_BODIES}  ` +
                `liveContacts ${liveContacts()}  minY ${minY.toFixed(2)}  belowVoid ${below}`,
        );
    }
    console.error(`[compound-heap] warmup ${(w - a).toFixed(0)}ms`);
}
