// Macro perf benchmark — joints.
//
// "Ragdoll-ish chains flopping on the ground." 24 chains of 6-8 capsules each
// are linked end-to-end with the library's joints (a mix of swing-twist, hinge
// and point constraints, one type per chain, following example-ragdoll /
// example-constraints) and dropped onto a static plane so they topple and pile.
// A round-robin churn teleports one chain back up (zeroing its velocities)
// every ~2s to keep the constraint solver hot. This puts weight on the
// constraint solver (constraint-part setup + iterative solve + island building)
// rather than narrowphase.
//
// Deterministic mulberry32 RNG. The measured op re-seeds with a FIXED seed each
// iteration so the workload is identical (world-state drift between ops is fine,
// see the cube-heap note). `runForProfiling` is the entry the cpu-prof rig
// invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';
import { type Quat, quat, type Vec3, vec3 } from 'mathcat';

import {
    addBroadphaseLayer,
    addObjectLayer,
    capsule,
    ConstraintSpace,
    createWorld,
    createWorldSettings,
    enableCollision,
    hingeConstraint,
    type Listener,
    MotionType,
    plane,
    pointConstraint,
    registerAll,
    rigidBody,
    type RigidBody,
    type Shape,
    swingTwistConstraint,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const NUMBER_OF_CHAINS = 24;
const STEPS_PER_OP = 300; // 5s of sim at 60Hz
const TIME_STEP = 1 / 60;
const GRAVITY: Vec3 = [0, -20, 0];

// each capsule segment: cylinder half-height 0.2 + radius 0.1 => segment length 0.6
const SEG_HALF_HEIGHT = 0.2;
const SEG_RADIUS = 0.1;
const SEG_LEN = 2 * (SEG_HALF_HEIGHT + SEG_RADIUS);

const CHURN_INTERVAL = 120; // teleport one chain every ~2s
const GRID_SPACING = 2.5;
const DROP_HEIGHT = 2.0; // base of each chain starts this high above the plane

const RNG_SEED = 0xc0ffee;
const OP_SEED = 0x5eed11fe;
const WARMUP_SEED = RNG_SEED ^ 0xdeadbeef;
const STEADY_WARMUP_STEPS = 240;

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
    const BPH_STATIC = addBroadphaseLayer(s);
    const BPH_MOVING = addBroadphaseLayer(s);
    const OL_STATIC = addObjectLayer(s, BPH_STATIC);
    const OL_MOVING = addObjectLayer(s, BPH_MOVING);
    enableCollision(s, OL_MOVING, OL_MOVING);
    enableCollision(s, OL_MOVING, OL_STATIC);
    s.gravity = GRAVITY;
    return s;
}

const OL_STATIC = 0;
const OL_MOVING = 1;

const segShape: Shape = capsule.create({ halfHeightOfCylinder: SEG_HALF_HEIGHT, radius: SEG_RADIUS });

// three joint kinds, one per chain, cycled round-robin
enum JointKind {
    SWING_TWIST = 0,
    HINGE = 1,
    POINT = 2,
}

type Chain = {
    bodies: RigidBody[];
    baseX: number;
    baseZ: number;
    // local segment centre offsets relative to (baseX, 0, baseZ) at rest layout
    offsets: Vec3[];
};

type State = {
    world: World;
    chains: Chain[];
    listener: Listener;
};

// skip collisions between constraint-connected bodies (as in example-ragdoll)
const listener: Listener = {
    onBodyPairValidate: (a: RigidBody, b: RigidBody): boolean => !rigidBody.bodiesShareConstraint(a, b),
};

function linkSegments(world: World, kind: JointKind, a: RigidBody, b: RigidBody, jointWorld: Vec3): void {
    if (kind === JointKind.POINT) {
        pointConstraint.create(world, {
            bodyIdA: a.id,
            bodyIdB: b.id,
            pointA: vec3.clone(jointWorld),
            pointB: vec3.clone(jointWorld),
            space: ConstraintSpace.WORLD,
        });
    } else if (kind === JointKind.HINGE) {
        hingeConstraint.create(world, {
            bodyIdA: a.id,
            bodyIdB: b.id,
            pointA: vec3.clone(jointWorld),
            pointB: vec3.clone(jointWorld),
            hingeAxisA: [1, 0, 0],
            hingeAxisB: [1, 0, 0],
            normalAxisA: [0, 1, 0],
            normalAxisB: [0, 1, 0],
            space: ConstraintSpace.WORLD,
        });
    } else {
        swingTwistConstraint.create(world, {
            bodyIdA: a.id,
            bodyIdB: b.id,
            position1: vec3.clone(jointWorld),
            position2: vec3.clone(jointWorld),
            twistAxis1: [0, 1, 0],
            twistAxis2: [0, 1, 0],
            planeAxis1: [1, 0, 0],
            planeAxis2: [1, 0, 0],
            normalHalfConeAngle: 0.25 * Math.PI,
            planeHalfConeAngle: 0.25 * Math.PI,
            twistMinAngle: -0.2 * Math.PI,
            twistMaxAngle: 0.2 * Math.PI,
            space: ConstraintSpace.WORLD,
        });
    }
}

function populate(world: World, rng: () => number): State {
    // static ground plane
    rigidBody.create(world, {
        shape: plane.create({ plane: { normal: [0, 1, 0], constant: 0 }, halfExtent: 60 }),
        objectLayer: OL_STATIC,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        friction: 0.6,
        restitution: 0,
    });

    const cols = 6;
    const chains: Chain[] = [];
    for (let c = 0; c < NUMBER_OF_CHAINS; c++) {
        const gx = c % cols;
        const gz = (c / cols) | 0;
        const baseX = (gx - (cols - 1) / 2) * GRID_SPACING;
        const baseZ = (gz - 1.5) * GRID_SPACING;

        const kind: JointKind = c % 3;
        const segCount = 6 + ((rng() * 3) | 0); // 6, 7 or 8 segments
        const bodies: RigidBody[] = [];
        const offsets: Vec3[] = [];

        for (let i = 0; i < segCount; i++) {
            const offset: Vec3 = [0, DROP_HEIGHT + SEG_LEN / 2 + i * SEG_LEN, 0];
            offsets.push(offset);
            const body = rigidBody.create(world, {
                shape: segShape,
                objectLayer: OL_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [baseX + offset[0], offset[1], baseZ + offset[2]],
                quaternion: quat.create(),
                linearDamping: 0.05,
                angularDamping: 0.05,
                restitution: 0,
                friction: 0.5,
                mass: 1,
                // keep the solver hot: chains never sleep, so joints + contacts
                // are solved every step even after a chain has come to rest.
                allowSleeping: false,
            });
            bodies.push(body);

            if (i > 0) {
                // joint at the shared endpoint between segment i-1 and i
                const jointWorld: Vec3 = [baseX, DROP_HEIGHT + i * SEG_LEN, baseZ];
                linkSegments(world, kind, bodies[i - 1], body, jointWorld);
            }
        }

        chains.push({ bodies, baseX, baseZ, offsets });
    }

    return { world, chains, listener };
}

const _zero: Vec3 = [0, 0, 0];

function teleportChain(world: World, chain: Chain, newBaseY: number): void {
    for (let i = 0; i < chain.bodies.length; i++) {
        const body = chain.bodies[i];
        const off = chain.offsets[i];
        rigidBody.setLinearVelocity(world, body, _zero);
        rigidBody.setAngularVelocity(world, body, _zero);
        rigidBody.setQuaternion(world, body, quat.create(), true);
        rigidBody.setPosition(world, body, [chain.baseX + off[0], newBaseY + off[1], chain.baseZ + off[2]], true);
    }
}

function runSim(state: State, rng: () => number, steps: number): void {
    const { world, chains } = state;
    let cursor = 0;
    for (let i = 0; i < steps; i++) {
        if (i % CHURN_INTERVAL === 0) {
            const chain = chains[cursor % chains.length];
            cursor++;
            // teleport chain back up with a small seeded height jitter, zero velocities
            teleportChain(world, chain, DROP_HEIGHT + rng() * 1.5);
        }
        updateWorld(world, state.listener, TIME_STEP);
    }
}

group('joints', () => {
    bench('joints', function* () {
        const state = populate(createWorld(makeWorldSettings()), makeRng(RNG_SEED));
        runSim(state, makeRng(WARMUP_SEED), STEADY_WARMUP_STEPS);
        yield () => {
            runSim(state, makeRng(OP_SEED), STEPS_PER_OP);
        };
    }).gc('inner');
});

export function runForProfiling(): void {
    const state = populate(createWorld(makeWorldSettings()), makeRng(RNG_SEED));
    runSim(state, makeRng(WARMUP_SEED), STEADY_WARMUP_STEPS);
    for (let i = 0; i < 5; i++) {
        runSim(state, makeRng(OP_SEED), STEPS_PER_OP);
    }
}

// TEMP sanity
export function __sanity(): void {
    const state = populate(createWorld(makeWorldSettings()), makeRng(RNG_SEED));
    const t0 = performance.now();
    runSim(state, makeRng(WARMUP_SEED), STEADY_WARMUP_STEPS);
    const t1 = performance.now();
    const opTimes: number[] = [];
    for (let op = 0; op < 5; op++) {
        const a = performance.now();
        runSim(state, makeRng(OP_SEED), STEPS_PER_OP);
        opTimes.push(performance.now() - a);
    }
    // chain integrity: max adjacent-segment centre distance vs the rest length
    let maxDist = 0;
    let maxDev = 0;
    for (const chain of state.chains) {
        for (let i = 1; i < chain.bodies.length; i++) {
            const p = chain.bodies[i].position;
            const q = chain.bodies[i - 1].position;
            const d = Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
            maxDist = Math.max(maxDist, d);
            maxDev = Math.max(maxDev, Math.abs(d - SEG_LEN));
        }
    }
    const med = opTimes.slice().sort((a, b) => a - b)[opTimes.length >> 1];
    console.error(
        `[joints] warmup ${(t1 - t0).toFixed(0)}ms  op(median) ${med.toFixed(0)}ms  ` +
            `segLen ${SEG_LEN.toFixed(2)}m  maxAdjDist ${maxDist.toFixed(3)}m  maxDeviation ${maxDev.toFixed(3)}m`,
    );
}
