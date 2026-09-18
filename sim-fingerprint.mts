// A bit-exact fingerprint of a simulation, for before/after comparison across a refactor.
//
// Not a benchmark and not a test — a safety net. The solver rewrite is far too broad for a unit test
// to cover, and crashcat's own suite checks behaviour rather than bit-identity against a reference.
// This runs scenes that exercise contact solving, warm starting and constraint parts, then hashes
// every body's full state as raw bits. Two runs that agree here did the same arithmetic.
//
// Usage: `./bench/node_modules/.bin/tsx sim-fingerprint.mts [label]`
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
} from './src';

registerAll();

function makeWorld() {
    const settings = createWorldSettings();
    const bpMoving = addBroadphaseLayer(settings);
    const bpStatic = addBroadphaseLayer(settings);
    const moving = addObjectLayer(settings, bpMoving);
    const stat = addObjectLayer(settings, bpStatic);
    enableCollision(settings, moving, moving);
    enableCollision(settings, moving, stat);
    return { world: createWorld(settings), moving, stat };
}

/** A stack of boxes plus a few spheres dropped onto it: contacts, warm starting, resting islands. */
function buildScene() {
    const { world, moving, stat } = makeWorld();

    rigidBody.create(world, {
        shape: box.create({ halfExtents: [40, 1, 40] }),
        objectLayer: stat,
        motionType: MotionType.STATIC,
        position: [0, -1, 0],
    } as never);

    for (let s = 0; s < 6; s++) {
        for (let i = 0; i < 6; i++) {
            rigidBody.create(world, {
                shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
                objectLayer: moving,
                motionType: MotionType.DYNAMIC,
                // Deliberately off-axis so the solver sees rotation, not just a clean vertical stack.
                position: [s * 1.7 - 4 + i * 0.03, 0.5 + i * 1.02, s * 0.9 - 2],
            } as never);
        }
    }

    for (let i = 0; i < 8; i++) {
        rigidBody.create(world, {
            shape: sphere.create({ radius: 0.45 }),
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [i * 1.1 - 4, 9 + i * 0.5, i * 0.3 - 1],
        } as never);
    }

    return world;
}

/** FNV-1a over the raw bits of every number, so no rounding can hide a difference. */
function hasher() {
    let h = 0x811c9dc5;
    const buf = new DataView(new ArrayBuffer(8));
    return {
        push(n: number) {
            buf.setFloat64(0, n);
            for (let i = 0; i < 8; i++) {
                h ^= buf.getUint8(i);
                h = Math.imul(h, 0x01000193) >>> 0;
            }
        },
        get value() {
            return h.toString(16).padStart(8, '0');
        },
    };
}

const world = buildScene();
const STEPS = 180;
const marks: string[] = [];

for (let step = 1; step <= STEPS; step++) {
    updateWorld(world, undefined, 1 / 60);
    if (step % 60 === 0 || step === STEPS) {
        const h = hasher();
        for (const body of world.bodies.pool) {
            if (!body) continue;
            for (let i = 0; i < 3; i++) h.push(body.position[i]);
            for (let i = 0; i < 4; i++) h.push(body.quaternion[i]);
            const mp = body.motionProperties;
            if (mp) {
                for (let i = 0; i < 3; i++) h.push(mp.linearVelocity[i]);
                for (let i = 0; i < 3; i++) h.push(mp.angularVelocity[i]);
            }
        }
        marks.push(`step ${String(step).padStart(3)}  ${h.value}`);
    }
}

const label = process.argv[2] ?? '';
console.log(`sim fingerprint ${label}`);
console.log(`bodies: ${world.bodies.pool.filter(Boolean).length}, active at end: ${world.bodies.activeBodyCount}`);
for (const m of marks) console.log('  ' + m);
