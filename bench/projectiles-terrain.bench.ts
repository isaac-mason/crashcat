// Macro perf benchmark — projectiles-terrain.
//
// "Fast projectiles ricocheting around blocky voxel terrain." This is the only
// scenario that exercises continuous collision detection (CCD): ~50 small
// spheres and boxes use MotionQuality.LINEAR_CAST and are launched at ~25 m/s,
// well above the CCD casting threshold (world.settings.ccd.linearCastThreshold
// * innerRadius = 0.05 * radius, i.e. any speed over ~0.6 m/s for a 0.2 m
// projectile), so every in-flight projectile does a swept-AABB broadphase query
// + shape-cast against the terrain BVH each step. Projectiles that settle (or
// leave the arena) are relaunched with a fresh seeded velocity. restitution
// 0.6 keeps them bouncing.
//
// NOTE ON TERRAIN: the voxel examples (example-voxel-triangle-mesh) build their
// collision geometry by meshing a voxel field into a triangle mesh — there is
// no dedicated voxel collider in crashcat. This scenario follows that pattern
// directly: it generates a BLOCKY, axis-aligned stepped landscape (integer
// per-cell heights with vertical walls between steps, plus tall perimeter
// walls) and bakes it into a single triangle-mesh shape. So the "voxel" terrain
// is a blocky triangle mesh, exactly as the examples do it.
//
// Deterministic mulberry32 RNG. The measured op re-seeds with a FIXED seed each
// iteration so the workload is identical (world-state drift between ops is fine,
// see the cube-heap note). `runForProfiling` is the entry the cpu-prof rig
// invokes via `run-scenario.ts`.

import { bench, group } from '@pmndrs/labs';
import type { Vec3 } from 'mathcat';

import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionQuality,
    MotionType,
    registerAll,
    rigidBody,
    type RigidBody,
    type Shape,
    sphere,
    triangleMesh,
    updateWorld,
    type World,
    type WorldSettings,
} from 'crashcat';

registerAll();

const NUMBER_OF_PROJECTILES = 50;
const STEPS_PER_OP = 300; // 5s of sim at 60Hz
const TIME_STEP = 1 / 60;
const GRAVITY: Vec3 = [0, -30, 0];

// blocky terrain: 24x24 cells over ~48x48m, integer step heights 0..3
const GRID = 24;
const CELL = 2.0;
const HALF = (GRID * CELL) / 2;
const STEP_H = 1.5; // metres per height level
const WALL_TOP = 20; // perimeter wall height (metres) — kept above spawn + max
// bounce height so fast ricochets stay contained rather than sailing over.

const LAUNCH_SPEED = 25; // m/s — comfortably above the CCD casting threshold
const PROJ_RADIUS = 0.25;
const SPAWN_Y = 10;
const SETTLE_SPEED = 1.5; // below this counts as "settled"
const SETTLE_STEPS = 30; // relaunch after this many consecutive settled steps
const FLOOR_Y = -1; // below this (under the slab top at y=0) => tunnelled

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

// blocky integer height per cell: a few big plateaus + steps
function cellLevel(ix: number, iz: number): number {
    const x = ix - GRID / 2;
    const z = iz - GRID / 2;
    const v = Math.sin(x * 0.5) * Math.cos(z * 0.4) + Math.sin((x + z) * 0.25);
    return Math.max(0, Math.min(3, Math.round(v + 1.5)));
}

// build a blocky triangle mesh: a top quad per cell plus vertical wall quads
// wherever a cell is taller than its neighbour, plus tall perimeter walls.
//
// WINDING MATTERS: CCD (shape cast) skips backfaces (collideWithBackfaces is
// false for casts), so every face's normal must point toward the side the
// projectiles approach from — up for the tops, toward the lower neighbour for
// the step walls, inward for the perimeter — or fast projectiles cast straight
// through the backface and tunnel. Vertices below are ordered accordingly.
function makeTerrainShape(): Shape {
    const positions: number[] = [];
    const indices: number[] = [];

    // v0,v1,v2,v3 must be wound CCW as seen from the outward-normal side
    const quad = (ax: number, ay: number, az: number, bx: number, by: number, bz: number, cx: number, cy: number, cz: number, dx: number, dy: number, dz: number) => {
        const base = positions.length / 3;
        positions.push(ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz);
        indices.push(base, base + 1, base + 2, base, base + 2, base + 3);
    };

    for (let iz = 0; iz < GRID; iz++) {
        for (let ix = 0; ix < GRID; ix++) {
            const level = cellLevel(ix, iz);
            const y = level * STEP_H;
            const x0 = ix * CELL - HALF;
            const x1 = x0 + CELL;
            const z0 = iz * CELL - HALF;
            const z1 = z0 + CELL;

            // top face (+Y up)
            quad(x0, y, z0, x0, y, z1, x1, y, z1, x1, y, z0);

            // vertical walls where neighbour is lower (edge of grid => level 0)
            const nx0 = ix > 0 ? cellLevel(ix - 1, iz) : 0;
            const nx1 = ix < GRID - 1 ? cellLevel(ix + 1, iz) : 0;
            const nz0 = iz > 0 ? cellLevel(ix, iz - 1) : 0;
            const nz1 = iz < GRID - 1 ? cellLevel(ix, iz + 1) : 0;

            if (nx0 < level) {
                const yn = nx0 * STEP_H; // faces -X (toward lower neighbour)
                quad(x0, y, z0, x0, yn, z0, x0, yn, z1, x0, y, z1);
            }
            if (nx1 < level) {
                const yn = nx1 * STEP_H; // faces +X
                quad(x1, y, z1, x1, yn, z1, x1, yn, z0, x1, y, z0);
            }
            if (nz0 < level) {
                const yn = nz0 * STEP_H; // faces -Z
                quad(x1, y, z0, x1, yn, z0, x0, yn, z0, x0, y, z0);
            }
            if (nz1 < level) {
                const yn = nz1 * STEP_H; // faces +Z
                quad(x0, y, z1, x0, yn, z1, x1, yn, z1, x1, y, z1);
            }
        }
    }

    // tall perimeter walls with INWARD-facing normals to contain ricochets
    const w = HALF;
    quad(-w, WALL_TOP, -w, -w, 0, -w, w, 0, -w, w, WALL_TOP, -w); // z=-w, +Z inward
    quad(w, WALL_TOP, w, w, 0, w, -w, 0, w, -w, WALL_TOP, w); // z=+w, -Z inward
    quad(-w, WALL_TOP, w, -w, 0, w, -w, 0, -w, -w, WALL_TOP, -w); // x=-w, +X inward
    quad(w, WALL_TOP, -w, w, 0, -w, w, 0, w, w, WALL_TOP, w); // x=+w, -X inward

    return triangleMesh.create({ positions, indices });
}

const terrainShape: Shape = makeTerrainShape();

// alternating projectile shapes: sphere and box (both small, both linear-cast)
const projShapes: Shape[] = [
    sphere.create({ radius: PROJ_RADIUS }),
    box.create({ halfExtents: [PROJ_RADIUS, PROJ_RADIUS, PROJ_RADIUS], convexRadius: 0.02 }),
];

type Projectile = {
    body: RigidBody;
    slowSteps: number;
};

type State = {
    world: World;
    projectiles: Projectile[];
};

// seeded launch velocity: mostly horizontal with a downward bias so projectiles
// slam into the steps/walls rather than sailing over them.
const _vel: Vec3 = [0, 0, 0];
function seededLaunchVelocity(rng: () => number): Vec3 {
    const az = rng() * Math.PI * 2;
    const pitch = -0.15 - rng() * 0.5; // downward
    const ch = Math.cos(pitch);
    _vel[0] = Math.cos(az) * ch * LAUNCH_SPEED;
    _vel[1] = Math.sin(pitch) * LAUNCH_SPEED;
    _vel[2] = Math.sin(az) * ch * LAUNCH_SPEED;
    return _vel;
}

function launch(world: World, p: Projectile, rng: () => number): void {
    const x = (rng() * 2 - 1) * (HALF * 0.3);
    const z = (rng() * 2 - 1) * (HALF * 0.3);
    rigidBody.setPosition(world, p.body, [x, SPAWN_Y, z], true);
    rigidBody.setAngularVelocity(world, p.body, [0, 0, 0]);
    rigidBody.setLinearVelocity(world, p.body, seededLaunchVelocity(rng));
    p.slowSteps = 0;
}

function populate(world: World, rng: () => number): State {
    rigidBody.create(world, {
        shape: terrainShape,
        objectLayer: OL_STATIC,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        friction: 0.4,
        restitution: 0.4,
    });

    // thick solid base slab under the arena (top flush with the level-0 cells at
    // y=0). The blocky mesh is a thin surface; this closes its underside so a
    // projectile that slips a rare step-corner seam lands on the slab instead of
    // the void — matching a real voxel world being solid below the surface.
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [HALF, 2, HALF] }),
        objectLayer: OL_STATIC,
        motionType: MotionType.STATIC,
        position: [0, -2, 0],
        friction: 0.4,
        restitution: 0.4,
    });

    const projectiles: Projectile[] = [];
    for (let i = 0; i < NUMBER_OF_PROJECTILES; i++) {
        const body = rigidBody.create(world, {
            shape: projShapes[i % projShapes.length],
            objectLayer: OL_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [0, SPAWN_Y, 0],
            restitution: 0.6,
            friction: 0.2,
            mass: 1,
            motionQuality: MotionQuality.LINEAR_CAST,
        });
        const p: Projectile = { body, slowSteps: 0 };
        launch(world, p, rng);
        projectiles.push(p);
    }
    return { world, projectiles };
}

function runSim(state: State, rng: () => number, steps: number): void {
    const { world, projectiles } = state;
    for (let i = 0; i < steps; i++) {
        for (const p of projectiles) {
            const v = p.body.motionProperties.linearVelocity;
            const speedSq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            const y = p.body.position[1];
            if (speedSq < SETTLE_SPEED * SETTLE_SPEED) {
                p.slowSteps++;
            } else {
                p.slowSteps = 0;
            }
            if (p.slowSteps >= SETTLE_STEPS || y < FLOOR_Y) {
                launch(world, p, rng);
            }
        }
        updateWorld(world, undefined, TIME_STEP);
    }
}

group('projectiles-terrain', () => {
    bench('projectiles-terrain', function* () {
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

    // step one op manually, sampling per-step CCD activity + tunnelling
    const rng = makeRng(OP_SEED);
    let ccdActiveSum = 0;
    let belowFloor = 0;
    let minY = Infinity;
    const a = performance.now();
    for (let i = 0; i < STEPS_PER_OP; i++) {
        for (const p of state.projectiles) {
            const v = p.body.motionProperties.linearVelocity;
            const speedSq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            const y = p.body.position[1];
            if (speedSq < SETTLE_SPEED * SETTLE_SPEED) p.slowSteps++;
            else p.slowSteps = 0;
            if (p.slowSteps >= SETTLE_STEPS || y < FLOOR_Y) launch(state.world, p, rng);
        }
        updateWorld(state.world, undefined, TIME_STEP);
        // after the step, ccdBodyIndex reflects which bodies did a CCD cast this step
        let active = 0;
        for (const p of state.projectiles) {
            if (p.body.ccdBodyIndex >= 0) active++;
            const y = p.body.position[1];
            if (y < minY) minY = y;
            if (y < FLOOR_Y) belowFloor++;
        }
        ccdActiveSum += active;
    }
    const opMs = performance.now() - a;
    console.error(
        `[projectiles-terrain] warmup ${(t1 - t0).toFixed(0)}ms  op ${opMs.toFixed(0)}ms  ` +
            `avgCCDactive/step ${(ccdActiveSum / STEPS_PER_OP).toFixed(1)}/${NUMBER_OF_PROJECTILES}  ` +
            `minY ${minY.toFixed(2)}m (floor ${FLOOR_Y})  belowFloorSamples ${belowFloor}`,
    );
}
