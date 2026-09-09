import { box, capsule, convexHull, MotionType, rigidBody, type Shape, sphere, updateWorld } from 'crashcat';
import {
    convexBlobPositions,
    createStandardWorld,
    createTerrainShape,
    LAYER_MOVING,
    LAYER_STATIC,
    makeRng,
    TIME_STEP,
    terrainHeight,
} from './common';
import { defineScenario } from './scenario';

// Jolt's PerformanceTest ConvexVsMeshScene: mixed convex bodies tumbling down a triangle-mesh
// terrain. Convex-vs-mesh is a different narrowphase path from convex-vs-convex — mesh BVH
// traversal, per-triangle collide, active-edge handling — and nothing else here covers it.

const BODY_COUNT = 200;
const TERRAIN_QUADS = 96;
const TERRAIN_EXTENT = 120;
const SCATTER_HALF = 40;
const RNG_SEED = 0x27d4eb2f;

export const convexVsMesh = defineScenario({
    name: 'convex-vs-mesh',
    description: 'Jolt PerformanceTest ConvexVsMeshScene — 200 mixed convexes on a triangle mesh',
    steps: 90,
    create() {
        const rng = makeRng(RNG_SEED);
        const world = createStandardWorld();

        rigidBody.create(world, {
            shape: createTerrainShape(TERRAIN_QUADS, TERRAIN_EXTENT),
            objectLayer: LAYER_STATIC,
            motionType: MotionType.STATIC,
            position: [0, 0, 0],
            friction: 0.5,
            restitution: 0,
        });

        const shapes: Shape[] = [
            box.create({ halfExtents: [0.4, 0.4, 0.4] }),
            sphere.create({ radius: 0.45 }),
            capsule.create({ halfHeightOfCylinder: 0.3, radius: 0.25 }),
            convexHull.create({ positions: convexBlobPositions(32, 0.45, rng), convexRadius: 0.02 }),
            convexHull.create({ positions: convexBlobPositions(48, 0.55, rng), convexRadius: 0.02 }),
        ];

        for (let i = 0; i < BODY_COUNT; i++) {
            const x = (rng() * 2 - 1) * SCATTER_HALF;
            const z = (rng() * 2 - 1) * SCATTER_HALF;
            rigidBody.create(world, {
                shape: shapes[i % shapes.length],
                objectLayer: LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [x, terrainHeight(x, z) + 2 + rng() * 6, z],
                mass: 1,
                friction: 0.5,
                restitution: 0,
            });
        }

        return {
            world,
            step() {
                updateWorld(world, undefined, TIME_STEP);
            },
        };
    },
});
