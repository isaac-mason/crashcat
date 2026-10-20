import {
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    filter,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
} from 'crashcat';
import {
    captureBodyState,
    createStandardWorld,
    createTerrainShape,
    LAYER_MOVING,
    LAYER_STATIC,
    makeRng,
    restoreBodyState,
    TIME_STEP,
    terrainHeight,
} from './common';
import { defineScenario } from './scenario';

// PEEL's raycast category (SceneRaycastVsStaticMeshes_Terrain and friends): a fan of closest-hit
// rays fired at a static triangle mesh every step, with a handful of dynamic bodies bouncing on it.
// This is the query path — broadphase castRay traversal plus castRayVsTriangleMesh.

const RAYS_PER_STEP = 256;
const RAY_LENGTH = 40;
const SPHERE_COUNT = 40;
const TERRAIN_QUADS = 96;
const TERRAIN_EXTENT = 100;
const RNG_SEED = 0x165667b1;

export const raycastMesh = defineScenario({
    name: 'raycast-mesh',
    description: 'PEEL raycast-vs-static-mesh — 256 closest-hit rays per step against a terrain BVH',
    steps: 200,
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

        const sphereShape = sphere.create({ radius: 0.5 });
        for (let i = 0; i < SPHERE_COUNT; i++) {
            const x = (rng() * 2 - 1) * 20;
            const z = (rng() * 2 - 1) * 20;
            rigidBody.create(world, {
                shape: sphereShape,
                objectLayer: LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [x, terrainHeight(x, z) + 3 + rng() * 4, z],
                mass: 1,
                friction: 0.4,
                restitution: 0.4,
            });
        }

        const queryFilter = filter.create(world.settings.layers);
        const collector = createClosestCastRayCollector();
        const raySettings = createDefaultCastRaySettings();
        const origin: [number, number, number] = [0, 0, 0];
        const direction: [number, number, number] = [0, 0, 0];

        const captured = captureBodyState(world);

        return {
            world,
            reset() {
                restoreBodyState(world, captured);
            },
            step(stepIndex: number) {
                // a sensor rig orbiting above the terrain, firing a fan of mostly-downward rays
                const t = stepIndex * TIME_STEP;
                const rigX = Math.cos(t * 0.7) * 18;
                const rigZ = Math.sin(t * 0.7) * 18;
                origin[0] = rigX;
                origin[1] = terrainHeight(rigX, rigZ) + 8;
                origin[2] = rigZ;

                for (let r = 0; r < RAYS_PER_STEP; r++) {
                    const azimuth = (r / RAYS_PER_STEP) * Math.PI * 2 + t * 0.3;
                    const downward = 0.25 + (0.7 * ((r * 2654435761) % 97)) / 97;
                    const flat = 1 - downward;
                    direction[0] = Math.cos(azimuth) * flat;
                    direction[1] = -downward;
                    direction[2] = Math.sin(azimuth) * flat;
                    const invLength = 1 / Math.hypot(direction[0], direction[1], direction[2]);
                    direction[0] *= invLength;
                    direction[1] *= invLength;
                    direction[2] *= invLength;

                    collector.reset();
                    castRay(world, collector, raySettings, origin, direction, RAY_LENGTH, queryFilter);
                }

                updateWorld(world, undefined, TIME_STEP);
            },
        };
    },
});
