import { convexHull, MotionType, rigidBody, type Shape, updateWorld } from 'crashcat';
import {
    addGroundPlane,
    captureBodyState,
    createStandardWorld,
    LAYER_MOVING,
    makeRng,
    restoreBodyState,
    TIME_STEP,
} from './common';
import { defineScenario } from './scenario';

// PEEL "PileOfMediumConvexes" (CATEGORY_PERFORMANCE), via GenerateConvexPile(5, 5, 20, 2, 16):
// a 5x5 footprint stacked 20 layers deep, every body sharing one 16-point convex hull of radius 2,
// dropped into a heap. Convex-vs-convex is the GJK/EPA path with hull support as its inner loop,
// so this is the scenario that moves when narrowphase changes.

const FOOTPRINT_X = 5;
const FOOTPRINT_Z = 5;
const LAYERS = 20;
const AMPLITUDE = 2;
const HULL_POINTS = 16;
const SCALE = 8;
const RNG_SEED = 0x9e3779b9;

export const convexPile = defineScenario({
    name: 'convex-pile',
    description: 'PEEL PileOfMediumConvexes — 500 bodies sharing one 16-point hull, GJK/EPA',
    steps: 100,
    create() {
        const rng = makeRng(RNG_SEED);
        const world = createStandardWorld();
        addGroundPlane(world);

        // PEEL's UnitRandomPt scaled by the amplitude — one hull, reused by every body
        const positions: number[] = [];
        for (let i = 0; i < HULL_POINTS; i++) {
            positions.push((rng() * 2 - 1) * AMPLITUDE, (rng() * 2 - 1) * AMPLITUDE, (rng() * 2 - 1) * AMPLITUDE);
        }
        const hullShape: Shape = convexHull.create({ positions, convexRadius: 0.05 });

        for (let layer = 0; layer < LAYERS; layer++) {
            for (let z = 0; z < FOOTPRINT_Z; z++) {
                for (let x = 0; x < FOOTPRINT_X; x++) {
                    rigidBody.create(world, {
                        shape: hullShape,
                        objectLayer: LAYER_MOVING,
                        motionType: MotionType.DYNAMIC,
                        position: [
                            (x - FOOTPRINT_X * 0.5) * SCALE,
                            AMPLITUDE + AMPLITUDE * 2 * layer,
                            (z - FOOTPRINT_Z * 0.5) * SCALE,
                        ],
                        mass: 1,
                        friction: 0.5,
                        restitution: 0,
                    });
                }
            }
        }

        const captured = captureBodyState(world);

        return {
            world,
            reset() {
                restoreBodyState(world, captured);
            },
            step() {
                updateWorld(world, undefined, TIME_STEP);
            },
        };
    },
});
