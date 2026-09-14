import { box, MotionType, rigidBody, type Shape, updateWorld } from 'crashcat';
import { captureBodyState, createStandardWorld, LAYER_STATIC, makeRng, restoreBodyState, TIME_STEP } from './common';
import { defineScenario } from './scenario';

// PEEL "SeaOfStaticBoxes" (CATEGORY_STATIC_SCENE), via CreateSeaOfStaticBoxes(40, 128, 128, 0):
// a 128x128 field of randomly-sized static boxes and nothing else — no ground, no dynamic bodies.
// PEEL uses it for memory and to expose "engines that take a significant amount of time to simulate
// a scene where everything is static". World construction is outside the timed window, so the tree
// build is not measured here — what the window measures is the second half of that: what a step
// costs when there is nothing to simulate. labs' heap column covers the memory half.
//
// One deviation: PEEL's extents are UnitRandom + 1 per axis, which reaches zero. This uses
// 1 + rng() so no box is degenerate.

const GRID = 128;
const AMPLITUDE = 40;
const RNG_SEED = 0xc2b2ae35;

export const seaOfStaticBoxes = defineScenario({
    name: 'sea-of-static-boxes',
    description: 'PEEL SeaOfStaticBoxes — 16384 static boxes, the cost of stepping a world at rest',
    steps: 600,
    create() {
        const rng = makeRng(RNG_SEED);
        const world = createStandardWorld();

        for (let z = 0; z < GRID; z++) {
            const coeffZ = 2 * (z / (GRID - 1) - 0.5);
            for (let x = 0; x < GRID; x++) {
                const coeffX = 2 * (x / (GRID - 1) - 0.5);
                const shape: Shape = box.create({ halfExtents: [1 + rng(), 1 + rng(), 1 + rng()] });
                rigidBody.create(world, {
                    shape,
                    objectLayer: LAYER_STATIC,
                    motionType: MotionType.STATIC,
                    position: [coeffX * AMPLITUDE + (rng() * 2 - 1), rng() * 2 - 1, coeffZ * AMPLITUDE + (rng() * 2 - 1)],
                    friction: 0.5,
                    restitution: 0,
                });
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
