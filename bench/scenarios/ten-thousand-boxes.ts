import { box, MotionType, rigidBody, type Shape, updateWorld } from 'crashcat';
import { addGroundPlane, createStandardWorld, LAYER_MOVING, makeRng, TIME_STEP } from './common';
import { defineScenario } from './scenario';

// PEEL "TenThousandsBoxes" (CATEGORY_PERFORMANCE): a 16x16 footprint stacked 40 layers deep with
// randomly-sized boxes, dropped into one giant pile. This is the raw body-count ceiling — the
// broadphase tree, active-body bookkeeping and the sleeping system dominate, not narrowphase.
//
// One deviation: PEEL builds a distinct box shape per body. Here 200 shapes are cycled instead, so
// that shape construction does not dominate an op that already builds 10240 bodies.

const FOOTPRINT = 16;
const LAYERS = 40;
const SCALE = 4;
const AMPLITUDE = 1.5;
const SHAPE_VARIANTS = 200;
const RNG_SEED = 0x85ebca6b;

export const tenThousandBoxes = defineScenario({
    name: 'ten-thousand-boxes',
    description: 'PEEL TenThousandsBoxes — 10240 dynamic boxes, broadphase and sleeping at scale',
    steps: 10,
    create() {
        const rng = makeRng(RNG_SEED);
        const world = createStandardWorld();
        addGroundPlane(world, 400);

        const variants: Shape[] = [];
        for (let i = 0; i < SHAPE_VARIANTS; i++) {
            variants.push(
                box.create({
                    halfExtents: [Math.abs(rng() * 2 - 1) + 0.2, Math.abs(rng() * 2 - 1) + 0.2, Math.abs(rng() * 2 - 1) + 0.2],
                }),
            );
        }

        let variant = 0;
        for (let layer = 0; layer < LAYERS; layer++) {
            for (let z = 0; z < FOOTPRINT; z++) {
                for (let x = 0; x < FOOTPRINT; x++) {
                    rigidBody.create(world, {
                        shape: variants[variant++ % SHAPE_VARIANTS],
                        objectLayer: LAYER_MOVING,
                        motionType: MotionType.DYNAMIC,
                        position: [
                            (x - FOOTPRINT * 0.5) * SCALE,
                            rng() * 2 + AMPLITUDE + AMPLITUDE * 2 * layer,
                            (z - FOOTPRINT * 0.5) * SCALE,
                        ],
                        mass: 1,
                        friction: 0.5,
                        restitution: 0,
                    });
                }
            }
        }

        return {
            world,
            step() {
                updateWorld(world, undefined, TIME_STEP);
            },
        };
    },
});
