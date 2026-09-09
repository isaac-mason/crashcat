import { box, MotionType, rigidBody, type Shape, staticCompound, updateWorld } from 'crashcat';
import { addGroundPlane, createStandardWorld, LAYER_MOVING, makeRng, TIME_STEP } from './common';
import { defineScenario } from './scenario';

// PEEL "PileOfSmallCompounds" (CATEGORY_PERFORMANCE): a heap of multi-part compound bodies. Every
// pair costs a sub-shape tree walk before any convex work, so this separates compound dispatch
// and sub-shape id handling from the plain convex path convex-pile covers.

const COMPOUND_COUNT = 200;
const GRID = 6;
const SPACING = 1.8;
const RNG_SEED = 0x2545f491;

function createCrossShape(armLength: number, armThickness: number): Shape {
    const arm = box.create({ halfExtents: [armLength, armThickness, armThickness] });
    const upright = box.create({ halfExtents: [armThickness, armLength, armThickness] });
    const cross = box.create({ halfExtents: [armThickness, armThickness, armLength] });
    return staticCompound.create({
        children: [
            { shape: arm, position: [0, 0, 0], quaternion: [0, 0, 0, 1] },
            { shape: upright, position: [0, 0, 0], quaternion: [0, 0, 0, 1] },
            { shape: cross, position: [0, 0, 0], quaternion: [0, 0, 0, 1] },
        ],
    });
}

export const compoundPile = defineScenario({
    name: 'compound-pile',
    description: 'PEEL PileOfSmallCompounds — 200 three-part compounds, sub-shape dispatch',
    steps: 120,
    create() {
        const rng = makeRng(RNG_SEED);
        const world = createStandardWorld();
        addGroundPlane(world);

        const variants: Shape[] = [];
        for (let i = 0; i < 8; i++) {
            variants.push(createCrossShape(0.4 + rng() * 0.2, 0.12 + rng() * 0.06));
        }

        for (let i = 0; i < COMPOUND_COUNT; i++) {
            const layer = Math.floor(i / (GRID * GRID));
            const withinLayer = i % (GRID * GRID);
            const x = ((withinLayer % GRID) - (GRID - 1) / 2) * SPACING;
            const z = (Math.floor(withinLayer / GRID) - (GRID - 1) / 2) * SPACING;
            rigidBody.create(world, {
                shape: variants[i % variants.length],
                objectLayer: LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [x + (rng() - 0.5) * 0.2, 0.8 + layer * 1.3, z + (rng() - 0.5) * 0.2],
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
