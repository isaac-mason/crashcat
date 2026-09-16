import { box, convexHull, MotionQuality, MotionType, rigidBody, type Shape, updateWorld } from 'crashcat';
import {
    addGroundPlane,
    captureBodyState,
    convexBlobPositions,
    createStandardWorld,
    LAYER_MOVING,
    makeRng,
    restoreBodyState,
    TIME_STEP,
} from './common';
import { defineScenario } from './scenario';

// PEEL "CCDTest_DynamicDynamic_ConvexCascade" (CATEGORY_CCD): a column of 100 dynamic bodies
// alternating between a wide thin box and a convex hull, spaced two apart, all falling into each
// other at once. Thin plates are exactly what discrete collision tunnels through, so every body
// runs on linear-cast motion quality and the post-solve CCD sweep is the point of the scenario.
//
// `steps` stops at 150 on purpose. The column takes about 385 steps to finish falling, and past
// that the window is dominated by a deep pile of interleaved plates rather than by bodies moving
// fast enough for CCD to do anything — which cost 8x as much per step and measured the wrong thing.

const BODY_COUNT = 100;
const SPACING = 2;
const RNG_SEED = 0x7feb352d;

const plateShape: Shape = box.create({ halfExtents: [10, 0.1, 10] });

export const ccdCascade = defineScenario({
    name: 'ccd-cascade',
    description: 'PEEL CCDTest ConvexCascade — 100 linear-cast bodies, thin plates alternating with hulls',
    steps: 150,
    create() {
        const rng = makeRng(RNG_SEED);
        const world = createStandardWorld();
        addGroundPlane(world, 50);

        const hullShape: Shape = convexHull.create({
            positions: convexBlobPositions(24, 1.5, rng),
            convexRadius: 0.05,
        });

        for (let i = 0; i < BODY_COUNT; i++) {
            rigidBody.create(world, {
                shape: i % 2 === 1 ? hullShape : plateShape,
                objectLayer: LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                motionQuality: MotionQuality.LINEAR_CAST,
                position: [0, 1 + i * SPACING, 0],
                mass: 1,
                friction: 0.5,
                restitution: 0,
            });
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
