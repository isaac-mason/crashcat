import { box, MotionType, rigidBody, type Shape, updateWorld } from 'crashcat';
import { addGroundPlane, createStandardWorld, LAYER_MOVING, TIME_STEP } from './common';
import { defineScenario } from './scenario';

// PEEL "ManySmallBoxStacks10" (CATEGORY_PERFORMANCE), via its CreateBoxStack(nb_stacks, base=10)
// helper: a row of box pyramids, ten boxes at the base narrowing to one. Many independent resting
// islands, so it loads island management and the contact solver's stacking behaviour at once.

const BOX_HALF_EXTENT = 1;
const BASE_BOXES = 10;
const STACK_SPACING = BOX_HALF_EXTENT * 4;
const STACKS = 30;

const boxShape: Shape = box.create({ halfExtents: [BOX_HALF_EXTENT, BOX_HALF_EXTENT, BOX_HALF_EXTENT] });

export const boxStacks = defineScenario({
    name: 'box-stacks',
    description: 'PEEL ManySmallBoxStacks10 — 30 box pyramids, island management plus stacking',
    steps: 10,
    create() {
        const world = createStandardWorld();
        addGroundPlane(world);

        for (let stack = 0; stack < STACKS; stack++) {
            const z = stack * STACK_SPACING;
            let rowBoxes = BASE_BOXES;
            let y = BOX_HALF_EXTENT;
            while (rowBoxes > 0) {
                for (let i = 0; i < rowBoxes; i++) {
                    const x = (i - rowBoxes * 0.5) * BOX_HALF_EXTENT * 2;
                    rigidBody.create(world, {
                        shape: boxShape,
                        objectLayer: LAYER_MOVING,
                        motionType: MotionType.DYNAMIC,
                        position: [x, y, z],
                        mass: 1,
                        friction: 0.5,
                        restitution: 0,
                    });
                }
                rowBoxes--;
                y += BOX_HALF_EXTENT * 2;
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
