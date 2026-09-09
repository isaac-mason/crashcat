import { box, ConstraintSpace, hingeConstraint, MotionType, type RigidBody, rigidBody, type Shape, updateWorld } from 'crashcat';
import { addGroundPlane, createStandardWorld, LAYER_MOVING, LAYER_STATIC, TIME_STEP } from './common';
import { defineScenario } from './scenario';

// PEEL "HingeJointChain" (CATEGORY_JOINTS): 20 rows of 20 hinged boxes, each row anchored by a
// static first link and hanging out along +x. Long serial constraint chains are the hardest case
// for an iterative solver, and each row is its own island, so this isolates hinge constraint-part
// cost from contacts. PEEL disables collision between neighbouring links with a two-group filter
// (they are placed edge to edge); the same is done here with alternating collision groups.

const ROWS = 20;
const BOXES_PER_ROW = 20;
const HALF_EXTENTS: [number, number, number] = [1, 1, 2];
const ANCHOR_HEIGHT = 40;

// two groups that collide with everything except each other, so link i never collides with i+1
const GROUP_A = 0x1;
const GROUP_B = 0x2;
const MASK_A = ~GROUP_B >>> 0;
const MASK_B = ~GROUP_A >>> 0;

const linkShape: Shape = box.create({ halfExtents: HALF_EXTENTS });

export const hingeChain = defineScenario({
    name: 'hinge-chain',
    description: 'PEEL HingeJointChain — 20 rows of 20 hinged links, serial constraint solve',
    steps: 100,
    create() {
        const world = createStandardWorld();
        addGroundPlane(world);

        for (let row = 0; row < ROWS; row++) {
            const z = row * HALF_EXTENTS[2] * 4;
            let previous: RigidBody | undefined;

            for (let i = 0; i < BOXES_PER_ROW; i++) {
                const even = i % 2 === 0;
                const link = rigidBody.create(world, {
                    shape: linkShape,
                    objectLayer: i === 0 ? LAYER_STATIC : LAYER_MOVING,
                    motionType: i === 0 ? MotionType.STATIC : MotionType.DYNAMIC,
                    position: [i * HALF_EXTENTS[0] * 2, ANCHOR_HEIGHT, z],
                    mass: 1,
                    friction: 0.5,
                    restitution: 0,
                    collisionGroups: even ? GROUP_A : GROUP_B,
                    collisionMask: even ? MASK_A : MASK_B,
                });

                if (previous) {
                    hingeConstraint.create(world, {
                        bodyIdA: previous.id,
                        bodyIdB: link.id,
                        pointA: [HALF_EXTENTS[0], 0, 0],
                        pointB: [-HALF_EXTENTS[0], 0, 0],
                        hingeAxisA: [0, 0, 1],
                        hingeAxisB: [0, 0, 1],
                        normalAxisA: [1, 0, 0],
                        normalAxisB: [1, 0, 0],
                        space: ConstraintSpace.LOCAL,
                    });
                }

                previous = link;
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
