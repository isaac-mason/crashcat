import {
    ConstraintSpace,
    capsule,
    MotionType,
    type RigidBody,
    rigidBody,
    type Shape,
    swingTwistConstraint,
    updateWorld,
} from 'crashcat';
import { addGroundPlane, captureBodyState, createStandardWorld, LAYER_MOVING, restoreBodyState, TIME_STEP } from './common';
import { defineScenario } from './scenario';

// PEEL "PileOfRagdolls_16" (CATEGORY_JOINTS), via GenerateColumnOfRagdolls(16, 1): a column of 16
// jointed figures spawned two apart so they interpenetrate and collapse into a heap. PEEL gives
// each ragdoll its own collision group so its own bones never collide with each other while
// ragdolls still collide with each other, and the same is done here.
//
// PEEL's figure is 19 bones on 18 hinges. This uses a torso with four limb chains on swing-twist
// joints instead: the same body and constraint count per figure, on the constraint type a ragdoll
// actually wants, and one that hinge-chain does not already cover.

const RAGDOLL_COUNT = 16;
const LIMB_HALF_HEIGHT = 0.2;
const LIMB_RADIUS = 0.1;
const LIMB_LENGTH = 2 * (LIMB_HALF_HEIGHT + LIMB_RADIUS);
const CHAINS_PER_RAGDOLL = 4;
const LIMBS_PER_CHAIN = 4;
const SPAWN_SPACING = 2;
const CONE_ANGLE = Math.PI / 4;

const limbShape: Shape = capsule.create({ halfHeightOfCylinder: LIMB_HALF_HEIGHT, radius: LIMB_RADIUS });

export const ragdollPile = defineScenario({
    name: 'ragdoll-pile',
    description: 'PEEL PileOfRagdolls_16 — 16 jointed figures collapsing into a heap',
    steps: 120,
    create() {
        const world = createStandardWorld();
        addGroundPlane(world);

        for (let r = 0; r < RAGDOLL_COUNT; r++) {
            // one bit per ragdoll: its bones share a group and mask it out, so they pass through
            // each other but still collide with every other ragdoll and the ground
            const group = 1 << (r % 31);
            const mask = ~group >>> 0;
            const originY = SPAWN_SPACING + r * SPAWN_SPACING;

            const torso = rigidBody.create(world, {
                shape: limbShape,
                objectLayer: LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [0, originY, 0],
                mass: 4,
                friction: 0.5,
                restitution: 0,
                collisionGroups: group,
                collisionMask: mask,
            });

            for (let c = 0; c < CHAINS_PER_RAGDOLL; c++) {
                const angle = (c / CHAINS_PER_RAGDOLL) * Math.PI * 2;
                const dirX = Math.cos(angle);
                const dirZ = Math.sin(angle);
                let parent: RigidBody = torso;

                for (let l = 0; l < LIMBS_PER_CHAIN; l++) {
                    const distance = (l + 1) * LIMB_LENGTH;
                    const child = rigidBody.create(world, {
                        shape: limbShape,
                        objectLayer: LAYER_MOVING,
                        motionType: MotionType.DYNAMIC,
                        position: [dirX * distance, originY, dirZ * distance],
                        mass: 1,
                        friction: 0.5,
                        restitution: 0,
                        collisionGroups: group,
                        collisionMask: mask,
                    });

                    swingTwistConstraint.create(world, {
                        bodyIdA: parent.id,
                        bodyIdB: child.id,
                        position1: [0, 0, LIMB_LENGTH * 0.5],
                        position2: [0, 0, -LIMB_LENGTH * 0.5],
                        twistAxis1: [0, 0, 1],
                        planeAxis1: [1, 0, 0],
                        twistAxis2: [0, 0, 1],
                        planeAxis2: [1, 0, 0],
                        space: ConstraintSpace.LOCAL,
                        normalHalfConeAngle: CONE_ANGLE,
                        planeHalfConeAngle: CONE_ANGLE,
                        twistMinAngle: -CONE_ANGLE,
                        twistMaxAngle: CONE_ANGLE,
                    });

                    parent = child;
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
