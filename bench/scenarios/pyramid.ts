import { box, MotionType, rigidBody, type Shape, updateWorld } from 'crashcat';
import { addGroundPlane, captureBodyState, createStandardWorld, LAYER_MOVING, restoreBodyState, TIME_STEP } from './common';
import { defineScenario } from './scenario';

// Jolt's PerformanceTest PyramidScene: one dense pyramid of boxes, with a heavy body fired down
// into it so the window covers an impact rather than just a settle. The deep contact graph of a
// single large island is the point — this is where solver iteration cost and stacking stability
// show up.

const BOX_SIZE = 1;
const BOX_HALF = BOX_SIZE * 0.5;
const HEIGHT = 8;
const BALL_DROP_SPEED = 30;

const boxShape: Shape = box.create({ halfExtents: [BOX_HALF, BOX_HALF, BOX_HALF] });
const ballShape: Shape = box.create({ halfExtents: [1.5, 1.5, 1.5] });

export const pyramid = defineScenario({
    name: 'pyramid',
    description: 'Jolt PerformanceTest PyramidScene — one deep contact island, plus a dropped weight',
    steps: 15,
    create() {
        const world = createStandardWorld();
        addGroundPlane(world);

        for (let y = 0; y < HEIGHT; y++) {
            const offset = HEIGHT - 1 - y;
            for (let x = -offset; x <= offset; x++) {
                for (let z = -offset; z <= offset; z++) {
                    rigidBody.create(world, {
                        shape: boxShape,
                        objectLayer: LAYER_MOVING,
                        motionType: MotionType.DYNAMIC,
                        position: [x * BOX_SIZE, y * BOX_SIZE + BOX_HALF, z * BOX_SIZE],
                        mass: 1,
                        friction: 0.5,
                        restitution: 0,
                    });
                }
            }
        }

        const ball = rigidBody.create(world, {
            shape: ballShape,
            objectLayer: LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: [0, HEIGHT * BOX_SIZE + 4, 0],
            mass: 50,
            friction: 0.5,
            restitution: 0.3,
        });

        const captured = captureBodyState(world);

        return {
            world,
            reset() {
                restoreBodyState(world, captured);
                // restoreBodyState leaves everything at rest, so the ball is re-launched here
                rigidBody.setLinearVelocity(world, ball, [0, -BALL_DROP_SPEED, 0]);
            },
            step() {
                updateWorld(world, undefined, TIME_STEP);
            },
        };
    },
});
