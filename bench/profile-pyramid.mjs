// run with: node --prof --no-turbo-inlining bench/profile-pyramid.mjs
// then: node --prof-process isolate-*.log > profile.txt
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAll,
    rigidBody,
    updateWorld,
} from '../dist';

registerAll();

const pyramidHeight = 10;
const steps = 60;

const ws = createWorldSettings();
const BP_MOVING = addBroadphaseLayer(ws);
const BP_STATIC = addBroadphaseLayer(ws);
const OL_MOVING = addObjectLayer(ws, BP_MOVING);
const OL_STATIC = addObjectLayer(ws, BP_STATIC);
enableCollision(ws, OL_MOVING, OL_MOVING);
enableCollision(ws, OL_MOVING, OL_STATIC);

const world = createWorld(ws);

rigidBody.create(world, {
    motionType: MotionType.STATIC,
    objectLayer: OL_STATIC,
    shape: box.create({ halfExtents: [50, 1, 50] }),
    position: [0, -1, 0],
});

const boxSize = 2.0;
const boxSep = 0.5;
const half = boxSize * 0.5;
const cubeShape = box.create({ halfExtents: [half, half, half], convexRadius: 0 });

let bodyCount = 0;
for (let i = 0; i < pyramidHeight; i++) {
    for (let j = Math.floor(i / 2); j < pyramidHeight - Math.ceil(i / 2); j++) {
        for (let k = Math.floor(i / 2); k < pyramidHeight - Math.ceil(i / 2); k++) {
            const x = -pyramidHeight + boxSize * j + (i & 1 ? half : 0);
            const y = 1.0 + (boxSize + boxSep) * i;
            const z = -pyramidHeight + boxSize * k + (i & 1 ? half : 0);
            rigidBody.create(world, {
                motionType: MotionType.DYNAMIC,
                objectLayer: OL_MOVING,
                shape: cubeShape,
                position: [x, y, z],
                mass: 1,
                allowSleeping: false,
            });
            bodyCount++;
        }
    }
}

console.log(`bodies: ${bodyCount}, steps: ${steps}`);

const start = performance.now();
const dt = 1 / 60;
for (let i = 0; i < steps; i++) {
    updateWorld(world, undefined, dt);
}
const elapsed = performance.now() - start;
console.log(`elapsed: ${elapsed.toFixed(1)}ms`);
