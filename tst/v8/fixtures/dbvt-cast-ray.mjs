// Hot-path fixture: dbvt.castRay traversing a 6×6×6 grid of bodies.
//
// The ray is aimed straight through one row so the BVH walk, internal-node
// distance sort, ray-AABB prune, the per-leaf filter ladder, and the
// monomorphic visitor.visit call are all exercised every iteration.

import { vec3 } from 'mathcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    dbvt,
    enableCollision,
    filter,
    MotionType,
    registerAll,
    rigidBody,
} from '../../../dist/index.js';

registerAll();

function createTestWorld() {
    const s = createWorldSettings();
    const BPH_MOVING = addBroadphaseLayer(s);
    const BPH_STATIC = addBroadphaseLayer(s);
    const OL_MOVING = addObjectLayer(s, BPH_MOVING);
    const OL_STATIC = addObjectLayer(s, BPH_STATIC);
    enableCollision(s, OL_MOVING, OL_MOVING);
    enableCollision(s, OL_MOVING, OL_STATIC);
    return createWorld(s);
}

const world = createTestWorld();
const tree = dbvt.create();

const SIZE = 6;
for (let i = 0; i < SIZE; i++) {
    for (let j = 0; j < SIZE; j++) {
        for (let k = 0; k < SIZE; k++) {
            dbvt.add(tree, rigidBody.create(world, {
                shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
                motionType: MotionType.DYNAMIC,
                objectLayer: 0,
                position: vec3.fromValues(i * 2, j * 2, k * 2),
            }));
        }
    }
}

const origin = vec3.fromValues(-5, 4, 4);
const direction = vec3.fromValues(1, 0, 0);
const length = 100;
const queryFilter = filter.create(world.settings.layers);
const visitor = {
    shouldExit: false,
    visit: (_body) => {},
};

export const name = 'dbvt.castRay';
export const fn = dbvt.castRay;
export const exercise = () =>
    dbvt.castRay(world, tree, origin, direction, length, queryFilter, visitor);
