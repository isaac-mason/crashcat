import { type Box3, box3, type Vec3 } from 'mathcat';
import type { RigidBody } from '../body/rigid-body';
import type { Filter } from '../filter';
import * as filter from '../filter';
import type { Layers } from '../layers';
import { assert } from '../utils/assert';
import type { World } from '../world';
import type { BodyVisitor } from './body-visitor';
import * as dbvt from './dbvt';

/** broadphase state for a physics world */
export type Broadphase = {
    /** dynamic bounding volume trees, one per broadphase layer */
    dbvts: dbvt.DBVT[];
    /** round-robin cursor for the dirty-gated rebuild (jolt's mNextLayerToUpdate) */
    nextTreeToOptimize: number;
};

/** initializes broadphase state */
export function init(layers: Layers): Broadphase {
    const numBroadphaseLayers = layers.broadphaseLayers;
    const dbvts: dbvt.DBVT[] = [];

    // create one DBVH per broadphase layer
    for (let i = 0; i < numBroadphaseLayers; i++) {
        dbvts.push(
            // future: options?
            dbvt.create(),
        );
    }

    return {
        dbvts,
        nextTreeToOptimize: 0,
    };
}

/** adds a body to the broadphase */
export function addBody(broadphase: Broadphase, body: RigidBody, layers: Layers): void {
    const objectLayer = body.objectLayer;

    // map object layer to broadphase layer
    const broadphaseLayer = layers.objectLayerToBroadphaseLayer[objectLayer];

    if (broadphaseLayer === undefined) {
        assert(false, `Object layer ${objectLayer} not mapped to broadphase layer`);
        return;
    }

    // get the dbvt for this broadphase layer
    const tree = broadphase.dbvts[broadphaseLayer];

    // insert into dbvt
    const node = dbvt.add(tree, body);

    // store broadphase layer and dbvt node on the body
    body.broadphaseLayer = broadphaseLayer;
    body.dbvtNode = node;
}

/** removes a body from the broadphase */
export function removeBody(broadphase: Broadphase, body: RigidBody): void {
    if (body.broadphaseLayer === -1) return;

    const tree = broadphase.dbvts[body.broadphaseLayer];

    if (body.dbvtNode !== -1) {
        dbvt.remove(tree, body);
    }

    body.broadphaseLayer = -1;
    body.dbvtNode = -1;
}

/**
 * Dirty-gated balanced rebuild (jolt's BroadPhaseQuadTree::UpdatePrepare). Called once per step by
 * updateWorld, before pair finding. Scans layers from the round-robin cursor and rebuilds the first
 * dirty tree found — at most one rebuild per step to cap worst-frame cost. A clean tree (e.g. a
 * settled static field) costs a single boolean check and is never touched; this replaces Bullet's
 * unconditional per-frame incremental rotation, which never balanced a large static tree.
 */
export function optimize(broadphase: Broadphase): void {
    const n = broadphase.dbvts.length;
    for (let i = 0; i < n; i++) {
        const idx = (broadphase.nextTreeToOptimize + i) % n;
        const tree = broadphase.dbvts[idx];
        if (tree.dirty) {
            dbvt.rebuild(tree);
            broadphase.nextTreeToOptimize = (idx + 1) % n;
            return;
        }
    }
}

/** updates a body's AABB in the broadphase; returns true iff the body escaped its fat leaf AABB */
export function updateBody(broadphase: Broadphase, body: RigidBody): boolean {
    if (body.dbvtNode === -1 || body.broadphaseLayer === -1) return false;
    const tree = broadphase.dbvts[body.broadphaseLayer];

    // an escape is a move event: the caller marks the body moved (pairs.markMoved) so it
    // rediscovers its overlaps in the next findCollidingPairs.
    return dbvt.update(tree, body);
}

/** removes and re-adds a body in the broadphase when its layer changes */
export function reinsertBody(broadphase: Broadphase, body: RigidBody, layers: Layers): void {
    removeBody(broadphase, body);
    addBody(broadphase, body, layers);
}

/** finds bodies with AABBs that intersect the given ray */
export function castRay(
    world: World,
    origin: Vec3,
    direction: Vec3,
    length: number,
    queryFilter: Filter,
    visitor: BodyVisitor,
): void {
    // query each broadphase layer that passes the filter
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.castRay(world, tree, origin, direction, length, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** finds bodies with AABBs that intersect the given AABB */
export function intersectAABB(world: World, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void {
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.intersectAABB(world, tree, aabb, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** finds bodies with AABBs that contain the given point */
export function intersectPoint(world: World, point: Vec3, queryFilter: Filter, visitor: BodyVisitor): void {
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.intersectPoint(world, tree, point, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** finds bodies with AABBs that intersect the given swept AABB */
export function castAABB(world: World, bounds: Box3, displacement: Vec3, queryFilter: Filter, visitor: BodyVisitor): void {
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.castAABB(world, tree, bounds, displacement, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** get the bounds of all DBVTs in the broadphase */
export function bounds(out: Box3, broadphase: Broadphase): Box3 {
    box3.empty(out);

    for (const tree of broadphase.dbvts) {
        if (tree.root === -1) continue;

        const rootNode = tree.nodes[tree.root];
        box3.union(out, out, rootNode.aabb);
    }

    return out;
}
