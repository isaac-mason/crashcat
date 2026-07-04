import { type Box3, type Vec3 } from 'mathcat';
import type { RigidBody } from '../body/rigid-body.js';
import type { Filter } from '../filter.js';
import type { World } from '../world.js';
import type { BodyVisitor } from './body-visitor.js';
export type DBVT = {
    nodes: DBVTNode[];
    freeNodeIndices: number[];
    root: number;
    expansionMargin: number;
    optimizationPath: number;
};
export type DBVTNode = {
    index: number;
    parent: number;
    left: number;
    right: number;
    aabb: Box3;
    height: number;
    bodyIndex: number;
};
export declare function create(): DBVT;
export declare function add(dbvt: DBVT, body: RigidBody): number;
export declare function remove(dbvt: DBVT, body: RigidBody): void;
/**
 * @optimize
 * returns true iff the leaf was reinserted (i.e. the body escaped its fat AABB), false when the
 * containment early-out fired or there was nothing to do. the persistent-pair broadphase treats a
 * reinsert as a "moved" event.
 */
export declare function update(dbvt: DBVT, body: RigidBody, lookahead: number): boolean;
export declare function optimizeBottomUp(dbvt: DBVT): void;
export declare function optimizeTopDown(dbvt: DBVT, buThreshold?: number): void;
export declare function optimizeIncremental(dbvt: DBVT, passes: number): void;
/**
 * Overlap traversal against the FAT leaf AABBs — no tight body-AABB re-test and no filtering.
 * Used by persistent-pair discovery: a pair must exist whenever the two fat boxes could
 * bring the bodies into contact while both coast inside them, so the leaf test must be the
 * fat node AABB (already tested during descent), NOT the current tight body AABB.
 */
export declare function intersectAABBFatLeaves(world: World, dbvt: DBVT, aabb: Box3, visitor: BodyVisitor): void;
export declare function intersectAABB(world: World, dbvt: DBVT, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void;
export declare function intersectPoint(world: World, dbvt: DBVT, point: Vec3, queryFilter: Filter, visitor: BodyVisitor): void;
export declare function walk(dbvt: DBVT, visitor: BodyVisitor, world: World): void;
export declare function castRay(world: World, dbvt: DBVT, origin: Vec3, direction: Vec3, length: number, queryFilter: Filter, visitor: BodyVisitor): void;
export declare function castAABB(world: World, dbvt: DBVT, bounds: Box3, displacement: Vec3, queryFilter: Filter, visitor: BodyVisitor): void;
/** get the bounds of the entire DBVT */
export declare function bounds(out: Box3, dbvt: DBVT): Box3;
