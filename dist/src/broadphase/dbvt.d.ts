import { type Vec3 } from 'mathcat';
import { type Box3 } from 'mathcat/shapes';
import type { RigidBody } from '../body/rigid-body.js';
import type { Filter } from '../filter.js';
import type { World } from '../world.js';
import type { BodyVisitor } from './body-visitor.js';
export type DBVT = {
    /** packed-double node bounds, stride 6 (see file header) */
    bounds: number[];
    /** packed-SMI node topology, stride 5 (see file header) */
    topo: number[];
    freeNodeIndices: number[];
    root: number;
    /** the fat-leaf expansion margin (in world units) applied to each leaf AABB */
    expansionMargin: number;
    /** rebuild marks the top N levels `changed` so they always re-partition */
    maxDepthMarkChanged: number;
    /**
     * any structural change (add / remove / fat-leaf escape) since the last rebuild. gates
     * the dirty-gated rebuild in broadphase.optimize — a tree that no body has disturbed
     * (e.g. a settled static field) stays clean and is never rebuilt.
     */
    dirty: boolean;
};
export declare function create(): DBVT;
export declare function add(dbvt: DBVT, body: RigidBody): number;
export declare function remove(dbvt: DBVT, body: RigidBody): void;
/**
 * @optimize
 * returns true iff the body escaped its fat AABB (a "moved" event the persistent-pair broadphase
 * consumes), false when the containment early-out fired.
 *
 * widen-in-place + defer: on escape we REPLACE the leaf's fat box (may shrink — the persistent-pair
 * sweep keys keep/destroy on the fat leaf boxes, so they must track the body, not grow forever), then
 * grow ancestors (grow-only until rebuild) and mark the path changed. no remove/reinsert — the
 * balanced restructure is deferred to the next rebuild. per-move cost is O(depth), typically O(1).
 */
export declare function update(dbvt: DBVT, body: RigidBody): boolean;
/**
 * Rebuild the tree into a balanced structure and clear the dirty flag. Leaf-preserving and
 * partial: unchanged internal subtrees graft in whole (indices untouched); only changed internals
 * are recycled. O(m log m) in the changed unit count m, not the body count.
 */
export declare function rebuild(dbvt: DBVT): void;
/**
 * Overlap traversal against the FAT leaf AABBs — no tight body-AABB re-test and no filtering.
 * Used by persistent-pair discovery: a pair must exist whenever the two fat boxes could
 * bring the bodies into contact while both coast inside them, so the leaf test must be the
 * fat node AABB (already tested during descent), NOT the current tight body AABB.
 *
 * @optimize
 */
export declare function intersectAABBFatLeaves(world: World, dbvt: DBVT, aabb: Box3, visitor: BodyVisitor): void;
/** @optimize */
export declare function intersectAABB(world: World, dbvt: DBVT, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void;
/** @optimize */
export declare function intersectPoint(world: World, dbvt: DBVT, point: Vec3, queryFilter: Filter, visitor: BodyVisitor): void;
/** visit every body in the tree — no filtering, no aabb tests */
export declare function walk(world: World, dbvt: DBVT, visitor: BodyVisitor): void;
/** @optimize */
export declare function castRay(world: World, dbvt: DBVT, origin: Vec3, direction: Vec3, length: number, queryFilter: Filter, visitor: BodyVisitor): void;
/** @optimize */
export declare function castAABB(world: World, dbvt: DBVT, castBounds: Box3, displacement: Vec3, queryFilter: Filter, visitor: BodyVisitor): void;
/** get the bounds of the entire DBVT */
export declare function bounds(out: Box3, dbvt: DBVT): Box3;
/** number of allocated node rows (including free-listed rows), i.e. the node pool high-water mark */
export declare function nodeCount(dbvt: DBVT): number;
/** copy node `n`'s fat AABB into `out` (out param first) */
export declare function readNodeAabb(out: Box3, dbvt: DBVT, n: number): Box3;
export declare function nodeParent(dbvt: DBVT, n: number): number;
export declare function nodeLeft(dbvt: DBVT, n: number): number;
export declare function nodeRight(dbvt: DBVT, n: number): number;
export declare function nodeBodyIndex(dbvt: DBVT, n: number): number;
export declare function nodeChanged(dbvt: DBVT, n: number): boolean;
