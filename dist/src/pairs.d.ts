import { type Quat, type Vec3 } from 'mathcat';
import type { RigidBody } from './body/rigid-body.js';
import { type Contacts } from './contacts.js';
import type { Listener } from './listener.js';
import type { World } from './world.js';
/**
 * persistent overlapping-pair state for a physics world.
 *
 * owns the persistent pair set (discovered by the broadphase move-set), its per-record body-pair
 * relative-pose cache (folded in from the old contacts.cachedBodyPairs map), and the per-frame
 * emitted narrowphase input.
 *
 * records are linked into per-body intrusive pair lists (body.headPairKey, mirroring the contact
 * edge lists in contacts.ts) — dedup and lookup walk a body's list instead of a key map, keeping
 * the whole state plain flat arrays (JSON-serializable, no Map).
 */
export type Pairs = {
    /**
     * persistent overlapping-pair records, flat with stride RECORD_STRIDE (6):
     * [bodyIndexA, bodyIndexB, prevEdgeA, nextEdgeA, prevEdgeB, nextEdgeB].
     * the prev/next slots hold packed pair edge keys (pairEdgeKey) linking the record into each
     * body's intrusive pair list; INVALID_PAIR_KEY marks a list end.
     * storage is a freelist with STABLE record indices (no swap-remove): a free slot is marked
     * bodyIndexA === -1 and its index sits in freeRecords for reuse.
     */
    records: number[];
    /** free record indices available for reuse */
    freeRecords: number[];
    /** number of LIVE records (records.length / RECORD_STRIDE counts slots: live + free) */
    recordCount: number;
    /**
     * per-record last-narrowphase pose cache, flat with stride CACHE_STRIDE (8), indexed by record
     * (parallel to records): [valid, dpx, dpy, dpz, drx, dry, drz, drw]. deltaPosition is body B's
     * COM relative to body A expressed in body A's local frame; deltaRotation is `inv(rA) * rB`.
     * pose slots are meaningful only when the valid flag is 1. free record slots simply go stale —
     * addPairRecord resets valid=0 on reuse.
     */
    poseCache: number[];
    /**
     * this frame's findCollidingPairs output — the pooled narrowphase input, flat with stride 3:
     * [bodyIndexA, bodyIndexB, pairRecordIndex]. collidingPairCount valid pairs. record indices
     * are stable for the whole frame (freelist storage — records never move).
     */
    collidingPairs: number[];
    /**
     * per-record chain head: the contact index at the head of each record's contact chain, or
     * INVALID_CONTACT_KEY when the record has no contacts. parallel-by-record like poseCache (NOT a
     * records-stride slot — this keeps contacts.ts free of any pair layout constant, so its only
     * dependency on pairs is the erased `Pairs` type). addPairRecord resets the head to empty.
     */
    firstContact: number[];
    /** number of colliding pairs emitted this frame */
    collidingPairCount: number;
    /**
     * the move set: indices of bodies whose fat leaf changed (escaped, added, or changed layer),
     * awaiting pair discovery. consumed by findCollidingPairs. deduped via body.inMoveSet
     * (invariant: flag set ⟺ index present here); insertion order is deterministic.
     */
    moved: number[];
};
/** stride of a pair record: [bodyIndexA, bodyIndexB, prevEdgeA, nextEdgeA, prevEdgeB, nextEdgeB] */
export declare const RECORD_STRIDE = 6;
/** invalid pair edge key constant - used to mark the end of a body's pair list */
export declare const INVALID_PAIR_KEY = -1;
/** stride of a cache record: [valid, dpx, dpy, dpz, drx, dry, drz, drw] */
export declare const CACHE_STRIDE = 8;
/** offset of the 0/1 valid flag within a cache record */
export declare const CACHE_VALID = 0;
/** offset of deltaPosition (3 slots) within a cache record */
export declare const CACHE_DP = 1;
/** offset of deltaRotation (4 slots) within a cache record */
export declare const CACHE_DR = 4;
/** initializes pairs state */
export declare function init(): Pairs;
/** add a body to the move set (deduped) so it runs pair discovery in the next findCollidingPairs */
export declare function markMoved(pairs: Pairs, body: RigidBody): void;
/**
 * Pack a record index and side into a single pair edge key.
 * Layout: [recordIndex][side: 1 bit], side 0 = bodyA's edge, side 1 = bodyB's edge.
 */
export declare function pairEdgeKey(recordIndex: number, side: 0 | 1): number;
/** extract the record index from a packed pair edge key */
export declare function getPairEdgeRecord(key: number): number;
/** extract the side from a packed pair edge key (0 = bodyA's edge, 1 = bodyB's edge) */
export declare function getPairEdgeSide(key: number): 0 | 1;
/**
 * add a persistent pair record and link it into both bodies' pair lists.
 * exported for the CCD find-or-create path (a CCD hit can involve a pair the sweep never discovered).
 */
export declare function addPairRecord(pairs: Pairs, bodyA: RigidBody, bodyB: RigidBody): number;
/** write last-narrowphase relative pose into the record's pose-cache block (marks it valid) */
export declare function setCache(pairs: Pairs, rec: number, deltaPosition: Vec3, deltaRotation: Quat): void;
/**
 * find the persistent pair record for a body pair by walking bodyA's pair list — O(pair degree).
 * returns the record index, or -1 if no record exists. (discovery dedup and the CCD find-or-create
 * path both use this.)
 */
export declare function findPairRecord(pairs: Pairs, bodyA: RigidBody, bodyB: RigidBody): number;
/**
 * destroy every persistent pair involving this body by walking its own pair list — O(pair degree).
 * each record's contact chain is cascaded (removal events queued onto pendingContactRemoved, since
 * the listener is unavailable at body-removal time). does not depend on the body's dbvt leaf, so it
 * is safe to call before or independently of tree-leaf removal.
 */
export declare function purgeBodyPairs(pairs: Pairs, contacts: Contacts, pool: RigidBody[], body: RigidBody): void;
/**
 * find potentially colliding body pairs, updates world.pairs.collidingPairs.
 *
 * moved-only persistent-pair broadphase: a persistent pair set is populated by fat-AABB queries from
 * bodies that moved (escaped their fat leaf) or were added, then swept every frame. the emitted pairs
 * array is semantically identical to the old per-active-body scan (same pair set each frame;
 * intra-pair/record order may differ).
 */
export declare function findCollidingPairs(world: World, speculativeContactDistance: number, listener: Listener | undefined): void;
