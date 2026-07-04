import { box3, type Quat, type Vec3 } from 'mathcat';
import { MotionType } from './body/motion-type';
import type { RigidBody } from './body/rigid-body';
import { INACTIVE_BODY_INDEX } from './body/sleep';
import type { BodyVisitor } from './broadphase/body-visitor';
import * as dbvt from './broadphase/dbvt';
import { type Contacts, destroyPairChain, INVALID_CONTACT_KEY } from './contacts';
import * as filter from './filter';
import {
    broadphaseLayerCollidesWithBroadphaseLayer,
    type Layers,
    objectLayerCollidesWithBroadphaseLayer,
    objectLayerCollidesWithObjectLayer,
} from './layers';
import type { Listener } from './listener';
import type { World } from './world';

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
export const RECORD_STRIDE = 6;

/** invalid pair edge key constant - used to mark the end of a body's pair list */
export const INVALID_PAIR_KEY = -1;

/** stride of a cache record: [valid, dpx, dpy, dpz, drx, dry, drz, drw] */
export const CACHE_STRIDE = 8;

/** offset of the 0/1 valid flag within a cache record */
export const CACHE_VALID = 0;

/** offset of deltaPosition (3 slots) within a cache record */
export const CACHE_DP = 1;

/** offset of deltaRotation (4 slots) within a cache record */
export const CACHE_DR = 4;

/** initializes pairs state */
export function init(): Pairs {
    return {
        records: [],
        freeRecords: [],
        recordCount: 0,
        poseCache: [],
        collidingPairs: [],
        firstContact: [],
        collidingPairCount: 0,
        moved: [],
    };
}

/** add a body to the move set (deduped) so it runs pair discovery in the next findCollidingPairs */
export function markMoved(pairs: Pairs, body: RigidBody): void {
    if (body.inMoveSet) return;
    body.inMoveSet = true;
    pairs.moved.push(body.index);
}

/**
 * Pack a record index and side into a single pair edge key.
 * Layout: [recordIndex][side: 1 bit], side 0 = bodyA's edge, side 1 = bodyB's edge.
 */
export function pairEdgeKey(recordIndex: number, side: 0 | 1): number {
    return recordIndex * 2 + side;
}

/** extract the record index from a packed pair edge key */
export function getPairEdgeRecord(key: number): number {
    return key >> 1;
}

/** extract the side from a packed pair edge key (0 = bodyA's edge, 1 = bodyB's edge) */
export function getPairEdgeSide(key: number): 0 | 1 {
    return (key & 1) as 0 | 1;
}

/** flat slot of the prev pointer for a record's edge on the given side (the next pointer is the slot after) */
function edgePrevSlot(recordIndex: number, side: number): number {
    return recordIndex * RECORD_STRIDE + 2 + side * 2;
}

// Type/role eligibility gate — whether two bodies can EVER collide given their motion types, sensor
// roles, and the kinematic-vs-non-dynamic opt-in, independent of position/layer/group. One must hold:
// a body is dynamic, a body opted into kinematic-vs-non-dynamic, or it's a kinematic-vs-sensor pair.
// Two non-dynamic bodies (static-static, static-kinematic without opt-in, …) can never collide.
// crashcat's port of jolt's `Body::sFindCollidingPairsCanCollide` (minus its id-ordering dedup, which
// crashcat handles separately). Shared between pair discovery (skip creating a dead record) and
// emission (skip reporting) so the two gates can't drift.
function bodiesCanCollide(a: RigidBody, b: RigidBody): boolean {
    return (
        a.collideKinematicVsNonDynamic ||
        b.collideKinematicVsNonDynamic ||
        a.motionType === MotionType.DYNAMIC ||
        b.motionType === MotionType.DYNAMIC ||
        (a.motionType === MotionType.KINEMATIC && b.sensor) ||
        (b.motionType === MotionType.KINEMATIC && a.sensor)
    );
}

/**
 * The pair-emission predicate for findCollidingPairs' sweep: decides whether a persistent record
 * emits an (activeBody, otherBody) narrowphase pair this frame. Layers/groups + the shared
 * {@link bodiesCanCollide} type gate + activeIndex dedup (jolt: Body::sFindCollidingPairsCanCollide
 * + ObjectLayerPairFilter).
 */
function shouldReportPair(layers: Layers, activeBody: RigidBody, otherBody: RigidBody): boolean {
    // self-collision + deduplication: report only when activeBody.activeIndex < otherBody.activeIndex.
    // sleeping and static bodies have activeIndex = INACTIVE_BODY_INDEX (the max value), so an
    // active body always reports pairs with them; two active bodies report exactly once. bodies
    // are woken only AFTER the pair loop, so the active list is frozen while querying — no
    // mid-step activation ordering cases exist (unlike jolt, which relies on activating bodies
    // appending to the end of the active list).
    if (activeBody.activeIndex >= otherBody.activeIndex) {
        return false;
    }

    // motion type / sensor gate
    if (!bodiesCanCollide(activeBody, otherBody)) {
        return false;
    }

    // object layer pair table
    if (!objectLayerCollidesWithObjectLayer(layers, activeBody.objectLayer, otherBody.objectLayer)) {
        return false;
    }

    // collision group / mask
    if (
        !filter.shouldPairCollide(
            activeBody.collisionGroups,
            activeBody.collisionMask,
            otherBody.collisionGroups,
            otherBody.collisionMask,
        )
    ) {
        return false;
    }

    return true;
}

/**
 * discovery visitor: fat-AABB query candidates for a moved body become persistent pair records.
 * semantics-free apart from the static object-layer pair prune — sleeping/kinematic/mask gates are
 * NOT applied here so the persistent set stays valid when those flags change later; they are applied
 * at emission time in the sweep. discovery never emits.
 */
const PairDiscoveryVisitor: BodyVisitor & {
    layers: Layers;
    pairs: Pairs;
    movedBody: RigidBody;
    setup(layers: Layers, pairs: Pairs, movedBody: RigidBody): void;
} = {
    shouldExit: false,

    // state configured via setup()
    layers: null!,
    pairs: null!,
    movedBody: null!,

    setup(layers: Layers, pairs: Pairs, movedBody: RigidBody): void {
        this.layers = layers;
        this.pairs = pairs;
        this.movedBody = movedBody;
        this.shouldExit = false;
    },

    visit(otherBody: RigidBody): void {
        const movedBody = this.movedBody;

        // self-skip
        if (otherBody.index === movedBody.index) return;

        // motion-type gate: two bodies that can't drive a collision (e.g. static-static) never need
        // a persistent record — emission (shouldReportPair) would only reject it. skipping it here
        // keeps the sweep off dead slots (a dense static field would otherwise accrue tens of
        // thousands). a later setMotionType re-marks the body moved, so the pair is (re)discovered
        // once a dynamic partner exists.
        if (!bodiesCanCollide(movedBody, otherBody)) return;

        // static prune only: object-layer pair table (a static gate; a layer change marks the
        // body moved for rediscovery, and the sweep re-applies the layer table at emission, so
        // stale records never emit)
        if (!objectLayerCollidesWithObjectLayer(this.layers, movedBody.objectLayer, otherBody.objectLayer)) {
            return;
        }

        // already tracked? O(pair degree), bounded by how many fat AABBs one body can overlap
        if (findPairRecord(this.pairs, movedBody, otherBody) !== -1) return;

        addPairRecord(this.pairs, movedBody, otherBody);
    },
};

/**
 * add a persistent pair record and link it into both bodies' pair lists.
 * exported for the CCD find-or-create path (a CCD hit can involve a pair the sweep never discovered).
 */
export function addPairRecord(pairs: Pairs, bodyA: RigidBody, bodyB: RigidBody): number {
    // allocate a record slot from the freelist, or grow
    let rec: number;
    if (pairs.freeRecords.length > 0) {
        rec = pairs.freeRecords.pop()!;
    } else {
        rec = pairs.records.length / RECORD_STRIDE;
    }

    const base = rec * RECORD_STRIDE;
    pairs.records[base] = bodyA.index;
    pairs.records[base + 1] = bodyB.index;

    // link each edge at the head of its body's pair list
    linkPairEdge(pairs, rec, 0, bodyA);
    linkPairEdge(pairs, rec, 1, bodyB);

    // a fresh record has no last-narrowphase pose and no contacts yet
    pairs.poseCache[rec * CACHE_STRIDE + CACHE_VALID] = 0;
    pairs.firstContact[rec] = INVALID_CONTACT_KEY;
    pairs.recordCount++;

    return rec;
}

/** link a record's edge at the head of the body's intrusive pair list */
function linkPairEdge(pairs: Pairs, rec: number, side: 0 | 1, body: RigidBody): void {
    const records = pairs.records;
    const prevSlot = edgePrevSlot(rec, side);
    records[prevSlot] = INVALID_PAIR_KEY;
    records[prevSlot + 1] = body.headPairKey;

    if (body.headPairKey !== INVALID_PAIR_KEY) {
        // fix the old head's prev pointer
        const headRec = getPairEdgeRecord(body.headPairKey);
        const headSide = getPairEdgeSide(body.headPairKey);
        records[edgePrevSlot(headRec, headSide)] = pairEdgeKey(rec, side);
    }

    body.headPairKey = pairEdgeKey(rec, side);
}

/** unlink a record's edge from its body's pair list, updating neighbours and the body's head */
function unlinkPairEdge(pairs: Pairs, pool: RigidBody[], rec: number, side: 0 | 1): void {
    const records = pairs.records;
    const prevSlot = edgePrevSlot(rec, side);
    const prevKey = records[prevSlot];
    const nextKey = records[prevSlot + 1];

    if (prevKey === INVALID_PAIR_KEY) {
        // this edge was the head - update the body's head pointer
        pool[records[rec * RECORD_STRIDE + side]].headPairKey = nextKey;
    } else {
        // update the previous edge's next pointer
        records[edgePrevSlot(getPairEdgeRecord(prevKey), getPairEdgeSide(prevKey)) + 1] = nextKey;
    }

    if (nextKey !== INVALID_PAIR_KEY) {
        // update the next edge's prev pointer
        records[edgePrevSlot(getPairEdgeRecord(nextKey), getPairEdgeSide(nextKey))] = prevKey;
    }

    // clear this edge
    records[prevSlot] = INVALID_PAIR_KEY;
    records[prevSlot + 1] = INVALID_PAIR_KEY;
}

/**
 * destroy the persistent pair record at the given record index. cascades the record's contact chain
 * FIRST (so the invariant "live contact ⟺ live record that lists it" holds — the chain is destroyed
 * before the slot can ever be reused), then unlinks both edges from their bodies' pair lists and
 * returns the slot to the freelist. record indices are stable — freed slots stay in place until
 * reused by addPairRecord.
 *
 * removal events for the cascaded contacts fire immediately on `listener` unless `queueEvents` is
 * true (the body-removal path, where the listener is unavailable — events go on pendingContactRemoved).
 */
function removePairRecordAt(
    pairs: Pairs,
    contacts: Contacts,
    pool: RigidBody[],
    rec: number,
    listener: Listener | undefined,
    queueEvents: boolean,
): void {
    destroyPairChain(contacts, pairs, rec, listener, queueEvents);

    unlinkPairEdge(pairs, pool, rec, 0);
    unlinkPairEdge(pairs, pool, rec, 1);

    // bodyIndexA === -1 marks a free slot
    pairs.records[rec * RECORD_STRIDE] = -1;
    pairs.freeRecords.push(rec);
    pairs.recordCount--;
}

/** write last-narrowphase relative pose into the record's pose-cache block (marks it valid) */
export function setCache(pairs: Pairs, rec: number, deltaPosition: Vec3, deltaRotation: Quat): void {
    const poseCache = pairs.poseCache;
    const base = rec * CACHE_STRIDE;

    poseCache[base + CACHE_VALID] = 1;

    poseCache[base + CACHE_DP] = deltaPosition[0];
    poseCache[base + CACHE_DP + 1] = deltaPosition[1];
    poseCache[base + CACHE_DP + 2] = deltaPosition[2];

    poseCache[base + CACHE_DR] = deltaRotation[0];
    poseCache[base + CACHE_DR + 1] = deltaRotation[1];
    poseCache[base + CACHE_DR + 2] = deltaRotation[2];
    poseCache[base + CACHE_DR + 3] = deltaRotation[3];
}

/**
 * find the persistent pair record for a body pair by walking bodyA's pair list — O(pair degree).
 * returns the record index, or -1 if no record exists. (discovery dedup and the CCD find-or-create
 * path both use this.)
 */
export function findPairRecord(pairs: Pairs, bodyA: RigidBody, bodyB: RigidBody): number {
    const records = pairs.records;
    let edgeKey = bodyA.headPairKey;
    while (edgeKey !== INVALID_PAIR_KEY) {
        const rec = getPairEdgeRecord(edgeKey);
        const side = getPairEdgeSide(edgeKey);
        if (records[rec * RECORD_STRIDE + (1 - side)] === bodyB.index) return rec;
        edgeKey = records[edgePrevSlot(rec, side) + 1];
    }
    return -1;
}

/**
 * destroy every persistent pair involving this body by walking its own pair list — O(pair degree).
 * each record's contact chain is cascaded (removal events queued onto pendingContactRemoved, since
 * the listener is unavailable at body-removal time). does not depend on the body's dbvt leaf, so it
 * is safe to call before or independently of tree-leaf removal.
 */
export function purgeBodyPairs(pairs: Pairs, contacts: Contacts, pool: RigidBody[], body: RigidBody): void {
    let edgeKey = body.headPairKey;
    while (edgeKey !== INVALID_PAIR_KEY) {
        const rec = getPairEdgeRecord(edgeKey);
        const side = getPairEdgeSide(edgeKey);

        // save the next key before destroying (the unlink clears this record's edge slots)
        edgeKey = pairs.records[edgePrevSlot(rec, side) + 1];

        removePairRecordAt(pairs, contacts, pool, rec, undefined, true);
    }
}

const _discovery_fatLeaf = /* @__PURE__ */ box3.create();
const _discovery_expandedFatAABB = /* @__PURE__ */ box3.create();
const _sweep_expandedAABB = /* @__PURE__ */ box3.create();
const _sweep_expandedFatAABB = /* @__PURE__ */ box3.create();
const _sweep_fatA = /* @__PURE__ */ box3.create();
const _sweep_fatB = /* @__PURE__ */ box3.create();

/**
 * find potentially colliding body pairs, updates world.pairs.collidingPairs.
 *
 * moved-only persistent-pair broadphase (box2d-v3 / bullet architecture): a persistent pair set is
 * populated by fat-AABB queries from bodies that moved (escaped their fat leaf) or were added, then
 * swept every frame. the emitted pairs array is semantically identical to the old per-active-body
 * scan (same pair set each frame; intra-pair/record order may differ).
 */
export function findCollidingPairs(world: World, speculativeContactDistance: number, listener: Listener | undefined): void {
    const layers = world.settings.layers;
    const broadphase = world.broadphase;
    const pairs = world.pairs;
    const pool = world.bodies.pool;

    // reset this frame's output
    pairs.collidingPairCount = 0;

    // --- (a) discovery: moved bodies query the trees with their fat leaf AABB (expanded by the
    // speculative distance) and register new persistent pairs. runs BEFORE the sweep so pairs
    // discovered this frame emit through the same single point this same frame (output-identity).
    const moved = pairs.moved;
    for (let i = 0; i < moved.length; i++) {
        const body = pool[moved[i]];

        // guard against bodies removed (and not reused) after being marked this frame
        if (body._pooled || body.dbvtNode === -1 || body.broadphaseLayer === -1) continue;

        const objectLayer = body.objectLayer;
        const bodyBroadphaseLayer = body.broadphaseLayer;

        // fat leaf AABB expanded by the speculative distance — the discovery reach
        const fatLeaf = dbvt.readNodeAabb(_discovery_fatLeaf, broadphase.dbvts[bodyBroadphaseLayer], body.dbvtNode);
        box3.expandByMargin(_discovery_expandedFatAABB, fatLeaf, speculativeContactDistance);

        for (let otherBroadphaseLayer = 0; otherBroadphaseLayer < broadphase.dbvts.length; otherBroadphaseLayer++) {
            if (!objectLayerCollidesWithBroadphaseLayer(layers, objectLayer, otherBroadphaseLayer)) continue;
            if (!broadphaseLayerCollidesWithBroadphaseLayer(layers, bodyBroadphaseLayer, otherBroadphaseLayer)) continue;

            const tree = broadphase.dbvts[otherBroadphaseLayer];

            PairDiscoveryVisitor.setup(layers, pairs, body);
            dbvt.intersectAABBFatLeaves(world, tree, _discovery_expandedFatAABB, PairDiscoveryVisitor);
        }
    }

    // moved flags intentionally stay set through the sweep — the frame-invariant skip below reads
    // them. the discovery batch is consumed (flags cleared, entries dequeued) after the sweep.
    const discoveredCount = moved.length;

    // --- (b) sweep: walk the persistent pair set, destroying separated pairs and emitting the rest
    // using exactly the old emission criterion. record indices are stable (freelist storage), so the
    // record indices written into collidingPairs stay valid for the whole frame. note: emission order
    // follows record-slot order (insertion history + freelist reuse) and is not stable across
    // different world histories — downstream must not depend on it (contact constraints are sorted
    // by deterministic sort keys).
    const contacts = world.contacts;
    const records = pairs.records;
    const numSlots = records.length / RECORD_STRIDE;
    for (let rec = 0; rec < numSlots; rec++) {
        const base = rec * RECORD_STRIDE;
        const bodyIndexA = records[base];

        // skip free slots
        if (bodyIndexA === -1) continue;

        const bodyA = pool[bodyIndexA];
        const bodyB = pool[records[base + 1]];

        // frame-invariant skip: two inactive, unmoved bodies with an empty contact chain. their
        // leaves are frozen (so the fat-overlap answer is last frame's "keep"), emission is
        // impossible (dedup rejects both-inactive), and there is no chain to reconcile — the
        // sweep's outcome cannot differ from last frame. this makes a steady-state sleeping pair
        // cost a few field compares per frame.
        if (
            bodyA.activeIndex === INACTIVE_BODY_INDEX &&
            bodyB.activeIndex === INACTIVE_BODY_INDEX &&
            !bodyA.inMoveSet &&
            !bodyB.inMoveSet &&
            pairs.firstContact[rec] === INVALID_CONTACT_KEY
        ) {
            continue;
        }

        // destroy if either leaf is gone, or the two FAT leaf AABBs no longer overlap
        if (bodyA.dbvtNode === -1 || bodyB.dbvtNode === -1) {
            removePairRecordAt(pairs, contacts, pool, rec, listener, false);
            continue;
        }
        const fatA = dbvt.readNodeAabb(_sweep_fatA, broadphase.dbvts[bodyA.broadphaseLayer], bodyA.dbvtNode);
        const fatB = dbvt.readNodeAabb(_sweep_fatB, broadphase.dbvts[bodyB.broadphaseLayer], bodyB.dbvtNode);
        // keep-condition mirrors the discovery reach (fat + speculative distance vs fat) so a
        // just-discovered pair always survives the same-frame sweep, with no dependence on the
        // relative sizes of expansionMargin and the speculative distance
        box3.expandByMargin(_sweep_expandedFatAABB, fatA, speculativeContactDistance);
        if (!box3.intersectsBox3(_sweep_expandedFatAABB, fatB)) {
            removePairRecordAt(pairs, contacts, pool, rec, listener, false);
            continue;
        }

        // emit using the old criterion. moreActive = smaller activeIndex, preserving
        // shouldReportPair's first-arg-is-active assumption (both inactive => dedup rejects).
        const moreActive = bodyA.activeIndex <= bodyB.activeIndex ? bodyA : bodyB;
        const lessActive = moreActive === bodyA ? bodyB : bodyA;

        // tight-AABB re-test: moreActive's aabb expanded by the speculative distance vs the other's
        // tight aabb (equivalent to the old active-body query leaf test)
        box3.expandByMargin(_sweep_expandedAABB, moreActive.aabb, speculativeContactDistance);
        let emitted = false;
        if (box3.intersectsBox3(_sweep_expandedAABB, lessActive.aabb) && shouldReportPair(layers, moreActive, lessActive)) {
            emitted = emitCollidingPair(pairs, moreActive, lessActive, rec, listener);
        }

        // a kept-but-not-emitted pair with a live contact chain has its chain destroyed here, with
        // immediate listener events. this single rule replaces the entire old global unprocessed
        // sweep: sleep transitions (both-inactive dedup), teleports/fat-separation still-overlapping,
        // motion/layer/group/mask gates, tight-AABB misses, and onBodyPairValidate rejections all
        // land here, each destroying a chain exactly once. the empty-chain check keeps steady-state
        // sleeping pairs O(1) per visit.
        if (!emitted && pairs.firstContact[rec] !== INVALID_CONTACT_KEY) {
            destroyPairChain(contacts, pairs, rec, listener, false);
        }
    }

    // consume the move set: clear the discovery batch's flags and dequeue it. marks queued during
    // the sweep (user mutations from inside onBodyPairValidate — outside the supported callback
    // contract, but preserved where possible) stay queued for the next frame's discovery.
    for (let i = 0; i < discoveredCount; i++) {
        pool[moved[i]].inMoveSet = false;
    }
    if (moved.length > discoveredCount) {
        moved.copyWithin(0, discoveredCount);
    }
    moved.length -= discoveredCount;
}

/**
 * Emit a colliding pair into this frame's output, after the optional user body pair filter.
 * The single emission point: every pair downstream consumes was pushed here. Returns true if the
 * pair was emitted, false if the user body pair filter rejected it (so the sweep can destroy its
 * contact chain, matching the old validate-reject removal).
 */
function emitCollidingPair(
    pairs: Pairs,
    moreActive: RigidBody,
    lessActive: RigidBody,
    recordIndex: number,
    listener: Listener | undefined,
): boolean {
    // user body pair filter — called after all built-in checks pass. bodies are sorted for
    // consistent callback ordering: higher motion type first, then lower id first.
    if (listener?.onBodyPairValidate) {
        let sortedA = moreActive;
        let sortedB = lessActive;
        if (sortedA.motionType > sortedB.motionType || (sortedA.motionType === sortedB.motionType && sortedB.id < sortedA.id)) {
            sortedA = lessActive;
            sortedB = moreActive;
        }
        if (!listener.onBodyPairValidate(sortedA, sortedB)) {
            return false;
        }
    }

    const collidingPairs = pairs.collidingPairs;
    const pairIndex = pairs.collidingPairCount * 3;
    if (pairIndex >= collidingPairs.length) {
        collidingPairs.push(moreActive.index, lessActive.index, recordIndex);
    } else {
        collidingPairs[pairIndex] = moreActive.index;
        collidingPairs[pairIndex + 1] = lessActive.index;
        collidingPairs[pairIndex + 2] = recordIndex;
    }
    pairs.collidingPairCount++;
    return true;
}
