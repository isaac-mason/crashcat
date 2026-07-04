import { type Box3, box3, raycast3, type Vec3, vec3 } from 'mathcat';
import type { RigidBody } from '../body/rigid-body';
import { rayDistanceToBox3, rayDistanceToBox3Flat, rayHitsBox3 } from '../collision/cast-utils';
import type { Filter } from '../filter';
import * as filter from '../filter';
import type { World } from '../world';
import type { BodyVisitor } from './body-visitor';

const STRIDE_BOUNDS = 6;
const STRIDE_TOPO = 5;
const T_PARENT = 0;
const T_LEFT = 1;
const T_RIGHT = 2;
const T_BODY = 3;
const T_CHANGED = 4;

export type DBVT = {
    /** packed-double node bounds, stride 6 (see file header) */
    bounds: number[];

    /** packed-SMI node topology, stride 5 (see file header) */
    topo: number[];
    freeNodeIndices: number[];
    root: number;

    /** the fat-leaf expansion margin (in world units) applied to each leaf AABB */
    expansionMargin: number;

    /**
     * any structural change (add / remove / fat-leaf escape) since the last rebuild. gates
     * the dirty-gated rebuild in broadphase.optimize — a tree that no body has disturbed
     * (e.g. a settled static field) stays clean and is never rebuilt.
     */
    dirty: boolean;
};

// flat SMI stack of node indices for overlap traversals (intersectAABB / intersectPoint /
// walk) — they carry no distances. grow-once with a size counter (no pop()/length churn,
// stays packed).
const _flatStack: number[] = [];

// parallel flat stacks for cast traversals: node indices (SMI array) + distances (double
// array) sharing one size counter — jolt keeps the same structure (a distance stack
// parallel to the node stack). separate arrays so each keeps its optimal element kind.
const _castStackNode: number[] = [];
const _castStackDist: number[] = [];

// binary analogue of jolt's cMaxDepthMarkChanged (5 quad levels ≈ 1024 partitions). the
// top MAX_DEPTH_MARK_CHANGED levels of a freshly built subtree are born `changed` so the
// most drift-prone upper structure is always re-partitioned on the next rebuild; deeper,
// settled subtrees graft in whole. ~1024 grafted units at binary depth 10.
const MAX_DEPTH_MARK_CHANGED = 10;

// grow-once scratch for rebuild (matches the _flatStack pattern — no pop()/length churn).
const _collectStack: number[] = []; // collection traversal node stack
const _buildUnits: number[] = []; // collected unit node indices, partitioned in place
const _buildCenters: number[] = []; // unit aabb centers, xyz interleaved, swapped alongside _buildUnits

export function create(): DBVT {
    const dbvt: DBVT = {
        bounds: [],
        topo: [],
        freeNodeIndices: [],
        root: -1,
        expansionMargin: 0.05,
        dirty: false,
    };

    return dbvt;
}

function requestNode(dbvt: DBVT): number {
    if (dbvt.freeNodeIndices.length > 0) {
        const n = dbvt.freeNodeIndices.pop()!;
        const t = n * STRIDE_TOPO;
        dbvt.topo[t] = -1;
        dbvt.topo[t + 1] = -1;
        dbvt.topo[t + 2] = -1;
        dbvt.topo[t + 3] = -1;
        dbvt.topo[t + 4] = 0;
        bEmpty(dbvt.bounds, n);
        return n;
    }
    const n = dbvt.bounds.length / STRIDE_BOUNDS;
    dbvt.topo.push(-1, -1, -1, -1, 0);
    dbvt.bounds.push(Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity);
    return n;
}

function releaseNode(dbvt: DBVT, n: number): void {
    const t = n * STRIDE_TOPO;
    dbvt.topo[t] = -1;
    dbvt.topo[t + 1] = -1;
    dbvt.topo[t + 2] = -1;
    dbvt.topo[t + 3] = -1;
    dbvt.topo[t + 4] = 0;
    dbvt.freeNodeIndices.push(n);
}

// ---- flat-bounds helpers (node-index args; @optimize call sites flatten them inline) ----------

function bEmpty(B: number[], n: number): void {
    const b = n * STRIDE_BOUNDS;
    B[b] = Infinity;
    B[b + 1] = Infinity;
    B[b + 2] = Infinity;
    B[b + 3] = -Infinity;
    B[b + 4] = -Infinity;
    B[b + 5] = -Infinity;
}

// d = a ∪ c (node indices). d may alias a or c.
function bUnionNodes(B: number[], d: number, a: number, c: number): void {
    const db = d * STRIDE_BOUNDS;
    const ab = a * STRIDE_BOUNDS;
    const cb = c * STRIDE_BOUNDS;
    const minX = B[ab] < B[cb] ? B[ab] : B[cb];
    const minY = B[ab + 1] < B[cb + 1] ? B[ab + 1] : B[cb + 1];
    const minZ = B[ab + 2] < B[cb + 2] ? B[ab + 2] : B[cb + 2];
    const maxX = B[ab + 3] > B[cb + 3] ? B[ab + 3] : B[cb + 3];
    const maxY = B[ab + 4] > B[cb + 4] ? B[ab + 4] : B[cb + 4];
    const maxZ = B[ab + 5] > B[cb + 5] ? B[ab + 5] : B[cb + 5];
    B[db] = minX;
    B[db + 1] = minY;
    B[db + 2] = minZ;
    B[db + 3] = maxX;
    B[db + 4] = maxY;
    B[db + 5] = maxZ;
}

// does node `o` fully contain node `i`?
function bContainsNode(B: number[], o: number, i: number): boolean {
    const ob = o * STRIDE_BOUNDS;
    const ib = i * STRIDE_BOUNDS;
    return (
        B[ob] <= B[ib] &&
        B[ob + 1] <= B[ib + 1] &&
        B[ob + 2] <= B[ib + 2] &&
        B[ob + 3] >= B[ib + 3] &&
        B[ob + 4] >= B[ib + 4] &&
        B[ob + 5] >= B[ib + 5]
    );
}

// does node `n` fully contain the standalone Box3 `box`?
function bContainsBox(B: number[], n: number, box: Box3): boolean {
    const nb = n * STRIDE_BOUNDS;
    return (
        B[nb] <= box[0] &&
        B[nb + 1] <= box[1] &&
        B[nb + 2] <= box[2] &&
        B[nb + 3] >= box[3] &&
        B[nb + 4] >= box[4] &&
        B[nb + 5] >= box[5]
    );
}

// node `n` bounds = `box` expanded outward by margin `m` on all sides.
function bSetExpandBox(B: number[], n: number, box: Box3, m: number): void {
    const nb = n * STRIDE_BOUNDS;
    B[nb] = box[0] - m;
    B[nb + 1] = box[1] - m;
    B[nb + 2] = box[2] - m;
    B[nb + 3] = box[3] + m;
    B[nb + 4] = box[4] + m;
    B[nb + 5] = box[5] + m;
}

// manhattan distance between the centers of node a and node b (bullet's btDbvt proximity).
function proximity(B: number[], a: number, b: number): number {
    const ab = a * STRIDE_BOUNDS;
    const bb = b * STRIDE_BOUNDS;
    const dx = B[ab] + B[ab + 3] - (B[bb] + B[bb + 3]);
    const dy = B[ab + 1] + B[ab + 4] - (B[bb + 1] + B[bb + 4]);
    const dz = B[ab + 2] + B[ab + 5] - (B[bb + 2] + B[bb + 5]);
    return Math.abs(dx) + Math.abs(dy) + Math.abs(dz);
}

function select(B: number[], o: number, a: number, b: number): number {
    return proximity(B, o, a) < proximity(B, o, b) ? 0 : 1;
}

function indexof(dbvt: DBVT, n: number): number {
    const parent = dbvt.topo[n * STRIDE_TOPO + T_PARENT];
    return dbvt.topo[parent * STRIDE_TOPO + T_RIGHT] === n ? 1 : 0;
}

/** @optimize */
function insertLeaf(dbvt: DBVT, rootIndex: number, leafIndex: number): void {
    const T = dbvt.topo;
    const B = dbvt.bounds;

    if (dbvt.root === -1) {
        dbvt.root = leafIndex;
        T[leafIndex * STRIDE_TOPO + T_PARENT] = -1;
        return;
    }

    // descend to find best leaf position
    let root = rootIndex;
    while (T[root * STRIDE_TOPO + T_LEFT] !== -1) {
        const child = select(B, leafIndex, T[root * STRIDE_TOPO + T_LEFT], T[root * STRIDE_TOPO + T_RIGHT]);
        root = child === 0 ? T[root * STRIDE_TOPO + T_LEFT] : T[root * STRIDE_TOPO + T_RIGHT];
    }

    const prev = T[root * STRIDE_TOPO + T_PARENT];
    const newParent = requestNode(dbvt);

    T[newParent * STRIDE_TOPO + T_PARENT] = prev;
    bUnionNodes(B, newParent, leafIndex, root);

    if (prev !== -1) {
        if (indexof(dbvt, root) === 0) {
            T[prev * STRIDE_TOPO + T_LEFT] = newParent;
        } else {
            T[prev * STRIDE_TOPO + T_RIGHT] = newParent;
        }
        T[newParent * STRIDE_TOPO + T_LEFT] = root;
        T[root * STRIDE_TOPO + T_PARENT] = newParent;
        T[newParent * STRIDE_TOPO + T_RIGHT] = leafIndex;
        T[leafIndex * STRIDE_TOPO + T_PARENT] = newParent;

        // refit: walk up the tree, checking if parent contains child
        let childNode = newParent;
        let parentIndex = prev;
        while (parentIndex !== -1) {
            if (!bContainsNode(B, parentIndex, childNode)) {
                bUnionNodes(B, parentIndex, T[parentIndex * STRIDE_TOPO + T_LEFT], T[parentIndex * STRIDE_TOPO + T_RIGHT]);
            } else {
                break;
            }
            childNode = parentIndex;
            parentIndex = T[parentIndex * STRIDE_TOPO + T_PARENT];
        }
    } else {
        T[newParent * STRIDE_TOPO + T_LEFT] = root;
        T[root * STRIDE_TOPO + T_PARENT] = newParent;
        T[newParent * STRIDE_TOPO + T_RIGHT] = leafIndex;
        T[leafIndex * STRIDE_TOPO + T_PARENT] = newParent;
        dbvt.root = newParent;
    }
}

// walk up from a changed node marking ancestors changed. stops at the first already-changed
// ancestor (the invariant changed ⇒ parent.changed guarantees everything above is done).
// jolt's MarkNodeAndParentsChanged.
function markNodeAndParentsChanged(dbvt: DBVT, nodeIndex: number): void {
    const T = dbvt.topo;
    let idx = nodeIndex;
    while (idx !== -1) {
        if (T[idx * STRIDE_TOPO + T_CHANGED] !== 0) break;
        T[idx * STRIDE_TOPO + T_CHANGED] = 1;
        idx = T[idx * STRIDE_TOPO + T_PARENT];
    }
}

// walk up from a leaf's parent widening each ancestor's aabb to encapsulate the (grown) source-node
// bounds and marking it changed. bounds only grow between rebuilds so containment holds at every
// instant (queries never miss); the next rebuild recomputes exact unions. once an ancestor already
// contains the source, only the changed-marking remains. jolt's WidenAndMarkNodeAndParentsChanged.
function widenAndMarkNodeAndParentsChanged(dbvt: DBVT, parentIndex: number, srcNode: number): void {
    const T = dbvt.topo;
    const B = dbvt.bounds;
    let idx = parentIndex;
    while (idx !== -1) {
        T[idx * STRIDE_TOPO + T_CHANGED] = 1;
        if (bContainsNode(B, idx, srcNode)) {
            // containment is monotone up the tree — nothing above needs widening, only marking
            markNodeAndParentsChanged(dbvt, T[idx * STRIDE_TOPO + T_PARENT]);
            break;
        }
        bUnionNodes(B, idx, idx, srcNode);
        idx = T[idx * STRIDE_TOPO + T_PARENT];
    }
}

/* @optimize */
function removeLeaf(dbvt: DBVT, leafIndex: number): number {
    const T = dbvt.topo;
    const B = dbvt.bounds;

    if (leafIndex === dbvt.root) {
        dbvt.root = -1;
        return -1;
    }

    const parentIndex = T[leafIndex * STRIDE_TOPO + T_PARENT];
    const prevIndex = T[parentIndex * STRIDE_TOPO + T_PARENT];
    const siblingIndex =
        T[parentIndex * STRIDE_TOPO + T_LEFT] === leafIndex ? T[parentIndex * STRIDE_TOPO + T_RIGHT] : T[parentIndex * STRIDE_TOPO + T_LEFT];

    if (prevIndex !== -1) {
        if (indexof(dbvt, parentIndex) === 0) {
            T[prevIndex * STRIDE_TOPO + T_LEFT] = siblingIndex;
        } else {
            T[prevIndex * STRIDE_TOPO + T_RIGHT] = siblingIndex;
        }
        T[siblingIndex * STRIDE_TOPO + T_PARENT] = prevIndex;
        releaseNode(dbvt, parentIndex);

        // the collapse restructured prev's children — mark it and its ancestors changed so the
        // next rebuild re-partitions this region (invariant: changed ⇒ parent.changed).
        markNodeAndParentsChanged(dbvt, prevIndex);

        // refit up until a node's bounds stop changing
        let nodeIndex = prevIndex;
        while (nodeIndex !== -1) {
            const nb = nodeIndex * STRIDE_BOUNDS;
            const o0 = B[nb];
            const o1 = B[nb + 1];
            const o2 = B[nb + 2];
            const o3 = B[nb + 3];
            const o4 = B[nb + 4];
            const o5 = B[nb + 5];
            bUnionNodes(B, nodeIndex, T[nodeIndex * STRIDE_TOPO + T_LEFT], T[nodeIndex * STRIDE_TOPO + T_RIGHT]);
            if (B[nb] !== o0 || B[nb + 1] !== o1 || B[nb + 2] !== o2 || B[nb + 3] !== o3 || B[nb + 4] !== o4 || B[nb + 5] !== o5) {
                nodeIndex = T[nodeIndex * STRIDE_TOPO + T_PARENT];
            } else {
                break;
            }
        }

        return prevIndex;
    } else {
        dbvt.root = siblingIndex;
        T[siblingIndex * STRIDE_TOPO + T_PARENT] = -1;
        releaseNode(dbvt, parentIndex);
        return dbvt.root;
    }
}

export function add(dbvt: DBVT, body: RigidBody): number {
    // create leaf node with fat (margin-expanded) bounds
    const leafIndex = requestNode(dbvt);
    bSetExpandBox(dbvt.bounds, leafIndex, body.aabb, dbvt.expansionMargin);
    dbvt.topo[leafIndex * STRIDE_TOPO + T_BODY] = body.index;

    // greedy insert so the body is immediately queryable (a rebuild only happens at the next
    // step); the balanced re-partition is deferred to the dirty-gated rebuild.
    insertLeaf(dbvt, dbvt.root, leafIndex);

    // mark the new leaf's ancestor chain changed + flag the tree for rebuild
    const parent = dbvt.topo[leafIndex * STRIDE_TOPO + T_PARENT];
    if (parent !== -1) markNodeAndParentsChanged(dbvt, parent);
    dbvt.dirty = true;

    return leafIndex;
}

export function remove(dbvt: DBVT, body: RigidBody): void {
    const leafIndex = body.dbvtNode;
    if (leafIndex === -1) return;

    removeLeaf(dbvt, leafIndex);
    releaseNode(dbvt, leafIndex);
    body.dbvtNode = -1;
    dbvt.dirty = true;
}

/**
 * @optimize
 * returns true iff the body escaped its fat AABB (a "moved" event the persistent-pair broadphase
 * consumes), false when the containment early-out fired.
 *
 * jolt-style widen-in-place + defer: on escape we REPLACE the leaf's fat box (may shrink — this is
 * the one deviation from jolt, forced by crashcat's persistent pairs, which key keep/destroy on the
 * fat leaf boxes and would leak stale pairs under monotone-growing leaves), then widen ancestors
 * (grow-only, monotone until rebuild) and mark the path changed. no remove/reinsert — the balanced
 * restructure is deferred to the next dirty-gated rebuild. per-move cost is O(depth), typically O(1).
 */
export function update(dbvt: DBVT, body: RigidBody): boolean {
    const leafIndex = body.dbvtNode;
    if (leafIndex === -1) return false;

    const B = dbvt.bounds;

    // early exit: if body still fits in the fat AABB, nothing to do
    if (bContainsBox(B, leafIndex, body.aabb)) {
        return false;
    }

    // replace the leaf fat box in place — bit-identical values, at the identical escape moments,
    // as the old remove+reinsert path, so persistent-pair discovery/sweep see unchanged fat-leaf data
    bSetExpandBox(B, leafIndex, body.aabb, dbvt.expansionMargin);

    // widen ancestors to keep containment (grow-only until rebuild) and mark the path changed
    const parent = dbvt.topo[leafIndex * STRIDE_TOPO + T_PARENT];
    if (parent !== -1) widenAndMarkNodeAndParentsChanged(dbvt, parent, leafIndex);
    dbvt.dirty = true;

    return true;
}

// -----------------------------------------------------------------------------------------------
// dirty-gated balanced rebuild (jolt QuadTree::UpdatePrepare, single-threaded + binary + in-place).
//
// leaf-preserving partial rebuild: leaves and unchanged-internal subtrees keep their node indices
// (so body.dbvtNode stays valid — no SetBodyLocation equivalent needed); only `changed` internals
// are recycled and re-partitioned, plus the always-rebuilt top MAX_DEPTH_MARK_CHANGED levels.
// because changed ⇒ parent.changed, the freed set is one root-connected subtree of f internals whose
// frontier is f+1 units and the build allocates exactly f internals → the node pool is invariant
// across rebuilds (the old optimizeTopDown leaked every internal on every call).
// -----------------------------------------------------------------------------------------------

// in-place median split of units[begin,end) (and their interleaved centers) on the widest-extent
// axis of the centers. returns the split midpoint; guarantees ≥1 element per side (degenerate
// identical/collinear centers fall back to the count midpoint), so the build always terminates.
function sPartition(begin: number, end: number): number {
    let minX = Infinity;
    let minY = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    let maxZ = -Infinity;
    for (let k = begin; k < end; k++) {
        const cx = _buildCenters[k * 3];
        const cy = _buildCenters[k * 3 + 1];
        const cz = _buildCenters[k * 3 + 2];
        if (cx < minX) minX = cx;
        if (cy < minY) minY = cy;
        if (cz < minZ) minZ = cz;
        if (cx > maxX) maxX = cx;
        if (cy > maxY) maxY = cy;
        if (cz > maxZ) maxZ = cz;
    }
    const ex = maxX - minX;
    const ey = maxY - minY;
    const ez = maxZ - minZ;
    const axis = ex >= ey ? (ex >= ez ? 0 : 2) : ey >= ez ? 1 : 2;
    const split = 0.5 * ((axis === 0 ? minX : axis === 1 ? minY : minZ) + (axis === 0 ? maxX : axis === 1 ? maxY : maxZ));

    // hoare partition: units[] and centers[] swapped together
    let lo = begin;
    let hi = end;
    while (lo < hi) {
        while (lo < hi && _buildCenters[lo * 3 + axis] < split) lo++;
        while (lo < hi && _buildCenters[(hi - 1) * 3 + axis] >= split) hi--;
        if (lo < hi) {
            hi--;
            const u = _buildUnits[lo];
            _buildUnits[lo] = _buildUnits[hi];
            _buildUnits[hi] = u;
            for (let c = 0; c < 3; c++) {
                const t = _buildCenters[lo * 3 + c];
                _buildCenters[lo * 3 + c] = _buildCenters[hi * 3 + c];
                _buildCenters[hi * 3 + c] = t;
            }
            lo++;
        }
    }
    // degenerate: everything on one side → split down the middle by count so both sides are non-empty
    if (lo === begin || lo === end) return begin + ((end - begin) >> 1);
    return lo;
}

// recursively build a balanced binary tree over units[begin,end). returns the subtree root index.
// nodes at depth < maxDepthMarkChanged are born `changed` so the drift-prone upper structure is
// always re-partitioned next rebuild. iterative (explicit stack): a geometric median split can
// legitimately produce 1/(n-1) partitions repeatedly, so recursion depth can reach O(units).
function buildTree(dbvt: DBVT, begin: number, end: number, maxDepthMarkChanged: number): number {
    // explicit frame stack — phase 0 = partition + build left, 1 = build right, 2 = combine.
    // frameSplit holds the split midpoint (phase 0→1) then the resolved left-child index (phase 1→2).
    const frameBegin: number[] = [];
    const frameEnd: number[] = [];
    const frameDepth: number[] = [];
    const framePhase: number[] = [];
    const frameSplit: number[] = [];

    let resultIndex = -1;
    frameBegin.push(begin);
    frameEnd.push(end);
    frameDepth.push(0);
    framePhase.push(0);
    frameSplit.push(-1);

    while (frameBegin.length > 0) {
        const top = frameBegin.length - 1;
        const b = frameBegin[top];
        const e = frameEnd[top];
        const depth = frameDepth[top];
        const phase = framePhase[top];

        if (e - b === 1) {
            // leaf or grafted unchanged subtree — return its index unchanged, parent set by caller
            resultIndex = _buildUnits[b];
            frameBegin.pop();
            frameEnd.pop();
            frameDepth.pop();
            framePhase.pop();
            frameSplit.pop();
            continue;
        }

        if (phase === 0) {
            // partition, then descend into the left child; remember the split point
            const mid = sPartition(b, e);
            framePhase[top] = 1;
            frameSplit[top] = mid;
            frameBegin.push(b);
            frameEnd.push(mid);
            frameDepth.push(depth + 1);
            framePhase.push(0);
            frameSplit.push(-1);
        } else if (phase === 1) {
            // left child resolved into resultIndex — stash it, descend into the right child
            const mid = frameSplit[top];
            framePhase[top] = 2;
            frameSplit[top] = resultIndex; // frameSplit now holds the left-child index
            frameBegin.push(mid);
            frameEnd.push(e);
            frameDepth.push(depth + 1);
            framePhase.push(0);
            frameSplit.push(-1);
        } else {
            // right child resolved — combine into a new internal parent
            const ri = resultIndex;
            const li = frameSplit[top];
            const p = requestNode(dbvt);
            dbvt.topo[p * STRIDE_TOPO + T_LEFT] = li;
            dbvt.topo[p * STRIDE_TOPO + T_RIGHT] = ri;
            dbvt.topo[li * STRIDE_TOPO + T_PARENT] = p;
            dbvt.topo[ri * STRIDE_TOPO + T_PARENT] = p;
            bUnionNodes(dbvt.bounds, p, li, ri);
            dbvt.topo[p * STRIDE_TOPO + T_CHANGED] = depth < maxDepthMarkChanged ? 1 : 0;
            resultIndex = p;
            frameBegin.pop();
            frameEnd.pop();
            frameDepth.pop();
            framePhase.pop();
            frameSplit.pop();
        }
    }

    return resultIndex;
}

/**
 * Rebuild the tree into a balanced structure and clear the dirty flag. Leaf-preserving and
 * partial: unchanged internal subtrees graft in whole (indices untouched); only changed internals
 * are recycled. O(m log m) in the changed unit count m, not the body count.
 */
export function rebuild(dbvt: DBVT, maxDepthMarkChanged = MAX_DEPTH_MARK_CHANGED): void {
    dbvt.dirty = false;
    if (dbvt.root === -1) return;

    const T = dbvt.topo;
    const B = dbvt.bounds;

    // phase 1: collect units (leaves + unchanged-internal subtree roots) and free changed internals
    _buildUnits.length = 0;
    _collectStack.length = 0;
    _collectStack.push(dbvt.root);
    while (_collectStack.length > 0) {
        const idx = _collectStack.pop()!;
        // leaf-ness tested BEFORE changed: leaves are always kept as units (never freed), preserving
        // body.dbvtNode; an unchanged internal grafts whole; a changed internal is opened + recycled
        if (T[idx * STRIDE_TOPO + T_LEFT] === -1 || T[idx * STRIDE_TOPO + T_CHANGED] === 0) {
            _buildUnits.push(idx);
        } else {
            _collectStack.push(T[idx * STRIDE_TOPO + T_LEFT]);
            _collectStack.push(T[idx * STRIDE_TOPO + T_RIGHT]);
            releaseNode(dbvt, idx);
        }
    }

    const m = _buildUnits.length;
    if (m === 1) {
        // lone leaf, or the whole tree was unchanged — graft as root
        dbvt.root = _buildUnits[0];
        T[dbvt.root * STRIDE_TOPO + T_PARENT] = -1;
        return;
    }

    // phase 2: unit centers (interleaved xyz), consumed + reordered in place by the build
    for (let k = 0; k < m; k++) {
        const ub = _buildUnits[k] * STRIDE_BOUNDS;
        _buildCenters[k * 3] = (B[ub] + B[ub + 3]) * 0.5;
        _buildCenters[k * 3 + 1] = (B[ub + 1] + B[ub + 4]) * 0.5;
        _buildCenters[k * 3 + 2] = (B[ub + 2] + B[ub + 5]) * 0.5;
    }

    // phase 3: median-split build over the units
    dbvt.root = buildTree(dbvt, 0, m, maxDepthMarkChanged);
    T[dbvt.root * STRIDE_TOPO + T_PARENT] = -1;
}

/**
 * Overlap traversal against the FAT leaf AABBs — no tight body-AABB re-test and no filtering.
 * Used by persistent-pair discovery: a pair must exist whenever the two fat boxes could
 * bring the bodies into contact while both coast inside them, so the leaf test must be the
 * fat node AABB (already tested during descent), NOT the current tight body AABB.
 */
export function intersectAABBFatLeaves(world: World, dbvt: DBVT, aabb: Box3, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

    const T = dbvt.topo;
    const B = dbvt.bounds;

    const qMinX = aabb[0];
    const qMinY = aabb[1];
    const qMinZ = aabb[2];
    const qMaxX = aabb[3];
    const qMaxY = aabb[4];
    const qMaxZ = aabb[5];

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];
        const nb = nodeIndex * STRIDE_BOUNDS;

        // node aabb test (for a leaf this IS the fat leaf test)
        if (B[nb] > qMaxX || B[nb + 3] < qMinX || B[nb + 1] > qMaxY || B[nb + 4] < qMinY || B[nb + 2] > qMaxZ || B[nb + 5] < qMinZ) {
            continue;
        }

        if (T[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        visitor.visit(world.bodies.pool[T[nodeIndex * STRIDE_TOPO + T_BODY]]);

        if (visitor.shouldExit) {
            return;
        }
    }
}

export function intersectAABB(world: World, dbvt: DBVT, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

    const T = dbvt.topo;
    const B = dbvt.bounds;

    const qMinX = aabb[0];
    const qMinY = aabb[1];
    const qMinZ = aabb[2];
    const qMaxX = aabb[3];
    const qMaxY = aabb[4];
    const qMaxZ = aabb[5];

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];
        const nb = nodeIndex * STRIDE_BOUNDS;

        // node aabb test
        if (B[nb] > qMaxX || B[nb + 3] < qMinX || B[nb + 1] > qMaxY || B[nb + 4] < qMinY || B[nb + 2] > qMaxZ || B[nb + 5] < qMinZ) {
            continue;
        }

        // if internal node, push children
        if (T[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[T[nodeIndex * STRIDE_TOPO + T_BODY]];

        // collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroups,
                queryFilter.collisionMask,
                body.collisionGroups,
                body.collisionMask,
            )
        ) {
            continue;
        }

        // object layer filtering
        if (!filter.filterObjectLayer(queryFilter, body.objectLayer)) {
            continue;
        }

        // body filter callback
        if (queryFilter.bodyFilter && !queryFilter.bodyFilter(body)) {
            continue;
        }

        // body aabb test
        if (
            body.aabb[0] > qMaxX ||
            body.aabb[3] < qMinX ||
            body.aabb[1] > qMaxY ||
            body.aabb[4] < qMinY ||
            body.aabb[2] > qMaxZ ||
            body.aabb[5] < qMinZ
        ) {
            continue;
        }

        // visit
        visitor.visit(body);

        // early exit?
        if (visitor.shouldExit) {
            return;
        }
    }
}

export function intersectPoint(world: World, dbvt: DBVT, point: Vec3, queryFilter: Filter, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

    const T = dbvt.topo;
    const B = dbvt.bounds;

    const px = point[0];
    const py = point[1];
    const pz = point[2];

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];
        const nb = nodeIndex * STRIDE_BOUNDS;

        // skip if point is not inside node's AABB
        if (px < B[nb] || px > B[nb + 3] || py < B[nb + 1] || py > B[nb + 4] || pz < B[nb + 2] || pz > B[nb + 5]) {
            continue;
        }

        // if internal node, push children
        if (T[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[T[nodeIndex * STRIDE_TOPO + T_BODY]];
        if (!body || body._pooled) continue;

        // collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroups,
                queryFilter.collisionMask,
                body.collisionGroups,
                body.collisionMask,
            )
        ) {
            continue;
        }

        // object layer filtering
        if (!filter.filterObjectLayer(queryFilter, body.objectLayer)) {
            continue;
        }

        // body filter callback
        if (queryFilter.bodyFilter && !queryFilter.bodyFilter(body)) {
            continue;
        }

        // body contains point
        if (
            px < body.aabb[0] ||
            px > body.aabb[3] ||
            py < body.aabb[1] ||
            py > body.aabb[4] ||
            pz < body.aabb[2] ||
            pz > body.aabb[5]
        ) {
            continue;
        }

        visitor.visit(body);

        if (visitor.shouldExit) {
            return;
        }
    }
}

export function walk(dbvt: DBVT, visitor: BodyVisitor, world: World): void {
    if (dbvt.root === -1) return;

    const T = dbvt.topo;

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];

        if (T[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = T[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        const body = world.bodies.pool[T[nodeIndex * STRIDE_TOPO + T_BODY]];
        if (!body || body._pooled) continue;

        visitor.visit(body);

        if (visitor.shouldExit) {
            return;
        }
    }
}

const _ray = /* @__PURE__ */ raycast3.create();
const _nodeBounds = /* @__PURE__ */ box3.create(); // scratch for expanded child bounds in castAABB distance sort

/* @optimize */
export function castRay(
    world: World,
    dbvt: DBVT,
    origin: Vec3,
    direction: Vec3,
    length: number,
    queryFilter: Filter,
    visitor: BodyVisitor,
): void {
    if (dbvt.root === -1) return;

    const T = dbvt.topo;
    const B = dbvt.bounds;

    raycast3.set(_ray, origin, direction, length);

    const originX = _ray.origin[0];
    const originY = _ray.origin[1];
    const originZ = _ray.origin[2];
    const dirX = _ray.direction[0];
    const dirY = _ray.direction[1];
    const dirZ = _ray.direction[2];
    const rayLen = _ray.length;

    // closest-hit fraction so far (jolt's GetEarlyOutFraction); any node whose fat-AABB entry
    // fraction is >= this can't hold a closer hit and is pruned. distances are normalized to
    // [0, 1] of the ray length (rayDistanceToBox3Flat), matching the collector's fraction. visitors
    // that don't cast omit it → Infinity → distance pruning off (only misses rejected).
    let bestFraction = visitor.earlyOutFraction ?? Infinity;

    let stackSize = 0;
    _castStackNode[stackSize] = dbvt.root;
    _castStackDist[stackSize] = rayDistanceToBox3Flat(originX, originY, originZ, dirX, dirY, dirZ, rayLen, B, dbvt.root * STRIDE_BOUNDS);
    stackSize++;

    while (stackSize > 0) {
        stackSize--;
        const nodeIndex = _castStackNode[stackSize];
        const nodeDistance = _castStackDist[stackSize];

        // prune: skip misses (Infinity distance) and nodes whose entry is beyond the closest hit
        // found so far (best-t). the node's own aabb was already ray-tested when its parent pushed
        // it — a finite stored distance means it hits — so re-testing node.aabb here is redundant.
        if (nodeDistance >= bestFraction) {
            continue;
        }

        const left = T[nodeIndex * STRIDE_TOPO + T_LEFT];

        // if internal node, push children sorted by distance, culling any beyond best-t
        if (left !== -1) {
            const right = T[nodeIndex * STRIDE_TOPO + T_RIGHT];

            const leftDist = rayDistanceToBox3Flat(originX, originY, originZ, dirX, dirY, dirZ, rayLen, B, left * STRIDE_BOUNDS);
            const rightDist = rayDistanceToBox3Flat(originX, originY, originZ, dirX, dirY, dirZ, rayLen, B, right * STRIDE_BOUNDS);

            // push in reverse order (furthest first) so closest is popped first
            if (leftDist < rightDist) {
                if (rightDist < bestFraction) {
                    _castStackNode[stackSize] = right;
                    _castStackDist[stackSize] = rightDist;
                    stackSize++;
                }
                if (leftDist < bestFraction) {
                    _castStackNode[stackSize] = left;
                    _castStackDist[stackSize] = leftDist;
                    stackSize++;
                }
            } else {
                if (leftDist < bestFraction) {
                    _castStackNode[stackSize] = left;
                    _castStackDist[stackSize] = leftDist;
                    stackSize++;
                }
                if (rightDist < bestFraction) {
                    _castStackNode[stackSize] = right;
                    _castStackDist[stackSize] = rightDist;
                    stackSize++;
                }
            }
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[T[nodeIndex * STRIDE_TOPO + T_BODY]];

        // early out: collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroups,
                queryFilter.collisionMask,
                body.collisionGroups,
                body.collisionMask,
            )
        ) {
            continue;
        }

        // early out: object layer filtering
        if (!filter.filterObjectLayer(queryFilter, body.objectLayer)) {
            continue;
        }

        // body filter callback
        if (queryFilter.bodyFilter && !queryFilter.bodyFilter(body)) {
            continue;
        }

        // early out: ray-aabb test on body bounds
        if (
            !rayHitsBox3(
                originX,
                originY,
                originZ,
                dirX,
                dirY,
                dirZ,
                rayLen,
                body.aabb[0],
                body.aabb[1],
                body.aabb[2],
                body.aabb[3],
                body.aabb[4],
                body.aabb[5],
            )
        ) {
            continue;
        }

        // visit
        visitor.visit(body);

        if (visitor.shouldExit) {
            return;
        }

        // a hit may have shrunk the collector's early-out fraction — tighten the pruning bound
        bestFraction = visitor.earlyOutFraction ?? Infinity;
    }
}

export function castAABB(
    world: World,
    dbvt: DBVT,
    bounds: Box3,
    displacement: Vec3,
    queryFilter: Filter,
    visitor: BodyVisitor,
): void {
    if (dbvt.root === -1) return;

    const T = dbvt.topo;
    const B = dbvt.bounds;

    // AABB cast is done by:
    // 1. Shrink the shape aabb by its own extents down to a point (compute ray origin from AABB center)
    // 2. Expand each node aabb by the shape's half extents
    // 3. Cast the point by the displacement against the expanded node aabb (ray-slab test)

    // compute ray origin from AABB center and half extents — all as plain scalars
    const originX = (bounds[0] + bounds[3]) * 0.5;
    const originY = (bounds[1] + bounds[4]) * 0.5;
    const originZ = (bounds[2] + bounds[5]) * 0.5;
    const halfX = (bounds[3] - bounds[0]) * 0.5;
    const halfY = (bounds[4] - bounds[1]) * 0.5;
    const halfZ = (bounds[5] - bounds[2]) * 0.5;

    const castLen = vec3.length(displacement);
    const dirX = castLen > 0 ? displacement[0] / castLen : 0;
    const dirY = castLen > 0 ? displacement[1] / castLen : 0;
    const dirZ = castLen > 0 ? displacement[2] / castLen : 0;

    let stackSize = 0;
    _castStackNode[stackSize] = dbvt.root;
    _castStackDist[stackSize] = -Infinity; // root always visited
    stackSize++;

    while (stackSize > 0) {
        stackSize--;
        const nodeIndex = _castStackNode[stackSize];
        const nodeDistance = _castStackDist[stackSize];
        const nb = nodeIndex * STRIDE_BOUNDS;

        // early-out: skip nodes beyond cast length
        if (nodeDistance > castLen) {
            continue;
        }

        // ray-slab test against node aabb expanded by the shape's half extents (minkowski sum)
        if (
            !rayHitsBox3(
                originX,
                originY,
                originZ,
                dirX,
                dirY,
                dirZ,
                castLen,
                B[nb] - halfX,
                B[nb + 1] - halfY,
                B[nb + 2] - halfZ,
                B[nb + 3] + halfX,
                B[nb + 4] + halfY,
                B[nb + 5] + halfZ,
            )
        ) {
            continue;
        }

        const left = T[nodeIndex * STRIDE_TOPO + T_LEFT];

        // if internal node, push children sorted by distance
        if (left !== -1) {
            const right = T[nodeIndex * STRIDE_TOPO + T_RIGHT];
            const lb = left * STRIDE_BOUNDS;
            const rb = right * STRIDE_BOUNDS;

            _nodeBounds[0] = B[lb] - halfX;
            _nodeBounds[1] = B[lb + 1] - halfY;
            _nodeBounds[2] = B[lb + 2] - halfZ;
            _nodeBounds[3] = B[lb + 3] + halfX;
            _nodeBounds[4] = B[lb + 4] + halfY;
            _nodeBounds[5] = B[lb + 5] + halfZ;
            const leftDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, castLen, _nodeBounds);

            _nodeBounds[0] = B[rb] - halfX;
            _nodeBounds[1] = B[rb + 1] - halfY;
            _nodeBounds[2] = B[rb + 2] - halfZ;
            _nodeBounds[3] = B[rb + 3] + halfX;
            _nodeBounds[4] = B[rb + 4] + halfY;
            _nodeBounds[5] = B[rb + 5] + halfZ;
            const rightDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, castLen, _nodeBounds);

            // push in reverse order (furthest first) so closest is popped first
            if (leftDist < rightDist) {
                _castStackNode[stackSize] = right;
                _castStackDist[stackSize] = rightDist;
                stackSize++;
                _castStackNode[stackSize] = left;
                _castStackDist[stackSize] = leftDist;
                stackSize++;
            } else {
                _castStackNode[stackSize] = left;
                _castStackDist[stackSize] = leftDist;
                stackSize++;
                _castStackNode[stackSize] = right;
                _castStackDist[stackSize] = rightDist;
                stackSize++;
            }
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[T[nodeIndex * STRIDE_TOPO + T_BODY]];
        if (!body || body._pooled) continue;

        // collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroups,
                queryFilter.collisionMask,
                body.collisionGroups,
                body.collisionMask,
            )
        ) {
            continue;
        }

        // object layer filtering
        if (!filter.filterObjectLayer(queryFilter, body.objectLayer)) {
            continue;
        }

        // body filter callback
        if (queryFilter.bodyFilter && !queryFilter.bodyFilter(body)) {
            continue;
        }

        // ray-slab test against body aabb expanded by the shape's half extents (minkowski sum)
        if (
            !rayHitsBox3(
                originX,
                originY,
                originZ,
                dirX,
                dirY,
                dirZ,
                castLen,
                body.aabb[0] - halfX,
                body.aabb[1] - halfY,
                body.aabb[2] - halfZ,
                body.aabb[3] + halfX,
                body.aabb[4] + halfY,
                body.aabb[5] + halfZ,
            )
        ) {
            continue;
        }

        visitor.visit(body);

        if (visitor.shouldExit) {
            return;
        }
    }
}

/** get the bounds of the entire DBVT */
export function bounds(out: Box3, dbvt: DBVT): Box3 {
    if (dbvt.root === -1) {
        return box3.empty(out);
    }
    return readNodeAabb(out, dbvt, dbvt.root);
}

// ---- accessors (keep external callers — pairs.ts, broadphase.ts, tests — layout-agnostic) -----

/** number of allocated node rows (including free-listed rows), i.e. the node pool high-water mark */
export function nodeCount(dbvt: DBVT): number {
    return dbvt.bounds.length / STRIDE_BOUNDS;
}

/** copy node `n`'s fat AABB into `out` (out param first) */
export function readNodeAabb(out: Box3, dbvt: DBVT, n: number): Box3 {
    const b = n * STRIDE_BOUNDS;
    out[0] = dbvt.bounds[b];
    out[1] = dbvt.bounds[b + 1];
    out[2] = dbvt.bounds[b + 2];
    out[3] = dbvt.bounds[b + 3];
    out[4] = dbvt.bounds[b + 4];
    out[5] = dbvt.bounds[b + 5];
    return out;
}

export function nodeParent(dbvt: DBVT, n: number): number {
    return dbvt.topo[n * STRIDE_TOPO + T_PARENT];
}

export function nodeLeft(dbvt: DBVT, n: number): number {
    return dbvt.topo[n * STRIDE_TOPO + T_LEFT];
}

export function nodeRight(dbvt: DBVT, n: number): number {
    return dbvt.topo[n * STRIDE_TOPO + T_RIGHT];
}

export function nodeBodyIndex(dbvt: DBVT, n: number): number {
    return dbvt.topo[n * STRIDE_TOPO + T_BODY];
}

export function nodeChanged(dbvt: DBVT, n: number): boolean {
    return dbvt.topo[n * STRIDE_TOPO + T_CHANGED] !== 0;
}
