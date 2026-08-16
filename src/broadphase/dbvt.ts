// dynamic bounding volume tree (DBVT) over flat parallel node arrays.
//
// node storage is two flat arrays indexed by node index:
// - bounds: packed doubles, stride 6 — [minX, minY, minZ, maxX, maxY, maxZ]
// - topo: packed SMIs, stride 5 — [parent, left, right, body, changed]
//   parent/left/right are node indices (-1 = none; a node with left === -1 is a leaf), body is
//   the body pool index (leaves only), changed flags the node for re-partitioning at the next
//   dirty-gated rebuild (invariant: changed ⇒ parent.changed).
//
// leaves always reference live bodies: body destroy removes the leaf from the tree before the
// body is pooled, so traversals never see a pooled body.

import { type Vec3, vec3 } from 'math';
import { type Box3, box3 } from 'math/shapes';
import type { RigidBody } from '../body/rigid-body';
import { rayDistanceToBox3, rayHitsBox3 } from '../collision/cast-utils';
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

    /** rebuild marks the top N levels `changed` so they always re-partition */
    maxDepthMarkChanged: number;

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

// parallel flat stacks for cast traversals: node indices (SMI array) + distances (double array)
// sharing one size counter. separate arrays so each keeps its optimal element kind.
const _castStackNode: number[] = [];
const _castStackDist: number[] = [];

// top N levels of a freshly built subtree are born `changed` so the drift-prone upper structure is
// always re-partitioned next rebuild; deeper settled subtrees graft in whole.
const MAX_DEPTH_MARK_CHANGED = 10;

// module-level scratch for rebuild, grown once and reused across calls.
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
        maxDepthMarkChanged: MAX_DEPTH_MARK_CHANGED,
        dirty: false,
    };

    return dbvt;
}

function requestNode(dbvt: DBVT): number {
    if (dbvt.freeNodeIndices.length > 0) {
        // topo was already reset by releaseNode
        const n = dbvt.freeNodeIndices.pop()!;
        setNodeBoundsEmpty(dbvt.bounds, n);
        return n;
    }
    const n = dbvt.bounds.length / STRIDE_BOUNDS;
    dbvt.topo.push(-1, -1, -1, -1, 0);
    dbvt.bounds.push(Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity);
    return n;
}

function releaseNode(dbvt: DBVT, n: number): void {
    const t = n * STRIDE_TOPO;
    dbvt.topo[t + T_PARENT] = -1;
    dbvt.topo[t + T_LEFT] = -1;
    dbvt.topo[t + T_RIGHT] = -1;
    dbvt.topo[t + T_BODY] = -1;
    dbvt.topo[t + T_CHANGED] = 0;
    dbvt.freeNodeIndices.push(n);
}

// flat-bounds helpers operating on node indices (bounds base = node * STRIDE_BOUNDS)

function setNodeBoundsEmpty(bounds: number[], node: number): void {
    const base = node * STRIDE_BOUNDS;
    bounds[base] = Infinity;
    bounds[base + 1] = Infinity;
    bounds[base + 2] = Infinity;
    bounds[base + 3] = -Infinity;
    bounds[base + 4] = -Infinity;
    bounds[base + 5] = -Infinity;
}

// out = a ∪ b (out may alias a or b)
function setNodeBoundsUnion(bounds: number[], out: number, a: number, b: number): void {
    const outBase = out * STRIDE_BOUNDS;
    const aBase = a * STRIDE_BOUNDS;
    const bBase = b * STRIDE_BOUNDS;
    const minX = bounds[aBase] < bounds[bBase] ? bounds[aBase] : bounds[bBase];
    const minY = bounds[aBase + 1] < bounds[bBase + 1] ? bounds[aBase + 1] : bounds[bBase + 1];
    const minZ = bounds[aBase + 2] < bounds[bBase + 2] ? bounds[aBase + 2] : bounds[bBase + 2];
    const maxX = bounds[aBase + 3] > bounds[bBase + 3] ? bounds[aBase + 3] : bounds[bBase + 3];
    const maxY = bounds[aBase + 4] > bounds[bBase + 4] ? bounds[aBase + 4] : bounds[bBase + 4];
    const maxZ = bounds[aBase + 5] > bounds[bBase + 5] ? bounds[aBase + 5] : bounds[bBase + 5];
    bounds[outBase] = minX;
    bounds[outBase + 1] = minY;
    bounds[outBase + 2] = minZ;
    bounds[outBase + 3] = maxX;
    bounds[outBase + 4] = maxY;
    bounds[outBase + 5] = maxZ;
}

// does node `outer` fully contain node `inner`?
function nodeBoundsContainsNode(bounds: number[], outer: number, inner: number): boolean {
    const outerBase = outer * STRIDE_BOUNDS;
    const innerBase = inner * STRIDE_BOUNDS;
    return (
        bounds[outerBase] <= bounds[innerBase] &&
        bounds[outerBase + 1] <= bounds[innerBase + 1] &&
        bounds[outerBase + 2] <= bounds[innerBase + 2] &&
        bounds[outerBase + 3] >= bounds[innerBase + 3] &&
        bounds[outerBase + 4] >= bounds[innerBase + 4] &&
        bounds[outerBase + 5] >= bounds[innerBase + 5]
    );
}

// does node `node` fully contain the standalone Box3 `box`?
function nodeBoundsContainsBox(bounds: number[], node: number, box: Box3): boolean {
    const base = node * STRIDE_BOUNDS;
    return (
        bounds[base] <= box[0] &&
        bounds[base + 1] <= box[1] &&
        bounds[base + 2] <= box[2] &&
        bounds[base + 3] >= box[3] &&
        bounds[base + 4] >= box[4] &&
        bounds[base + 5] >= box[5]
    );
}

// node bounds = box grown outward by margin on all sides
function setNodeBoundsFromBoxExpanded(bounds: number[], node: number, box: Box3, margin: number): void {
    const base = node * STRIDE_BOUNDS;
    bounds[base] = box[0] - margin;
    bounds[base + 1] = box[1] - margin;
    bounds[base + 2] = box[2] - margin;
    bounds[base + 3] = box[3] + margin;
    bounds[base + 4] = box[4] + margin;
    bounds[base + 5] = box[5] + margin;
}

// manhattan distance between the centers of nodes a and b
function proximity(bounds: number[], a: number, b: number): number {
    const aBase = a * STRIDE_BOUNDS;
    const bBase = b * STRIDE_BOUNDS;
    const dx = bounds[aBase] + bounds[aBase + 3] - (bounds[bBase] + bounds[bBase + 3]);
    const dy = bounds[aBase + 1] + bounds[aBase + 4] - (bounds[bBase + 1] + bounds[bBase + 4]);
    const dz = bounds[aBase + 2] + bounds[aBase + 5] - (bounds[bBase + 2] + bounds[bBase + 5]);
    return Math.abs(dx) + Math.abs(dy) + Math.abs(dz);
}

// which of nodes a, b has the closer center to `node` — returns the chosen node index
function select(bounds: number[], node: number, a: number, b: number): number {
    return proximity(bounds, node, a) < proximity(bounds, node, b) ? a : b;
}

function isLeftChild(dbvt: DBVT, n: number): boolean {
    const parent = dbvt.topo[n * STRIDE_TOPO + T_PARENT];
    return dbvt.topo[parent * STRIDE_TOPO + T_RIGHT] !== n;
}

/** @optimize */
function insertLeaf(dbvt: DBVT, leafIndex: number): void {
    const topo = dbvt.topo;
    const bounds = dbvt.bounds;

    if (dbvt.root === -1) {
        dbvt.root = leafIndex;
        topo[leafIndex * STRIDE_TOPO + T_PARENT] = -1;
        return;
    }

    // descend to find best leaf position
    let root = dbvt.root;
    let left = topo[root * STRIDE_TOPO + T_LEFT];
    while (left !== -1) {
        root = select(bounds, leafIndex, left, topo[root * STRIDE_TOPO + T_RIGHT]);
        left = topo[root * STRIDE_TOPO + T_LEFT];
    }

    const prev = topo[root * STRIDE_TOPO + T_PARENT];
    const newParent = requestNode(dbvt);

    topo[newParent * STRIDE_TOPO + T_PARENT] = prev;
    setNodeBoundsUnion(bounds, newParent, leafIndex, root);

    if (prev !== -1) {
        if (isLeftChild(dbvt, root)) {
            topo[prev * STRIDE_TOPO + T_LEFT] = newParent;
        } else {
            topo[prev * STRIDE_TOPO + T_RIGHT] = newParent;
        }
        topo[newParent * STRIDE_TOPO + T_LEFT] = root;
        topo[root * STRIDE_TOPO + T_PARENT] = newParent;
        topo[newParent * STRIDE_TOPO + T_RIGHT] = leafIndex;
        topo[leafIndex * STRIDE_TOPO + T_PARENT] = newParent;

        // refit: walk up the tree, checking if parent contains child
        let childNode = newParent;
        let parentIndex = prev;
        while (parentIndex !== -1) {
            if (!nodeBoundsContainsNode(bounds, parentIndex, childNode)) {
                setNodeBoundsUnion(
                    bounds,
                    parentIndex,
                    topo[parentIndex * STRIDE_TOPO + T_LEFT],
                    topo[parentIndex * STRIDE_TOPO + T_RIGHT],
                );
            } else {
                break;
            }
            childNode = parentIndex;
            parentIndex = topo[parentIndex * STRIDE_TOPO + T_PARENT];
        }
    } else {
        topo[newParent * STRIDE_TOPO + T_LEFT] = root;
        topo[root * STRIDE_TOPO + T_PARENT] = newParent;
        topo[newParent * STRIDE_TOPO + T_RIGHT] = leafIndex;
        topo[leafIndex * STRIDE_TOPO + T_PARENT] = newParent;
        dbvt.root = newParent;
    }
}

// walk up from a changed node marking ancestors changed. stops at the first already-changed
// ancestor (the invariant changed ⇒ parent.changed means everything above is already done).
function markNodeAndParentsChanged(dbvt: DBVT, nodeIndex: number): void {
    const topo = dbvt.topo;
    let idx = nodeIndex;
    while (idx !== -1) {
        if (topo[idx * STRIDE_TOPO + T_CHANGED] !== 0) break;
        topo[idx * STRIDE_TOPO + T_CHANGED] = 1;
        idx = topo[idx * STRIDE_TOPO + T_PARENT];
    }
}

// walk up from a leaf's parent, growing each ancestor's aabb to cover the (grown) source-node bounds
// and marking it changed. bounds only grow between rebuilds so containment always holds; the next
// rebuild recomputes exact unions. once an ancestor already covers the source, only marking remains.
function widenAndMarkNodeAndParentsChanged(dbvt: DBVT, parentIndex: number, srcNode: number): void {
    const topo = dbvt.topo;
    const bounds = dbvt.bounds;
    let idx = parentIndex;
    while (idx !== -1) {
        topo[idx * STRIDE_TOPO + T_CHANGED] = 1;
        if (nodeBoundsContainsNode(bounds, idx, srcNode)) {
            // containment is monotone up the tree — nothing above needs widening, only marking
            markNodeAndParentsChanged(dbvt, topo[idx * STRIDE_TOPO + T_PARENT]);
            break;
        }
        setNodeBoundsUnion(bounds, idx, idx, srcNode);
        idx = topo[idx * STRIDE_TOPO + T_PARENT];
    }
}

/** @optimize */
function removeLeaf(dbvt: DBVT, leafIndex: number): number {
    const topo = dbvt.topo;
    const bounds = dbvt.bounds;

    if (leafIndex === dbvt.root) {
        dbvt.root = -1;
        return -1;
    }

    const parentIndex = topo[leafIndex * STRIDE_TOPO + T_PARENT];
    const prevIndex = topo[parentIndex * STRIDE_TOPO + T_PARENT];
    const siblingIndex =
        topo[parentIndex * STRIDE_TOPO + T_LEFT] === leafIndex
            ? topo[parentIndex * STRIDE_TOPO + T_RIGHT]
            : topo[parentIndex * STRIDE_TOPO + T_LEFT];

    if (prevIndex !== -1) {
        if (isLeftChild(dbvt, parentIndex)) {
            topo[prevIndex * STRIDE_TOPO + T_LEFT] = siblingIndex;
        } else {
            topo[prevIndex * STRIDE_TOPO + T_RIGHT] = siblingIndex;
        }
        topo[siblingIndex * STRIDE_TOPO + T_PARENT] = prevIndex;
        releaseNode(dbvt, parentIndex);

        // the collapse restructured prev's children — mark it and its ancestors changed so the
        // next rebuild re-partitions this region (invariant: changed ⇒ parent.changed).
        markNodeAndParentsChanged(dbvt, prevIndex);

        // refit up until a node's bounds stop changing
        let nodeIndex = prevIndex;
        while (nodeIndex !== -1) {
            const nb = nodeIndex * STRIDE_BOUNDS;
            const o0 = bounds[nb];
            const o1 = bounds[nb + 1];
            const o2 = bounds[nb + 2];
            const o3 = bounds[nb + 3];
            const o4 = bounds[nb + 4];
            const o5 = bounds[nb + 5];
            setNodeBoundsUnion(
                bounds,
                nodeIndex,
                topo[nodeIndex * STRIDE_TOPO + T_LEFT],
                topo[nodeIndex * STRIDE_TOPO + T_RIGHT],
            );
            if (
                bounds[nb] !== o0 ||
                bounds[nb + 1] !== o1 ||
                bounds[nb + 2] !== o2 ||
                bounds[nb + 3] !== o3 ||
                bounds[nb + 4] !== o4 ||
                bounds[nb + 5] !== o5
            ) {
                nodeIndex = topo[nodeIndex * STRIDE_TOPO + T_PARENT];
            } else {
                break;
            }
        }

        return prevIndex;
    } else {
        dbvt.root = siblingIndex;
        topo[siblingIndex * STRIDE_TOPO + T_PARENT] = -1;
        releaseNode(dbvt, parentIndex);
        return dbvt.root;
    }
}

export function add(dbvt: DBVT, body: RigidBody): number {
    // create leaf node with fat (margin-expanded) bounds
    const leafIndex = requestNode(dbvt);
    setNodeBoundsFromBoxExpanded(dbvt.bounds, leafIndex, body.aabb, dbvt.expansionMargin);
    dbvt.topo[leafIndex * STRIDE_TOPO + T_BODY] = body.index;

    // greedy insert so the body is immediately queryable (a rebuild only happens at the next
    // step); the balanced re-partition is deferred to the dirty-gated rebuild.
    insertLeaf(dbvt, leafIndex);

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
 * widen-in-place + defer: on escape we REPLACE the leaf's fat box (may shrink — the persistent-pair
 * sweep keys keep/destroy on the fat leaf boxes, so they must track the body, not grow forever), then
 * grow ancestors (grow-only until rebuild) and mark the path changed. no remove/reinsert — the
 * balanced restructure is deferred to the next rebuild. per-move cost is O(depth), typically O(1).
 */
export function update(dbvt: DBVT, body: RigidBody): boolean {
    const leafIndex = body.dbvtNode;
    if (leafIndex === -1) return false;

    const bounds = dbvt.bounds;

    // early exit: if body still fits in the fat AABB, nothing to do
    if (nodeBoundsContainsBox(bounds, leafIndex, body.aabb)) {
        return false;
    }

    // replace the leaf fat box in place, so the persistent-pair sweep sees the current body bounds
    setNodeBoundsFromBoxExpanded(bounds, leafIndex, body.aabb, dbvt.expansionMargin);

    // widen ancestors to keep containment (grow-only until rebuild) and mark the path changed
    const parent = dbvt.topo[leafIndex * STRIDE_TOPO + T_PARENT];
    if (parent !== -1) widenAndMarkNodeAndParentsChanged(dbvt, parent, leafIndex);
    dbvt.dirty = true;

    return true;
}

// -----------------------------------------------------------------------------------------------
// dirty-gated balanced rebuild — leaf-preserving and partial: leaves and unchanged-internal subtrees
// keep their node indices (so body.dbvtNode stays valid); only `changed` internals are recycled and
// re-partitioned, plus the always-rebuilt top MAX_DEPTH_MARK_CHANGED levels. because changed ⇒
// parent.changed, the freed internals form one root-connected subtree whose frontier has one more
// unit than it has nodes, and the build allocates exactly that many internals → the node pool size
// is invariant across rebuilds.
// -----------------------------------------------------------------------------------------------

// in-place median split of units[begin,end) (and their interleaved centers) on the widest-extent
// axis of the centers. returns the split midpoint; guarantees ≥1 element per side (degenerate
// identical/collinear centers fall back to the count midpoint), so the build always terminates.
function partitionUnits(begin: number, end: number): number {
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
    const extentX = maxX - minX;
    const extentY = maxY - minY;
    const extentZ = maxZ - minZ;
    const axis = extentX >= extentY ? (extentX >= extentZ ? 0 : 2) : extentY >= extentZ ? 1 : 2;
    const split = 0.5 * ((axis === 0 ? minX : axis === 1 ? minY : minZ) + (axis === 0 ? maxX : axis === 1 ? maxY : maxZ));

    // hoare partition: units[] and centers[] swapped together
    let lo = begin;
    let hi = end;
    while (lo < hi) {
        while (lo < hi && _buildCenters[lo * 3 + axis] < split) lo++;
        while (lo < hi && _buildCenters[(hi - 1) * 3 + axis] >= split) hi--;
        if (lo < hi) {
            hi--;
            const tmpUnit = _buildUnits[lo];
            _buildUnits[lo] = _buildUnits[hi];
            _buildUnits[hi] = tmpUnit;
            for (let k = 0; k < 3; k++) {
                const tmpCenter = _buildCenters[lo * 3 + k];
                _buildCenters[lo * 3 + k] = _buildCenters[hi * 3 + k];
                _buildCenters[hi * 3 + k] = tmpCenter;
            }
            lo++;
        }
    }
    // degenerate: everything on one side → split down the middle by count so both sides are non-empty
    if (lo === begin || lo === end) return begin + ((end - begin) >> 1);
    return lo;
}

// build a balanced binary tree over units[begin,end); returns the subtree root index. nodes at
// depth < maxDepthMarkChanged are born `changed` so the upper structure re-partitions next rebuild.
// iterative (a median split can degenerate to 1/(n-1), so recursion depth could reach O(units)).
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
        const nodeBegin = frameBegin[top];
        const nodeEnd = frameEnd[top];
        const depth = frameDepth[top];
        const phase = framePhase[top];

        if (nodeEnd - nodeBegin === 1) {
            // leaf or grafted unchanged subtree — return its index unchanged, parent set by caller
            resultIndex = _buildUnits[nodeBegin];
            frameBegin.pop();
            frameEnd.pop();
            frameDepth.pop();
            framePhase.pop();
            frameSplit.pop();
            continue;
        }

        if (phase === 0) {
            // partition, then descend into the left child; remember the split point
            const mid = partitionUnits(nodeBegin, nodeEnd);
            framePhase[top] = 1;
            frameSplit[top] = mid;
            frameBegin.push(nodeBegin);
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
            frameEnd.push(nodeEnd);
            frameDepth.push(depth + 1);
            framePhase.push(0);
            frameSplit.push(-1);
        } else {
            // right child resolved — combine into a new internal parent
            const rightIndex = resultIndex;
            const leftIndex = frameSplit[top];
            const parent = requestNode(dbvt);
            dbvt.topo[parent * STRIDE_TOPO + T_LEFT] = leftIndex;
            dbvt.topo[parent * STRIDE_TOPO + T_RIGHT] = rightIndex;
            dbvt.topo[leftIndex * STRIDE_TOPO + T_PARENT] = parent;
            dbvt.topo[rightIndex * STRIDE_TOPO + T_PARENT] = parent;
            setNodeBoundsUnion(dbvt.bounds, parent, leftIndex, rightIndex);
            dbvt.topo[parent * STRIDE_TOPO + T_CHANGED] = depth < maxDepthMarkChanged ? 1 : 0;
            resultIndex = parent;
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
export function rebuild(dbvt: DBVT): void {
    dbvt.dirty = false;
    if (dbvt.root === -1) return;

    const topo = dbvt.topo;
    const bounds = dbvt.bounds;
    const maxDepthMarkChanged = dbvt.maxDepthMarkChanged;

    // phase 1: collect units (leaves + unchanged-internal subtree roots) and free changed internals
    _buildUnits.length = 0;
    _collectStack.length = 0;
    _collectStack.push(dbvt.root);
    while (_collectStack.length > 0) {
        const idx = _collectStack.pop()!;
        // leaf-ness tested BEFORE changed: leaves are always kept as units (never freed), preserving
        // body.dbvtNode; an unchanged internal grafts whole; a changed internal is opened + recycled
        if (topo[idx * STRIDE_TOPO + T_LEFT] === -1 || topo[idx * STRIDE_TOPO + T_CHANGED] === 0) {
            _buildUnits.push(idx);
        } else {
            _collectStack.push(topo[idx * STRIDE_TOPO + T_LEFT]);
            _collectStack.push(topo[idx * STRIDE_TOPO + T_RIGHT]);
            releaseNode(dbvt, idx);
        }
    }

    const m = _buildUnits.length;
    if (m === 1) {
        // lone leaf, or the whole tree was unchanged — graft as root
        dbvt.root = _buildUnits[0];
        topo[dbvt.root * STRIDE_TOPO + T_PARENT] = -1;
        return;
    }

    // phase 2: unit centers (interleaved xyz), consumed + reordered in place by the build
    for (let k = 0; k < m; k++) {
        const ub = _buildUnits[k] * STRIDE_BOUNDS;
        _buildCenters[k * 3] = (bounds[ub] + bounds[ub + 3]) * 0.5;
        _buildCenters[k * 3 + 1] = (bounds[ub + 1] + bounds[ub + 4]) * 0.5;
        _buildCenters[k * 3 + 2] = (bounds[ub + 2] + bounds[ub + 5]) * 0.5;
    }

    // phase 3: median-split build over the units
    dbvt.root = buildTree(dbvt, 0, m, maxDepthMarkChanged);
    topo[dbvt.root * STRIDE_TOPO + T_PARENT] = -1;
}

/**
 * Overlap traversal against the FAT leaf AABBs — no tight body-AABB re-test and no filtering.
 * Used by persistent-pair discovery: a pair must exist whenever the two fat boxes could
 * bring the bodies into contact while both coast inside them, so the leaf test must be the
 * fat node AABB (already tested during descent), NOT the current tight body AABB.
 *
 * @optimize
 */
export function intersectAABBFatLeaves(world: World, dbvt: DBVT, aabb: Box3, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

    const topo = dbvt.topo;
    const bounds = dbvt.bounds;

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
        if (
            bounds[nb] > qMaxX ||
            bounds[nb + 3] < qMinX ||
            bounds[nb + 1] > qMaxY ||
            bounds[nb + 4] < qMinY ||
            bounds[nb + 2] > qMaxZ ||
            bounds[nb + 5] < qMinZ
        ) {
            continue;
        }

        if (topo[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        visitor.visit(world.bodies.pool[topo[nodeIndex * STRIDE_TOPO + T_BODY]]);

        if (visitor.shouldExit) {
            return;
        }
    }
}

/** @optimize */
export function intersectAABB(world: World, dbvt: DBVT, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

    const topo = dbvt.topo;
    const bounds = dbvt.bounds;

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
        if (
            bounds[nb] > qMaxX ||
            bounds[nb + 3] < qMinX ||
            bounds[nb + 1] > qMaxY ||
            bounds[nb + 4] < qMinY ||
            bounds[nb + 2] > qMaxZ ||
            bounds[nb + 5] < qMinZ
        ) {
            continue;
        }

        // if internal node, push children
        if (topo[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[topo[nodeIndex * STRIDE_TOPO + T_BODY]];

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

/** @optimize */
export function intersectPoint(world: World, dbvt: DBVT, point: Vec3, queryFilter: Filter, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

    const topo = dbvt.topo;
    const bounds = dbvt.bounds;

    const px = point[0];
    const py = point[1];
    const pz = point[2];

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];
        const nb = nodeIndex * STRIDE_BOUNDS;

        // skip if point is not inside node's AABB
        if (
            px < bounds[nb] ||
            px > bounds[nb + 3] ||
            py < bounds[nb + 1] ||
            py > bounds[nb + 4] ||
            pz < bounds[nb + 2] ||
            pz > bounds[nb + 5]
        ) {
            continue;
        }

        // if internal node, push children
        if (topo[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[topo[nodeIndex * STRIDE_TOPO + T_BODY]];

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

/** visit every body in the tree — no filtering, no aabb tests */
export function walk(world: World, dbvt: DBVT, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

    const topo = dbvt.topo;

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];

        if (topo[nodeIndex * STRIDE_TOPO + T_LEFT] !== -1) {
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_LEFT];
            _flatStack[stackSize++] = topo[nodeIndex * STRIDE_TOPO + T_RIGHT];
            continue;
        }

        const body = world.bodies.pool[topo[nodeIndex * STRIDE_TOPO + T_BODY]];

        visitor.visit(body);

        if (visitor.shouldExit) {
            return;
        }
    }
}

/** @optimize */
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

    const topo = dbvt.topo;
    const bounds = dbvt.bounds;

    const originX = origin[0];
    const originY = origin[1];
    const originZ = origin[2];
    const dirX = direction[0];
    const dirY = direction[1];
    const dirZ = direction[2];

    // closest-hit fraction so far; any node whose fat-AABB entry fraction is >= this can't hold a
    // closer hit and is pruned. both are normalized to [0, 1] of the ray length. visitors that don't
    // cast omit it → Infinity → distance pruning off (only misses rejected).
    let bestFraction = visitor.earlyOutFraction ?? Infinity;

    let stackSize = 0;
    _castStackNode[stackSize] = dbvt.root;
    const rootB = dbvt.root * STRIDE_BOUNDS;
    _castStackDist[stackSize] = rayDistanceToBox3(
        originX,
        originY,
        originZ,
        dirX,
        dirY,
        dirZ,
        length,
        bounds[rootB],
        bounds[rootB + 1],
        bounds[rootB + 2],
        bounds[rootB + 3],
        bounds[rootB + 4],
        bounds[rootB + 5],
    );
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

        const left = topo[nodeIndex * STRIDE_TOPO + T_LEFT];

        // if internal node, push children sorted by distance, culling any beyond best-t
        if (left !== -1) {
            const right = topo[nodeIndex * STRIDE_TOPO + T_RIGHT];

            const lb = left * STRIDE_BOUNDS;
            const leftDist = rayDistanceToBox3(
                originX,
                originY,
                originZ,
                dirX,
                dirY,
                dirZ,
                length,
                bounds[lb],
                bounds[lb + 1],
                bounds[lb + 2],
                bounds[lb + 3],
                bounds[lb + 4],
                bounds[lb + 5],
            );
            const rb = right * STRIDE_BOUNDS;
            const rightDist = rayDistanceToBox3(
                originX,
                originY,
                originZ,
                dirX,
                dirY,
                dirZ,
                length,
                bounds[rb],
                bounds[rb + 1],
                bounds[rb + 2],
                bounds[rb + 3],
                bounds[rb + 4],
                bounds[rb + 5],
            );

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
        const body = world.bodies.pool[topo[nodeIndex * STRIDE_TOPO + T_BODY]];

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
                length,
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

/** @optimize */
export function castAABB(
    world: World,
    dbvt: DBVT,
    castBounds: Box3,
    displacement: Vec3,
    queryFilter: Filter,
    visitor: BodyVisitor,
): void {
    if (dbvt.root === -1) return;

    const topo = dbvt.topo;
    const bounds = dbvt.bounds;

    // AABB cast is done by:
    // 1. Shrink the shape aabb by its own extents down to a point (compute ray origin from AABB center)
    // 2. Expand each node aabb by the shape's half extents
    // 3. Cast the point by the displacement against the expanded node aabb (ray-slab test)

    // compute ray origin from AABB center and half extents — all as plain scalars
    const originX = (castBounds[0] + castBounds[3]) * 0.5;
    const originY = (castBounds[1] + castBounds[4]) * 0.5;
    const originZ = (castBounds[2] + castBounds[5]) * 0.5;
    const halfX = (castBounds[3] - castBounds[0]) * 0.5;
    const halfY = (castBounds[4] - castBounds[1]) * 0.5;
    const halfZ = (castBounds[5] - castBounds[2]) * 0.5;

    const castLen = vec3.length(displacement);
    const dirX = castLen > 0 ? displacement[0] / castLen : 0;
    const dirY = castLen > 0 ? displacement[1] / castLen : 0;
    const dirZ = castLen > 0 ? displacement[2] / castLen : 0;

    // closest-hit fraction so far (positive early-out for shape casts). a node whose expanded-AABB
    // entry fraction is >= this can't hold a closer impact and is pruned. distances are normalized
    // to [0, 1] of the cast length. visitors that don't cast omit it → Infinity → only misses rejected.
    let bestFraction = visitor.earlyOutFraction ?? Infinity;

    let stackSize = 0;
    _castStackNode[stackSize] = dbvt.root;
    const rootBase = dbvt.root * STRIDE_BOUNDS;
    _castStackDist[stackSize] = rayDistanceToBox3(
        originX,
        originY,
        originZ,
        dirX,
        dirY,
        dirZ,
        castLen,
        bounds[rootBase] - halfX,
        bounds[rootBase + 1] - halfY,
        bounds[rootBase + 2] - halfZ,
        bounds[rootBase + 3] + halfX,
        bounds[rootBase + 4] + halfY,
        bounds[rootBase + 5] + halfZ,
    );
    stackSize++;

    while (stackSize > 0) {
        stackSize--;
        const nodeIndex = _castStackNode[stackSize];
        const nodeDistance = _castStackDist[stackSize];

        // prune misses (Infinity) and nodes past the closest impact so far (best-t). the expanded
        // node was already slab-tested when its parent pushed it, so no re-test here.
        if (nodeDistance >= bestFraction) {
            continue;
        }

        const left = topo[nodeIndex * STRIDE_TOPO + T_LEFT];

        // if internal node, push children sorted by distance
        if (left !== -1) {
            const right = topo[nodeIndex * STRIDE_TOPO + T_RIGHT];
            const lb = left * STRIDE_BOUNDS;
            const rb = right * STRIDE_BOUNDS;

            // node bounds expanded by the shape's half extents (minkowski sum), passed inline
            const leftDist = rayDistanceToBox3(
                originX,
                originY,
                originZ,
                dirX,
                dirY,
                dirZ,
                castLen,
                bounds[lb] - halfX,
                bounds[lb + 1] - halfY,
                bounds[lb + 2] - halfZ,
                bounds[lb + 3] + halfX,
                bounds[lb + 4] + halfY,
                bounds[lb + 5] + halfZ,
            );
            const rightDist = rayDistanceToBox3(
                originX,
                originY,
                originZ,
                dirX,
                dirY,
                dirZ,
                castLen,
                bounds[rb] - halfX,
                bounds[rb + 1] - halfY,
                bounds[rb + 2] - halfZ,
                bounds[rb + 3] + halfX,
                bounds[rb + 4] + halfY,
                bounds[rb + 5] + halfZ,
            );

            // push furthest first (closest popped first), culling children past best-t
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
        const body = world.bodies.pool[topo[nodeIndex * STRIDE_TOPO + T_BODY]];

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

        // a hit may have shrunk the collector's early-out fraction — tighten the pruning bound
        bestFraction = visitor.earlyOutFraction ?? Infinity;
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
