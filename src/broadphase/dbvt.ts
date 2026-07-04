import { type Box3, box3, raycast3, type Vec3, vec3 } from 'mathcat';
import type { RigidBody } from '../body/rigid-body';
import { rayDistanceToBox3, rayHitsBox3 } from '../collision/cast-utils';
import type { Filter } from '../filter';
import * as filter from '../filter';
import type { World } from '../world';
import type { BodyVisitor } from './body-visitor';

export type DBVT = {
    nodes: DBVTNode[];
    freeNodeIndices: number[];
    root: number;

    expansionMargin: number;

    // any structural change (add / remove / fat-leaf escape) since the last rebuild. gates
    // the dirty-gated rebuild in broadphase.optimize — a tree that no body has disturbed
    // (e.g. a settled static field) stays clean and is never rebuilt. jolt's IsDirty().
    dirty: boolean;
};

export type DBVTNode = {
    index: number;

    parent: number;
    left: number;
    right: number;

    aabb: Box3;

    // internal nodes only (meaningless on leaves — mark/widen starts at a leaf's parent).
    // invariant: changed ⇒ parent.changed. set when a descendant leaf's fat aabb widens or
    // the local structure changes; a rebuild keeps every !changed subtree whole and only
    // re-partitions the changed region + the always-rebuilt top levels. jolt's Node::mIsChanged.
    changed: boolean;

    bodyIndex: number;
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
        nodes: [],
        freeNodeIndices: [],
        root: -1,
        expansionMargin: 0.05,
        dirty: false,
    };

    return dbvt;
}

function requestNode(bvh: DBVT): number {
    let nodeIndex: number;
    if (bvh.freeNodeIndices.length > 0) {
        nodeIndex = bvh.freeNodeIndices.pop()!;
        const node = bvh.nodes[nodeIndex];
        node.parent = -1;
        node.left = -1;
        node.right = -1;
        box3.empty(node.aabb);
        node.changed = false;
        node.bodyIndex = -1;
    } else {
        nodeIndex = bvh.nodes.length;
        bvh.nodes.push({
            index: nodeIndex,
            parent: -1,
            left: -1,
            right: -1,
            aabb: box3.create(),
            changed: false,
            bodyIndex: -1,
        });
    }
    return nodeIndex;
}

function releaseNode(bvh: DBVT, nodeIndex: number): void {
    const node = bvh.nodes[nodeIndex];
    node.parent = -1;
    node.left = -1;
    node.right = -1;
    node.changed = false;
    node.bodyIndex = -1;
    bvh.freeNodeIndices.push(nodeIndex);
}

function isLeaf(node: DBVTNode): boolean {
    return node.left === -1 && node.right === -1;
}

function proximity(a: Box3, b: Box3): number {
    const dx = a[0] + a[3] - (b[0] + b[3]);
    const dy = a[1] + a[4] - (b[1] + b[4]);
    const dz = a[2] + a[5] - (b[2] + b[5]);
    return Math.abs(dx) + Math.abs(dy) + Math.abs(dz);
}

function select(o: Box3, a: Box3, b: Box3): number {
    return proximity(o, a) < proximity(o, b) ? 0 : 1;
}

function indexof(dbvt: DBVT, nodeIndex: number): number {
    const node = dbvt.nodes[nodeIndex];
    const parent = dbvt.nodes[node.parent];
    return parent.right === nodeIndex ? 1 : 0;
}

/** @optimize */
function insertLeaf(dbvt: DBVT, rootIndex: number, leafIndex: number): void {
    const leaf = dbvt.nodes[leafIndex];

    if (dbvt.root === -1) {
        dbvt.root = leafIndex;
        leaf.parent = -1;
        return;
    }

    // descend to find best leaf position
    let root = rootIndex;
    let rootNode = dbvt.nodes[root];
    while (!isLeaf(rootNode)) {
        const leftNode = dbvt.nodes[rootNode.left];
        const rightNode = dbvt.nodes[rootNode.right];
        const child = select(leaf.aabb, leftNode.aabb, rightNode.aabb);
        root = child === 0 ? rootNode.left : rootNode.right;
        rootNode = dbvt.nodes[root];
    }

    const prev = rootNode.parent;
    const newParentIndex = requestNode(dbvt);
    const newParent = dbvt.nodes[newParentIndex];

    newParent.parent = prev;
    box3.union(newParent.aabb, leaf.aabb, rootNode.aabb);

    if (prev !== -1) {
        const prevNode = dbvt.nodes[prev];
        if (indexof(dbvt, root) === 0) {
            prevNode.left = newParentIndex;
        } else {
            prevNode.right = newParentIndex;
        }
        newParent.left = root;
        rootNode.parent = newParentIndex;
        newParent.right = leafIndex;
        leaf.parent = newParentIndex;

        // refit: walk up the tree, checking if parent contains child
        let childNode = newParent;
        let parentIndex = prev;
        while (parentIndex !== -1) {
            const parentNode = dbvt.nodes[parentIndex];
            if (!box3.containsBox3(parentNode.aabb, childNode.aabb)) {
                const leftNode = dbvt.nodes[parentNode.left];
                const rightNode = dbvt.nodes[parentNode.right];
                box3.union(parentNode.aabb, leftNode.aabb, rightNode.aabb);
            } else {
                break;
            }
            childNode = parentNode;
            parentIndex = parentNode.parent;
        }
    } else {
        newParent.left = root;
        rootNode.parent = newParentIndex;
        newParent.right = leafIndex;
        leaf.parent = newParentIndex;
        dbvt.root = newParentIndex;
    }
}

const _prevAabb = /* @__PURE__ */ box3.create();

// walk up from a changed node marking ancestors changed. stops at the first already-changed
// ancestor (the invariant changed ⇒ parent.changed guarantees everything above is done).
// jolt's MarkNodeAndParentsChanged.
function markNodeAndParentsChanged(dbvt: DBVT, nodeIndex: number): void {
    let idx = nodeIndex;
    while (idx !== -1) {
        const node = dbvt.nodes[idx];
        if (node.changed) break;
        node.changed = true;
        idx = node.parent;
    }
}

// walk up from a leaf's parent widening each ancestor's aabb to encapsulate the (grown) leaf
// bounds and marking it changed. bounds only grow between rebuilds so containment holds at
// every instant (queries never miss); the next rebuild recomputes exact unions. once an
// ancestor already contains the new bounds, only the changed-marking remains. jolt's
// WidenAndMarkNodeAndParentsChanged, adapted to bounds-on-node.
function widenAndMarkNodeAndParentsChanged(dbvt: DBVT, parentIndex: number, newBounds: Box3): void {
    let idx = parentIndex;
    while (idx !== -1) {
        const node = dbvt.nodes[idx];
        node.changed = true;
        if (box3.containsBox3(node.aabb, newBounds)) {
            // containment is monotone up the tree — nothing above needs widening, only marking
            markNodeAndParentsChanged(dbvt, node.parent);
            break;
        }
        box3.union(node.aabb, node.aabb, newBounds);
        idx = node.parent;
    }
}

/* @optimize */
function removeLeaf(dbvt: DBVT, leafIndex: number): number {
    if (leafIndex === dbvt.root) {
        dbvt.root = -1;
        return -1;
    }

    const leaf = dbvt.nodes[leafIndex];
    const parentIndex = leaf.parent;
    const parent = dbvt.nodes[parentIndex];
    const prevIndex = parent.parent;
    const siblingIndex = parent.left === leafIndex ? parent.right : parent.left;
    const sibling = dbvt.nodes[siblingIndex];

    if (prevIndex !== -1) {
        const prev = dbvt.nodes[prevIndex];
        if (indexof(dbvt, parentIndex) === 0) {
            prev.left = siblingIndex;
        } else {
            prev.right = siblingIndex;
        }
        sibling.parent = prevIndex;
        releaseNode(dbvt, parentIndex);

        // the collapse restructured prev's children — mark it and its ancestors changed so the
        // next rebuild re-partitions this region (invariant: changed ⇒ parent.changed).
        markNodeAndParentsChanged(dbvt, prevIndex);

        // refit
        let nodeIndex = prevIndex;
        while (nodeIndex !== -1) {
            const node = dbvt.nodes[nodeIndex];
            box3.copy(_prevAabb, node.aabb);

            const leftNode = dbvt.nodes[node.left];
            const rightNode = dbvt.nodes[node.right];
            box3.union(node.aabb, leftNode.aabb, rightNode.aabb);

            if (!box3.exactEquals(node.aabb, _prevAabb)) {
                nodeIndex = node.parent;
            } else {
                break;
            }
        }

        return prevIndex !== -1 ? prevIndex : dbvt.root;
    } else {
        dbvt.root = siblingIndex;
        sibling.parent = -1;
        releaseNode(dbvt, parentIndex);
        return dbvt.root;
    }
}

export function add(dbvt: DBVT, body: RigidBody): number {
    // create leaf node with fat (margin-expanded) bounds
    const leafIndex = requestNode(dbvt);
    const leaf = dbvt.nodes[leafIndex];
    box3.expandByMargin(leaf.aabb, body.aabb, dbvt.expansionMargin);
    leaf.bodyIndex = body.index;

    // greedy insert so the body is immediately queryable (a rebuild only happens at the next
    // step); the balanced re-partition is deferred to the dirty-gated rebuild.
    insertLeaf(dbvt, dbvt.root, leafIndex);

    // mark the new leaf's ancestor chain changed + flag the tree for rebuild
    const parent = dbvt.nodes[leafIndex].parent;
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

    const leaf = dbvt.nodes[leafIndex];

    // early exit: if body still fits in the fat AABB, nothing to do
    if (box3.containsBox3(leaf.aabb, body.aabb)) {
        return false;
    }

    // replace the leaf fat box in place — bit-identical values, at the identical escape moments,
    // as the old remove+reinsert path, so persistent-pair discovery/sweep see unchanged fat-leaf data
    box3.expandByMargin(leaf.aabb, body.aabb, dbvt.expansionMargin);

    // widen ancestors to keep containment (grow-only until rebuild) and mark the path changed
    if (leaf.parent !== -1) widenAndMarkNodeAndParentsChanged(dbvt, leaf.parent, leaf.aabb);
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
// frontier is f+1 units and the build allocates exactly f internals → nodes.length is invariant
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
            const parent = dbvt.nodes[p];
            const left = dbvt.nodes[li];
            const right = dbvt.nodes[ri];
            parent.left = li;
            parent.right = ri;
            left.parent = p;
            right.parent = p;
            box3.union(parent.aabb, left.aabb, right.aabb);
            parent.changed = depth < maxDepthMarkChanged;
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

    // phase 1: collect units (leaves + unchanged-internal subtree roots) and free changed internals
    _buildUnits.length = 0;
    _collectStack.length = 0;
    _collectStack.push(dbvt.root);
    while (_collectStack.length > 0) {
        const idx = _collectStack.pop()!;
        const node = dbvt.nodes[idx];
        // leaf-ness tested BEFORE changed: leaves are always kept as units (never freed), preserving
        // body.dbvtNode; an unchanged internal grafts whole; a changed internal is opened + recycled
        if (node.left === -1 || !node.changed) {
            _buildUnits.push(idx);
        } else {
            _collectStack.push(node.left);
            _collectStack.push(node.right);
            releaseNode(dbvt, idx);
        }
    }

    const m = _buildUnits.length;
    if (m === 1) {
        // lone leaf, or the whole tree was unchanged — graft as root
        dbvt.root = _buildUnits[0];
        dbvt.nodes[dbvt.root].parent = -1;
        return;
    }

    // phase 2: unit centers (interleaved xyz), consumed + reordered in place by the build
    for (let k = 0; k < m; k++) {
        const aabb = dbvt.nodes[_buildUnits[k]].aabb;
        _buildCenters[k * 3] = (aabb[0] + aabb[3]) * 0.5;
        _buildCenters[k * 3 + 1] = (aabb[1] + aabb[4]) * 0.5;
        _buildCenters[k * 3 + 2] = (aabb[2] + aabb[5]) * 0.5;
    }

    // phase 3: median-split build over the units
    dbvt.root = buildTree(dbvt, 0, m, maxDepthMarkChanged);
    dbvt.nodes[dbvt.root].parent = -1;
}

/**
 * Overlap traversal against the FAT leaf AABBs — no tight body-AABB re-test and no filtering.
 * Used by persistent-pair discovery: a pair must exist whenever the two fat boxes could
 * bring the bodies into contact while both coast inside them, so the leaf test must be the
 * fat node AABB (already tested during descent), NOT the current tight body AABB.
 */
export function intersectAABBFatLeaves(world: World, dbvt: DBVT, aabb: Box3, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

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
        const node = dbvt.nodes[nodeIndex];

        // node aabb test (for a leaf this IS the fat leaf test)
        if (
            node.aabb[0] > qMaxX ||
            node.aabb[3] < qMinX ||
            node.aabb[1] > qMaxY ||
            node.aabb[4] < qMinY ||
            node.aabb[2] > qMaxZ ||
            node.aabb[5] < qMinZ
        ) {
            continue;
        }

        if (!isLeaf(node)) {
            if (node.left !== -1) _flatStack[stackSize++] = node.left;
            if (node.right !== -1) _flatStack[stackSize++] = node.right;
            continue;
        }

        visitor.visit(world.bodies.pool[node.bodyIndex]);

        if (visitor.shouldExit) {
            return;
        }
    }
}

export function intersectAABB(world: World, dbvt: DBVT, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void {
    if (dbvt.root === -1) return;

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
        const node = dbvt.nodes[nodeIndex];

        // node aabb test
        if (
            node.aabb[0] > qMaxX ||
            node.aabb[3] < qMinX ||
            node.aabb[1] > qMaxY ||
            node.aabb[4] < qMinY ||
            node.aabb[2] > qMaxZ ||
            node.aabb[5] < qMinZ
        ) {
            continue;
        }

        // if internal node, push children
        if (!isLeaf(node)) {
            if (node.left !== -1) _flatStack[stackSize++] = node.left;
            if (node.right !== -1) _flatStack[stackSize++] = node.right;
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];

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

    const px = point[0];
    const py = point[1];
    const pz = point[2];

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];
        const node = dbvt.nodes[nodeIndex];

        // skip if point is not inside node's AABB
        if (
            px < node.aabb[0] ||
            px > node.aabb[3] ||
            py < node.aabb[1] ||
            py > node.aabb[4] ||
            pz < node.aabb[2] ||
            pz > node.aabb[5]
        ) {
            continue;
        }

        // if internal node, push children
        if (!isLeaf(node)) {
            if (node.left !== -1) _flatStack[stackSize++] = node.left;
            if (node.right !== -1) _flatStack[stackSize++] = node.right;
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];
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

    let stackSize = 0;
    _flatStack[stackSize++] = dbvt.root;

    while (stackSize > 0) {
        const nodeIndex = _flatStack[--stackSize];
        const node = dbvt.nodes[nodeIndex];

        if (!isLeaf(node)) {
            if (node.left !== -1) _flatStack[stackSize++] = node.left;
            if (node.right !== -1) _flatStack[stackSize++] = node.right;
            continue;
        }

        const body = world.bodies.pool[node.bodyIndex];
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
    // [0, 1] of the ray length (rayDistanceToBox3), matching the collector's fraction. visitors
    // that don't cast omit it → Infinity → distance pruning off (only misses rejected).
    let bestFraction = visitor.earlyOutFraction ?? Infinity;

    let stackSize = 0;
    _castStackNode[stackSize] = dbvt.root;
    _castStackDist[stackSize] = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, rayLen, dbvt.nodes[dbvt.root].aabb);
    stackSize++;

    while (stackSize > 0) {
        stackSize--;
        const nodeIndex = _castStackNode[stackSize];
        const nodeDistance = _castStackDist[stackSize];
        const node = dbvt.nodes[nodeIndex];

        // prune: skip misses (Infinity distance) and nodes whose entry is beyond the closest hit
        // found so far (best-t). the node's own aabb was already ray-tested when its parent pushed
        // it — a finite stored distance means it hits — so re-testing node.aabb here is redundant.
        if (nodeDistance >= bestFraction) {
            continue;
        }

        // if internal node, push children sorted by distance, culling any beyond best-t
        if (!isLeaf(node)) {
            const leftNode = dbvt.nodes[node.left];
            const rightNode = dbvt.nodes[node.right];

            const leftDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, rayLen, leftNode.aabb);
            const rightDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, rayLen, rightNode.aabb);

            // push in reverse order (furthest first) so closest is popped first
            if (leftDist < rightDist) {
                if (rightDist < bestFraction) {
                    _castStackNode[stackSize] = node.right;
                    _castStackDist[stackSize] = rightDist;
                    stackSize++;
                }
                if (leftDist < bestFraction) {
                    _castStackNode[stackSize] = node.left;
                    _castStackDist[stackSize] = leftDist;
                    stackSize++;
                }
            } else {
                if (leftDist < bestFraction) {
                    _castStackNode[stackSize] = node.left;
                    _castStackDist[stackSize] = leftDist;
                    stackSize++;
                }
                if (rightDist < bestFraction) {
                    _castStackNode[stackSize] = node.right;
                    _castStackDist[stackSize] = rightDist;
                    stackSize++;
                }
            }
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];

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
        const node = dbvt.nodes[nodeIndex];

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
                node.aabb[0] - halfX,
                node.aabb[1] - halfY,
                node.aabb[2] - halfZ,
                node.aabb[3] + halfX,
                node.aabb[4] + halfY,
                node.aabb[5] + halfZ,
            )
        ) {
            continue;
        }

        // if internal node, push children sorted by distance
        if (!isLeaf(node)) {
            const leftNode = dbvt.nodes[node.left];
            const rightNode = dbvt.nodes[node.right];

            _nodeBounds[0] = leftNode.aabb[0] - halfX;
            _nodeBounds[1] = leftNode.aabb[1] - halfY;
            _nodeBounds[2] = leftNode.aabb[2] - halfZ;
            _nodeBounds[3] = leftNode.aabb[3] + halfX;
            _nodeBounds[4] = leftNode.aabb[4] + halfY;
            _nodeBounds[5] = leftNode.aabb[5] + halfZ;
            const leftDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, castLen, _nodeBounds);

            _nodeBounds[0] = rightNode.aabb[0] - halfX;
            _nodeBounds[1] = rightNode.aabb[1] - halfY;
            _nodeBounds[2] = rightNode.aabb[2] - halfZ;
            _nodeBounds[3] = rightNode.aabb[3] + halfX;
            _nodeBounds[4] = rightNode.aabb[4] + halfY;
            _nodeBounds[5] = rightNode.aabb[5] + halfZ;
            const rightDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, castLen, _nodeBounds);

            // push in reverse order (furthest first) so closest is popped first
            if (leftDist < rightDist) {
                if (node.right !== -1) {
                    _castStackNode[stackSize] = node.right;
                    _castStackDist[stackSize] = rightDist;
                    stackSize++;
                }
                if (node.left !== -1) {
                    _castStackNode[stackSize] = node.left;
                    _castStackDist[stackSize] = leftDist;
                    stackSize++;
                }
            } else {
                if (node.left !== -1) {
                    _castStackNode[stackSize] = node.left;
                    _castStackDist[stackSize] = leftDist;
                    stackSize++;
                }
                if (node.right !== -1) {
                    _castStackNode[stackSize] = node.right;
                    _castStackDist[stackSize] = rightDist;
                    stackSize++;
                }
            }
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];
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

    const rootNode = dbvt.nodes[dbvt.root];
    return box3.copy(out, rootNode.aabb);
}
