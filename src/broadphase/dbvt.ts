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

// flat SMI stack of node indices for overlap traversals (intersectAABB / intersectPoint /
// walk) — they carry no distances. grow-once with a size counter (no pop()/length churn,
// stays packed).
const _flatStack: number[] = [];

// parallel flat stacks for cast traversals: node indices (SMI array) + distances (double
// array) sharing one size counter — jolt keeps the same structure (a distance stack
// parallel to the node stack). separate arrays so each keeps its optimal element kind.
const _castStackNode: number[] = [];
const _castStackDist: number[] = [];

export function create(): DBVT {
    const dbvt: DBVT = {
        nodes: [],
        freeNodeIndices: [],
        root: -1,
        expansionMargin: 0.05,
        optimizationPath: 0,
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
        node.height = 0;
        node.bodyIndex = -1;
    } else {
        nodeIndex = bvh.nodes.length;
        bvh.nodes.push({
            index: nodeIndex,
            parent: -1,
            left: -1,
            right: -1,
            aabb: box3.create(),
            height: 0,
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
    newParent.height = rootNode.height + 1;

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
            if (!(box3.containsBox3(parentNode.aabb, childNode.aabb))) {
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

// todo: stack impl?
function fetchLeaves(dbvt: DBVT, rootIndex: number, leaves: number[], depth = -1): void {
    if (rootIndex === -1) return;

    const root = dbvt.nodes[rootIndex];
    if (isLeaf(root) || depth === 0) {
        leaves.push(rootIndex);
    } else {
        fetchLeaves(dbvt, root.left, leaves, depth - 1);
        fetchLeaves(dbvt, root.right, leaves, depth - 1);
    }
}

function surfaceArea(aabb: Box3): number {
    const sx = aabb[3] - aabb[0];
    const sy = aabb[4] - aabb[1];
    const sz = aabb[5] - aabb[2];
    return 2 * (sx * sy + sx * sz + sy * sz);
}

const _boundsResult = /* @__PURE__ */ box3.create();

/* @optimize */
function boundsOfLeaves(dbvt: DBVT, leaves: number[]): Box3 {
    const result = _boundsResult;
    box3.empty(result);
    for (const leafIndex of leaves) {
        const leaf = dbvt.nodes[leafIndex];
        box3.union(result, result, leaf.aabb);
    }
    return result;
}

const _mergedAabb = /* @__PURE__ */ box3.create();

function bottomup(dbvt: DBVT, leaves: number[]): void {
    while (leaves.length > 1) {
        let minSize = Number.POSITIVE_INFINITY;
        let minIdx0 = -1;
        let minIdx1 = -1;

        // find pair with smallest merged AABB
        for (let i = 0; i < leaves.length; i++) {
            for (let j = i + 1; j < leaves.length; j++) {
                const leaf0 = dbvt.nodes[leaves[i]];
                const leaf1 = dbvt.nodes[leaves[j]];
                box3.union(_mergedAabb, leaf0.aabb, leaf1.aabb);
                const sz = surfaceArea(_mergedAabb);
                if (sz < minSize) {
                    minSize = sz;
                    minIdx0 = i;
                    minIdx1 = j;
                }
            }
        }

        // merge the pair
        const n0Index = leaves[minIdx0];
        const n1Index = leaves[minIdx1];
        const n0 = dbvt.nodes[n0Index];
        const n1 = dbvt.nodes[n1Index];

        const parentIndex = requestNode(dbvt);
        const parent = dbvt.nodes[parentIndex];
        box3.union(parent.aabb, n0.aabb, n1.aabb);
        parent.left = n0Index;
        parent.right = n1Index;
        n0.parent = parentIndex;
        n1.parent = parentIndex;
        parent.height = Math.max(n0.height, n1.height) + 1;

        leaves[minIdx0] = parentIndex;
        leaves[minIdx1] = leaves[leaves.length - 1];
        leaves.pop();
    }
}

const _center: Vec3 = [0, 0, 0];

function topdown(dbvt: DBVT, leaves: number[], buThreshold: number): number {
    if (leaves.length > 1) {
        if (leaves.length > buThreshold) {
            const vol = boundsOfLeaves(dbvt, leaves);
            const org = [(vol[0] + vol[3]) * 0.5, (vol[1] + vol[4]) * 0.5, (vol[2] + vol[5]) * 0.5] as Vec3;

            // find best axis to split on
            let bestAxis = -1;
            let bestMidp = leaves.length;
            const splitCount = [
                [0, 0],
                [0, 0],
                [0, 0],
            ];

            for (const leafIndex of leaves) {
                const leaf = dbvt.nodes[leafIndex];
                _center[0] = (leaf.aabb[0] + leaf.aabb[3]) * 0.5;
                _center[1] = (leaf.aabb[1] + leaf.aabb[4]) * 0.5;
                _center[2] = (leaf.aabb[2] + leaf.aabb[5]) * 0.5;

                for (let j = 0; j < 3; j++) {
                    splitCount[j][_center[j] > org[j] ? 1 : 0]++;
                }
            }

            for (let i = 0; i < 3; i++) {
                if (splitCount[i][0] > 0 && splitCount[i][1] > 0) {
                    const midp = Math.abs(splitCount[i][0] - splitCount[i][1]);
                    if (midp < bestMidp) {
                        bestAxis = i;
                        bestMidp = midp;
                    }
                }
            }

            if (bestAxis >= 0) {
                // partition leaves along best axis
                const sets: [number[], number[]] = [[], []];

                for (const leafIndex of leaves) {
                    const leaf = dbvt.nodes[leafIndex];
                    _center[0] = (leaf.aabb[0] + leaf.aabb[3]) * 0.5;
                    _center[1] = (leaf.aabb[1] + leaf.aabb[4]) * 0.5;
                    _center[2] = (leaf.aabb[2] + leaf.aabb[5]) * 0.5;
                    const side = _center[bestAxis] > org[bestAxis] ? 1 : 0;
                    sets[side].push(leafIndex);
                }

                // recursively build subtrees
                const leftIndex = topdown(dbvt, sets[0], buThreshold);
                const rightIndex = topdown(dbvt, sets[1], buThreshold);

                const left = dbvt.nodes[leftIndex];
                const right = dbvt.nodes[rightIndex];

                const parentIndex = requestNode(dbvt);
                const parent = dbvt.nodes[parentIndex];
                box3.union(parent.aabb, left.aabb, right.aabb);
                parent.left = leftIndex;
                parent.right = rightIndex;
                left.parent = parentIndex;
                right.parent = parentIndex;
                parent.height = Math.max(left.height, right.height) + 1;

                return parentIndex;
            } else {
                // couldn't find good split, fall back to bottomup
                bottomup(dbvt, leaves);
                return leaves[0];
            }
        } else {
            // count <= threshold, use bottomup
            bottomup(dbvt, leaves);
            return leaves[0];
        }
    } else {
        return leaves[0];
    }
}

/* @optimize */
function sort(dbvt: DBVT, nodeIndex: number): number {
    const n = dbvt.nodes[nodeIndex];
    const parentIndex = n.parent;
    if (parentIndex === -1) return nodeIndex;

    const p = dbvt.nodes[parentIndex];

    // heuristic: rotate if parent index > node index
    if (parentIndex > nodeIndex) {
        const i = indexof(dbvt, nodeIndex);
        const siblingIndex = i === 0 ? p.right : p.left;
        const s = dbvt.nodes[siblingIndex];
        const grandparentIndex = p.parent;

        if (grandparentIndex !== -1) {
            const q = dbvt.nodes[grandparentIndex];
            if (indexof(dbvt, parentIndex) === 0) {
                q.left = nodeIndex;
            } else {
                q.right = nodeIndex;
            }
        } else {
            dbvt.root = nodeIndex;
        }

        s.parent = nodeIndex;
        p.parent = nodeIndex;
        n.parent = grandparentIndex;

        const n0Index = n.left;
        const n1Index = n.right;
        p.left = n0Index;
        p.right = n1Index;
        dbvt.nodes[n0Index].parent = parentIndex;
        dbvt.nodes[n1Index].parent = parentIndex;

        if (i === 0) {
            n.left = parentIndex;
            n.right = siblingIndex;
        } else {
            n.left = siblingIndex;
            n.right = parentIndex;
        }

        // swap volumes
        const tempAabb = box3.create();
        box3.copy(tempAabb, p.aabb);
        box3.copy(p.aabb, n.aabb);
        box3.copy(n.aabb, tempAabb);

        return parentIndex;
    }

    return nodeIndex;
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

        // refit
        let nodeIndex = prevIndex;
        while (nodeIndex !== -1) {
            const node = dbvt.nodes[nodeIndex];
            box3.copy(_prevAabb, node.aabb);

            const leftNode = dbvt.nodes[node.left];
            const rightNode = dbvt.nodes[node.right];
            box3.union(node.aabb, leftNode.aabb, rightNode.aabb);

            if (!(box3.exactEquals(node.aabb, _prevAabb))) {
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

const _bounds = /* @__PURE__ */ box3.create();

export function add(dbvt: DBVT, body: RigidBody): number {
    const bounds = box3.create();

    // expand body bounds by margin
    box3.expandByMargin(bounds, body.aabb, dbvt.expansionMargin);

    // create leaf node
    const leafIndex = requestNode(dbvt);
    const leaf = dbvt.nodes[leafIndex];
    box3.copy(leaf.aabb, bounds);
    leaf.bodyIndex = body.index;
    leaf.height = 0;

    // insert into tree
    insertLeaf(dbvt, dbvt.root, leafIndex);

    return leafIndex;
}

export function remove(dbvt: DBVT, body: RigidBody): void {
    const leafIndex = body.dbvtNode;
    if (leafIndex === -1) return;

    removeLeaf(dbvt, leafIndex);
    releaseNode(dbvt, leafIndex);
    body.dbvtNode = -1;
}

/* @optimize */
export function update(dbvt: DBVT, body: RigidBody, lookahead: number): void {
    const leafIndex = body.dbvtNode;
    if (leafIndex === -1) return;

    const leaf = dbvt.nodes[leafIndex];

    // early exit: if body still fits in the fat AABB, nothing to do
    if (box3.containsBox3(leaf.aabb, body.aabb)) {
        return;
    }

    // expand body bounds by margin for fat AABB.
    // margin-only (no motion-directed padding) matches the reference consensus:
    // bullet ships its size-based directional pad disabled, box2d v3 removed the
    // displacement pad from the lineage, and jolt widens nodes in place instead.
    box3.expandByMargin(_bounds, body.aabb, dbvt.expansionMargin);

    // remove leaf and get root for reinsertion
    let rootIndex = removeLeaf(dbvt, leafIndex);

    // walk up from returned root using lookahead
    if (rootIndex !== -1) {
        if (lookahead >= 0) {
            // walk up `lookahead` levels from the returned root
            for (let i = 0; i < lookahead && rootIndex !== -1; i++) {
                const root = dbvt.nodes[rootIndex];
                rootIndex = root.parent;
            }
            // if we walked off the tree, use tree root
            if (rootIndex === -1) {
                rootIndex = dbvt.root;
            }
        } else {
            // use tree root
            rootIndex = dbvt.root;
        }
    }

    // update leaf volume
    box3.copy(leaf.aabb, _bounds);

    // reinsert from computed root
    insertLeaf(dbvt, rootIndex, leafIndex);
}

export function optimizeBottomUp(dbvt: DBVT): void {
    if (dbvt.root === -1) return;

    const leaves: number[] = [];
    fetchLeaves(dbvt, dbvt.root, leaves);
    bottomup(dbvt, leaves);
    dbvt.root = leaves[0];
}

export function optimizeTopDown(dbvt: DBVT, buThreshold = 128): void {
    if (dbvt.root === -1) return;

    const leaves: number[] = [];
    fetchLeaves(dbvt, dbvt.root, leaves);
    dbvt.root = topdown(dbvt, leaves, buThreshold);
}

export function optimizeIncremental(dbvt: DBVT, passes: number): void {
    if (dbvt.root === -1) return;
    if (passes < 0) {
        // negative passes means optimize all leaves
        const leaves: number[] = [];
        fetchLeaves(dbvt, dbvt.root, leaves);
        passes = leaves.length;
    }
    if (passes === 0) return;

    for (let i = 0; i < passes; i++) {
        let nodeIndex = dbvt.root;
        let bit = 0;

        // descend following optimization path bits
        while (nodeIndex !== -1) {
            const node = dbvt.nodes[nodeIndex];
            if (isLeaf(node)) break;

            nodeIndex = sort(dbvt, nodeIndex);
            const sortedNode = dbvt.nodes[nodeIndex];
            const childBit = (dbvt.optimizationPath >> bit) & 1;
            nodeIndex = childBit === 0 ? sortedNode.left : sortedNode.right;
            bit = (bit + 1) & 31; // wrap at 32 bits
        }

        // update the leaf
        if (nodeIndex !== -1) {
            const rootIndex = removeLeaf(dbvt, nodeIndex);
            if (rootIndex !== -1) {
                insertLeaf(dbvt, rootIndex, nodeIndex);
            } else {
                insertLeaf(dbvt, dbvt.root, nodeIndex);
            }
        }

        dbvt.optimizationPath++;
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

    let stackSize = 0;
    _castStackNode[stackSize] = dbvt.root;
    _castStackDist[stackSize] = -Infinity; // root always visited
    stackSize++;

    while (stackSize > 0) {
        stackSize--;
        const nodeIndex = _castStackNode[stackSize];
        const nodeDistance = _castStackDist[stackSize];
        const node = dbvt.nodes[nodeIndex];

        // early-out: skip nodes beyond ray length
        if (nodeDistance > length) {
            continue;
        }

        // early out: ray x node aabb
        if (
            !rayHitsBox3(
                originX, originY, originZ,
                dirX, dirY, dirZ,
                rayLen,
                node.aabb[0], node.aabb[1], node.aabb[2],
                node.aabb[3], node.aabb[4], node.aabb[5],
            )
        ) {
            continue;
        }

        // if internal node, push children sorted by distance
        if (!isLeaf(node)) {
            const leftNode = dbvt.nodes[node.left];
            const rightNode = dbvt.nodes[node.right];

            const leftDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, rayLen, leftNode.aabb);
            const rightDist = rayDistanceToBox3(originX, originY, originZ, dirX, dirY, dirZ, rayLen, rightNode.aabb);

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
                originX, originY, originZ,
                dirX, dirY, dirZ,
                rayLen,
                body.aabb[0], body.aabb[1], body.aabb[2],
                body.aabb[3], body.aabb[4], body.aabb[5],
            )
        ) {
            continue;
        }

        // visit
        visitor.visit(body);

        if (visitor.shouldExit) {
            return;
        }
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
                originX, originY, originZ,
                dirX, dirY, dirZ,
                castLen,
                node.aabb[0] - halfX, node.aabb[1] - halfY, node.aabb[2] - halfZ,
                node.aabb[3] + halfX, node.aabb[4] + halfY, node.aabb[5] + halfZ,
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
                originX, originY, originZ,
                dirX, dirY, dirZ,
                castLen,
                body.aabb[0] - halfX, body.aabb[1] - halfY, body.aabb[2] - halfZ,
                body.aabb[3] + halfX, body.aabb[4] + halfY, body.aabb[5] + halfZ,
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
