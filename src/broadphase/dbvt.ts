import { type Box3, box3, raycast3, type Vec3, vec3 } from 'mathcat';
import type { RigidBody } from '../body/rigid-body';
import { rayDistanceToBox3 } from '../collision/cast-utils';
import type { Filter } from '../filter';
import * as filter from '../filter';
import * as bvhStack from '../utils/bvh-stack';
import type { World } from '../world';
import type { BodyVisitor } from './body-visitor';

export type DBVT = {
    nodes: DBVTNode[];
    freeNodeIndices: number[];
    root: number;

    expansionMargin: number;
    optimizationPath: number;

    /**
     * Velocity prediction factor for AABB expansion
     *
     * When > 0, expands AABBs in the direction of motion to reduce update frequency.
     * Higher values = fewer updates but larger AABBs (more false positives in broadphase).
     *
     * @default 0.0
     */
    velocityPrediction: number;
};

export type DBVTNode = {
    index: number;

    parent: number;
    left: number;
    right: number;

    aabb: Box3;
    height: number;

    bodyIndex: number;

    previousAabb: Box3;
};

const _stack = /* @__PURE__ */ bvhStack.create(128);

export function create(): DBVT {
    const dbvt: DBVT = {
        nodes: [],
        freeNodeIndices: [],
        root: -1,
        expansionMargin: 0.05,
        optimizationPath: 0,
        velocityPrediction: 0.0,
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
        box3.empty(node.previousAabb);
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
            previousAabb: box3.create(),
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
    const dx = a[0][0] + a[1][0] - (b[0][0] + b[1][0]);
    const dy = a[0][1] + a[1][1] - (b[0][1] + b[1][1]);
    const dz = a[0][2] + a[1][2] - (b[0][2] + b[1][2]);
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
    const sx = aabb[1][0] - aabb[0][0];
    const sy = aabb[1][1] - aabb[0][1];
    const sz = aabb[1][2] - aabb[0][2];
    return 2 * (sx * sy + sx * sz + sy * sz);
}

const _boundsResult = /* @__PURE__ */ box3.create();

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
            const org = [(vol[0][0] + vol[1][0]) * 0.5, (vol[0][1] + vol[1][1]) * 0.5, (vol[0][2] + vol[1][2]) * 0.5] as Vec3;

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
                _center[0] = (leaf.aabb[0][0] + leaf.aabb[1][0]) * 0.5;
                _center[1] = (leaf.aabb[0][1] + leaf.aabb[1][1]) * 0.5;
                _center[2] = (leaf.aabb[0][2] + leaf.aabb[1][2]) * 0.5;

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
                    _center[0] = (leaf.aabb[0][0] + leaf.aabb[1][0]) * 0.5;
                    _center[1] = (leaf.aabb[0][1] + leaf.aabb[1][1]) * 0.5;
                    _center[2] = (leaf.aabb[0][2] + leaf.aabb[1][2]) * 0.5;
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

const _bounds = /* @__PURE__ */ box3.create();

export function add(dbvt: DBVT, body: RigidBody): number {
    // expand body bounds by margin
    box3.expandByMargin(_bounds, body.aabb, dbvt.expansionMargin);

    // create leaf node
    const leafIndex = requestNode(dbvt);
    const leaf = dbvt.nodes[leafIndex];
    box3.copy(leaf.aabb, _bounds);
    leaf.bodyIndex = body.index;
    leaf.height = 0;

    // initialize previous AABB for velocity prediction
    box3.copy(leaf.previousAabb, body.aabb);

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

export function update(dbvt: DBVT, body: RigidBody, lookahead: number): void {
    const leafIndex = body.dbvtNode;
    if (leafIndex === -1) return;

    const leaf = dbvt.nodes[leafIndex];

    // early exit: if body still fits in the fat AABB, nothing to do
    if (box3.containsBox3(leaf.aabb, body.aabb)) {
        return;
    }

    // expand body bounds by margin for fat AABB
    box3.expandByMargin(_bounds, body.aabb, dbvt.expansionMargin);

    // velocity-based expansion, expands AABB only in the direction of motion to reduce update frequency
    if (dbvt.velocityPrediction > 0) {
        // compute delta from AABB min movement
        const deltaX = body.aabb[0][0] - leaf.previousAabb[0][0];
        const deltaY = body.aabb[0][1] - leaf.previousAabb[0][1];
        const deltaZ = body.aabb[0][2] - leaf.previousAabb[0][2];

        // compute half extents from PREVIOUS AABB
        const halfExtentX = (leaf.previousAabb[1][0] - leaf.previousAabb[0][0]) * 0.5;
        const halfExtentY = (leaf.previousAabb[1][1] - leaf.previousAabb[0][1]) * 0.5;
        const halfExtentZ = (leaf.previousAabb[1][2] - leaf.previousAabb[0][2]) * 0.5;

        // velocity expansion = (half extents) * prediction factor
        let velocityX = halfExtentX * dbvt.velocityPrediction;
        let velocityY = halfExtentY * dbvt.velocityPrediction;
        let velocityZ = halfExtentZ * dbvt.velocityPrediction;

        // apply sign based on movement direction
        if (deltaX < 0) velocityX = -velocityX;
        if (deltaY < 0) velocityY = -velocityY;
        if (deltaZ < 0) velocityZ = -velocityZ;

        // expand min/max based on velocity direction
        if (velocityX > 0) {
            _bounds[1][0] += velocityX;
        } else {
            _bounds[0][0] += velocityX;
        }
        if (velocityY > 0) {
            _bounds[1][1] += velocityY;
        } else {
            _bounds[0][1] += velocityY;
        }
        if (velocityZ > 0) {
            _bounds[1][2] += velocityZ;
        } else {
            _bounds[0][2] += velocityZ;
        }
    }

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

    // update previous AABB for next velocity prediction
    box3.copy(leaf.previousAabb, body.aabb);
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

    bvhStack.reset(_stack);
    bvhStack.push(_stack, dbvt.root, 0);

    while (_stack.size > 0) {
        const entry = bvhStack.pop(_stack)!;
        const nodeIndex = entry.nodeIndex;
        const node = dbvt.nodes[nodeIndex];

        // skip if node doesn't intersect the AABB
        if (!box3.intersectsBox3(node.aabb, aabb)) {
            continue;
        }

        // if internal node, push children
        if (!isLeaf(node)) {
            if (node.left !== -1) bvhStack.push(_stack, node.left, 0);
            if (node.right !== -1) bvhStack.push(_stack, node.right, 0);
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];
        if (!body || body._pooled) continue;

        // collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroup,
                queryFilter.collisionMask,
                body.collisionGroup,
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

        // aabb test
        if (!box3.intersectsBox3(body.aabb, aabb)) {
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

    bvhStack.reset(_stack);
    bvhStack.push(_stack, dbvt.root, 0);

    while (_stack.size > 0) {
        const entry = bvhStack.pop(_stack)!;
        const nodeIndex = entry.nodeIndex;
        const node = dbvt.nodes[nodeIndex];

        // skip if point is not inside node's AABB
        if (!box3.containsPoint(node.aabb, point)) {
            continue;
        }

        // if internal node, push children
        if (!isLeaf(node)) {
            if (node.left !== -1) bvhStack.push(_stack, node.left, 0);
            if (node.right !== -1) bvhStack.push(_stack, node.right, 0);
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];
        if (!body || body._pooled) continue;

        // collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroup,
                queryFilter.collisionMask,
                body.collisionGroup,
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

        if (!box3.containsPoint(body.aabb, point)) {
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

    bvhStack.reset(_stack);
    bvhStack.push(_stack, dbvt.root, 0);

    while (_stack.size > 0) {
        const entry = bvhStack.pop(_stack)!;
        const nodeIndex = entry.nodeIndex;
        const node = dbvt.nodes[nodeIndex];

        if (!isLeaf(node)) {
            if (node.left !== -1) bvhStack.push(_stack, node.left, 0);
            if (node.right !== -1) bvhStack.push(_stack, node.right, 0);
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
const _rayDirection = /* @__PURE__ */ vec3.create();
const _rayOrigin = /* @__PURE__ */ vec3.create();
const _halfExtents = /* @__PURE__ */ vec3.create();
const _nodeBounds = /* @__PURE__ */ box3.create();
const _bodyBounds = /* @__PURE__ */ box3.create();

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
    bvhStack.reset(_stack);
    bvhStack.push(_stack, dbvt.root, -Infinity); // root always visited

    while (_stack.size > 0) {
        const entry = bvhStack.pop(_stack)!;
        const nodeIndex = entry.nodeIndex;
        const nodeDistance = entry.distance;
        const node = dbvt.nodes[nodeIndex];

        // early-out: skip nodes beyond ray length
        if (nodeDistance > length) {
            continue;
        }

        // early out: ray x node aabb
        if (!raycast3.intersectsBox3(_ray, node.aabb)) {
            continue;
        }

        // if internal node, push children sorted by distance
        if (!isLeaf(node)) {
            const leftNode = dbvt.nodes[node.left];
            const rightNode = dbvt.nodes[node.right];

            // calculate distances to children
            const leftDist = rayDistanceToBox3(_ray, leftNode.aabb);
            const rightDist = rayDistanceToBox3(_ray, rightNode.aabb);

            // push in reverse order (furthest first) so closest is popped first
            if (leftDist < rightDist) {
                if (node.right !== -1) bvhStack.push(_stack, node.right, rightDist);
                if (node.left !== -1) bvhStack.push(_stack, node.left, leftDist);
            } else {
                if (node.left !== -1) bvhStack.push(_stack, node.left, leftDist);
                if (node.right !== -1) bvhStack.push(_stack, node.right, rightDist);
            }
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];

        // early out: collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroup,
                queryFilter.collisionMask,
                body.collisionGroup,
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

        // early out: ray-aabb test
        if (!raycast3.intersectsBox3(_ray, body.aabb)) {
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
    // 2. Expand the block aabb by the extents of the shape aabb
    // 3. Cast the point by the displacement against the expanded block aabb (raycast vs aabb test)

    // Compute ray origin from AABB center
    _rayOrigin[0] = (bounds[0][0] + bounds[1][0]) * 0.5;
    _rayOrigin[1] = (bounds[0][1] + bounds[1][1]) * 0.5;
    _rayOrigin[2] = (bounds[0][2] + bounds[1][2]) * 0.5;
    vec3.normalize(_rayDirection, displacement);
    const rayLength = vec3.length(displacement);
    raycast3.set(_ray, _rayOrigin, _rayDirection, rayLength);

    box3.size(_halfExtents, bounds);
    vec3.scale(_halfExtents, _halfExtents, 0.5);

    bvhStack.reset(_stack);
    bvhStack.push(_stack, dbvt.root, -Infinity); // root always visited

    while (_stack.size > 0) {
        const entry = bvhStack.pop(_stack)!;
        const nodeIndex = entry.nodeIndex;
        const nodeDistance = entry.distance;
        const node = dbvt.nodes[nodeIndex];

        // early-out: skip nodes beyond cast length
        if (nodeDistance > rayLength) {
            continue;
        }

        // skip if node's expanded AABB doesn't intersect with the shape's ray
        box3.expandByExtents(_nodeBounds, node.aabb, _halfExtents);
        if (!raycast3.intersectsBox3(_ray, _nodeBounds)) {
            continue;
        }

        // if internal node, push children sorted by distance
        if (!isLeaf(node)) {
            const leftNode = dbvt.nodes[node.left];
            const rightNode = dbvt.nodes[node.right];

            // calculate distances to expanded children bounds
            box3.expandByExtents(_nodeBounds, leftNode.aabb, _halfExtents);
            const leftDist = rayDistanceToBox3(_ray, _nodeBounds);

            box3.expandByExtents(_nodeBounds, rightNode.aabb, _halfExtents);
            const rightDist = rayDistanceToBox3(_ray, _nodeBounds);

            // push in reverse order (furthest first) so closest is popped first
            if (leftDist < rightDist) {
                if (node.right !== -1) bvhStack.push(_stack, node.right, rightDist);
                if (node.left !== -1) bvhStack.push(_stack, node.left, leftDist);
            } else {
                if (node.left !== -1) bvhStack.push(_stack, node.left, leftDist);
                if (node.right !== -1) bvhStack.push(_stack, node.right, rightDist);
            }
            continue;
        }

        // leaf node - check body
        const body = world.bodies.pool[node.bodyIndex];
        if (!body || body._pooled) continue;

        // collision group/mask filtering
        if (
            !filter.shouldPairCollide(
                queryFilter.collisionGroup,
                queryFilter.collisionMask,
                body.collisionGroup,
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

        // expand the body aabb by the shape's half extents
        box3.expandByExtents(_bodyBounds, body.aabb, _halfExtents);
        if (!raycast3.intersectsBox3(_ray, _bodyBounds)) {
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
