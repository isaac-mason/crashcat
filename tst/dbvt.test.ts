import { box3, vec3 } from 'mathcat';
import { describe, expect, it } from 'vitest';
import { type BodyVisitor, box, dbvt, filter, MotionType, type RigidBody, rigidBody, type World } from '../src';
import { createTestWorld } from './helpers';

function makeBody(
    world: World,
    minX: number,
    minY: number,
    minZ: number,
    maxX: number,
    maxY: number,
    maxZ: number,
    collisionGroups = 0xffffffff,
    collisionMask = 0xffffffff,
) {
    const width = maxX - minX;
    const height = maxY - minY;
    const depth = maxZ - minZ;
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const centerZ = (minZ + maxZ) / 2;
    return rigidBody.create(world, {
        shape: box.create({ halfExtents: [width / 2, height / 2, depth / 2] }),
        motionType: MotionType.DYNAMIC,
        objectLayer: 0,
        position: vec3.fromValues(centerX, centerY, centerZ),
        collisionGroups,
        collisionMask,
    });
}

describe('DBVT', () => {
    describe('create', () => {
        it('should create an empty DBVT', () => {
            const tree = dbvt.create();
            expect(tree.root).toBe(-1);
            expect(tree.nodes).toEqual([]);
            expect(tree.dirty).toBe(false);
        });
    });

    describe('add', () => {
        it('should add a single body to an empty tree', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body = makeBody(world, 0, 0, 0, 1, 1, 1);

            const leafIndex = dbvt.add(tree, body);

            expect(leafIndex).toBe(0);
            expect(tree.root).toBe(0);
            expect(tree.nodes[leafIndex].bodyIndex).toBe(body.index);
            expect(tree.nodes[leafIndex].left).toBe(-1);
            expect(tree.nodes[leafIndex].right).toBe(-1);
            expect(tree.dirty).toBe(true);
        });

        it('should add multiple bodies and build tree structure', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1);
            const body2 = makeBody(world, 2, 0, 0, 3, 1, 1);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);

            expect(tree.nodes.length).toBe(3); // 2 leaves + 1 internal node
            const root = tree.nodes[tree.root];
            expect(root.left).not.toBe(-1);
            expect(root.right).not.toBe(-1);
        });

        it('should expand body AABB by margin', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body = makeBody(world, 0, 0, 0, 1, 1, 1);

            const leafIndex = dbvt.add(tree, body);
            const leaf = tree.nodes[leafIndex];

            // leaf AABB should be larger than body AABB due to margin
            expect(leaf.aabb[0]).toBeLessThan(body.aabb[0]);
            expect(leaf.aabb[3]).toBeGreaterThan(body.aabb[3]);
        });
    });

    describe('remove', () => {
        it('should remove the only body from tree', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body = makeBody(world, 0, 0, 0, 1, 1, 1);

            dbvt.add(tree, body);
            dbvt.remove(tree, body);

            expect(tree.root).toBe(-1);
            expect(body.dbvtNode).toBe(-1);
        });

        it('should remove a body and restructure tree', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1);
            const body2 = makeBody(world, 2, 0, 0, 3, 1, 1);
            const body3 = makeBody(world, 4, 0, 0, 5, 1, 1);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);
            dbvt.add(tree, body3);

            dbvt.remove(tree, body2);

            // After removing one body, the tree should still be valid
            expect(tree.root).not.toBe(-1);
            expect(body2.dbvtNode).toBe(-1);
            // Should have reused nodes through free list
            expect(tree.freeNodeIndices.length).toBeGreaterThan(0);
        });
    });

    describe('update', () => {
        it('should not update if body still fits in fat AABB', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body = makeBody(world, 0, 0, 0, 1, 1, 1);

            const leafIndex = dbvt.add(tree, body);
            const originalAABB = box3.clone(tree.nodes[leafIndex].aabb);

            // Move body slightly (still within fat AABB)
            body.position[0] += 0.05;
            dbvt.update(tree, body);

            // AABB should not have changed
            expect(box3.equals(tree.nodes[leafIndex].aabb, originalAABB)).toBe(true);
        });

        it('should update when body moves outside fat AABB', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body = makeBody(world, 0, 0, 0, 1, 1, 1);

            dbvt.add(tree, body);

            // Move body significantly
            body.position[0] += 5;
            box3.set(body.aabb, 4.5, -0.5, -0.5, 5.5, 0.5, 0.5);
            dbvt.update(tree, body);

            const leaf = tree.nodes[body.dbvtNode];
            expect(leaf.bodyIndex).toBe(body.index);
            // New AABB should contain the new position
            expect(box3.containsBox3(leaf.aabb, body.aabb)).toBe(true);
        });
    });

    describe('intersectAABB', () => {
        it('should find bodies intersecting query AABB', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1);
            const body2 = makeBody(world, 5, 0, 0, 6, 1, 1);
            const body3 = makeBody(world, 0.5, 0, 0, 1.5, 1, 1);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);
            dbvt.add(tree, body3);

            const queryAABB = box3.set(box3.create(), -0.5, -0.5, -0.5, 1.5, 1.5, 1.5);
            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };

            const queryFilter = filter.create(world.settings.layers);
            dbvt.intersectAABB(world, tree, queryAABB, queryFilter, visitor);

            expect(found).toContain(body1);
            expect(found).toContain(body3);
            expect(found).not.toContain(body2);
        });

        it('should respect collision filtering', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1, 0b01, 0xffffffff);
            const body2 = makeBody(world, 0, 0, 0, 1, 1, 1, 0b10, 0xffffffff);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);

            const queryAABB = box3.set(box3.create(), -1, -1, -1, 2, 2, 2);
            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };

            // Query with group 0b01, should only find body1
            const queryFilter = filter.create(world.settings.layers);
            queryFilter.collisionGroups = 0b01;
            queryFilter.collisionMask = 0b01;
            dbvt.intersectAABB(world, tree, queryAABB, queryFilter, visitor);

            expect(found).toContain(body1);
            expect(found).not.toContain(body2);
        });

        it('should support early exit via visitor.shouldExit', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1);
            const body2 = makeBody(world, 0.5, 0, 0, 1.5, 1, 1);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);

            const queryAABB = box3.set(box3.create(), -1, -1, -1, 2, 2, 2);
            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    found.push(body);
                    visitor.shouldExit = true; // Exit after first hit
                },
            };

            const queryFilter = filter.create(world.settings.layers);
            dbvt.intersectAABB(world, tree, queryAABB, queryFilter, visitor);

            expect(found.length).toBe(1);
        });
    });

    describe('walk', () => {
        it('should visit all bodies in tree', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies = [
                makeBody(world, 0, 0, 0, 1, 1, 1),
                makeBody(world, 2, 0, 0, 3, 1, 1),
                makeBody(world, 4, 0, 0, 5, 1, 1),
            ];

            for (const body of bodies) {
                dbvt.add(tree, body);
            }

            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };

            dbvt.walk(tree, visitor, world);

            expect(found.length).toBe(3);
            expect(found).toContain(bodies[0]);
            expect(found).toContain(bodies[1]);
            expect(found).toContain(bodies[2]);
        });
    });

    describe('castRay', () => {
        it('should find bodies intersecting ray', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1);
            const body2 = makeBody(world, 0, 5, 0, 1, 6, 1);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);

            const origin = vec3.fromValues(0.5, -2, 0.5);
            const direction = vec3.fromValues(0, 1, 0);
            const length = 10;

            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };

            const queryFilter = filter.create(world.settings.layers);
            dbvt.castRay(world, tree, origin, direction, length, queryFilter, visitor);

            expect(found).toContain(body1);
            expect(found).toContain(body2);
        });

        it('should not find bodies not intersecting ray', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1);
            const body2 = makeBody(world, 5, 0, 0, 6, 1, 1);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);

            const origin = vec3.fromValues(0.5, -2, 0.5);
            const direction = vec3.fromValues(0, 1, 0);
            const length = 10;

            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };

            const queryFilter = filter.create(world.settings.layers);
            dbvt.castRay(world, tree, origin, direction, length, queryFilter, visitor);

            expect(found).toContain(body1);
            expect(found).not.toContain(body2); // body2 is not on the ray path
        });
    });

    describe('castAABB', () => {
        it('should find bodies intersecting swept AABB', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1);
            const body2 = makeBody(world, 3, 0, 0, 4, 1, 1);

            dbvt.add(tree, body1);
            dbvt.add(tree, body2);

            const bounds = box3.set(box3.create(), 0, 0, 0, 0.5, 0.5, 0.5);
            const displacement = vec3.fromValues(5, 0, 0);

            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };

            const queryFilter = filter.create(world.settings.layers);
            dbvt.castAABB(world, tree, bounds, displacement, queryFilter, visitor);

            expect(found).toContain(body1);
            expect(found).toContain(body2);
        });
    });

    describe('rebuild', () => {
        it('should rebuild into a valid tree with all bodies findable and clear dirty', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies = [
                makeBody(world, 0, 0, 0, 1, 1, 1),
                makeBody(world, 2, 0, 0, 3, 1, 1),
                makeBody(world, 4, 0, 0, 5, 1, 1),
                makeBody(world, 6, 0, 0, 7, 1, 1),
            ];
            for (const body of bodies) dbvt.add(tree, body);
            expect(tree.dirty).toBe(true);

            dbvt.rebuild(tree);

            expect(tree.dirty).toBe(false);
            expect(tree.root).not.toBe(-1);
            expectInvariants(tree, bodies);
            expect(walkBodies(tree, world).length).toBe(4);
        });

        it('should preserve leaf identity (body.dbvtNode) and fat leaf boxes across rebuild', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies: RigidBody[] = [];
            for (let i = 0; i < 32; i++) bodies.push(makeBody(world, i * 3, 0, 0, i * 3 + 1, 1, 1));
            for (const b of bodies) b.dbvtNode = dbvt.add(tree, b);

            const nodeBefore = bodies.map((b) => b.dbvtNode);
            const aabbBefore = bodies.map((b) => box3.clone(tree.nodes[b.dbvtNode].aabb));

            dbvt.rebuild(tree);

            for (let i = 0; i < bodies.length; i++) {
                expect(bodies[i].dbvtNode).toBe(nodeBefore[i]); // leaf index unchanged
                expect(box3.equals(tree.nodes[bodies[i].dbvtNode].aabb, aabbBefore[i])).toBe(true);
            }
            expectInvariants(tree, bodies);
        });

        it('should not grow the node pool across repeated rebuild cycles (no internal leak)', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies: RigidBody[] = [];
            for (let i = 0; i < 64; i++) bodies.push(makeBody(world, i, 0, 0, i + 1, 1, 1));
            for (const b of bodies) b.dbvtNode = dbvt.add(tree, b);
            dbvt.rebuild(tree); // warm up: one rebuild settles the pool

            const nodesLen = tree.nodes.length;
            const freeLen = tree.freeNodeIndices.length;

            for (let cycle = 0; cycle < 5; cycle++) {
                // move a subset far enough to escape their fat leaves, then rebuild
                for (let i = 0; i < bodies.length; i += 4) {
                    const b = bodies[i];
                    box3.set(b.aabb, b.aabb[0] + 100, b.aabb[1], b.aabb[2], b.aabb[3] + 100, b.aabb[4], b.aabb[5]);
                    dbvt.update(tree, b);
                }
                dbvt.rebuild(tree);
                expect(tree.nodes.length).toBe(nodesLen);
                expect(tree.freeNodeIndices.length).toBe(freeLen);
            }
        });

        it('should balance a worst-case insertion order (grid field)', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies: RigidBody[] = [];
            const side = 32; // 1024 leaves
            for (let z = 0; z < side; z++) {
                for (let x = 0; x < side; x++) {
                    const b = makeBody(world, x * 2, 0, z * 2, x * 2 + 1, 1, z * 2 + 1);
                    bodies.push(b);
                    b.dbvtNode = dbvt.add(tree, b);
                }
            }
            dbvt.rebuild(tree);

            // balanced binary tree over n leaves has depth ~log2(n); allow generous slack
            expect(treeHeight(tree)).toBeLessThanOrEqual(2 * Math.ceil(Math.log2(bodies.length)));
            expectInvariants(tree, bodies);
        });

        it('should graft unchanged subtrees whole (partial rebuild)', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies: RigidBody[] = [];
            for (let i = 0; i < 64; i++) bodies.push(makeBody(world, i, 0, 0, i + 1, 1, 1));
            for (const b of bodies) b.dbvtNode = dbvt.add(tree, b);
            // shallow always-rebuild depth so most of the tree is eligible to graft
            dbvt.rebuild(tree, 1);

            // snapshot a settled (unchanged) internal subtree's structure
            const someLeaf = bodies[10].dbvtNode;
            let subtreeRoot = tree.nodes[someLeaf].parent;
            while (subtreeRoot !== -1 && tree.nodes[subtreeRoot].changed) subtreeRoot = tree.nodes[subtreeRoot].parent;
            expect(subtreeRoot).not.toBe(-1);
            const before = snapshotSubtree(tree, subtreeRoot);

            // touch a body in a different region, rebuild again
            const far = bodies[63];
            box3.set(far.aabb, 500, 0, 0, 501, 1, 1);
            dbvt.update(tree, far);
            dbvt.rebuild(tree, 1);

            // the untouched subtree grafted in unchanged (same node indices + structure)
            expect(snapshotSubtree(tree, subtreeRoot)).toEqual(before);
            expectInvariants(tree, bodies);
        });

        it('should handle rebuild on empty and single-body trees', () => {
            const { world } = createTestWorld();
            const empty = dbvt.create();
            dbvt.rebuild(empty);
            expect(empty.root).toBe(-1);
            expect(empty.dirty).toBe(false);

            const tree = dbvt.create();
            const b = makeBody(world, 0, 0, 0, 1, 1, 1);
            b.dbvtNode = dbvt.add(tree, b);
            dbvt.rebuild(tree);
            expect(tree.root).toBe(b.dbvtNode);
            expect(tree.nodes[tree.root].parent).toBe(-1);
        });
    });

    describe('update (widen-in-place)', () => {
        it('should replace the leaf box, widen ancestors, and mark dirty on escape', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const a = makeBody(world, 0, 0, 0, 1, 1, 1);
            const b = makeBody(world, 10, 0, 0, 11, 1, 1);
            a.dbvtNode = dbvt.add(tree, a);
            b.dbvtNode = dbvt.add(tree, b);
            dbvt.rebuild(tree);
            expect(tree.dirty).toBe(false);

            box3.set(a.aabb, 4.5, -0.5, -0.5, 5.5, 0.5, 0.5);
            const escaped = dbvt.update(tree, a);

            expect(escaped).toBe(true);
            expect(tree.dirty).toBe(true);
            const leaf = tree.nodes[a.dbvtNode];
            // leaf box == margin-expanded body box (replace, not union — may shrink)
            const expected = box3.create();
            box3.expandByMargin(expected, a.aabb, tree.expansionMargin);
            expect(box3.equals(leaf.aabb, expected)).toBe(true);
            // ancestors contain the moved leaf (containment invariant holds pre-rebuild)
            let idx = leaf.parent;
            while (idx !== -1) {
                expect(box3.containsBox3(tree.nodes[idx].aabb, leaf.aabb)).toBe(true);
                expect(tree.nodes[idx].changed).toBe(true);
                idx = tree.nodes[idx].parent;
            }
        });
    });
});

// ---- test helpers ----------------------------------------------------------------------------

function walkBodies(tree: dbvt.DBVT, world: World): RigidBody[] {
    const found: RigidBody[] = [];
    dbvt.walk(tree, { shouldExit: false, visit: (b: RigidBody) => found.push(b) }, world);
    return found;
}

function treeHeight(tree: dbvt.DBVT): number {
    const rec = (idx: number): number => {
        if (idx === -1) return 0;
        const n = tree.nodes[idx];
        if (n.left === -1 && n.right === -1) return 1;
        return 1 + Math.max(rec(n.left), rec(n.right));
    };
    return rec(tree.root);
}

function snapshotSubtree(tree: dbvt.DBVT, idx: number): unknown {
    const n = tree.nodes[idx];
    if (n.left === -1 && n.right === -1) return { idx, body: n.bodyIndex };
    return { idx, left: snapshotSubtree(tree, n.left), right: snapshotSubtree(tree, n.right) };
}

// structural invariants: pointer consistency, aabb containment, changed⇒parent.changed,
// every live body in exactly one reachable leaf.
function expectInvariants(tree: dbvt.DBVT, bodies: RigidBody[]): void {
    const seenBodies = new Set<number>();
    const rec = (idx: number, parent: number): void => {
        const n = tree.nodes[idx];
        expect(n.parent).toBe(parent);
        if (n.left === -1 && n.right === -1) {
            expect(seenBodies.has(n.bodyIndex)).toBe(false);
            seenBodies.add(n.bodyIndex);
            return;
        }
        const l = tree.nodes[n.left];
        const r = tree.nodes[n.right];
        // node aabb contains both children
        expect(box3.containsBox3(n.aabb, l.aabb)).toBe(true);
        expect(box3.containsBox3(n.aabb, r.aabb)).toBe(true);
        // changed ⇒ parent changed
        if (parent !== -1 && n.changed) expect(tree.nodes[parent].changed).toBe(true);
        rec(n.left, idx);
        rec(n.right, idx);
    };
    if (tree.root !== -1) rec(tree.root, -1);
    for (const b of bodies) expect(seenBodies.has(b.index)).toBe(true);
}
