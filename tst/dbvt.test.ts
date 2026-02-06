import { box3, vec3 } from 'mathcat';
import { describe, expect, it } from 'vitest';
import { type World, box, filter, type BodyVisitor, dbvt, MotionType, type RigidBody, rigidBody } from '../src';
import { createTestWorld } from './helpers';

function makeBody(
    world: World,
    minX: number,
    minY: number,
    minZ: number,
    maxX: number,
    maxY: number,
    maxZ: number,
    collisionGroup = 0xffffffff,
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
        collisionGroup,
        collisionMask,
    });
}

describe('DBVT', () => {
    describe('create', () => {
        it('should create an empty DBVT', () => {
            const tree = dbvt.create();
            expect(tree.root).toBe(-1);
            expect(tree.nodes).toEqual([]);
            expect(tree.optimizationPath).toBe(0);
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
            expect(tree.nodes[leafIndex].height).toBe(0);
            expect(tree.nodes[leafIndex].left).toBe(-1);
            expect(tree.nodes[leafIndex].right).toBe(-1);
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
            expect(leaf.aabb[0][0]).toBeLessThan(body.aabb[0][0]);
            expect(leaf.aabb[1][0]).toBeGreaterThan(body.aabb[1][0]);
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
            dbvt.update(tree, body, -1);

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
            box3.set(body.aabb, [4.5, -0.5, -0.5], [5.5, 0.5, 0.5]);
            dbvt.update(tree, body, -1);

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

            const queryAABB = box3.set(box3.create(), [-0.5, -0.5, -0.5], [1.5, 1.5, 1.5]);
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

            const queryAABB = box3.set(box3.create(), [-1, -1, -1], [2, 2, 2]);
            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };

            // Query with group 0b01, should only find body1
            const queryFilter = filter.create(world.settings.layers);
            queryFilter.collisionGroup = 0b01;
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

            const queryAABB = box3.set(box3.create(), [-1, -1, -1], [2, 2, 2]);
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

            const bounds = box3.set(box3.create(), [0, 0, 0], [0.5, 0.5, 0.5]);
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

    describe('optimizeBottomUp', () => {
        it('should rebuild tree using bottom-up strategy', () => {
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

            dbvt.optimizeBottomUp(tree);

            // Tree should still be valid
            expect(tree.root).not.toBe(-1);
            // All bodies should still be findable
            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };
            dbvt.walk(tree, visitor, world);
            expect(found.length).toBe(3);
        });
    });

    describe('optimizeTopDown', () => {
        it('should rebuild tree using top-down strategy', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies = [
                makeBody(world, 0, 0, 0, 1, 1, 1),
                makeBody(world, 2, 0, 0, 3, 1, 1),
                makeBody(world, 4, 0, 0, 5, 1, 1),
                makeBody(world, 6, 0, 0, 7, 1, 1),
            ];

            for (const body of bodies) {
                dbvt.add(tree, body);
            }

            dbvt.optimizeTopDown(tree);

            // Tree should still be valid
            expect(tree.root).not.toBe(-1);
            // All bodies should still be findable
            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };
            dbvt.walk(tree, visitor, world);
            expect(found.length).toBe(4);
        });
    });

    describe('optimizeIncremental', () => {
        it('should incrementally optimize tree', () => {
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

            const pathBefore = tree.optimizationPath;
            dbvt.optimizeIncremental(tree, 2);

            // Optimization path should have advanced
            expect(tree.optimizationPath).toBe(pathBefore + 2);
            // Tree should still be valid
            const found: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => found.push(body),
            };
            dbvt.walk(tree, visitor, world);
            expect(found.length).toBe(3);
        });

        it('should handle negative passes (optimize all leaves)', () => {
            const { world } = createTestWorld();
            const tree = dbvt.create();
            const bodies = [makeBody(world, 0, 0, 0, 1, 1, 1), makeBody(world, 2, 0, 0, 3, 1, 1)];

            for (const body of bodies) {
                dbvt.add(tree, body);
            }

            dbvt.optimizeIncremental(tree, -1);

            // Should have optimized all leaves
            expect(tree.optimizationPath).toBeGreaterThan(0);
        });
    });
});
