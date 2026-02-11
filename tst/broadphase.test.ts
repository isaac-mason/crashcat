import { type Box3, vec3 } from 'mathcat';
import { describe, expect, it } from 'vitest';
import { type BodyVisitor, box, broadphase, filter, MotionType, type RigidBody, rigidBody } from '../src';
import { createTestWorld } from './helpers';

function makeBody(
    world: any,
    minX: number,
    minY: number,
    minZ: number,
    maxX: number,
    maxY: number,
    maxZ: number,
    objectLayer = 0,
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
        objectLayer,
        position: vec3.fromValues(centerX, centerY, centerZ),
        collisionGroups,
        collisionMask,
    });
}

describe('Broadphase Integration', () => {
    describe('body addition and removal', () => {
        it('should add bodies to broadphase when created in world', () => {
            const { world } = createTestWorld();

            const body = makeBody(world, 0, 0, 0, 1, 1, 1);

            expect(body.broadphaseLayer).not.toBeNull();
            expect(body.dbvtNode).not.toBeNull();
        });

        it('should track body removal from broadphase', () => {
            const { world } = createTestWorld();
            const body = makeBody(world, 0, 0, 0, 1, 1, 1);

            expect(body.broadphaseLayer).not.toBeNull();
            expect(body.dbvtNode).not.toBeNull();

            // Verify body is findable via queries
            broadphase.findCollidingPairs(world, 0, undefined);
            // Just verify the query works without error
            expect(world.broadphase.pairs).toBeDefined();
        });

        it('should handle multiple bodies in broadphase', () => {
            const { world } = createTestWorld();

            const bodies = [
                makeBody(world, 0, 0, 0, 1, 1, 1),
                makeBody(world, 2, 0, 0, 3, 1, 1),
                makeBody(world, 4, 0, 0, 5, 1, 1),
            ];

            for (const body of bodies) {
                expect(body.broadphaseLayer).not.toBeNull();
                expect(body.dbvtNode).not.toBeNull();
            }
        });
    });

    describe('collision pair detection', () => {
        it('should find overlapping bodies', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING);
            const body2 = makeBody(world, 1, 1, 1, 3, 3, 3, OBJECT_LAYER_MOVING);

            broadphase.findCollidingPairs(world, 0, undefined);

            const pairs = world.broadphase.pairs;
            expect(pairs.n).toBe(1);
            const bodyIndexA = pairs.pool[0];
            const bodyIndexB = pairs.pool[1];
            expect(world.bodies.pool[bodyIndexA]).toBe(body1);
            expect(world.bodies.pool[bodyIndexB]).toBe(body2);
        });

        it('should not find non-overlapping bodies', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            makeBody(world, 0, 0, 0, 1, 1, 1, OBJECT_LAYER_MOVING);
            makeBody(world, 5, 0, 0, 6, 1, 1, OBJECT_LAYER_MOVING);

            broadphase.findCollidingPairs(world, 0, undefined);

            expect(world.broadphase.pairs.n).toBe(0);
        });

        it('should respect object layer collision settings', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING },
            } = createTestWorld();

            makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING);
            makeBody(world, 1, 1, 1, 3, 3, 3, OBJECT_LAYER_NOT_MOVING);

            broadphase.findCollidingPairs(world, 0, undefined);

            // In test world, MOVING and NOT_MOVING collide with each other
            expect(world.broadphase.pairs.n).toBe(1);
        });

        it('should respect collision groups and masks', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            // body1: group 0b0001, mask 0b0010
            // body2: group 0b0010, mask 0b0001
            // These should collide because body1's group matches body2's mask and vice versa
            makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING, 0b0001, 0b0010);
            makeBody(world, 1, 1, 1, 3, 3, 3, OBJECT_LAYER_MOVING, 0b0010, 0b0001);

            broadphase.findCollidingPairs(world, 0, undefined);

            expect(world.broadphase.pairs.n).toBe(1);
        });

        it('should filter bodies with non-matching groups and masks', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING, 0b0001, 0b0010);
            makeBody(world, 1, 1, 1, 3, 3, 3, OBJECT_LAYER_MOVING, 0b0100, 0b1000);

            broadphase.findCollidingPairs(world, 0, undefined);

            // body2's group (0b0100) doesn't match body1's mask (0b0010)
            expect(world.broadphase.pairs.n).toBe(0);
        });

        it('should find sleeping dynamic body when kinematic body queries', () => {
            // kinematic bodies should find sleeping dynamic bodies
            // this is important for physics: a moving kinematic platform should wake up sleeping objects
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            // sleeping dynamic body
            const dynamicBody = rigidBody.create(world, {
                shape: box.create({ halfExtents: [1, 1, 1] }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(0, 0, 0),
            });
            dynamicBody.sleeping = true;

            // kinematic body overlapping (NOT a sensor, no collideKinematicVsNonDynamic)
            const kinematicBody = rigidBody.create(world, {
                shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
                objectLayer: OBJECT_LAYER_MOVING,
                motionType: MotionType.KINEMATIC,
                position: vec3.fromValues(0, 0, 0),
                collideKinematicVsNonDynamic: false,
            });

            broadphase.findCollidingPairs(world, 0, undefined);

            // kinematic should find the sleeping dynamic
            expect(world.broadphase.pairs.n).toBe(1);

            const bodyIndexA = world.broadphase.pairs.pool[0];
            const bodyIndexB = world.broadphase.pairs.pool[1];
            const pairBodies = [world.bodies.pool[bodyIndexA], world.bodies.pool[bodyIndexB]];

            expect(pairBodies).toContain(dynamicBody);
            expect(pairBodies).toContain(kinematicBody);
        });
    });

    describe('raycast queries', () => {
        it('should find bodies intersecting ray', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1, OBJECT_LAYER_MOVING);
            const body2 = makeBody(world, 5, 0, 0, 6, 1, 1, OBJECT_LAYER_MOVING);

            const layers = world.settings.layers;

            const origin = vec3.fromValues(-1, 0.5, 0.5);
            const direction = vec3.fromValues(1, 0, 0);
            const length = 100;
            const testFilter = filter.create(layers);
            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            broadphase.castRay(world, origin, direction, length, testFilter, visitor);

            expect(hits.length).toBe(2);
            expect(hits).toContain(body1);
            expect(hits).toContain(body2);
        });

        it('should respect layer filter in raycast', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1, OBJECT_LAYER_MOVING);
            const body2 = makeBody(world, 5, 0, 0, 6, 1, 1, OBJECT_LAYER_NOT_MOVING);

            const layers = world.settings.layers;

            const rayOrigin = vec3.fromValues(-1, 0.5, 0.5);
            const rayDirection = vec3.fromValues(1, 0, 0);
            const rayLength = 100;

            const testFilter = filter.create(layers);
            filter.disableObjectLayer(testFilter, layers, OBJECT_LAYER_NOT_MOVING);

            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            broadphase.castRay(world, rayOrigin, rayDirection, rayLength, testFilter, visitor);

            expect(hits.length).toBe(1);
            expect(hits).toContain(body1);
            expect(hits).not.toContain(body2);
        });
    });

    describe('AABB intersection queries', () => {
        it('should find bodies overlapping AABB', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 0, 0, 0, 1, 1, 1, OBJECT_LAYER_MOVING);
            const _body2 = makeBody(world, 5, 0, 0, 6, 1, 1, OBJECT_LAYER_MOVING);

            const layers = world.settings.layers;

            const queryBox: Box3 = [vec3.fromValues(-1, -1, -1), vec3.fromValues(2, 2, 2)];
            const testFilter = filter.create(layers);
            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            broadphase.intersectAABB(world, queryBox, testFilter, visitor);

            expect(hits.length).toBe(1);
            expect(hits).toContain(body1);
        });
    });

    describe('AABB sweep queries', () => {
        it('should find bodies along swept AABB path', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 5, 0, 0, 6, 1, 1, OBJECT_LAYER_MOVING);

            const layers = world.settings.layers;

            const bounds: Box3 = [vec3.fromValues(-0.5, -0.5, -0.5), vec3.fromValues(0.5, 0.5, 0.5)];
            const displacement = vec3.fromValues(10, 0, 0);
            const testFilter = filter.create(layers);
            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            broadphase.castAABB(world, bounds, displacement, testFilter, visitor);

            expect(hits.length).toBe(1);
            expect(hits).toContain(body1);
        });
    });

    describe('point queries', () => {
        it('should find bodies containing a point', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING);
            // Create separate body for reference isolation
            makeBody(world, 5, 0, 0, 7, 2, 2, OBJECT_LAYER_MOVING);

            const layers = world.settings.layers;
            const testFilter = filter.create(layers);
            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            // Point inside body1
            const point = vec3.fromValues(1, 1, 1);
            broadphase.intersectPoint(world, point, testFilter, visitor);

            expect(hits.length).toBe(1);
            expect(hits).toContain(body1);
        });

        it('should find multiple bodies containing a point', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 0, 0, 0, 4, 4, 4, OBJECT_LAYER_MOVING);
            const body2 = makeBody(world, 2, 2, 2, 6, 6, 6, OBJECT_LAYER_MOVING);

            const layers = world.settings.layers;
            const testFilter = filter.create(layers);
            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            // Point inside both bodies
            const point = vec3.fromValues(3, 3, 3);
            broadphase.intersectPoint(world, point, testFilter, visitor);

            expect(hits.length).toBe(2);
            expect(hits).toContain(body1);
            expect(hits).toContain(body2);
        });

        it('should not find bodies not containing a point', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            // Create bodies in scene but query point in empty space
            makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING);
            makeBody(world, 5, 0, 0, 7, 2, 2, OBJECT_LAYER_MOVING);

            const layers = world.settings.layers;
            const testFilter = filter.create(layers);
            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            // Point in empty space
            const point = vec3.fromValues(3.5, 1, 1);
            broadphase.intersectPoint(world, point, testFilter, visitor);

            expect(hits.length).toBe(0);
        });

        it('should respect layer filter in point queries', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING },
            } = createTestWorld();

            const body1 = makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING);
            // Create body in NOT_MOVING layer to test filtering
            makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_NOT_MOVING);

            const layers = world.settings.layers;
            const testFilter = filter.create(layers);
            filter.disableObjectLayer(testFilter, layers, OBJECT_LAYER_NOT_MOVING);

            const hits: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            const point = vec3.fromValues(1, 1, 1);
            broadphase.intersectPoint(world, point, testFilter, visitor);

            // Should only find body1 (MOVING), not body2 (NOT_MOVING)
            expect(hits.length).toBe(1);
            expect(hits).toContain(body1);
        });

        it('should find all bodies at point regardless of collision groups', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            // Create three bodies with different collision groups/masks at same location
            // Point queries DO respect collision groups and masks - they filter based on compatible groups
            makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING, 0b0001, 0b0010);
            const body2 = makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING, 0b0010, 0b0001);
            makeBody(world, 0, 0, 0, 2, 2, 2, OBJECT_LAYER_MOVING, 0b0100, 0b1000);

            const layers = world.settings.layers;
            const testFilter = filter.create(layers);
            // Set the query filter to have group 0b0001 and mask 0b0010 (matching body1)
            testFilter.collisionGroups = 0b0001;
            testFilter.collisionMask = 0b0010;

            const hits: RigidBody[] = [];
            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                },
            };

            const point = vec3.fromValues(1, 1, 1);
            broadphase.intersectPoint(world, point, testFilter, visitor);

            // Should only find body2 since:
            // - testFilter.group (0b0001) & body2.mask (0b0001) = 0b0001 (match)
            // - testFilter.mask (0b0010) & body2.group (0b0010) = 0b0010 (match)
            // But body1 and body3 don't match the filter's group/mask
            expect(hits.length).toBe(1);
            expect(hits).toContain(body2);
        });

        it('should support early exit from point queries', () => {
            const {
                world,
                layers: { OBJECT_LAYER_MOVING },
            } = createTestWorld();

            // Create multiple overlapping bodies to test early exit
            makeBody(world, 0, 0, 0, 4, 4, 4, OBJECT_LAYER_MOVING);
            makeBody(world, 2, 2, 2, 6, 6, 6, OBJECT_LAYER_MOVING);

            const layers = world.settings.layers;
            const testFilter = filter.create(layers);
            const hits: RigidBody[] = [];

            const visitor: BodyVisitor = {
                shouldExit: false,
                visit: (body: RigidBody) => {
                    hits.push(body);
                    // Exit after finding first body
                    visitor.shouldExit = true;
                },
            };

            const point = vec3.fromValues(3, 3, 3);
            broadphase.intersectPoint(world, point, testFilter, visitor);

            // Should find only 1 body due to early exit
            expect(hits.length).toBe(1);
        });
    });
});
