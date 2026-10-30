import { type Mat4, mat4, type Quat, quat, type Vec3, vec3 } from 'math';
import type { Box3 } from 'math/shapes';
import { describe, expect, test } from 'vitest';
import {
    type BodyId,
    box,
    broadphaseQuery,
    CastRayStatus,
    CastShapeStatus,
    castRay,
    castShape,
    collidePoint,
    collideShape,
    createAllCollidePointCollector,
    createAllCollideShapeCollector,
    createClosestCastRayCollector,
    createClosestCastShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    filter,
    MotionType,
    rigidBody,
    type Shape,
    sphere,
} from '../src';
import { createTestWorld } from './helpers';

// a grid of unturned boxes with no convex radius, so each body's bounds are its shape: the broadphase queries are checked
// against the shape queries on the same boxes, a separate path to the same answer

const GRID = 6;

function createGridWorld() {
    const { world, layers } = createTestWorld();
    for (let x = 0; x < GRID; x++) {
        for (let y = 0; y < 3; y++) {
            for (let z = 0; z < GRID; z++) {
                const halfExtents: Vec3 = [0.3 + 0.1 * ((x + y) % 4), 0.4 + 0.1 * (z % 3), 0.3 + 0.1 * ((x * z) % 3)];
                rigidBody.create(world, {
                    shape: box.create({ halfExtents, convexRadius: 0 }),
                    objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
                    motionType: MotionType.STATIC,
                    position: [2.5 * x, 2.5 * y, 2.5 * z],
                });
            }
        }
    }
    return { world, layers, queryFilter: filter.create(world.settings.layers) };
}

/** a seeded generator, so a failure repeats */
function makeRandom(seed: number) {
    let state = seed >>> 0;
    return (min: number, max: number) => {
        state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
        return min + ((max - min) * (state >>> 8)) / 16777216;
    };
}

type World = ReturnType<typeof createGridWorld>['world'];

/** the bodies the shape at the pose overlaps, by the shape query */
function overlapped(world: World, shape: Shape, position: Vec3, quaternion: Quat, queryFilter: filter.Filter): BodyId[] {
    const collector = createAllCollideShapeCollector();
    collideShape(world, collector, createDefaultCollideShapeSettings(), shape, position, quaternion, [1, 1, 1], queryFilter);
    return [...new Set(collector.hits.map((hit) => hit.bodyIdB))];
}

/**
 * every body the shape grown by a margin overlaps holds what the broadphase found, and the broadphase found every body
 * the shape shrunk by the margin overlaps: the two only differ where a body barely touches
 */
function expectBetween(found: BodyId[], inner: BodyId[], outer: BodyId[]): void {
    for (const id of inner) expect(found).toContain(id);
    for (const id of found) expect(outer).toContain(id);
}

describe('broadphaseQuery', () => {
    test('an oriented box meets the bodies its shape does', () => {
        const { world, queryFilter } = createGridWorld();
        const found = broadphaseQuery.createAllBodyCollector();
        const random = makeRandom(7);
        let hits = 0;
        for (let i = 0; i < 200; i++) {
            const position: Vec3 = [random(-1, 14), random(-1, 6), random(-1, 14)];
            const axis = vec3.normalize(vec3.create(), [random(-1, 1), random(-1, 1), random(-1, 1)]);
            const rotation = quat.setAxisAngle(quat.create(), axis, random(0, 3.1));
            const half: Vec3 = [random(0.1, 2), random(0.1, 2), random(0.1, 2)];

            found.reset();
            const orientation: Mat4 = mat4.fromRotationTranslation(mat4.create(), rotation, position);
            broadphaseQuery.collideOrientedBox(world, found, { orientation, halfExtents: half }, queryFilter);

            const shrunk = box.create({ halfExtents: [half[0] - 1e-3, half[1] - 1e-3, half[2] - 1e-3], convexRadius: 0 });
            const grown = box.create({ halfExtents: [half[0] + 1e-3, half[1] + 1e-3, half[2] + 1e-3], convexRadius: 0 });
            expectBetween(
                found.hits,
                overlapped(world, shrunk, position, rotation, queryFilter),
                overlapped(world, grown, position, rotation, queryFilter),
            );
            hits += found.hits.length;
        }
        // the queries met the grid, not only empty space
        expect(hits).toBeGreaterThan(200);
    });

    test('a sphere meets the bodies its shape does', () => {
        const { world, queryFilter } = createGridWorld();
        const found = broadphaseQuery.createAllBodyCollector();
        const random = makeRandom(11);
        let hits = 0;
        for (let i = 0; i < 200; i++) {
            const center: Vec3 = [random(-1, 14), random(-1, 6), random(-1, 14)];
            const radius = random(0.1, 2.5);

            found.reset();
            broadphaseQuery.collideSphere(world, found, center, radius, queryFilter);

            const identity = quat.create();
            expectBetween(
                found.hits,
                overlapped(world, sphere.create({ radius: radius - 1e-3 }), center, identity, queryFilter),
                overlapped(world, sphere.create({ radius: radius + 1e-3 }), center, identity, queryFilter),
            );
            hits += found.hits.length;
        }
        expect(hits).toBeGreaterThan(200);
    });

    test('a box and a point meet the bodies their shapes do', () => {
        const { world, queryFilter } = createGridWorld();
        const found = broadphaseQuery.createAllBodyCollector();
        const random = makeRandom(13);
        const identity = quat.create();
        for (let i = 0; i < 200; i++) {
            const center: Vec3 = [random(-1, 14), random(-1, 6), random(-1, 14)];
            const half: Vec3 = [random(0.1, 2), random(0.1, 2), random(0.1, 2)];
            const aabb: Box3 = [
                center[0] - half[0],
                center[1] - half[1],
                center[2] - half[2],
                center[0] + half[0],
                center[1] + half[1],
                center[2] + half[2],
            ];

            found.reset();
            broadphaseQuery.collideAABB(world, found, aabb, queryFilter);
            const shrunk = box.create({ halfExtents: [half[0] - 1e-3, half[1] - 1e-3, half[2] - 1e-3], convexRadius: 0 });
            const grown = box.create({ halfExtents: [half[0] + 1e-3, half[1] + 1e-3, half[2] + 1e-3], convexRadius: 0 });
            expectBetween(
                found.hits,
                overlapped(world, shrunk, center, identity, queryFilter),
                overlapped(world, grown, center, identity, queryFilter),
            );

            // a point inside a body is in that body alone, as the grid's boxes are apart
            found.reset();
            broadphaseQuery.collidePoint(world, found, center, queryFilter);
            const inside = createAllCollidePointCollector();
            collidePoint(world, inside, createDefaultCollidePointSettings(), center, queryFilter);
            expect(found.hits.length).toBe(inside.hits.length);
            if (inside.hits.length === 1) expect(found.hits).toContain(inside.hits[0].bodyIdB);
        }
    });

    test('the casts find the nearest body where the shape casts do', () => {
        const { world, queryFilter } = createGridWorld();
        const nearest = broadphaseQuery.createClosestBodyCastCollector();
        const all = broadphaseQuery.createAllBodyCastCollector();
        const random = makeRandom(17);
        // back faces on: a cast's first touch can give a contact axis against the motion, which jolt drops as a back
        // face, and a start inside a box is a hit at 0 only with them
        const castSettings = createDefaultCastShapeSettings();
        castSettings.collideWithBackfacesConvex = true;
        let hits = 0;
        for (let i = 0; i < 200; i++) {
            const origin: Vec3 = [random(-3, 16), random(-3, 8), random(-3, 16)];
            const direction = vec3.normalize(vec3.create(), [random(-1, 1), random(-1, 1), random(-1, 1)]);
            const length = random(1, 15);

            nearest.reset();
            broadphaseQuery.castRay(world, nearest, origin, direction, length, queryFilter);
            const shapeRay = createClosestCastRayCollector();
            castRay(world, shapeRay, createDefaultCastRaySettings(), origin, direction, length, queryFilter);
            const shapeHit = shapeRay.hit.status === CastRayStatus.COLLIDING;
            const bodyHit = nearest.hit.bodyId !== -1;
            // a ray starting inside a box: the bounds are entered at 0
            expect(shapeHit === bodyHit || (bodyHit && nearest.hit.fraction === 0)).toBe(true);
            if (shapeHit && bodyHit) {
                expect(nearest.hit.fraction).toBeCloseTo(shapeRay.hit.fraction, 4);
                hits++;
            }

            // every hit, sorted, starts with the nearest
            all.reset();
            broadphaseQuery.castRay(world, all, origin, direction, length, queryFilter);
            all.sort();
            expect(all.hits.length === 0).toBe(!bodyHit);
            if (bodyHit) expect(all.hits[0].fraction).toBe(nearest.hit.fraction);

            // a box swept as far: the shape cast of the same box agrees
            const half: Vec3 = [random(0.1, 1), random(0.1, 1), random(0.1, 1)];
            const displacement = vec3.scale(vec3.create(), direction, length);
            const aabb: Box3 = [
                origin[0] - half[0],
                origin[1] - half[1],
                origin[2] - half[2],
                origin[0] + half[0],
                origin[1] + half[1],
                origin[2] + half[2],
            ];
            nearest.reset();
            broadphaseQuery.castAABB(world, nearest, aabb, displacement, queryFilter);
            const shapeCast = createClosestCastShapeCollector();
            castShape(
                world,
                shapeCast,
                castSettings,
                box.create({ halfExtents: half, convexRadius: 0 }),
                origin,
                quat.create(),
                [1, 1, 1],
                displacement,
                queryFilter,
            );
            const castHit = shapeCast.hit.status === CastShapeStatus.COLLIDING;
            expect(castHit).toBe(nearest.hit.bodyId !== -1);
            if (castHit) expect(nearest.hit.fraction).toBeCloseTo(shapeCast.hit.fraction, 3);
        }
        expect(hits).toBeGreaterThan(20);
    });

    test('the collectors stop where they should, and the filter holds', () => {
        const { world, layers, queryFilter } = createGridWorld();
        const everything: Box3 = [-5, -5, -5, 20, 20, 20];

        const all = broadphaseQuery.createAllBodyCollector();
        broadphaseQuery.collideAABB(world, all, everything, queryFilter);
        expect(all.hits.length).toBe(GRID * 3 * GRID);

        // any: one body, then the walk stops
        const any = broadphaseQuery.createAnyBodyCollector();
        broadphaseQuery.collideAABB(world, any, everything, queryFilter);
        expect(any.hit).not.toBe(-1);

        // a collector of its own: stopping after three
        let seen = 0;
        const custom: broadphaseQuery.BodyCollector = {
            addHit: () => {
                seen++;
            },
            shouldEarlyOut: () => seen >= 3,
        };
        broadphaseQuery.collideAABB(world, custom, everything, queryFilter);
        expect(seen).toBe(3);

        // the cast any: one hit, at once
        const anyCast = broadphaseQuery.createAnyBodyCastCollector();
        broadphaseQuery.castRay(world, anyCast, [-3, 0, 0], [1, 0, 0], 30, queryFilter);
        expect(anyCast.hit.bodyId).not.toBe(-1);
        expect(anyCast.shouldEarlyOut()).toBe(true);

        // the filter's layers hold: the grid is all on the static layer
        filter.disableObjectLayer(queryFilter, world.settings.layers, layers.OBJECT_LAYER_NOT_MOVING);
        all.reset();
        broadphaseQuery.collideAABB(world, all, everything, queryFilter);
        broadphaseQuery.collideSphere(world, all, [5, 5, 5], 10, queryFilter);
        broadphaseQuery.collidePoint(world, all, [0, 0, 0], queryFilter);
        broadphaseQuery.collideOrientedBox(
            world,
            all,
            { orientation: mat4.fromRotationTranslation(mat4.create(), quat.create(), [5, 5, 5]), halfExtents: [10, 10, 10] },
            queryFilter,
        );
        expect(all.hits.length).toBe(0);
    });
});
