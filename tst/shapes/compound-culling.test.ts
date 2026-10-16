import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    type CastRayCollector,
    type CastRayHit,
    type CastShapeSettings,
    type CollideShapeSettings,
    castRayVsShape,
    castShapeVsShape,
    collideShapeVsShape,
    compound,
    createAllCastShapeCollector,
    createAllCollideShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    type Shape,
    sphere,
} from '../../src';

// children 10 apart along x: the one at the origin is the only one a query near the origin can touch
function row(child: Shape, count: number) {
    const children = [];
    for (let i = 0; i < count; i++) {
        children.push({ position: vec3.fromValues(i * 10, 0, 0), quaternion: quat.create(), shape: child });
    }
    return compound.create({ children });
}

/** counts every child the walk hands the ray to: a box ray reports either a hit or a miss */
function countingRayCollector(): CastRayCollector & { visits: number } {
    return {
        bodyIdB: -1,
        earlyOutFraction: 1,
        visits: 0,
        addHit(_hit: CastRayHit) {
            this.visits++;
        },
        addMiss() {
            this.visits++;
        },
        shouldEarlyOut: () => false,
    };
}

describe('compound child culling', () => {
    test('a ray visits only the child its path crosses', () => {
        const cube = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0 });
        const collector = countingRayCollector();

        // biome-ignore format: pretty
        castRayVsShape(
            collector, createDefaultCastRaySettings(),
            0, 10, 0,
            0, -1, 0, 20,
            row(cube, 8), EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );

        expect(collector.visits).toBe(1);
    });

    test('a collide visits only the child the other shape overlaps', () => {
        const ball = sphere.create({ radius: 0.5 });

        // sphere vs sphere reads the max separation distance once per child it is handed
        const reads = (shape: Shape) => {
            const base = createDefaultCollideShapeSettings();
            let count = 0;
            const settings: CollideShapeSettings = {
                ...base,
                get maxSeparationDistance() {
                    count++;
                    return base.maxSeparationDistance;
                },
            };
            const collector = createAllCollideShapeCollector();
            // biome-ignore format: pretty
            collideShapeVsShape(
                collector, settings,
                ball, EMPTY_SUB_SHAPE_ID, 0,
                0, 0.8, 0,
                0, 0, 0, 1,
                1, 1, 1,
                shape, EMPTY_SUB_SHAPE_ID, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            expect(collector.hits.length).toBe(1);
            return count;
        };

        expect(reads(row(ball, 8))).toBe(reads(row(ball, 1)));
    });

    test('a cast visits only the child the swept shape can reach', () => {
        const ball = sphere.create({ radius: 0.5 });

        // the convex cast reads the penetration tolerance once per child it is handed
        const reads = (shape: Shape) => {
            const base = createDefaultCastShapeSettings();
            let count = 0;
            const settings: CastShapeSettings = {
                ...base,
                get penetrationTolerance() {
                    count++;
                    return base.penetrationTolerance;
                },
            };
            const collector = createAllCastShapeCollector();
            // biome-ignore format: pretty
            castShapeVsShape(
                collector, settings,
                ball, EMPTY_SUB_SHAPE_ID, 0,
                0, 5, 0,
                0, 0, 0, 1,
                1, 1, 1,
                0, -10, 0,
                shape, EMPTY_SUB_SHAPE_ID, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            expect(collector.hits.length).toBe(1);
            return count;
        };

        expect(reads(row(ball, 8))).toBe(reads(row(ball, 1)));
    });

    test('the compound as a collides only through the child overlapping the other shape', () => {
        const ball = sphere.create({ radius: 0.5 });

        const reads = (shape: Shape) => {
            const base = createDefaultCollideShapeSettings();
            let count = 0;
            const settings: CollideShapeSettings = {
                ...base,
                get maxSeparationDistance() {
                    count++;
                    return base.maxSeparationDistance;
                },
            };
            const collector = createAllCollideShapeCollector();
            // biome-ignore format: pretty
            collideShapeVsShape(
                collector, settings,
                shape, EMPTY_SUB_SHAPE_ID, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
                ball, EMPTY_SUB_SHAPE_ID, 0,
                0, 0.8, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            expect(collector.hits.length).toBe(1);
            return count;
        };

        expect(reads(row(ball, 8))).toBe(reads(row(ball, 1)));
    });

    test('culling follows the compound pose and scale: the far child is still found', () => {
        const ball = sphere.create({ radius: 0.5 });
        const cube = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0 });
        // a quarter turn about y takes local +x to world -z; scaled by 2 the child at local x = 30 sits at world z = -60
        const turn = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), Math.PI / 2);
        const [qx, qy, qz, qw] = turn;
        const target = { x: 5, y: 0, z: 5 - 60 };

        const rayCollector = countingRayCollector();
        // biome-ignore format: pretty
        castRayVsShape(
            rayCollector, createDefaultCastRaySettings(),
            target.x, 10, target.z,
            0, -1, 0, 20,
            row(cube, 8), EMPTY_SUB_SHAPE_ID, 0,
            5, 0, 5,
            qx, qy, qz, qw,
            2, 2, 2,
        );
        expect(rayCollector.visits).toBe(1);

        const collideCollector = createAllCollideShapeCollector();
        // biome-ignore format: pretty
        collideShapeVsShape(
            collideCollector, createDefaultCollideShapeSettings(),
            ball, EMPTY_SUB_SHAPE_ID, 0,
            target.x, 1.4, target.z,
            0, 0, 0, 1,
            1, 1, 1,
            row(ball, 8), EMPTY_SUB_SHAPE_ID, 0,
            5, 0, 5,
            qx, qy, qz, qw,
            2, 2, 2,
        );
        expect(collideCollector.hits.length).toBe(1);

        const castCollector = createAllCastShapeCollector();
        // biome-ignore format: pretty
        castShapeVsShape(
            castCollector, createDefaultCastShapeSettings(),
            ball, EMPTY_SUB_SHAPE_ID, 0,
            target.x, 5, target.z,
            0, 0, 0, 1,
            1, 1, 1,
            0, -10, 0,
            row(ball, 8), EMPTY_SUB_SHAPE_ID, 0,
            5, 0, 5,
            qx, qy, qz, qw,
            2, 2, 2,
        );
        expect(castCollector.hits.length).toBe(1);
        expect(castCollector.hits[0].fraction).toBeCloseTo((5 - 0.5 - 1) / 10, 3);
    });
});
