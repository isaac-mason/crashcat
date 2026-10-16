import type { Quat, Vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    type CastShapeSettings,
    CastShapeStatus,
    castRayVsShape,
    castShapeVsShape,
    collidePointVsShape,
    collideShapeVsShape,
    createAllCollidePointCollector,
    createAllCollideShapeCollector,
    createClosestCastRayCollector,
    createClosestCastShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    plane,
    sphere,
} from '../../src';

// a y-up plane through its body origin, so the body pose alone places it
const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
const sphereShape = sphere.create({ radius: 1 });

type PlanePose = { position: Vec3; quaternion: Quat };

// surface at y = -3, normal +y
const loweredPose: PlanePose = { position: [0, -3, 0], quaternion: [0, 0, 0, 1] };

// rotated 90 degrees about z, the normal becomes -x; surface at x = 4, solid towards +x
const rotatedPose: PlanePose = { position: [4, 0, 0], quaternion: [0, 0, Math.SQRT1_2, Math.SQRT1_2] };

function castSphere(settings: CastShapeSettings, start: Vec3, displacement: Vec3, pose: PlanePose) {
    const collector = createClosestCastShapeCollector();
    // biome-ignore format: readability
    castShapeVsShape(
        collector,
        settings,
        sphereShape,
        EMPTY_SUB_SHAPE_ID, 0,
        start[0], start[1], start[2],
        0, 0, 0, 1,
        1, 1, 1,
        displacement[0], displacement[1], displacement[2],
        planeShape,
        EMPTY_SUB_SHAPE_ID, 0,
        pose.position[0], pose.position[1], pose.position[2],
        pose.quaternion[0], pose.quaternion[1], pose.quaternion[2], pose.quaternion[3],
        1, 1, 1,
    );
    return collector;
}

function collideSphere(center: Vec3, pose: PlanePose) {
    const collector = createAllCollideShapeCollector();
    // biome-ignore format: readability
    collideShapeVsShape(
        collector,
        createDefaultCollideShapeSettings(),
        sphereShape,
        EMPTY_SUB_SHAPE_ID, 0,
        center[0], center[1], center[2],
        0, 0, 0, 1,
        1, 1, 1,
        planeShape,
        EMPTY_SUB_SHAPE_ID, 0,
        pose.position[0], pose.position[1], pose.position[2],
        pose.quaternion[0], pose.quaternion[1], pose.quaternion[2], pose.quaternion[3],
        1, 1, 1,
    );
    return collector;
}

function expectVec3(actual: Vec3, expected: Vec3) {
    expect(actual[0]).toBeCloseTo(expected[0], 5);
    expect(actual[1]).toBeCloseTo(expected[1], 5);
    expect(actual[2]).toBeCloseTo(expected[2], 5);
}

describe('plane - cast convex vs plane at a pose', () => {
    test('a sphere dropped onto a plane at y = -3 hits at fraction 0.2', () => {
        // the bottom of the unit sphere travels from y = -1 to y = -3: 2 of 10
        const collector = castSphere(createDefaultCastShapeSettings(), [0, 0, 0], [0, -10, 0], loweredPose);

        const hit = collector.hit;
        expect(hit.status).toBe(CastShapeStatus.COLLIDING);
        expect(hit.fraction).toBeCloseTo(0.2, 5);
        expectVec3(hit.normal, [0, 1, 0]);
        expectVec3(hit.pointA, [0, -3, 0]);
        expectVec3(hit.pointB, [0, -3, 0]);
    });

    test('a sphere cast along +x onto a rotated plane at x = 4 hits at fraction 0.3', () => {
        // the leading point of the unit sphere travels from x = 1 to x = 4: 3 of 10
        const collector = castSphere(createDefaultCastShapeSettings(), [0, 0, 0], [10, 0, 0], rotatedPose);

        const hit = collector.hit;
        expect(hit.status).toBe(CastShapeStatus.COLLIDING);
        expect(hit.fraction).toBeCloseTo(0.3, 5);
        expectVec3(hit.normal, [-1, 0, 0]);
        expectVec3(hit.pointA, [4, 0, 0]);
        expectVec3(hit.pointB, [4, 0, 0]);
        expect(hit.penetrationDepth).toBeCloseTo(0, 5);
    });

    test('a sphere starting 0.5 into a rotated plane reports the depth at fraction 0', () => {
        const collector = castSphere(createDefaultCastShapeSettings(), [3.5, 0, 0], [1, 0, 0], rotatedPose);

        const hit = collector.hit;
        expect(hit.status).toBe(CastShapeStatus.COLLIDING);
        expect(hit.fraction).toBe(0);
        expect(hit.penetrationDepth).toBeCloseTo(0.5, 5);
        expectVec3(hit.normal, [-1, 0, 0]);
        expectVec3(hit.pointA, [4.5, 0, 0]);
        expectVec3(hit.pointB, [4, 0, 0]);
    });

    test('a box cast onto a rotated plane collects both faces on the plane at the hit', () => {
        // the box's +x face travels from x = 1 to x = 4: 3 of 10
        const boxShape = box.create({ halfExtents: [1, 1, 1] });
        const settings = createDefaultCastShapeSettings();
        settings.collectFaces = true;
        const collector = createClosestCastShapeCollector();
        const { position: p, quaternion: q } = rotatedPose;
        // biome-ignore format: readability
        castShapeVsShape(
            collector,
            settings,
            boxShape,
            EMPTY_SUB_SHAPE_ID, 0,
            0, 2, 0,
            0, 0, 0, 1,
            1, 1, 1,
            10, 0, 0,
            planeShape,
            EMPTY_SUB_SHAPE_ID, 0,
            p[0], p[1], p[2],
            q[0], q[1], q[2], q[3],
            1, 1, 1,
        );

        const hit = collector.hit;
        expect(hit.fraction).toBeCloseTo(0.3, 5);
        expect(hit.faceA.numVertices).toBe(4);
        expect(hit.faceB.numVertices).toBe(4);
        for (let i = 0; i < 4; i++) {
            expect(hit.faceA.vertices[i * 3]).toBeCloseTo(4, 5);
            expect(hit.faceB.vertices[i * 3]).toBeCloseTo(4, 5);
        }
    });
});

describe('plane - collide convex vs plane at a pose', () => {
    test('a sphere 0.5 into a plane at y = -0.5', () => {
        const collector = collideSphere([0, 0, 0], { position: [0, -0.5, 0], quaternion: [0, 0, 0, 1] });

        expect(collector.hits.length).toBe(1);
        const hit = collector.hits[0];
        expect(hit.penetration).toBeCloseTo(0.5, 5);
        expectVec3(hit.penetrationAxis, [0, -1, 0]);
        expectVec3(hit.pointA, [0, -1, 0]);
        expectVec3(hit.pointB, [0, -0.5, 0]);
    });

    test('a sphere 0.5 into a rotated plane at x = 4', () => {
        const collector = collideSphere([3.5, 0, 0], rotatedPose);

        expect(collector.hits.length).toBe(1);
        const hit = collector.hits[0];
        expect(hit.penetration).toBeCloseTo(0.5, 5);
        expectVec3(hit.penetrationAxis, [1, 0, 0]);
        expectVec3(hit.pointA, [4.5, 0, 0]);
        expectVec3(hit.pointB, [4, 0, 0]);
    });

    test('a sphere clear of a rotated plane at x = 4 does not collide', () => {
        const collector = collideSphere([2.5, 0, 0], rotatedPose);
        expect(collector.hits.length).toBe(0);
    });
});

describe('plane - ray and point at a pose', () => {
    test('a ray along +x hits a rotated plane at x = 4 at fraction 0.4', () => {
        const collector = createClosestCastRayCollector();
        const settings = createDefaultCastRaySettings();
        const { position: p, quaternion: q } = rotatedPose;
        // biome-ignore format: readability
        castRayVsShape(
            collector,
            settings,
            0, 0, 0,
            1, 0, 0,
            10,
            planeShape,
            EMPTY_SUB_SHAPE_ID, 0,
            p[0], p[1], p[2],
            q[0], q[1], q[2], q[3],
            1, 1, 1,
        );

        expect(collector.hit.fraction).toBeCloseTo(0.4, 5);
    });

    test('points either side of a rotated plane at x = 4', () => {
        const settings = createDefaultCollidePointSettings();
        const { position: p, quaternion: q } = rotatedPose;
        const collidePoint = (x: number) => {
            const collector = createAllCollidePointCollector();
            // biome-ignore format: readability
            collidePointVsShape(
                collector,
                settings,
                x, 0, 0,
                planeShape,
                EMPTY_SUB_SHAPE_ID, 0,
                p[0], p[1], p[2],
                q[0], q[1], q[2], q[3],
                1, 1, 1,
            );
            return collector.hits.length;
        };

        expect(collidePoint(4.5)).toBe(1);
        expect(collidePoint(3.5)).toBe(0);
    });
});
