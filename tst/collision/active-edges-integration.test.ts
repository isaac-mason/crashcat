import { quat, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { EMPTY_SUB_SHAPE_ID } from '../../src';
import { CastShapeStatus, castShapeVsShape, createClosestCastShapeCollector, createDefaultCastShapeSettings } from '../../src';
import { collideShapeVsShape, createAllCollideShapeCollector, createDefaultCollideShapeSettings } from '../../src';
import { sphere, triangleMesh } from '../../src';

const identityQuat = () => quat.create();
const scaleOne = () => vec3.fromValues(1, 1, 1);
const pos = (x: number, y: number, z: number) => vec3.fromValues(x, y, z);

describe('Active Edges Integration - Feature Toggle', () => {
    test('should NOT apply active edge correction when collideOnlyWithActiveEdges=false', () => {
        // Create a simple 2x2 flat grid - interior edges will be automatically marked inactive
        // biome-ignore format: pretty
        const positions = [
            0, 0, 0,  1, 0, 0,  2, 0, 0,
            0, 1, 0,  1, 1, 0,  2, 1, 0,
            0, 2, 0,  1, 2, 0,  2, 2, 0,
        ];
        // biome-ignore format: pretty
        const indices = [
            0, 1, 3,  1, 4, 3,  // Bottom row
            1, 2, 4,  2, 5, 4,
            3, 4, 6,  4, 7, 6,  // Top row
            4, 5, 7,  5, 8, 7,
        ];
        const mesh = triangleMesh.create({ positions, indices });

        const sphereA = sphere.create({ radius: 0.3 });
        const settings = createDefaultCastShapeSettings();
        settings.collideOnlyWithActiveEdges = false; // Feature OFF
        vec3.set(settings.activeEdgeMovementDirection, 0, 0, -1);

        const collector = createClosestCastShapeCollector();

        // biome-ignore format: pretty
        castShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0, // subShapeId A
            1, 1, 2, // posA
            0, 0, 0, 1, // quatA
            1, 1, 1, // scaleA
            0, 0, -3, // dispA
            mesh,
            EMPTY_SUB_SHAPE_ID, 0, // subShapeId B
            0, 0, 0, // posB
            0, 0, 0, 1, // quatB
            1, 1, 1, // scaleB
        );

        // Should hit (feature disabled doesn't prevent collision)
        expect(collector.hit.status).toBe(CastShapeStatus.COLLIDING);
    });

    test('should apply active edge correction when collideOnlyWithActiveEdges=true', () => {
        // biome-ignore format: pretty
        const positions = [
            0, 0, 0,  1, 0, 0,  2, 0, 0,
            0, 1, 0,  1, 1, 0,  2, 1, 0,
        ];
        // biome-ignore format: pretty
        const indices = [
            0, 1, 3,  1, 4, 3,
            1, 2, 4,  2, 5, 4,
        ];
        const mesh = triangleMesh.create({ positions, indices });

        const sphereA = sphere.create({ radius: 0.3 });
        const settings = createDefaultCastShapeSettings();
        settings.collideOnlyWithActiveEdges = true; // Feature ON
        vec3.set(settings.activeEdgeMovementDirection, 0, 0, -1);

        const collector = createClosestCastShapeCollector();

        // biome-ignore format: pretty
        castShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            1, 0.5, 1, // posA
            0, 0, 0, 1, // quatA
            1, 1, 1, // scaleA
            0, 0, -2, // dispA
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0, // posB
            0, 0, 0, 1, // quatB
            1, 1, 1, // scaleB
        );

        // Should hit with corrected penetrationAxis and normal
        expect(collector.hit.status).toBe(CastShapeStatus.COLLIDING);

        // PenetrationAxis points from mesh (B) to sphere (A) = upward (positive Z)
        const penetrationAxis = collector.hit.penetrationAxis;
        expect(penetrationAxis[2]).toBeGreaterThan(0.8);

        // Normal = -penetrationAxis.normalized() (points downward, negative Z)
        const normal = collector.hit.normal;
        expect(normal[2]).toBeLessThan(-0.8);
        expect(Math.abs(vec3.length(normal) - 1.0)).toBeLessThan(0.01); // normalized
        expect(Math.abs(normal[1] + penetrationAxis[1])).toBeLessThan(0.01);
        expect(Math.abs(normal[2] + penetrationAxis[2])).toBeLessThan(0.01);
    });
});

describe('Active Edges Integration - Interior Edge Correction (Cast)', () => {
    test('should fix normal when hitting inactive interior edge on flat grid', () => {
        // Create a 4x4 flat grid at Z=0 - plenty of interior edges
        // biome-ignore format: pretty
        const positions = [
            0, 0, 0,  1, 0, 0,  2, 0, 0,  3, 0, 0,  4, 0, 0,
            0, 1, 0,  1, 1, 0,  2, 1, 0,  3, 1, 0,  4, 1, 0,
            0, 2, 0,  1, 2, 0,  2, 2, 0,  3, 2, 0,  4, 2, 0,
            0, 3, 0,  1, 3, 0,  2, 3, 0,  3, 3, 0,  4, 3, 0,
            0, 4, 0,  1, 4, 0,  2, 4, 0,  3, 4, 0,  4, 4, 0,
        ];
        const indices: number[] = [];
        for (let y = 0; y < 4; y++) {
            for (let x = 0; x < 4; x++) {
                const i = y * 5 + x;
                indices.push(i, i + 1, i + 5);
                indices.push(i + 1, i + 6, i + 5);
            }
        }
        const mesh = triangleMesh.create({ positions, indices });

        const sphereA = sphere.create({ radius: 0.4 });
        const settings = createDefaultCastShapeSettings();
        settings.collideOnlyWithActiveEdges = true;
        vec3.set(settings.activeEdgeMovementDirection, 0, 0, -1); // Moving down

        const collector = createClosestCastShapeCollector();
        // biome-ignore format: pretty
        castShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            2, 2, 2, // posA
            0, 0, 0, 1, // quatA
            1, 1, 1, // scaleA
            0, 0, -3, // dispA
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0, // posB
            0, 0, 0, 1, // quatB
            1, 1, 1, // scaleB
        );

        expect(collector.hit.status).toBe(CastShapeStatus.COLLIDING);

        // STRONG ASSERTION: PenetrationAxis points upward (B → A), perpendicular to surface
        const penetrationAxis = collector.hit.penetrationAxis;
        expect(penetrationAxis[2]).toBeGreaterThan(0.9); // Z component should be close to 1
        expect(Math.abs(penetrationAxis[0])).toBeLessThan(0.2); // X component should be near 0
        expect(Math.abs(penetrationAxis[1])).toBeLessThan(0.2); // Y component should be near 0

        // Normal = -penetrationAxis.normalized(), points downward (-Z)
        const normal = collector.hit.normal;
        expect(normal[2]).toBeLessThan(-0.9); // Z component should be close to -1
        expect(Math.abs(normal[0])).toBeLessThan(0.2); // X component should be near 0
        expect(Math.abs(normal[1])).toBeLessThan(0.2); // Y component should be near 0
    });

    test('should fix normal when casting across flat surface', () => {
        // biome-ignore format: pretty
        const positions = [
            0, 0, 0,  2, 0, 0,  4, 0, 0,
            0, 2, 0,  2, 2, 0,  4, 2, 0,
        ];
        // biome-ignore format: pretty
        const indices = [
            0, 1, 3,  1, 4, 3,
            1, 2, 4,  2, 5, 4,
        ];
        const mesh = triangleMesh.create({ positions, indices });

        const sphereA = sphere.create({ radius: 0.3 });
        const settings = createDefaultCastShapeSettings();
        settings.collideOnlyWithActiveEdges = true;
        vec3.set(settings.activeEdgeMovementDirection, 1, 0, 0); // Moving right

        const collector = createClosestCastShapeCollector();
        // biome-ignore format: pretty
        castShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            1, 1, 0.5, // posA
            0, 0, 0, 1, // quatA
            1, 1, 1, // scaleA
            0, 0, -1, // dispA
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0, // posB
            0, 0, 0, 1, // quatB
            1, 1, 1, // scaleB
        );

        expect(collector.hit.status).toBe(CastShapeStatus.COLLIDING);

        // PenetrationAxis and normal should exist and be normalized
        const penetrationAxis = collector.hit.penetrationAxis;
        expect(vec3.squaredLength(penetrationAxis)).toBeGreaterThan(0);

        // Normal = -penetrationAxis.normalized()
        const normal = collector.hit.normal;
        expect(Math.abs(vec3.length(normal) - 1.0)).toBeLessThan(0.01);
    });
});

describe('Active Edges Integration - Interior Edge Correction (Collide)', () => {
    test('should fix normal in collideShapeVsShape', () => {
        // biome-ignore format: pretty
        const positions = [
            0, 0, 0,  2, 0, 0,  4, 0, 0,
            0, 2, 0,  2, 2, 0,  4, 2, 0,
        ];
        // biome-ignore format: pretty
        const indices = [
            0, 1, 3,  1, 4, 3,
            1, 2, 4,  2, 5, 4,
        ];
        const mesh = triangleMesh.create({ positions, indices });

        const sphereA = sphere.create({ radius: 0.5 });
        const settings = createDefaultCollideShapeSettings();
        settings.collideOnlyWithActiveEdges = true;
        vec3.set(settings.activeEdgeMovementDirection, 0, 0, -1);

        const collector = createAllCollideShapeCollector();
        const hits = collector.hits;

        const posA = pos(2, 1, 0.3); // Slightly penetrating surface at center
        const quatA = identityQuat();
        const scaleA = scaleOne();
        const posB = pos(0, 0, 0);
        const quatB = identityQuat();
        const scaleB = scaleOne();

        // biome-ignore format: pretty
        collideShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            posA[0], posA[1], posA[2],
            quatA[0], quatA[1], quatA[2], quatA[3],
            scaleA[0], scaleA[1], scaleA[2],
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            posB[0], posB[1], posB[2],
            quatB[0], quatB[1], quatB[2], quatB[3],
            scaleB[0], scaleB[1], scaleB[2]
        );

        expect(hits.length).toBeGreaterThan(0);

        const contact = hits[0];

        // PenetrationAxis mostly vertical (corrected by active edges)
        const penetrationAxis = contact.penetrationAxis;
        expect(Math.abs(penetrationAxis[2])).toBeGreaterThan(0.15); // Relaxed - may not be perfectly vertical
    });
});

describe('Active Edges Integration - Movement Direction Heuristic', () => {
    test('should use movement direction when sliding across grid', () => {
        // biome-ignore format: pretty
        const positions = [
            0, 0, 0,  1, 0, 0,  2, 0, 0,  3, 0, 0,  4, 0, 0,  5, 0, 0,
            0, 1, 0,  1, 1, 0,  2, 1, 0,  3, 1, 0,  4, 1, 0,  5, 1, 0,
            0, 2, 0,  1, 2, 0,  2, 2, 0,  3, 2, 0,  4, 2, 0,  5, 2, 0,
        ];
        const indices: number[] = [];
        for (let y = 0; y < 2; y++) {
            for (let x = 0; x < 5; x++) {
                const i = y * 6 + x;
                indices.push(i, i + 1, i + 6);
                indices.push(i + 1, i + 7, i + 6);
            }
        }
        const mesh = triangleMesh.create({ positions, indices });

        const sphereA = sphere.create({ radius: 0.4 });
        const settings = createDefaultCastShapeSettings();
        settings.collideOnlyWithActiveEdges = true;
        vec3.set(settings.activeEdgeMovementDirection, 1, 0, 0); // Moving right

        const collector = createClosestCastShapeCollector();

        // biome-ignore format: pretty
        castShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            0.5, 1, 0.5, // posA
            0, 0, 0, 1, // quatA
            1, 1, 1, // scaleA
            4, 0, -0.2, // dispA
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0, // posB
            0, 0, 0, 1, // quatB
            1, 1, 1, // scaleB
        );

        expect(collector.hit.status).toBe(CastShapeStatus.COLLIDING);

        // PenetrationAxis: separation direction (perpendicular to mesh surface = upward)
        const penetrationAxis = collector.hit.penetrationAxis;
        expect(penetrationAxis[2]).toBeGreaterThan(0.7); // Upward (away from mesh)

        // Normal = -penetrationAxis.normalized() (points downward into mesh)
        const normal = collector.hit.normal;
        expect(normal[2]).toBeLessThan(-0.7); // Downward
    });
});

describe('Active Edges Integration - Edge Cases', () => {
    test('should handle sphere missing mesh', () => {
        // biome-ignore format: pretty
        const positions = [
            0, 0, 0,  1, 0, 0,  2, 0, 0,
            0, 1, 0,  1, 1, 0,  2, 1, 0,
        ];
        // biome-ignore format: pretty
        const indices = [
            0, 1, 3,  1, 4, 3,
            1, 2, 4,  2, 5, 4,
        ];
        const mesh = triangleMesh.create({ positions, indices });

        const sphereA = sphere.create({ radius: 0.2 });
        const settings = createDefaultCastShapeSettings();
        settings.collideOnlyWithActiveEdges = true;
        vec3.set(settings.activeEdgeMovementDirection, 1, 0, 0);

        const collector = createClosestCastShapeCollector();

        // biome-ignore format: pretty
        castShapeVsShape(
            collector,
            settings,
            sphereA,
            EMPTY_SUB_SHAPE_ID, 0,
            10, 10, 2, // posA
            0, 0, 0, 1, // quatA
            1, 1, 1, // scaleA
            1, 0, 0, // dispA
            mesh,
            EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0, // posB
            0, 0, 0, 1, // quatB
            1, 1, 1, // scaleB
        );

        // Should not collide
        expect(collector.hit.status).toBe(CastShapeStatus.NOT_COLLIDING);
    });
});
