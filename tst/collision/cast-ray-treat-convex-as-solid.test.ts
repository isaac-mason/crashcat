import { vec3 } from 'math';
import { describe, expect, it } from 'vitest';
import {
    CastRayStatus,
    createAllCastRayCollector,
    createAnyCastRayCollector,
    createDefaultCastRaySettings,
} from '../../src/collision/cast-ray-vs-shape';
import { castRayVsShape } from '../../src/collision/narrowphase';
import * as box from '../../src/shapes/box';
import * as capsule from '../../src/shapes/capsule';
import * as plane from '../../src/shapes/plane';
import * as sphere from '../../src/shapes/sphere';

// base settings with common defaults
const baseSettings = createDefaultCastRaySettings();

// pre-configured settings for different test scenarios
const settingsWithConvexSolid = { ...baseSettings, treatConvexAsSolid: true };
const settingsWithoutConvexSolid = { ...baseSettings, treatConvexAsSolid: false };

describe('treatConvexAsSolid ray casting', () => {
    describe('sphere', () => {
        it('treatConvexAsSolid=true: ray starting inside returns hit at fraction 0', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            // ray starting inside sphere
            const originX = 0,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                sphereShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });

        it('treatConvexAsSolid=false: ray starting inside returns no hit', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            // ray starting inside sphere
            const originX = 0,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithoutConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                sphereShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });

        it('treatConvexAsSolid=true: ray starting outside returns hit at entry point', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            // ray starting outside sphere
            const originX = 3,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(-1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                sphereShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2); // (3 - 1) / 10 = 0.2
        });

        it('treatConvexAsSolid=false: ray starting outside returns hit at entry point', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            // ray starting outside sphere
            const originX = 3,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(-1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithoutConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                sphereShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2);
        });
    });

    describe('box', () => {
        it('treatConvexAsSolid=true: ray starting inside returns hit at fraction 0', () => {
            const boxShape = box.create({ halfExtents: [1, 1, 1] });
            // ray starting inside box
            const originX = 0,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                boxShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });

        it('treatConvexAsSolid=false: ray starting inside returns no hit', () => {
            const boxShape = box.create({ halfExtents: [1, 1, 1] });
            // ray starting inside box
            const originX = 0,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithoutConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                boxShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });
    });

    describe('capsule', () => {
        it('treatConvexAsSolid=true: ray starting inside returns hit at fraction 0', () => {
            const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 0.5 });
            // ray starting inside capsule
            const originX = 0,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                capsuleShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });

        it('treatConvexAsSolid=false: ray starting inside returns no hit', () => {
            const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 0.5 });
            // ray starting inside capsule
            const originX = 0,
                originY = 0,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithoutConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                capsuleShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });
    });

    describe('plane', () => {
        it('treatConvexAsSolid=true: ray starting in negative half-space returns hit at fraction 0', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            // ray below plane (negative half-space), moving parallel to plane
            const originX = 0,
                originY = -1,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                planeShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });

        it('treatConvexAsSolid=false: ray starting in negative half-space returns no hit at fraction 0', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            // ray below plane (negative half-space), moving parallel to plane
            const originX = 0,
                originY = -1,
                originZ = 0;
            const dir = vec3.fromValues(1, 0, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithoutConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                planeShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            // should not hit at fraction 0, but may hit the plane if ray crosses it
            // in this case, ray is parallel so no hit at all
            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });

        it('treatConvexAsSolid=true: ray starting in positive half-space hits plane normally', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            // ray above plane (positive half-space), moving down toward plane
            const originX = 0,
                originY = 2,
                originZ = 0;
            const dir = vec3.fromValues(0, -1, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                planeShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2); // 2/10 = 0.2
        });

        it('treatConvexAsSolid=false: ray starting in positive half-space hits plane normally', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            // ray above plane (positive half-space), moving down toward plane
            const originX = 0,
                originY = 2,
                originZ = 0;
            const dir = vec3.fromValues(0, -1, 0);
            vec3.normalize(dir, dir);
            const length = 10;

            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;

            castRayVsShape(
                collector,
                settingsWithoutConvexSolid,
                originX,
                originY,
                originZ,
                dir[0],
                dir[1],
                dir[2],
                length,
                planeShape,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
                1,
                1,
                1,
            );

            // should hit plane normally - treatConvexAsSolid only affects inside rays
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2);
        });
    });
});

describe('collideWithBackfaces convex ray casting', () => {
    // ray from inside a unit sphere along +x: it exits at x = 1, so at fraction (1 - 0.25) / 10
    const castFromInsideSphere = (settings: typeof baseSettings) => {
        const sphereShape = sphere.create({ radius: 1.0 });
        const collector = createAllCastRayCollector();
        collector.bodyIdB = 1;
        // biome-ignore format: readability
        castRayVsShape(
            collector,
            settings,
            0.25, 0, 0,
            1, 0, 0,
            10,
            sphereShape,
            0, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );
        return collector.hits;
    };

    it('a hollow convex with back faces hits the far surface on the way out', () => {
        const hits = castFromInsideSphere({ ...baseSettings, treatConvexAsSolid: false, collideWithBackfaces: true });
        expect(hits.length).toBe(1);
        expect(hits[0].status).toBe(CastRayStatus.COLLIDING);
        expect(hits[0].fraction).toBeCloseTo(0.075, 4);
    });

    it('a hollow convex without back faces misses', () => {
        const hits = castFromInsideSphere({ ...baseSettings, treatConvexAsSolid: false, collideWithBackfaces: false });
        expect(hits.length).toBe(0);
    });

    it('a solid convex hits at fraction 0', () => {
        const hits = castFromInsideSphere({ ...baseSettings, treatConvexAsSolid: true, collideWithBackfaces: false });
        expect(hits.length).toBe(1);
        expect(hits[0].fraction).toBe(0);
    });
});
