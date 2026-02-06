import { describe, expect, it } from 'vitest';
import { quat, raycast3, vec3 } from 'mathcat';
import { createAnyCastRayCollector, CastRayStatus } from '../../src/collision/cast-ray-vs-shape';
import * as sphere from '../../src/shapes/sphere';
import * as box from '../../src/shapes/box';
import * as capsule from '../../src/shapes/capsule';
import * as plane from '../../src/shapes/plane';
import { castRayVsShape } from '../../src/collision/narrowphase';

describe('treatConvexAsSolid ray casting', () => {
    describe('sphere', () => {
        it('treatConvexAsSolid=true: ray starting inside returns hit at fraction 0', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 0, 0); // inside sphere
            vec3.set(ray.direction, 1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: true },
                ray,
                sphereShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });
        
        it('treatConvexAsSolid=false: ray starting inside returns no hit', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 0, 0); // inside sphere
            vec3.set(ray.direction, 1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: false },
                ray,
                sphereShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });
        
        it('treatConvexAsSolid=true: ray starting outside returns hit at entry point', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            const ray = raycast3.create();
            vec3.set(ray.origin, 3, 0, 0); // outside sphere
            vec3.set(ray.direction, -1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: true },
                ray,
                sphereShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2); // (3 - 1) / 10 = 0.2
        });
        
        it('treatConvexAsSolid=false: ray starting outside returns hit at entry point', () => {
            const sphereShape = sphere.create({ radius: 1.0 });
            const ray = raycast3.create();
            vec3.set(ray.origin, 3, 0, 0); // outside sphere
            vec3.set(ray.direction, -1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: false },
                ray,
                sphereShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2);
        });
    });
    
    describe('box', () => {
        it('treatConvexAsSolid=true: ray starting inside returns hit at fraction 0', () => {
            const boxShape = box.create({ halfExtents: [1, 1, 1] });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 0, 0); // inside box
            vec3.set(ray.direction, 1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: true },
                ray,
                boxShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });
        
        it('treatConvexAsSolid=false: ray starting inside returns no hit', () => {
            const boxShape = box.create({ halfExtents: [1, 1, 1] });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 0, 0); // inside box
            vec3.set(ray.direction, 1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: false },
                ray,
                boxShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });
    });
    
    describe('capsule', () => {
        it('treatConvexAsSolid=true: ray starting inside returns hit at fraction 0', () => {
            const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 0.5 });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 0, 0); // inside capsule
            vec3.set(ray.direction, 1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: true },
                ray,
                capsuleShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });
        
        it('treatConvexAsSolid=false: ray starting inside returns no hit', () => {
            const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 0.5 });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 0, 0); // inside capsule
            vec3.set(ray.direction, 1, 0, 0);
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: false },
                ray,
                capsuleShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });
    });
    
    describe('plane', () => {
        it('treatConvexAsSolid=true: ray starting in negative half-space returns hit at fraction 0', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, -1, 0); // below plane (negative half-space)
            vec3.set(ray.direction, 1, 0, 0); // moving parallel to plane
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: true },
                ray,
                planeShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBe(0);
        });
        
        it('treatConvexAsSolid=false: ray starting in negative half-space returns no hit at fraction 0', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, -1, 0); // below plane (negative half-space)
            vec3.set(ray.direction, 1, 0, 0); // moving parallel to plane
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: false },
                ray,
                planeShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            // should not hit at fraction 0, but may hit the plane if ray crosses it
            // in this case, ray is parallel so no hit at all
            expect(collector.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        });
        
        it('treatConvexAsSolid=true: ray starting in positive half-space hits plane normally', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 2, 0); // above plane (positive half-space)
            vec3.set(ray.direction, 0, -1, 0); // moving down toward plane
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: true },
                ray,
                planeShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2); // 2/10 = 0.2
        });
        
        it('treatConvexAsSolid=false: ray starting in positive half-space hits plane normally', () => {
            // plane at y=0, normal pointing up (positive y)
            const planeShape = plane.create({ plane: { normal: [0, 1, 0], constant: 0 } });
            const ray = raycast3.create();
            vec3.set(ray.origin, 0, 2, 0); // above plane (positive half-space)
            vec3.set(ray.direction, 0, -1, 0); // moving down toward plane
            vec3.normalize(ray.direction, ray.direction);
            ray.length = 10;
            
            const collector = createAnyCastRayCollector();
            collector.bodyIdB = 1;
            
            castRayVsShape(
                collector,
                { collideWithBackfaces: false, treatConvexAsSolid: false },
                ray,
                planeShape,
                0, 0,
                0, 0, 0,
                0, 0, 0, 1,
                1, 1, 1,
            );
            
            // should hit plane normally - treatConvexAsSolid only affects inside rays
            expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
            expect(collector.hit.fraction).toBeGreaterThan(0);
            expect(collector.hit.fraction).toBeCloseTo(0.2, 2);
        });
    });
});
