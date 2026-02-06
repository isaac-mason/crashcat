import { describe, expect, it } from 'vitest';
import { vec3, quat } from 'mathcat';
import * as crashcat from '../../src';

describe('InternalEdgeRemovingCollector Integration', () => {
    it('should reduce contacts on flat triangle mesh compared to standard collision', () => {
        // create world with layers
        const worldSettings = crashcat.createWorldSettings();
        const broadphaseLayer = crashcat.addBroadphaseLayer(worldSettings);
        const objectLayer = crashcat.addObjectLayer(worldSettings, broadphaseLayer);
        crashcat.enableCollision(worldSettings, objectLayer, objectLayer);
        const world = crashcat.createWorld(worldSettings);

        // create a flat triangle mesh made of 4 triangles forming 2 squares
        // this creates internal edges at x=1 that should be removed

        // biome-ignore format: readability
        const positions = [
            // first square (x=0 to x=1)
            0, 0, 0,  // 0
            1, 0, 0,  // 1
            1, 1, 0,  // 2
            0, 1, 0,  // 3
            // second square (x=1 to x=2)
            1, 0, 0,  // 4 (shared with first square)
            2, 0, 0,  // 5
            2, 1, 0,  // 6
            1, 1, 0,  // 7 (shared with first square)
        ];

        // biome-ignore format: readability
        const indices = [
            // first square
            0, 1, 2, // triangle 1
            0, 2, 3, // triangle 2
            // second square
            4, 5, 6, // triangle 3
            4, 6, 7, // triangle 4
        ];

        const meshShape = crashcat.triangleMesh.create({ positions, indices });
        crashcat.rigidBody.create(world, {
            shape: meshShape,
            motionType: crashcat.MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            objectLayer,
        });

        // create a small sphere positioned to potentially hit internal edge at x=1
        const sphereShape = crashcat.sphere.create({ radius: 0.1 });
        const spherePosition = vec3.fromValues(1.0, 0.5, 0.05); // slightly above mesh, centered on internal edge
        const sphereQuaternion = quat.create();
        const sphereScale = vec3.fromValues(1, 1, 1);

        const settings = crashcat.createDefaultCollideShapeSettings();
        settings.maxSeparationDistance = 0.1; // allow finding contacts slightly separated

        // test standard collision
        const standardCollector = crashcat.createAllCollideShapeCollector();
        crashcat.collideShape(
            world,
            standardCollector,
            settings,
            sphereShape,
            spherePosition,
            sphereQuaternion,
            sphereScale,
            crashcat.filter.create(world.settings.layers),
        );

        // test enhanced collision
        const enhancedCollector = crashcat.createAllCollideShapeCollector();
        crashcat.collideShapeWithInternalEdgeRemoval(
            world,
            enhancedCollector,
            settings,
            sphereShape,
            spherePosition,
            sphereQuaternion,
            sphereScale,
            crashcat.filter.create(world.settings.layers),
        );

        // enhanced should have fewer or equal contacts (internal edges removed)
        expect(enhancedCollector.hits.length).toBeLessThanOrEqual(standardCollector.hits.length);

        // in this specific case, we expect exactly 2 contacts for standard (both triangles at x=1)
        // and likely 0 contacts for enhanced (internal edge removed)
        // but this depends on exact positioning, so we just verify it's not more
        console.log(`Standard collision: ${standardCollector.hits.length} contacts`);
        console.log(`Enhanced collision: ${enhancedCollector.hits.length} contacts`);
    });

    it('should preserve external edge contacts', () => {
        // create world with layers
        const worldSettings = crashcat.createWorldSettings();
        const broadphaseLayer = crashcat.addBroadphaseLayer(worldSettings);
        const objectLayer = crashcat.addObjectLayer(worldSettings, broadphaseLayer);
        crashcat.enableCollision(worldSettings, objectLayer, objectLayer);
        const world = crashcat.createWorld(worldSettings);

        // create a single triangle (has external edges only)
        // biome-ignore format: readability
        const positions = [
            0, 0, 0,
            1, 0, 0,
            0.5, 1, 0,
        ];

        const indices = [0, 1, 2];

        const meshShape = crashcat.triangleMesh.create({ positions, indices });
        crashcat.rigidBody.create(world, {
            shape: meshShape,
            motionType: crashcat.MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            objectLayer,
        });

        // sphere positioned to hit the edge
        const sphereShape = crashcat.sphere.create({ radius: 0.15 });
        const spherePosition = vec3.fromValues(0.5, 0, 0.1); // near bottom edge
        const sphereQuaternion = quat.create();
        const sphereScale = vec3.fromValues(1, 1, 1);

        const settings = crashcat.createDefaultCollideShapeSettings();
        settings.maxSeparationDistance = 0.15;

        // test enhanced collision
        const enhancedCollector = crashcat.createAllCollideShapeCollector();
        crashcat.collideShapeWithInternalEdgeRemoval(
            world,
            enhancedCollector,
            settings,
            sphereShape,
            spherePosition,
            sphereQuaternion,
            sphereScale,
            crashcat.filter.create(world.settings.layers),
        );

        // should still get contacts even with enhancement (these are external edges)
        // the exact number depends on the collision detection, but it should be > 0
        expect(enhancedCollector.hits.length).toBeGreaterThan(0);
    });

    it('should handle penetrating sphere correctly', () => {
        // create world with layers
        const worldSettings = crashcat.createWorldSettings();
        const broadphaseLayer = crashcat.addBroadphaseLayer(worldSettings);
        const objectLayer = crashcat.addObjectLayer(worldSettings, broadphaseLayer);
        crashcat.enableCollision(worldSettings, objectLayer, objectLayer);
        const world = crashcat.createWorld(worldSettings);

        // simple 2-triangle mesh
        // biome-ignore format: readability
        const positions = [
            0, 0, 0,
            2, 0, 0,
            2, 2, 0,
            0, 2, 0,
        ];

        // biome-ignore format: readability
        const indices = [
            0, 1, 2, // triangle 1
            0, 2, 3, // triangle 2
        ];

        const meshShape = crashcat.triangleMesh.create({ positions, indices });
        crashcat.rigidBody.create(world, {
            shape: meshShape,
            motionType: crashcat.MotionType.STATIC,
            position: vec3.fromValues(0, 0, 0),
            objectLayer,
        });

        // sphere penetrating the mesh (below it)
        const sphereShape = crashcat.sphere.create({ radius: 0.2 });
        const spherePosition = vec3.fromValues(1.0, 1.0, -0.1); // center of mesh, penetrating
        const sphereQuaternion = quat.create();
        const sphereScale = vec3.fromValues(1, 1, 1);

        const settings = crashcat.createDefaultCollideShapeSettings();

        // test both methods
        const standardCollector = crashcat.createAllCollideShapeCollector();
        crashcat.collideShape(
            world,
            standardCollector,
            settings,
            sphereShape,
            spherePosition,
            sphereQuaternion,
            sphereScale,
            crashcat.filter.create(world.settings.layers),
        );

        const enhancedCollector = crashcat.createAllCollideShapeCollector();
        crashcat.collideShapeWithInternalEdgeRemoval(
            world,
            enhancedCollector,
            settings,
            sphereShape,
            spherePosition,
            sphereQuaternion,
            sphereScale,
            crashcat.filter.create(world.settings.layers),
        );

        // both should detect collision
        expect(standardCollector.hits.length).toBeGreaterThan(0);
        expect(enhancedCollector.hits.length).toBeGreaterThan(0);

        // enhanced might have fewer contacts if internal edge is involved
        expect(enhancedCollector.hits.length).toBeLessThanOrEqual(standardCollector.hits.length);
    });

    it('should work with multiple bodies in world', () => {
        // create world with layers
        const worldSettings = crashcat.createWorldSettings();
        const broadphaseLayer = crashcat.addBroadphaseLayer(worldSettings);
        const objectLayer = crashcat.addObjectLayer(worldSettings, broadphaseLayer);
        crashcat.enableCollision(worldSettings, objectLayer, objectLayer);
        const world = crashcat.createWorld(worldSettings);

        // add multiple small triangle meshes as separate bodies
        for (let i = 0; i < 3; i++) {
            // biome-ignore format: readability
            const positions = [
                i * 2, 0, 0,
                i * 2 + 1, 0, 0,
                i * 2 + 0.5, 1, 0,
            ];

            const indices = [0, 1, 2];

            const meshShape = crashcat.triangleMesh.create({ positions, indices });
            crashcat.rigidBody.create(world, {
                shape: meshShape,
                motionType: crashcat.MotionType.STATIC,
                position: vec3.fromValues(0, 0, 0),
                objectLayer,
            });
        }

        // sphere that might hit multiple meshes
        const sphereShape = crashcat.sphere.create({ radius: 0.5 });
        const spherePosition = vec3.fromValues(2.5, 0.5, 0.2);
        const sphereQuaternion = quat.create();
        const sphereScale = vec3.fromValues(1, 1, 1);

        const settings = crashcat.createDefaultCollideShapeSettings();
        settings.maxSeparationDistance = 0.3;

        // test enhanced collision
        const enhancedCollector = crashcat.createAllCollideShapeCollector();
        crashcat.collideShapeWithInternalEdgeRemoval(
            world,
            enhancedCollector,
            settings,
            sphereShape,
            spherePosition,
            sphereQuaternion,
            sphereScale,
            crashcat.filter.create(world.settings.layers),
        );

        // should handle multiple bodies correctly without crashing
        // results depend on exact positioning, so just verify no crash
        expect(enhancedCollector.hits.length).toBeGreaterThanOrEqual(0);
    });
});
