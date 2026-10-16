import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { INACTIVE_BODY_INDEX, MotionType, rigidBody, sphere, updateWorld } from '../src';
import { createTestWorld } from './helpers';

describe('Islands', () => {
    test('should group connected bodies and exclude static bodies', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 0.5 });

        // create two dynamic bodies that are overlapping (will have contact)
        const body1 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0), // overlapping with body2
        });

        const body2 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // create a static body (should not be in any island)
        const staticBody = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(5, 0, 0),
        });

        // create a separate dynamic body far away (should be in separate island, no contacts)
        const body3 = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(10, 0, 0),
        });

        // step the world to generate contacts and build islands
        updateWorld(world, undefined, 1 / 60);

        // verify static body is not in active bodies
        expect(staticBody.activeIndex).toBe(INACTIVE_BODY_INDEX);
        expect(staticBody.islandIndex).toBe(-1);

        // verify dynamic bodies have valid active indices
        expect(body1.activeIndex).toBeGreaterThanOrEqual(0);
        expect(body2.activeIndex).toBeGreaterThanOrEqual(0);
        expect(body3.activeIndex).toBeGreaterThanOrEqual(0);

        // body1 and body2 should be in the same island (they're in contact)
        expect(body1.islandIndex).toBe(body2.islandIndex);
        expect(body1.islandIndex).toBeGreaterThanOrEqual(0);

        // body3 should be in a different island (no contacts with body1/body2)
        expect(body3.islandIndex).toBeGreaterThanOrEqual(0);
        expect(body3.islandIndex).not.toBe(body1.islandIndex);

        // verify islands were created (should have at least 2 islands)
        expect(world.islands.islands.length).toBeGreaterThanOrEqual(2);

        // verify static body is not in activeBodyIndices
        expect(world.bodies.activeBodyIndices).not.toContain(world.bodies.pool.indexOf(staticBody));
    });

    test('should calculate correct velocity and position steps per island', () => {
        const { world, layers } = createTestWorld();

        // override default solver iterations
        world.settings.solver.velocityIterations = 10;
        world.settings.solver.positionIterations = 5;

        const shape = sphere.create({ radius: 0.5 });

        // create body with default iterations (uses 0, should get world defaults)
        const bodyDefault = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.6, 0), // overlapping with bodyCustom
        });

        // create body with custom iterations
        const bodyCustom = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });

        // set custom iteration overrides
        bodyCustom.motionProperties.numVelocityStepsOverride = 20;
        bodyCustom.motionProperties.numPositionStepsOverride = 15;

        // create another body far away with different overrides
        const bodyOther = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(10, 0, 0),
        });

        bodyOther.motionProperties.numVelocityStepsOverride = 8;
        bodyOther.motionProperties.numPositionStepsOverride = 3;

        // step the world to build islands
        updateWorld(world, undefined, 1 / 60);

        // bodyDefault and bodyCustom should be in same island (overlapping)
        expect(bodyDefault.islandIndex).toBe(bodyCustom.islandIndex);
        expect(bodyDefault.islandIndex).toBeGreaterThanOrEqual(0);

        // find the island containing bodyDefault and bodyCustom
        const island1 = world.islands.islands.find((isl) => isl.index === bodyDefault.islandIndex);
        expect(island1).toBeDefined();

        // island should use max of all body overrides and default
        // bodyDefault uses 0 (apply default 10), bodyCustom uses 20 -> max(10, 20) = 20
        expect(island1!.numVelocitySteps).toBe(20);
        // bodyDefault uses 0 (apply default 5), bodyCustom uses 15 -> max(5, 15) = 15
        expect(island1!.numPositionSteps).toBe(15);

        // find the island containing bodyOther (separate island)
        const island2 = world.islands.islands.find((isl) => isl.index === bodyOther.islandIndex);
        expect(island2).toBeDefined();

        // bodyOther is alone in its island with explicit override 8
        // since no body in this island uses 0, default is NOT applied -> uses 8
        expect(island2!.numVelocitySteps).toBe(8);
        // bodyOther requests 3 position steps, no body uses 0 -> uses 3
        expect(island2!.numPositionSteps).toBe(3);
    });

    test('body island index names the island that lists the body after islands are sorted', () => {
        const { world, layers } = createTestWorld();

        const shape = sphere.create({ radius: 0.5 });
        const create = (x: number, y: number) =>
            rigidBody.create(world, {
                shape,
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(x, y, 0),
            });

        // built in this order: a lone body (no contacts), a touching pair (1 contact), a chain of three (2 contacts).
        // islands sort by contact count, largest first, so the build order reverses
        const lone = create(20, 0);
        const pair = [create(10, 0), create(10, 0.9)];
        const chain = [create(0, 0), create(0, 0.9), create(0, 1.8)];

        updateWorld(world, undefined, 1 / 60);

        const islands = world.islands.islands;
        expect(islands.length).toBe(3);
        expect(islands[0].bodyIndices.length).toBe(3);
        expect(islands[1].bodyIndices.length).toBe(2);
        expect(islands[2].bodyIndices.length).toBe(1);

        for (let i = 0; i < islands.length; i++) {
            expect(islands[i].index).toBe(i);
            for (const activeIndex of islands[i].bodyIndices) {
                const body = world.bodies.pool[world.bodies.activeBodyIndices[activeIndex]];
                expect(body.islandIndex).toBe(i);
            }
        }

        expect(lone.islandIndex).toBe(2);
        for (const body of pair) expect(body.islandIndex).toBe(1);
        for (const body of chain) expect(body.islandIndex).toBe(0);
    });
});
