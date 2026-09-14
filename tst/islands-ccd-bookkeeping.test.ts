import { vec3 } from 'math';
import { describe, expect, it } from 'vitest';
import { box, MotionQuality, MotionType, rigidBody, sphere, updateWorld } from '../src';
import { createTestWorld } from './helpers';

describe('per-body island and ccd bookkeeping', () => {
    it('clears islandIndex when a body leaves the active set and reassigns it on wake', () => {
        const { world, layers } = createTestWorld();

        const ground = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(10, 0.5, 10) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(0, -0.5, 0),
        });

        const resting = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0.5, 0),
        });

        updateWorld(world, undefined, 1 / 60);
        expect(resting.islandIndex).toBeGreaterThanOrEqual(0);
        // static bodies are never in an island
        expect(ground.islandIndex).toBe(-1);

        for (let i = 0; i < 240 && !resting.sleeping; i++) {
            updateWorld(world, undefined, 1 / 60);
        }
        expect(resting.sleeping).toBe(true);
        expect(resting.islandIndex).toBe(-1);

        // wake it by dropping a ball on it
        const ball = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.25 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 1.3, 0),
        });
        rigidBody.setLinearVelocity(world, ball, vec3.fromValues(0, -5, 0));

        for (let i = 0; i < 30 && resting.sleeping; i++) {
            updateWorld(world, undefined, 1 / 60);
        }
        expect(resting.sleeping).toBe(false);
        expect(resting.islandIndex).toBeGreaterThanOrEqual(0);
        expect(resting.islandIndex).toBe(ball.islandIndex);
        expect(ground.islandIndex).toBe(-1);
    });

    it('clears ccdBodyIndex only for bodies that had a ccd record', () => {
        const { world, layers } = createTestWorld();
        world.settings.gravityEnabled = false;

        const wall = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(0.5, 5, 5) }),
            objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(20, 0, 0),
        });

        const bullet = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            motionQuality: MotionQuality.LINEAR_CAST,
            position: vec3.fromValues(0, 0, 0),
        });
        rigidBody.setLinearVelocity(world, bullet, vec3.fromValues(200, 0, 0));

        const bystander = rigidBody.create(world, {
            shape: sphere.create({ radius: 0.1 }),
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 5, 0),
        });

        updateWorld(world, undefined, 1 / 60);

        // the fast body went through ccd this step and keeps its record until the next step clears it
        expect(world.ccd.ccdBodies.length).toBe(1);
        expect(bullet.ccdBodyIndex).toBe(0);
        expect(bystander.ccdBodyIndex).toBe(-1);
        expect(wall.ccdBodyIndex).toBe(-1);

        // stop it: the next step clears the record and no body qualifies for ccd
        rigidBody.setLinearVelocity(world, bullet, vec3.fromValues(0, 0, 0));
        updateWorld(world, undefined, 1 / 60);

        expect(world.ccd.ccdBodies.length).toBe(0);
        expect(bullet.ccdBodyIndex).toBe(-1);
        expect(bystander.ccdBodyIndex).toBe(-1);
        expect(wall.ccdBodyIndex).toBe(-1);
    });
});
