import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { ContactValidateResult, type Listener, MotionType, rigidBody, sphere, triangleMesh, updateWorld } from '../src';
import { createTestWorld } from './helpers';

describe('narrowphase collide collectors', () => {
    // rejecting every contact for a pair forces the collector's early out, which bounds every
    // further candidate away: a sphere sunk into both triangles of a floor is validated once
    for (const useManifoldReduction of [true, false]) {
        test(`reject all contacts stops the pair (manifold reduction ${useManifoldReduction ? 'on' : 'off'})`, () => {
            const { world, layers } = createTestWorld();
            world.settings.narrowphase.useManifoldReduction = useManifoldReduction;

            // a 4 x 4 floor at y = 0 split along its diagonal, facing up
            const floor = triangleMesh.create({
                positions: [-2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2],
                indices: [0, 2, 1, 0, 3, 2],
            });

            rigidBody.create(world, {
                shape: floor,
                objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
                motionType: MotionType.STATIC,
                position: vec3.fromValues(0, 0, 0),
            });

            // sunk 0.5 into the floor over the diagonal, so it overlaps both triangles
            rigidBody.create(world, {
                shape: sphere.create({ radius: 1 }),
                objectLayer: layers.OBJECT_LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: vec3.fromValues(0, 0.5, 0),
            });

            let validateCalls = 0;
            let added = 0;
            const listener: Listener = {
                onContactValidate: () => {
                    validateCalls++;
                    return ContactValidateResult.REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR;
                },
                onContactAdded: () => added++,
            };

            updateWorld(world, listener, 1 / 60);

            expect(validateCalls).toBe(1);
            expect(added).toBe(0);
        });
    }
});
