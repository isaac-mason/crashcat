import { mat4, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, dof, MotionType, rigidBody } from '../../../src';
import * as pointConstraintPart from '../../../src/constraints/constraint-part/point-constraint-part';
import { createTestWorld } from '../../helpers';

describe('pointConstraintPart', () => {
    test('a body with translation locked on y gains no y velocity from the velocity solve', () => {
        const { world, layers } = createTestWorld();
        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        const free = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });
        const locked = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
            allowedDegreesOfFreedom: dof(true, false, true, true, true, true),
        });

        // pinned at both centres of mass, the free body moving away diagonally
        vec3.set(free.motionProperties.linearVelocity, 3, 3, 0);

        const part = pointConstraintPart.create();
        const rotation = mat4.create();
        pointConstraintPart.calculateConstraintProperties(
            part,
            free,
            rotation,
            vec3.fromValues(0, 0, 0),
            locked,
            rotation,
            vec3.fromValues(0, 0, 0),
            -1,
        );

        expect(pointConstraintPart.solveVelocityConstraint(part, free, locked)).toBe(true);

        // per axis lambda = (v1 - v2) / (1/m1 + 1/m2), the same on x and y; the locked body takes lambda / m2 on x only
        const invMassFree = free.motionProperties.invMass;
        const invMassLocked = locked.motionProperties.invMass;
        const axisLambda = 3 / (invMassFree + invMassLocked);

        expect(locked.motionProperties.linearVelocity[0]).toBeCloseTo(axisLambda * invMassLocked, 5);
        expect(locked.motionProperties.linearVelocity[1]).toBe(0);
        expect(locked.motionProperties.linearVelocity[2]).toBe(0);

        // the free body still gives up its share on both axes
        expect(free.motionProperties.linearVelocity[0]).toBeCloseTo(3 - axisLambda * invMassFree, 5);
        expect(free.motionProperties.linearVelocity[1]).toBeCloseTo(3 - axisLambda * invMassFree, 5);
    });
});
