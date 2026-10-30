import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, MotionType, rigidBody } from '../../../src';
import * as dualAxisConstraintPart from '../../../src/constraints/constraint-part/dual-axis-constraint-part';
import { createTestWorld } from '../../helpers';

describe('dualAxisConstraintPart', () => {
    test('a spinning kinematic body drags the dynamic body sliding on it with its point velocity', () => {
        const { world, layers } = createTestWorld();
        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });

        // kinematic a spins about z; the slider point is at r1 + u = (1, 0, 0) from its centre
        const kinematic = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.KINEMATIC,
            position: vec3.fromValues(0, 0, 0),
        });
        const spin = 2;
        vec3.set(kinematic.motionProperties.angularVelocity, 0, 0, spin);

        // dynamic b slides along x with its centre of mass on the slider point, r2 = 0
        const dynamic = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(1, 0, 0),
        });

        const n1 = vec3.fromValues(0, 1, 0);
        const n2 = vec3.fromValues(0, 0, 1);

        const part = dualAxisConstraintPart.create();
        dualAxisConstraintPart.calculateConstraintProperties(
            part,
            kinematic,
            vec3.fromValues(1, 0, 0),
            dynamic,
            vec3.fromValues(0, 0, 0),
            n1,
            n2,
            -1,
        );

        expect(dualAxisConstraintPart.solveVelocityConstraint(part, kinematic, dynamic, n1, n2)).toBe(true);

        // the slider point on a moves at w x r1 = (0, 0, 2) x (1, 0, 0) = (0, 2, 0). k^-1 = 1/m2 on both axes,
        // so lambda = (2 m2, 0) and b leaves with the point's velocity across the slider
        const invMassB = dynamic.motionProperties.invMass;
        expect(part.totalLambda[0]).toBeCloseTo(spin / invMassB, 5);
        expect(part.totalLambda[1]).toBeCloseTo(0, 6);
        expect(dynamic.motionProperties.linearVelocity[0]).toBeCloseTo(0, 6);
        expect(dynamic.motionProperties.linearVelocity[1]).toBeCloseTo(spin, 5);
        expect(dynamic.motionProperties.linearVelocity[2]).toBeCloseTo(0, 6);
        expect(kinematic.motionProperties.angularVelocity[2]).toBe(spin);
    });
});
