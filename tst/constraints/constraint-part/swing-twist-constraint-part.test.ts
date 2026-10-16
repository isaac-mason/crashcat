import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, MotionType, rigidBody, SwingType } from '../../../src';
import * as angleConstraintPart from '../../../src/constraints/constraint-part/angle-constraint-part';
import * as swingTwistConstraintPart from '../../../src/constraints/constraint-part/swing-twist-constraint-part';
import { createTestWorld } from '../../helpers';

describe('swingTwistConstraintPart', () => {
    test('a swing a fraction of a microradian past a pyramid limit still activates the swing limit', () => {
        const { world, layers } = createTestWorld();
        const shape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5) });
        const bodyA = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(0, 0, 0),
        });
        const bodyB = rigidBody.create(world, {
            shape,
            objectLayer: layers.OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position: vec3.fromValues(1, 0, 0),
        });

        const part = swingTwistConstraintPart.create();
        part.swingType = SwingType.PYRAMID;
        swingTwistConstraintPart.setLimits(part, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5);

        // swing about y by 4e-7 rad past the 0.5 rad limit: the pyramid clamps the half angle back to exactly 0.25,
        // so the axis from the clamped swing to the swing has length sin(4e-7)
        const overshoot = 4e-7;
        const rotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), 0.5 + overshoot);
        swingTwistConstraintPart.calculateConstraintProperties(part, bodyA, bodyB, rotation, quat.create());

        expect(angleConstraintPart.isActive(part.swingLimitYConstraintPart)).toBe(true);
        // desired x current is the rotation from the clamped swing on to the swing: +y
        expect(part.worldSpaceSwingLimitYRotationAxis[0]).toBeCloseTo(0, 6);
        expect(part.worldSpaceSwingLimitYRotationAxis[1]).toBeCloseTo(1, 6);
        expect(part.worldSpaceSwingLimitYRotationAxis[2]).toBeCloseTo(0, 6);
        expect(angleConstraintPart.isActive(part.swingLimitZConstraintPart)).toBe(false);
    });

    test('the cone clamp of a far out swing on the most eccentric cone lands on the ellipse', () => {
        // y just wider than the locked angle, z nearly free: the narrowest ellipse setLimits allows
        const part = swingTwistConstraintPart.create();
        const deg = Math.PI / 180;
        swingTwistConstraintPart.setLimits(part, -0.5, 0.5, -0.6 * deg, 0.6 * deg, -170 * deg, 170 * deg);

        const swing = quat.fromValues(0, 0.6, 0.3, Math.sqrt(1 - 0.6 * 0.6 - 0.3 * 0.3));
        const twist = quat.create();
        swingTwistConstraintPart.clampSwingTwist(part, swing, twist);

        // newton exits once g(t) = (y/a)^2 + (z/b)^2 - 1 is under 1e-6
        const a = part.sinSwingYHalfMaxAngle;
        const b = part.sinSwingZHalfMaxAngle;
        const ellipse = (swing[1] / a) ** 2 + (swing[2] / b) ** 2;
        expect(Math.abs(ellipse - 1)).toBeLessThan(1e-6);
        expect(quat.length(swing)).toBeCloseTo(1, 12);
    });
});
