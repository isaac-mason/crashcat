import { vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { DOF_ALL } from '../../src/body/dof';
import * as motionPropertiesModule from '../../src/body/motion-properties';
import { MotionType } from '../../src/body/motion-type';
import type { RigidBody } from '../../src/body/rigid-body';
import * as axisConstraintPart from '../../src/constraints/constraint-part/axis-constraint-part';
import * as contactConstraintPart from '../../src/constraints/constraint-part/contact-constraint-part';

/**
 * create a minimal rigid body for testing constraint parts.
 * only the fields accessed by axis-constraint-part functions are populated.
 */
function createTestBody(
    motionType: MotionType,
    linearVelocity: [number, number, number],
    angularVelocity: [number, number, number],
    invMass: number,
    allowedDOFs: number = DOF_ALL,
): RigidBody {
    const mp = motionPropertiesModule.create();
    vec3.set(mp.linearVelocity, linearVelocity[0], linearVelocity[1], linearVelocity[2]);
    vec3.set(mp.angularVelocity, angularVelocity[0], angularVelocity[1], angularVelocity[2]);
    mp.invMass = invMass;
    mp.allowedDegreesOfFreedom = allowedDOFs;

    return {
        motionType,
        motionProperties: mp,
    } as RigidBody;
}

/**
 * set up a constraint part with known values for testing.
 * mimics what calculateConstraintProperties would produce.
 */
function setupConstraintPart(
    part: axisConstraintPart.AxisConstraintPart,
    opts: {
        r1PlusUxAxis: [number, number, number];
        r2xAxis: [number, number, number];
        invI1_r1PlusUxAxis: [number, number, number];
        invI2_r2xAxis: [number, number, number];
        effectiveMass: number;
        totalLambda: number;
    },
): void {
    vec3.set(part.r1PlusUxAxis, opts.r1PlusUxAxis[0], opts.r1PlusUxAxis[1], opts.r1PlusUxAxis[2]);
    vec3.set(part.r2xAxis, opts.r2xAxis[0], opts.r2xAxis[1], opts.r2xAxis[2]);
    vec3.set(part.invI1_r1PlusUxAxis, opts.invI1_r1PlusUxAxis[0], opts.invI1_r1PlusUxAxis[1], opts.invI1_r1PlusUxAxis[2]);
    vec3.set(part.invI2_r2xAxis, opts.invI2_r2xAxis[0], opts.invI2_r2xAxis[1], opts.invI2_r2xAxis[2]);
    part.effectiveMass = opts.effectiveMass;
    part.totalLambda = opts.totalLambda;
}

function clonePart(src: axisConstraintPart.AxisConstraintPart): axisConstraintPart.AxisConstraintPart {
    const dst = axisConstraintPart.create();
    vec3.copy(dst.r1PlusUxAxis, src.r1PlusUxAxis);
    vec3.copy(dst.r2xAxis, src.r2xAxis);
    vec3.copy(dst.invI1_r1PlusUxAxis, src.invI1_r1PlusUxAxis);
    vec3.copy(dst.invI2_r2xAxis, src.invI2_r2xAxis);
    dst.effectiveMass = src.effectiveMass;
    dst.totalLambda = src.totalLambda;
    dst.springPart.bias = src.springPart.bias;
    dst.springPart.softness = src.springPart.softness;
    return dst;
}

const CONSTRAINT_SETUP = {
    r1PlusUxAxis: [0.3, -0.5, 0.1] as [number, number, number],
    r2xAxis: [-0.2, 0.4, -0.3] as [number, number, number],
    invI1_r1PlusUxAxis: [0.15, -0.25, 0.05] as [number, number, number],
    invI2_r2xAxis: [-0.1, 0.2, -0.15] as [number, number, number],
    effectiveMass: 0.8,
    totalLambda: 2.5,
};

describe('contactConstraintPart vs axisConstraintPart equivalence', () => {
    test('getTotalLambda produces identical result — two dynamic bodies', () => {
        const axis = vec3.fromValues(0, 1, 0);
        const bodyA = createTestBody(MotionType.DYNAMIC, [1, 2, 3], [0.1, 0.2, 0.3], 1.0);
        const bodyB = createTestBody(MotionType.DYNAMIC, [-1, 0.5, 1], [-0.1, 0.15, -0.2], 2.0);

        const partRef = axisConstraintPart.create();
        setupConstraintPart(partRef, CONSTRAINT_SETUP);
        const partLocal = clonePart(partRef);

        // reference: body-based
        const refResult = axisConstraintPart.getTotalLambda(partRef, bodyA, bodyB, axis);

        // local: velocity-local based
        const linVelA = vec3.clone(bodyA.motionProperties.linearVelocity);
        const angVelA = vec3.clone(bodyA.motionProperties.angularVelocity);
        const linVelB = vec3.clone(bodyB.motionProperties.linearVelocity);
        const angVelB = vec3.clone(bodyB.motionProperties.angularVelocity);
        const localResult = contactConstraintPart.getTotalLambda(partLocal, linVelA, angVelA, linVelB, angVelB, true, true, axis);

        expect(localResult).toBeCloseTo(refResult, 10);
    });

    test('getTotalLambda produces identical result — dynamic vs static', () => {
        const axis = vec3.fromValues(0, 1, 0);
        const bodyA = createTestBody(MotionType.DYNAMIC, [1, 2, 3], [0.1, 0.2, 0.3], 1.0);
        const bodyB = createTestBody(MotionType.STATIC, [0, 0, 0], [0, 0, 0], 0);

        const partRef = axisConstraintPart.create();
        setupConstraintPart(partRef, CONSTRAINT_SETUP);
        const partLocal = clonePart(partRef);

        const refResult = axisConstraintPart.getTotalLambda(partRef, bodyA, bodyB, axis);

        const linVelA = vec3.clone(bodyA.motionProperties.linearVelocity);
        const angVelA = vec3.clone(bodyA.motionProperties.angularVelocity);
        const linVelB = vec3.create(); // static
        const angVelB = vec3.create();
        const localResult = contactConstraintPart.getTotalLambda(
            partLocal,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            false,
            axis,
        );

        expect(localResult).toBeCloseTo(refResult, 10);
    });

    test('getTotalLambda produces identical result — dynamic vs kinematic', () => {
        const axis = vec3.fromValues(0.577, 0.577, 0.577); // ~normalized diagonal
        const bodyA = createTestBody(MotionType.DYNAMIC, [2, -1, 0.5], [0.3, -0.1, 0.2], 1.5);
        const bodyB = createTestBody(MotionType.KINEMATIC, [0.5, 0.5, 0.5], [0, 0, 0], 0);

        const partRef = axisConstraintPart.create();
        setupConstraintPart(partRef, CONSTRAINT_SETUP);
        const partLocal = clonePart(partRef);

        const refResult = axisConstraintPart.getTotalLambda(partRef, bodyA, bodyB, axis);

        // kinematic is "moving" (not static) — has velocity contribution to Jv
        const linVelA = vec3.clone(bodyA.motionProperties.linearVelocity);
        const angVelA = vec3.clone(bodyA.motionProperties.angularVelocity);
        const linVelB = vec3.clone(bodyB.motionProperties.linearVelocity);
        const angVelB = vec3.clone(bodyB.motionProperties.angularVelocity);
        const localResult = contactConstraintPart.getTotalLambda(partLocal, linVelA, angVelA, linVelB, angVelB, true, true, axis);

        expect(localResult).toBeCloseTo(refResult, 10);
    });

    test('applyLambda produces identical velocity changes — two dynamic bodies', () => {
        const axis = vec3.fromValues(0, 1, 0);
        const invMassA = 1.0;
        const invMassB = 2.0;
        const newTotalLambda = 3.7;

        const bodyA = createTestBody(MotionType.DYNAMIC, [1, 2, 3], [0.1, 0.2, 0.3], invMassA);
        const bodyB = createTestBody(MotionType.DYNAMIC, [-1, 0.5, 1], [-0.1, 0.15, -0.2], invMassB);

        // reference path: use axisConstraintPart.applyLambda which writes to body.motionProperties
        const partRef = axisConstraintPart.create();
        setupConstraintPart(partRef, CONSTRAINT_SETUP);
        axisConstraintPart.applyLambda(partRef, bodyA, bodyB, invMassA, invMassB, axis, newTotalLambda);

        // local path: create fresh body copies and use contactConstraintPart
        const bodyA2 = createTestBody(MotionType.DYNAMIC, [1, 2, 3], [0.1, 0.2, 0.3], invMassA);
        const bodyB2 = createTestBody(MotionType.DYNAMIC, [-1, 0.5, 1], [-0.1, 0.15, -0.2], invMassB);
        const partLocal = axisConstraintPart.create();
        setupConstraintPart(partLocal, CONSTRAINT_SETUP);

        const linVelA = vec3.clone(bodyA2.motionProperties.linearVelocity);
        const angVelA = vec3.clone(bodyA2.motionProperties.angularVelocity);
        const linVelB = vec3.clone(bodyB2.motionProperties.linearVelocity);
        const angVelB = vec3.clone(bodyB2.motionProperties.angularVelocity);

        contactConstraintPart.applyLambda(
            partLocal,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            invMassA,
            invMassB,
            axis,
            newTotalLambda,
        );

        // the local version does NOT apply DOF masking, while the axisConstraintPart version does.
        // since DOF_ALL is set, the masking is a no-op, so results should be identical.
        expect(linVelA[0]).toBeCloseTo(bodyA.motionProperties.linearVelocity[0], 10);
        expect(linVelA[1]).toBeCloseTo(bodyA.motionProperties.linearVelocity[1], 10);
        expect(linVelA[2]).toBeCloseTo(bodyA.motionProperties.linearVelocity[2], 10);
        expect(angVelA[0]).toBeCloseTo(bodyA.motionProperties.angularVelocity[0], 10);
        expect(angVelA[1]).toBeCloseTo(bodyA.motionProperties.angularVelocity[1], 10);
        expect(angVelA[2]).toBeCloseTo(bodyA.motionProperties.angularVelocity[2], 10);

        expect(linVelB[0]).toBeCloseTo(bodyB.motionProperties.linearVelocity[0], 10);
        expect(linVelB[1]).toBeCloseTo(bodyB.motionProperties.linearVelocity[1], 10);
        expect(linVelB[2]).toBeCloseTo(bodyB.motionProperties.linearVelocity[2], 10);
        expect(angVelB[0]).toBeCloseTo(bodyB.motionProperties.angularVelocity[0], 10);
        expect(angVelB[1]).toBeCloseTo(bodyB.motionProperties.angularVelocity[1], 10);
        expect(angVelB[2]).toBeCloseTo(bodyB.motionProperties.angularVelocity[2], 10);

        // totalLambda should match
        expect(partLocal.totalLambda).toBe(partRef.totalLambda);
    });

    test('warmStart produces identical velocity changes — two dynamic bodies', () => {
        const axis = vec3.fromValues(0, 1, 0);
        const invMassA = 1.0;
        const invMassB = 2.0;
        const warmStartRatio = 0.95;

        const bodyA = createTestBody(MotionType.DYNAMIC, [1, 2, 3], [0.1, 0.2, 0.3], invMassA);
        const bodyB = createTestBody(MotionType.DYNAMIC, [-1, 0.5, 1], [-0.1, 0.15, -0.2], invMassB);

        // reference path
        const partRef = axisConstraintPart.create();
        setupConstraintPart(partRef, CONSTRAINT_SETUP);
        axisConstraintPart.warmStart(partRef, bodyA, bodyB, invMassA, invMassB, axis, warmStartRatio);

        // local path
        const bodyA2 = createTestBody(MotionType.DYNAMIC, [1, 2, 3], [0.1, 0.2, 0.3], invMassA);
        const bodyB2 = createTestBody(MotionType.DYNAMIC, [-1, 0.5, 1], [-0.1, 0.15, -0.2], invMassB);
        const partLocal = axisConstraintPart.create();
        setupConstraintPart(partLocal, CONSTRAINT_SETUP);

        const linVelA = vec3.clone(bodyA2.motionProperties.linearVelocity);
        const angVelA = vec3.clone(bodyA2.motionProperties.angularVelocity);
        const linVelB = vec3.clone(bodyB2.motionProperties.linearVelocity);
        const angVelB = vec3.clone(bodyB2.motionProperties.angularVelocity);

        contactConstraintPart.warmStart(
            partLocal,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            invMassA,
            invMassB,
            axis,
            warmStartRatio,
        );

        // DOF_ALL means no masking difference
        expect(linVelA[0]).toBeCloseTo(bodyA.motionProperties.linearVelocity[0], 10);
        expect(linVelA[1]).toBeCloseTo(bodyA.motionProperties.linearVelocity[1], 10);
        expect(linVelA[2]).toBeCloseTo(bodyA.motionProperties.linearVelocity[2], 10);
        expect(angVelA[0]).toBeCloseTo(bodyA.motionProperties.angularVelocity[0], 10);
        expect(angVelA[1]).toBeCloseTo(bodyA.motionProperties.angularVelocity[1], 10);
        expect(angVelA[2]).toBeCloseTo(bodyA.motionProperties.angularVelocity[2], 10);

        expect(linVelB[0]).toBeCloseTo(bodyB.motionProperties.linearVelocity[0], 10);
        expect(linVelB[1]).toBeCloseTo(bodyB.motionProperties.linearVelocity[1], 10);
        expect(linVelB[2]).toBeCloseTo(bodyB.motionProperties.linearVelocity[2], 10);
        expect(angVelB[0]).toBeCloseTo(bodyB.motionProperties.angularVelocity[0], 10);
        expect(angVelB[1]).toBeCloseTo(bodyB.motionProperties.angularVelocity[1], 10);
        expect(angVelB[2]).toBeCloseTo(bodyB.motionProperties.angularVelocity[2], 10);

        expect(partLocal.totalLambda).toBe(partRef.totalLambda);
    });

    test('multiple sequential operations accumulate identically', () => {
        // simulate a 2-contact-point constraint: friction1, friction2, normal1, normal2
        const normal = vec3.fromValues(0, 1, 0);
        const tangent1 = vec3.fromValues(1, 0, 0);
        const tangent2 = vec3.fromValues(0, 0, 1);
        const invMassA = 1.0;
        const invMassB = 0.5;

        // set up two contact points worth of constraint parts
        const normalPart1Ref = axisConstraintPart.create();
        const normalPart2Ref = axisConstraintPart.create();
        const tan1Part1Ref = axisConstraintPart.create();
        const tan1Part2Ref = axisConstraintPart.create();

        setupConstraintPart(normalPart1Ref, { ...CONSTRAINT_SETUP, totalLambda: 1.0 });
        setupConstraintPart(normalPart2Ref, { ...CONSTRAINT_SETUP, totalLambda: 1.5 });
        setupConstraintPart(tan1Part1Ref, { ...CONSTRAINT_SETUP, totalLambda: 0.3, effectiveMass: 0.6 });
        setupConstraintPart(tan1Part2Ref, { ...CONSTRAINT_SETUP, totalLambda: -0.2, effectiveMass: 0.6 });

        const normalPart1Local = clonePart(normalPart1Ref);
        const normalPart2Local = clonePart(normalPart2Ref);
        const tan1Part1Local = clonePart(tan1Part1Ref);
        const tan1Part2Local = clonePart(tan1Part2Ref);

        // create two sets of bodies with same initial state
        const bodyARef = createTestBody(MotionType.DYNAMIC, [0.5, -1, 0.3], [0.1, -0.05, 0.2], invMassA);
        const bodyBRef = createTestBody(MotionType.DYNAMIC, [-0.3, 0.2, 0.1], [-0.02, 0.1, -0.05], invMassB);

        // reference path: 4 getTotalLambda + 4 applyLambda on body directly
        let l1 = axisConstraintPart.getTotalLambda(tan1Part1Ref, bodyARef, bodyBRef, tangent1);
        axisConstraintPart.applyLambda(tan1Part1Ref, bodyARef, bodyBRef, invMassA, invMassB, tangent1, l1);

        let l2 = axisConstraintPart.getTotalLambda(tan1Part2Ref, bodyARef, bodyBRef, tangent2);
        axisConstraintPart.applyLambda(tan1Part2Ref, bodyARef, bodyBRef, invMassA, invMassB, tangent2, l2);

        let n1 = axisConstraintPart.getTotalLambda(normalPart1Ref, bodyARef, bodyBRef, normal);
        n1 = Math.max(0, n1);
        axisConstraintPart.applyLambda(normalPart1Ref, bodyARef, bodyBRef, invMassA, invMassB, normal, n1);

        let n2 = axisConstraintPart.getTotalLambda(normalPart2Ref, bodyARef, bodyBRef, normal);
        n2 = Math.max(0, n2);
        axisConstraintPart.applyLambda(normalPart2Ref, bodyARef, bodyBRef, invMassA, invMassB, normal, n2);

        // local path: same operations on cached velocity locals
        const linVelA = vec3.fromValues(0.5, -1, 0.3);
        const angVelA = vec3.fromValues(0.1, -0.05, 0.2);
        const linVelB = vec3.fromValues(-0.3, 0.2, 0.1);
        const angVelB = vec3.fromValues(-0.02, 0.1, -0.05);

        l1 = contactConstraintPart.getTotalLambda(tan1Part1Local, linVelA, angVelA, linVelB, angVelB, true, true, tangent1);
        contactConstraintPart.applyLambda(
            tan1Part1Local,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            invMassA,
            invMassB,
            tangent1,
            l1,
        );

        l2 = contactConstraintPart.getTotalLambda(tan1Part2Local, linVelA, angVelA, linVelB, angVelB, true, true, tangent2);
        contactConstraintPart.applyLambda(
            tan1Part2Local,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            invMassA,
            invMassB,
            tangent2,
            l2,
        );

        n1 = contactConstraintPart.getTotalLambda(normalPart1Local, linVelA, angVelA, linVelB, angVelB, true, true, normal);
        n1 = Math.max(0, n1);
        contactConstraintPart.applyLambda(
            normalPart1Local,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            invMassA,
            invMassB,
            normal,
            n1,
        );

        n2 = contactConstraintPart.getTotalLambda(normalPart2Local, linVelA, angVelA, linVelB, angVelB, true, true, normal);
        n2 = Math.max(0, n2);
        contactConstraintPart.applyLambda(
            normalPart2Local,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            invMassA,
            invMassB,
            normal,
            n2,
        );

        // compare final velocities (reference path has DOF masking per-call, but DOF_ALL = no-op)
        expect(linVelA[0]).toBeCloseTo(bodyARef.motionProperties.linearVelocity[0], 10);
        expect(linVelA[1]).toBeCloseTo(bodyARef.motionProperties.linearVelocity[1], 10);
        expect(linVelA[2]).toBeCloseTo(bodyARef.motionProperties.linearVelocity[2], 10);
        expect(angVelA[0]).toBeCloseTo(bodyARef.motionProperties.angularVelocity[0], 10);
        expect(angVelA[1]).toBeCloseTo(bodyARef.motionProperties.angularVelocity[1], 10);
        expect(angVelA[2]).toBeCloseTo(bodyARef.motionProperties.angularVelocity[2], 10);

        expect(linVelB[0]).toBeCloseTo(bodyBRef.motionProperties.linearVelocity[0], 10);
        expect(linVelB[1]).toBeCloseTo(bodyBRef.motionProperties.linearVelocity[1], 10);
        expect(linVelB[2]).toBeCloseTo(bodyBRef.motionProperties.linearVelocity[2], 10);
        expect(angVelB[0]).toBeCloseTo(bodyBRef.motionProperties.angularVelocity[0], 10);
        expect(angVelB[1]).toBeCloseTo(bodyBRef.motionProperties.angularVelocity[1], 10);
        expect(angVelB[2]).toBeCloseTo(bodyBRef.motionProperties.angularVelocity[2], 10);
    });

    test('applyLambda with zero delta returns false', () => {
        const axis = vec3.fromValues(0, 1, 0);
        const partLocal = axisConstraintPart.create();
        setupConstraintPart(partLocal, { ...CONSTRAINT_SETUP, totalLambda: 5.0 });

        const linVelA = vec3.create();
        const angVelA = vec3.create();
        const linVelB = vec3.create();
        const angVelB = vec3.create();

        // applying the same totalLambda should produce no change
        const result = contactConstraintPart.applyLambda(
            partLocal,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            1.0,
            1.0,
            axis,
            5.0,
        );
        expect(result).toBe(false);
    });

    test('warmStart with zero totalLambda returns false', () => {
        const axis = vec3.fromValues(0, 1, 0);
        const partLocal = axisConstraintPart.create();
        setupConstraintPart(partLocal, { ...CONSTRAINT_SETUP, totalLambda: 0 });

        const linVelA = vec3.create();
        const angVelA = vec3.create();
        const linVelB = vec3.create();
        const angVelB = vec3.create();

        const result = contactConstraintPart.warmStart(
            partLocal,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            true,
            1.0,
            1.0,
            axis,
            0.95,
        );
        expect(result).toBe(false);
        // velocities should remain unchanged
        expect(linVelA[0]).toBe(0);
        expect(linVelA[1]).toBe(0);
        expect(linVelA[2]).toBe(0);
    });

    test('only dynamic body velocities are mutated by applyLambda', () => {
        const axis = vec3.fromValues(0, 1, 0);
        const partLocal = axisConstraintPart.create();
        setupConstraintPart(partLocal, { ...CONSTRAINT_SETUP, totalLambda: 1.0 });

        // body A is dynamic, body B is kinematic (not dynamic)
        const linVelA = vec3.fromValues(1, 2, 3);
        const angVelA = vec3.fromValues(0.1, 0.2, 0.3);
        const linVelB = vec3.fromValues(0.5, 0.5, 0.5);
        const angVelB = vec3.fromValues(0, 0, 0);

        contactConstraintPart.applyLambda(
            partLocal,
            linVelA,
            angVelA,
            linVelB,
            angVelB,
            true,
            false, // isDynamicA=true, isDynamicB=false
            1.0,
            0,
            axis,
            3.0,
        );

        // body B should be unchanged
        expect(linVelB[0]).toBe(0.5);
        expect(linVelB[1]).toBe(0.5);
        expect(linVelB[2]).toBe(0.5);
        expect(angVelB[0]).toBe(0);
        expect(angVelB[1]).toBe(0);
        expect(angVelB[2]).toBe(0);

        // body A should have changed
        expect(linVelA[1]).not.toBe(2);
    });
});
