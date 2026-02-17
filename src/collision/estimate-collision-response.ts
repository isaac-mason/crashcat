import { type Mat4, mat4, type Vec3, vec3 } from 'mathcat';
import { getInverseInertiaForRotation } from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
import type { RigidBody } from '../body/rigid-body';
import type { ContactManifold } from '../manifold/manifold';

/** impulse data for a single contact point in the collision estimation */
export type CollisionEstimationImpulse = {
    /** normal impulse (kg⋅m/s) */
    contactImpulse: number;

    /** friction impulse along tangent1 direction */
    frictionImpulse1: number;

    /** friction impulse along tangent2 direction */
    frictionImpulse2: number;
};

/** result of estimating collision response between two bodies, contains predicted post-collision velocities and per-contact-point impulses */
export type CollisionEstimationResult = {
    /** predicted post-collision linear velocity of body 1 */
    linearVelocity1: Vec3;

    /** predicted post-collision angular velocity of body 1 */
    angularVelocity1: Vec3;

    /** predicted post-collision linear velocity of body 2 */
    linearVelocity2: Vec3;

    /** predicted post-collision angular velocity of body 2 */
    angularVelocity2: Vec3;

    /** first friction tangent direction (perpendicular to normal) */
    tangent1: Vec3;

    /** second friction tangent direction (tangent1 × normal) */
    tangent2: Vec3;

    /** impulses for each contact point */
    impulses: CollisionEstimationImpulse[];

    /** number of active impulses (matches manifold.numContactPoints) */
    numImpulses: number;
};

/** create a new collision estimation result */
export function createCollisionEstimationResult(): CollisionEstimationResult {
    const impulses: CollisionEstimationImpulse[] = [];

    for (let i = 0; i < 64; i++) {
        impulses.push({
            contactImpulse: 0,
            frictionImpulse1: 0,
            frictionImpulse2: 0,
        });
    }

    return {
        linearVelocity1: vec3.create(),
        angularVelocity1: vec3.create(),
        linearVelocity2: vec3.create(),
        angularVelocity2: vec3.create(),
        tangent1: vec3.create(),
        tangent2: vec3.create(),
        impulses,
        numImpulses: 0,
    };
}
/**
 * lightweight axis constraint for collision estimation
 *
 * mirrors AxisConstraintPart but operates on local velocity copies
 */
type MiniAxisConstraint = {
    r1PlusUxAxis: Vec3;
    r2xAxis: Vec3;
    invI1_r1PlusUxAxis: Vec3;
    invI2_r2xAxis: Vec3;
    effectiveMass: number;
    bias: number;
};

function createMiniConstraint(): MiniAxisConstraint {
    return {
        r1PlusUxAxis: vec3.create(),
        r2xAxis: vec3.create(),
        invI1_r1PlusUxAxis: vec3.create(),
        invI2_r2xAxis: vec3.create(),
        effectiveMass: 0,
        bias: 0,
    };
}

const _ecr_r1CrossAxis = /* @__PURE__ */ vec3.create();
const _ecr_r2CrossAxis = /* @__PURE__ */ vec3.create();
const _ecr_temp = /* @__PURE__ */ vec3.create();

/**
 * initialize a mini axis constraint
 */
function initConstraint(
    constraint: MiniAxisConstraint,
    invMass1: number,
    invMass2: number,
    invInertia1: Mat4,
    invInertia2: Mat4,
    r1: Vec3,
    r2: Vec3,
    axis: Vec3,
): void {
    // compute r1 × axis and r2 × axis
    vec3.cross(_ecr_r1CrossAxis, r1, axis);
    vec3.cross(_ecr_r2CrossAxis, r2, axis);

    vec3.copy(constraint.r1PlusUxAxis, _ecr_r1CrossAxis);
    vec3.copy(constraint.r2xAxis, _ecr_r2CrossAxis);

    // invI1 × (r1 × axis)
    mat4.multiply3x3Vec(constraint.invI1_r1PlusUxAxis, invInertia1, _ecr_r1CrossAxis);

    // invI2 × (r2 × axis)
    mat4.multiply3x3Vec(constraint.invI2_r2xAxis, invInertia2, _ecr_r2CrossAxis);

    // compute effective mass: K = invM1 + (r1×axis)ᵀ I1⁻¹ (r1×axis) + invM2 + (r2×axis)ᵀ I2⁻¹ (r2×axis)
    let invEffectiveMass = invMass1 + invMass2;
    invEffectiveMass += vec3.dot(_ecr_r1CrossAxis, constraint.invI1_r1PlusUxAxis);
    invEffectiveMass += vec3.dot(_ecr_r2CrossAxis, constraint.invI2_r2xAxis);

    constraint.effectiveMass = invEffectiveMass > 0 ? 1 / invEffectiveMass : 0;
}

/** compute relative velocity along constraint axis */
function getRelativeVelocity(
    constraint: MiniAxisConstraint,
    linVel1: Vec3,
    angVel1: Vec3,
    linVel2: Vec3,
    angVel2: Vec3,
    axis: Vec3,
): number {
    // Jv = axis · (v1 - v2) + (r1×axis) · ω1 - (r2×axis) · ω2
    // this is the velocity of body 1 relative to body 2 along the constraint axis
    const linearComponent = vec3.dot(axis, vec3.sub(_ecr_temp, linVel1, linVel2));
    const angularComponent1 = vec3.dot(constraint.r1PlusUxAxis, angVel1);
    const angularComponent2 = vec3.dot(constraint.r2xAxis, angVel2);

    return linearComponent + angularComponent1 - angularComponent2;
}

/** solve and get lambda (impulse magnitude) */
function solveGetLambda(
    constraint: MiniAxisConstraint,
    linVel1: Vec3,
    angVel1: Vec3,
    linVel2: Vec3,
    angVel2: Vec3,
    axis: Vec3,
): number {
    const jv = getRelativeVelocity(constraint, linVel1, angVel1, linVel2, angVel2, axis);
    return constraint.effectiveMass * (jv - constraint.bias);
}

/** apply lambda to velocities */
function applyLambda(
    constraint: MiniAxisConstraint,
    lambda: number,
    invMass1: number,
    invMass2: number,
    linVel1: Vec3,
    angVel1: Vec3,
    linVel2: Vec3,
    angVel2: Vec3,
    axis: Vec3,
): void {
    // v1' = v1 - (lambda / m1) * axis
    vec3.scaleAndAdd(linVel1, linVel1, axis, -lambda * invMass1);

    // ω1' = ω1 - lambda * I1⁻¹(r1 × axis)
    vec3.scaleAndAdd(angVel1, angVel1, constraint.invI1_r1PlusUxAxis, -lambda);

    // v2' = v2 + (lambda / m2) * axis
    vec3.scaleAndAdd(linVel2, linVel2, axis, lambda * invMass2);

    // ω2' = ω2 + lambda * I2⁻¹(r2 × axis)
    vec3.scaleAndAdd(angVel2, angVel2, constraint.invI2_r2xAxis, lambda);
}

/** solve constraint with clamping, returns the new total lambda */
function solve(
    constraint: MiniAxisConstraint,
    currentTotalLambda: number,
    minLambda: number,
    maxLambda: number,
    invMass1: number,
    invMass2: number,
    linVel1: Vec3,
    angVel1: Vec3,
    linVel2: Vec3,
    angVel2: Vec3,
    axis: Vec3,
): number {
    const lambda = solveGetLambda(constraint, linVel1, angVel1, linVel2, angVel2, axis);
    const newTotalLambda = Math.max(minLambda, Math.min(maxLambda, currentTotalLambda + lambda));
    const deltaLambda = newTotalLambda - currentTotalLambda;

    if (deltaLambda !== 0) {
        applyLambda(constraint, deltaLambda, invMass1, invMass2, linVel1, angVel1, linVel2, angVel2, axis);
    }

    return newTotalLambda;
}

const _ecr_com1 = /* @__PURE__ */ vec3.create();
const _ecr_com2 = /* @__PURE__ */ vec3.create();
const _ecr_contactPoint = /* @__PURE__ */ vec3.create();
const _ecr_r1 = /* @__PURE__ */ vec3.create();
const _ecr_r2 = /* @__PURE__ */ vec3.create();
const _ecr_relVel = /* @__PURE__ */ vec3.create();
const _ecr_vel1AtPoint = /* @__PURE__ */ vec3.create();
const _ecr_vel2AtPoint = /* @__PURE__ */ vec3.create();
const _ecr_invInertia1 = /* @__PURE__ */ mat4.create();
const _ecr_invInertia2 = /* @__PURE__ */ mat4.create();
const _ecr_rotation1 = /* @__PURE__ */ mat4.create();
const _ecr_rotation2 = /* @__PURE__ */ mat4.create();

const _ecr_normalConstraints: MiniAxisConstraint[] = [];
const _ecr_frictionConstraints1: MiniAxisConstraint[] = [];
const _ecr_frictionConstraints2: MiniAxisConstraint[] = [];

for (let i = 0; i < 64; i++) {
    _ecr_normalConstraints.push(createMiniConstraint());
    _ecr_frictionConstraints1.push(createMiniConstraint());
    _ecr_frictionConstraints2.push(createMiniConstraint());
}

/**
 * estimate collision response between two bodies
 *
 * predicts post-collision velocities and impulses by running a mini PGS solver
 * on a local copy of the velocities. designed to be called from onContactAdded
 * to estimate impact strength before the actual solver runs.
 *
 * @param result - output structure to fill with predicted velocities and impulses
 * @param body1 - first body
 * @param body2 - second body
 * @param manifold - contact manifold
 * @param combinedFriction - combined friction coefficient
 * @param combinedRestitution - combined restitution coefficient
 * @param minVelocityForRestitution - minimum relative velocity to apply restitution (default 1.0 m/s)
 * @param numIterations - number of PGS iterations (default 10)
 */
export function estimateCollisionResponse(
    result: CollisionEstimationResult,
    body1: RigidBody,
    body2: RigidBody,
    manifold: ContactManifold,
    combinedFriction: number,
    combinedRestitution: number,
    minVelocityForRestitution = 1.0,
    numIterations = 10,
): void {
    const numContactPoints = manifold.numContactPoints;
    result.numImpulses = numContactPoints;

    // phase 1: setup

    // zero all impulses
    for (let i = 0; i < numContactPoints; i++) {
        result.impulses[i].contactImpulse = 0;
        result.impulses[i].frictionImpulse1 = 0;
        result.impulses[i].frictionImpulse2 = 0;
    }

    // compute friction basis: tangent1 = perpendicular to normal, tangent2 = normal × tangent1
    const normal = manifold.worldSpaceNormal;
    vec3.perpendicular(result.tangent1, normal);
    vec3.normalize(result.tangent1, result.tangent1);
    vec3.cross(result.tangent2, normal, result.tangent1);

    // copy body velocities (zero only for static, preserve for dynamic/kinematic)
    const isStatic1 = body1.motionType === MotionType.STATIC;
    const isStatic2 = body2.motionType === MotionType.STATIC;

    if (!isStatic1 && body1.motionProperties) {
        vec3.copy(result.linearVelocity1, body1.motionProperties.linearVelocity);
        vec3.copy(result.angularVelocity1, body1.motionProperties.angularVelocity);
    } else {
        vec3.set(result.linearVelocity1, 0, 0, 0);
        vec3.set(result.angularVelocity1, 0, 0, 0);
    }

    if (!isStatic2 && body2.motionProperties) {
        vec3.copy(result.linearVelocity2, body2.motionProperties.linearVelocity);
        vec3.copy(result.angularVelocity2, body2.motionProperties.angularVelocity);
    } else {
        vec3.set(result.linearVelocity2, 0, 0, 0);
        vec3.set(result.angularVelocity2, 0, 0, 0);
    }

    // get inverse mass and inertia ONLY for dynamic bodies
    // kinematic bodies have zero invMass/invInertia in constraint solving
    const isDynamic1 = body1.motionType === MotionType.DYNAMIC;
    const isDynamic2 = body2.motionType === MotionType.DYNAMIC;

    const invMass1 = isDynamic1 && body1.motionProperties ? body1.motionProperties.invMass : 0;
    const invMass2 = isDynamic2 && body2.motionProperties ? body2.motionProperties.invMass : 0;

    if (isDynamic1 && body1.motionProperties) {
        mat4.fromQuat(_ecr_rotation1, body1.quaternion);
        getInverseInertiaForRotation(_ecr_invInertia1, body1.motionProperties, _ecr_rotation1);
    } else {
        mat4.zero(_ecr_invInertia1); // static/kinematic bodies have zero inverse inertia
    }

    if (isDynamic2 && body2.motionProperties) {
        mat4.fromQuat(_ecr_rotation2, body2.quaternion);
        getInverseInertiaForRotation(_ecr_invInertia2, body2.motionProperties, _ecr_rotation2);
    } else {
        mat4.zero(_ecr_invInertia2); // static/kinematic bodies have zero inverse inertia
    }

    // compute COM relative to manifold.baseOffset
    vec3.sub(_ecr_com1, body1.centerOfMassPosition, manifold.baseOffset);
    vec3.sub(_ecr_com2, body2.centerOfMassPosition, manifold.baseOffset);

    // phase 3: per-contact-point initialization
    const relativePointsA = manifold.relativeContactPointsOnA;
    const relativePointsB = manifold.relativeContactPointsOnB;

    for (let c = 0; c < numContactPoints; c++) {
        const i = c * 3;

        // contact point = midpoint of pointOnA and pointOnB
        const ax = relativePointsA[i];
        const ay = relativePointsA[i + 1];
        const az = relativePointsA[i + 2];
        const bx = relativePointsB[i];
        const by = relativePointsB[i + 1];
        const bz = relativePointsB[i + 2];

        vec3.set(_ecr_contactPoint, (ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5);

        // lever arms
        vec3.sub(_ecr_r1, _ecr_contactPoint, _ecr_com1);
        vec3.sub(_ecr_r2, _ecr_contactPoint, _ecr_com2);

        // initialize normal constraint
        const normalConstraint = _ecr_normalConstraints[c];
        initConstraint(normalConstraint, invMass1, invMass2, _ecr_invInertia1, _ecr_invInertia2, _ecr_r1, _ecr_r2, normal);

        // restitution bias
        normalConstraint.bias = 0;
        if (combinedRestitution > 0) {
            // compute relative velocity at contact point
            vec3.cross(_ecr_vel1AtPoint, result.angularVelocity1, _ecr_r1);
            vec3.add(_ecr_vel1AtPoint, result.linearVelocity1, _ecr_vel1AtPoint);

            vec3.cross(_ecr_vel2AtPoint, result.angularVelocity2, _ecr_r2);
            vec3.add(_ecr_vel2AtPoint, result.linearVelocity2, _ecr_vel2AtPoint);

            // relative velocity of body 2 wrt body 1 at contact point
            vec3.sub(_ecr_relVel, _ecr_vel2AtPoint, _ecr_vel1AtPoint);

            const vn = vec3.dot(_ecr_relVel, normal);
            // note: vn is negative when bodies are approaching (separating velocity is positive)
            if (vn < -minVelocityForRestitution) {
                normalConstraint.bias = combinedRestitution * vn;
            }
        }

        // initialize friction constraints
        if (combinedFriction > 0) {
            const frictionConstraint1 = _ecr_frictionConstraints1[c];
            initConstraint(
                frictionConstraint1,
                invMass1,
                invMass2,
                _ecr_invInertia1,
                _ecr_invInertia2,
                _ecr_r1,
                _ecr_r2,
                result.tangent1,
            );
            frictionConstraint1.bias = 0;

            const frictionConstraint2 = _ecr_frictionConstraints2[c];
            initConstraint(
                frictionConstraint2,
                invMass1,
                invMass2,
                _ecr_invInertia1,
                _ecr_invInertia2,
                _ecr_r1,
                _ecr_r2,
                result.tangent2,
            );
            frictionConstraint2.bias = 0;
        }
    }

    // phase 4: iterative PGS solve

    // determine iteration count: if single point + no friction, 1 iteration suffices
    const iterations = numContactPoints === 1 && combinedFriction === 0 ? 1 : numIterations;

    for (let iter = 0; iter < iterations; iter++) {
        // friction pass (skip iteration 0 since normal impulse is still 0)
        if (iter > 0 && combinedFriction > 0) {
            for (let c = 0; c < numContactPoints; c++) {
                const frictionConstraint1 = _ecr_frictionConstraints1[c];
                const frictionConstraint2 = _ecr_frictionConstraints2[c];
                const impulse = result.impulses[c];

                // get candidate lambdas (total, not delta)
                let lambda1 =
                    impulse.frictionImpulse1 +
                    solveGetLambda(
                        frictionConstraint1,
                        result.linearVelocity1,
                        result.angularVelocity1,
                        result.linearVelocity2,
                        result.angularVelocity2,
                        result.tangent1,
                    );

                let lambda2 =
                    impulse.frictionImpulse2 +
                    solveGetLambda(
                        frictionConstraint2,
                        result.linearVelocity1,
                        result.angularVelocity1,
                        result.linearVelocity2,
                        result.angularVelocity2,
                        result.tangent2,
                    );

                // circular coulomb friction cone clamping
                const maxImpulse = combinedFriction * impulse.contactImpulse;
                const totalLambdaSquared = lambda1 * lambda1 + lambda2 * lambda2;

                if (totalLambdaSquared > maxImpulse * maxImpulse) {
                    const scale = maxImpulse / Math.sqrt(totalLambdaSquared);
                    lambda1 *= scale;
                    lambda2 *= scale;
                }

                // apply delta impulses
                applyLambda(
                    frictionConstraint1,
                    lambda1 - impulse.frictionImpulse1,
                    invMass1,
                    invMass2,
                    result.linearVelocity1,
                    result.angularVelocity1,
                    result.linearVelocity2,
                    result.angularVelocity2,
                    result.tangent1,
                );

                applyLambda(
                    frictionConstraint2,
                    lambda2 - impulse.frictionImpulse2,
                    invMass1,
                    invMass2,
                    result.linearVelocity1,
                    result.angularVelocity1,
                    result.linearVelocity2,
                    result.angularVelocity2,
                    result.tangent2,
                );

                impulse.frictionImpulse1 = lambda1;
                impulse.frictionImpulse2 = lambda2;
            }
        }

        // normal pass (always)
        for (let c = 0; c < numContactPoints; c++) {
            const normalConstraint = _ecr_normalConstraints[c];

            const normalImpulse = solve(
                normalConstraint,
                result.impulses[c].contactImpulse,
                0, // no pulling
                Number.MAX_VALUE, // no upper limit on pushing
                invMass1,
                invMass2,
                result.linearVelocity1,
                result.angularVelocity1,
                result.linearVelocity2,
                result.angularVelocity2,
                normal,
            );

            result.impulses[c].contactImpulse = normalImpulse;
        }
    }
}
