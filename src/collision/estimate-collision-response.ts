import { type Mat4, mat4, type Vec3, vec3 } from 'math';
import { getInverseInertiaForRotation } from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
import type { RigidBody } from '../body/rigid-body';
import type { ContactManifold } from '../manifold/manifold';
import { MIN_NORMAL } from '../utils/float';

/**
 * Result of estimating collision response between two bodies.
 *
 * Contains predicted post-collision velocities, per-point normal (contact) impulses,
 * and the manifold-level friction impulses (matching the per-manifold friction model
 * used by the real solver).
 */
export type CollisionEstimationResult = {
    /** predicted post-collision linear velocity of body 1 */
    linearVelocity1: Vec3;

    /** predicted post-collision angular velocity of body 1 */
    angularVelocity1: Vec3;

    /** predicted post-collision linear velocity of body 2 */
    linearVelocity2: Vec3;

    /** predicted post-collision angular velocity of body 2 */
    angularVelocity2: Vec3;

    /**
     * Where friction acts: halfway between the two bodies' friction points (each the mean of that body's
     * contact points), relative to `manifold.baseOffset` like the manifold's relativeContactPoints arrays.
     */
    frictionPoint: Vec3;

    /** per-contact-point normal impulse (kg⋅m/s) */
    contactImpulse: number[];

    /** manifold-level linear friction impulse (kg⋅m/s), in the contact plane */
    frictionImpulse: Vec3;

    /** manifold-level angular friction impulse around the contact normal */
    angularFrictionImpulse: number;

    /** number of contact points (length of contactImpulse) */
    numImpulses: number;
};

/** create a new collision estimation result */
export function createCollisionEstimationResult(): CollisionEstimationResult {
    const contactImpulse: number[] = [];
    for (let i = 0; i < 64; i++) contactImpulse.push(0);

    return {
        linearVelocity1: vec3.create(),
        angularVelocity1: vec3.create(),
        linearVelocity2: vec3.create(),
        angularVelocity2: vec3.create(),
        frictionPoint: vec3.create(),
        contactImpulse,
        frictionImpulse: vec3.create(),
        angularFrictionImpulse: 0,
        numImpulses: 0,
    };
}

/** lightweight axis constraint for collision estimation (mirrors AxisConstraintPart) */
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

/** lightweight angular-only constraint for the angular friction part */
type MiniAngularConstraint = {
    invI1_Axis: Vec3;
    invI2_Axis: Vec3;
    effectiveMass: number;
    bias: number;
};

function createMiniAngularConstraint(): MiniAngularConstraint {
    return {
        invI1_Axis: vec3.create(),
        invI2_Axis: vec3.create(),
        effectiveMass: 0,
        bias: 0,
    };
}

const _ecr_r1CrossAxis = /* @__PURE__ */ vec3.create();
const _ecr_r2CrossAxis = /* @__PURE__ */ vec3.create();

/** initialize a mini axis constraint */
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
    vec3.cross(_ecr_r1CrossAxis, r1, axis);
    vec3.cross(_ecr_r2CrossAxis, r2, axis);

    vec3.copy(constraint.r1PlusUxAxis, _ecr_r1CrossAxis);
    vec3.copy(constraint.r2xAxis, _ecr_r2CrossAxis);

    mat4.multiply3x3Vec(constraint.invI1_r1PlusUxAxis, invInertia1, _ecr_r1CrossAxis);
    mat4.multiply3x3Vec(constraint.invI2_r2xAxis, invInertia2, _ecr_r2CrossAxis);

    let invEffectiveMass = invMass1 + invMass2;
    invEffectiveMass += vec3.dot(_ecr_r1CrossAxis, constraint.invI1_r1PlusUxAxis);
    invEffectiveMass += vec3.dot(_ecr_r2CrossAxis, constraint.invI2_r2xAxis);

    constraint.effectiveMass = invEffectiveMass > 0 ? 1 / invEffectiveMass : 0;
}

/** initialize the mini angular friction constraint (axis is the contact normal) */
function initAngularConstraint(constraint: MiniAngularConstraint, invInertia1: Mat4, invInertia2: Mat4, axis: Vec3): void {
    mat4.multiply3x3Vec(constraint.invI1_Axis, invInertia1, axis);
    mat4.multiply3x3Vec(constraint.invI2_Axis, invInertia2, axis);

    const invEffectiveMass = vec3.dot(axis, constraint.invI1_Axis) + vec3.dot(axis, constraint.invI2_Axis);
    constraint.effectiveMass = invEffectiveMass > 0 ? 1 / invEffectiveMass : 0;
}

/** compute relative velocity along constraint axis: the linear and angular terms summed per component, reduced once */
function getRelativeVelocity(
    constraint: MiniAxisConstraint,
    linVel1: Vec3,
    angVel1: Vec3,
    linVel2: Vec3,
    angVel2: Vec3,
    axis: Vec3,
): number {
    const r1 = constraint.r1PlusUxAxis;
    const r2 = constraint.r2xAxis;
    const x = axis[0] * (linVel1[0] - linVel2[0]) + r1[0] * angVel1[0] - r2[0] * angVel2[0];
    const y = axis[1] * (linVel1[1] - linVel2[1]) + r1[1] * angVel1[1] - r2[1] * angVel2[1];
    const z = axis[2] * (linVel1[2] - linVel2[2]) + r1[2] * angVel1[2] - r2[2] * angVel2[2];
    return x + y + z;
}

/** solve and get the unclamped lambda increment */
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

/** apply a delta lambda to the velocities */
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
    vec3.scaleAndAdd(linVel1, linVel1, axis, -lambda * invMass1);
    vec3.scaleAndAdd(angVel1, angVel1, constraint.invI1_r1PlusUxAxis, -lambda);
    vec3.scaleAndAdd(linVel2, linVel2, axis, lambda * invMass2);
    vec3.scaleAndAdd(angVel2, angVel2, constraint.invI2_r2xAxis, lambda);
}

/** solve normal constraint with clamping, returning the new total lambda */
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

/** angular friction: get unclamped lambda increment (jv = axis · (ω1 - ω2)) */
function solveGetAngularLambda(constraint: MiniAngularConstraint, angVel1: Vec3, angVel2: Vec3, axis: Vec3): number {
    const jv = vec3.dot(axis, angVel1) - vec3.dot(axis, angVel2);
    return constraint.effectiveMass * (jv - constraint.bias);
}

/** angular friction: apply delta lambda (no linear terms) */
function applyAngularLambda(constraint: MiniAngularConstraint, lambda: number, angVel1: Vec3, angVel2: Vec3): void {
    vec3.scaleAndAdd(angVel1, angVel1, constraint.invI1_Axis, -lambda);
    vec3.scaleAndAdd(angVel2, angVel2, constraint.invI2_Axis, lambda);
}

const _ecr_com1 = /* @__PURE__ */ vec3.create();
const _ecr_com2 = /* @__PURE__ */ vec3.create();
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
const _ecr_distanceToFrictionCenter: number[] = [];

for (let i = 0; i < 64; i++) {
    _ecr_normalConstraints.push(createMiniConstraint());
    _ecr_distanceToFrictionCenter.push(0);
}

const _ecr_tangent1 = /* @__PURE__ */ vec3.create();
const _ecr_tangent2 = /* @__PURE__ */ vec3.create();
const _ecr_frictionConstraint1: MiniAxisConstraint = /* @__PURE__ */ createMiniConstraint();
const _ecr_frictionConstraint2: MiniAxisConstraint = /* @__PURE__ */ createMiniConstraint();
const _ecr_angularFrictionConstraint: MiniAngularConstraint = /* @__PURE__ */ createMiniAngularConstraint();

/**
 * estimate collision response between two bodies
 *
 * predicts post-collision velocities and impulses by running a mini PGS solver
 * on a local copy of the velocities. designed to be called from onContactAdded
 * to estimate impact strength before the actual solver runs.
 *
 * @param result result object to write to
 * @param body1 first body
 * @param body2 second body
 * @param manifold contact manifold
 * @param combinedFriction combined friction coefficient
 * @param combinedRestitution combined restitution coefficient
 * @param minVelocityForRestitution minimum relative velocity to apply restitution (default 1.0 m/s)
 * @param numIterations number of PGS iterations (default 10)
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

    // zero impulses
    for (let i = 0; i < numContactPoints; i++) result.contactImpulse[i] = 0;
    vec3.zero(result.frictionImpulse);
    result.angularFrictionImpulse = 0;

    // friction basis: tangent1 perpendicular to normal, tangent2 = normal × tangent1
    const normal = manifold.worldSpaceNormal;
    const tangent1 = _ecr_tangent1;
    const tangent2 = _ecr_tangent2;
    vec3.perpendicular(tangent1, normal);
    vec3.normalize(tangent1, tangent1);
    vec3.cross(tangent2, normal, tangent1);

    // copy body velocities (zero only for static)
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

    // inverse mass / inertia: only dynamic bodies contribute
    const isDynamic1 = body1.motionType === MotionType.DYNAMIC;
    const isDynamic2 = body2.motionType === MotionType.DYNAMIC;
    const invMass1 = isDynamic1 && body1.motionProperties ? body1.motionProperties.invMass : 0;
    const invMass2 = isDynamic2 && body2.motionProperties ? body2.motionProperties.invMass : 0;

    if (isDynamic1 && body1.motionProperties) {
        mat4.fromQuat(_ecr_rotation1, body1.quaternion);
        getInverseInertiaForRotation(_ecr_invInertia1, body1.motionProperties, _ecr_rotation1);
    } else {
        mat4.zero(_ecr_invInertia1);
    }

    if (isDynamic2 && body2.motionProperties) {
        mat4.fromQuat(_ecr_rotation2, body2.quaternion);
        getInverseInertiaForRotation(_ecr_invInertia2, body2.motionProperties, _ecr_rotation2);
    } else {
        mat4.zero(_ecr_invInertia2);
    }

    // COM relative to manifold.baseOffset
    vec3.sub(_ecr_com1, body1.centerOfMassPosition, manifold.baseOffset);
    vec3.sub(_ecr_com2, body2.centerOfMassPosition, manifold.baseOffset);

    // per-contact-point setup: each body's own contact point gives its moment arm
    const relativePointsA = manifold.relativeContactPointsOnA;
    const relativePointsB = manifold.relativeContactPointsOnB;

    // each body's friction point accumulates its own contact points, so the friction lever arms are not perturbed by
    // the separation along the normal of a speculative contact
    let fpax = 0;
    let fpay = 0;
    let fpaz = 0;
    let fpbx = 0;
    let fpby = 0;
    let fpbz = 0;

    for (let c = 0; c < numContactPoints; c++) {
        const i = c * 3;
        const ax = relativePointsA[i];
        const ay = relativePointsA[i + 1];
        const az = relativePointsA[i + 2];
        const bx = relativePointsB[i];
        const by = relativePointsB[i + 1];
        const bz = relativePointsB[i + 2];

        fpax += ax;
        fpay += ay;
        fpaz += az;
        fpbx += bx;
        fpby += by;
        fpbz += bz;

        vec3.set(_ecr_r1, ax - _ecr_com1[0], ay - _ecr_com1[1], az - _ecr_com1[2]);
        vec3.set(_ecr_r2, bx - _ecr_com2[0], by - _ecr_com2[1], bz - _ecr_com2[2]);

        const normalConstraint = _ecr_normalConstraints[c];
        initConstraint(normalConstraint, invMass1, invMass2, _ecr_invInertia1, _ecr_invInertia2, _ecr_r1, _ecr_r2, normal);

        normalConstraint.bias = 0;
        if (combinedRestitution > 0) {
            vec3.cross(_ecr_vel1AtPoint, result.angularVelocity1, _ecr_r1);
            vec3.add(_ecr_vel1AtPoint, result.linearVelocity1, _ecr_vel1AtPoint);
            vec3.cross(_ecr_vel2AtPoint, result.angularVelocity2, _ecr_r2);
            vec3.add(_ecr_vel2AtPoint, result.linearVelocity2, _ecr_vel2AtPoint);
            vec3.sub(_ecr_relVel, _ecr_vel2AtPoint, _ecr_vel1AtPoint);

            const vn = vec3.dot(_ecr_relVel, normal);
            if (vn < -minVelocityForRestitution) {
                normalConstraint.bias = combinedRestitution * vn;
            }
        }
    }

    if (numContactPoints === 0) return;

    // each body's friction point = the mean of its contact points; per-point moment arm to body 1's
    fpax /= numContactPoints;
    fpay /= numContactPoints;
    fpaz /= numContactPoints;
    fpbx /= numContactPoints;
    fpby /= numContactPoints;
    fpbz /= numContactPoints;
    vec3.set(result.frictionPoint, 0.5 * (fpax + fpbx), 0.5 * (fpay + fpby), 0.5 * (fpaz + fpbz));

    const nx = normal[0];
    const ny = normal[1];
    const nz = normal[2];

    for (let c = 0; c < numContactPoints; c++) {
        const i = c * 3;
        const dx = relativePointsA[i] - fpax;
        const dy = relativePointsA[i + 1] - fpay;
        const dz = relativePointsA[i + 2] - fpaz;
        const dotN = dx * nx + dy * ny + dz * nz;
        const tx = dx - dotN * nx;
        const ty = dy - dotN * ny;
        const tz = dz - dotN * nz;
        _ecr_distanceToFrictionCenter[c] = Math.sqrt(tx * tx + ty * ty + tz * tz);
    }

    // manifold-level friction constraints anchored at each body's friction point
    if (combinedFriction > 0) {
        vec3.set(_ecr_r1, fpax - _ecr_com1[0], fpay - _ecr_com1[1], fpaz - _ecr_com1[2]);
        vec3.set(_ecr_r2, fpbx - _ecr_com2[0], fpby - _ecr_com2[1], fpbz - _ecr_com2[2]);

        initConstraint(
            _ecr_frictionConstraint1,
            invMass1,
            invMass2,
            _ecr_invInertia1,
            _ecr_invInertia2,
            _ecr_r1,
            _ecr_r2,
            tangent1,
        );
        _ecr_frictionConstraint1.bias = 0;

        initConstraint(
            _ecr_frictionConstraint2,
            invMass1,
            invMass2,
            _ecr_invInertia1,
            _ecr_invInertia2,
            _ecr_r1,
            _ecr_r2,
            tangent2,
        );
        _ecr_frictionConstraint2.bias = 0;

        if (numContactPoints > 1) {
            initAngularConstraint(_ecr_angularFrictionConstraint, _ecr_invInertia1, _ecr_invInertia2, normal);
            _ecr_angularFrictionConstraint.bias = 0;
        } else {
            _ecr_angularFrictionConstraint.effectiveMass = 0;
        }
    }

    // iterative PGS solve. Caps are derived from the previous iteration's accumulated
    // contact impulse (matches the real solver — no special "skip iter 0" branch).
    let frictionLambda1 = 0;
    let frictionLambda2 = 0;
    const iterations = numContactPoints === 1 && combinedFriction === 0 ? 1 : numIterations;

    for (let iter = 0; iter < iterations; iter++) {
        if (combinedFriction > 0) {
            // friction caps from current contact impulses
            let sumNormal = 0;
            let sumDistanceWeighted = 0;
            for (let c = 0; c < numContactPoints; c++) {
                const ln = result.contactImpulse[c];
                sumNormal += ln;
                sumDistanceWeighted += _ecr_distanceToFrictionCenter[c] * ln;
            }

            // joint cone clamp on the two linear friction parts
            let lambda1 =
                frictionLambda1 +
                solveGetLambda(
                    _ecr_frictionConstraint1,
                    result.linearVelocity1,
                    result.angularVelocity1,
                    result.linearVelocity2,
                    result.angularVelocity2,
                    tangent1,
                );
            let lambda2 =
                frictionLambda2 +
                solveGetLambda(
                    _ecr_frictionConstraint2,
                    result.linearVelocity1,
                    result.angularVelocity1,
                    result.linearVelocity2,
                    result.angularVelocity2,
                    tangent2,
                );

            const maxLinear = combinedFriction * sumNormal;
            const magSq = lambda1 * lambda1 + lambda2 * lambda2;
            // the + MIN_NORMAL keeps a magSq too small to divide by from being scaled
            if (magSq > maxLinear * maxLinear + MIN_NORMAL) {
                const scale = maxLinear / Math.sqrt(magSq);
                lambda1 *= scale;
                lambda2 *= scale;
            }

            applyLambda(
                _ecr_frictionConstraint1,
                lambda1 - frictionLambda1,
                invMass1,
                invMass2,
                result.linearVelocity1,
                result.angularVelocity1,
                result.linearVelocity2,
                result.angularVelocity2,
                tangent1,
            );
            applyLambda(
                _ecr_frictionConstraint2,
                lambda2 - frictionLambda2,
                invMass1,
                invMass2,
                result.linearVelocity1,
                result.angularVelocity1,
                result.linearVelocity2,
                result.angularVelocity2,
                tangent2,
            );
            frictionLambda1 = lambda1;
            frictionLambda2 = lambda2;

            // angular friction with symmetric clamp
            if (_ecr_angularFrictionConstraint.effectiveMass !== 0) {
                const lambdaAngularInc = solveGetAngularLambda(
                    _ecr_angularFrictionConstraint,
                    result.angularVelocity1,
                    result.angularVelocity2,
                    normal,
                );
                const maxAngular = combinedFriction * sumDistanceWeighted;
                const candidate = result.angularFrictionImpulse + lambdaAngularInc;
                const clamped = Math.max(-maxAngular, Math.min(maxAngular, candidate));
                const deltaAngular = clamped - result.angularFrictionImpulse;
                if (deltaAngular !== 0) {
                    applyAngularLambda(
                        _ecr_angularFrictionConstraint,
                        deltaAngular,
                        result.angularVelocity1,
                        result.angularVelocity2,
                    );
                }
                result.angularFrictionImpulse = clamped;
            }
        }

        // normal pass
        for (let c = 0; c < numContactPoints; c++) {
            const normalConstraint = _ecr_normalConstraints[c];
            result.contactImpulse[c] = solve(
                normalConstraint,
                result.contactImpulse[c],
                0,
                Number.MAX_VALUE,
                invMass1,
                invMass2,
                result.linearVelocity1,
                result.angularVelocity1,
                result.linearVelocity2,
                result.angularVelocity2,
                normal,
            );
        }
    }

    vec3.scale(result.frictionImpulse, tangent1, frictionLambda1);
    vec3.scaleAndAdd(result.frictionImpulse, result.frictionImpulse, tangent2, frictionLambda2);
}
