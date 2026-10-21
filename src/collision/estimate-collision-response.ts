import { type Mat4, mat4, type Vec3, vec3 } from 'math';
import { getInverseInertiaForRotation } from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
import type { RigidBody } from '../body/rigid-body';
import * as angularFrictionConstraintPart from '../constraints/constraint-part/angular-friction-constraint-part';
import * as axisConstraintPart from '../constraints/constraint-part/axis-constraint-part';
import type { ContactSettings } from '../constraints/contact-constraints';
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

const _ecr_com1 = /* @__PURE__ */ vec3.create();
const _ecr_com2 = /* @__PURE__ */ vec3.create();
const _ecr_r1 = /* @__PURE__ */ vec3.create();
const _ecr_r2 = /* @__PURE__ */ vec3.create();
const _ecr_vel1AtPoint = /* @__PURE__ */ vec3.create();
const _ecr_vel2AtPoint = /* @__PURE__ */ vec3.create();
const _ecr_surfaceVelocity = /* @__PURE__ */ vec3.create();
const _ecr_rotation = /* @__PURE__ */ mat4.create();
const _ecr_invInertia1 = /* @__PURE__ */ mat4.create();
const _ecr_invInertia2 = /* @__PURE__ */ mat4.create();
const _ecr_tangent1 = /* @__PURE__ */ vec3.create();
const _ecr_tangent2 = /* @__PURE__ */ vec3.create();

// the solver's own parts; each is reset before it is set up, as the setup skips the terms of a body that is not dynamic
const _ecr_contactConstraints: axisConstraintPart.AxisConstraintPart[] = [];
const _ecr_distanceToFrictionCenter: number[] = [];
for (let i = 0; i < 64; i++) {
    _ecr_contactConstraints.push(axisConstraintPart.create());
    _ecr_distanceToFrictionCenter.push(0);
}
const _ecr_friction1 = /* @__PURE__ */ axisConstraintPart.create();
const _ecr_friction2 = /* @__PURE__ */ axisConstraintPart.create();
const _ecr_angularFriction = /* @__PURE__ */ angularFrictionConstraintPart.create();

/** the body's world inverse inertia, scaled; zero unless it is dynamic */
function inverseInertiaOf(out: Mat4, body: RigidBody, scale: number): Mat4 {
    if (body.motionType !== MotionType.DYNAMIC) return mat4.zero(out);
    mat4.fromQuat(_ecr_rotation, body.quaternion);
    getInverseInertiaForRotation(out, body.motionProperties, _ecr_rotation);
    for (let i = 0; i < 16; i++) out[i] *= scale;
    return out;
}

/** the part's jacobian times the estimate's velocities */
function jacobianVelocity(part: axisConstraintPart.AxisConstraintPart, axis: Vec3, result: CollisionEstimationResult): number {
    const v1 = result.linearVelocity1;
    const w1 = result.angularVelocity1;
    const v2 = result.linearVelocity2;
    const w2 = result.angularVelocity2;
    const r1 = part.r1PlusUxAxis;
    const r2 = part.r2xAxis;
    return (
        axis[0] * (v1[0] - v2[0]) +
        axis[1] * (v1[1] - v2[1]) +
        axis[2] * (v1[2] - v2[2]) +
        r1[0] * w1[0] +
        r1[1] * w1[1] +
        r1[2] * w1[2] -
        r2[0] * w2[0] -
        r2[1] * w2[1] -
        r2[2] * w2[2]
    );
}

/** commits a new total lambda to the part and applies the change to the estimate's velocities */
function applyTotalLambda(
    part: axisConstraintPart.AxisConstraintPart,
    totalLambda: number,
    invMass1: number,
    invMass2: number,
    axis: Vec3,
    result: CollisionEstimationResult,
): void {
    const delta = axisConstraintPart.deltaLambdaFor(part, totalLambda);
    if (delta === 0) return;
    vec3.scaleAndAdd(result.linearVelocity1, result.linearVelocity1, axis, -delta * invMass1);
    vec3.scaleAndAdd(result.angularVelocity1, result.angularVelocity1, part.invI1_r1PlusUxAxis, -delta);
    vec3.scaleAndAdd(result.linearVelocity2, result.linearVelocity2, axis, delta * invMass2);
    vec3.scaleAndAdd(result.angularVelocity2, result.angularVelocity2, part.invI2_r2xAxis, delta);
}

/**
 * Estimate the contact and friction impulses of a collision, and the body velocities they leave, before the solver
 * runs: for the strength of an impact in onContactAdded, to play a sound or spawn particles. Accurate when two bodies
 * collide, not when more collide at once, as it knows only this contact.
 *
 * @param result result object to write to
 * @param body1 first body
 * @param body2 second body
 * @param manifold contact manifold
 * @param settings the contact's settings: its materials, mass and inertia scales and surface velocities (a sensor has no response)
 * @param minVelocityForRestitution minimum relative velocity to apply restitution (default 1.0 m/s)
 * @param numIterations number of PGS iterations (default 10); with 1, no friction is calculated
 */
export function estimateCollisionResponse(
    result: CollisionEstimationResult,
    body1: RigidBody,
    body2: RigidBody,
    manifold: ContactManifold,
    settings: ContactSettings,
    minVelocityForRestitution = 1.0,
    numIterations = 10,
): void {
    const numPoints = manifold.numContactPoints;
    const normal = manifold.worldSpaceNormal;

    // the friction directions, as the solver's
    const tangent1 = _ecr_tangent1;
    const tangent2 = _ecr_tangent2;
    const nx = normal[0];
    const ny = normal[1];
    const nz = normal[2];
    if (Math.abs(nx) > Math.abs(ny)) {
        const length = Math.sqrt(nx * nx + nz * nz);
        vec3.set(tangent1, nz / length, 0, -nx / length);
    } else {
        const length = Math.sqrt(ny * ny + nz * nz);
        vec3.set(tangent1, 0, nz / length, -ny / length);
    }
    vec3.cross(tangent2, normal, tangent1);

    // the bodies' velocities, zero for a static body
    if (body1.motionType !== MotionType.STATIC) {
        vec3.copy(result.linearVelocity1, body1.motionProperties.linearVelocity);
        vec3.copy(result.angularVelocity1, body1.motionProperties.angularVelocity);
    } else {
        vec3.zero(result.linearVelocity1);
        vec3.zero(result.angularVelocity1);
    }
    if (body2.motionType !== MotionType.STATIC) {
        vec3.copy(result.linearVelocity2, body2.motionProperties.linearVelocity);
        vec3.copy(result.angularVelocity2, body2.motionProperties.angularVelocity);
    } else {
        vec3.zero(result.linearVelocity2);
        vec3.zero(result.angularVelocity2);
    }

    result.numImpulses = numPoints;
    for (let c = 0; c < numPoints; c++) result.contactImpulse[c] = 0;
    vec3.zero(result.frictionPoint);
    vec3.zero(result.frictionImpulse);
    result.angularFrictionImpulse = 0;

    // sensors have no collision response, and a manifold without points nothing to respond to
    if (settings.isSensor || numPoints === 0) return;

    // the inverse masses and inertias, scaled for this contact
    const invMass1 = body1.motionType === MotionType.DYNAMIC ? settings.invMassScale1 * body1.motionProperties.invMass : 0;
    const invMass2 = body2.motionType === MotionType.DYNAMIC ? settings.invMassScale2 * body2.motionProperties.invMass : 0;
    const invInertia1 = inverseInertiaOf(_ecr_invInertia1, body1, settings.invInertiaScale1);
    const invInertia2 = inverseInertiaOf(_ecr_invInertia2, body2, settings.invInertiaScale2);

    // the centres of mass relative to the base offset
    vec3.subtract(_ecr_com1, body1.centerOfMassPosition, manifold.baseOffset);
    vec3.subtract(_ecr_com2, body2.centerOfMassPosition, manifold.baseOffset);

    // the non-penetration parts, and each body's friction point as the sum of its own contact points: per body, so the
    // friction lever arm is not perturbed by the separation along the normal of a speculative contact
    const pointsA = manifold.relativeContactPointsOnA;
    const pointsB = manifold.relativeContactPointsOnB;
    let fp1x = 0;
    let fp1y = 0;
    let fp1z = 0;
    let fp2x = 0;
    let fp2y = 0;
    let fp2z = 0;
    for (let c = 0; c < numPoints; c++) {
        const i = c * 3;
        fp1x += pointsA[i];
        fp1y += pointsA[i + 1];
        fp1z += pointsA[i + 2];
        fp2x += pointsB[i];
        fp2y += pointsB[i + 1];
        fp2z += pointsB[i + 2];

        // the contact point relative to each centre of mass
        vec3.set(_ecr_r1, pointsA[i] - _ecr_com1[0], pointsA[i + 1] - _ecr_com1[1], pointsA[i + 2] - _ecr_com1[2]);
        vec3.set(_ecr_r2, pointsB[i] - _ecr_com2[0], pointsB[i + 1] - _ecr_com2[1], pointsB[i + 2] - _ecr_com2[2]);

        // elastic collisions: restitution when the approach is fast enough
        let bias = 0;
        if (settings.combinedRestitution > 0) {
            vec3.cross(_ecr_vel1AtPoint, result.angularVelocity1, _ecr_r1);
            vec3.add(_ecr_vel1AtPoint, result.linearVelocity1, _ecr_vel1AtPoint);
            vec3.cross(_ecr_vel2AtPoint, result.angularVelocity2, _ecr_r2);
            vec3.add(_ecr_vel2AtPoint, result.linearVelocity2, _ecr_vel2AtPoint);
            const normalVelocity =
                (_ecr_vel2AtPoint[0] - _ecr_vel1AtPoint[0]) * nx +
                (_ecr_vel2AtPoint[1] - _ecr_vel1AtPoint[1]) * ny +
                (_ecr_vel2AtPoint[2] - _ecr_vel1AtPoint[2]) * nz;
            if (normalVelocity < -minVelocityForRestitution) bias = settings.combinedRestitution * normalVelocity;
        }

        const constraint = _ecr_contactConstraints[c];
        axisConstraintPart.resetAxisConstraintPart(constraint);
        axisConstraintPart.calculateConstraintProperties(
            constraint,
            body1,
            body2,
            invMass1,
            invMass2,
            invInertia1,
            invInertia2,
            _ecr_r1,
            _ecr_r2,
            normal,
            bias,
        );
    }

    // each point's distance to body 1's friction point in the contact plane
    fp1x /= numPoints;
    fp1y /= numPoints;
    fp1z /= numPoints;
    fp2x /= numPoints;
    fp2y /= numPoints;
    fp2z /= numPoints;
    for (let c = 0; c < numPoints; c++) {
        const i = c * 3;
        const dx = pointsA[i] - fp1x;
        const dy = pointsA[i + 1] - fp1y;
        const dz = pointsA[i + 2] - fp1z;
        const dotN = dx * nx + dy * ny + dz * nz;
        const tx = dx - dotN * nx;
        const ty = dy - dotN * ny;
        const tz = dz - dotN * nz;
        _ecr_distanceToFrictionCenter[c] = Math.sqrt(tx * tx + ty * ty + tz * tz);
    }
    vec3.set(result.frictionPoint, 0.5 * (fp1x + fp2x), 0.5 * (fp1y + fp2y), 0.5 * (fp1z + fp2z));

    // the friction parts, their bias the surface velocity along each tangent
    const friction1 = _ecr_friction1;
    const friction2 = _ecr_friction2;
    const angularFriction = _ecr_angularFriction;
    axisConstraintPart.resetAxisConstraintPart(friction1);
    axisConstraintPart.resetAxisConstraintPart(friction2);
    angularFrictionConstraintPart.reset(angularFriction);
    angularFriction.totalLambda = 0;
    if (settings.combinedFriction > 0) {
        vec3.set(_ecr_r1, fp1x - _ecr_com1[0], fp1y - _ecr_com1[1], fp1z - _ecr_com1[2]);
        vec3.set(_ecr_r2, fp2x - _ecr_com2[0], fp2y - _ecr_com2[1], fp2z - _ecr_com2[2]);
        const angularSurfaceVelocity = settings.relativeAngularSurfaceVelocity;
        vec3.set(_ecr_surfaceVelocity, fp1x, fp1y, fp1z);
        vec3.cross(_ecr_surfaceVelocity, angularSurfaceVelocity, _ecr_surfaceVelocity);
        vec3.add(_ecr_surfaceVelocity, settings.relativeLinearSurfaceVelocity, _ecr_surfaceVelocity);
        axisConstraintPart.calculateConstraintProperties(
            friction1,
            body1,
            body2,
            invMass1,
            invMass2,
            invInertia1,
            invInertia2,
            _ecr_r1,
            _ecr_r2,
            tangent1,
            vec3.dot(tangent1, _ecr_surfaceVelocity),
        );
        axisConstraintPart.calculateConstraintProperties(
            friction2,
            body1,
            body2,
            invMass1,
            invMass2,
            invInertia1,
            invInertia2,
            _ecr_r1,
            _ecr_r2,
            tangent2,
            vec3.dot(tangent2, _ecr_surfaceVelocity),
        );
        if (numPoints > 1) {
            angularFrictionConstraintPart.calculateConstraintProperties(
                angularFriction,
                body1,
                body2,
                invInertia1,
                invInertia2,
                normal,
                vec3.dot(angularSurfaceVelocity, normal),
            );
        }
    }

    // a single point without friction needs only one iteration
    const iterations = settings.combinedFriction <= 0 && numPoints === 1 ? 1 : numIterations;
    for (let iteration = 0; iteration < iterations; iteration++) {
        // friction first, capped by the non-penetration impulses so far
        if (settings.combinedFriction > 0) {
            let maxLinearLambda = 0;
            let maxAngularLambda = 0;
            for (let c = 0; c < numPoints; c++) {
                const lambda = _ecr_contactConstraints[c].totalLambda;
                maxLinearLambda += lambda;
                maxAngularLambda += _ecr_distanceToFrictionCenter[c] * lambda;
            }
            maxLinearLambda *= settings.combinedFriction;
            maxAngularLambda *= settings.combinedFriction;

            // the impulse to stop the motion along the tangents, scaled back onto the friction cone
            let lambda1 = axisConstraintPart.totalLambdaFor(friction1, jacobianVelocity(friction1, tangent1, result));
            let lambda2 = axisConstraintPart.totalLambdaFor(friction2, jacobianVelocity(friction2, tangent2, result));
            // the + MIN_NORMAL keeps a totalLambdaSq too small to divide by from being scaled
            const totalLambdaSq = lambda1 * lambda1 + lambda2 * lambda2;
            if (totalLambdaSq > maxLinearLambda * maxLinearLambda + MIN_NORMAL) {
                const scale = maxLinearLambda / Math.sqrt(totalLambdaSq);
                lambda1 *= scale;
                lambda2 *= scale;
            }
            applyTotalLambda(friction1, lambda1, invMass1, invMass2, tangent1, result);
            applyTotalLambda(friction2, lambda2, invMass1, invMass2, tangent2, result);

            if (numPoints > 1) {
                const w1 = result.angularVelocity1;
                const w2 = result.angularVelocity2;
                const jv = nx * (w1[0] - w2[0]) + ny * (w1[1] - w2[1]) + nz * (w1[2] - w2[2]);
                const unclamped = angularFrictionConstraintPart.totalLambdaFor(angularFriction, jv);
                const clamped = Math.max(-maxAngularLambda, Math.min(maxAngularLambda, unclamped));
                const delta = angularFrictionConstraintPart.deltaLambdaFor(angularFriction, clamped);
                if (delta !== 0) {
                    vec3.scaleAndAdd(w1, w1, angularFriction.invI1_Axis, -delta);
                    vec3.scaleAndAdd(w2, w2, angularFriction.invI2_Axis, delta);
                }
            }
        }

        // non-penetration last: contacts only push
        for (let c = 0; c < numPoints; c++) {
            const constraint = _ecr_contactConstraints[c];
            const totalLambda = Math.max(
                0,
                axisConstraintPart.totalLambdaFor(constraint, jacobianVelocity(constraint, normal, result)),
            );
            applyTotalLambda(constraint, totalLambda, invMass1, invMass2, normal, result);
        }
    }

    for (let c = 0; c < numPoints; c++) result.contactImpulse[c] = _ecr_contactConstraints[c].totalLambda;
    vec3.scale(result.frictionImpulse, tangent1, friction1.totalLambda);
    vec3.scaleAndAdd(result.frictionImpulse, result.frictionImpulse, tangent2, friction2.totalLambda);
    result.angularFrictionImpulse = angularFriction.totalLambda;
}
