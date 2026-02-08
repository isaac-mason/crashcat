import type { Quat, Vec3 } from 'mathcat';
import { mat4, quat, vec2, vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body';
import type { AngleConstraintPart } from './angle-constraint-part';
import * as angleConstraintPart from './angle-constraint-part';
import * as rotationEulerConstraintPart from './rotation-euler-constraint-part';

/** how the swing limit behaves */
export enum SwingType {
    /** swing is limited by a cone shape (symmetric around 0) */
    CONE = 0,
    /** swing is limited by a pyramid shape (asymmetric limits supported) */
    PYRAMID = 1,
}

/** flags for clamped axes during ClampSwingTwist */
enum ClampedAxis {
    TWIST_MIN = 1 << 0,
    TWIST_MAX = 1 << 1,
    SWING_Y_MIN = 1 << 2,
    SWING_Y_MAX = 1 << 3,
    SWING_Z_MIN = 1 << 4,
    SWING_Z_MAX = 1 << 5,
}

/** flags indicating rotation state */
enum RotationFlags {
    TWIST_X_LOCKED = 1 << 0,
    SWING_Y_LOCKED = 1 << 1,
    SWING_Z_LOCKED = 1 << 2,
    TWIST_X_FREE = 1 << 3,
    SWING_Y_FREE = 1 << 4,
    SWING_Z_FREE = 1 << 5,
    SWING_YZ_FREE = (1 << 4) | (1 << 5),
}

/**
 * SwingTwistConstraintPart decomposes rotation into swing and twist components
 * and constrains them within limits.
 *
 * Quaternion decomposition: q = q_swing * q_twist
 * where q_swing.x = 0 and q_twist.y = q_twist.z = 0
 *
 * - Twist (rotation around X axis) is within [twistMinAngle, twistMaxAngle]
 * - Swing (rotation around Y and Z axes) is limited by an ellipsoid (cone) or pyramid
 */
export type SwingTwistConstraintPart = {
    // configuration
    swingType: SwingType;
    rotationFlags: number;

    // twist limits (pre-computed sin/cos of half angles)
    sinTwistHalfMinAngle: number;
    sinTwistHalfMaxAngle: number;
    cosTwistHalfMinAngle: number;
    cosTwistHalfMaxAngle: number;

    // swing y limits (pre-computed sin/cos of half angles)
    swingYHalfMinAngle: number;
    swingYHalfMaxAngle: number;
    sinSwingYHalfMinAngle: number;
    sinSwingYHalfMaxAngle: number;
    cosSwingYHalfMinAngle: number;
    cosSwingYHalfMaxAngle: number;

    // swing z limits (pre-computed sin/cos of half angles)
    swingZHalfMinAngle: number;
    swingZHalfMaxAngle: number;
    sinSwingZHalfMinAngle: number;
    sinSwingZHalfMaxAngle: number;
    cosSwingZHalfMinAngle: number;
    cosSwingZHalfMaxAngle: number;

    // runtime properties - world space rotation axes
    worldSpaceSwingLimitYRotationAxis: Vec3;
    worldSpaceSwingLimitZRotationAxis: Vec3;
    worldSpaceTwistLimitRotationAxis: Vec3;

    // constraint parts for each DOF
    swingLimitYConstraintPart: AngleConstraintPart;
    swingLimitZConstraintPart: AngleConstraintPart;
    twistLimitConstraintPart: AngleConstraintPart;
};

/** create a new SwingTwistConstraintPart with zero-initialized values */
export function create(): SwingTwistConstraintPart {
    return {
        swingType: SwingType.CONE,
        rotationFlags: 0,

        sinTwistHalfMinAngle: 0,
        sinTwistHalfMaxAngle: 0,
        cosTwistHalfMinAngle: 1,
        cosTwistHalfMaxAngle: 1,

        swingYHalfMinAngle: 0,
        swingYHalfMaxAngle: 0,
        sinSwingYHalfMinAngle: 0,
        sinSwingYHalfMaxAngle: 0,
        cosSwingYHalfMinAngle: 1,
        cosSwingYHalfMaxAngle: 1,

        swingZHalfMinAngle: 0,
        swingZHalfMaxAngle: 0,
        sinSwingZHalfMinAngle: 0,
        sinSwingZHalfMaxAngle: 0,
        cosSwingZHalfMinAngle: 1,
        cosSwingZHalfMaxAngle: 1,

        worldSpaceSwingLimitYRotationAxis: vec3.create(),
        worldSpaceSwingLimitZRotationAxis: vec3.create(),
        worldSpaceTwistLimitRotationAxis: vec3.create(),

        swingLimitYConstraintPart: angleConstraintPart.create(),
        swingLimitZConstraintPart: angleConstraintPart.create(),
        twistLimitConstraintPart: angleConstraintPart.create(),
    };
}

/** deactivate this constraint part */
export function deactivate(part: SwingTwistConstraintPart): void {
    angleConstraintPart.deactivate(part.swingLimitYConstraintPart);
    angleConstraintPart.deactivate(part.swingLimitZConstraintPart);
    angleConstraintPart.deactivate(part.twistLimitConstraintPart);
}

/** check if any constraint part is active */
export function isActive(part: SwingTwistConstraintPart): boolean {
    return (
        angleConstraintPart.isActive(part.swingLimitYConstraintPart) ||
        angleConstraintPart.isActive(part.swingLimitZConstraintPart) ||
        angleConstraintPart.isActive(part.twistLimitConstraintPart)
    );
}

/** degrees to radians constant */
const DEG_TO_RAD = Math.PI / 180;

/**
 * Set limits for this constraint.
 * @param part the constraint part to configure
 * @param twistMinAngle minimum twist angle (radians), in [-PI, PI]
 * @param twistMaxAngle maximum twist angle (radians), in [-PI, PI]
 * @param swingYMinAngle minimum swing Y angle (radians), in [-PI, PI]
 * @param swingYMaxAngle maximum swing Y angle (radians), in [-PI, PI]
 * @param swingZMinAngle minimum swing Z angle (radians), in [-PI, PI]
 * @param swingZMaxAngle maximum swing Z angle (radians), in [-PI, PI]
 */
export function setLimits(
    part: SwingTwistConstraintPart,
    twistMinAngle: number,
    twistMaxAngle: number,
    swingYMinAngle: number,
    swingYMaxAngle: number,
    swingZMinAngle: number,
    swingZMaxAngle: number,
): void {
    const lockedAngle = 0.5 * DEG_TO_RAD;
    const freeAngle = 179.5 * DEG_TO_RAD;

    // calculate half angles for twist
    const halfTwistMin = 0.5 * twistMinAngle;
    const halfTwistMax = 0.5 * twistMaxAngle;

    // calculate half angles for swing
    const halfSwingYMin = 0.5 * swingYMinAngle;
    const halfSwingYMax = 0.5 * swingYMaxAngle;
    const halfSwingZMin = 0.5 * swingZMinAngle;
    const halfSwingZMax = 0.5 * swingZMaxAngle;

    // store half angles for pyramid limit
    part.swingYHalfMinAngle = halfSwingYMin;
    part.swingYHalfMaxAngle = halfSwingYMax;
    part.swingZHalfMinAngle = halfSwingZMin;
    part.swingZHalfMaxAngle = halfSwingZMax;

    // reset rotation flags
    part.rotationFlags = 0;

    // configure twist
    if (twistMinAngle > -lockedAngle && twistMaxAngle < lockedAngle) {
        part.rotationFlags |= RotationFlags.TWIST_X_LOCKED;
        part.sinTwistHalfMinAngle = 0;
        part.sinTwistHalfMaxAngle = 0;
        part.cosTwistHalfMinAngle = 1;
        part.cosTwistHalfMaxAngle = 1;
    } else if (twistMinAngle < -freeAngle && twistMaxAngle > freeAngle) {
        part.rotationFlags |= RotationFlags.TWIST_X_FREE;
        part.sinTwistHalfMinAngle = -1;
        part.sinTwistHalfMaxAngle = 1;
        part.cosTwistHalfMinAngle = 0;
        part.cosTwistHalfMaxAngle = 0;
    } else {
        part.sinTwistHalfMinAngle = Math.sin(halfTwistMin);
        part.sinTwistHalfMaxAngle = Math.sin(halfTwistMax);
        part.cosTwistHalfMinAngle = Math.cos(halfTwistMin);
        part.cosTwistHalfMaxAngle = Math.cos(halfTwistMax);
    }

    // configure swing Y
    if (swingYMinAngle > -lockedAngle && swingYMaxAngle < lockedAngle) {
        part.rotationFlags |= RotationFlags.SWING_Y_LOCKED;
        part.sinSwingYHalfMinAngle = 0;
        part.sinSwingYHalfMaxAngle = 0;
        part.cosSwingYHalfMinAngle = 1;
        part.cosSwingYHalfMaxAngle = 1;
    } else if (swingYMinAngle < -freeAngle && swingYMaxAngle > freeAngle) {
        part.rotationFlags |= RotationFlags.SWING_Y_FREE;
        part.sinSwingYHalfMinAngle = -1;
        part.sinSwingYHalfMaxAngle = 1;
        part.cosSwingYHalfMinAngle = 0;
        part.cosSwingYHalfMaxAngle = 0;
    } else {
        part.sinSwingYHalfMinAngle = Math.sin(halfSwingYMin);
        part.sinSwingYHalfMaxAngle = Math.sin(halfSwingYMax);
        part.cosSwingYHalfMinAngle = Math.cos(halfSwingYMin);
        part.cosSwingYHalfMaxAngle = Math.cos(halfSwingYMax);
    }

    // configure swing Z
    if (swingZMinAngle > -lockedAngle && swingZMaxAngle < lockedAngle) {
        part.rotationFlags |= RotationFlags.SWING_Z_LOCKED;
        part.sinSwingZHalfMinAngle = 0;
        part.sinSwingZHalfMaxAngle = 0;
        part.cosSwingZHalfMinAngle = 1;
        part.cosSwingZHalfMaxAngle = 1;
    } else if (swingZMinAngle < -freeAngle && swingZMaxAngle > freeAngle) {
        part.rotationFlags |= RotationFlags.SWING_Z_FREE;
        part.sinSwingZHalfMinAngle = -1;
        part.sinSwingZHalfMaxAngle = 1;
        part.cosSwingZHalfMinAngle = 0;
        part.cosSwingZHalfMaxAngle = 0;
    } else {
        part.sinSwingZHalfMinAngle = Math.sin(halfSwingZMin);
        part.sinSwingZHalfMaxAngle = Math.sin(halfSwingZMax);
        part.cosSwingZHalfMinAngle = Math.cos(halfSwingZMin);
        part.cosSwingZHalfMaxAngle = Math.cos(halfSwingZMax);
    }
}

/** helper to determine if we're closer to the min or max limit. */
function distanceToMinShorter(deltaMin: number, deltaMax: number): boolean {
    // handle wrap-around for angles
    let absMin = Math.abs(deltaMin);
    if (absMin > 1) absMin = 2 - absMin;
    let absMax = Math.abs(deltaMax);
    if (absMax > 1) absMax = 2 - absMax;
    return absMin < absMax;
}

/**
 * Decompose a quaternion into swing and twist components.
 * q = swing * twist where swing.x = 0 and twist.y = twist.z = 0
 */
export function getSwingTwist(q: Quat, outSwing: Quat, outTwist: Quat): void {
    const x = q[0];
    const y = q[1];
    const z = q[2];
    const w = q[3];

    const s = Math.sqrt(w * w + x * x);
    if (s !== 0) {
        quat.set(outTwist, x / s, 0, 0, w / s);
        quat.set(outSwing, 0, (w * y - x * z) / s, (w * z + x * y) / s, s);
    } else {
        // If both x and w are zero, this must be a 180 degree rotation around either y or z
        quat.identity(outTwist);
        quat.set(outSwing, x, y, z, w);
    }
}

const _clampSwingTwist_ellipseClosest = /* @__PURE__ */ vec2.create();

/**
 * Clamp swing and twist quaternions against limits.
 * @param part the constraint part with limits
 * @param ioSwing swing quaternion to clamp (modified in place)
 * @param ioTwist twist quaternion to clamp (modified in place)
 * @returns flags indicating which axes were clamped
 */
export function clampSwingTwist(part: SwingTwistConstraintPart, ioSwing: Quat, ioTwist: Quat): number {
    let clampedAxis = 0;

    // ensure quaternions have w > 0 for consistent clamping
    const negateSwing = ioSwing[3] < 0;
    if (negateSwing) {
        quat.scale(ioSwing, ioSwing, -1);
    }
    const negateTwist = ioTwist[3] < 0;
    if (negateTwist) {
        quat.scale(ioTwist, ioTwist, -1);
    }

    // clamp twist
    if (part.rotationFlags & RotationFlags.TWIST_X_LOCKED) {
        // twist axis is locked
        if (ioTwist[0] !== 0) {
            clampedAxis |= ClampedAxis.TWIST_MIN | ClampedAxis.TWIST_MAX;
        }
        quat.identity(ioTwist);
    } else if ((part.rotationFlags & RotationFlags.TWIST_X_FREE) === 0) {
        // twist has limits
        const deltaMin = part.sinTwistHalfMinAngle - ioTwist[0];
        const deltaMax = ioTwist[0] - part.sinTwistHalfMaxAngle;
        if (deltaMin > 0 || deltaMax > 0) {
            if (distanceToMinShorter(deltaMin, deltaMax)) {
                quat.set(ioTwist, part.sinTwistHalfMinAngle, 0, 0, part.cosTwistHalfMinAngle);
                clampedAxis |= ClampedAxis.TWIST_MIN;
            } else {
                quat.set(ioTwist, part.sinTwistHalfMaxAngle, 0, 0, part.cosTwistHalfMaxAngle);
                clampedAxis |= ClampedAxis.TWIST_MAX;
            }
        }
    }

    // clamp swing
    if (part.rotationFlags & RotationFlags.SWING_Y_LOCKED) {
        if (part.rotationFlags & RotationFlags.SWING_Z_LOCKED) {
            // both swing Y and Z are locked
            if (ioSwing[1] !== 0) clampedAxis |= ClampedAxis.SWING_Y_MIN | ClampedAxis.SWING_Y_MAX;
            if (ioSwing[2] !== 0) clampedAxis |= ClampedAxis.SWING_Z_MIN | ClampedAxis.SWING_Z_MAX;
            quat.identity(ioSwing);
        } else {
            // only swing Y is locked
            if (ioSwing[1] !== 0) clampedAxis |= ClampedAxis.SWING_Y_MIN | ClampedAxis.SWING_Y_MAX;
            const deltaMin = part.sinSwingZHalfMinAngle - ioSwing[2];
            const deltaMax = ioSwing[2] - part.sinSwingZHalfMaxAngle;
            if (deltaMin > 0 || deltaMax > 0) {
                if (distanceToMinShorter(deltaMin, deltaMax)) {
                    quat.set(ioSwing, 0, 0, part.sinSwingZHalfMinAngle, part.cosSwingZHalfMinAngle);
                    clampedAxis |= ClampedAxis.SWING_Z_MIN;
                } else {
                    quat.set(ioSwing, 0, 0, part.sinSwingZHalfMaxAngle, part.cosSwingZHalfMaxAngle);
                    clampedAxis |= ClampedAxis.SWING_Z_MAX;
                }
            } else if (clampedAxis & ClampedAxis.SWING_Y_MIN) {
                const z = ioSwing[2];
                quat.set(ioSwing, 0, 0, z, Math.sqrt(1 - z * z));
            }
        }
    } else if (part.rotationFlags & RotationFlags.SWING_Z_LOCKED) {
        // only swing Z is locked
        if (ioSwing[2] !== 0) clampedAxis |= ClampedAxis.SWING_Z_MIN | ClampedAxis.SWING_Z_MAX;
        const deltaMin = part.sinSwingYHalfMinAngle - ioSwing[1];
        const deltaMax = ioSwing[1] - part.sinSwingYHalfMaxAngle;
        if (deltaMin > 0 || deltaMax > 0) {
            if (distanceToMinShorter(deltaMin, deltaMax)) {
                quat.set(ioSwing, 0, part.sinSwingYHalfMinAngle, 0, part.cosSwingYHalfMinAngle);
                clampedAxis |= ClampedAxis.SWING_Y_MIN;
            } else {
                quat.set(ioSwing, 0, part.sinSwingYHalfMaxAngle, 0, part.cosSwingYHalfMaxAngle);
                clampedAxis |= ClampedAxis.SWING_Y_MAX;
            }
        } else if (clampedAxis & ClampedAxis.SWING_Z_MIN) {
            const y = ioSwing[1];
            quat.set(ioSwing, 0, y, 0, Math.sqrt(1 - y * y));
        }
    } else {
        // two degrees of freedom in swing
        if (part.swingType === SwingType.CONE) {
            // use ellipse to solve limits
            const y = ioSwing[1];
            const z = ioSwing[2];
            // check if inside ellipse: (y/a)^2 + (z/b)^2 <= 1
            const a = part.sinSwingYHalfMaxAngle;
            const b = part.sinSwingZHalfMaxAngle;
            if (a > 0 && b > 0) {
                const ellipseValue = (y * y) / (a * a) + (z * z) / (b * b);
                if (ellipseValue > 1) {
                    // project to ellipse boundary
                    getClosestPointOnEllipse(_clampSwingTwist_ellipseClosest, y, z, a, b);
                    const newW = Math.sqrt(Math.max(0, 1 - _clampSwingTwist_ellipseClosest[0] * _clampSwingTwist_ellipseClosest[0] - _clampSwingTwist_ellipseClosest[1] * _clampSwingTwist_ellipseClosest[1]));
                    quat.set(ioSwing, 0, _clampSwingTwist_ellipseClosest[0], _clampSwingTwist_ellipseClosest[1], newW);
                    clampedAxis |=
                        ClampedAxis.SWING_Y_MIN | ClampedAxis.SWING_Y_MAX | ClampedAxis.SWING_Z_MIN | ClampedAxis.SWING_Z_MAX;
                }
            }
        } else {
            // pyramid limit - clamp Y and Z independently based on atan2
            // for pyramid: y/2 = atan2(q.y, q.w) and z/2 = atan2(q.z, q.w)
            const halfAngleY = Math.atan2(ioSwing[1], ioSwing[3]);
            const halfAngleZ = Math.atan2(ioSwing[2], ioSwing[3]);

            const clampedHalfY = Math.max(part.swingYHalfMinAngle, Math.min(part.swingYHalfMaxAngle, halfAngleY));
            const clampedHalfZ = Math.max(part.swingZHalfMinAngle, Math.min(part.swingZHalfMaxAngle, halfAngleZ));

            if (halfAngleY !== clampedHalfY || halfAngleZ !== clampedHalfZ) {
                // reconstruct quaternion without introducing twist
                const sinY = Math.sin(clampedHalfY);
                const cosY = Math.cos(clampedHalfY);
                const sinZ = Math.sin(clampedHalfZ);
                const cosZ = Math.cos(clampedHalfZ);
                // q = [0, sinY * cosZ, cosY * sinZ, cosY * cosZ] (normalized)
                const newY = sinY * cosZ;
                const newZ = cosY * sinZ;
                const newW = cosY * cosZ;
                const len = Math.sqrt(newY * newY + newZ * newZ + newW * newW);
                quat.set(ioSwing, 0, newY / len, newZ / len, newW / len);
                clampedAxis |=
                    ClampedAxis.SWING_Y_MIN | ClampedAxis.SWING_Y_MAX | ClampedAxis.SWING_Z_MIN | ClampedAxis.SWING_Z_MAX;
            }
        }
    }

    // flip sign back
    if (negateSwing) {
        quat.scale(ioSwing, ioSwing, -1);
    }
    if (negateTwist) {
        quat.scale(ioTwist, ioTwist, -1);
    }

    return clampedAxis;
}

/**
 * Get the closest point on an ellipse to a given point.
 * Assumes the point is outside the ellipse.
 *
 * Uses Newton-Raphson iteration on the Lagrange multiplier formulation.
 *
 * @see Rotation Joint Limits in Quaternion Space by Gino van den Bergen,
 *      section 10.1 in Game Engine Gems 3.
 */
function getClosestPointOnEllipse(out: [number, number], px: number, py: number, a: number, b: number): void {
    // handle degenerate cases
    if (a <= 0) {
        out[0] = 0;
        out[1] = py >= 0 ? b : -b;
        return;
    }
    if (b <= 0) {
        out[0] = px >= 0 ? a : -a;
        out[1] = 0;
        return;
    }

    const aSq = a * a;
    const bSq = b * b;

    // Equation of ellipse: f(x, y) = (x/a)^2 + (y/b)^2 - 1 = 0
    // Normal on surface: (df/dx, df/dy) = (2 x / a^2, 2 y / b^2)
    // Closest point (x', y') on ellipse to point (x, y): (x', y') + t (x / a^2, y / b^2) = (x, y)
    // <=> (x', y') = (a^2 x / (t + a^2), b^2 y / (t + b^2))
    // Requiring point to be on ellipse: g(t) = (a x / (t + a^2))^2 + (b y / (t + b^2))^2 - 1 = 0

    // Newton Raphson iteration, starting at t = 0
    let t = 0;
    for (let i = 0; i < 100; i++) {
        // Calculate g(t)
        const tPlusASq = t + aSq;
        const tPlusBSq = t + bSq;
        const gt = (a * px / tPlusASq) ** 2 + (b * py / tPlusBSq) ** 2 - 1;

        // Check if g(t) is close enough to zero
        if (Math.abs(gt) < 1e-6) {
            out[0] = (aSq * px) / tPlusASq;
            out[1] = (bSq * py) / tPlusBSq;
            return;
        }

        // Get derivative dg/dt = g'(t) = -2 (a^2 x^2 / (t + a^2)^3 + b^2 y^2 / (t + b^2)^3)
        const gtAccent =
            -2 * (aSq * px * px / (tPlusASq * tPlusASq * tPlusASq) + bSq * py * py / (tPlusBSq * tPlusBSq * tPlusBSq));

        // Calculate t for next iteration: t_n+1 = t_n - g(t) / g'(t)
        t = t - gt / gtAccent;
    }

    // Emergency fallback after max iterations (should rarely happen)
    const tPlusASq = t + aSq;
    const tPlusBSq = t + bSq;
    out[0] = (aSq * px) / tPlusASq;
    out[1] = (bSq * py) / tPlusBSq;
}

const _calc_q_swing = /* @__PURE__ */ quat.create();
const _calc_q_twist = /* @__PURE__ */ quat.create();
const _calc_q_clamped_swing = /* @__PURE__ */ quat.create();
const _calc_q_clamped_twist = /* @__PURE__ */ quat.create();
const _calc_twist_to_world = /* @__PURE__ */ quat.create();
const _calc_axisY = /* @__PURE__ */ vec3.create();

const _calc_axisZ = /* @__PURE__ */ vec3.create();
const _calc_axisX = /* @__PURE__ */ vec3.create();
const _calc_current = /* @__PURE__ */ vec3.create();
const _calc_desired = /* @__PURE__ */ vec3.create();
const _calc_cross = /* @__PURE__ */ vec3.create();

/**
 * Calculate constraint properties for the swing-twist limits.
 * @param part the constraint part to configure
 * @param bodyA first body
 * @param bodyB second body
 * @param constraintRotation current rotation of constraint in constraint space
 * @param constraintToWorld rotation from constraint space to world space
 */
export function calculateConstraintProperties(
    part: SwingTwistConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    constraintRotation: Quat,
    constraintToWorld: Quat,
): void {
    // decompose into swing and twist
    getSwingTwist(constraintRotation, _calc_q_swing, _calc_q_twist);

    // clamp against limits
    quat.copy(_calc_q_clamped_swing, _calc_q_swing);
    quat.copy(_calc_q_clamped_twist, _calc_q_twist);
    const clampedAxis = clampSwingTwist(part, _calc_q_clamped_swing, _calc_q_clamped_twist);

    // handle swing constraints
    if (part.rotationFlags & RotationFlags.SWING_Y_LOCKED) {
        // calculate twist_to_world = constraintToWorld * q_swing
        quat.multiply(_calc_twist_to_world, constraintToWorld, _calc_q_swing);

        // get Y and Z axes in world space
        vec3.set(_calc_axisY, 0, 1, 0);
        vec3.set(_calc_axisZ, 0, 0, 1);
        vec3.transformQuat(part.worldSpaceSwingLimitYRotationAxis, _calc_axisY, _calc_twist_to_world);
        vec3.transformQuat(part.worldSpaceSwingLimitZRotationAxis, _calc_axisZ, _calc_twist_to_world);

        if (part.rotationFlags & RotationFlags.SWING_Z_LOCKED) {
            // both swing axes locked
            angleConstraintPart.calculateConstraintProperties(
                part.swingLimitYConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceSwingLimitYRotationAxis,
            );
            angleConstraintPart.calculateConstraintProperties(
                part.swingLimitZConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceSwingLimitZRotationAxis,
            );
        } else {
            // only Y locked
            angleConstraintPart.calculateConstraintProperties(
                part.swingLimitYConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceSwingLimitYRotationAxis,
            );
            if (clampedAxis & (ClampedAxis.SWING_Z_MIN | ClampedAxis.SWING_Z_MAX)) {
                if (clampedAxis & ClampedAxis.SWING_Z_MIN) {
                    vec3.negate(part.worldSpaceSwingLimitZRotationAxis, part.worldSpaceSwingLimitZRotationAxis);
                }
                angleConstraintPart.calculateConstraintProperties(
                    part.swingLimitZConstraintPart,
                    bodyA,
                    bodyB,
                    part.worldSpaceSwingLimitZRotationAxis,
                );
            } else {
                angleConstraintPart.deactivate(part.swingLimitZConstraintPart);
            }
        }
    } else if (part.rotationFlags & RotationFlags.SWING_Z_LOCKED) {
        // only Z locked
        quat.multiply(_calc_twist_to_world, constraintToWorld, _calc_q_swing);

        vec3.set(_calc_axisY, 0, 1, 0);
        vec3.set(_calc_axisZ, 0, 0, 1);
        vec3.transformQuat(part.worldSpaceSwingLimitYRotationAxis, _calc_axisY, _calc_twist_to_world);
        vec3.transformQuat(part.worldSpaceSwingLimitZRotationAxis, _calc_axisZ, _calc_twist_to_world);

        if (clampedAxis & (ClampedAxis.SWING_Y_MIN | ClampedAxis.SWING_Y_MAX)) {
            if (clampedAxis & ClampedAxis.SWING_Y_MIN) {
                vec3.negate(part.worldSpaceSwingLimitYRotationAxis, part.worldSpaceSwingLimitYRotationAxis);
            }
            angleConstraintPart.calculateConstraintProperties(
                part.swingLimitYConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceSwingLimitYRotationAxis,
            );
        } else {
            angleConstraintPart.deactivate(part.swingLimitYConstraintPart);
        }
        angleConstraintPart.calculateConstraintProperties(
            part.swingLimitZConstraintPart,
            bodyA,
            bodyB,
            part.worldSpaceSwingLimitZRotationAxis,
        );
    } else if ((part.rotationFlags & RotationFlags.SWING_YZ_FREE) !== RotationFlags.SWING_YZ_FREE) {
        // swing has limits around Y and Z
        if (
            clampedAxis &
            (ClampedAxis.SWING_Y_MIN | ClampedAxis.SWING_Y_MAX | ClampedAxis.SWING_Z_MIN | ClampedAxis.SWING_Z_MAX)
        ) {
            // calculate axis of rotation from clamped swing to swing
            vec3.set(_calc_axisX, 1, 0, 0);

            // current = (constraintToWorld * q_swing).RotateAxisX()
            quat.multiply(_calc_twist_to_world, constraintToWorld, _calc_q_swing);
            vec3.transformQuat(_calc_current, _calc_axisX, _calc_twist_to_world);

            // desired = (constraintToWorld * q_clamped_swing).RotateAxisX()
            quat.multiply(_calc_twist_to_world, constraintToWorld, _calc_q_clamped_swing);
            vec3.transformQuat(_calc_desired, _calc_axisX, _calc_twist_to_world);

            // axis = desired x current
            vec3.cross(_calc_cross, _calc_desired, _calc_current);
            const len = vec3.length(_calc_cross);
            if (len > 1e-6) {
                vec3.scale(part.worldSpaceSwingLimitYRotationAxis, _calc_cross, 1 / len);
                angleConstraintPart.calculateConstraintProperties(
                    part.swingLimitYConstraintPart,
                    bodyA,
                    bodyB,
                    part.worldSpaceSwingLimitYRotationAxis,
                );
            } else {
                angleConstraintPart.deactivate(part.swingLimitYConstraintPart);
            }
        } else {
            angleConstraintPart.deactivate(part.swingLimitYConstraintPart);
        }
        angleConstraintPart.deactivate(part.swingLimitZConstraintPart);
    } else {
        // no swing limits
        angleConstraintPart.deactivate(part.swingLimitYConstraintPart);
        angleConstraintPart.deactivate(part.swingLimitZConstraintPart);
    }

    // handle twist constraint
    if (part.rotationFlags & RotationFlags.TWIST_X_LOCKED) {
        // twist locked, always activate constraint
        vec3.set(_calc_axisX, 1, 0, 0);
        quat.multiply(_calc_twist_to_world, constraintToWorld, _calc_q_swing);
        vec3.transformQuat(part.worldSpaceTwistLimitRotationAxis, _calc_axisX, _calc_twist_to_world);
        angleConstraintPart.calculateConstraintProperties(
            part.twistLimitConstraintPart,
            bodyA,
            bodyB,
            part.worldSpaceTwistLimitRotationAxis,
        );
    } else if ((part.rotationFlags & RotationFlags.TWIST_X_FREE) === 0) {
        // twist has limits
        if (clampedAxis & (ClampedAxis.TWIST_MIN | ClampedAxis.TWIST_MAX)) {
            vec3.set(_calc_axisX, 1, 0, 0);
            quat.multiply(_calc_twist_to_world, constraintToWorld, _calc_q_swing);
            vec3.transformQuat(part.worldSpaceTwistLimitRotationAxis, _calc_axisX, _calc_twist_to_world);
            if (clampedAxis & ClampedAxis.TWIST_MIN) {
                vec3.negate(part.worldSpaceTwistLimitRotationAxis, part.worldSpaceTwistLimitRotationAxis);
            }
            angleConstraintPart.calculateConstraintProperties(
                part.twistLimitConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceTwistLimitRotationAxis,
            );
        } else {
            angleConstraintPart.deactivate(part.twistLimitConstraintPart);
        }
    } else {
        // no twist limits
        angleConstraintPart.deactivate(part.twistLimitConstraintPart);
    }
}

/** Warm start velocity constraints */
export function warmStart(
    part: SwingTwistConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    warmStartImpulseRatio: number,
): void {
    angleConstraintPart.warmStart(part.swingLimitYConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    angleConstraintPart.warmStart(part.swingLimitZConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
    angleConstraintPart.warmStart(part.twistLimitConstraintPart, bodyA, bodyB, warmStartImpulseRatio);
}

/**
 * Solve velocity constraints for swing-twist limits.
 * @returns true if any impulse was applied
 */
export function solveVelocityConstraint(part: SwingTwistConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean {
    let impulse = false;

    // solve swing constraint
    if (angleConstraintPart.isActive(part.swingLimitYConstraintPart)) {
        const maxLambda = part.sinSwingYHalfMinAngle === part.sinSwingYHalfMaxAngle ? Infinity : 0;
        impulse =
            angleConstraintPart.solveVelocityConstraint(
                part.swingLimitYConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceSwingLimitYRotationAxis,
                -Infinity,
                maxLambda,
            ) || impulse;
    }

    if (angleConstraintPart.isActive(part.swingLimitZConstraintPart)) {
        const maxLambda = part.sinSwingZHalfMinAngle === part.sinSwingZHalfMaxAngle ? Infinity : 0;
        impulse =
            angleConstraintPart.solveVelocityConstraint(
                part.swingLimitZConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceSwingLimitZRotationAxis,
                -Infinity,
                maxLambda,
            ) || impulse;
    }

    // solve twist constraint
    if (angleConstraintPart.isActive(part.twistLimitConstraintPart)) {
        const maxLambda = part.sinTwistHalfMinAngle === part.sinTwistHalfMaxAngle ? Infinity : 0;
        impulse =
            angleConstraintPart.solveVelocityConstraint(
                part.twistLimitConstraintPart,
                bodyA,
                bodyB,
                part.worldSpaceTwistLimitRotationAxis,
                -Infinity,
                maxLambda,
            ) || impulse;
    }

    return impulse;
}

const _pos_q_swing = /* @__PURE__ */ quat.create();
const _pos_q_twist = /* @__PURE__ */ quat.create();
const _pos_inv_initial = /* @__PURE__ */ quat.create();
const _pos_rotA = /* @__PURE__ */ mat4.create();
const _pos_rotB = /* @__PURE__ */ mat4.create();
const _pos_rotation_part = rotationEulerConstraintPart.create();

/**
 * Solve position constraints for swing-twist limits.
 * @param part the constraint part
 * @param bodyA first body
 * @param bodyB second body
 * @param constraintRotation current rotation in constraint space
 * @param constraintToBody1 rotation from constraint space to body 1 space
 * @param constraintToBody2 rotation from constraint space to body 2 space
 * @param baumgarte baumgarte stabilization factor
 * @returns true if any correction was applied
 */
export function solvePositionConstraint(
    part: SwingTwistConstraintPart,
    bodyA: RigidBody,
    bodyB: RigidBody,
    constraintRotation: Quat,
    constraintToBody1: Quat,
    constraintToBody2: Quat,
    baumgarte: number,
): boolean {
    getSwingTwist(constraintRotation, _pos_q_swing, _pos_q_twist);

    const clampedAxis = clampSwingTwist(part, _pos_q_swing, _pos_q_twist);

    // solve rotation violations
    if (clampedAxis !== 0) {
        // inv_initial_orientation = constraintToBody2 * (constraintToBody1 * q_swing * q_twist)^-1
        const temp = quat.create();
        quat.multiply(temp, constraintToBody1, _pos_q_swing);
        quat.multiply(temp, temp, _pos_q_twist);
        quat.conjugate(temp, temp);
        quat.multiply(_pos_inv_initial, constraintToBody2, temp);

        // create a fresh rotation euler constraint part
        mat4.fromQuat(_pos_rotA, bodyA.quaternion);
        mat4.fromQuat(_pos_rotB, bodyB.quaternion);
        rotationEulerConstraintPart.calculateConstraintProperties(_pos_rotation_part, bodyA, _pos_rotA, bodyB, _pos_rotB);

        return rotationEulerConstraintPart.solvePositionConstraint(_pos_rotation_part, bodyA, bodyB, _pos_inv_initial, baumgarte);
    }

    return false;
}

/** get total swing Y lambda */
export function getTotalSwingYLambda(part: SwingTwistConstraintPart): number {
    return part.swingLimitYConstraintPart.totalLambda;
}

/** get total swing Z lambda */
export function getTotalSwingZLambda(part: SwingTwistConstraintPart): number {
    return part.swingLimitZConstraintPart.totalLambda;
}

/** get total twist lambda */
export function getTotalTwistLambda(part: SwingTwistConstraintPart): number {
    return part.twistLimitConstraintPart.totalLambda;
}
