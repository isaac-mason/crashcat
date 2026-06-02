import type { Quat, Vec3 } from 'mathcat';
import type { RigidBody } from '../../body/rigid-body.js';
import type { AngleConstraintPart } from './angle-constraint-part.js';
/** how the swing limit behaves */
export declare enum SwingType {
    /** swing is limited by a cone shape (symmetric around 0) */
    CONE = 0,
    /** swing is limited by a pyramid shape (asymmetric limits supported) */
    PYRAMID = 1
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
    swingType: SwingType;
    rotationFlags: number;
    sinTwistHalfMinAngle: number;
    sinTwistHalfMaxAngle: number;
    cosTwistHalfMinAngle: number;
    cosTwistHalfMaxAngle: number;
    swingYHalfMinAngle: number;
    swingYHalfMaxAngle: number;
    sinSwingYHalfMinAngle: number;
    sinSwingYHalfMaxAngle: number;
    cosSwingYHalfMinAngle: number;
    cosSwingYHalfMaxAngle: number;
    swingZHalfMinAngle: number;
    swingZHalfMaxAngle: number;
    sinSwingZHalfMinAngle: number;
    sinSwingZHalfMaxAngle: number;
    cosSwingZHalfMinAngle: number;
    cosSwingZHalfMaxAngle: number;
    worldSpaceSwingLimitYRotationAxis: Vec3;
    worldSpaceSwingLimitZRotationAxis: Vec3;
    worldSpaceTwistLimitRotationAxis: Vec3;
    swingLimitYConstraintPart: AngleConstraintPart;
    swingLimitZConstraintPart: AngleConstraintPart;
    twistLimitConstraintPart: AngleConstraintPart;
};
/** create a new SwingTwistConstraintPart with zero-initialized values */
export declare function create(): SwingTwistConstraintPart;
/** deactivate this constraint part */
export declare function deactivate(part: SwingTwistConstraintPart): void;
/** check if any constraint part is active */
export declare function isActive(part: SwingTwistConstraintPart): boolean;
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
export declare function setLimits(part: SwingTwistConstraintPart, twistMinAngle: number, twistMaxAngle: number, swingYMinAngle: number, swingYMaxAngle: number, swingZMinAngle: number, swingZMaxAngle: number): void;
/**
 * Decompose a quaternion into swing and twist components.
 * q = swing * twist where swing.x = 0 and twist.y = twist.z = 0
 */
export declare function getSwingTwist(q: Quat, outSwing: Quat, outTwist: Quat): void;
/**
 * Clamp swing and twist quaternions against limits.
 * @param part the constraint part with limits
 * @param ioSwing swing quaternion to clamp (modified in place)
 * @param ioTwist twist quaternion to clamp (modified in place)
 * @returns flags indicating which axes were clamped
 */
export declare function clampSwingTwist(part: SwingTwistConstraintPart, ioSwing: Quat, ioTwist: Quat): number;
/**
 * Calculate constraint properties for the swing-twist limits.
 * @param part the constraint part to configure
 * @param bodyA first body
 * @param bodyB second body
 * @param constraintRotation current rotation of constraint in constraint space
 * @param constraintToWorld rotation from constraint space to world space
 */
export declare function calculateConstraintProperties(part: SwingTwistConstraintPart, bodyA: RigidBody, bodyB: RigidBody, constraintRotation: Quat, constraintToWorld: Quat): void;
/** Warm start velocity constraints */
export declare function warmStart(part: SwingTwistConstraintPart, bodyA: RigidBody, bodyB: RigidBody, warmStartImpulseRatio: number): void;
/**
 * Solve velocity constraints for swing-twist limits.
 * @returns true if any impulse was applied
 */
export declare function solveVelocityConstraint(part: SwingTwistConstraintPart, bodyA: RigidBody, bodyB: RigidBody): boolean;
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
export declare function solvePositionConstraint(part: SwingTwistConstraintPart, bodyA: RigidBody, bodyB: RigidBody, constraintRotation: Quat, constraintToBody1: Quat, constraintToBody2: Quat, baumgarte: number): boolean;
/** get total swing Y lambda */
export declare function getTotalSwingYLambda(part: SwingTwistConstraintPart): number;
/** get total swing Z lambda */
export declare function getTotalSwingZLambda(part: SwingTwistConstraintPart): number;
/** get total twist lambda */
export declare function getTotalTwistLambda(part: SwingTwistConstraintPart): number;
