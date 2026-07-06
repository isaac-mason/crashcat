import type { Mat3, Mat4, Quat, Vec3 } from 'mathcat';
import type { MassProperties } from './mass-properties.js';
/** motion quality options for collision detection */
export declare enum MotionQuality {
    /**
     * Discrete collision detection - standard discrete update.
     * Fast but can tunnel through thin geometry at high velocities.
     */
    DISCRETE = 0,
    /**
     * Continuous collision detection via linear cast.
     * Prevents fast-moving objects from tunneling through thin geometry
     * by casting shapes along their velocity vectors.
     * More expensive than DISCRETE.
     */
    LINEAR_CAST = 1
}
/** motion properties of a rigid body */
export type MotionProperties = {
    /** linear velocity of the body */
    linearVelocity: Vec3;
    /** angular velocity of the body */
    angularVelocity: Vec3;
    /** inverse inertia diagonal */
    invInertiaDiagonal: Vec3;
    /** inertia rotation */
    inertiaRotation: Quat;
    /** force accumulator */
    force: Vec3;
    /** torque accumulator */
    torque: Vec3;
    /** inverse mass (1/mass) */
    invMass: number;
    /** linear damping coefficient */
    linearDamping: number;
    /** angular damping coefficient */
    angularDamping: number;
    /** maximum linear velocity */
    maxLinearVelocity: number;
    /** maximum angular velocity */
    maxAngularVelocity: number;
    /** gravity multiplier factor */
    gravityFactor: number;
    /** motion quality for collision detection */
    motionQuality: MotionQuality;
    /** allowed degrees of freedom bitmask */
    allowedDegreesOfFreedom: number;
    /** velocity solver iterations override (0 = use default) */
    numVelocityStepsOverride: number;
    /** position solver iterations override (0 = use default) */
    numPositionStepsOverride: number;
    /** spheres used for sleeping test */
    sleepTestSpheres: Array<{
        center: Vec3;
        radius: number;
    }>;
    /** whether the body is allowed to sleep */
    allowSleeping: boolean;
    /** timer for sleeping test */
    sleepTestTimer: number;
};
export declare function create(): MotionProperties;
/** Adds a force to the force accumulator. */
export declare function addForce(motionProperties: MotionProperties, force: Vec3): void;
/** Adds a torque to the torque accumulator. */
export declare function addTorque(motionProperties: MotionProperties, torque: Vec3): void;
/** Adds a force at a specific world-space position, generating both force and torque. */
export declare function addForceAtPosition(motionProperties: MotionProperties, force: Vec3, worldPosition: Vec3, centerOfMassPosition: Vec3): void;
/** Applies an impulse at the center of mass, instantly changing linear velocity. */
export declare function addImpulse(motionProperties: MotionProperties, impulse: Vec3): void;
/** Applies an angular impulse, instantly changing angular velocity. */
export declare function addAngularImpulse(motionProperties: MotionProperties, impulse: Vec3, quaternion: Quat): void;
/** Applies an impulse at a specific world-space position, changing both linear and angular velocity. */
export declare function addImpulseAtPosition(motionProperties: MotionProperties, impulse: Vec3, worldPosition: Vec3, centerOfMassPosition: Vec3, quaternion: Quat): void;
/**
 * Decomposes the inertia tensor into principal moments (eigenvalues) and rotation matrix (eigenvectors)
 * Uses Jacobi's eigenvalue algorithm for 3x3 symmetric matrices
 *
 * Based on Jolt Physics EigenValueSymmetric implementation from Numerical Recipes paragraph 11.1
 * @see https://en.wikipedia.org/wiki/Eigenvalues_and_eigenvectors
 */
export declare function decomposePrincipalMomentsOfInertia(inertia: Mat4, outRotation: Mat3, outDiagonal: Vec3): boolean;
export declare function setMassProperties(motionProperties: MotionProperties, allowedDOFs: number, massProperties: MassProperties): void;
/**
 * Computes the world-space inverse inertia matrix for a given body rotation.
 *
 * Formula: I_inv_world = R * diag(invInertiaDiagonal) * R * mInertiaRotation * R^T
 * where R is the body's rotation matrix.
 *
 * @param out output Mat4 to store the result
 * @param motionProperties motion properties containing inertia data
 * @param bodyRotation body's rotation matrix (Mat4)
 * @returns out parameter
 *
 * @optimize
 */
export declare function getInverseInertiaForRotation(out: Mat4, motionProperties: MotionProperties, bodyRotation: Mat4): Mat4;
/** Clamps linear velocity to the maximum allowed value */
export declare function clampLinearVelocity(motionProperties: MotionProperties): void;
/** Clamps angular velocity to the maximum allowed value */
export declare function clampAngularVelocity(motionProperties: MotionProperties): void;
/**
 * Sets linear velocity and clamps it to the maximum allowed value.
 * This setter combines assignment with clamping to prevent invalid states.
 * @param motionProperties motion properties to update
 * @param velocity new linear velocity
 */
export declare function setLinearVelocity(motionProperties: MotionProperties, velocity: Vec3): void;
/**
 * Sets angular velocity and clamps it to the maximum allowed value.
 * This setter combines assignment with clamping to prevent invalid states.
 * @param motionProperties motion properties to update
 * @param velocity new angular velocity
 */
export declare function setAngularVelocity(motionProperties: MotionProperties, velocity: Vec3): void;
/**
 * Adds to the linear velocity and clamps to the maximum allowed value.
 *
 * @param motionProperties motion properties to update
 * @param velocityDelta velocity change to add
 */
export declare function addLinearVelocity(motionProperties: MotionProperties, velocityDelta: Vec3): void;
/**
 * Adds to the angular velocity and clamps to the maximum allowed value.
 * @param motionProperties motion properties to update
 * @param velocityDelta angular velocity change to add
 */
export declare function addAngularVelocity(motionProperties: MotionProperties, velocityDelta: Vec3): void;
/**
 * Add a linear velocity step (used during constraint solving).
 * Applies translation locking based on allowed degrees of freedom.
 * @param motionProperties motion properties to update
 * @param linearVelocityChange velocity change to add
 *
 * @optimize
 */
export declare function addLinearVelocityStep(motionProperties: MotionProperties, linearVelocityChange: Vec3): void;
/**
 * Subtract a linear velocity step (used during constraint solving).
 * Applies translation locking based on allowed degrees of freedom.
 * @param motionProperties motion properties to update
 * @param linearVelocityChange velocity change to subtract
 *
 * @optimize
 */
export declare function subLinearVelocityStep(motionProperties: MotionProperties, linearVelocityChange: Vec3): void;
/**
 * Add an angular velocity step (used during constraint solving).
 * @param motionProperties motion properties to update
 * @param angularVelocityChange velocity change to add
 *
 * @optimize
 */
export declare function addAngularVelocityStep(motionProperties: MotionProperties, angularVelocityChange: Vec3): void;
/**
 * Subtract an angular velocity step (used during constraint solving).
 * @param motionProperties motion properties to update
 * @param angularVelocityChange velocity change to subtract
 *
 * @optimize
 */
export declare function subAngularVelocityStep(motionProperties: MotionProperties, angularVelocityChange: Vec3): void;
/**
 * Scales the inverse inertia diagonal when mass changes at runtime.
 * This maintains the inertia tensor's proportionality to mass.
 *
 * @param motionProperties motion properties to scale
 * @param newMass new mass value (must be > 0)
 *
 * @optimize
 */
export declare function scaleToMass(motionProperties: MotionProperties, newMass: number): void;
/**
 * Set velocity of body such that it will be rotate/translate by inDeltaPosition/Rotation in inDeltaTime seconds.
 *
 * @param motionProperties motion properties to update
 * @param deltaPosition desired position change
 * @param deltaRotation desired rotation change as quaternion
 * @param deltaTime time step (must be > 0)
 */
export declare function moveKinematic(motionProperties: MotionProperties, deltaPosition: Vec3, deltaRotation: Quat, deltaTime: number): void;
/**
 * Applies translation DOF constraint to a velocity vector.
 * Zeros out velocity components for locked translation axes.
 */
export declare function applyTranslationDOFConstraint(velocity: Vec3, allowedDOFs: number): void;
/**
 * Applies rotation DOF constraint to an angular velocity vector.
 * Zeros out angular velocity components for locked rotation axes.
 */
export declare function applyRotationDOFConstraint(angularVelocity: Vec3, allowedDOFs: number): void;
/**
 * Multiplies a vector by the world-space inverse inertia matrix more efficiently than computing the full matrix.
 * This applies both the inertia transform and DOF constraints.
 *
 * @param out output vector to store result
 * @param motionProperties motion properties containing inertia data
 * @param bodyQuaternion body quaternion
 * @param vector vector to multiply
 * @returns out parameter
 */
export declare function multiplyWorldSpaceInverseInertiaByVector(out: Vec3, motionProperties: MotionProperties, bodyQuaternion: Quat, vector: Vec3): Vec3;
/**
 * Apply the gyroscopic force (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
 * This simulates the realistic behavior of spinning bodies by applying torque T = -ω × (I·ω)
 *
 * @param motionProperties motion properties to update
 * @param bodyQuaternion body quaternion
 * @param deltaTime time step for integration
 */
export declare function applyGyroscopicForce(motionProperties: MotionProperties, bodyQuaternion: Quat, deltaTime: number): void;
