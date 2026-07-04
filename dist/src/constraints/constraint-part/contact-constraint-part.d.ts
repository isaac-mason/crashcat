import type { Vec3 } from 'mathcat';
import type { AxisConstraintPart } from './axis-constraint-part.js';
/**
 * contact-specific constraint part functions that operate on cached velocity locals
 * instead of reading/writing body motion properties directly.
 *
 * this mirrors jolt's ContactConstraintPart — a thin layer over AxisConstraintPart
 * that avoids repeated body property access during the contact solve loop.
 *
 * the caller is responsible for:
 * 1. loading velocities from bodies into locals before the solve loop
 * 2. passing these locals through all getTotalLambda/applyLambda calls
 * 3. writing velocities back to bodies + applying DOF masking once after the loop
 *
 * uses the same AxisConstraintPart data type — no new data structure needed.
 */
/**
 * calculate what the total lambda would be (without applying impulse).
 * velocity-local version — reads from cached velocity vectors instead of body properties.
 *
 * @param part the constraint part
 * @param linVelA linear velocity of body A (local copy)
 * @param angVelA angular velocity of body A (local copy)
 * @param linVelB linear velocity of body B (local copy)
 * @param angVelB angular velocity of body B (local copy)
 * @param movingA true if body A is not static (dynamic or kinematic)
 * @param movingB true if body B is not static (dynamic or kinematic)
 * @param axis constraint axis
 * @returns new total lambda (unclamped)
 *
 * @inline
 * @optimize
 */
export declare function getTotalLambda(part: AxisConstraintPart, linVelA: Vec3, angVelA: Vec3, linVelB: Vec3, angVelB: Vec3, movingA: boolean, movingB: boolean, axis: Vec3): number;
/**
 * apply a total lambda value to cached velocity locals.
 * velocity-local version — writes to cached velocity vectors instead of body properties.
 * does NOT apply DOF masking — the caller handles that once after all constraint parts are solved.
 *
 * @param part the constraint part
 * @param linVelA linear velocity of body A (local copy, mutated)
 * @param angVelA angular velocity of body A (local copy, mutated)
 * @param linVelB linear velocity of body B (local copy, mutated)
 * @param angVelB angular velocity of body B (local copy, mutated)
 * @param isDynamicA true if body A is dynamic
 * @param isDynamicB true if body B is dynamic
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param axis constraint axis
 * @param totalLambda new total lambda to apply
 * @returns true if impulse was applied
 *
 * @inline
 * @optimize
 */
export declare function applyLambda(part: AxisConstraintPart, linVelA: Vec3, angVelA: Vec3, linVelB: Vec3, angVelB: Vec3, isDynamicA: boolean, isDynamicB: boolean, invMassA: number, invMassB: number, axis: Vec3, totalLambda: number): boolean;
/**
 * apply warm start impulse to cached velocity locals.
 * velocity-local version — writes to cached velocity vectors instead of body properties.
 * does NOT apply DOF masking — the caller handles that once after all constraint parts are warmed.
 *
 * @param part the constraint part
 * @param linVelA linear velocity of body A (local copy, mutated)
 * @param angVelA angular velocity of body A (local copy, mutated)
 * @param linVelB linear velocity of body B (local copy, mutated)
 * @param angVelB angular velocity of body B (local copy, mutated)
 * @param isDynamicA true if body A is dynamic
 * @param isDynamicB true if body B is dynamic
 * @param invMassA inverse mass of body A
 * @param invMassB inverse mass of body B
 * @param axis constraint axis
 * @param warmStartRatio scale factor for warm start (dt_new / dt_old)
 * @returns true if impulse was applied
 *
 * @inline
 * @optimize
 */
export declare function warmStart(part: AxisConstraintPart, linVelA: Vec3, angVelA: Vec3, linVelB: Vec3, angVelB: Vec3, isDynamicA: boolean, isDynamicB: boolean, invMassA: number, invMassB: number, axis: Vec3, warmStartRatio: number): boolean;
