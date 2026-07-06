import { type SpringSettings } from './spring-settings.js';
/**
 * Class used in constraint parts to calculate the required bias factor
 * in the lagrange multiplier for creating springs.
 *
 * Implements soft constraints as per:
 * "Soft Constraints: Reinventing The Spring" - Erin Catto - GDC 2011
 */
export type SpringPart = {
    /** Bias term for the constraint impulse: lambda = J v + b */
    bias: number;
    /**
     * Softness (gamma in Erin Catto's slides).
     * softness = 1 / (dt * (c + dt * k))
     * where k = spring stiffness, c = spring damping
     */
    softness: number;
};
/** Create a new SpringPart with zero values (hard constraint) */
export declare function createSpringPart(): SpringPart;
/** Reset a SpringPart to zero values (hard constraint) */
export declare function resetSpringPart(part: SpringPart): void;
/**
 * Turn off the spring and set a bias only (hard constraint).
 *
 * @param part - Spring part to update
 * @param bias - Bias term (b) for the constraint impulse: lambda = J v + b
 */
export declare function calculateSpringPropertiesWithBias(part: SpringPart, bias: number): void;
/**
 * Calculate spring properties based on frequency and damping ratio.
 *
 * @param part - Spring part to update
 * @param deltaTime - Time step
 * @param invEffectiveMass - Inverse effective mass K
 * @param bias - Bias term (b) for the constraint impulse: lambda = J v + b
 * @param C - Value of the constraint equation (C). Set to zero if you don't want to drive the constraint to zero with a spring.
 * @param frequency - Oscillation frequency (Hz). Set to zero if you don't want to drive the constraint to zero with a spring.
 * @param damping - Damping factor (0 = no damping, 1 = critical damping). Set to zero if you don't want to drive the constraint to zero with a spring.
 * @returns Updated effective mass K^-1 (after spring adjustment)
 */
export declare function calculateSpringPropertiesWithFrequencyAndDamping(part: SpringPart, deltaTime: number, invEffectiveMass: number, bias: number, C: number, frequency: number, damping: number): number;
/**
 * Calculate spring properties with spring stiffness (k) and damping (c).
 * This is based on the spring equation: F = -k * x - c * v
 *
 * @param part - Spring part to update
 * @param deltaTime - Time step
 * @param invEffectiveMass - Inverse effective mass K
 * @param bias - Bias term (b) for the constraint impulse: lambda = J v + b
 * @param C - Value of the constraint equation (C). Set to zero if you don't want to drive the constraint to zero with a spring.
 * @param stiffness - Spring stiffness k. Set to zero if you don't want to drive the constraint to zero with a spring.
 * @param damping - Spring damping coefficient c. Set to zero if you don't want to drive the constraint to zero with a spring.
 * @returns Updated effective mass K^-1 (after spring adjustment)
 */
export declare function calculateSpringPropertiesWithStiffnessAndDamping(part: SpringPart, deltaTime: number, invEffectiveMass: number, bias: number, C: number, stiffness: number, damping: number): number;
/**
 * Calculate spring properties using SpringSettings.
 * Selects the appropriate calculation method based on the spring mode.
 *
 * @param part - Spring part to update
 * @param deltaTime - Time step
 * @param invEffectiveMass - Inverse effective mass K
 * @param bias - Bias term (b) for the constraint impulse: lambda = J v + b
 * @param C - Value of the constraint equation (C)
 * @param settings - Spring settings (mode, frequency/stiffness, damping)
 * @returns Updated effective mass K^-1 (after spring adjustment)
 */
export declare function calculateSpringPropertiesWithSettings(part: SpringPart, deltaTime: number, invEffectiveMass: number, bias: number, C: number, settings: SpringSettings): number;
/**
 * Returns if this spring is active (soft constraint).
 */
export declare function isSpringActive(part: SpringPart): boolean;
/**
 * Get total bias b, including supplied bias and bias for spring: lambda = J v + b
 *
 * From Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
 *
 * Each iteration we are not computing the whole impulse, we are computing an increment
 * to the impulse and we are updating the velocity. Also, as we solve each constraint we
 * get a perfect v2, but then some other constraint will come along and mess it up.
 * So we want to patch up the constraint while acknowledging the accumulated impulse and
 * the damaged velocity. To help with that we use P for the accumulated impulse and lambda
 * as the update. Mathematically we have:
 *
 * M * (v2new - v2damaged) = J^T * lambda
 * J * v2new + softness * (total_lambda + lambda) + b = 0
 *
 * If we solve this we get:
 *
 * v2new = v2damaged + M^-1 * J^T * lambda
 * J * (v2damaged + M^-1 * J^T * lambda) + softness * total_lambda + softness * lambda + b = 0
 *
 * (J * M^-1 * J^T + softness) * lambda = -(J * v2damaged + softness * total_lambda + b)
 *
 * So our lagrange multiplier becomes:
 *
 * lambda = -K^-1 (J v + softness * total_lambda + b)
 *
 * So we return the bias: softness * total_lambda + b
 *
 * @param part - Spring part
 * @param totalLambda - Total accumulated lambda from previous iterations
 * @returns Total bias for the constraint
 */
export declare function getSpringBias(part: SpringPart, totalLambda: number): number;
