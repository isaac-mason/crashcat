import { SpringMode, type SpringSettings } from './spring-settings';

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
export function createSpringPart(): SpringPart {
    return {
        bias: 0,
        softness: 0,
    };
}

/** Reset a SpringPart to zero values (hard constraint) */
export function resetSpringPart(part: SpringPart): void {
    part.bias = 0;
    part.softness = 0;
}

/**
 * Helper function to calculate spring properties with stiffness and damping.
 * 
 * Note: The calculation of beta and gamma below are based on the solution of an
 * implicit Euler integration scheme. This scheme is unconditionally stable but
 * has built-in damping, so even when you set the damping ratio to 0 there will
 * still be damping. See Erin Catto GDC 2011 slides page 16 and 32.
 * 
 * @param part spring part to update
 * @param deltaTime time step
 * @param invEffectiveMass inverse effective mass K (before spring adjustment)
 * @param bias bias term (b) for the constraint impulse
 * @param C value of the constraint equation (C)
 * @param stiffness spring stiffness k
 * @param damping spring damping coefficient c
 * @returns updated effective mass K^-1 (after spring adjustment)
 */
function calculateSpringPropertiesHelper(
    part: SpringPart,
    deltaTime: number,
    invEffectiveMass: number,
    bias: number,
    C: number,
    stiffness: number,
    damping: number,
): number {
    // Calculate softness (gamma in the slides)
    // See page 34 and note that the gamma needs to be divided by delta time
    // since we're working with impulses rather than forces:
    // softness = 1 / (dt * (c + dt * k))
    // Note that the spring stiffness is k and the spring damping is c
    part.softness = 1.0 / (deltaTime * (damping + deltaTime * stiffness));

    // Calculate bias factor (baumgarte stabilization):
    // beta = dt * k / (c + dt * k) = dt * k^2 * softness
    // b = beta / dt * C = dt * k * softness * C
    part.bias = bias + deltaTime * stiffness * part.softness * C;

    // Update the effective mass
    // See post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
    //
    // Newton's Law:
    // M * (v2 - v1) = J^T * lambda
    //
    // Velocity constraint with softness and Baumgarte:
    // J * v2 + softness * lambda + b = 0
    //
    // where b = beta * C / dt
    //
    // We know everything except v2 and lambda.
    //
    // First solve Newton's law for v2 in terms of lambda:
    //
    // v2 = v1 + M^-1 * J^T * lambda
    //
    // Substitute this expression into the velocity constraint:
    //
    // J * (v1 + M^-1 * J^T * lambda) + softness * lambda + b = 0
    //
    // Now collect coefficients of lambda:
    //
    // (J * M^-1 * J^T + softness) * lambda = - J * v1 - b
    //
    // Now we define:
    //
    // K = J * M^-1 * J^T + softness
    //
    // So our new effective mass is K^-1
    return 1.0 / (invEffectiveMass + part.softness);
}

/**
 * Turn off the spring and set a bias only (hard constraint).
 * 
 * @param part - Spring part to update
 * @param bias - Bias term (b) for the constraint impulse: lambda = J v + b
 */
export function calculateSpringPropertiesWithBias(part: SpringPart, bias: number): void {
    part.softness = 0;
    part.bias = bias;
}

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
export function calculateSpringPropertiesWithFrequencyAndDamping(
    part: SpringPart,
    deltaTime: number,
    invEffectiveMass: number,
    bias: number,
    C: number,
    frequency: number,
    damping: number,
): number {
    const effectiveMass = 1.0 / invEffectiveMass;

    if (frequency > 0) {
        // Calculate angular frequency
        const omega = 2.0 * Math.PI * frequency;

        // Calculate spring stiffness k and damping constant c (page 45)
        const k = effectiveMass * omega * omega;
        const c = 2.0 * effectiveMass * damping * omega;

        return calculateSpringPropertiesHelper(part, deltaTime, invEffectiveMass, bias, C, k, c);
    } else {
        calculateSpringPropertiesWithBias(part, bias);
        return effectiveMass;
    }
}

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
export function calculateSpringPropertiesWithStiffnessAndDamping(
    part: SpringPart,
    deltaTime: number,
    invEffectiveMass: number,
    bias: number,
    C: number,
    stiffness: number,
    damping: number,
): number {
    if (stiffness > 0) {
        return calculateSpringPropertiesHelper(part, deltaTime, invEffectiveMass, bias, C, stiffness, damping);
    } else {
        const effectiveMass = 1.0 / invEffectiveMass;
        calculateSpringPropertiesWithBias(part, bias);
        return effectiveMass;
    }
}

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
export function calculateSpringPropertiesWithSettings(
    part: SpringPart,
    deltaTime: number,
    invEffectiveMass: number,
    bias: number,
    C: number,
    settings: SpringSettings,
): number {
    if (settings.mode === SpringMode.FREQUENCY_AND_DAMPING) {
        return calculateSpringPropertiesWithFrequencyAndDamping(
            part,
            deltaTime,
            invEffectiveMass,
            bias,
            C,
            settings.frequencyOrStiffness,
            settings.damping,
        );
    } else {
        return calculateSpringPropertiesWithStiffnessAndDamping(
            part,
            deltaTime,
            invEffectiveMass,
            bias,
            C,
            settings.frequencyOrStiffness,
            settings.damping,
        );
    }
}

/**
 * Returns if this spring is active (soft constraint).
 */
export function isSpringActive(part: SpringPart): boolean {
    return part.softness !== 0;
}

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
export function getSpringBias(part: SpringPart, totalLambda: number): number {
    return part.softness * totalLambda + part.bias;
}
