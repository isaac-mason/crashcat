/** spring mode enum - specifies how the spring is defined */
export enum SpringMode {
    /** frequency and damping are specified (oscillation frequency in Hz) */
    FREQUENCY_AND_DAMPING = 0,
    /** stiffness and damping are specified (spring equation: F = -k * x - c * v) */
    STIFFNESS_AND_DAMPING = 1,
}

/** settings for a linear or angular spring */
export type SpringSettings = {
    /**
     * Selects the way in which the spring is defined.
     * If mode is StiffnessAndDamping, frequencyOrStiffness becomes the stiffness (k)
     * and damping becomes the damping ratio (c) in: F = -k * x - c * v
     */
    mode: SpringMode;

    /**
     * When mode = FrequencyAndDamping:
     *   If > 0: constraint is soft, this specifies oscillation frequency in Hz
     *   If <= 0: damping is ignored, constraint has hard limits
     *
     * When mode = StiffnessAndDamping:
     *   If > 0: constraint is soft, this specifies stiffness (k) in spring equation
     *   If <= 0: damping is ignored, constraint has hard limits
     *
     * Note: Stiffness values are large numbers. Ballpark calculation:
     * force = stiffness * delta_spring_length = mass * gravity
     * => stiffness = mass * gravity / delta_spring_length
     * Example: 1500 kg object, 2m compression => stiffness ~ 1500 * 9.81 / 2 ~ 7500 N/m
     */
    frequencyOrStiffness: number;

    /**
     * When mode = FrequencyAndDamping:
     *   Damping ratio (0 = no damping, 1 = critical damping)
     *
     * When mode = StiffnessAndDamping:
     *   Damping coefficient (c) in spring equation: F = -k * x - c * v
     *
     * Note: Even with damping = 0, there's energy loss due to explicit Euler integration.
     * This prevents infinite oscillation and simulation explosion.
     */
    damping: number;
};

/** create default spring settings (hard constraint, no spring) */
export function create(): SpringSettings {
    return {
        mode: SpringMode.FREQUENCY_AND_DAMPING,
        frequencyOrStiffness: 0,
        damping: 0,
    };
}

/** copy spring settings from source to out */
export function copy(out: SpringSettings, source: SpringSettings) {
    out.mode = source.mode;
    out.frequencyOrStiffness = source.frequencyOrStiffness;
    out.damping = source.damping;
}

/** check if the spring has valid frequency/stiffness (soft constraint), if false, the spring will be hard */
export function hasStiffness(settings: SpringSettings): boolean {
    return settings.frequencyOrStiffness > 0;
}
