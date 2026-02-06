import type { SpringSettings } from './spring-settings';
import * as springSettings from './spring-settings';

/** motor state for controlling constraint motors (hinge, slider, etc.) */
export enum MotorState {
    /** Motor off, can free spin */
    OFF = 0,
    /** Motor drives to target velocity */
    VELOCITY = 1,
    /** Motor drives to target position */
    POSITION = 2,
}

/** settings for a constraint motor */
export type MotorSettings = {
    /**
     * Settings for the spring that is used to drive to the position target.
     * Not used when motor is a velocity motor.
     */
    springSettings: SpringSettings;

    /**
     * Minimum force to apply in case of a linear constraint (N).
     * Usually this is -maxForceLimit unless you want a motor that can e.g. push but not pull.
     * Not used when motor is an angular motor.
     */
    minForceLimit: number;

    /**
     * Maximum force to apply in case of a linear constraint (N).
     * Not used when motor is an angular motor.
     */
    maxForceLimit: number;

    /**
     * Minimum torque to apply in case of an angular constraint (N*m).
     * Usually this is -maxTorqueLimit unless you want a motor that can e.g. push but not pull.
     * Not used when motor is a position motor.
     */
    minTorqueLimit: number;

    /**
     * Maximum torque to apply in case of an angular constraint (N*m).
     * Not used when motor is a position motor.
     */
    maxTorqueLimit: number;
};

/**
 * Create default motor settings.
 * Defaults to frequency/damping mode with 2 Hz frequency and 1.0 damping ratio.
 */
export function create(): MotorSettings {
    const settings = springSettings.create();
    settings.mode = springSettings.SpringMode.FREQUENCY_AND_DAMPING;
    settings.frequencyOrStiffness = 2.0;
    settings.damping = 1.0;

    return {
        springSettings: settings,
        minForceLimit: -Infinity,
        maxForceLimit: Infinity,
        minTorqueLimit: -Infinity,
        maxTorqueLimit: Infinity,
    };
}

/** Reset motor settings to default values. */
export function reset(out: MotorSettings): MotorSettings {
    out.springSettings.mode = springSettings.SpringMode.FREQUENCY_AND_DAMPING;
    out.springSettings.frequencyOrStiffness = 2.0;
    out.springSettings.damping = 1.0;
    out.minForceLimit = -Infinity;
    out.maxForceLimit = Infinity;
    out.minTorqueLimit = -Infinity;
    out.maxTorqueLimit = Infinity;
    return out;
}

/**
 * Set symmetric force limits.
 * Sets minForceLimit = -limit and maxForceLimit = limit.
 */
export function setForceLimit(settings: MotorSettings, limit: number): void {
    settings.minForceLimit = -limit;
    settings.maxForceLimit = limit;
}

/**
 * Set asymmetric force limits.
 */
export function setForceLimits(settings: MotorSettings, min: number, max: number): void {
    settings.minForceLimit = min;
    settings.maxForceLimit = max;
}

/**
 * Set symmetric torque limits.
 * Sets minTorqueLimit = -limit and maxTorqueLimit = limit.
 */
export function setTorqueLimit(settings: MotorSettings, limit: number): void {
    settings.minTorqueLimit = -limit;
    settings.maxTorqueLimit = limit;
}

/**
 * Set asymmetric torque limits.
 */
export function setTorqueLimits(settings: MotorSettings, min: number, max: number): void {
    settings.minTorqueLimit = min;
    settings.maxTorqueLimit = max;
}

/**
 * Check if motor settings are valid.
 */
export function isValid(settings: MotorSettings): boolean {
    return (
        settings.springSettings.frequencyOrStiffness >= 0 &&
        settings.springSettings.damping >= 0 &&
        settings.minForceLimit <= settings.maxForceLimit &&
        settings.minTorqueLimit <= settings.maxTorqueLimit
    );
}

/**
 * Check if the motor spring has valid frequency/stiffness (soft constraint).
 */
export function hasStiffness(settings: MotorSettings): boolean {
    return settings.springSettings.frequencyOrStiffness > 0;
}

/**
 * Copy motor settings from a partial source to a target.
 * Only copies properties that are defined in the source.
 */
export function copy(out: MotorSettings, source: MotorSettings): void {
    springSettings.copy(out.springSettings, source.springSettings);
    out.minForceLimit = source.minForceLimit;
    out.maxForceLimit = source.maxForceLimit;
    out.minTorqueLimit = source.minTorqueLimit;
    out.maxTorqueLimit = source.maxTorqueLimit;
}
