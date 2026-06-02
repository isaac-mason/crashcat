import type { SpringSettings } from './spring-settings.js';
/** motor state for controlling constraint motors (hinge, slider, etc.) */
export declare enum MotorState {
    /** Motor off, can free spin */
    OFF = 0,
    /** Motor drives to target velocity */
    VELOCITY = 1,
    /** Motor drives to target position */
    POSITION = 2
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
export declare function create(): MotorSettings;
/** Reset motor settings to default values. */
export declare function reset(out: MotorSettings): MotorSettings;
/**
 * Set symmetric force limits.
 * Sets minForceLimit = -limit and maxForceLimit = limit.
 */
export declare function setForceLimit(settings: MotorSettings, limit: number): void;
/**
 * Set asymmetric force limits.
 */
export declare function setForceLimits(settings: MotorSettings, min: number, max: number): void;
/**
 * Set symmetric torque limits.
 * Sets minTorqueLimit = -limit and maxTorqueLimit = limit.
 */
export declare function setTorqueLimit(settings: MotorSettings, limit: number): void;
/**
 * Set asymmetric torque limits.
 */
export declare function setTorqueLimits(settings: MotorSettings, min: number, max: number): void;
/**
 * Check if motor settings are valid.
 */
export declare function isValid(settings: MotorSettings): boolean;
/**
 * Check if the motor spring has valid frequency/stiffness (soft constraint).
 */
export declare function hasStiffness(settings: MotorSettings): boolean;
/**
 * Copy motor settings from a partial source to a target.
 * Only copies properties that are defined in the source.
 */
export declare function copy(out: MotorSettings, source: MotorSettings): void;
