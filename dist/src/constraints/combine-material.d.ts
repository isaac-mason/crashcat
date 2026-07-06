/**
 * Mode for combining material properties (friction, restitution) when two bodies collide.
 * Determines how to calculate the effective value from both bodies' properties.
 */
export declare enum MaterialCombineMode {
    /** average: (a + b) / 2 */
    AVERAGE = 0,
    /** multiply: a * b */
    MULTIPLY = 1,
    /** minimum: min(a, b) - Use the smaller value */
    MIN = 2,
    /** maximum: max(a, b) - Use the larger value */
    MAX = 3,
    /** geometric mean: sqrt(a * b) */
    GEOMETRIC_MEAN = 4
}
/**
 * Combine two material property values based on the combine mode.
 *
 * @param valueA first body's material value
 * @param valueB second body's material value
 * @param modeA first body's combine mode
 * @param modeB second body's combine mode
 * @returns Combined value
 *
 * Note: When modes differ, we use the "more restrictive" mode:
 * Max > Min > GeometricMean > Multiply > Average
 */
export declare function combineMaterial(valueA: number, valueB: number, modeA: MaterialCombineMode, modeB: MaterialCombineMode): number;
