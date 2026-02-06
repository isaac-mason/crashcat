/**
 * Mode for combining material properties (friction, restitution) when two bodies collide.
 * Determines how to calculate the effective value from both bodies' properties.
 */
export enum MaterialCombineMode {
    /** average: (a + b) / 2 */
    AVERAGE = 0,
    /** multiply: a * b */
    MULTIPLY = 1,
    /** minimum: min(a, b) - Use the smaller value */
    MIN = 2,
    /** maximum: max(a, b) - Use the larger value */
    MAX = 3,
    /** geometric mean: sqrt(a * b) */
    GEOMETRIC_MEAN = 4,
}

// determine which mode to use (pick "more restrictive" when they differ)
// this matches typical physics engine behavior
const modePriority = {
    [MaterialCombineMode.MAX]: 4,
    [MaterialCombineMode.MIN]: 3,
    [MaterialCombineMode.GEOMETRIC_MEAN]: 2,
    [MaterialCombineMode.MULTIPLY]: 1,
    [MaterialCombineMode.AVERAGE]: 0,
};

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
export function combineMaterial(valueA: number, valueB: number, modeA: MaterialCombineMode, modeB: MaterialCombineMode): number {
    const mode = modePriority[modeA] >= modePriority[modeB] ? modeA : modeB;

    switch (mode) {
        case MaterialCombineMode.MULTIPLY:
            return valueA * valueB;
        case MaterialCombineMode.MIN:
            return Math.min(valueA, valueB);
        case MaterialCombineMode.MAX:
            return Math.max(valueA, valueB);
        case MaterialCombineMode.GEOMETRIC_MEAN:
            return Math.sqrt(valueA * valueB);
        // fallback to average
        // case MaterialCombineMode.AVERAGE:
        default:
            return (valueA + valueB) / 2;
    }
}
