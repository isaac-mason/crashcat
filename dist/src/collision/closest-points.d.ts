import type { Vec3 } from 'mathcat';
export type BarycentricCoordinatesResult = {
    u: number;
    v: number;
    w: number;
    isValid: boolean;
};
export declare function createBarycentricCoordinatesResult(): BarycentricCoordinatesResult;
export declare function computeBarycentricCoordinates2d(out: BarycentricCoordinatesResult, a: Vec3, b: Vec3, squaredTolerance: number): void;
export declare function computeBarycentricCoordinates3d(out: BarycentricCoordinatesResult, a: Vec3, b: Vec3, c: Vec3, squaredTolerance: number): void;
