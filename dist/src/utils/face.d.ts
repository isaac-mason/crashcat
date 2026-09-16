import type { Mat4, Vec3 } from 'math';
export type Face = {
    /**
     * the face vertices as a flat array of numbers
     * [x1, y1, z1, x2, y2, z2, ...]
     */
    vertices: number[];
    /** the number of vertices in the face */
    numVertices: number;
};
export declare const FACE_MAX_VERTICES = 64;
export declare function createFace(): Face;
export declare function cloneFace(face: Face): Face;
/**
 * transform face vertices using a pre-computed transformation matrix (rotation + translation only, no scale).
 * assumes vertices are already scaled.
 */
export declare function transformFaceWithMat4RotationTranslation(face: Face, matrix: Mat4): void;
/**
 * transform face vertices using a rotation+translation matrix plus separate scale.
 * the matrix provides rotation and translation, scale is applied separately.
 * this allows callers to pass pre-computed rotation+translation matrices.
 */
export declare function transformFaceWithMat4Scale(face: Face, matrix: Mat4, scale: Vec3): void;
/** check if scale produces inside-out geometry (negative determinant) */
export declare function isScaleInsideOut(scale: Vec3): boolean;
