import type { Mat4, Vec3 } from 'mathcat';

export type Face = {
    /**
     * the face vertices as a flat array of numbers
     * [x1, y1, z1, x2, y2, z2, ...]
     */
    vertices: number[];

    /** the number of vertices in the face */
    numVertices: number;
};

export const FACE_MAX_VERTICES = 64;

export function createFace(): Face {
    return {
        vertices: Array(FACE_MAX_VERTICES * 3).fill(0.0),
        numVertices: 0,
    };
}

export function cloneFace(face: Face): Face {
    return {
        vertices: [...face.vertices.slice(0, face.numVertices * 3)],
        numVertices: face.numVertices,
    };
}

/**
 * transform face vertices using a pre-computed transformation matrix (rotation + translation only, no scale).
 * assumes vertices are already scaled.
 */
export function transformFaceWithMat4RotationTranslation(face: Face, matrix: Mat4): void {
    // extract matrix components for affine transformation
    const m00 = matrix[0];
    const m01 = matrix[1];
    const m02 = matrix[2];
    const m10 = matrix[4];
    const m11 = matrix[5];
    const m12 = matrix[6];
    const m20 = matrix[8];
    const m21 = matrix[9];
    const m22 = matrix[10];
    const m30 = matrix[12];
    const m31 = matrix[13];
    const m32 = matrix[14];

    // apply transformation to each vertex
    for (let i = 0; i < face.numVertices; i++) {
        const idx = i * 3;
        const x = face.vertices[idx];
        const y = face.vertices[idx + 1];
        const z = face.vertices[idx + 2];

        // mat4 × vec3 (affine transformation)
        face.vertices[idx] = m00 * x + m10 * y + m20 * z + m30;
        face.vertices[idx + 1] = m01 * x + m11 * y + m21 * z + m31;
        face.vertices[idx + 2] = m02 * x + m12 * y + m22 * z + m32;
    }
}

/**
 * transform face vertices using a rotation+translation matrix plus separate scale.
 * the matrix provides rotation and translation, scale is applied separately.
 * this allows callers to pass pre-computed rotation+translation matrices.
 */
export function transformFaceWithMat4Scale(face: Face, matrix: Mat4, scale: Vec3): void {
    // extract rotation from matrix and combine with scale
    const sx = scale[0],
        sy = scale[1],
        sz = scale[2];
    const m00 = matrix[0] * sx;
    const m01 = matrix[1] * sx;
    const m02 = matrix[2] * sx;
    const m10 = matrix[4] * sy;
    const m11 = matrix[5] * sy;
    const m12 = matrix[6] * sy;
    const m20 = matrix[8] * sz;
    const m21 = matrix[9] * sz;
    const m22 = matrix[10] * sz;
    const m30 = matrix[12];
    const m31 = matrix[13];
    const m32 = matrix[14];

    // apply transformation to each vertex
    for (let i = 0; i < face.numVertices; i++) {
        const idx = i * 3;
        const x = face.vertices[idx];
        const y = face.vertices[idx + 1];
        const z = face.vertices[idx + 2];

        // mat4 × vec3 (affine transformation with scale)
        face.vertices[idx] = m00 * x + m10 * y + m20 * z + m30;
        face.vertices[idx + 1] = m01 * x + m11 * y + m21 * z + m31;
        face.vertices[idx + 2] = m02 * x + m12 * y + m22 * z + m32;
    }
}

/** check if scale produces inside-out geometry (negative determinant) */
export function isScaleInsideOut(scale: Vec3): boolean {
    return scale[0] * scale[1] * scale[2] < 0;
}
