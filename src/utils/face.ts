import type { Quat, Vec3 } from 'mathcat';

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

export function transformFace(face: Face, position: Vec3, quaternion: Quat, scale: Vec3): void {
    // inline mat4.fromRotationTranslationScale to build transformation matrix
    const qx = quaternion[0];
    const qy = quaternion[1];
    const qz = quaternion[2];
    const qw = quaternion[3];

    const qx2 = qx + qx;
    const qy2 = qy + qy;
    const qz2 = qz + qz;
    const qxx = qx * qx2;
    const qyx = qy * qx2;
    const qyy = qy * qy2;
    const qzx = qz * qx2;
    const qzy = qz * qy2;
    const qzz = qz * qz2;
    const qwx = qw * qx2;
    const qwy = qw * qy2;
    const qwz = qw * qz2;
    
    const sx = scale[0];
    const sy = scale[1];
    const sz = scale[2];
    
    // transformation matrix columns (rotation × scale)
    // column 0
    const m00 = (1 - qyy - qzz) * sx;
    const m01 = (qyx + qwz) * sx;
    const m02 = (qzx - qwy) * sx;
    // column 1
    const m10 = (qyx - qwz) * sy;
    const m11 = (1 - qxx - qzz) * sy;
    const m12 = (qzy + qwx) * sy;
    // column 2
    const m20 = (qzx + qwy) * sz;
    const m21 = (qzy - qwx) * sz;
    const m22 = (1 - qxx - qyy) * sz;
    // column 3 (translation)
    const m30 = position[0];
    const m31 = position[1];
    const m32 = position[2];

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

/** check if scale produces inside-out geometry (negative determinant) */
export function isScaleInsideOut(scale: Vec3): boolean {
    return scale[0] * scale[1] * scale[2] < 0;
}
