import type { Quat, Vec3 } from 'mathcat';
import { vec3 } from 'mathcat';

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

const _transformVertex = /* @__PURE__ */ vec3.create();

export function transformFace(face: Face, position: Vec3, quaternion: Quat, scale: Vec3): void {
    for (let i = 0; i < face.numVertices; i++) {
        // scale
        _transformVertex[0] = face.vertices[i * 3] * scale[0];
        _transformVertex[1] = face.vertices[i * 3 + 1] * scale[1];
        _transformVertex[2] = face.vertices[i * 3 + 2] * scale[2];

        // rotate
        vec3.transformQuat(_transformVertex, _transformVertex, quaternion);

        // translate
        vec3.add(_transformVertex, _transformVertex, position);

        // store
        face.vertices[i * 3] = _transformVertex[0];
        face.vertices[i * 3 + 1] = _transformVertex[1];
        face.vertices[i * 3 + 2] = _transformVertex[2];
    }
}

/** check if scale produces inside-out geometry (negative determinant) */
export function isScaleInsideOut(scale: Vec3): boolean {
    return scale[0] * scale[1] * scale[2] < 0;
}
