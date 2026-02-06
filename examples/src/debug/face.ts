import * as THREE from 'three';
import type { Face } from 'crashcat';

export function createFaceGeometry(face: Face): THREE.BufferGeometry {
    const geometry = new THREE.BufferGeometry();
    const positions: number[] = [];

    for (let i = 0; i < face.numVertices * 3; i++) {
        positions.push(face.vertices[i]);
    }

    const indices: number[] = [];
    for (let i = 0; i < face.numVertices; i++) {
        indices.push(i, (i + 1) % face.numVertices);
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
    geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(indices), 1));
    return geometry;
}
