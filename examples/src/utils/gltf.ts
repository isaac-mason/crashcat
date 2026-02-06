import * as THREE from 'three';
import { GLTFLoader, type GLTF } from 'three/addons/loaders/GLTFLoader.js';

export async function loadGLTFPoints(path: string): Promise<number[]> {
    const gltfLoader = new GLTFLoader();
    const gltf = await gltfLoader.loadAsync(path);

    // Find the first mesh in the scene hierarchy
    let mesh: THREE.Mesh | undefined;

    gltf.scene.traverse((object) => {
        if (!mesh && object instanceof THREE.Mesh) {
            mesh = object;
        }
    });

    if (!mesh) {
        throw new Error('No mesh found in GLTF file');
    }

    const geometry = mesh.geometry as THREE.BufferGeometry;
    const positions = geometry.getAttribute('position');

    const points: number[] = [];

    for (let i = 0; i < positions.count; i++) {
        points.push(positions.getX(i), positions.getY(i), positions.getZ(i));
    }

    return points;
}

const gltfLoader = new GLTFLoader();

export async function loadGLTF(path: string): Promise<GLTF> {
    return await gltfLoader.loadAsync(path);
}
