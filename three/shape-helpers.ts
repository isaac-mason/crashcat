import * as THREE from 'three';
import {
    type BoxShape,
    type CapsuleShape,
    type CompoundShape,
    type ConvexHullShape,
    type CylinderShape,
    type PlaneShape,
    type ScaledShape,
    type Shape,
    ShapeType,
    type SphereShape,
    type TransformedShape,
    type TriangleMeshShape,
} from 'crashcat';

export type ShapeHelperOptions = {
    material?: THREE.Material;
}

export type ShapeHelper = {
    object: THREE.Object3D;
    dispose: () => void;
}

export function createShapeHelper(shape: Shape, options?: ShapeHelperOptions): ShapeHelper {
    switch (shape.type) {
        case ShapeType.SPHERE:
            return createSphereHelper(shape, options);
        case ShapeType.BOX:
            return createBoxHelper(shape, options);
        case ShapeType.CAPSULE:
            return createCapsuleHelper(shape, options);
        case ShapeType.CYLINDER:
            return createCylinderHelper(shape, options);
        case ShapeType.CONVEX_HULL:
            return createConvexHullHelper(shape, options);
        case ShapeType.TRIANGLE_MESH:
            return createTriangleMeshHelper(shape, options);
        case ShapeType.COMPOUND:
            return createCompoundHelper(shape, options);
        case ShapeType.TRANSFORMED:
            return createTransformedHelper(shape, options);
        case ShapeType.SCALED:
            return createScaledHelper(shape, options);
        case ShapeType.PLANE:
            return createPlaneHelper(shape, options);
        default:
            throw new Error(`Unsupported shape type: ${(shape as Shape).type}`);
    }
}

function createSphereHelper(shape: SphereShape, options?: ShapeHelperOptions): ShapeHelper {
    const geometry = new THREE.SphereGeometry(shape.radius, 32, 32);
    const material = options?.material || new THREE.MeshStandardMaterial({ color: 0x888888, wireframe: false });
    const mesh = new THREE.Mesh(geometry, material);

    return {
        object: mesh,
        dispose: () => {
            geometry.dispose();
            if (!options?.material) {
                material.dispose();
            }
        },
    };
}

function createBoxHelper(shape: BoxShape, options?: ShapeHelperOptions): ShapeHelper {
    const [he0, he1, he2] = shape.halfExtents;
    const geometry = new THREE.BoxGeometry(he0 * 2, he1 * 2, he2 * 2);
    const material = options?.material || new THREE.MeshStandardMaterial({ color: 0x888888, wireframe: false });
    const mesh = new THREE.Mesh(geometry, material);

    return {
        object: mesh,
        dispose: () => {
            geometry.dispose();
            if (!options?.material) {
                material.dispose();
            }
        },
    };
}

function createCapsuleHelper(shape: CapsuleShape, options?: ShapeHelperOptions): ShapeHelper {
    // Three.js CapsuleGeometry params: (radius, length, capSegments, radialSegments)
    // length is the height of the cylindrical portion (not including caps)
    const geometry = new THREE.CapsuleGeometry(shape.radius, shape.halfHeightOfCylinder * 2, 16, 32);
    const material = options?.material || new THREE.MeshStandardMaterial({ color: 0x888888, wireframe: false });
    const mesh = new THREE.Mesh(geometry, material);

    return {
        object: mesh,
        dispose: () => {
            geometry.dispose();
            if (!options?.material) {
                material.dispose();
            }
        },
    };
}

function createCylinderHelper(shape: CylinderShape, options?: ShapeHelperOptions): ShapeHelper {
    // Three.js CylinderGeometry params: (radiusTop, radiusBottom, height, radialSegments)
    const geometry = new THREE.CylinderGeometry(shape.radius, shape.radius, shape.halfHeight * 2, 64);
    const material = options?.material || new THREE.MeshStandardMaterial({ color: 0x888888, wireframe: false });
    const mesh = new THREE.Mesh(geometry, material);

    return {
        object: mesh,
        dispose: () => {
            geometry.dispose();
            if (!options?.material) {
                material.dispose();
            }
        },
    };
}

function createConvexHullHelper(shape: ConvexHullShape, options?: ShapeHelperOptions): ShapeHelper {
    const geometry = new THREE.BufferGeometry();
    const vertices: number[] = [];
    const indices: number[] = [];

    // Build geometry from faces
    for (const face of shape.faces) {
        const faceVertices: number[][] = [];
        
        // Collect vertex positions for this face
        for (let i = 0; i < face.numVertices; i++) {
            const vertexIdx = shape.vertexIndices[face.firstVertex + i];
            const point = shape.points[vertexIdx];
            faceVertices.push([point.position[0], point.position[1], point.position[2]]);
        }

        // Triangulate the face (simple fan triangulation from first vertex)
        if (faceVertices.length >= 3) {
            const baseIdx = vertices.length / 3;
            
            // Add all vertices
            for (const v of faceVertices) {
                vertices.push(v[0], v[1], v[2]);
            }
            
            // Create triangle fan
            for (let i = 1; i < faceVertices.length - 1; i++) {
                indices.push(baseIdx, baseIdx + i, baseIdx + i + 1);
            }
        }
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(vertices), 3));
    if (indices.length > 0) {
        geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(indices), 1));
        geometry.computeVertexNormals();
    }

    const material = options?.material || new THREE.MeshStandardMaterial({ color: 0x888888, wireframe: false });
    const mesh = new THREE.Mesh(geometry, material);

    return {
        object: mesh,
        dispose: () => {
            geometry.dispose();
            if (!options?.material) {
                material.dispose();
            }
        },
    };
}

function createTriangleMeshHelper(shape: TriangleMeshShape, options?: ShapeHelperOptions): ShapeHelper {
    const geometry = new THREE.BufferGeometry();

    // Use deduplicated positions directly
    const positions = new Float32Array(shape.data.positions);
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    // Build index buffer from triangle buffer
    const indexCount = shape.data.triangleCount * 3;
    const indices = new Uint32Array(indexCount);

    for (let i = 0; i < shape.data.triangleCount; i++) {
        const offset = i * 8; // TRIANGLE_STRIDE = 8
        indices[i * 3 + 0] = shape.data.triangleBuffer[offset + 0]; // OFFSET_INDEX_A
        indices[i * 3 + 1] = shape.data.triangleBuffer[offset + 1]; // OFFSET_INDEX_B
        indices[i * 3 + 2] = shape.data.triangleBuffer[offset + 2]; // OFFSET_INDEX_C
    }

    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    geometry.computeVertexNormals();

    const material = options?.material || new THREE.MeshStandardMaterial({ color: 0x888888, wireframe: false });
    const mesh = new THREE.Mesh(geometry, material);

    return {
        object: mesh,
        dispose: () => {
            geometry.dispose();
            if (!options?.material) {
                material.dispose();
            }
        },
    };
}

function createCompoundHelper(shape: CompoundShape, options?: ShapeHelperOptions): ShapeHelper {
    const group = new THREE.Group();
    const childHelpers: ShapeHelper[] = [];

    for (const child of shape.children) {
        const childHelper = createShapeHelper(child.shape, options);
        
        // Apply child position and rotation
        childHelper.object.position.set(child.position[0], child.position[1], child.position[2]);
        childHelper.object.quaternion.set(
            child.quaternion[0],
            child.quaternion[1],
            child.quaternion[2],
            child.quaternion[3],
        );
        
        childHelpers.push(childHelper);
        group.add(childHelper.object);
    }

    return {
        object: group,
        dispose: () => {
            for (const childHelper of childHelpers) {
                childHelper.dispose();
            }
        },
    };
}

function createTransformedHelper(shape: TransformedShape, options?: ShapeHelperOptions): ShapeHelper {
    const group = new THREE.Group();
    const innerHelper = createShapeHelper(shape.shape, options);

    // Apply translation
    group.position.set(shape.position[0], shape.position[1], shape.position[2]);

    // Apply rotation
    group.quaternion.set(shape.quaternion[0], shape.quaternion[1], shape.quaternion[2], shape.quaternion[3]);

    group.add(innerHelper.object);

    return {
        object: group,
        dispose: () => {
            innerHelper.dispose();
        },
    };
}

function createScaledHelper(shape: ScaledShape, options?: ShapeHelperOptions): ShapeHelper {
    const group = new THREE.Group();
    const innerHelper = createShapeHelper(shape.shape, options);

    // Apply scale
    group.scale.set(shape.scale[0], shape.scale[1], shape.scale[2]);

    group.add(innerHelper.object);

    return {
        object: group,
        dispose: () => {
            innerHelper.dispose();
        },
    };
}

function createPlaneHelper(shape: PlaneShape, options?: ShapeHelperOptions): ShapeHelper {
    // create a large thin box to represent the plane
    // size based on halfExtent from the plane shape
    const size = shape.halfExtent * 2;
    
    const geometry = new THREE.PlaneGeometry(size, size);
    const material = options?.material || new THREE.MeshStandardMaterial({ 
        color: 0x888888, 
        wireframe: false,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.5
    });
    const mesh = new THREE.Mesh(geometry, material);
    
    // orient the plane based on its normal
    const normal = shape.plane.normal;
    
    // create rotation from Y-axis to plane normal
    const up = new THREE.Vector3(0, 1, 0);
    const target = new THREE.Vector3(normal[0], normal[1], normal[2]);
    const quaternion = new THREE.Quaternion();
    quaternion.setFromUnitVectors(up, target);
    mesh.quaternion.copy(quaternion);
    
    // position plane at distance from origin along normal
    const distance = -shape.plane.constant;
    mesh.position.set(
        normal[0] * distance,
        normal[1] * distance,
        normal[2] * distance
    );

    return {
        object: mesh,
        dispose: () => {
            geometry.dispose();
            if (!options?.material) {
                material.dispose();
            }
        },
    };
}
