import { type Quat, quat, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import type {
    ConeConstraint,
    DistanceConstraint,
    FixedConstraint,
    HingeConstraint,
    PointConstraint,
    RigidBody,
    Shape,
    SixDOFConstraint,
    SliderConstraint,
    SwingTwistConstraint,
    World,
} from 'crashcat';
import { ConstraintType, MotionType, rigidBody, ShapeType, triangleMeshBvh } from 'crashcat';

export enum BodyColorMode {
    INSTANCE,
    MOTION_TYPE,
    SLEEPING,
    ISLAND,
}

type ShapeKey =
    | 'unit-sphere'
    | 'unit-box'
    | 'unit-capsule'
    | 'unit-cone'
    | 'unit-cylinder'
    | 'unit-plane'
    | `triangle-mesh:${number}`
    | `convex-hull:${number}`;

type GeometryCache = Map<
    ShapeKey,
    {
        geometry: THREE.BufferGeometry;
        geometryId: number;
        refCount: number;
    }
>;

function createUnitSphereGeometry(): THREE.BufferGeometry {
    return new THREE.SphereGeometry(1, 32, 32);
}

function createUnitBoxGeometry(): THREE.BufferGeometry {
    return new THREE.BoxGeometry(1, 1, 1);
}

function createUnitPlaneGeometry(): THREE.BufferGeometry {
    // unit plane: 1x1 square in XZ plane (Y-up), centered at origin
    // will be scaled by halfExtent*2 and rotated to match plane normal
    return new THREE.PlaneGeometry(1, 1, 1, 1);
}

function createUnitCapsuleGeometry(): THREE.BufferGeometry {
    // Unit capsule: radius 1, cylinder height 2 (halfHeight 1)
    // Total height is 4 (2 * halfHeight + 2 * radius)
    return new THREE.CapsuleGeometry(1, 2, 16, 32);
}

function createUnitConeGeometry(): THREE.BufferGeometry {
    return new THREE.ConeGeometry(0.5, 1, 8);
}

function createUnitCylinderGeometry(): THREE.BufferGeometry {
    return new THREE.CylinderGeometry(0.5, 0.5, 1, 16);
}

function createTriangleMeshGeometry(shape: Shape & { type: ShapeType.TRIANGLE_MESH }): THREE.BufferGeometry {
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

    // Generate dummy UVs (required by BatchedMesh)
    const vertexCount = positions.length / 3;
    const uvs = new Float32Array(vertexCount * 2);
    for (let i = 0; i < vertexCount; i++) {
        uvs[i * 2 + 0] = 0;
        uvs[i * 2 + 1] = 0;
    }
    geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));

    geometry.computeVertexNormals();

    return geometry;
}

function createConvexHullGeometry(shape: Shape & { type: ShapeType.CONVEX_HULL }): THREE.BufferGeometry {
    const geometry = new THREE.BufferGeometry();
    const vertices: number[] = [];
    const indices: number[] = [];
    const uvs: number[] = [];

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
                // Add dummy UVs (required by BatchedMesh)
                uvs.push(0, 0);
            }

            // Create triangle fan
            for (let i = 1; i < faceVertices.length - 1; i++) {
                indices.push(baseIdx, baseIdx + i, baseIdx + i + 1);
            }
        }
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(vertices), 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uvs), 2));
    if (indices.length > 0) {
        geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(indices), 1));
        geometry.computeVertexNormals();
    }

    return geometry;
}

type BodyInstance = {
    bodyId: number;
    shape: Shape;
    instanceIds: number[];
};

type EdgeWireframeInstance = {
    bodyId: number;
    shape: Shape;
    lineSegments: THREE.LineSegments;
    localTransforms: THREE.Matrix4[];
    geometries: THREE.EdgesGeometry[];
};

export type DebugRendererOptions = {
    bodies: {
        enabled: boolean;
        color: BodyColorMode;
        wireframe: boolean;
        showLinearVelocity: boolean;
        showAngularVelocity: boolean;
    };
    contacts: {
        enabled: boolean;
    };
    contactConstraints: {
        enabled: boolean;
    };
    constraints: {
        enabled: boolean;
        drawLimits: boolean;
        size: number;
    };
    broadphaseDbvt: {
        enabled: boolean;
        showLeafNodes: boolean;
        showNonLeafNodes: boolean;
    };
    triangleMeshBvh: {
        enabled: boolean;
        showLeafNodes: boolean;
        showNonLeafNodes: boolean;
    };
};

type BodiesState = {
    batchedMesh: THREE.BatchedMesh;
    velocitiesBatchedMesh: THREE.BatchedMesh;
    material: THREE.Material;
    velocitiesMaterial: THREE.Material;
    maxVertexCount: number;
    maxIndexCount: number;
    currentVertexCount: number;
    currentIndexCount: number;
    geometryCache: GeometryCache;
    velocityCylinderGeometryId: number | null;
    velocityConeGeometryId: number | null;
    instances: Map<number, BodyInstance>;
    instanceColors: Map<number, THREE.Color>;
    edgeWireframes: Map<number, EdgeWireframeInstance>;
    edgeWireframeContainer: THREE.Object3D;
    previousWireframe: boolean;
    linearVelocityArrows: Map<number, { shaftId: number; tipId: number }>;
    angularVelocityArrows: Map<number, { shaftId: number; tipId: number }>;
};

type ContactsState = {
    container: THREE.Object3D;
    spheresMesh: THREE.InstancedMesh;
    cylindersMesh: THREE.InstancedMesh;
    conesMesh: THREE.InstancedMesh;
    spheresMaxCount: number;
    cylindersMaxCount: number;
    conesMaxCount: number;
};

type ContactConstraintsState = {
    container: THREE.Object3D;
    spheresMesh: THREE.InstancedMesh;
    cylindersMesh: THREE.InstancedMesh;
    conesMesh: THREE.InstancedMesh;
    spheresMaxCount: number;
    cylindersMaxCount: number;
    conesMaxCount: number;
};

type ConstraintsState = {
    lineSegments: THREE.LineSegments;
};

type BroadphaseState = {
    dbvtLines: THREE.LineSegments;
};

type TriangleMeshBvhState = {
    lines: THREE.LineSegments;
    cache: Map<number, THREE.LineSegments>;
    container: THREE.Object3D;
    previousShowLeafNodes: boolean;
    previousShowNonLeafNodes: boolean;
};

export type State = {
    object3d: THREE.Object3D;
    options: DebugRendererOptions;
    nextTriangleMeshId: number;
    bodies: BodiesState;
    contacts: ContactsState;
    contactConstraints: ContactConstraintsState;
    constraints: ConstraintsState;
    broadphase: BroadphaseState;
    triangleMeshBvh: TriangleMeshBvhState;
};

/** create default debug renderer options */
export function createDefaultOptions(): DebugRendererOptions {
    return {
        bodies: {
            enabled: true,
            color: BodyColorMode.INSTANCE,
            wireframe: false,
            showLinearVelocity: false,
            showAngularVelocity: false,
        },
        contacts: {
            enabled: false,
        },
        contactConstraints: {
            enabled: false,
        },
        constraints: {
            enabled: false,
            drawLimits: false,
            size: 0.5,
        },
        broadphaseDbvt: {
            enabled: false,
            showLeafNodes: true,
            showNonLeafNodes: true,
        },
        triangleMeshBvh: {
            enabled: false,
            showLeafNodes: true,
            showNonLeafNodes: true,
        },
    };
}

/**
 * initialize the debug renderer
 * @param options - optional debug renderer options. if not provided, default options will be used.
 */
export function init(options?: DebugRendererOptions): State {
    const maxInstances = 10_000;
    const maxVertexCount = 100_000;
    const maxIndexCount = 100_000;

    const bodiesMaterial = new THREE.MeshPhongMaterial();
    const bodiesBatchedMesh = new THREE.BatchedMesh(maxInstances, maxVertexCount, maxIndexCount, bodiesMaterial);

    const velocitiesMaterial = new THREE.MeshBasicMaterial();
    velocitiesMaterial.depthTest = false;
    const velocitiesBatchedMesh = new THREE.BatchedMesh(maxInstances, maxVertexCount, maxIndexCount, velocitiesMaterial);
    velocitiesBatchedMesh.frustumCulled = false;
    velocitiesBatchedMesh.renderOrder = 1000; // render velocities on top of everything

    // shared geometries for contact visualization
    const sphereGeometry = createUnitSphereGeometry();
    const cylinderGeometry = createUnitCylinderGeometry();
    const coneGeometry = createUnitConeGeometry();

    // contacts visualization (simple contact points + normals)
    const contactsSpheresMaxCount = 500;
    const contactsCylindersMaxCount = 500;
    const contactsConesMaxCount = 500;

    const contactsMaterial = new THREE.MeshPhongMaterial();
    contactsMaterial.depthTest = false;

    const contactsSpheresMesh = new THREE.InstancedMesh(sphereGeometry, contactsMaterial, contactsSpheresMaxCount);
    contactsSpheresMesh.frustumCulled = false;
    contactsSpheresMesh.renderOrder = 1;
    contactsSpheresMesh.count = 0;

    const contactsCylindersMesh = new THREE.InstancedMesh(cylinderGeometry, contactsMaterial, contactsCylindersMaxCount);
    contactsCylindersMesh.frustumCulled = false;
    contactsCylindersMesh.renderOrder = 1;
    contactsCylindersMesh.count = 0;

    const contactsConesMesh = new THREE.InstancedMesh(coneGeometry, contactsMaterial, contactsConesMaxCount);
    contactsConesMesh.frustumCulled = false;
    contactsConesMesh.renderOrder = 1;
    contactsConesMesh.count = 0;

    const contactsContainer = new THREE.Object3D();
    contactsContainer.add(contactsSpheresMesh);
    contactsContainer.add(contactsCylindersMesh);
    contactsContainer.add(contactsConesMesh);

    // contact constraints visualization (detailed manifold view)
    const constraintContactsSpheresMaxCount = 1000;
    const constraintContactsCylindersMaxCount = 1000;
    const constraintContactsConesMaxCount = 500;

    const constraintContactsMaterial = new THREE.MeshPhongMaterial();
    constraintContactsMaterial.depthTest = false;

    const constraintContactsSpheresMesh = new THREE.InstancedMesh(
        sphereGeometry,
        constraintContactsMaterial,
        constraintContactsSpheresMaxCount,
    );
    constraintContactsSpheresMesh.frustumCulled = false;
    constraintContactsSpheresMesh.renderOrder = 1;
    constraintContactsSpheresMesh.count = 0;

    const constraintContactsCylindersMesh = new THREE.InstancedMesh(
        cylinderGeometry,
        constraintContactsMaterial,
        constraintContactsCylindersMaxCount,
    );
    constraintContactsCylindersMesh.frustumCulled = false;
    constraintContactsCylindersMesh.renderOrder = 1;
    constraintContactsCylindersMesh.count = 0;

    const constraintContactsConesMesh = new THREE.InstancedMesh(
        coneGeometry,
        constraintContactsMaterial,
        constraintContactsConesMaxCount,
    );
    constraintContactsConesMesh.frustumCulled = false;
    constraintContactsConesMesh.renderOrder = 1;
    constraintContactsConesMesh.count = 0;

    const constraintContactsContainer = new THREE.Object3D();
    constraintContactsContainer.add(constraintContactsSpheresMesh);
    constraintContactsContainer.add(constraintContactsCylindersMesh);
    constraintContactsContainer.add(constraintContactsConesMesh);

    // create broadphase dbvt visualization
    const broadphaseDbvtGeometry = new THREE.BufferGeometry();
    const broadphaseDbvtMaterial = new THREE.LineBasicMaterial({ vertexColors: true });
    const broadphaseDbvtLines = new THREE.LineSegments(broadphaseDbvtGeometry, broadphaseDbvtMaterial);
    broadphaseDbvtLines.frustumCulled = false;

    // create triangle mesh bvh visualization container
    const triangleMeshBvhContainer = new THREE.Object3D();
    const triangleMeshBvhLines = new THREE.LineSegments(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ vertexColors: true }),
    );
    triangleMeshBvhLines.frustumCulled = false;

    // create constraints visualization
    const constraintsGeometry = new THREE.BufferGeometry();
    const constraintsMaterial = new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, depthWrite: false });
    const constraintsLineSegments = new THREE.LineSegments(constraintsGeometry, constraintsMaterial);
    constraintsLineSegments.frustumCulled = false;
    constraintsLineSegments.renderOrder = 999; // render on top of everything

    // create container for edge wireframe line segments
    const edgeWireframeContainer = new THREE.Object3D();

    const object3d = new THREE.Object3D();
    object3d.add(bodiesBatchedMesh);
    object3d.add(contactsContainer);
    object3d.add(constraintContactsContainer);
    object3d.add(velocitiesBatchedMesh);
    object3d.add(broadphaseDbvtLines);
    object3d.add(constraintsLineSegments);
    object3d.add(edgeWireframeContainer);
    object3d.add(triangleMeshBvhContainer);

    const resolvedOptions = options ?? createDefaultOptions();

    return {
        object3d,
        options: resolvedOptions,
        nextTriangleMeshId: 0,
        bodies: {
            batchedMesh: bodiesBatchedMesh,
            velocitiesBatchedMesh,
            material: bodiesMaterial,
            velocitiesMaterial,
            maxVertexCount,
            maxIndexCount,
            currentVertexCount: 0,
            currentIndexCount: 0,
            geometryCache: new Map(),
            velocityCylinderGeometryId: null,
            velocityConeGeometryId: null,
            instances: new Map(),
            instanceColors: new Map(),
            edgeWireframes: new Map(),
            edgeWireframeContainer,
            previousWireframe: false,
            linearVelocityArrows: new Map(),
            angularVelocityArrows: new Map(),
        },
        contacts: {
            container: contactsContainer,
            spheresMesh: contactsSpheresMesh,
            cylindersMesh: contactsCylindersMesh,
            conesMesh: contactsConesMesh,
            spheresMaxCount: contactsSpheresMaxCount,
            cylindersMaxCount: contactsCylindersMaxCount,
            conesMaxCount: contactsConesMaxCount,
        },
        contactConstraints: {
            container: constraintContactsContainer,
            spheresMesh: constraintContactsSpheresMesh,
            cylindersMesh: constraintContactsCylindersMesh,
            conesMesh: constraintContactsConesMesh,
            spheresMaxCount: constraintContactsSpheresMaxCount,
            cylindersMaxCount: constraintContactsCylindersMaxCount,
            conesMaxCount: constraintContactsConesMaxCount,
        },
        constraints: {
            lineSegments: constraintsLineSegments,
        },
        broadphase: {
            dbvtLines: broadphaseDbvtLines,
        },
        triangleMeshBvh: {
            lines: triangleMeshBvhLines,
            cache: new Map(),
            container: triangleMeshBvhContainer,
            previousShowLeafNodes: true,
            previousShowNonLeafNodes: true,
        },
    };
}

function ensureBatchedMeshCapacity(mesh: THREE.BatchedMesh, requiredInstances: number): void {
    const currentMax = mesh.maxInstanceCount;

    if (mesh.instanceCount + requiredInstances <= currentMax) {
        return;
    }

    mesh.optimize();

    if (mesh.instanceCount + requiredInstances <= currentMax) {
        return;
    }

    const newMaxInstances = Math.max(currentMax * 2, mesh.instanceCount + requiredInstances);
    mesh.setInstanceCount(newMaxInstances);
}

function ensureBatchedMeshGeometryCapacity(mesh: THREE.BatchedMesh, state: State, geometry: THREE.BufferGeometry): void {
    const positionAttr = geometry.attributes.position;
    const indexAttr = geometry.index;

    const requiredVertices = positionAttr ? positionAttr.count : 0;
    const requiredIndices = indexAttr ? indexAttr.count : 0;

    // check if we need to expand geometry buffers
    const needsExpansion =
        state.bodies.currentVertexCount + requiredVertices > state.bodies.maxVertexCount ||
        state.bodies.currentIndexCount + requiredIndices > state.bodies.maxIndexCount;

    if (needsExpansion) {
        // grow conservatively
        const newMaxVertexCount = Math.max(
            state.bodies.maxVertexCount,
            state.bodies.currentVertexCount + requiredVertices + 5000,
        );
        const newMaxIndexCount = Math.max(state.bodies.maxIndexCount, state.bodies.currentIndexCount + requiredIndices + 5000);

        state.bodies.maxVertexCount = newMaxVertexCount;
        state.bodies.maxIndexCount = newMaxIndexCount;
        mesh.setGeometrySize(newMaxVertexCount, newMaxIndexCount);
    }

    // track the added geometry
    state.bodies.currentVertexCount += requiredVertices;
    state.bodies.currentIndexCount += requiredIndices;
}

function addInstanceToBatchedMesh(
    mesh: THREE.BatchedMesh,
    geometryId: number,
    matrix: THREE.Matrix4,
    color: THREE.Color,
): number {
    ensureBatchedMeshCapacity(mesh, 1);

    const instanceId = mesh.addInstance(geometryId);
    mesh.setMatrixAt(instanceId, matrix);
    mesh.setColorAt(instanceId, color);

    return instanceId;
}

function getOrCreateGeometry(mesh: THREE.BatchedMesh, cache: GeometryCache, key: ShapeKey, state: State): number {
    const cached = cache.get(key);
    if (cached) {
        cached.refCount++;
        return cached.geometryId;
    }

    let geometry: THREE.BufferGeometry;

    if (key === 'unit-sphere') {
        geometry = createUnitSphereGeometry();
    } else if (key === 'unit-box') {
        geometry = createUnitBoxGeometry();
    } else if (key === 'unit-capsule') {
        geometry = createUnitCapsuleGeometry();
    } else if (key === 'unit-cone') {
        geometry = createUnitConeGeometry();
    } else if (key === 'unit-cylinder') {
        geometry = createUnitCylinderGeometry();
    } else if (key === 'unit-plane') {
        geometry = createUnitPlaneGeometry();
    } else {
        // should not reach here - triangle meshes handled separately
        throw new Error(`Unexpected shape key: ${key}`);
    }

    // ensure we have enough geometry buffer space before adding
    ensureBatchedMeshGeometryCapacity(mesh, state, geometry);

    const geometryId = mesh.addGeometry(geometry);
    cache.set(key, {
        geometry,
        geometryId,
        refCount: 1,
    });

    return geometryId;
}

const _matrix = new THREE.Matrix4();
const _position = new THREE.Vector3();
const _quaternion = new THREE.Quaternion();
const _scale = new THREE.Vector3();

const _shapeLocalMatrix = new THREE.Matrix4(); // leaf shape's own local transform (e.g., sphere scale)
const _childLocalMatrix = new THREE.Matrix4(); // accumulated local transform for recursion
const _combinedLocalMatrix = new THREE.Matrix4(); // combined parent local + shape local
const _worldShapeMatrix = new THREE.Matrix4();
const _instanceColor = new THREE.Color();
const _planeNormalDefault = new THREE.Vector3();
const _planeNormalTarget = new THREE.Vector3();
const _identityMatrix = new THREE.Matrix4();

// mutable index for tracking position during shape traversal
type MutableIndex = { value: number };

const _up = new THREE.Vector3();
const _shaftMidpoint = new THREE.Vector3();
const _tipPosition = new THREE.Vector3();
const _edgeDir = new THREE.Vector3();
const _edgeMidpoint = new THREE.Vector3();
const _currentPointA = new THREE.Vector3();
const _prevPointA = new THREE.Vector3();
const _firstPoint = new THREE.Vector3();
const _normalStart = new THREE.Vector3();
const _normalVec = new THREE.Vector3();
const _normalShaftMidpoint = new THREE.Vector3();
const _normalTipPosition = new THREE.Vector3();
const _tangent1Vec = new THREE.Vector3();
const _tangent1Midpoint = new THREE.Vector3();
const _tangent2Vec = new THREE.Vector3();
const _tangent2Midpoint = new THREE.Vector3();

// adds shape instances to batched mesh during traversal, returns instance ids
function addShapeInstances(
    state: State,
    shape: Shape,
    bodyMatrix: THREE.Matrix4,
    localMatrix: THREE.Matrix4,
    color: THREE.Color,
    instanceIds: number[],
): void {
    switch (shape.type) {
        case ShapeType.SPHERE: {
            const key: ShapeKey = 'unit-sphere';
            const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, key, state);

            // compute local transform for this shape
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(shape.radius, shape.radius, shape.radius);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
            instanceIds.push(instanceId);
            break;
        }

        case ShapeType.BOX: {
            const key: ShapeKey = 'unit-box';
            const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, key, state);

            const [hx, hy, hz] = shape.halfExtents;
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(hx * 2, hy * 2, hz * 2);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
            instanceIds.push(instanceId);
            break;
        }

        case ShapeType.CAPSULE: {
            const key: ShapeKey = 'unit-capsule';
            const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, key, state);

            const r = shape.radius;
            const h = shape.halfHeightOfCylinder;
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(r, h, r);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
            instanceIds.push(instanceId);
            break;
        }

        case ShapeType.CYLINDER: {
            const key: ShapeKey = 'unit-cylinder';
            const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, key, state);

            const r = shape.radius;
            const h = shape.halfHeight;
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(r * 2, h * 2, r * 2);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
            instanceIds.push(instanceId);
            break;
        }

        case ShapeType.PLANE: {
            const key: ShapeKey = 'unit-plane';
            const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, key, state);

            const [nx, ny, nz] = shape.plane.normal;
            const distance = -shape.plane.constant;
            const size = shape.halfExtent * 2;

            _planeNormalDefault.set(0, 0, 1);
            _planeNormalTarget.set(nx, ny, nz);
            _quaternion.setFromUnitVectors(_planeNormalDefault, _planeNormalTarget);
            _position.set(nx * distance, ny * distance, nz * distance);
            _scale.set(size, size, 1);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
            instanceIds.push(instanceId);
            break;
        }

        case ShapeType.TRIANGLE_MESH: {
            const triangleMeshId = state.nextTriangleMeshId++;
            const key: ShapeKey = `triangle-mesh:${triangleMeshId}`;

            const geometry = createTriangleMeshGeometry(shape);
            ensureBatchedMeshGeometryCapacity(state.bodies.batchedMesh, state, geometry);
            const geometryId = state.bodies.batchedMesh.addGeometry(geometry);

            state.bodies.geometryCache.set(key, {
                geometry,
                geometryId,
                refCount: 1,
            });

            // triangle mesh uses identity local transform (geometry is already in local space)
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
            const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
            instanceIds.push(instanceId);
            break;
        }

        case ShapeType.CONVEX_HULL: {
            const convexHullId = state.nextTriangleMeshId++;
            const key: ShapeKey = `convex-hull:${convexHullId}`;

            const geometry = createConvexHullGeometry(shape);
            ensureBatchedMeshGeometryCapacity(state.bodies.batchedMesh, state, geometry);
            const geometryId = state.bodies.batchedMesh.addGeometry(geometry);

            state.bodies.geometryCache.set(key, {
                geometry,
                geometryId,
                refCount: 1,
            });

            // convex hull uses identity local transform (geometry is already in local space)
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
            const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
            instanceIds.push(instanceId);
            break;
        }

        case ShapeType.COMPOUND: {
            for (const child of shape.children) {
                const [tx, ty, tz] = child.position;
                const [qx, qy, qz, qw] = child.quaternion;

                _position.set(tx, ty, tz);
                _quaternion.set(qx, qy, qz, qw);
                _scale.set(1, 1, 1);
                _matrix.compose(_position, _quaternion, _scale);
                _childLocalMatrix.multiplyMatrices(localMatrix, _matrix);

                addShapeInstances(state, child.shape, bodyMatrix, _childLocalMatrix, color, instanceIds);
            }
            break;
        }

        case ShapeType.TRANSFORMED: {
            const [tx, ty, tz] = shape.position;
            const [qx, qy, qz, qw] = shape.quaternion;

            _position.set(tx, ty, tz);
            _quaternion.set(qx, qy, qz, qw);
            _scale.set(1, 1, 1);
            _matrix.compose(_position, _quaternion, _scale);
            _childLocalMatrix.multiplyMatrices(localMatrix, _matrix);

            addShapeInstances(state, shape.shape, bodyMatrix, _childLocalMatrix, color, instanceIds);
            break;
        }

        case ShapeType.SCALED: {
            const [sx, sy, sz] = shape.scale;

            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(sx, sy, sz);
            _matrix.compose(_position, _quaternion, _scale);
            _childLocalMatrix.multiplyMatrices(localMatrix, _matrix);

            addShapeInstances(state, shape.shape, bodyMatrix, _childLocalMatrix, color, instanceIds);
            break;
        }

        case ShapeType.OFFSET_CENTER_OF_MASS: {
            // geometry is at local origin - offset only affects COM, not geometry position
            addShapeInstances(state, shape.shape, bodyMatrix, localMatrix, color, instanceIds);
            break;
        }
    }
}

// updates shape instance matrices by recomputing local transforms on-demand
function updateShapeInstances(
    mesh: THREE.BatchedMesh,
    shape: Shape,
    bodyMatrix: THREE.Matrix4,
    localMatrix: THREE.Matrix4,
    instanceIds: number[],
    index: MutableIndex,
    color: THREE.Color,
): void {
    switch (shape.type) {
        case ShapeType.SPHERE: {
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(shape.radius, shape.radius, shape.radius);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
            mesh.setColorAt(instanceIds[index.value], color);
            index.value++;
            break;
        }

        case ShapeType.BOX: {
            const [hx, hy, hz] = shape.halfExtents;
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(hx * 2, hy * 2, hz * 2);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
            mesh.setColorAt(instanceIds[index.value], color);
            index.value++;
            break;
        }

        case ShapeType.CAPSULE: {
            const r = shape.radius;
            const h = shape.halfHeightOfCylinder;
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(r, h, r);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
            mesh.setColorAt(instanceIds[index.value], color);
            index.value++;
            break;
        }

        case ShapeType.CYLINDER: {
            const r = shape.radius;
            const h = shape.halfHeight;
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(r * 2, h * 2, r * 2);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
            mesh.setColorAt(instanceIds[index.value], color);
            index.value++;
            break;
        }

        case ShapeType.PLANE: {
            const [nx, ny, nz] = shape.plane.normal;
            const distance = -shape.plane.constant;
            const size = shape.halfExtent * 2;

            _planeNormalDefault.set(0, 0, 1);
            _planeNormalTarget.set(nx, ny, nz);
            _quaternion.setFromUnitVectors(_planeNormalDefault, _planeNormalTarget);
            _position.set(nx * distance, ny * distance, nz * distance);
            _scale.set(size, size, 1);
            _shapeLocalMatrix.compose(_position, _quaternion, _scale);
            _combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);

            mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
            mesh.setColorAt(instanceIds[index.value], color);
            index.value++;
            break;
        }

        case ShapeType.TRIANGLE_MESH: {
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
            mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
            mesh.setColorAt(instanceIds[index.value], color);
            index.value++;
            break;
        }

        case ShapeType.CONVEX_HULL: {
            _worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
            mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
            mesh.setColorAt(instanceIds[index.value], color);
            index.value++;
            break;
        }

        case ShapeType.COMPOUND: {
            for (const child of shape.children) {
                const [tx, ty, tz] = child.position;
                const [qx, qy, qz, qw] = child.quaternion;

                _position.set(tx, ty, tz);
                _quaternion.set(qx, qy, qz, qw);
                _scale.set(1, 1, 1);
                _matrix.compose(_position, _quaternion, _scale);
                _childLocalMatrix.multiplyMatrices(localMatrix, _matrix);

                updateShapeInstances(mesh, child.shape, bodyMatrix, _childLocalMatrix, instanceIds, index, color);
            }
            break;
        }

        case ShapeType.TRANSFORMED: {
            const [tx, ty, tz] = shape.position;
            const [qx, qy, qz, qw] = shape.quaternion;

            _position.set(tx, ty, tz);
            _quaternion.set(qx, qy, qz, qw);
            _scale.set(1, 1, 1);
            _matrix.compose(_position, _quaternion, _scale);
            _childLocalMatrix.multiplyMatrices(localMatrix, _matrix);

            updateShapeInstances(mesh, shape.shape, bodyMatrix, _childLocalMatrix, instanceIds, index, color);
            break;
        }

        case ShapeType.SCALED: {
            const [sx, sy, sz] = shape.scale;

            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(sx, sy, sz);
            _matrix.compose(_position, _quaternion, _scale);
            _childLocalMatrix.multiplyMatrices(localMatrix, _matrix);

            updateShapeInstances(mesh, shape.shape, bodyMatrix, _childLocalMatrix, instanceIds, index, color);
            break;
        }

        case ShapeType.OFFSET_CENTER_OF_MASS: {
            updateShapeInstances(mesh, shape.shape, bodyMatrix, localMatrix, instanceIds, index, color);
            break;
        }
    }
}

const _colorWhite = new THREE.Color(0xffffff);
const _colorGreen = new THREE.Color(0x00ff00);
const _colorYellow = new THREE.Color(0xffff00);
const _colorBlue = new THREE.Color(0x0000ff);
const _colorRed = new THREE.Color(0xff0000);

function getBodyColor(state: State, body: RigidBody): THREE.Color {
    switch (state.options.bodies.color) {
        case BodyColorMode.INSTANCE: {
            let color = state.bodies.instanceColors.get(body.id);
            if (!color) {
                color = new THREE.Color();
                const hash = body.id * 137.5;

                if (body.motionType === MotionType.STATIC) {
                    // static bodies get slight variance in dark grays
                    const lightness = 0.22 + ((hash % 100) / 100) * 0.2;
                    color.setHSL(0, 0, lightness);
                } else {
                    // dynamic/kinematic bodies get bright colors
                    const hue = (hash % 360) / 360;
                    color.setHSL(hue, 0.7, 0.6);
                }
                state.bodies.instanceColors.set(body.id, color);
            }
            return color;
        }

        case BodyColorMode.MOTION_TYPE: {
            switch (body.motionType) {
                case MotionType.DYNAMIC:
                    return _colorGreen;
                case MotionType.KINEMATIC:
                    return _colorYellow;
                case MotionType.STATIC:
                    return _colorBlue;
                default:
                    return _colorWhite;
            }
        }

        case BodyColorMode.SLEEPING: {
            return body.sleeping ? _colorBlue : _colorRed;
        }

        case BodyColorMode.ISLAND: {
            // color by island index using HSL
            const islandIndex = body.islandIndex;
            if (islandIndex === -1) {
                return _colorWhite; // no island
            }

            _instanceColor.setHSL(((islandIndex * 137.5) % 360) / 360, 0.8, 0.6);
            return _instanceColor;
        }

        default:
            return _colorWhite;
    }
}
/**
 * Invalidate a body's shape to force recreation of its visual instances.
 * Call this when you mutate a body's shape properties in-place.
 * @param state Debug renderer state
 * @param bodyId The ID of the body whose shape was mutated
 */
export function invalidateBodyShape(state: State, bodyId: number): void {
    // Invalidate batched mesh instance
    const instance = state.bodies.instances.get(bodyId);
    if (instance) {
        for (const instanceId of instance.instanceIds) {
            state.bodies.batchedMesh.deleteInstance(instanceId);
        }
        state.bodies.instances.delete(bodyId);
    }

    // Invalidate edge wireframe instance
    const wireframe = state.bodies.edgeWireframes.get(bodyId);
    if (wireframe) {
        state.bodies.edgeWireframeContainer.remove(wireframe.lineSegments);
        wireframe.lineSegments.geometry.dispose();
        for (const geometry of wireframe.geometries) {
            geometry.dispose();
        }
        state.bodies.edgeWireframes.delete(bodyId);
    }
}

const _updateMatrix = new THREE.Matrix4();
const _updatePosition = new THREE.Vector3();
const _updateQuaternion = new THREE.Quaternion();
const _updateScale = new THREE.Vector3();

const _contactPointColor = new THREE.Color(0xff0000);
const _contactNormalColor = new THREE.Color(0x00ff00);
const CONTACT_POINT_RADIUS = 0.05;
const CONTACT_NORMAL_LENGTH = 0.5;
const ARROW_SHAFT_RADIUS = 0.01;
const ARROW_TIP_HEIGHT = 0.1;

const linearVelocityColor = new THREE.Color(0x00ff00); // Green
const angularVelocityColor = new THREE.Color(0xff0000); // Red

const constraintManifoldPointColorA = new THREE.Color(0x00ffff);
const constraintManifoldPointColorB = new THREE.Color(0xff00ff);
const constraintManifoldPointColorAActive = new THREE.Color(0x00ffff);
const constraintManifoldPointColorBActive = new THREE.Color(0xff00ff);
const constraintManifoldEdgeColor = new THREE.Color(0x00ffff);
const constraintNormalColor = new THREE.Color(0xff0000);
const constraintTangent1Color = new THREE.Color(0x00ff00);
const constraintTangent2Color = new THREE.Color(0x0000ff);

const CONSTRAINT_POINT_RADIUS_INACTIVE = 0.1;
const CONSTRAINT_POINT_RADIUS_ACTIVE = 0.2;
const CONSTRAINT_EDGE_RADIUS = 0.01;
const CONSTRAINT_TANGENT_LENGTH = 0.5;

const _edgeWireframeMaterial = new THREE.LineBasicMaterial({ vertexColors: true });

function clearBatchedMeshBodies(state: State): void {
    for (const instance of state.bodies.instances.values()) {
        for (const instanceId of instance.instanceIds) {
            state.bodies.batchedMesh.deleteInstance(instanceId);
        }
    }
    state.bodies.instances.clear();
}

function clearEdgeWireframes(state: State): void {
    for (const instance of state.bodies.edgeWireframes.values()) {
        state.bodies.edgeWireframeContainer.remove(instance.lineSegments);
        instance.lineSegments.geometry.dispose();
        for (const geometry of instance.geometries) {
            geometry.dispose();
        }
    }
    state.bodies.edgeWireframes.clear();
}

function createEdgeWireframeForShape(
    state: State,
    shape: Shape,
    parentMatrix: THREE.Matrix4,
    linePositions: number[],
    lineColors: number[],
    geometries: THREE.EdgesGeometry[],
    localTransforms: THREE.Matrix4[],
    color: THREE.Color,
): void {
    switch (shape.type) {
        case ShapeType.SPHERE: {
            const geometry = new THREE.SphereGeometry(shape.radius, 16, 12);
            const edgesGeometry = new THREE.EdgesGeometry(geometry, 15);
            geometry.dispose();

            const positions = edgesGeometry.attributes.position.array as Float32Array;
            for (let i = 0; i < positions.length; i += 3) {
                _position.set(positions[i], positions[i + 1], positions[i + 2]);
                _position.applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);
            }
            geometries.push(edgesGeometry);
            localTransforms.push(parentMatrix.clone());
            break;
        }

        case ShapeType.BOX: {
            const [hx, hy, hz] = shape.halfExtents;
            const geometry = new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2);
            const edgesGeometry = new THREE.EdgesGeometry(geometry);
            geometry.dispose();

            const positions = edgesGeometry.attributes.position.array as Float32Array;
            for (let i = 0; i < positions.length; i += 3) {
                _position.set(positions[i], positions[i + 1], positions[i + 2]);
                _position.applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);
            }
            geometries.push(edgesGeometry);
            localTransforms.push(parentMatrix.clone());
            break;
        }

        case ShapeType.CAPSULE: {
            const r = shape.radius;
            const h = shape.halfHeightOfCylinder;
            const geometry = new THREE.CapsuleGeometry(r, h * 2, 8, 16);
            const edgesGeometry = new THREE.EdgesGeometry(geometry, 15);
            geometry.dispose();

            const positions = edgesGeometry.attributes.position.array as Float32Array;
            for (let i = 0; i < positions.length; i += 3) {
                _position.set(positions[i], positions[i + 1], positions[i + 2]);
                _position.applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);
            }
            geometries.push(edgesGeometry);
            localTransforms.push(parentMatrix.clone());
            break;
        }

        case ShapeType.CYLINDER: {
            const r = shape.radius;
            const h = shape.halfHeight;
            const geometry = new THREE.CylinderGeometry(r, r, h * 2, 32);
            const edgesGeometry = new THREE.EdgesGeometry(geometry);
            geometry.dispose();

            const positions = edgesGeometry.attributes.position.array as Float32Array;
            for (let i = 0; i < positions.length; i += 3) {
                _position.set(positions[i], positions[i + 1], positions[i + 2]);
                _position.applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);
            }
            geometries.push(edgesGeometry);
            localTransforms.push(parentMatrix.clone());
            break;
        }

        case ShapeType.PLANE: {
            const [nx, ny, nz] = shape.plane.normal;
            const distance = -shape.plane.constant;
            const size = shape.halfExtent * 2;

            const geometry = new THREE.PlaneGeometry(size, size, 1, 1);
            const edgesGeometry = new THREE.EdgesGeometry(geometry);
            geometry.dispose();

            // compute rotation and position same as in collectShapeInstances
            // THREE.PlaneGeometry is in XY plane with normal [0,0,1]
            const defaultNormal = new THREE.Vector3(0, 0, 1);
            const planeNormal = new THREE.Vector3(nx, ny, nz);
            _quaternion.setFromUnitVectors(defaultNormal, planeNormal);
            _position.set(nx * distance, ny * distance, nz * distance);
            _scale.set(1, 1, 1);
            _matrix.compose(_position, _quaternion, _scale);
            const matrix = parentMatrix.clone().multiply(_matrix);

            const positions = edgesGeometry.attributes.position.array as Float32Array;
            for (let i = 0; i < positions.length; i += 3) {
                _position.set(positions[i], positions[i + 1], positions[i + 2]);
                _position.applyMatrix4(matrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);
            }
            geometries.push(edgesGeometry);
            localTransforms.push(matrix.clone());
            break;
        }

        case ShapeType.TRIANGLE_MESH: {
            // For triangle meshes, render all triangle edges directly instead of using EdgesGeometry
            // EdgesGeometry with angle threshold often produces poor results for complex meshes
            const positions = shape.data.positions;
            const triangleBuffer = shape.data.triangleBuffer;
            const triangleCount = shape.data.triangleCount;

            for (let i = 0; i < triangleCount; i++) {
                const offset = i * 8; // TRIANGLE_STRIDE = 8
                const idxA = triangleBuffer[offset + 0]; // OFFSET_INDEX_A
                const idxB = triangleBuffer[offset + 1]; // OFFSET_INDEX_B
                const idxC = triangleBuffer[offset + 2]; // OFFSET_INDEX_C

                // Get the three vertices
                const ax = positions[idxA * 3 + 0];
                const ay = positions[idxA * 3 + 1];
                const az = positions[idxA * 3 + 2];

                const bx = positions[idxB * 3 + 0];
                const by = positions[idxB * 3 + 1];
                const bz = positions[idxB * 3 + 2];

                const cx = positions[idxC * 3 + 0];
                const cy = positions[idxC * 3 + 1];
                const cz = positions[idxC * 3 + 2];

                // Add three edges: A->B, B->C, C->A
                _position.set(ax, ay, az).applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);

                _position.set(bx, by, bz).applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);

                _position.set(bx, by, bz).applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);

                _position.set(cx, cy, cz).applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);

                _position.set(cx, cy, cz).applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);

                _position.set(ax, ay, az).applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);
            }

            localTransforms.push(parentMatrix.clone());
            break;
        }

        case ShapeType.CONVEX_HULL: {
            const geometry = createConvexHullGeometry(shape);
            const edgesGeometry = new THREE.EdgesGeometry(geometry, 30);
            geometry.dispose();

            const positions = edgesGeometry.attributes.position.array as Float32Array;
            for (let i = 0; i < positions.length; i += 3) {
                _position.set(positions[i], positions[i + 1], positions[i + 2]);
                _position.applyMatrix4(parentMatrix);
                linePositions.push(_position.x, _position.y, _position.z);
                lineColors.push(color.r, color.g, color.b);
            }
            geometries.push(edgesGeometry);
            localTransforms.push(parentMatrix.clone());
            break;
        }

        case ShapeType.COMPOUND: {
            for (const child of shape.children) {
                const [tx, ty, tz] = child.position;
                const [qx, qy, qz, qw] = child.quaternion;

                _position.set(tx, ty, tz);
                _quaternion.set(qx, qy, qz, qw);
                _scale.set(1, 1, 1);
                _matrix.compose(_position, _quaternion, _scale);
                const childMatrix = parentMatrix.clone().multiply(_matrix);

                createEdgeWireframeForShape(
                    state,
                    child.shape,
                    childMatrix,
                    linePositions,
                    lineColors,
                    geometries,
                    localTransforms,
                    color,
                );
            }
            break;
        }

        case ShapeType.TRANSFORMED: {
            const [tx, ty, tz] = shape.position;
            const [qx, qy, qz, qw] = shape.quaternion;

            _position.set(tx, ty, tz);
            _quaternion.set(qx, qy, qz, qw);
            _scale.set(1, 1, 1);
            _matrix.compose(_position, _quaternion, _scale);
            const transformedMatrix = parentMatrix.clone().multiply(_matrix);

            createEdgeWireframeForShape(
                state,
                shape.shape,
                transformedMatrix,
                linePositions,
                lineColors,
                geometries,
                localTransforms,
                color,
            );
            break;
        }

        case ShapeType.SCALED: {
            const [sx, sy, sz] = shape.scale;

            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(sx, sy, sz);
            _matrix.compose(_position, _quaternion, _scale);
            const scaledMatrix = parentMatrix.clone().multiply(_matrix);

            createEdgeWireframeForShape(
                state,
                shape.shape,
                scaledMatrix,
                linePositions,
                lineColors,
                geometries,
                localTransforms,
                color,
            );
            break;
        }

        case ShapeType.OFFSET_CENTER_OF_MASS: {
            createEdgeWireframeForShape(
                state,
                shape.shape,
                parentMatrix,
                linePositions,
                lineColors,
                geometries,
                localTransforms,
                color,
            );
            break;
        }
    }
}

function updateBodiesBatchedMesh(state: State, world: World): void {
    state.bodies.batchedMesh.visible = true;
    state.bodies.edgeWireframeContainer.visible = false;

    // Remove bodies that no longer exist
    const activeBodyIds = new Set<number>();
    for (const body of rigidBody.iterate(world)) {
        activeBodyIds.add(body.id);
    }

    for (const [bodyId, instance] of state.bodies.instances) {
        if (!activeBodyIds.has(bodyId)) {
            for (const instanceId of instance.instanceIds) {
                state.bodies.batchedMesh.deleteInstance(instanceId);
            }
            state.bodies.instances.delete(bodyId);
        }
    }

    // Add or update bodies
    for (const body of rigidBody.iterate(world)) {
        let instance = state.bodies.instances.get(body.id);

        // Check if shape has changed and needs recreation
        if (instance && instance.shape !== body.shape) {
            // Shape changed - remove old instances
            for (const instanceId of instance.instanceIds) {
                state.bodies.batchedMesh.deleteInstance(instanceId);
            }
            instance = undefined;
        }

        // compute body transform
        _updatePosition.set(body.position[0], body.position[1], body.position[2]);
        _updateQuaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
        _updateScale.set(1, 1, 1);
        _updateMatrix.compose(_updatePosition, _updateQuaternion, _updateScale);

        const color = getBodyColor(state, body);

        if (!instance) {
            // create new instances for this body
            const instanceIds: number[] = [];
            _identityMatrix.identity();
            addShapeInstances(state, body.shape, _updateMatrix, _identityMatrix, color, instanceIds);

            const newInstance: BodyInstance = {
                bodyId: body.id,
                shape: body.shape,
                instanceIds,
            };
            state.bodies.instances.set(body.id, newInstance);
            continue;
        }

        // update existing instances by recomputing local transforms on-demand
        _identityMatrix.identity();
        const index: MutableIndex = { value: 0 };
        updateShapeInstances(
            state.bodies.batchedMesh,
            body.shape,
            _updateMatrix,
            _identityMatrix,
            instance.instanceIds,
            index,
            color,
        );
    }
}

function updateBodiesEdgeWireframe(state: State, world: World): void {
    state.bodies.batchedMesh.visible = false;
    state.bodies.edgeWireframeContainer.visible = true;

    // Remove wireframes for bodies that no longer exist
    const activeBodyIds = new Set<number>();
    for (const body of rigidBody.iterate(world)) {
        activeBodyIds.add(body.id);
    }

    for (const [bodyId, instance] of state.bodies.edgeWireframes) {
        if (!activeBodyIds.has(bodyId)) {
            state.bodies.edgeWireframeContainer.remove(instance.lineSegments);
            instance.lineSegments.geometry.dispose();
            for (const geometry of instance.geometries) {
                geometry.dispose();
            }
            state.bodies.edgeWireframes.delete(bodyId);
        }
    }

    // Add or update bodies
    for (const body of rigidBody.iterate(world)) {
        let instance = state.bodies.edgeWireframes.get(body.id);

        // Check if shape has changed and needs recreation
        if (instance && instance.shape !== body.shape) {
            // Shape changed - remove old wireframe
            state.bodies.edgeWireframeContainer.remove(instance.lineSegments);
            instance.lineSegments.geometry.dispose();
            for (const geometry of instance.geometries) {
                geometry.dispose();
            }
            state.bodies.edgeWireframes.delete(body.id);
            instance = undefined;
        }

        if (!instance) {
            const linePositions: number[] = [];
            const lineColors: number[] = [];
            const geometries: THREE.EdgesGeometry[] = [];
            const localTransforms: THREE.Matrix4[] = [];
            const color = getBodyColor(state, body);

            const identityMatrix = new THREE.Matrix4();
            createEdgeWireframeForShape(
                state,
                body.shape,
                identityMatrix,
                linePositions,
                lineColors,
                geometries,
                localTransforms,
                color,
            );

            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(linePositions), 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(lineColors), 3));

            const lineSegments = new THREE.LineSegments(geometry, _edgeWireframeMaterial);
            lineSegments.frustumCulled = false;

            state.bodies.edgeWireframeContainer.add(lineSegments);

            const newInstance: EdgeWireframeInstance = {
                bodyId: body.id,
                shape: body.shape,
                lineSegments,
                localTransforms,
                geometries,
            };
            state.bodies.edgeWireframes.set(body.id, newInstance);

            // Update transform
            _updatePosition.set(body.position[0], body.position[1], body.position[2]);
            _updateQuaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
            _updateScale.set(1, 1, 1);
            _updateMatrix.compose(_updatePosition, _updateQuaternion, _updateScale);

            newInstance.lineSegments.position.set(_updatePosition.x, _updatePosition.y, _updatePosition.z);
            newInstance.lineSegments.quaternion.set(
                _updateQuaternion.x,
                _updateQuaternion.y,
                _updateQuaternion.z,
                _updateQuaternion.w,
            );
            continue;
        }

        // Update transform
        _updatePosition.set(body.position[0], body.position[1], body.position[2]);
        _updateQuaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
        _updateScale.set(1, 1, 1);
        _updateMatrix.compose(_updatePosition, _updateQuaternion, _updateScale);

        instance.lineSegments.position.set(_updatePosition.x, _updatePosition.y, _updatePosition.z);
        instance.lineSegments.quaternion.set(_updateQuaternion.x, _updateQuaternion.y, _updateQuaternion.z, _updateQuaternion.w);
    }
}

function updateBodies(state: State, world: World): void {
    const wireframeChanged = state.bodies.previousWireframe !== state.options.bodies.wireframe;

    if (!state.options.bodies.enabled) {
        // Clear everything when disabled
        if (state.bodies.instances.size > 0) {
            clearBatchedMeshBodies(state);
        }
        if (state.bodies.edgeWireframes.size > 0) {
            clearEdgeWireframes(state);
        }
        state.bodies.batchedMesh.visible = false;
        state.bodies.edgeWireframeContainer.visible = false;
        state.bodies.previousWireframe = state.options.bodies.wireframe;
        return;
    }

    // Handle mode switching - clear the old mode's data
    if (wireframeChanged) {
        if (state.bodies.previousWireframe) {
            clearEdgeWireframes(state);
        } else {
            clearBatchedMeshBodies(state);
        }
    }

    // Update based on current wireframe setting
    if (state.options.bodies.wireframe) {
        updateBodiesEdgeWireframe(state, world);
    } else {
        updateBodiesBatchedMesh(state, world);
    }

    state.bodies.previousWireframe = state.options.bodies.wireframe;
}

// helpers to ensure InstancedMesh has enough capacity for contacts
function ensureContactsSpheresCapacity(state: State, requiredCount: number): void {
    if (requiredCount <= state.contacts.spheresMaxCount) return;

    const newMaxCount = Math.max(state.contacts.spheresMaxCount * 2, requiredCount);
    const oldMesh = state.contacts.spheresMesh;

    const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
    newMesh.frustumCulled = false;
    newMesh.renderOrder = oldMesh.renderOrder;
    newMesh.count = 0;

    state.contacts.container.remove(oldMesh);
    state.contacts.container.add(newMesh);
    state.contacts.spheresMesh = newMesh;
    state.contacts.spheresMaxCount = newMaxCount;
}

function ensureContactsCylindersCapacity(state: State, requiredCount: number): void {
    if (requiredCount <= state.contacts.cylindersMaxCount) return;

    const newMaxCount = Math.max(state.contacts.cylindersMaxCount * 2, requiredCount);
    const oldMesh = state.contacts.cylindersMesh;

    const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
    newMesh.frustumCulled = false;
    newMesh.renderOrder = oldMesh.renderOrder;
    newMesh.count = 0;

    state.contacts.container.remove(oldMesh);
    state.contacts.container.add(newMesh);
    state.contacts.cylindersMesh = newMesh;
    state.contacts.cylindersMaxCount = newMaxCount;
}

function ensureContactsConesCapacity(state: State, requiredCount: number): void {
    if (requiredCount <= state.contacts.conesMaxCount) return;

    const newMaxCount = Math.max(state.contacts.conesMaxCount * 2, requiredCount);
    const oldMesh = state.contacts.conesMesh;

    const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
    newMesh.frustumCulled = false;
    newMesh.renderOrder = oldMesh.renderOrder;
    newMesh.count = 0;

    state.contacts.container.remove(oldMesh);
    state.contacts.container.add(newMesh);
    state.contacts.conesMesh = newMesh;
    state.contacts.conesMaxCount = newMaxCount;
}

// helpers to ensure InstancedMesh has enough capacity for contact constraints
function ensureConstraintContactsSpheresCapacity(state: State, requiredCount: number): void {
    if (requiredCount <= state.contactConstraints.spheresMaxCount) return;

    const newMaxCount = Math.max(state.contactConstraints.spheresMaxCount * 2, requiredCount);
    const oldMesh = state.contactConstraints.spheresMesh;

    const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
    newMesh.frustumCulled = false;
    newMesh.renderOrder = oldMesh.renderOrder;
    newMesh.count = 0;

    state.contactConstraints.container.remove(oldMesh);
    state.contactConstraints.container.add(newMesh);
    state.contactConstraints.spheresMesh = newMesh;
    state.contactConstraints.spheresMaxCount = newMaxCount;
}

function ensureConstraintContactsCylindersCapacity(state: State, requiredCount: number): void {
    if (requiredCount <= state.contactConstraints.cylindersMaxCount) return;

    const newMaxCount = Math.max(state.contactConstraints.cylindersMaxCount * 2, requiredCount);
    const oldMesh = state.contactConstraints.cylindersMesh;

    const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
    newMesh.frustumCulled = false;
    newMesh.renderOrder = oldMesh.renderOrder;
    newMesh.count = 0;

    state.contactConstraints.container.remove(oldMesh);
    state.contactConstraints.container.add(newMesh);
    state.contactConstraints.cylindersMesh = newMesh;
    state.contactConstraints.cylindersMaxCount = newMaxCount;
}

function ensureConstraintContactsConesCapacity(state: State, requiredCount: number): void {
    if (requiredCount <= state.contactConstraints.conesMaxCount) return;

    const newMaxCount = Math.max(state.contactConstraints.conesMaxCount * 2, requiredCount);
    const oldMesh = state.contactConstraints.conesMesh;

    const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
    newMesh.frustumCulled = false;
    newMesh.renderOrder = oldMesh.renderOrder;
    newMesh.count = 0;

    state.contactConstraints.container.remove(oldMesh);
    state.contactConstraints.container.add(newMesh);
    state.contactConstraints.conesMesh = newMesh;
    state.contactConstraints.conesMaxCount = newMaxCount;
}

function updateContacts(state: State, world: World): void {
    if (!state.options.contacts.enabled) {
        state.contacts.spheresMesh.count = 0;
        state.contacts.cylindersMesh.count = 0;
        state.contacts.conesMesh.count = 0;
        return;
    }

    // count total contact points
    let totalContacts = 0;
    for (let i = 0; i < world.contactConstraints.count; i++) {
        const constraint = world.contactConstraints.pool[i];
        totalContacts += constraint.numContactPoints;
    }

    // each contact: 1 sphere, 1 cylinder (shaft), 1 cone (tip)
    const sphereCount = totalContacts;
    const cylinderCount = totalContacts;
    const coneCount = totalContacts;

    // ensure capacity
    ensureContactsSpheresCapacity(state, sphereCount);
    ensureContactsCylindersCapacity(state, cylinderCount);
    ensureContactsConesCapacity(state, coneCount);

    // set instance counts
    state.contacts.spheresMesh.count = sphereCount;
    state.contacts.cylindersMesh.count = cylinderCount;
    state.contacts.conesMesh.count = coneCount;

    // populate instances
    let sphereIdx = 0;
    let cylinderIdx = 0;
    let coneIdx = 0;

    for (let constraintIdx = 0; constraintIdx < world.contactConstraints.count; constraintIdx++) {
        const constraint = world.contactConstraints.pool[constraintIdx];
        for (let i = 0; i < constraint.numContactPoints; i++) {
            const contactPoint = constraint.contactPoints[i];
            const normal = constraint.normal;

            // contact point sphere at positionA
            _position.set(contactPoint.positionA[0], contactPoint.positionA[1], contactPoint.positionA[2]);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(CONTACT_POINT_RADIUS, CONTACT_POINT_RADIUS, CONTACT_POINT_RADIUS);
            _matrix.compose(_position, _quaternion, _scale);
            state.contacts.spheresMesh.setMatrixAt(sphereIdx, _matrix);
            state.contacts.spheresMesh.setColorAt(sphereIdx, _contactPointColor);
            sphereIdx++;

            // arrow shaft (cylinder along normal)
            const shaftLength = CONTACT_NORMAL_LENGTH - ARROW_TIP_HEIGHT;
            _up.set(0, 1, 0);
            _normalVec.set(normal[0], normal[1], normal[2]);
            _quaternion.setFromUnitVectors(_up, _normalVec);
            _shaftMidpoint.copy(_position).addScaledVector(_normalVec, shaftLength / 2);
            _scale.set(ARROW_SHAFT_RADIUS, shaftLength, ARROW_SHAFT_RADIUS);
            _matrix.compose(_shaftMidpoint, _quaternion, _scale);
            state.contacts.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
            state.contacts.cylindersMesh.setColorAt(cylinderIdx, _contactNormalColor);
            cylinderIdx++;

            // arrow tip (cone at end of shaft)
            _tipPosition.copy(_position).addScaledVector(_normalVec, shaftLength + ARROW_TIP_HEIGHT / 2);
            _scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);
            _matrix.compose(_tipPosition, _quaternion, _scale);
            state.contacts.conesMesh.setMatrixAt(coneIdx, _matrix);
            state.contacts.conesMesh.setColorAt(coneIdx, _contactNormalColor);
            coneIdx++;
        }
    }

    // flag buffers as needing update
    state.contacts.spheresMesh.instanceMatrix.needsUpdate = true;
    state.contacts.cylindersMesh.instanceMatrix.needsUpdate = true;
    state.contacts.conesMesh.instanceMatrix.needsUpdate = true;
    if (state.contacts.spheresMesh.instanceColor) state.contacts.spheresMesh.instanceColor.needsUpdate = true;
    if (state.contacts.cylindersMesh.instanceColor) state.contacts.cylindersMesh.instanceColor.needsUpdate = true;
    if (state.contacts.conesMesh.instanceColor) state.contacts.conesMesh.instanceColor.needsUpdate = true;
}

function updateContactConstraints(state: State, world: World): void {
    if (!state.options.contactConstraints.enabled) {
        state.contactConstraints.spheresMesh.count = 0;
        state.contactConstraints.cylindersMesh.count = 0;
        state.contactConstraints.conesMesh.count = 0;
        return;
    }

    // count instances needed:
    // - 2 spheres per contact point (positionA and positionB)
    // - edges connecting consecutive points on body A (numPoints - 1 + 1 for closing loop if numPoints > 2)
    // - 1 normal arrow per constraint (shaft + tip)
    // - 2 tangent lines per constraint (just shafts, no tips)
    let sphereCount = 0;
    let cylinderCount = 0;
    let coneCount = 0;

    for (let i = 0; i < world.contactConstraints.count; i++) {
        const constraint = world.contactConstraints.pool[i];
        if (constraint.numContactPoints === 0) continue;

        // 2 spheres per contact point
        sphereCount += constraint.numContactPoints * 2;

        // edges between consecutive points (n-1) + closing edge if n > 2
        const edgeCount = constraint.numContactPoints > 2 ? constraint.numContactPoints : constraint.numContactPoints - 1;
        cylinderCount += Math.max(0, edgeCount);

        // 1 normal arrow (shaft cylinder), 2 tangent lines (shafts)
        cylinderCount += 3;

        // 1 normal arrow tip
        coneCount += 1;
    }

    // ensure capacity
    ensureConstraintContactsSpheresCapacity(state, sphereCount);
    ensureConstraintContactsCylindersCapacity(state, cylinderCount);
    ensureConstraintContactsConesCapacity(state, coneCount);

    // set instance counts
    state.contactConstraints.spheresMesh.count = sphereCount;
    state.contactConstraints.cylindersMesh.count = cylinderCount;
    state.contactConstraints.conesMesh.count = coneCount;

    // populate instances
    let sphereIdx = 0;
    let cylinderIdx = 0;
    let coneIdx = 0;

    for (let constraintIdx = 0; constraintIdx < world.contactConstraints.count; constraintIdx++) {
        const constraint = world.contactConstraints.pool[constraintIdx];
        if (constraint.numContactPoints === 0) continue;

        // draw contact points with different sizes based on impulse transfer
        for (let i = 0; i < constraint.numContactPoints; i++) {
            const contactPoint = constraint.contactPoints[i];

            // check if any lambda from previous frame was transferred
            const hasImpulse =
                contactPoint.normalConstraint.totalLambda !== 0 ||
                contactPoint.tangentConstraint1.totalLambda !== 0 ||
                contactPoint.tangentConstraint2.totalLambda !== 0;

            const radius = hasImpulse ? CONSTRAINT_POINT_RADIUS_ACTIVE : CONSTRAINT_POINT_RADIUS_INACTIVE;
            const colorA = hasImpulse ? constraintManifoldPointColorAActive : constraintManifoldPointColorA;
            const colorB = hasImpulse ? constraintManifoldPointColorBActive : constraintManifoldPointColorB;

            // point on body A (cyan)
            _position.set(contactPoint.positionA[0], contactPoint.positionA[1], contactPoint.positionA[2]);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(radius, radius, radius);
            _matrix.compose(_position, _quaternion, _scale);
            state.contactConstraints.spheresMesh.setMatrixAt(sphereIdx, _matrix);
            state.contactConstraints.spheresMesh.setColorAt(sphereIdx, colorA);
            sphereIdx++;

            // point on body B (magenta)
            _position.set(contactPoint.positionB[0], contactPoint.positionB[1], contactPoint.positionB[2]);
            _matrix.compose(_position, _quaternion, _scale);
            state.contactConstraints.spheresMesh.setMatrixAt(sphereIdx, _matrix);
            state.contactConstraints.spheresMesh.setColorAt(sphereIdx, colorB);
            sphereIdx++;

            // edge connecting to previous point on body A
            _currentPointA.set(contactPoint.positionA[0], contactPoint.positionA[1], contactPoint.positionA[2]);
            if (i > 0) {
                _edgeDir.subVectors(_currentPointA, _prevPointA);
                const edgeLength = _edgeDir.length();
                if (edgeLength > 0.001) {
                    _edgeDir.normalize();
                    _edgeMidpoint.addVectors(_prevPointA, _currentPointA).multiplyScalar(0.5);
                    _up.set(0, 1, 0);
                    _quaternion.setFromUnitVectors(_up, _edgeDir);
                    _scale.set(CONSTRAINT_EDGE_RADIUS, edgeLength, CONSTRAINT_EDGE_RADIUS);
                    _matrix.compose(_edgeMidpoint, _quaternion, _scale);
                    state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
                    state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintManifoldEdgeColor);
                    cylinderIdx++;
                }
            }
            _prevPointA.copy(_currentPointA);
        }

        // close the loop - connect last point to first
        if (constraint.numContactPoints > 2) {
            _firstPoint.set(
                constraint.contactPoints[0].positionA[0],
                constraint.contactPoints[0].positionA[1],
                constraint.contactPoints[0].positionA[2],
            );
            _edgeDir.subVectors(_firstPoint, _prevPointA);
            const edgeLength = _edgeDir.length();
            if (edgeLength > 0.001) {
                _edgeDir.normalize();
                _edgeMidpoint.addVectors(_prevPointA, _firstPoint).multiplyScalar(0.5);
                _up.set(0, 1, 0);
                _quaternion.setFromUnitVectors(_up, _edgeDir);
                _scale.set(CONSTRAINT_EDGE_RADIUS, edgeLength, CONSTRAINT_EDGE_RADIUS);
                _matrix.compose(_edgeMidpoint, _quaternion, _scale);
                state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
                state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintManifoldEdgeColor);
                cylinderIdx++;
            }
        }

        // normal arrow from first contact point
        const firstContactPoint = constraint.contactPoints[0];
        _normalStart.set(firstContactPoint.positionA[0], firstContactPoint.positionA[1], firstContactPoint.positionA[2]);
        _normalVec.set(constraint.normal[0], constraint.normal[1], constraint.normal[2]);

        const normalShaftLength = CONTACT_NORMAL_LENGTH - ARROW_TIP_HEIGHT;
        _up.set(0, 1, 0);
        _quaternion.setFromUnitVectors(_up, _normalVec);
        _normalShaftMidpoint.copy(_normalStart).addScaledVector(_normalVec, normalShaftLength / 2);
        _scale.set(ARROW_SHAFT_RADIUS, normalShaftLength, ARROW_SHAFT_RADIUS);
        _matrix.compose(_normalShaftMidpoint, _quaternion, _scale);
        state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
        state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintNormalColor);
        cylinderIdx++;

        // normal arrow tip
        _normalTipPosition.copy(_normalStart).addScaledVector(_normalVec, normalShaftLength + ARROW_TIP_HEIGHT / 2);
        _scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);
        _matrix.compose(_normalTipPosition, _quaternion, _scale);
        state.contactConstraints.conesMesh.setMatrixAt(coneIdx, _matrix);
        state.contactConstraints.conesMesh.setColorAt(coneIdx, constraintNormalColor);
        coneIdx++;

        // tangent1 line (green)
        _tangent1Vec.set(constraint.tangent1[0], constraint.tangent1[1], constraint.tangent1[2]);
        _quaternion.setFromUnitVectors(_up, _tangent1Vec);
        _tangent1Midpoint.copy(_normalStart).addScaledVector(_tangent1Vec, CONSTRAINT_TANGENT_LENGTH / 2);
        _scale.set(ARROW_SHAFT_RADIUS, CONSTRAINT_TANGENT_LENGTH, ARROW_SHAFT_RADIUS);
        _matrix.compose(_tangent1Midpoint, _quaternion, _scale);
        state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
        state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintTangent1Color);
        cylinderIdx++;

        // tangent2 line (blue)
        _tangent2Vec.set(constraint.tangent2[0], constraint.tangent2[1], constraint.tangent2[2]);
        _quaternion.setFromUnitVectors(_up, _tangent2Vec);
        _tangent2Midpoint.copy(_normalStart).addScaledVector(_tangent2Vec, CONSTRAINT_TANGENT_LENGTH / 2);
        _scale.set(ARROW_SHAFT_RADIUS, CONSTRAINT_TANGENT_LENGTH, ARROW_SHAFT_RADIUS);
        _matrix.compose(_tangent2Midpoint, _quaternion, _scale);
        state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
        state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintTangent2Color);
        cylinderIdx++;
    }

    // flag buffers as needing update
    state.contactConstraints.spheresMesh.instanceMatrix.needsUpdate = true;
    state.contactConstraints.cylindersMesh.instanceMatrix.needsUpdate = true;
    state.contactConstraints.conesMesh.instanceMatrix.needsUpdate = true;
    if (state.contactConstraints.spheresMesh.instanceColor) state.contactConstraints.spheresMesh.instanceColor.needsUpdate = true;
    if (state.contactConstraints.cylindersMesh.instanceColor)
        state.contactConstraints.cylindersMesh.instanceColor.needsUpdate = true;
    if (state.contactConstraints.conesMesh.instanceColor) state.contactConstraints.conesMesh.instanceColor.needsUpdate = true;
}

function updateBroadphaseDbvt(state: State, world: World): void {
    if (!state.options.broadphaseDbvt.enabled) {
        state.broadphase.dbvtLines.visible = false;
        return;
    }

    const positions: number[] = [];
    const colors: number[] = [];

    // Color palette for different broadphase layers (bright colors for leaf nodes)
    const layerColorsLeaf = [
        [0.0, 1.0, 0.0], // Green
        [1.0, 0.5, 0.0], // Orange
        [0.0, 0.5, 1.0], // Blue
        [1.0, 0.0, 1.0], // Magenta
        [1.0, 1.0, 0.0], // Yellow
        [0.0, 1.0, 1.0], // Cyan
    ];

    // Darker colors for non-leaf nodes (internal nodes)
    const layerColorsNonLeaf = [
        [0.0, 0.3, 0.0], // Dark green
        [0.3, 0.15, 0.0], // Dark orange
        [0.0, 0.15, 0.3], // Dark blue
        [0.3, 0.0, 0.3], // Dark magenta
        [0.3, 0.3, 0.0], // Dark yellow
        [0.0, 0.3, 0.3], // Dark cyan
    ];

    // Iterate through all broadphase layers
    for (let layerIndex = 0; layerIndex < world.broadphase.dbvts.length; layerIndex++) {
        const dbvt = world.broadphase.dbvts[layerIndex];
        if (dbvt.root === -1) continue;

        // Traverse DBVT and collect AABBs
        const stack: number[] = [dbvt.root];
        while (stack.length > 0) {
            const nodeIndex = stack.pop()!;
            const node = dbvt.nodes[nodeIndex];

            // Check if this is a leaf node (both left and right are -1)
            const isLeaf = node.left === -1 && node.right === -1;

            // Skip this node if it's filtered out by the options
            if (isLeaf && !state.options.broadphaseDbvt.showLeafNodes) {
                if (node.left !== -1) stack.push(node.left);
                if (node.right !== -1) stack.push(node.right);
                continue;
            }
            if (!isLeaf && !state.options.broadphaseDbvt.showNonLeafNodes) {
                if (node.left !== -1) stack.push(node.left);
                if (node.right !== -1) stack.push(node.right);
                continue;
            }

            const color = isLeaf
                ? layerColorsLeaf[layerIndex % layerColorsLeaf.length]
                : layerColorsNonLeaf[layerIndex % layerColorsNonLeaf.length];

            // Add box wireframe for this node
            const min = node.aabb[0];
            const max = node.aabb[1];

            // Bottom face (4 edges)
            positions.push(min[0], min[1], min[2], max[0], min[1], min[2]);
            colors.push(...color, ...color);
            positions.push(max[0], min[1], min[2], max[0], min[1], max[2]);
            colors.push(...color, ...color);
            positions.push(max[0], min[1], max[2], min[0], min[1], max[2]);
            colors.push(...color, ...color);
            positions.push(min[0], min[1], max[2], min[0], min[1], min[2]);
            colors.push(...color, ...color);

            // Top face (4 edges)
            positions.push(min[0], max[1], min[2], max[0], max[1], min[2]);
            colors.push(...color, ...color);
            positions.push(max[0], max[1], min[2], max[0], max[1], max[2]);
            colors.push(...color, ...color);
            positions.push(max[0], max[1], max[2], min[0], max[1], max[2]);
            colors.push(...color, ...color);
            positions.push(min[0], max[1], max[2], min[0], max[1], min[2]);
            colors.push(...color, ...color);

            // Vertical edges (4 edges)
            positions.push(min[0], min[1], min[2], min[0], max[1], min[2]);
            colors.push(...color, ...color);
            positions.push(max[0], min[1], min[2], max[0], max[1], min[2]);
            colors.push(...color, ...color);
            positions.push(max[0], min[1], max[2], max[0], max[1], max[2]);
            colors.push(...color, ...color);
            positions.push(min[0], min[1], max[2], min[0], max[1], max[2]);
            colors.push(...color, ...color);

            // Continue traversal
            if (node.left !== -1) stack.push(node.left);
            if (node.right !== -1) stack.push(node.right);
        }
    }

    // Update geometry
    const geometry = state.broadphase.dbvtLines.geometry;
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
    geometry.computeBoundingSphere();
    state.broadphase.dbvtLines.visible = true;
}

function updateVelocities(state: State, world: World): void {
    // Clear old linear velocity arrows
    if (state.bodies.linearVelocityArrows.size > 0) {
        for (const arrow of state.bodies.linearVelocityArrows.values()) {
            state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.shaftId);
            state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.tipId);
        }
        state.bodies.linearVelocityArrows.clear();
    }

    // Clear old angular velocity arrows
    if (state.bodies.angularVelocityArrows.size > 0) {
        for (const arrow of state.bodies.angularVelocityArrows.values()) {
            state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.shaftId);
            state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.tipId);
        }
        state.bodies.angularVelocityArrows.clear();
    }

    // Return early if neither is enabled
    if (!state.options.bodies.showLinearVelocity && !state.options.bodies.showAngularVelocity) {
        state.bodies.velocitiesBatchedMesh.visible = false;
        return;
    }

    // Make visible when at least one is enabled
    state.bodies.velocitiesBatchedMesh.visible = true;

    // Create geometries on first use
    if (state.bodies.velocityCylinderGeometryId === null) {
        state.bodies.velocityCylinderGeometryId = state.bodies.velocitiesBatchedMesh.addGeometry(createUnitCylinderGeometry());
    }
    if (state.bodies.velocityConeGeometryId === null) {
        state.bodies.velocityConeGeometryId = state.bodies.velocitiesBatchedMesh.addGeometry(createUnitConeGeometry());
    }

    const cylinderGeomId = state.bodies.velocityCylinderGeometryId;
    const coneGeomId = state.bodies.velocityConeGeometryId;

    // Iterate through all bodies
    for (const body of world.bodies.pool) {
        if (body._pooled) continue;

        // Skip static bodies (they have zero velocity)
        if (body.motionType === MotionType.STATIC) continue;

        const comX = body.centerOfMassPosition[0];
        const comY = body.centerOfMassPosition[1];
        const comZ = body.centerOfMassPosition[2];

        // Draw linear velocity arrow
        if (state.options.bodies.showLinearVelocity) {
            const velX = body.motionProperties.linearVelocity[0];
            const velY = body.motionProperties.linearVelocity[1];
            const velZ = body.motionProperties.linearVelocity[2];

            const velMag = Math.sqrt(velX * velX + velY * velY + velZ * velZ);

            // Only draw if velocity is significant
            if (velMag > 0.01) {
                const velNormX = velX / velMag;
                const velNormY = velY / velMag;
                const velNormZ = velZ / velMag;

                // Arrow shaft
                const shaftLength = Math.min(velMag, 5.0) - ARROW_TIP_HEIGHT; // Cap at 5.0 total
                const shaftMidX = comX + velNormX * shaftLength * 0.5;
                const shaftMidY = comY + velNormY * shaftLength * 0.5;
                const shaftMidZ = comZ + velNormZ * shaftLength * 0.5;

                _position.set(shaftMidX, shaftMidY, shaftMidZ);
                _up.set(0, 1, 0);
                _normalVec.set(velNormX, velNormY, velNormZ);
                _quaternion.setFromUnitVectors(_up, _normalVec);
                _scale.set(ARROW_SHAFT_RADIUS, shaftLength, ARROW_SHAFT_RADIUS);

                const shaftMatrix = new THREE.Matrix4();
                shaftMatrix.compose(_position, _quaternion, _scale);

                const shaftId = addInstanceToBatchedMesh(
                    state.bodies.velocitiesBatchedMesh,
                    cylinderGeomId,
                    shaftMatrix,
                    linearVelocityColor,
                );

                // Arrow tip
                const tipX = comX + velNormX * (shaftLength + ARROW_TIP_HEIGHT * 0.5);
                const tipY = comY + velNormY * (shaftLength + ARROW_TIP_HEIGHT * 0.5);
                const tipZ = comZ + velNormZ * (shaftLength + ARROW_TIP_HEIGHT * 0.5);

                _position.set(tipX, tipY, tipZ);
                _scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);

                const tipMatrix = new THREE.Matrix4();
                tipMatrix.compose(_position, _quaternion, _scale);

                const tipId = addInstanceToBatchedMesh(
                    state.bodies.velocitiesBatchedMesh,
                    coneGeomId,
                    tipMatrix,
                    linearVelocityColor,
                );

                state.bodies.linearVelocityArrows.set(body.id, { shaftId, tipId });
            }
        }

        // Draw angular velocity arrow
        if (state.options.bodies.showAngularVelocity) {
            const angVelX = body.motionProperties.angularVelocity[0];
            const angVelY = body.motionProperties.angularVelocity[1];
            const angVelZ = body.motionProperties.angularVelocity[2];

            const angVelMag = Math.sqrt(angVelX * angVelX + angVelY * angVelY + angVelZ * angVelZ);

            // Only draw if angular velocity is significant
            if (angVelMag > 0.01) {
                const angVelNormX = angVelX / angVelMag;
                const angVelNormY = angVelY / angVelMag;
                const angVelNormZ = angVelZ / angVelMag;

                // Arrow shaft - scale length by angular velocity magnitude
                const shaftLength = Math.min(angVelMag * 0.5, 3.0) - ARROW_TIP_HEIGHT; // Scale and cap
                const shaftMidX = comX + angVelNormX * shaftLength * 0.5;
                const shaftMidY = comY + angVelNormY * shaftLength * 0.5;
                const shaftMidZ = comZ + angVelNormZ * shaftLength * 0.5;

                _position.set(shaftMidX, shaftMidY, shaftMidZ);
                _up.set(0, 1, 0);
                _normalVec.set(angVelNormX, angVelNormY, angVelNormZ);
                _quaternion.setFromUnitVectors(_up, _normalVec);
                _scale.set(ARROW_SHAFT_RADIUS, shaftLength, ARROW_SHAFT_RADIUS);

                const shaftMatrix = new THREE.Matrix4();
                shaftMatrix.compose(_position, _quaternion, _scale);

                const shaftId = addInstanceToBatchedMesh(
                    state.bodies.velocitiesBatchedMesh,
                    cylinderGeomId,
                    shaftMatrix,
                    angularVelocityColor,
                );

                // Arrow tip
                const tipX = comX + angVelNormX * (shaftLength + ARROW_TIP_HEIGHT * 0.5);
                const tipY = comY + angVelNormY * (shaftLength + ARROW_TIP_HEIGHT * 0.5);
                const tipZ = comZ + angVelNormZ * (shaftLength + ARROW_TIP_HEIGHT * 0.5);

                _position.set(tipX, tipY, tipZ);
                _scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);

                const tipMatrix = new THREE.Matrix4();
                tipMatrix.compose(_position, _quaternion, _scale);

                const tipId = addInstanceToBatchedMesh(
                    state.bodies.velocitiesBatchedMesh,
                    coneGeomId,
                    tipMatrix,
                    angularVelocityColor,
                );

                state.bodies.angularVelocityArrows.set(body.id, { shaftId, tipId });
            }
        }
    }
}

/**
 * Helper to collect triangle mesh shapes from a shape hierarchy.
 * Returns array of { shape, worldMatrix } for each triangle mesh found.
 */
function collectTriangleMeshShapes(
    shape: Shape,
    parentMatrix: THREE.Matrix4,
    output: { shape: Shape & { type: ShapeType.TRIANGLE_MESH }; worldMatrix: THREE.Matrix4 }[],
): void {
    switch (shape.type) {
        case ShapeType.TRIANGLE_MESH:
            output.push({ shape, worldMatrix: parentMatrix.clone() });
            break;

        case ShapeType.COMPOUND:
            for (const child of shape.children) {
                const [tx, ty, tz] = child.position;
                const [qx, qy, qz, qw] = child.quaternion;
                _position.set(tx, ty, tz);
                _quaternion.set(qx, qy, qz, qw);
                _scale.set(1, 1, 1);
                _matrix.compose(_position, _quaternion, _scale);
                const childMatrix = parentMatrix.clone().multiply(_matrix);
                collectTriangleMeshShapes(child.shape, childMatrix, output);
            }
            break;

        case ShapeType.TRANSFORMED: {
            const [tx, ty, tz] = shape.position;
            const [qx, qy, qz, qw] = shape.quaternion;
            _position.set(tx, ty, tz);
            _quaternion.set(qx, qy, qz, qw);
            _scale.set(1, 1, 1);
            _matrix.compose(_position, _quaternion, _scale);
            const transformedMatrix = parentMatrix.clone().multiply(_matrix);
            collectTriangleMeshShapes(shape.shape, transformedMatrix, output);
            break;
        }

        case ShapeType.SCALED: {
            const [sx, sy, sz] = shape.scale;
            _position.set(0, 0, 0);
            _quaternion.set(0, 0, 0, 1);
            _scale.set(sx, sy, sz);
            _matrix.compose(_position, _quaternion, _scale);
            const scaledMatrix = parentMatrix.clone().multiply(_matrix);
            collectTriangleMeshShapes(shape.shape, scaledMatrix, output);
            break;
        }

        // Other shape types don't contain triangle meshes
        default:
            break;
    }
}

/**
 * Add wireframe box edges to position/color arrays for a BVH AABB,
 * transformed by a given world matrix.
 */
function addBvhBoxWireframe(
    positions: number[],
    colors: number[],
    aabbMin: readonly [number, number, number],
    aabbMax: readonly [number, number, number],
    worldMatrix: THREE.Matrix4,
    color: readonly [number, number, number],
): void {
    const min = aabbMin;
    const max = aabbMax;

    // Transform all 8 corners
    const corners = [
        new THREE.Vector3(min[0], min[1], min[2]),
        new THREE.Vector3(max[0], min[1], min[2]),
        new THREE.Vector3(max[0], min[1], max[2]),
        new THREE.Vector3(min[0], min[1], max[2]),
        new THREE.Vector3(min[0], max[1], min[2]),
        new THREE.Vector3(max[0], max[1], min[2]),
        new THREE.Vector3(max[0], max[1], max[2]),
        new THREE.Vector3(min[0], max[1], max[2]),
    ];

    for (const corner of corners) {
        corner.applyMatrix4(worldMatrix);
    }

    // 12 edges of a box
    const edges: [number, number][] = [
        // Bottom face
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 0],
        // Top face
        [4, 5],
        [5, 6],
        [6, 7],
        [7, 4],
        // Vertical edges
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ];

    for (const [a, b] of edges) {
        positions.push(corners[a].x, corners[a].y, corners[a].z);
        positions.push(corners[b].x, corners[b].y, corners[b].z);
        colors.push(color[0], color[1], color[2]);
        colors.push(color[0], color[1], color[2]);
    }
}

function updateTriangleMeshBvh(state: State, world: World): void {
    if (!state.options.triangleMeshBvh.enabled) {
        state.triangleMeshBvh.container.visible = false;
        return;
    }

    state.triangleMeshBvh.container.visible = true;

    const showLeafNodes = state.options.triangleMeshBvh.showLeafNodes;
    const showNonLeafNodes = state.options.triangleMeshBvh.showNonLeafNodes;

    // Invalidate cache if filter options changed
    if (
        showLeafNodes !== state.triangleMeshBvh.previousShowLeafNodes ||
        showNonLeafNodes !== state.triangleMeshBvh.previousShowNonLeafNodes
    ) {
        // Clear all cached visualizations
        for (const [_bodyId, lineSegments] of state.triangleMeshBvh.cache) {
            state.triangleMeshBvh.container.remove(lineSegments);
            lineSegments.geometry.dispose();
            if (lineSegments.material instanceof THREE.Material) {
                lineSegments.material.dispose();
            }
        }
        state.triangleMeshBvh.cache.clear();
        state.triangleMeshBvh.previousShowLeafNodes = showLeafNodes;
        state.triangleMeshBvh.previousShowNonLeafNodes = showNonLeafNodes;
    }

    // Track which bodies we've seen this frame
    const seenBodyIds = new Set<number>();

    // Iterate through all rigid bodies
    for (const body of rigidBody.iterate(world)) {
        seenBodyIds.add(body.id);

        // Check if we already have cached visualization for this body
        if (state.triangleMeshBvh.cache.has(body.id)) {
            // Update the transform of existing line segments
            const cachedLines = state.triangleMeshBvh.cache.get(body.id)!;
            _position.set(body.position[0], body.position[1], body.position[2]);
            _quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
            _scale.set(1, 1, 1);
            cachedLines.position.copy(_position);
            cachedLines.quaternion.copy(_quaternion);
            cachedLines.scale.copy(_scale);
            continue;
        }

        // No cache - check if this body has triangle mesh shapes
        const identityMatrix = new THREE.Matrix4();
        const triangleMeshes: { shape: Shape & { type: ShapeType.TRIANGLE_MESH }; worldMatrix: THREE.Matrix4 }[] = [];
        collectTriangleMeshShapes(body.shape, identityMatrix, triangleMeshes);

        if (triangleMeshes.length === 0) {
            // No triangle meshes in this body - cache empty marker
            // We use a dummy empty LineSegments so we don't re-check every frame
            const emptyLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
            emptyLines.visible = false;
            state.triangleMeshBvh.cache.set(body.id, emptyLines);
            state.triangleMeshBvh.container.add(emptyLines);
            continue;
        }

        // Create combined visualization for all triangle meshes in this body
        const positions: number[] = [];
        const colors: number[] = [];

        for (const { shape: meshShape, worldMatrix } of triangleMeshes) {
            const buffer = meshShape.bvh.buffer;
            if (buffer.length === 0) continue;

            // Color palette by depth
            const leafColors: [number, number, number][] = [
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 0.5],
                [0.0, 0.8, 1.0],
                [0.0, 0.5, 1.0],
                [0.5, 0.0, 1.0],
                [1.0, 0.0, 1.0],
            ];

            const nonLeafColors: [number, number, number][] = [
                [0.0, 0.4, 0.0],
                [0.0, 0.4, 0.2],
                [0.0, 0.3, 0.4],
                [0.0, 0.2, 0.4],
                [0.2, 0.0, 0.4],
                [0.4, 0.0, 0.4],
            ];

            // Traverse BVH using flat buffer
            const stack: { offset: number; depth: number }[] = [{ offset: 0, depth: 0 }];
            while (stack.length > 0) {
                const { offset, depth } = stack.pop()!;

                const isLeaf = triangleMeshBvh.nodeIsLeaf(buffer, offset);

                if (isLeaf && !showLeafNodes) {
                    continue;
                }
                if (!isLeaf && !showNonLeafNodes) {
                    stack.push({ offset: triangleMeshBvh.nodeLeft(offset), depth: depth + 1 });
                    stack.push({ offset: triangleMeshBvh.nodeRight(buffer, offset), depth: depth + 1 });
                    continue;
                }

                const colorPalette = isLeaf ? leafColors : nonLeafColors;
                const color = colorPalette[depth % colorPalette.length];

                const aabbMin: [number, number, number] = [
                    buffer[offset + triangleMeshBvh.NODE_MIN_X],
                    buffer[offset + triangleMeshBvh.NODE_MIN_Y],
                    buffer[offset + triangleMeshBvh.NODE_MIN_Z],
                ];
                const aabbMax: [number, number, number] = [
                    buffer[offset + triangleMeshBvh.NODE_MAX_X],
                    buffer[offset + triangleMeshBvh.NODE_MAX_Y],
                    buffer[offset + triangleMeshBvh.NODE_MAX_Z],
                ];

                addBvhBoxWireframe(positions, colors, aabbMin, aabbMax, worldMatrix, color);

                if (!isLeaf) {
                    stack.push({ offset: triangleMeshBvh.nodeLeft(offset), depth: depth + 1 });
                    stack.push({ offset: triangleMeshBvh.nodeRight(buffer, offset), depth: depth + 1 });
                }
            }
        }

        // Create the line segments for this body
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));

        const material = new THREE.LineBasicMaterial({ vertexColors: true });
        const lineSegments = new THREE.LineSegments(geometry, material);
        lineSegments.frustumCulled = false;

        // Set transform
        _position.set(body.position[0], body.position[1], body.position[2]);
        _quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
        _scale.set(1, 1, 1);
        lineSegments.position.copy(_position);
        lineSegments.quaternion.copy(_quaternion);
        lineSegments.scale.copy(_scale);

        state.triangleMeshBvh.cache.set(body.id, lineSegments);
        state.triangleMeshBvh.container.add(lineSegments);
    }

    // Remove cached entries for bodies that no longer exist
    for (const [bodyId, lineSegments] of state.triangleMeshBvh.cache) {
        if (!seenBodyIds.has(bodyId)) {
            state.triangleMeshBvh.container.remove(lineSegments);
            lineSegments.geometry.dispose();
            if (lineSegments.material instanceof THREE.Material) {
                lineSegments.material.dispose();
            }
            state.triangleMeshBvh.cache.delete(bodyId);
        }
    }
}

export function update(state: State, world: World): void {
    updateBodies(state, world);
    updateContacts(state, world);
    updateContactConstraints(state, world);
    updateBroadphaseDbvt(state, world);
    drawConstraints(state, world);
    updateVelocities(state, world);
    updateTriangleMeshBvh(state, world);
}

// Helper to add a line to constraints visualization
function addConstraintLine(
    positions: number[],
    colors: number[],
    from: [number, number, number],
    to: [number, number, number],
    color: [number, number, number],
): void {
    positions.push(from[0], from[1], from[2], to[0], to[1], to[2]);
    colors.push(...color, ...color);
}

// Helper to add a marker (3 axis lines) to constraints visualization
function addConstraintMarker(
    positions: number[],
    colors: number[],
    pos: [number, number, number],
    size: number,
    color: [number, number, number],
): void {
    addConstraintLine(positions, colors, [pos[0] - size, pos[1], pos[2]], [pos[0] + size, pos[1], pos[2]], color);
    addConstraintLine(positions, colors, [pos[0], pos[1] - size, pos[2]], [pos[0], pos[1] + size, pos[2]], color);
    addConstraintLine(positions, colors, [pos[0], pos[1], pos[2] - size], [pos[0], pos[1], pos[2] + size], color);
}

// Body transform type for constraint helpers
type BodyTransform = {
    position: Vec3;
    quaternion: Quat;
    centerOfMassPosition: Vec3;
} | null;

const _transformPointOut = vec3.create();
const _transformDirOut = vec3.create();
const _transformPointOut2 = vec3.create();

const _transformPoint_rotated = vec3.create();

const _constraint_twistAxis = vec3.create();
const _constraint_planeAxis = vec3.create();
const _constraint_normalAxis = vec3.create();
const _constraint_xAxis = vec3.create();
const _constraint_yAxis = vec3.create();
const _constraint_zAxis = vec3.create();
const _constraint_quat = quat.create();
const _constraint_normal1 = vec3.create();

// Helper to transform local point to world space
function transformPointToWorld(out: Vec3, localPos: Vec3, bodyTransform: BodyTransform): void {
    if (!bodyTransform) {
        vec3.set(out, 0, 0, 0);
        return;
    }

    // Rotate local position by quaternion and add center of mass position
    vec3.transformQuat(_transformPoint_rotated, localPos, bodyTransform.quaternion);
    vec3.add(out, _transformPoint_rotated, bodyTransform.centerOfMassPosition);
}

// Helper to transform local direction to world space (no translation)
function transformDirectionToWorld(out: Vec3, localDir: Vec3, bodyTransform: BodyTransform): void {
    if (!bodyTransform) {
        vec3.set(out, 0, 0, 0);
        return;
    }

    // Rotate direction by quaternion (no translation)
    vec3.transformQuat(out, localDir, bodyTransform.quaternion);
}

const _pieRotationQuat = quat.create();
const _pieAxisVec = vec3.create();
const _pieRotatedAxis = vec3.create();

// Helper to rotate a vector around an axis by an angle (using quaternion rotation)
function rotateVectorAroundAxis(out: Vec3, vector: Vec3, axis: Vec3, angle: number): void {
    quat.setAxisAngle(_pieRotationQuat, axis, angle);
    vec3.transformQuat(out, vector, _pieRotationQuat);
}

// Helper to draw a pie/arc shape for angular limits
function drawPie(
    positions: number[],
    colors: number[],
    center: [number, number, number],
    radius: number,
    normal: Vec3,
    axis: Vec3,
    minAngle: number,
    maxAngle: number,
    color: [number, number, number],
): void {
    if (minAngle >= maxAngle) return;

    const numSegments = 32;
    const deltaAngle = maxAngle - minAngle;
    const angleStep = deltaAngle / numSegments;

    vec3.copy(_pieAxisVec, axis);

    // Draw radial lines from center
    let prevPoint: [number, number, number] | null = null;
    for (let i = 0; i <= numSegments; i++) {
        const angle = minAngle + i * angleStep;
        rotateVectorAroundAxis(_pieRotatedAxis, _pieAxisVec, normal, angle);
        const point: [number, number, number] = [
            center[0] + _pieRotatedAxis[0] * radius,
            center[1] + _pieRotatedAxis[1] * radius,
            center[2] + _pieRotatedAxis[2] * radius,
        ];

        // Line from center to arc
        addConstraintLine(positions, colors, center, point, color);

        // Line along arc
        if (prevPoint !== null) {
            addConstraintLine(positions, colors, prevPoint, point, color);
        }
        prevPoint = point;
    }
}

// Helper to draw cone limits for swing constraints
function drawSwingConeLimits(
    positions: number[],
    colors: number[],
    center: [number, number, number],
    twistAxis: Vec3,
    planeAxis: Vec3,
    normalAxis: Vec3,
    planeHalfAngle: number,
    normalHalfAngle: number,
    radius: number,
    color: [number, number, number],
): void {
    // Use the maximum of the two angles for the cone
    const maxAngle = Math.max(planeHalfAngle, normalHalfAngle);
    if (maxAngle <= 0) return;

    const numSegments = 32;
    const angleStep = (2 * Math.PI) / numSegments;

    // Draw cone outline
    let prevPoint: [number, number, number] | null = null;
    for (let i = 0; i <= numSegments; i++) {
        const theta = i * angleStep;

        // Calculate point on cone surface
        // Point is at angle maxAngle from twist axis, rotated around it
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const cosMaxAngle = Math.cos(maxAngle);
        const sinMaxAngle = Math.sin(maxAngle);

        // Direction on cone surface
        const dirX = twistAxis[0] * cosMaxAngle + (planeAxis[0] * cosTheta + normalAxis[0] * sinTheta) * sinMaxAngle;
        const dirY = twistAxis[1] * cosMaxAngle + (planeAxis[1] * cosTheta + normalAxis[1] * sinTheta) * sinMaxAngle;
        const dirZ = twistAxis[2] * cosMaxAngle + (planeAxis[2] * cosTheta + normalAxis[2] * sinTheta) * sinMaxAngle;

        const point: [number, number, number] = [center[0] + dirX * radius, center[1] + dirY * radius, center[2] + dirZ * radius];

        // Line from center to cone surface
        if (i % 4 === 0) {
            addConstraintLine(positions, colors, center, point, color);
        }

        // Line along cone outline
        if (prevPoint !== null) {
            addConstraintLine(positions, colors, prevPoint, point, color);
        }
        prevPoint = point;
    }
}

// Draw all constraints
function drawConstraints(state: State, world: World): void {
    if (!state.options.constraints.enabled) {
        state.constraints.lineSegments.visible = false;
        return;
    }

    state.constraints.lineSegments.visible = true;
    const positions: number[] = [];
    const colors: number[] = [];
    const size = state.options.constraints.size;
    const drawLimits = state.options.constraints.drawLimits;

    const redColor: [number, number, number] = [1, 0, 0];
    const greenColor: [number, number, number] = [0, 1, 0];
    const blueColor: [number, number, number] = [0, 0, 1];
    const whiteColor: [number, number, number] = [1, 1, 1];
    const purpleColor: [number, number, number] = [0.5, 0, 0.5];
    const yellowColor: [number, number, number] = [1, 1, 0];

    const constraints = world.constraints;
    const bodies = world.bodies.pool;

    // Helper to get body transform
    const getBodyTransform = (bodyIndex: number): BodyTransform => {
        const body = bodies[bodyIndex];
        if (!body || body._pooled) return null;
        return {
            position: body.position,
            quaternion: body.quaternion,
            centerOfMassPosition: body.centerOfMassPosition,
        };
    };

    // Draw hinge constraints
    const hingePool = constraints.pools[ConstraintType.HINGE];
    if (hingePool) {
    for (const constraint of hingePool.constraints as HingeConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
        transformDirectionToWorld(_transformDirOut, constraint.localSpaceHingeAxis1, bodyA);

        const pos1 = _transformPointOut;
        const pos2 = _transformPointOut2;
        const axis1 = _transformDirOut;

        addConstraintMarker(positions, colors, pos1, size * 0.1, redColor);
        addConstraintLine(
            positions,
            colors,
            pos1,
            [pos1[0] + axis1[0] * size, pos1[1] + axis1[1] * size, pos1[2] + axis1[2] * size],
            redColor,
        );

        addConstraintMarker(positions, colors, pos2, size * 0.1, greenColor);

        // Transform normal axis for limit drawing
        if (drawLimits && constraint.hasLimits && constraint.limitsMax > constraint.limitsMin) {
            vec3.set(
                _constraint_normal1,
                constraint.localSpaceNormalAxis1[0],
                constraint.localSpaceNormalAxis1[1],
                constraint.localSpaceNormalAxis1[2],
            );
            transformDirectionToWorld(_constraint_normal1, _constraint_normal1, bodyA);
            addConstraintLine(
                positions,
                colors,
                pos2,
                [
                    pos2[0] + _constraint_normal1[0] * size,
                    pos2[1] + _constraint_normal1[1] * size,
                    pos2[2] + _constraint_normal1[2] * size,
                ],
                whiteColor,
            );
            drawPie(
                positions,
                colors,
                pos1,
                size,
                axis1,
                _constraint_normal1,
                constraint.limitsMin,
                constraint.limitsMax,
                purpleColor,
            );
        } else {
            vec3.set(
                _constraint_normal1,
                constraint.localSpaceNormalAxis1[0],
                constraint.localSpaceNormalAxis1[1],
                constraint.localSpaceNormalAxis1[2],
            );
            transformDirectionToWorld(_constraint_normal1, _constraint_normal1, bodyA);
            addConstraintLine(
                positions,
                colors,
                pos2,
                [
                    pos2[0] + _constraint_normal1[0] * size,
                    pos2[1] + _constraint_normal1[1] * size,
                    pos2[2] + _constraint_normal1[2] * size,
                ],
                whiteColor,
            );
        }
    }
    }

    // Draw swing-twist constraints
    const swingTwistPool = constraints.pools[ConstraintType.SWING_TWIST];
    if (swingTwistPool) {
    for (const constraint of swingTwistPool.constraints as SwingTwistConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);

        const pos1 = _transformPointOut;
        const pos2 = _transformPointOut2;

        addConstraintMarker(positions, colors, pos1, size * 0.1, redColor);
        addConstraintMarker(positions, colors, pos2, size * 0.1, greenColor);
        addConstraintLine(positions, colors, pos1, pos2, whiteColor);

        // Draw limits if enabled
        if (drawLimits) {
            // Get constraint space orientation in world space
            // constraintToBody1 transforms from constraint space to body1 space
            // So: worldQuat = bodyA.quaternion * constraintToBody1
            const c2b1 = constraint.constraintToBody1;
            const qA = bodyA.quaternion;

            // Multiply quaternions: qA * c2b1
            quat.multiply(_constraint_quat, qA, c2b1 as Quat);

            // Get constraint space axes in world space
            // In constraint space: X = twist axis, Y = plane axis, Z = normal axis
            const constraintTransform: BodyTransform = { ...bodyA, quaternion: _constraint_quat };

            vec3.set(_constraint_xAxis, 1, 0, 0);
            vec3.set(_constraint_yAxis, 0, 1, 0);
            vec3.set(_constraint_zAxis, 0, 0, 1);

            transformDirectionToWorld(_constraint_twistAxis, _constraint_xAxis, constraintTransform);
            transformDirectionToWorld(_constraint_planeAxis, _constraint_yAxis, constraintTransform);
            transformDirectionToWorld(_constraint_normalAxis, _constraint_zAxis, constraintTransform);

            // Draw swing cone limits
            const hasSwingLimits = constraint.planeHalfConeAngle > 0 || constraint.normalHalfConeAngle > 0;
            if (hasSwingLimits) {
                drawSwingConeLimits(
                    positions,
                    colors,
                    pos1,
                    _constraint_twistAxis,
                    _constraint_planeAxis,
                    _constraint_normalAxis,
                    constraint.planeHalfConeAngle,
                    constraint.normalHalfConeAngle,
                    size,
                    greenColor,
                );
            }

            // Draw twist limits as a pie around the twist axis
            const hasTwistLimits = constraint.twistMaxAngle > constraint.twistMinAngle;
            if (hasTwistLimits) {
                drawPie(
                    positions,
                    colors,
                    pos1,
                    size,
                    _constraint_twistAxis,
                    _constraint_planeAxis,
                    constraint.twistMinAngle,
                    constraint.twistMaxAngle,
                    purpleColor,
                );
            }
        }
    }
    }

    // Draw distance constraints
    const distancePool = constraints.pools[ConstraintType.DISTANCE];
    if (distancePool) {
    for (const constraint of distancePool.constraints as DistanceConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);

        const pos1 = _transformPointOut;
        const pos2 = _transformPointOut2;

        addConstraintLine(positions, colors, pos1, pos2, greenColor);

        // Draw limits if enabled
        if (drawLimits && constraint.minDistance !== constraint.maxDistance) {
            // Draw spheres at min and max distances from pos1
            const direction: [number, number, number] = [pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]];
            const currentDistance = Math.sqrt(
                direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2],
            );

            if (currentDistance > 0.001) {
                // Normalize direction
                const invLen = 1 / currentDistance;
                direction[0] *= invLen;
                direction[1] *= invLen;
                direction[2] *= invLen;

                // Draw min distance sphere (as circle segments)
                if (constraint.minDistance > 0) {
                    const numSegments = 16;
                    const angleStep = (2 * Math.PI) / numSegments;

                    // Find a perpendicular vector to direction
                    let perpX: number, perpY: number, perpZ: number;
                    if (Math.abs(direction[0]) < 0.9) {
                        perpX = 0;
                        perpY = -direction[2];
                        perpZ = direction[1];
                    } else {
                        perpX = -direction[1];
                        perpY = direction[0];
                        perpZ = 0;
                    }
                    const perpLen = Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
                    perpX /= perpLen;
                    perpY /= perpLen;
                    perpZ /= perpLen;

                    let prevPoint: [number, number, number] | null = null;
                    for (let i = 0; i <= numSegments; i++) {
                        const angle = i * angleStep;
                        const cos = Math.cos(angle);

                        // Rotate perpendicular vector around direction
                        const px = pos1[0] + perpX * cos * constraint.minDistance;
                        const py = pos1[1] + perpY * cos * constraint.minDistance;
                        const pz = pos1[2] + perpZ * cos * constraint.minDistance;

                        const point: [number, number, number] = [px, py, pz];

                        if (prevPoint !== null) {
                            addConstraintLine(positions, colors, prevPoint, point, blueColor);
                        }
                        prevPoint = point;
                    }
                }

                // Draw max distance sphere (as circle segments)
                if (constraint.maxDistance < Infinity) {
                    const numSegments = 16;
                    const angleStep = (2 * Math.PI) / numSegments;

                    // Find a perpendicular vector to direction
                    let perpX: number, perpY: number, perpZ: number;
                    if (Math.abs(direction[0]) < 0.9) {
                        perpX = 0;
                        perpY = -direction[2];
                        perpZ = direction[1];
                    } else {
                        perpX = -direction[1];
                        perpY = direction[0];
                        perpZ = 0;
                    }
                    const perpLen = Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
                    perpX /= perpLen;
                    perpY /= perpLen;
                    perpZ /= perpLen;

                    let prevPoint: [number, number, number] | null = null;
                    for (let i = 0; i <= numSegments; i++) {
                        const angle = i * angleStep;
                        const cos = Math.cos(angle);

                        const px = pos1[0] + perpX * cos * constraint.maxDistance;
                        const py = pos1[1] + perpY * cos * constraint.maxDistance;
                        const pz = pos1[2] + perpZ * cos * constraint.maxDistance;

                        const point: [number, number, number] = [px, py, pz];

                        if (prevPoint !== null) {
                            addConstraintLine(positions, colors, prevPoint, point, redColor);
                        }
                        prevPoint = point;
                    }
                }
            }
        }
    }
    }

    // Draw cone constraints
    const conePool = constraints.pools[ConstraintType.CONE];
    if (conePool) {
    for (const constraint of conePool.constraints as ConeConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
        transformDirectionToWorld(_transformDirOut, constraint.localSpaceTwistAxis1, bodyA);

        const pos1 = _transformPointOut;
        const pos2 = _transformPointOut2;
        const axis1 = _transformDirOut;

        addConstraintMarker(positions, colors, pos1, size * 0.1, redColor);
        addConstraintLine(
            positions,
            colors,
            pos1,
            [pos1[0] + axis1[0] * size, pos1[1] + axis1[1] * size, pos1[2] + axis1[2] * size],
            redColor,
        );

        const axis2: [number, number, number] = [0, 0, 0];
        transformDirectionToWorld(axis2, constraint.localSpaceTwistAxis2, bodyB);

        addConstraintMarker(positions, colors, pos2, size * 0.1, greenColor);
        addConstraintLine(
            positions,
            colors,
            pos2,
            [pos2[0] + axis2[0] * size, pos2[1] + axis2[1] * size, pos2[2] + axis2[2] * size],
            greenColor,
        );

        // Draw limits if enabled
        if (drawLimits && constraint.cosHalfConeAngle < 1) {
            const halfConeAngle = Math.acos(constraint.cosHalfConeAngle);

            // Get perpendicular axes to twist axis
            const perpX: [number, number, number] = [0, 0, 0];
            const perpY: [number, number, number] = [0, 0, 0];

            // Find first perpendicular
            if (Math.abs(axis1[0]) < 0.9) {
                perpX[0] = 0;
                perpX[1] = -axis1[2];
                perpX[2] = axis1[1];
            } else {
                perpX[0] = -axis1[1];
                perpX[1] = axis1[0];
                perpX[2] = 0;
            }
            const perpXLen = Math.sqrt(perpX[0] * perpX[0] + perpX[1] * perpX[1] + perpX[2] * perpX[2]);
            perpX[0] /= perpXLen;
            perpX[1] /= perpXLen;
            perpX[2] /= perpXLen;

            // Second perpendicular = twist axis × first perpendicular
            perpY[0] = axis1[1] * perpX[2] - axis1[2] * perpX[1];
            perpY[1] = axis1[2] * perpX[0] - axis1[0] * perpX[2];
            perpY[2] = axis1[0] * perpX[1] - axis1[1] * perpX[0];

            // Draw cone using drawSwingConeLimits helper
            drawSwingConeLimits(positions, colors, pos1, axis1, perpX, perpY, halfConeAngle, halfConeAngle, size, yellowColor);
        }
    }
    }

    // Draw fixed constraints
    const fixedPool = constraints.pools[ConstraintType.FIXED];
    if (fixedPool) {
    for (const constraint of fixedPool.constraints as FixedConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);

        addConstraintMarker(positions, colors, _transformPointOut, size * 0.1, blueColor);
        addConstraintMarker(positions, colors, _transformPointOut2, size * 0.1, blueColor);
    }
    }

    // Draw point constraints
    const pointPool = constraints.pools[ConstraintType.POINT];
    if (pointPool) {
    for (const constraint of pointPool.constraints as PointConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);

        addConstraintMarker(positions, colors, _transformPointOut, size * 0.1, whiteColor);
        addConstraintMarker(positions, colors, _transformPointOut2, size * 0.1, whiteColor);
        addConstraintLine(positions, colors, _transformPointOut, _transformPointOut2, whiteColor);
    }
    }

    // Draw slider constraints
    const sliderPool = constraints.pools[ConstraintType.SLIDER];
    if (sliderPool) {
    for (const constraint of sliderPool.constraints as SliderConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePositionA, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePositionB, bodyB);
        transformDirectionToWorld(_transformDirOut, constraint.localSpaceSliderAxisA, bodyA);

        const pos1 = _transformPointOut;
        const pos2 = _transformPointOut2;
        const axis1 = _transformDirOut;

        addConstraintMarker(positions, colors, pos1, size * 0.1, redColor);
        addConstraintLine(
            positions,
            colors,
            pos1,
            [pos1[0] + axis1[0] * size, pos1[1] + axis1[1] * size, pos1[2] + axis1[2] * size],
            redColor,
        );

        addConstraintMarker(positions, colors, pos2, size * 0.1, greenColor);

        // Draw limits if enabled
        if (drawLimits && constraint.hasLimits) {
            // Draw min limit marker (blue)
            if (constraint.limitsMin > -Infinity) {
                const minPos: [number, number, number] = [
                    pos1[0] + axis1[0] * constraint.limitsMin,
                    pos1[1] + axis1[1] * constraint.limitsMin,
                    pos1[2] + axis1[2] * constraint.limitsMin,
                ];
                addConstraintMarker(positions, colors, minPos, size * 0.15, blueColor);
            }

            // Draw max limit marker (red)
            if (constraint.limitsMax < Infinity) {
                const maxPos: [number, number, number] = [
                    pos1[0] + axis1[0] * constraint.limitsMax,
                    pos1[1] + axis1[1] * constraint.limitsMax,
                    pos1[2] + axis1[2] * constraint.limitsMax,
                ];
                addConstraintMarker(positions, colors, maxPos, size * 0.15, redColor);
            }

            // Draw line showing the slider range
            if (constraint.limitsMin > -Infinity && constraint.limitsMax < Infinity) {
                const minPos: [number, number, number] = [
                    pos1[0] + axis1[0] * constraint.limitsMin,
                    pos1[1] + axis1[1] * constraint.limitsMin,
                    pos1[2] + axis1[2] * constraint.limitsMin,
                ];
                const maxPos: [number, number, number] = [
                    pos1[0] + axis1[0] * constraint.limitsMax,
                    pos1[1] + axis1[1] * constraint.limitsMax,
                    pos1[2] + axis1[2] * constraint.limitsMax,
                ];
                addConstraintLine(positions, colors, minPos, maxPos, purpleColor);
            }
        }
    }
    }

    // Draw six DOF constraints
    const sixDOFPool = constraints.pools[ConstraintType.SIX_DOF];
    if (sixDOFPool) {
    for (const constraint of sixDOFPool.constraints as SixDOFConstraint[]) {
        if (constraint._pooled || !constraint.enabled) continue;

        const bodyA = getBodyTransform(constraint.bodyIndexA);
        const bodyB = getBodyTransform(constraint.bodyIndexB);
        if (!bodyA || !bodyB) continue;

        transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
        transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);

        const pos1 = _transformPointOut;
        const pos2 = _transformPointOut2;

        addConstraintMarker(positions, colors, pos1, size * 0.1, purpleColor);
        addConstraintMarker(positions, colors, pos2, size * 0.1, purpleColor);

        // Draw limits if enabled
        if (drawLimits) {
            // Get constraint space orientation in world space (similar to swing-twist)
            const c2b1 = constraint.constraintToBody1;
            const qA = bodyA.quaternion;

            // Multiply quaternions: qA * c2b1
            const constraintQuat: [number, number, number, number] = [
                qA[3] * c2b1[0] + qA[0] * c2b1[3] + qA[1] * c2b1[2] - qA[2] * c2b1[1],
                qA[3] * c2b1[1] + qA[1] * c2b1[3] + qA[2] * c2b1[0] - qA[0] * c2b1[2],
                qA[3] * c2b1[2] + qA[2] * c2b1[3] + qA[0] * c2b1[1] - qA[1] * c2b1[0],
                qA[3] * c2b1[3] - qA[0] * c2b1[0] - qA[1] * c2b1[1] - qA[2] * c2b1[2],
            ];

            // Get constraint space axes in world space
            const constraintTransform: BodyTransform = { ...bodyA, quaternion: constraintQuat };
            const axisX: [number, number, number] = [0, 0, 0];
            const axisY: [number, number, number] = [0, 0, 0];
            const axisZ: [number, number, number] = [0, 0, 0];

            transformDirectionToWorld(axisX, [1, 0, 0], constraintTransform);
            transformDirectionToWorld(axisY, [0, 1, 0], constraintTransform);
            transformDirectionToWorld(axisZ, [0, 0, 1], constraintTransform);

            const axes = [axisX, axisY, axisZ];
            const axisColors: Array<[number, number, number]> = [redColor, greenColor, blueColor];

            // Draw translation limits for X, Y, Z axes
            for (let i = 0; i < 3; i++) {
                const minLimit = constraint.limitMin[i];
                const maxLimit = constraint.limitMax[i];
                const axis = axes[i];
                const color = axisColors[i];

                // Check if this axis has limits (not free)
                const isFree = minLimit <= -3.4e38 && maxLimit >= 3.4e38;
                const isFixed = minLimit >= 3.4e38 && maxLimit <= -3.4e38;

                if (!isFree && !isFixed && minLimit < maxLimit) {
                    // Draw min limit marker
                    if (minLimit > -3.4e38) {
                        const minPos: [number, number, number] = [
                            pos1[0] + axis[0] * minLimit,
                            pos1[1] + axis[1] * minLimit,
                            pos1[2] + axis[2] * minLimit,
                        ];
                        addConstraintMarker(positions, colors, minPos, size * 0.08, color);
                    }

                    // Draw max limit marker
                    if (maxLimit < 3.4e38) {
                        const maxPos: [number, number, number] = [
                            pos1[0] + axis[0] * maxLimit,
                            pos1[1] + axis[1] * maxLimit,
                            pos1[2] + axis[2] * maxLimit,
                        ];
                        addConstraintMarker(positions, colors, maxPos, size * 0.08, color);
                    }

                    // Draw line showing the range
                    if (minLimit > -3.4e38 && maxLimit < 3.4e38) {
                        const minPos: [number, number, number] = [
                            pos1[0] + axis[0] * minLimit,
                            pos1[1] + axis[1] * minLimit,
                            pos1[2] + axis[2] * minLimit,
                        ];
                        const maxPos: [number, number, number] = [
                            pos1[0] + axis[0] * maxLimit,
                            pos1[1] + axis[1] * maxLimit,
                            pos1[2] + axis[2] * maxLimit,
                        ];
                        addConstraintLine(positions, colors, minPos, maxPos, color);
                    }
                }
            }

            // Draw rotation limits for X, Y, Z axes (as small arcs)
            for (let i = 0; i < 3; i++) {
                const minLimit = constraint.limitMin[3 + i];
                const maxLimit = constraint.limitMax[3 + i];
                const axis = axes[i];
                const color = axisColors[i];

                // Check if this rotation axis has limits
                const isFree = minLimit <= -3.4e38 && maxLimit >= 3.4e38;
                const isFixed = minLimit >= 3.4e38 && maxLimit <= -3.4e38;

                if (!isFree && !isFixed && minLimit < maxLimit && Math.abs(maxLimit - minLimit) > 0.001) {
                    // Get a perpendicular axis for drawing the arc
                    const perpAxis = axes[(i + 1) % 3];

                    // Draw a small pie showing the rotation limits
                    drawPie(positions, colors, pos1, size * 0.5, axis, perpAxis, minLimit, maxLimit, color);
                }
            }
        }
    }
    }

    // Update geometry
    const geometry = state.constraints.lineSegments.geometry;
    if (positions.length > 0) {
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        geometry.computeBoundingSphere();
    } else {
        // Clear geometry if no constraints
        geometry.deleteAttribute('position');
        geometry.deleteAttribute('color');
    }
}

// Clears all debug renderer state and removes all debug visuals
export function clear(state: State): void {
    // Remove all batched mesh body instances
    clearBatchedMeshBodies(state);

    // Remove all edge wireframes
    clearEdgeWireframes(state);

    // Clear contact visualization (just set counts to 0 - no add/delete needed with InstancedMesh)
    state.contacts.spheresMesh.count = 0;
    state.contacts.cylindersMesh.count = 0;
    state.contacts.conesMesh.count = 0;

    // Clear instance colors but keep geometry cache
    // Geometries are reusable unit shapes and should persist across scene changes
    state.bodies.instanceColors.clear();

    // Note: We don't clear bodies.geometryCache
    // because these geometries are reusable and should persist across scenes
    // to avoid hitting BatchedMesh capacity limits

    // Hide broadphase DBVT lines
    state.broadphase.dbvtLines.visible = false;
    if (state.broadphase.dbvtLines.geometry) {
        state.broadphase.dbvtLines.geometry.dispose();
        state.broadphase.dbvtLines.geometry = new THREE.BufferGeometry();
    }

    // Clear triangle mesh BVH visualization
    for (const [_bodyId, lineSegments] of state.triangleMeshBvh.cache) {
        state.triangleMeshBvh.container.remove(lineSegments);
        lineSegments.geometry.dispose();
        if (lineSegments.material instanceof THREE.Material) {
            lineSegments.material.dispose();
        }
    }
    state.triangleMeshBvh.cache.clear();
    state.triangleMeshBvh.container.visible = false;
}
