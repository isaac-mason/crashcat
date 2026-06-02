import type { Shape, World } from 'crashcat';
import * as THREE from 'three';
export declare enum BodyColorMode {
    INSTANCE = 0,
    MOTION_TYPE = 1,
    SLEEPING = 2,
    ISLAND = 3
}
type ShapeKey = 'unit-sphere' | 'unit-box' | 'unit-capsule' | 'unit-cone' | 'unit-cylinder' | 'unit-plane' | `triangle-mesh:${number}` | `convex-hull:${number}`;
type GeometryCache = Map<ShapeKey, {
    geometry: THREE.BufferGeometry;
    geometryId: number;
    refCount: number;
}>;
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
    staticCompoundBvh: {
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
    linearVelocityArrows: Map<number, {
        shaftId: number;
        tipId: number;
    }>;
    angularVelocityArrows: Map<number, {
        shaftId: number;
        tipId: number;
    }>;
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
type StaticCompoundBvhState = {
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
    staticCompoundBvh: StaticCompoundBvhState;
};
/** create default debug renderer options */
export declare function createDefaultOptions(): DebugRendererOptions;
/**
 * initialize the debug renderer
 * @param options - optional debug renderer options. if not provided, default options will be used.
 */
export declare function init(options?: DebugRendererOptions): State;
/**
 * Invalidate a body's shape to force recreation of its visual instances.
 * Call this when you mutate a body's shape properties in-place.
 * @param state Debug renderer state
 * @param bodyId The ID of the body whose shape was mutated
 */
export declare function invalidateBodyShape(state: State, bodyId: number): void;
export declare function update(state: State, world: World): void;
/** dispose all gpu resources and remove everything from the scene graph. the state should not be used after calling this. */
export declare function dispose(state: State): void;
export declare function clear(state: State): void;
export {};
