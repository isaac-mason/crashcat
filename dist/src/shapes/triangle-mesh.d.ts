import { type Vec3 } from 'math';
import { type Box3 } from 'math/shapes';
import { ShapeType } from './shapes.js';
import type { BvhSplitStrategy, TriangleMeshBVH } from './utils/triangle-mesh-bvh.js';
import type { TriangleMeshData } from './utils/triangle-mesh-data.js';
export { BvhSplitStrategy } from './utils/triangle-mesh-bvh.js';
export type TriangleMeshShapeSettings = {
    /** flat array of vertex positions [x1, y1, z1, x2, y2, z2, ...] */
    positions: number[];
    /** flat array of triangle vertex indices [i1, i2, i3, i4, i5, i6, ...] */
    indices: number[];
    /**
     * Optional per-triangle material indices.
     * Length should match number of triangles (indices.length / 3).
     * Default: -1 for all triangles (no material).
     */
    materialIndices?: number[];
    /** bvh split strategy */
    bvhSplitStrategy?: BvhSplitStrategy;
    /** maximum triangles per leaf node in the bvh */
    bvhMaxLeafTris?: number;
    /** degenerate tolerance for triangles */
    degenerateTolerance?: number;
    /**
     * cosine threshold for active edge determination.
     * edges with cos(dihedral_angle) >= this value are considered smooth/inactive.
     * default: cos(5°) = 0.996195
     * set to -1.0 to disable active edge determination (all edges active)
     * set to 1.0 to make all edges inactive
     */
    activeEdgeCosThresholdAngle?: number;
};
export declare const DEFAULT_TRIANGLE_MESH_OPTIONS: {
    bvhSplitStrategy: BvhSplitStrategy;
    bvhMaxLeafTris: number;
    degenerateTolerance: number;
    activeEdgeCosThresholdAngle: number;
};
export type TriangleMeshShape = {
    type: ShapeType.TRIANGLE_MESH;
    bvh: TriangleMeshBVH;
    data: TriangleMeshData;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
export declare function create(o: TriangleMeshShapeSettings): TriangleMeshShape;
export declare const def: import("./shapes").ShapeDef<TriangleMeshShape>;
