import { type Box3, type Mat4, type Vec3 } from 'mathcat';
import { ShapeType } from './shapes.js';
/** settings for creating a convex hull shape */
export type ConvexHullShapeSettings = {
    /** flat array of input point positions [x1, y1, z1, x2, y2, z2, ...] (can include interior points) */
    positions: number[];
    /** @default 0.05 @see DEFAULT_CONVEX_RADIUS */
    convexRadius?: number;
    /** points are allowed this far outside of the hull, increase to get a hull with less vertices, note that the actual used value can be larger if the points of the hull are far apart. @default 1e-3 */
    hullTolerance?: number;
    /** maximum allowed error when shrinking hull by convex radius. Used to validate that vertices don't shift too far at sharp edges. @default 0.05 */
    maxErrorConvexRadius?: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
    /** bake the vertex 1-ring adjacency used to accelerate support queries by hill climbing. undefined = auto (bake when numPoints > SUPPORT_HILL_CLIMB_MIN_POINTS), false = never, true = always */
    bakeSupportAdjacency?: boolean;
};
/** a convex hull shape */
export type ConvexHullShape = {
    type: ShapeType.CONVEX_HULL;
    /** flat vertex positions `[x, y, z, ...]`, 3 per point */
    pointPositions: number[];
    /** convex-radius-shrunk vertex positions `[x, y, z, ...]` (derived, immutable); the same reference as `pointPositions` when `convexRadius` is 0. computed once at create so the exclude-mode support fill can borrow it per pair instead of rebuilding it every frame */
    shrunkPointPositions: number[];
    /** number of neighbouring faces per point (1..3), 1 per point (used by the convex-radius shrink) */
    pointNumFaces: number[];
    /** up to 3 neighbouring face indices per point `[f0, f1, f2, ...]` (-1 = unused), 3 per point */
    pointFaces: number[];
    /** number of hull vertices (`pointPositions.length / 3`) */
    numPoints: number;
    /** faces of the convex hull */
    faces: ConvexHullFace[];
    /** plane equations for each face (1-to-1 with faces) */
    planes: ConvexHullPlane[];
    /** flattened vertex indices for all faces */
    vertexIndices: number[];
    /**
     * CSR vertex 1-ring adjacency for support hill climbing; empty ⇔ not baked (runtime brute-scan dispatch).
     * prefix offsets into `pointNeighbors`, length numPoints+1: vertex `p`'s neighbours are
     * `pointNeighbors[pointNeighborsStart[p] .. pointNeighborsStart[p+1])`.
     */
    pointNeighborsStart: number[];
    /** flat neighbour point indices, indexed via `pointNeighborsStart`; empty ⇔ not baked */
    pointNeighbors: number[];
    /** convex radius */
    convexRadius: number;
    /** shape density */
    density: number;
    /** material identifier */
    materialId: number;
    /** local bounds */
    aabb: Box3;
    /** center of mass */
    centerOfMass: Vec3;
    /** volume */
    volume: number;
    /** inertia tensor (column-major mat4) */
    inertia: Mat4;
};
export type ConvexHullFace = {
    /** index of the first vertex in the face */
    firstVertex: number;
    /** number of vertices in this face */
    numVertices: number;
};
export type ConvexHullPlane = {
    /** plane normal pointing outwards from hull */
    normal: Vec3;
    /** plane constant */
    constant: number;
};
/** auto-bake the support hill-climb adjacency above this vertex count (below it, brute scan wins) */
export declare const SUPPORT_HILL_CLIMB_MIN_POINTS = 32;
/** create a convex hull shape */
export declare function create(o: ConvexHullShapeSettings): ConvexHullShape;
export declare const def: import("./shapes").ShapeDef<ConvexHullShape>;
