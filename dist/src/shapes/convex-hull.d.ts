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
};
/** a convex hull shape */
export type ConvexHullShape = {
    type: ShapeType.CONVEX_HULL;
    /**
     * flat vertex positions of the convex hull `[x, y, z, ...]`, 3 per point.
     * stored as a plain `number[]` (packed-double) rather than a typed array —
     * benchmarked fastest for the max-dot support scan on V8 (beats Float64Array).
     */
    pointPositions: number[];
    /** number of neighbouring faces per point (1..3), 1 per point — only used by the convex-radius shrink */
    pointNumFaces: number[];
    /** up to 3 neighbouring face indices per point `[f0, f1, f2, ...]` (-1 = unused), 3 per point — shrink only */
    pointFaces: number[];
    /** number of hull vertices (`pointPositions.length / 3`) */
    numPoints: number;
    /** faces of the convex hull */
    faces: ConvexHullFace[];
    /** plane equations for each face (1-to-1 with faces) */
    planes: ConvexHullPlane[];
    /** flattened vertex indices for all faces */
    vertexIndices: number[];
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
/** create a convex hull shape */
export declare function create(o: ConvexHullShapeSettings): ConvexHullShape;
export declare const def: import("./shapes").ShapeDef<ConvexHullShape>;
/**
 * ConvexHull support for INCLUDE_CONVEX_RADIUS mode (unscaled).
 * Returns original hull geometry, convexRadius is 0.
 * Stores only reference to shape (cheap construction, O(n) GetSupport).
 */
export type ConvexHullWithConvexSupport = {
    shape: ConvexHullShape;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createConvexHullWithConvexSupport(): ConvexHullWithConvexSupport;
export declare function setConvexHullWithConvexSupport(out: ConvexHullWithConvexSupport, shape: ConvexHullShape): void;
/**
 * ConvexHull support for INCLUDE_CONVEX_RADIUS mode (scaled).
 * Returns scaled hull geometry, convexRadius is 0.
 * Scales vertices on-the-fly during GetSupport.
 */
export type ConvexHullWithConvexSupportScaled = {
    shape: ConvexHullShape;
    scale: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createConvexHullWithConvexSupportScaled(): ConvexHullWithConvexSupportScaled;
export declare function setConvexHullWithConvexSupportScaled(out: ConvexHullWithConvexSupportScaled, shape: ConvexHullShape, scale: Vec3): void;
/**
 * ConvexHull support for EXCLUDE_CONVEX_RADIUS mode (unscaled).
 * Pre-computes shrunk vertices using plane intersection.
 * More expensive construction, fast queries.
 */
export type ConvexHullNoConvexSupport = {
    points: number[];
    numPoints: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createConvexHullNoConvexSupport(): ConvexHullNoConvexSupport;
export declare function setConvexHullNoConvexSupport(out: ConvexHullNoConvexSupport, shape: ConvexHullShape): void;
/**
 * ConvexHull support for EXCLUDE_CONVEX_RADIUS mode (scaled).
 * Pre-computes shrunk + scaled vertices using plane intersection with inverse scale transform.
 * More expensive construction, fast queries.
 */
export type ConvexHullNoConvexSupportScaled = {
    points: number[];
    numPoints: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createConvexHullNoConvexSupportScaled(): ConvexHullNoConvexSupportScaled;
export declare function setConvexHullNoConvexSupportScaled(out: ConvexHullNoConvexSupportScaled, shape: ConvexHullShape, scale: Vec3): void;
