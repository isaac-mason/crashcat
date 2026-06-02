import { type Mat4, type Vec3 } from 'mathcat';
import { type Shape, type ShapeDef, type ShapeType } from '../shapes/shapes.js';
import type { Face } from '../utils/face.js';
export declare const DEFAULT_CONVEX_RADIUS = 0.05;
export type ShapeSupportPool = {
    shapes: Partial<Record<ShapeType, unknown>>;
    dispose: () => void;
};
export declare enum SupportFunctionMode {
    INCLUDE_CONVEX_RADIUS = 0,
    EXCLUDE_CONVEX_RADIUS = 1,
    DEFAULT = 2
}
export type Support = {
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
/** creates a new shape support pool */
export declare function createShapeSupportPool(): ShapeSupportPool;
export declare function allocateShapeSupportPools(def: ShapeDef<any>): void;
export declare function getShapeSupportFunction(pool: ShapeSupportPool, shape: Shape, mode: SupportFunctionMode, scale: Vec3): Support;
export type TriangleSupport = {
    a: Vec3;
    b: Vec3;
    c: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createTriangleSupport(): TriangleSupport;
export declare function setTriangleSupport(out: TriangleSupport, a: Vec3, b: Vec3, c: Vec3): void;
export type PointSupport = {
    point: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createPointSupport(): PointSupport;
export declare function setPointSupport(out: PointSupport, point: Vec3): void;
export type PolygonSupport = {
    vertices: number[];
    numVertices: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createPolygonSupport(): PolygonSupport;
export declare function setPolygonSupport(out: PolygonSupport, face: Face): void;
export type TransformedSupport = {
    convexRadius: number;
    support: Support | null;
    transform: Mat4;
    getSupport(direction: Vec3, out: Vec3): void;
    innerSupport: Support | null;
};
export declare function createTransformedSupport(): TransformedSupport;
export declare function setTransformedSupport(out: TransformedSupport, transform: Mat4, innerSupport: Support): void;
export type AddConvexRadiusSupport = {
    innerSupport: Support;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createAddConvexRadiusSupport(): AddConvexRadiusSupport;
export declare function setAddConvexRadiusSupport(out: AddConvexRadiusSupport, convexRadius: number, innerSupport: Support): void;
export type BoxSupport = {
    halfExtents: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};
export declare function createBoxSupport(): BoxSupport;
export declare function setBoxSupport(out: BoxSupport, halfExtents: Vec3, convexRadius: number, mode: SupportFunctionMode, scale: Vec3): void;
