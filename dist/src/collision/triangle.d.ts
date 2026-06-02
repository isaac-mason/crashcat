import type { Vec3 } from 'mathcat';
/**
 * Feature bits for getClosestPointOnTriangle result.
 * Indicates which part of the triangle contains the closest point.
 */
export declare enum TriangleFeature {
    /** closest to vertex A (v0) */
    VERTEX_A = 1,
    /** closest to vertex B (v1) */
    VERTEX_B = 2,
    /** closest to vertex C (v2) */
    VERTEX_C = 4,
    /** closest to edge AB (v0-v1) */
    EDGE_AB = 3,
    /** closest to edge AC (v0-v2) */
    EDGE_AC = 5,
    /** closest to edge BC (v1-v2) */
    EDGE_BC = 6,
    /** closest to interior face */
    FACE = 7
}
export type ClosestPointOnTriangleResult = {
    /** closest point on triangle (in same coordinate space as input vertices) */
    point: Vec3;
    /** squared distance from origin to closest point */
    distanceSq: number;
    /** feature bitmask indicating which part of triangle is closest (see TriangleFeature) */
    feature: number;
};
export declare function createClosestPointOnTriangleResult(): ClosestPointOnTriangleResult;
/**
 * Get the closest point on a triangle to the origin.
 * Based on JoltPhysics ClosestPoint::GetClosestPointOnTriangle and
 * "Real-Time Collision Detection" by Christer Ericson.
 *
 * @param out result object to store the closest point, squared distance, and feature
 * @param a first vertex of triangle (relative to query point, i.e., query point is at origin)
 * @param b second vertex of triangle (relative to query point)
 * @param c third vertex of triangle (relative to query point)
 */
export declare function getClosestPointOnTriangle(out: ClosestPointOnTriangleResult, a: Vec3, b: Vec3, c: Vec3): void;
