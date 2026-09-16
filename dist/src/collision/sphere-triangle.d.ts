import type { Vec3 } from 'math';
/**
 * Test ray against infinite cylinder, returning smallest positive fraction in [0, 1].
 * Based on "Real Time Collision Detection" by Christer Ericson, Chapter 5.3.7.
 *
 * Note: The ray origin is assumed to be at the origin (0, 0, 0).
 * The cylinder is defined by two endpoints (cylinderA, cylinderB) and a radius.
 * This test only considers hits on the cylindrical surface, not the caps.
 *
 * @param direction ray direction (length defines max distance, ray starts at origin)
 * @param cylinderA first endpoint of cylinder axis
 * @param cylinderB second endpoint of cylinder axis
 * @param radius cylinder radius
 * @returns fraction [0, 1] if hit on cylinder surface, or Infinity if no hit
 */
export declare function rayCylinder(direction: Vec3, cylinderA: Vec3, cylinderB: Vec3, radius: number): number;
/**
 * Test ray against sphere, returning smallest positive fraction in [0, 1].
 * @param origin ray origin
 * @param direction ray direction (length defines max distance)
 * @param sphereCenter center of the sphere
 * @param sphereRadius radius of the sphere
 * @returns Fraction [0, 1] if hit, or Infinity if no hit. Returns 0 if ray starts inside sphere.
 */
export declare function raySphere(origin: Vec3, direction: Vec3, sphereCenter: Vec3, sphereRadius: number): number;
/**
 * Test ray (starting at origin) against sphere, returning smallest positive fraction in [0, 1].
 * Optimized version where ray origin is assumed to be at (0, 0, 0).
 * @param direction ray direction (length defines max distance)
 * @param sphereCenter center of the sphere
 * @param sphereRadius radius of the sphere
 * @returns Fraction [0, 1] if hit, or Infinity if no hit. Returns 0 if ray starts inside sphere.
 */
export declare function raySphereFromOrigin(direction: Vec3, sphereCenter: Vec3, sphereRadius: number): number;
/**
 * Maps triangle feature (from getClosestPointOnTriangle) to required active edges bitmask.
 *
 * Feature bits: 0b001 = vertex A, 0b010 = vertex B, 0b100 = vertex C
 * Edge bits: 0b001 = edge AB, 0b010 = edge BC, 0b100 = edge CA
 *
 * When hitting a vertex, either of the adjacent edges must be active.
 * When hitting an edge, that specific edge must be active.
 * When hitting the face interior (0b111), no edge check is needed.
 */
export declare const FEATURE_TO_ACTIVE_EDGES: number[];
