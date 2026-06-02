import { type Vec3 } from 'mathcat';
import type { Face } from '../utils/face.js';
/** Maximum number of contact points in a manifold after reduction */
export declare const MAX_CONTACT_POINTS = 4;
/** Maximum number of contact points during polygon clipping (before reduction) */
export declare const MAX_CLIPPING_CONTACT_POINTS = 64;
/**
 * Contact manifold between two shapes.
 *
 * Contact points are stored relative to baseOffset.
 *
 * Pre-allocates space for 64 contact points
 * During manifold generation:
 * - ManifoldBetweenTwoFaces fills with up to 64 points
 * - PruneContactPoints reduces to max 4 points in-place
 * - numContactPoints tracks the current count
 */
export type ContactManifold = {
    /** number of active contact points (0-64 during clipping, 0-4 after reduction) */
    numContactPoints: number;
    /** base offset for all contact points */
    baseOffset: Vec3;
    /** contact point positions on shape A relative to baseOffset (flat array: [x1,y1,z1, x2,y2,z2, ...]) */
    relativeContactPointsOnA: number[];
    /** contact point positions on shape B relative to baseOffset (flat array: [x1,y1,z1, x2,y2,z2, ...]) */
    relativeContactPointsOnB: number[];
    /** contact normal (world space, normalized, points from A to B) */
    worldSpaceNormal: Vec3;
    /** penetration depth for the entire manifold (single value, not per-point) */
    penetrationDepth: number;
    /** sub-shape id of shape A */
    subShapeIdA: number;
    /** sub-shape id of shape B */
    subShapeIdB: number;
    /** material id of sub-shape A */
    materialIdA: number;
    /** material id of sub-shape B */
    materialIdB: number;
};
/** creates a new contact manifold with pre-allocated storage for 64 contact points */
export declare function createContactManifold(): ContactManifold;
export declare function resetContactManifold(manifold: ContactManifold): void;
export declare function swapShapes(manifold: ContactManifold): void;
/** gets relative contact point on shape A (relative to baseOffset) */
export declare function getRelativeContactPointOnA(out: Vec3, manifold: ContactManifold, index: number): Vec3;
/** gets relative contact point on shape B (relative to baseOffset) */
export declare function getRelativeContactPointOnB(out: Vec3, manifold: ContactManifold, index: number): Vec3;
/** gets world-space contact point on shape A (baseOffset + relativePoint) */
export declare function getWorldSpaceContactPointOnA(out: Vec3, manifold: ContactManifold, index: number): Vec3;
/** gets world-space contact point on shape B (baseOffset + relativePoint) */
export declare function getWorldSpaceContactPointOnB(out: Vec3, manifold: ContactManifold, index: number): Vec3;
/**
 * Reduces contact points from up to 64 down to max 4.
 *
 * Algorithm:
 * 1. Project all contact points onto plane perpendicular to penetrationAxis
 * 2. Calculate metric for each point: (distance_in_plane²) × (penetration_depth²)
 *    where penetration_depth = distance between contact points on shape1 vs shape2
 * 3. Greedily select 4 points:
 *    - Point 1: Maximum metric (furthest from COM in plane + deepest penetration)
 *    - Point 2: Maximum distance from Point 1 (maximize edge length)
 *    - Point 3 & 4: On opposite sides of P1-P2 edge (maximize area)
 * 4. Output in winding order: [P1, P3, P2, P4] to form a proper quad
 *
 * In-place reduction: selected points are moved to indices 0-3,
 * manifold.numContactPoints is updated to final count (1-4).
 *
 * @param manifold contactManifold with contact points to prune (modifies in-place)
 * @param penetrationAxis normalized penetration direction (from shape 1 to shape 2)
 */
export declare function pruneContactPoints(manifold: ContactManifold, penetrationAxis: Vec3): void;
/**
 * Generates contact manifold from two supporting faces using polygon clipping.
 *
 * Algorithm:
 * 1. Clip face 2 (incident) against face 1 (reference)
 * 2. Project clipped points onto face 1's plane
 * 3. Filter by max contact distance
 * 4. Fallback to original contact points if clipping produces nothing
 *
 * Contact points are stored RELATIVE to manifold.baseOffset for numerical precision.
 *
 * @param out output contact manifold (points/numContactPoints populated)
 * @param inContactPoint1 initial contact point on shape 1 (world space)
 * @param inContactPoint2 initial contact point on shape 2 (world space)
 * @param inPenetrationAxis penetration direction (unnormalized, from shape 1 to 2, world space)
 * @param inMaxContactDistance max distance for contact filtering
 * @param inShape1Face supporting face on shape 1 (reference face, world space)
 * @param inShape2Face supporting face on shape 2 (incident face, world space)
 */
export declare function manifoldBetweenTwoFaces(out: ContactManifold, inContactPoint1: Vec3, inContactPoint2: Vec3, inPenetrationAxis: Vec3, inMaxContactDistance: number, inShape1Face: Face, inShape2Face: Face): void;
