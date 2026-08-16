import { type Vec3 } from 'math';
/**
 * Determine if an edge between two triangles should be treated as collidable.
 *
 * An active edge is one that either:
 * - Has no neighboring triangle (mesh boundary)
 * - Forms a sharp convex crease (angle > threshold)
 *
 * Inactive edges occur at:
 * - Nearly coplanar triangle junctions (smooth surfaces)
 * - Concave creases (always inactive to prevent snagging)
 *
 * @param normal1 normal of left triangle (looking along edge from above)
 * @param normal2 normal of right triangle
 * @param edgeDirection vector pointing along the edge
 * @param cosThresholdAngle cosine of threshold angle (typically cos(10°) ≈ 0.9848)
 * @returns true if edge should generate collisions, false if inactive
 */
export declare function isEdgeActive(normal1: Vec3, normal2: Vec3, edgeDirection: Vec3, cosThresholdAngle: number): boolean;
/**
 * Replace collision normal with triangle normal when hitting an inactive edge.
 *
 * Prevents "ghost collisions" at interior triangle mesh edges by detecting
 * when a collision point lies on or near an inactive edge, and returning
 * the triangle normal instead of the calculated collision normal.
 *
 * @param v0 first triangle vertex (local space)
 * @param v1 second triangle vertex (local space)
 * @param v2 third triangle vertex (local space)
 * @param triangleNormal triangle normal (unnormalized OK)
 * @param activeEdges 3-bit mask: bit 0 = edge v0-v1, bit 1 = edge v1-v2, bit 2 = edge v2-v0
 * @param point collision point on triangle (local space)
 * @param normal collision normal from GJK/EPA (unnormalized OK)
 * @param movementDirection velocity hint for resolving ambiguous cases (can be zero)
 * @returns corrected collision normal (returns input normal or triangle normal)
 */
export declare function fixNormal(v0: Vec3, v1: Vec3, v2: Vec3, triangleNormal: Vec3, activeEdges: number, point: Vec3, normal: Vec3, movementDirection: Vec3): Vec3;
