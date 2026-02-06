import { type Vec3, vec3 } from 'mathcat';
import { computeBarycentricCoordinates3d, createBarycentricCoordinatesResult } from './closest-points';

/**
 * Active edges module - handles detection and correction of collision normals at triangle mesh edges.
 *
 * An active edge is an edge that either has no neighbouring edge or if the angle between
 * the two connecting faces is too large. Active edges should generate collisions, while
 * inactive edges (coplanar or nearly-coplanar interior edges) should not cause "ghost collisions".
 */

const COS_179_DEGREES = -0.999848; // cos(179°) - back-to-back triangle threshold
const COS_1_DEGREE = 0.999848; // cos(1°) - parallel normal threshold
const BARYCENTRIC_EPSILON = 1.0e-4;
const BARYCENTRIC_ONE_MINUS_EPSILON = 1.0 - BARYCENTRIC_EPSILON;

const _barycentricCoords = createBarycentricCoordinatesResult();
const _v0Shifted = vec3.create();
const _v1Shifted = vec3.create();
const _v2Shifted = vec3.create();
const _cross = vec3.create();

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
export function isEdgeActive(normal1: Vec3, normal2: Vec3, edgeDirection: Vec3, cosThresholdAngle: number): boolean {
    // If normals are opposite the edges are active (the triangles are back to back)
    const cosAngleNormals = vec3.dot(normal1, normal2);
    if (cosAngleNormals < COS_179_DEGREES) {
        return true;
    }

    // Check if concave edge, if so we are not active
    vec3.cross(_cross, normal1, normal2);
    if (vec3.dot(_cross, edgeDirection) < 0.0) {
        return false;
    }

    // Convex edge, active when angle bigger than threshold
    return cosAngleNormals < cosThresholdAngle;
}

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
export function fixNormal(
    v0: Vec3,
    v1: Vec3,
    v2: Vec3,
    triangleNormal: Vec3,
    activeEdges: number,
    point: Vec3,
    normal: Vec3,
    movementDirection: Vec3,
): Vec3 {
    // Check: All of the edges are active, we have the correct normal already. No need to call this function!
    // TODO: assert?
    if (activeEdges === 0b111) {
        return normal;
    }

    // If inNormal would affect movement less than inTriangleNormal use inNormal
    // This is done since it is really hard to make a distinction between sliding over a horizontal
    // triangulated grid and hitting an edge (in this case you want to use the triangle normal)
    // and sliding over a triangulated grid and grazing a vertical triangle with an inactive edge
    // (in this case using the triangle normal will cause the object to bounce back so we want to use
    // the calculated normal).
    // To solve this we take a movement hint to give an indication of what direction our object is moving.
    // If the edge normal results in less motion difference than the triangle normal we use the edge normal.
    const normalLength = vec3.length(normal);
    const triangleNormalLength = vec3.length(triangleNormal);
    if (vec3.dot(movementDirection, normal) * triangleNormalLength < vec3.dot(movementDirection, triangleNormal) * normalLength) {
        return normal;
    }

    // Check: None of the edges are active, we need to use the triangle normal
    if (activeEdges === 0) {
        return triangleNormal;
    }

    // Some edges are active.
    // If normal is parallel to the triangle normal we don't need to check the active edges.
    if (vec3.dot(triangleNormal, normal) > COS_1_DEGREE * normalLength * triangleNormalLength) {
        return normal;
    }

    // Test where the contact point is in the triangle
    // Shift vertices to be relative to collision point
    vec3.subtract(_v0Shifted, v0, point);
    vec3.subtract(_v1Shifted, v1, point);
    vec3.subtract(_v2Shifted, v2, point);

    computeBarycentricCoordinates3d(_barycentricCoords, _v0Shifted, _v1Shifted, _v2Shifted, 1e-10);

    const { u, v, w } = _barycentricCoords;

    // Determine which edge(s) the collision point is near
    let collidingEdge: number;

    if (u > BARYCENTRIC_ONE_MINUS_EPSILON) {
        // Colliding with v0, edge 0 or 2 needs to be active
        collidingEdge = 0b101;
    } else if (v > BARYCENTRIC_ONE_MINUS_EPSILON) {
        // Colliding with v1, edge 0 or 1 needs to be active
        collidingEdge = 0b011;
    } else if (w > BARYCENTRIC_ONE_MINUS_EPSILON) {
        // Colliding with v2, edge 1 or 2 needs to be active
        collidingEdge = 0b110;
    } else if (u < BARYCENTRIC_EPSILON) {
        // Colliding with edge v1, v2, edge 1 needs to be active
        collidingEdge = 0b010;
    } else if (v < BARYCENTRIC_EPSILON) {
        // Colliding with edge v0, v2, edge 2 needs to be active
        collidingEdge = 0b100;
    } else if (w < BARYCENTRIC_EPSILON) {
        // Colliding with edge v0, v1, edge 0 needs to be active
        collidingEdge = 0b001;
    } else {
        // Interior hit
        return triangleNormal;
    }

    // If this edge is active, use the provided normal instead of the triangle normal
    return (activeEdges & collidingEdge) !== 0 ? normal : triangleNormal;
}
