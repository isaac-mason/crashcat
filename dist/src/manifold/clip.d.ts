import { type Vec3 } from 'mathcat';
import { type Face } from '../utils/face.js';
/**
 * Clips a polygon against a half-space plane using Sutherland-Hodgeman algorithm.
 * Keeps vertices in the negative half-space: (origin - vertex) · normal < 0
 *
 * @param out output face with clipped vertices
 * @param inPolygon input polygon to clip
 * @param inPlaneOrigin point on the clipping plane
 * @param inPlaneNormal plane normal (does NOT need to be normalized)
 */
export declare function clipPolyVsPlane(out: Face, inPolygon: Face, inPlaneOrigin: Vec3, inPlaneNormal: Vec3): void;
/**
 * Clips one polygon against another using Sutherland-Hodgeman algorithm.
 * Both polygons assumed counter-clockwise order.
 *
 * @param out output face with clipped vertices
 * @param inPolygonToClip polygon to clip
 * @param inClippingPolygon polygon to clip against (must have >= 3 vertices)
 * @param inClippingPolygonNormal normal of clipping polygon (does NOT need to be normalized)
 */
export declare function clipPolyVsPoly(out: Face, inPolygonToClip: Face, inClippingPolygon: Face, inClippingPolygonNormal: Vec3): void;
/**
 * Clips a polygon against an edge. The edge is projected onto the polygon's plane,
 * then the polygon is clipped against the edge's half-space.
 *
 * IMPORTANT: This function only outputs intersection points where the polygon
 * boundary crosses the edge projection, NOT the interior vertices.
 *
 * @param out output face with clipped vertices
 * @param inPolygon polygon to clip (must have >= 3 vertices)
 * @param inEdgeVertex1 first vertex of edge
 * @param inEdgeVertex2 second vertex of edge
 * @param inClippingEdgeNormal normal used to orient clipping plane
 */
export declare function clipPolyVsEdge(out: Face, inPolygon: Face, inEdgeVertex1: Vec3, inEdgeVertex2: Vec3, inClippingEdgeNormal: Vec3): void;
