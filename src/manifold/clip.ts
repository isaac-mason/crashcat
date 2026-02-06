import { type Vec3, vec3 } from 'mathcat';
import { createFace, type Face } from '../utils/face';

const _clipPlane_diff = vec3.create();
const _clipPlane_e1 = vec3.create();
const _clipPlane_e2 = vec3.create();
const _clipPlane_e12 = vec3.create();
const _clipPlane_intersection = vec3.create();

/**
 * Clips a polygon against a half-space plane using Sutherland-Hodgeman algorithm.
 * Keeps vertices in the negative half-space: (origin - vertex) · normal < 0
 *
 * @param out - Output face with clipped vertices
 * @param inPolygon - Input polygon to clip
 * @param inPlaneOrigin - Point on the clipping plane
 * @param inPlaneNormal - Plane normal (does NOT need to be normalized)
 */
export function clipPolyVsPlane(out: Face, inPolygon: Face, inPlaneOrigin: Vec3, inPlaneNormal: Vec3): void {
    // reset output
    out.numVertices = 0;

    if (inPolygon.numVertices < 2) {
        return;
    }

    // get last vertex (wrapping loop)
    const e1Index = (inPolygon.numVertices - 1) * 3;
    _clipPlane_e1[0] = inPolygon.vertices[e1Index];
    _clipPlane_e1[1] = inPolygon.vertices[e1Index + 1];
    _clipPlane_e1[2] = inPolygon.vertices[e1Index + 2];

    // determine state of last point
    vec3.sub(_clipPlane_diff, inPlaneOrigin, _clipPlane_e1);
    let prevNum = vec3.dot(_clipPlane_diff, inPlaneNormal);
    let prevInside = prevNum < 0.0;

    // loop through all vertices
    for (let j = 0; j < inPolygon.numVertices; j++) {
        // check if second point is inside
        const e2Index = j * 3;
        _clipPlane_e2[0] = inPolygon.vertices[e2Index];
        _clipPlane_e2[1] = inPolygon.vertices[e2Index + 1];
        _clipPlane_e2[2] = inPolygon.vertices[e2Index + 2];
        vec3.sub(_clipPlane_diff, inPlaneOrigin, _clipPlane_e2);
        const num = vec3.dot(_clipPlane_diff, inPlaneNormal);
        let curInside = num < 0.0;

        // in -> out or out -> in: add point on clipping plane
        if (curInside !== prevInside) {
            // solve: (X - inPlaneOrigin) . inPlaneNormal = 0 and X = e1 + t * (e2 - e1) for X
            vec3.sub(_clipPlane_e12, _clipPlane_e2, _clipPlane_e1);
            const denom = vec3.dot(_clipPlane_e12, inPlaneNormal);

            if (denom !== 0.0) {
                // intersection: e1 + (prevNum / denom) * e12
                const t = prevNum / denom;
                vec3.scaleAndAdd(_clipPlane_intersection, _clipPlane_e1, _clipPlane_e12, t);
                const outIndex = out.numVertices * 3;
                out.vertices[outIndex] = _clipPlane_intersection[0];
                out.vertices[outIndex + 1] = _clipPlane_intersection[1];
                out.vertices[outIndex + 2] = _clipPlane_intersection[2];
                out.numVertices++;
            } else {
                // edge is parallel to plane, treat point as if it were on the same side as the last point
                curInside = prevInside;
            }
        }

        // point inside, add it
        if (curInside) {
            const outIndex = out.numVertices * 3;
            out.vertices[outIndex] = _clipPlane_e2[0];
            out.vertices[outIndex + 1] = _clipPlane_e2[1];
            out.vertices[outIndex + 2] = _clipPlane_e2[2];
            out.numVertices++;
        }

        // update previous state
        prevNum = num;
        prevInside = curInside;
        _clipPlane_e1[0] = _clipPlane_e2[0];
        _clipPlane_e1[1] = _clipPlane_e2[1];
        _clipPlane_e1[2] = _clipPlane_e2[2];
    }
}

const _clipPoly_tmpFace1 = createFace();
const _clipPoly_tmpFace2 = createFace();
const _clipPoly_clipE1 = vec3.create();
const _clipPoly_clipE2 = vec3.create();
const _clipPoly_edge = vec3.create();
const _clipPoly_clipNormal = vec3.create();

/**
 * Clips one polygon against another using Sutherland-Hodgeman algorithm.
 * Both polygons assumed counter-clockwise order.
 *
 * @param out output face with clipped vertices
 * @param inPolygonToClip polygon to clip
 * @param inClippingPolygon polygon to clip against (must have >= 3 vertices)
 * @param inClippingPolygonNormal normal of clipping polygon (does NOT need to be normalized)
 */
export function clipPolyVsPoly(out: Face, inPolygonToClip: Face, inClippingPolygon: Face, inClippingPolygonNormal: Vec3): void {
    // reset output
    out.numVertices = 0;

    if (inPolygonToClip.numVertices < 2) {
        return;
    }

    if (inClippingPolygon.numVertices < 3) {
        return;
    }

    let tmpIdx = 0;
    let srcFace: Face;
    let dstFace: Face;

    // loop through edges of clipping polygon
    for (let i = 0; i < inClippingPolygon.numVertices; i++) {
        // get edge to clip against
        const clipE1Index = i * 3;
        _clipPoly_clipE1[0] = inClippingPolygon.vertices[clipE1Index];
        _clipPoly_clipE1[1] = inClippingPolygon.vertices[clipE1Index + 1];
        _clipPoly_clipE1[2] = inClippingPolygon.vertices[clipE1Index + 2];
        const clipE2Index = ((i + 1) % inClippingPolygon.numVertices) * 3;
        _clipPoly_clipE2[0] = inClippingPolygon.vertices[clipE2Index];
        _clipPoly_clipE2[1] = inClippingPolygon.vertices[clipE2Index + 1];
        _clipPoly_clipE2[2] = inClippingPolygon.vertices[clipE2Index + 2];

        // edge normal pointing inward: normal × (e2 - e1)
        vec3.sub(_clipPoly_edge, _clipPoly_clipE2, _clipPoly_clipE1);
        vec3.cross(_clipPoly_clipNormal, inClippingPolygonNormal, _clipPoly_edge);

        // get source and target faces
        if (i === 0) {
            srcFace = inPolygonToClip;
        } else {
            srcFace = tmpIdx === 0 ? _clipPoly_tmpFace1 : _clipPoly_tmpFace2;
        }

        tmpIdx ^= 1; // toggle between 0 and 1

        if (i === inClippingPolygon.numVertices - 1) {
            dstFace = out;
        } else {
            dstFace = tmpIdx === 0 ? _clipPoly_tmpFace1 : _clipPoly_tmpFace2;
        }

        // clip against the edge
        clipPolyVsPlane(dstFace, srcFace, _clipPoly_clipE1, _clipPoly_clipNormal);

        // break out if no polygon left
        if (dstFace.numVertices < 3) {
            out.numVertices = 0;
            break;
        }
    }
}

const _clipEdge_edge = vec3.create();
const _clipEdge_edgeNormal = vec3.create();
const _clipEdge_p0 = vec3.create();
const _clipEdge_p1 = vec3.create();
const _clipEdge_p2 = vec3.create();
const _clipEdge_polygonNormal = vec3.create();
const _clipEdge_diff1 = vec3.create();
const _clipEdge_diff2 = vec3.create();
const _clipEdge_v1 = vec3.create();
const _clipEdge_v2 = vec3.create();
const _clipEdge_v12 = vec3.create();
const _clipEdge_e1 = vec3.create();
const _clipEdge_e2 = vec3.create();
const _clipEdge_diff = vec3.create();
const _clipEdge_e12 = vec3.create();
const _clipEdge_clippedPoint = vec3.create();
const _clipEdge_clippedDiff = vec3.create();
const _clipEdge_v0v1 = vec3.create();
const _clipEdge_v0v2 = vec3.create();

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
export function clipPolyVsEdge(
    out: Face,
    inPolygon: Face,
    inEdgeVertex1: Vec3,
    inEdgeVertex2: Vec3,
    inClippingEdgeNormal: Vec3,
): void {
    // reset output
    out.numVertices = 0;

    if (inPolygon.numVertices < 3) {
        return;
    }

    // get normal that is perpendicular to the edge and the clipping edge normal
    vec3.sub(_clipEdge_edge, inEdgeVertex2, inEdgeVertex1);
    vec3.cross(_clipEdge_edgeNormal, inClippingEdgeNormal, _clipEdge_edge);

    // project vertices of edge on inPolygon
    // compute polygon normal from first 3 vertices: (p2 - p0) × (p1 - p0)
    _clipEdge_p0[0] = inPolygon.vertices[0];
    _clipEdge_p0[1] = inPolygon.vertices[1];
    _clipEdge_p0[2] = inPolygon.vertices[2];
    _clipEdge_p1[0] = inPolygon.vertices[3];
    _clipEdge_p1[1] = inPolygon.vertices[4];
    _clipEdge_p1[2] = inPolygon.vertices[5];
    _clipEdge_p2[0] = inPolygon.vertices[6];
    _clipEdge_p2[1] = inPolygon.vertices[7];
    _clipEdge_p2[2] = inPolygon.vertices[8];

    vec3.sub(_clipEdge_v0v1, _clipEdge_p1, _clipEdge_p0);
    vec3.sub(_clipEdge_v0v2, _clipEdge_p2, _clipEdge_p0);
    vec3.cross(_clipEdge_polygonNormal, _clipEdge_v0v2, _clipEdge_v0v1);
    const polygonNormalLenSq = vec3.squaredLength(_clipEdge_polygonNormal);

    // project edge vertex 1 onto polygon plane:
    // v1 = inEdgeVertex1 + [(p0 - inEdgeVertex1) · polyNormal] * polyNormal / |polyNormal|²
    vec3.sub(_clipEdge_diff1, _clipEdge_p0, inEdgeVertex1);
    const proj1 = vec3.dot(_clipEdge_diff1, _clipEdge_polygonNormal);
    vec3.scaleAndAdd(_clipEdge_v1, inEdgeVertex1, _clipEdge_polygonNormal, proj1 / polygonNormalLenSq);

    // project edge vertex 2 onto polygon plane
    vec3.sub(_clipEdge_diff2, _clipEdge_p0, inEdgeVertex2);
    const proj2 = vec3.dot(_clipEdge_diff2, _clipEdge_polygonNormal);
    vec3.scaleAndAdd(_clipEdge_v2, inEdgeVertex2, _clipEdge_polygonNormal, proj2 / polygonNormalLenSq);

    vec3.sub(_clipEdge_v12, _clipEdge_v2, _clipEdge_v1);
    const v12LenSq = vec3.squaredLength(_clipEdge_v12);

    // determine state of last point
    const e1Index = (inPolygon.numVertices - 1) * 3;
    _clipEdge_e1[0] = inPolygon.vertices[e1Index];
    _clipEdge_e1[1] = inPolygon.vertices[e1Index + 1];
    _clipEdge_e1[2] = inPolygon.vertices[e1Index + 2];
    vec3.sub(_clipEdge_diff, inEdgeVertex1, _clipEdge_e1);
    let prevNum = vec3.dot(_clipEdge_diff, _clipEdge_edgeNormal);
    let prevInside = prevNum < 0.0;

    // loop through all vertices
    for (let j = 0; j < inPolygon.numVertices; j++) {
        // check if second point is inside
        const e2Index = j * 3;
        _clipEdge_e2[0] = inPolygon.vertices[e2Index];
        _clipEdge_e2[1] = inPolygon.vertices[e2Index + 1];
        _clipEdge_e2[2] = inPolygon.vertices[e2Index + 2];
        vec3.sub(_clipEdge_diff, inEdgeVertex1, _clipEdge_e2);
        const num = vec3.dot(_clipEdge_diff, _clipEdge_edgeNormal);
        const curInside = num < 0.0;

        // in -> out or out -> in: add point on clipping plane
        if (curInside !== prevInside) {
            // solve: (inEdgeVertex1 - X) . edge_normal = 0 and X = e1 + t * (e2 - e1) for X
            vec3.sub(_clipEdge_e12, _clipEdge_e2, _clipEdge_e1);
            const denom = vec3.dot(_clipEdge_e12, _clipEdge_edgeNormal);

            if (denom !== 0.0) {
                const t = prevNum / denom;
                vec3.scaleAndAdd(_clipEdge_clippedPoint, _clipEdge_e1, _clipEdge_e12, t);
            } else {
                vec3.copy(_clipEdge_clippedPoint, _clipEdge_e1);
            }

            // project point on line segment v1, v2 to see if it falls outside of the edge
            vec3.sub(_clipEdge_clippedDiff, _clipEdge_clippedPoint, _clipEdge_v1);
            const projection = vec3.dot(_clipEdge_clippedDiff, _clipEdge_v12);

            const outIndex = out.numVertices * 3;
            if (projection < 0.0) {
                out.vertices[outIndex] = _clipEdge_v1[0];
                out.vertices[outIndex + 1] = _clipEdge_v1[1];
                out.vertices[outIndex + 2] = _clipEdge_v1[2];
            } else if (projection > v12LenSq) {
                out.vertices[outIndex] = _clipEdge_v2[0];
                out.vertices[outIndex + 1] = _clipEdge_v2[1];
                out.vertices[outIndex + 2] = _clipEdge_v2[2];
            } else {
                out.vertices[outIndex] = _clipEdge_clippedPoint[0];
                out.vertices[outIndex + 1] = _clipEdge_clippedPoint[1];
                out.vertices[outIndex + 2] = _clipEdge_clippedPoint[2];
            }
            out.numVertices++;
        }

        // IMPORTANT: Unlike ClipPolyVsPlane, we do NOT add interior vertices
        // This function only outputs intersection points

        // update previous state
        prevNum = num;
        prevInside = curInside;
        vec3.copy(_clipEdge_e1, _clipEdge_e2);
    }
}
