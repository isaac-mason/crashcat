import { type Vec3, vec3 } from 'mathcat';
import { createFace, type Face } from '../utils/face';

/**
 * Clips a polygon against a half-space plane using Sutherland-Hodgeman algorithm.
 * Keeps vertices in the negative half-space: (origin - vertex) · normal < 0
 *
 * @param out output face with clipped vertices
 * @param inPolygon input polygon to clip
 * @param inPlaneOrigin point on the clipping plane
 * @param inPlaneNormal plane normal (does NOT need to be normalized)
 */
export function clipPolyVsPlane(out: Face, inPolygon: Face, inPlaneOrigin: Vec3, inPlaneNormal: Vec3): void {
    // reset output
    out.numVertices = 0;

    if (inPolygon.numVertices < 2) {
        return;
    }

    // get last vertex (wrapping loop)
    const e1Index = (inPolygon.numVertices - 1) * 3;
    let e1x = inPolygon.vertices[e1Index];
    let e1y = inPolygon.vertices[e1Index + 1];
    let e1z = inPolygon.vertices[e1Index + 2];

    // determine state of last point
    // sub: inPlaneOrigin - e1
    const dx = inPlaneOrigin[0] - e1x;
    const dy = inPlaneOrigin[1] - e1y;
    const dz = inPlaneOrigin[2] - e1z;
    // dot: diff · inPlaneNormal
    let prevNum = dx * inPlaneNormal[0] + dy * inPlaneNormal[1] + dz * inPlaneNormal[2];
    let prevInside = prevNum < 0.0;

    // loop through all vertices
    for (let j = 0; j < inPolygon.numVertices; j++) {
        // check if second point is inside
        const e2Index = j * 3;
        const e2x = inPolygon.vertices[e2Index];
        const e2y = inPolygon.vertices[e2Index + 1];
        const e2z = inPolygon.vertices[e2Index + 2];
        // sub: inPlaneOrigin - e2
        const dx2 = inPlaneOrigin[0] - e2x;
        const dy2 = inPlaneOrigin[1] - e2y;
        const dz2 = inPlaneOrigin[2] - e2z;
        // dot: diff · inPlaneNormal
        const num = dx2 * inPlaneNormal[0] + dy2 * inPlaneNormal[1] + dz2 * inPlaneNormal[2];
        let curInside = num < 0.0;

        // in -> out or out -> in: add point on clipping plane
        if (curInside !== prevInside) {
            // solve: (X - inPlaneOrigin) . inPlaneNormal = 0 and X = e1 + t * (e2 - e1) for X
            // sub: e2 - e1
            const e12x = e2x - e1x;
            const e12y = e2y - e1y;
            const e12z = e2z - e1z;
            // dot: e12 · inPlaneNormal
            const denom = e12x * inPlaneNormal[0] + e12y * inPlaneNormal[1] + e12z * inPlaneNormal[2];

            if (denom !== 0.0) {
                // intersection: e1 + (prevNum / denom) * e12
                const t = prevNum / denom;
                const outIndex = out.numVertices * 3;
                // scaleAndAdd: e1 + e12 * t
                out.vertices[outIndex] = e1x + e12x * t;
                out.vertices[outIndex + 1] = e1y + e12y * t;
                out.vertices[outIndex + 2] = e1z + e12z * t;
                out.numVertices++;
            } else {
                // edge is parallel to plane, treat point as if it were on the same side as the last point
                curInside = prevInside;
            }
        }

        // point inside, add it
        if (curInside) {
            const outIndex = out.numVertices * 3;
            out.vertices[outIndex] = e2x;
            out.vertices[outIndex + 1] = e2y;
            out.vertices[outIndex + 2] = e2z;
            out.numVertices++;
        }

        // update previous state
        prevNum = num;
        prevInside = curInside;
        e1x = e2x;
        e1y = e2y;
        e1z = e2z;
    }
}

const _clipPoly_tmpFace1 = /* @__PURE__ */ createFace();
const _clipPoly_tmpFace2 = /* @__PURE__ */ createFace();
const _clipPoly_clipE1 = /* @__PURE__ */ vec3.create();
const _clipPoly_clipNormal = /* @__PURE__ */ vec3.create();

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
        const clipE1x = inClippingPolygon.vertices[clipE1Index];
        const clipE1y = inClippingPolygon.vertices[clipE1Index + 1];
        const clipE1z = inClippingPolygon.vertices[clipE1Index + 2];
        const clipE2Index = ((i + 1) % inClippingPolygon.numVertices) * 3;
        const clipE2x = inClippingPolygon.vertices[clipE2Index];
        const clipE2y = inClippingPolygon.vertices[clipE2Index + 1];
        const clipE2z = inClippingPolygon.vertices[clipE2Index + 2];

        // sub: clipE2 - clipE1
        const edgeX = clipE2x - clipE1x;
        const edgeY = clipE2y - clipE1y;
        const edgeZ = clipE2z - clipE1z;
        // cross: inClippingPolygonNormal × edge
        const clipNormalX = inClippingPolygonNormal[1] * edgeZ - inClippingPolygonNormal[2] * edgeY;
        const clipNormalY = inClippingPolygonNormal[2] * edgeX - inClippingPolygonNormal[0] * edgeZ;
        const clipNormalZ = inClippingPolygonNormal[0] * edgeY - inClippingPolygonNormal[1] * edgeX;

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
        _clipPoly_clipE1[0] = clipE1x;
        _clipPoly_clipE1[1] = clipE1y;
        _clipPoly_clipE1[2] = clipE1z;
        _clipPoly_clipNormal[0] = clipNormalX;
        _clipPoly_clipNormal[1] = clipNormalY;
        _clipPoly_clipNormal[2] = clipNormalZ;
        clipPolyVsPlane(dstFace, srcFace, _clipPoly_clipE1, _clipPoly_clipNormal);

        // break out if no polygon left
        if (dstFace.numVertices < 3) {
            out.numVertices = 0;
            break;
        }
    }
}

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

    // sub: inEdgeVertex2 - inEdgeVertex1
    const edgeX = inEdgeVertex2[0] - inEdgeVertex1[0];
    const edgeY = inEdgeVertex2[1] - inEdgeVertex1[1];
    const edgeZ = inEdgeVertex2[2] - inEdgeVertex1[2];
    // cross: inClippingEdgeNormal × edge
    const edgeNormalX = inClippingEdgeNormal[1] * edgeZ - inClippingEdgeNormal[2] * edgeY;
    const edgeNormalY = inClippingEdgeNormal[2] * edgeX - inClippingEdgeNormal[0] * edgeZ;
    const edgeNormalZ = inClippingEdgeNormal[0] * edgeY - inClippingEdgeNormal[1] * edgeX;

    // project vertices of edge on inPolygon
    // compute polygon normal from first 3 vertices: (p2 - p0) × (p1 - p0)
    const p0x = inPolygon.vertices[0];
    const p0y = inPolygon.vertices[1];
    const p0z = inPolygon.vertices[2];
    const p1x = inPolygon.vertices[3];
    const p1y = inPolygon.vertices[4];
    const p1z = inPolygon.vertices[5];
    const p2x = inPolygon.vertices[6];
    const p2y = inPolygon.vertices[7];
    const p2z = inPolygon.vertices[8];

    // sub: p1 - p0
    const v0v1x = p1x - p0x;
    const v0v1y = p1y - p0y;
    const v0v1z = p1z - p0z;
    // sub: p2 - p0
    const v0v2x = p2x - p0x;
    const v0v2y = p2y - p0y;
    const v0v2z = p2z - p0z;
    // cross: v0v2 × v0v1
    const polyNormalX = v0v2y * v0v1z - v0v2z * v0v1y;
    const polyNormalY = v0v2z * v0v1x - v0v2x * v0v1z;
    const polyNormalZ = v0v2x * v0v1y - v0v2y * v0v1x;
    // squaredLength: polyNormal · polyNormal
    const polygonNormalLenSq = polyNormalX * polyNormalX + polyNormalY * polyNormalY + polyNormalZ * polyNormalZ;

    // project edge vertex 1 onto polygon plane:
    // v1 = inEdgeVertex1 + [(p0 - inEdgeVertex1) · polyNormal] * polyNormal / |polyNormal|²
    // sub: p0 - inEdgeVertex1
    const diff1x = p0x - inEdgeVertex1[0];
    const diff1y = p0y - inEdgeVertex1[1];
    const diff1z = p0z - inEdgeVertex1[2];
    // dot: diff1 · polyNormal
    const proj1 = diff1x * polyNormalX + diff1y * polyNormalY + diff1z * polyNormalZ;
    const scale1 = proj1 / polygonNormalLenSq;
    // scaleAndAdd: inEdgeVertex1 + polyNormal * scale1
    const v1x = inEdgeVertex1[0] + polyNormalX * scale1;
    const v1y = inEdgeVertex1[1] + polyNormalY * scale1;
    const v1z = inEdgeVertex1[2] + polyNormalZ * scale1;

    // project edge vertex 2 onto polygon plane
    // sub: p0 - inEdgeVertex2
    const diff2x = p0x - inEdgeVertex2[0];
    const diff2y = p0y - inEdgeVertex2[1];
    const diff2z = p0z - inEdgeVertex2[2];
    // dot: diff2 · polyNormal
    const proj2 = diff2x * polyNormalX + diff2y * polyNormalY + diff2z * polyNormalZ;
    const scale2 = proj2 / polygonNormalLenSq;
    // scaleAndAdd: inEdgeVertex2 + polyNormal * scale2
    const v2x = inEdgeVertex2[0] + polyNormalX * scale2;
    const v2y = inEdgeVertex2[1] + polyNormalY * scale2;
    const v2z = inEdgeVertex2[2] + polyNormalZ * scale2;

    // sub: v2 - v1
    const v12x = v2x - v1x;
    const v12y = v2y - v1y;
    const v12z = v2z - v1z;
    // squaredLength: v12 · v12
    const v12LenSq = v12x * v12x + v12y * v12y + v12z * v12z;

    // determine state of last point
    const e1Index = (inPolygon.numVertices - 1) * 3;
    let e1x = inPolygon.vertices[e1Index];
    let e1y = inPolygon.vertices[e1Index + 1];
    let e1z = inPolygon.vertices[e1Index + 2];
    // sub: inEdgeVertex1 - e1
    const diffx = inEdgeVertex1[0] - e1x;
    const diffy = inEdgeVertex1[1] - e1y;
    const diffz = inEdgeVertex1[2] - e1z;
    // dot: diff · edgeNormal
    let prevNum = diffx * edgeNormalX + diffy * edgeNormalY + diffz * edgeNormalZ;
    let prevInside = prevNum < 0.0;

    // loop through all vertices
    for (let j = 0; j < inPolygon.numVertices; j++) {
        // check if second point is inside
        const e2Index = j * 3;
        const e2x = inPolygon.vertices[e2Index];
        const e2y = inPolygon.vertices[e2Index + 1];
        const e2z = inPolygon.vertices[e2Index + 2];
        // sub: inEdgeVertex1 - e2
        const diff2x_ = inEdgeVertex1[0] - e2x;
        const diff2y_ = inEdgeVertex1[1] - e2y;
        const diff2z_ = inEdgeVertex1[2] - e2z;
        // dot: diff · edgeNormal
        const num = diff2x_ * edgeNormalX + diff2y_ * edgeNormalY + diff2z_ * edgeNormalZ;
        const curInside = num < 0.0;

        // in -> out or out -> in: add point on clipping plane
        if (curInside !== prevInside) {
            // solve: (inEdgeVertex1 - X) . edge_normal = 0 and X = e1 + t * (e2 - e1) for X
            // sub: e2 - e1
            const e12x = e2x - e1x;
            const e12y = e2y - e1y;
            const e12z = e2z - e1z;
            // dot: e12 · edgeNormal
            const denom = e12x * edgeNormalX + e12y * edgeNormalY + e12z * edgeNormalZ;

            let clippedX: number;
            let clippedY: number;
            let clippedZ: number;

            if (denom !== 0.0) {
                const t = prevNum / denom;
                // scaleAndAdd: e1 + e12 * t
                clippedX = e1x + e12x * t;
                clippedY = e1y + e12y * t;
                clippedZ = e1z + e12z * t;
            } else {
                clippedX = e1x;
                clippedY = e1y;
                clippedZ = e1z;
            }

            // project point on line segment v1, v2 to see if it falls outside of the edge
            // sub: clipped - v1
            const clippedDiffX = clippedX - v1x;
            const clippedDiffY = clippedY - v1y;
            const clippedDiffZ = clippedZ - v1z;
            // dot: clippedDiff · v12
            const projection = clippedDiffX * v12x + clippedDiffY * v12y + clippedDiffZ * v12z;

            const outIndex = out.numVertices * 3;
            if (projection < 0.0) {
                out.vertices[outIndex] = v1x;
                out.vertices[outIndex + 1] = v1y;
                out.vertices[outIndex + 2] = v1z;
            } else if (projection > v12LenSq) {
                out.vertices[outIndex] = v2x;
                out.vertices[outIndex + 1] = v2y;
                out.vertices[outIndex + 2] = v2z;
            } else {
                out.vertices[outIndex] = clippedX;
                out.vertices[outIndex + 1] = clippedY;
                out.vertices[outIndex + 2] = clippedZ;
            }
            out.numVertices++;
        }

        // IMPORTANT: Unlike ClipPolyVsPlane, we do NOT add interior vertices
        // This function only outputs intersection points

        // update previous state
        prevNum = num;
        prevInside = curInside;
        e1x = e2x;
        e1y = e2y;
        e1z = e2z;
    }
}
