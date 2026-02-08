import { type Box3, box3, type Mat4, mat4, plane3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import { DEFAULT_CONVEX_RADIUS, type Support, SupportFunctionMode } from '../collision/support';
import { assert } from '../utils/assert';
import { isScaleInsideOut, transformFace } from '../utils/face';
import * as convex from './convex';
import {
    DEFAULT_SHAPE_DENSITY,
    defineShape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
    shapeDefs,
} from './shapes';
import * as convexHullBuilder from './utils/convex-hull-builder';

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
    /** points of the convex hull */
    points: ConvexHullPoint[];
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

export type ConvexHullPoint = {
    /** position of the vertex */
    position: Vec3;
    /** number of faces in the face array */
    numFaces: number;
    /** indices of 3 neighboring faces with the biggest difference in normal (used to shift vertices for convex radius) */
    faces: [number, number, number];
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

const MAX_POINTS_IN_HULL = 256;
const MAX_FACE_VERTICES = 32;

const _tetrahedronVertex1 = /* @__PURE__ */ vec3.create();
const _tetrahedronVertex2 = /* @__PURE__ */ vec3.create();
const _tetrahedronVertex3 = /* @__PURE__ */ vec3.create();
const _covarianceTemp = /* @__PURE__ */ mat4.create();
const _covarianceTransposed = /* @__PURE__ */ mat4.create();
const _covarianceResult = /* @__PURE__ */ mat4.create();
const _normalMatrixAdjoint = /* @__PURE__ */ mat4.create();
const _vertexShiftDirection = /* @__PURE__ */ vec3.create();
const _faceNormalCross = /* @__PURE__ */ vec3.create();
const _perpendicularNormal = /* @__PURE__ */ vec3.create();
const _affineTransform = /* @__PURE__ */ mat4.create();

const _supportingFace_invScale = /* @__PURE__ */ vec3.create();
const _supportingFace_planeNormal = /* @__PURE__ */ vec3.create();

/** create a convex hull shape */
export function create(o: ConvexHullShapeSettings): ConvexHullShape {
    const hullTolerance = o.hullTolerance ?? 1e-3;
    const convexRadius = o.convexRadius ?? DEFAULT_CONVEX_RADIUS;
    const density = o.density ?? DEFAULT_SHAPE_DENSITY;

    // validate convex radius
    if (convexRadius < 0) {
        throw new Error('Invalid convex radius');
    }

    // convert flat positions array to Vec3[]
    const inputPoints: Vec3[] = [];
    for (let i = 0; i < o.positions.length / 3; i++) {
        const idx = i * 3;
        inputPoints.push(vec3.fromValues(o.positions[idx], o.positions[idx + 1], o.positions[idx + 2]));
    }

    // build convex hull
    const builder = convexHullBuilder.create(inputPoints);
    const initResult = convexHullBuilder.initialize(builder, MAX_POINTS_IN_HULL, hullTolerance);

    if (
        initResult.result !== convexHullBuilder.Result.Success &&
        initResult.result !== convexHullBuilder.Result.MaxVerticesReached
    ) {
        throw new Error(`Convex hull build failed: ${initResult.error}`);
    }

    // get center of mass and volume
    const { centerOfMass, volume } = convexHullBuilder.getCenterOfMassAndVolume(builder);

    // calculate covariance matrix for inertia tensor
    // canonical covariance matrix for unit tetrahedron
    // see: http://number-none.com/blow/inertia/deriving_i.html
    // biome-ignore format: readability
    const covarianceCanonical = mat4.set(
        _affineTransform,
        1.0 / 60.0 , 1.0 / 120.0, 1.0 / 120.0, 0,
        1.0 / 120.0, 1.0 / 60.0 , 1.0 / 120.0, 0,
        1.0 / 120.0, 1.0 / 120.0, 1.0 / 60.0 , 0,
        0,           0,           0,           1,
    );
    const covarianceMatrix = mat4.create();
    mat4.identity(covarianceMatrix);
    mat4.multiplyScalar(covarianceMatrix, covarianceMatrix, 0); // zero it

    const faces = builder.faces;
    for (const face of faces) {
        // fourth point of tetrahedron is at center of mass
        // subtract center of mass from other points to get tetrahedron with one vertex at origin
        let edge = face.firstEdge;
        vec3.subtract(_tetrahedronVertex1, inputPoints[edge!.mStartIdx], centerOfMass);

        // get second point
        edge = edge!.mNextEdge;
        vec3.subtract(_tetrahedronVertex2, inputPoints[edge!.mStartIdx], centerOfMass);

        // loop over triangle fan
        for (edge = edge!.mNextEdge; edge !== face.firstEdge; edge = edge!.mNextEdge) {
            vec3.subtract(_tetrahedronVertex3, inputPoints[edge!.mStartIdx], centerOfMass);

            // affine transform that transforms unit tetrahedron to this tetrahedron
            // biome-ignore format: readability
            const a = mat4.fromValues(
                _tetrahedronVertex1[0], _tetrahedronVertex1[1], _tetrahedronVertex1[2], 0,
                _tetrahedronVertex2[0], _tetrahedronVertex2[1], _tetrahedronVertex2[2], 0,
                _tetrahedronVertex3[0], _tetrahedronVertex3[1], _tetrahedronVertex3[2], 0,
                0,                      0,                      0,                      1,
            );

            // calculate determinant (3x3 part)
            const det_a =
                _tetrahedronVertex1[0] *
                    (_tetrahedronVertex2[1] * _tetrahedronVertex3[2] - _tetrahedronVertex2[2] * _tetrahedronVertex3[1]) -
                _tetrahedronVertex1[1] *
                    (_tetrahedronVertex2[0] * _tetrahedronVertex3[2] - _tetrahedronVertex2[2] * _tetrahedronVertex3[0]) +
                _tetrahedronVertex1[2] *
                    (_tetrahedronVertex2[0] * _tetrahedronVertex3[1] - _tetrahedronVertex2[1] * _tetrahedronVertex3[0]);

            // calculate covariance matrix for this tetrahedron: det_a * (a * covarianceCanonical * a^T)
            mat4.multiply(_covarianceTemp, a, covarianceCanonical);
            mat4.transpose(_covarianceTransposed, a);
            mat4.multiply(_covarianceResult, _covarianceTemp, _covarianceTransposed);
            mat4.multiplyScalar(_covarianceResult, _covarianceResult, det_a);

            // add to total covariance matrix
            mat4.add(covarianceMatrix, covarianceMatrix, _covarianceResult);

            // prepare for next triangle
            vec3.copy(_tetrahedronVertex2, _tetrahedronVertex3);
        }
    }

    // calculate inertia matrix: I = identity * trace(C) - C
    // trace(C) = C[0] + C[5] + C[10]
    const trace = covarianceMatrix[0] + covarianceMatrix[5] + covarianceMatrix[10];
    const inertia = mat4.create();
    mat4.identity(inertia);
    mat4.multiplyScalar(inertia, inertia, trace);
    mat4.subtract(inertia, inertia, covarianceMatrix);

    // convert polygons from builder to runtime representation
    const vertexMap = new Map<number, number>();
    const points: ConvexHullPoint[] = [];
    const hullFaces: ConvexHullFace[] = [];
    const planes: ConvexHullPlane[] = [];
    const vertexIndices: number[] = [];
    const localBounds = box3.create();
    box3.set(localBounds, [Infinity, Infinity, Infinity], [-Infinity, -Infinity, -Infinity]);

    for (const builderFace of faces) {
        const firstVertex = vertexIndices.length;
        let numVertices = 0;

        // loop over vertices in face
        let edge = builderFace.firstEdge;
        do {
            const originalIdx = edge!.mStartIdx;
            let newIdx = vertexMap.get(originalIdx);

            if (newIdx === undefined) {
                // new point - make relative to center of mass
                const p = vec3.subtract(vec3.create(), inputPoints[originalIdx], centerOfMass);

                // update local bounds
                box3.expandByPoint(localBounds, localBounds, p);

                // add to point list
                newIdx = points.length;
                points.push({
                    position: p,
                    numFaces: 0,
                    faces: [-1, -1, -1],
                });
                vertexMap.set(originalIdx, newIdx);
            }

            // append to vertex list
            vertexIndices.push(newIdx);
            numVertices++;

            edge = edge!.mNextEdge;
        } while (edge !== builderFace.firstEdge);

        // add face
        hullFaces.push({ firstVertex, numVertices });

        // add plane
        // plane equation: normal · (x - centroid) = 0  =>  normal · x = normal · centroid
        // standard form: normal · x + constant = 0  =>  constant = -normal · centroid
        const normal = vec3.normalize(vec3.create(), builderFace.normal);
        const centroidRelative = vec3.subtract(vec3.create(), builderFace.centroid, centerOfMass);
        const constant = -vec3.dot(normal, centroidRelative);
        planes.push({ normal, constant });
    }

    // validate number of points
    if (points.length > MAX_POINTS_IN_HULL) {
        throw new Error(
            `Too many points in hull (${points.length}), max allowed ${MAX_POINTS_IN_HULL}, try increasing hullTolerance`,
        );
    }

    // for each point, find neighboring faces for convex radius support
    for (let p = 0; p < points.length; p++) {
        const neighboringFaces: number[] = [];

        for (let f = 0; f < hullFaces.length; f++) {
            const face = hullFaces[f];
            for (let v = 0; v < face.numVertices; v++) {
                if (vertexIndices[face.firstVertex + v] === p) {
                    neighboringFaces.push(f);
                    break;
                }
            }
        }

        if (neighboringFaces.length < 2) {
            throw new Error('A point must be connected to 2 or more faces!');
        }

        // find the 3 normals that form the largest tetrahedron
        // if volume < 5% of max possible (1.0), fall back to 2 normals
        // if 2 normals are too close (angle < 1°), fall back to 1 normal
        let biggestVolume = 0.05;
        const best3 = [-1, -1, -1];
        let smallestDot = Math.cos((1.0 * Math.PI) / 180.0); // 1 degree in radians
        const best2 = [-1, -1];

        for (let face1 = 0; face1 < neighboringFaces.length; face1++) {
            const normal1 = planes[neighboringFaces[face1]].normal;
            for (let face2 = face1 + 1; face2 < neighboringFaces.length; face2++) {
                const normal2 = planes[neighboringFaces[face2]].normal;
                vec3.cross(_faceNormalCross, normal1, normal2);
                const dot = vec3.dot(normal1, normal2);

                if (dot < smallestDot) {
                    smallestDot = dot;
                    best2[0] = neighboringFaces[face1];
                    best2[1] = neighboringFaces[face2];
                }

                for (let face3 = face2 + 1; face3 < neighboringFaces.length; face3++) {
                    const normal3 = planes[neighboringFaces[face3]].normal;
                    const volume = Math.abs(vec3.dot(_faceNormalCross, normal3));
                    if (volume > biggestVolume) {
                        biggestVolume = volume;
                        best3[0] = neighboringFaces[face1];
                        best3[1] = neighboringFaces[face2];
                        best3[2] = neighboringFaces[face3];
                    }
                }
            }
        }

        // select best faces
        const point = points[p];
        if (best3[0] !== -1) {
            point.numFaces = 3;
            point.faces = [best3[0], best3[1], best3[2]];
        } else if (best2[0] !== -1) {
            point.numFaces = 2;
            point.faces = [best2[0], best2[1], -1];
        } else {
            point.numFaces = 1;
            point.faces = [neighboringFaces[0], -1, -1];
        }
    }

    // reduce convex radius if hull is too thin or has sharp edges
    let finalConvexRadius = convexRadius;

    // pass 1: check hull thickness - ensure convex radius fits
    if (finalConvexRadius > 0) {
        let minSize = Number.MAX_VALUE;

        // for each plane, find the furthest point behind it (hull thickness in that direction)
        for (const plane of planes) {
            let maxDist = 0;
            for (const point of points) {
                // point is always behind plane (hull is convex), so negate signed distance
                const dist = -(vec3.dot(plane.normal, point.position) + plane.constant);
                if (dist > maxDist) {
                    maxDist = dist;
                }
            }
            minSize = Math.min(minSize, maxDist);
        }

        // we need to fit 2× convex radius in minSize
        finalConvexRadius = Math.min(finalConvexRadius, 0.5 * minSize);
    }

    // pass 2: check sharp edges - ensure vertices don't shift too far when planes are offset
    if (finalConvexRadius > 0) {
        const maxErrorConvexRadius = o.maxErrorConvexRadius ?? 0.05;

        for (const point of points) {
            // skip if only 1 face (no sharp edge, shifting back is simple)
            if (point.numFaces === 1) continue;

            // get the 3 planes that define this vertex
            const p1 = planes[point.faces[0]];
            const p2 = planes[point.faces[1]];
            let p3: ConvexHullPlane;
            let offsetMask: Vec3;

            if (point.numFaces === 3) {
                // all 3 neighboring planes are offset by convex radius
                p3 = planes[point.faces[2]];
                offsetMask = [1, 1, 1];
            } else {
                // only 2 planes, create perpendicular plane through vertex
                const n1 = p1.normal;
                const n2 = p2.normal;
                vec3.cross(_perpendicularNormal, n1, n2);
                vec3.normalize(_perpendicularNormal, _perpendicularNormal);
                const perpConstant = -vec3.dot(_perpendicularNormal, point.position);
                p3 = { normal: _perpendicularNormal, constant: perpConstant };
                offsetMask = [1, 1, 0]; // third plane not offset
            }

            // build matrix n with plane normals as rows (transposed from column form)
            // n = [n1; n2; n3] where each normal is a row
            const n = mat4.fromValues(
                p1.normal[0],
                p2.normal[0],
                p3.normal[0],
                0,
                p1.normal[1],
                p2.normal[1],
                p3.normal[1],
                0,
                p1.normal[2],
                p2.normal[2],
                p3.normal[2],
                0,
                0,
                0,
                0,
                1,
            );

            const detN = mat4.determinant(n);
            if (Math.abs(detN) < 1e-12) {
                // planes are coplanar/degenerate, can't offset safely
                finalConvexRadius = 0;
                break;
            }

            // calculate shift direction: adjoint(n) × offsetMask / det(n)
            mat4.adjoint(_normalMatrixAdjoint, n);
            vec3.transformMat4(_vertexShiftDirection, offsetMask, _normalMatrixAdjoint);
            vec3.scale(_vertexShiftDirection, _vertexShiftDirection, 1 / detN);
            const offset = vec3.length(_vertexShiftDirection);

            // max radius: error = r × (offset - 1)  =>  r = error / (offset - 1)
            if (offset > 1) {
                const maxRadius = maxErrorConvexRadius / (offset - 1);
                finalConvexRadius = Math.min(finalConvexRadius, maxRadius);
            }
            // if offset ≤ 1, vertex moves inward, no constraint needed
        }
    }

    const shape: ConvexHullShape = {
        type: ShapeType.CONVEX_HULL,
        points,
        faces: hullFaces,
        planes,
        vertexIndices,
        convexRadius: finalConvexRadius,
        density,
        materialId: o.materialId ?? -1,
        aabb: localBounds,
        centerOfMass,
        volume,
        inertia,
    };

    return shape;
}

/* shape def */

export const def = /* @__PURE__ */ (() =>
    defineShape<ConvexHullShape>({
        type: ShapeType.CONVEX_HULL,
        category: ShapeCategory.CONVEX,
        computeMassProperties(out: MassProperties, shape: ConvexHullShape): void {
            // mass = volume * density
            out.mass = shape.volume * shape.density;

            // scale inertia tensor by density
            mat4.multiplyScalar(out.inertia, shape.inertia, shape.density);
        },
        getSurfaceNormal(ioResult: SurfaceNormalResult, shape: ConvexHullShape, subShapeId: number): void {
            assert(subShape.isEmpty(subShapeId), 'Invalid subshape ID for ConvexHullShape');

            // find the plane with the smallest absolute signed distance to the surface point
            const firstPlane = shape.planes[0];
            vec3.copy(ioResult.normal, firstPlane.normal);
            let bestDist = Math.abs(firstPlane.constant + vec3.dot(firstPlane.normal, ioResult.position));

            for (let i = 1; i < shape.planes.length; i++) {
                const plane = shape.planes[i];
                const dist = Math.abs(plane.constant + vec3.dot(plane.normal, ioResult.position));
                if (dist < bestDist) {
                    bestDist = dist;
                    vec3.copy(ioResult.normal, plane.normal);
                }
            }
        },
        getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: ConvexHullShape, _subShapeId: number): void {
            const face = ioResult.face;
            const scale = ioResult.scale;
            const quaternion = ioResult.quaternion;
            const position = ioResult.position;

            // compute inverse scale for normal transformation
            // normals transform by (M^-1)^T, for diagonal scale matrix this is 1/scale
            vec3.set(_supportingFace_invScale, 1 / scale[0], 1 / scale[1], 1 / scale[2]);

            // transform first plane normal and find initial best
            vec3.multiply(_supportingFace_planeNormal, _supportingFace_invScale, shape.planes[0].normal);
            const plane0NormalLength = vec3.length(_supportingFace_planeNormal);
            let bestDot = vec3.dot(_supportingFace_planeNormal, direction) / plane0NormalLength;
            let bestFaceIdx = 0;

            // find face with smallest (most negative) dot product
            for (let i = 1; i < shape.planes.length; i++) {
                vec3.multiply(_supportingFace_planeNormal, _supportingFace_invScale, shape.planes[i].normal);
                const planeNormalLength = vec3.length(_supportingFace_planeNormal);
                const dot = vec3.dot(_supportingFace_planeNormal, direction) / planeNormalLength;

                if (dot < bestDot) {
                    bestDot = dot;
                    bestFaceIdx = i;
                }
            }

            // get vertices of best face
            const bestFace = shape.faces[bestFaceIdx];
            const firstVtxIdx = bestFace.firstVertex;
            const numVertices = bestFace.numVertices;

            // downsample if too many vertices (prevent overflow in contact clipping)
            // TODO: (comment from JoltPhysics) This really needs a better algorithm to determine which vertices are important!
            const maxVerticesToReturn = Math.floor(MAX_FACE_VERTICES / 2);
            const deltaVtx = Math.floor((numVertices + maxVerticesToReturn - 1) / maxVerticesToReturn);

            // check if scale inverts winding (negative determinant)
            const insideOut = isScaleInsideOut(scale);

            // store local vertices
            face.numVertices = 0;

            if (insideOut) {
                // flip winding of supporting face
                for (let i = numVertices - 1; i >= 0; i -= deltaVtx) {
                    const vtxIdx = shape.vertexIndices[firstVtxIdx + i];
                    const vertex = shape.points[vtxIdx].position;
                    const base = face.numVertices * 3;
                    face.vertices[base] = vertex[0];
                    face.vertices[base + 1] = vertex[1];
                    face.vertices[base + 2] = vertex[2];
                    face.numVertices++;
                }
            } else {
                // normal winding of supporting face
                for (let i = 0; i < numVertices; i += deltaVtx) {
                    const vtxIdx = shape.vertexIndices[firstVtxIdx + i];
                    const vertex = shape.points[vtxIdx].position;
                    const base = face.numVertices * 3;
                    face.vertices[base] = vertex[0];
                    face.vertices[base + 1] = vertex[1];
                    face.vertices[base + 2] = vertex[2];
                    face.numVertices++;
                }
            }

            transformFace(face, position, quaternion, scale);
        },
        getInnerRadius(shape: ConvexHullShape): number {
            // calculate the inner radius by getting the minimum distance from the origin to the planes of the hull
            let innerRadius = Number.MAX_VALUE;
            for (const plane of shape.planes) {
                // distance from origin (0,0,0) to plane: -constant (since points are relative to COM)
                innerRadius = Math.min(innerRadius, -plane.constant);
            }
            // clamp against zero for numerical stability (flat convex hulls may have round-off issues)
            return Math.max(0.0, innerRadius);
        },
        castRay: convex.castRayVsConvex,
        collidePoint: convex.collidePointVsConvex,
        createSupportPool: createConvexHullSupportPool,
        getSupportFunction: getConvexHullSupportFunction,
        register: () => {
            for (const shapeDef of Object.values(shapeDefs)) {
                if (shapeDef.category === ShapeCategory.CONVEX) {
                    setCollideShapeFn(ShapeType.CONVEX_HULL, shapeDef.type, convex.collideConvexVsConvex);
                    setCollideShapeFn(shapeDef.type, ShapeType.CONVEX_HULL, convex.collideConvexVsConvex);
                    setCastShapeFn(ShapeType.CONVEX_HULL, shapeDef.type, convex.castConvexVsConvex);
                    setCastShapeFn(shapeDef.type, ShapeType.CONVEX_HULL, convex.castConvexVsConvex);
                }
            }
        },
    }))();

/* support functions */

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

function convexHullWithConvexGetSupport(this: ConvexHullWithConvexSupport, direction: Vec3, out: Vec3): void {
    // find point with highest projection on direction
    let bestDot = -Infinity;
    let bestPoint: Vec3 | null = null;

    for (const point of this.shape.points) {
        // const dot = vec3.dot(point.position, direction);
        const dot = point.position[0] * direction[0] + point.position[1] * direction[1] + point.position[2] * direction[2];

        if (dot > bestDot) {
            bestDot = dot;
            bestPoint = point.position;
        }
    }

    if (bestPoint) {
        // vec3.copy(out, bestPoint);
        out[0] = bestPoint[0];
        out[1] = bestPoint[1];
        out[2] = bestPoint[2];
    } else {
        // vec3.zero(out);
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
    }
}

export function createConvexHullWithConvexSupport(): ConvexHullWithConvexSupport {
    return {
        shape: null!,
        convexRadius: 0,
        getSupport: convexHullWithConvexGetSupport,
    };
}

export function setConvexHullWithConvexSupport(out: ConvexHullWithConvexSupport, shape: ConvexHullShape): void {
    out.shape = shape;
    out.convexRadius = 0;
}

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

function convexHullWithConvexScaledGetSupport(this: ConvexHullWithConvexSupportScaled, direction: Vec3, out: Vec3): void {
    // Find point with highest projection on direction
    let bestDot = -Infinity;
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;

    for (const point of this.shape.points) {
        // Apply scale per-vertex
        const scaledX = point.position[0] * this.scale[0];
        const scaledY = point.position[1] * this.scale[1];
        const scaledZ = point.position[2] * this.scale[2];

        const dot = scaledX * direction[0] + scaledY * direction[1] + scaledZ * direction[2];
        if (dot > bestDot) {
            bestDot = dot;
            out[0] = scaledX;
            out[1] = scaledY;
            out[2] = scaledZ;
        }
    }
}

export function createConvexHullWithConvexSupportScaled(): ConvexHullWithConvexSupportScaled {
    return {
        shape: null!,
        scale: vec3.create(),
        convexRadius: 0,
        getSupport: convexHullWithConvexScaledGetSupport,
    };
}

export function setConvexHullWithConvexSupportScaled(
    out: ConvexHullWithConvexSupportScaled,
    shape: ConvexHullShape,
    scale: Vec3,
): void {
    out.shape = shape;
    vec3.copy(out.scale, scale);
    out.convexRadius = 0;
}

/**
 * ConvexHull support for EXCLUDE_CONVEX_RADIUS mode (unscaled).
 * Pre-computes shrunk vertices using plane intersection.
 * More expensive construction, fast queries.
 */
export type ConvexHullNoConvexSupport = {
    points: number[]; // Pre-allocated flat array [x,y,z,x,y,z,...]
    numPoints: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

function convexHullNoConvexGetSupport(this: ConvexHullNoConvexSupport, direction: Vec3, out: Vec3): void {
    // Find point with highest projection on direction
    let bestDot = -Infinity;
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;

    for (let i = 0; i < this.numPoints; i++) {
        const x = this.points[i * 3 + 0];
        const y = this.points[i * 3 + 1];
        const z = this.points[i * 3 + 2];
        const dot = x * direction[0] + y * direction[1] + z * direction[2];

        if (dot > bestDot) {
            bestDot = dot;
            out[0] = x;
            out[1] = y;
            out[2] = z;
        }
    }
}

export function createConvexHullNoConvexSupport(): ConvexHullNoConvexSupport {
    return {
        points: [],
        numPoints: 0,
        convexRadius: 0,
        getSupport: convexHullNoConvexGetSupport,
    };
}

const _convexHullNoConvex_p1 = /* @__PURE__ */ plane3.create();
const _convexHullNoConvex_p2 = /* @__PURE__ */ plane3.create();
const _convexHullNoConvex_p3 = /* @__PURE__ */ plane3.create();
const _convexHullNoConvex_newPoint = /* @__PURE__ */ vec3.create();
const _convexHullNoConvex_perpNormal = /* @__PURE__ */ vec3.create();

export function setConvexHullNoConvexSupport(out: ConvexHullNoConvexSupport, shape: ConvexHullShape): void {
    const convexRadius = shape.convexRadius;
    const numPoints = shape.points.length;

    // extend array only if needed
    const requiredLength = numPoints * 3;
    while (out.points.length < requiredLength) {
        out.points.push(0);
    }
    out.numPoints = numPoints;
    out.convexRadius = convexRadius;

    let writeIndex = 0;

    for (const point of shape.points) {
        let newPoint: Vec3;

        if (point.numFaces === 1) {
            // simple case: shift back by convex radius using the one plane
            const plane = shape.planes[point.faces[0]];
            newPoint = _convexHullNoConvex_newPoint;
            vec3.scaleAndAdd(newPoint, point.position, plane.normal, -convexRadius);
        } else {
            // get first two planes and offset inwards by convex radius
            plane3.fromNormalAndPoint(_convexHullNoConvex_p1, shape.planes[point.faces[0]].normal, point.position);
            plane3.offset(_convexHullNoConvex_p1, _convexHullNoConvex_p1, -convexRadius);

            plane3.fromNormalAndPoint(_convexHullNoConvex_p2, shape.planes[point.faces[1]].normal, point.position);
            plane3.offset(_convexHullNoConvex_p2, _convexHullNoConvex_p2, -convexRadius);

            if (point.numFaces === 3) {
                // three plane intersection
                plane3.fromNormalAndPoint(_convexHullNoConvex_p3, shape.planes[point.faces[2]].normal, point.position);
                plane3.offset(_convexHullNoConvex_p3, _convexHullNoConvex_p3, -convexRadius);
            } else {
                // two faces: third plane is perpendicular to first two, through vertex
                vec3.cross(_convexHullNoConvex_perpNormal, _convexHullNoConvex_p1.normal, _convexHullNoConvex_p2.normal);
                vec3.normalize(_convexHullNoConvex_perpNormal, _convexHullNoConvex_perpNormal);
                plane3.fromNormalAndPoint(_convexHullNoConvex_p3, _convexHullNoConvex_perpNormal, point.position);
            }

            // intersect three planes
            newPoint = _convexHullNoConvex_newPoint;
            if (!plane3.intersect(_convexHullNoConvex_p1, _convexHullNoConvex_p2, _convexHullNoConvex_p3, newPoint)) {
                // fallback: just push back using first plane
                vec3.scaleAndAdd(newPoint, point.position, _convexHullNoConvex_p1.normal, -convexRadius);
            }
        }

        // write to flat array
        out.points[writeIndex++] = newPoint[0];
        out.points[writeIndex++] = newPoint[1];
        out.points[writeIndex++] = newPoint[2];
    }
}

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

function convexHullNoConvexScaledGetSupport(this: ConvexHullNoConvexSupportScaled, direction: Vec3, out: Vec3): void {
    // find point with highest projection on direction
    let bestDot = -Infinity;
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;

    for (let i = 0; i < this.numPoints; i++) {
        const x = this.points[i * 3 + 0];
        const y = this.points[i * 3 + 1];
        const z = this.points[i * 3 + 2];
        const dot = x * direction[0] + y * direction[1] + z * direction[2];

        if (dot > bestDot) {
            bestDot = dot;
            out[0] = x;
            out[1] = y;
            out[2] = z;
        }
    }
}

export function createConvexHullNoConvexSupportScaled(): ConvexHullNoConvexSupportScaled {
    return {
        points: [],
        numPoints: 0,
        convexRadius: 0,
        getSupport: convexHullNoConvexScaledGetSupport,
    };
}

const _convexHullNoConvexScaled_invScale = /* @__PURE__ */ vec3.create();
const _convexHullNoConvexScaled_scaledPos = /* @__PURE__ */ vec3.create();
const _convexHullNoConvexScaled_n1 = /* @__PURE__ */ vec3.create();
const _convexHullNoConvexScaled_n2 = /* @__PURE__ */ vec3.create();
const _convexHullNoConvexScaled_n3 = /* @__PURE__ */ vec3.create();
const _convexHullNoConvexScaled_p1 = /* @__PURE__ */ plane3.create();
const _convexHullNoConvexScaled_p2 = /* @__PURE__ */ plane3.create();
const _convexHullNoConvexScaled_p3 = /* @__PURE__ */ plane3.create();
const _convexHullNoConvexScaled_newPoint = /* @__PURE__ */ vec3.create();
const _convexHullNoConvexScaled_perpNormal = /* @__PURE__ */ vec3.create();

function scaleConvexRadius(radius: number, scale: Vec3): number {
    // use minimum absolute scale component
    const minScale = Math.min(Math.abs(scale[0]), Math.abs(scale[1]), Math.abs(scale[2]));
    return radius * minScale;
}

export function setConvexHullNoConvexSupportScaled(
    out: ConvexHullNoConvexSupportScaled,
    shape: ConvexHullShape,
    scale: Vec3,
): void {
    // scale convex radius
    const scaledRadius = scaleConvexRadius(shape.convexRadius, scale);

    const numPoints = shape.points.length;

    // extend array only if needed
    const requiredLength = numPoints * 3;
    while (out.points.length < requiredLength) {
        out.points.push(0);
    }
    out.numPoints = numPoints;
    out.convexRadius = scaledRadius;

    // compute inverse scale for normal transformation
    vec3.set(_convexHullNoConvexScaled_invScale, 1 / scale[0], 1 / scale[1], 1 / scale[2]);

    let writeIndex = 0;

    for (const point of shape.points) {
        // calculate scaled position
        vec3.multiply(_convexHullNoConvexScaled_scaledPos, point.position, scale);

        let newPoint: Vec3;

        if (point.numFaces === 1) {
            // transform normal with inverse scale and renormalize
            vec3.multiply(_convexHullNoConvexScaled_n1, _convexHullNoConvexScaled_invScale, shape.planes[point.faces[0]].normal);
            vec3.normalize(_convexHullNoConvexScaled_n1, _convexHullNoConvexScaled_n1);

            // simple shift
            newPoint = _convexHullNoConvexScaled_newPoint;
            vec3.scaleAndAdd(newPoint, _convexHullNoConvexScaled_scaledPos, _convexHullNoConvexScaled_n1, -scaledRadius);
        } else {
            // transform normals with inverse scale
            vec3.multiply(_convexHullNoConvexScaled_n1, _convexHullNoConvexScaled_invScale, shape.planes[point.faces[0]].normal);
            vec3.normalize(_convexHullNoConvexScaled_n1, _convexHullNoConvexScaled_n1);

            vec3.multiply(_convexHullNoConvexScaled_n2, _convexHullNoConvexScaled_invScale, shape.planes[point.faces[1]].normal);
            vec3.normalize(_convexHullNoConvexScaled_n2, _convexHullNoConvexScaled_n2);

            // create planes from scaled position and transformed normals
            plane3.fromNormalAndPoint(
                _convexHullNoConvexScaled_p1,
                _convexHullNoConvexScaled_n1,
                _convexHullNoConvexScaled_scaledPos,
            );
            plane3.offset(_convexHullNoConvexScaled_p1, _convexHullNoConvexScaled_p1, -scaledRadius);

            plane3.fromNormalAndPoint(
                _convexHullNoConvexScaled_p2,
                _convexHullNoConvexScaled_n2,
                _convexHullNoConvexScaled_scaledPos,
            );
            plane3.offset(_convexHullNoConvexScaled_p2, _convexHullNoConvexScaled_p2, -scaledRadius);

            if (point.numFaces === 3) {
                // transform third normal
                vec3.multiply(
                    _convexHullNoConvexScaled_n3,
                    _convexHullNoConvexScaled_invScale,
                    shape.planes[point.faces[2]].normal,
                );
                vec3.normalize(_convexHullNoConvexScaled_n3, _convexHullNoConvexScaled_n3);

                plane3.fromNormalAndPoint(
                    _convexHullNoConvexScaled_p3,
                    _convexHullNoConvexScaled_n3,
                    _convexHullNoConvexScaled_scaledPos,
                );
                plane3.offset(_convexHullNoConvexScaled_p3, _convexHullNoConvexScaled_p3, -scaledRadius);
            } else {
                // third plane perpendicular to first two (no offset)
                vec3.cross(_convexHullNoConvexScaled_perpNormal, _convexHullNoConvexScaled_n1, _convexHullNoConvexScaled_n2);
                vec3.normalize(_convexHullNoConvexScaled_perpNormal, _convexHullNoConvexScaled_perpNormal);
                plane3.fromNormalAndPoint(
                    _convexHullNoConvexScaled_p3,
                    _convexHullNoConvexScaled_perpNormal,
                    _convexHullNoConvexScaled_scaledPos,
                );
            }

            // intersect three planes
            newPoint = _convexHullNoConvexScaled_newPoint;
            if (
                !plane3.intersect(
                    _convexHullNoConvexScaled_p1,
                    _convexHullNoConvexScaled_p2,
                    _convexHullNoConvexScaled_p3,
                    newPoint,
                )
            ) {
                // fallback: just push back using first plane
                vec3.scaleAndAdd(newPoint, _convexHullNoConvexScaled_scaledPos, _convexHullNoConvexScaled_n1, -scaledRadius);
            }
        }

        // write to flat array
        out.points[writeIndex++] = newPoint[0];
        out.points[writeIndex++] = newPoint[1];
        out.points[writeIndex++] = newPoint[2];
    }
}

type ConvexHullSupportPool = {
    withConvex: ConvexHullWithConvexSupport;
    withConvexScaled: ConvexHullWithConvexSupportScaled;
    noConvex: ConvexHullNoConvexSupport;
    noConvexScaled: ConvexHullNoConvexSupportScaled;
};

function createConvexHullSupportPool(): ConvexHullSupportPool {
    return {
        withConvex: createConvexHullWithConvexSupport(),
        withConvexScaled: createConvexHullWithConvexSupportScaled(),
        noConvex: createConvexHullNoConvexSupport(),
        noConvexScaled: createConvexHullNoConvexSupportScaled(),
    };
}

function getConvexHullSupportFunction(
    pool: ConvexHullSupportPool,
    shape: ConvexHullShape,
    mode: SupportFunctionMode,
    scale: Vec3,
): Support {
    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS || shape.convexRadius === 0) {
        // use original hull geometry (convexRadius = 0)
        if (scale[0] !== 1 || scale[1] !== 1 || scale[2] !== 1) {
            setConvexHullWithConvexSupportScaled(pool.withConvexScaled, shape, scale);
            return pool.withConvexScaled;
        } else {
            setConvexHullWithConvexSupport(pool.withConvex, shape);
            return pool.withConvex;
        }
    } else {
        // shrink hull if EXCLUDE_CONVEX_RADIUS or DEFAULT
        if (scale[0] !== 1 || scale[1] !== 1 || scale[2] !== 1) {
            setConvexHullNoConvexSupportScaled(pool.noConvexScaled, shape, scale);
            return pool.noConvexScaled;
        } else {
            setConvexHullNoConvexSupport(pool.noConvex, shape);
            return pool.noConvex;
        }
    }
}
