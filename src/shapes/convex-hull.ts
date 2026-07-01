import { type Box3, box3, type Mat4, mat4, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import { DEFAULT_CONVEX_RADIUS, type Support, SupportFunctionMode } from '../collision/support';
import { assert } from '../utils/assert';
import { isScaleInsideOut, transformFaceWithMat4Scale } from '../utils/face';
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
    /** flat vertex positions `[x, y, z, ...]`, 3 per point */
    pointPositions: number[];
    /** number of neighbouring faces per point (1..3), 1 per point (used by the convex-radius shrink) */
    pointNumFaces: number[];
    /** up to 3 neighbouring face indices per point `[f0, f1, f2, ...]` (-1 = unused), 3 per point */
    pointFaces: number[];
    /** number of hull vertices (`pointPositions.length / 3`) */
    numPoints: number;
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

    // convert polygons from the builder into the shape's flat vertex arrays
    const vertexMap = new Map<number, number>();
    const pointPositions: number[] = []; // [x,y,z, ...] 3 per point
    const pointNumFaces: number[] = []; // 1 per point (1..3)
    const pointFaces: number[] = []; // [f0,f1,f2, ...] 3 per point, -1 = unused
    let numPoints = 0;
    const hullFaces: ConvexHullFace[] = [];
    const planes: ConvexHullPlane[] = [];
    const vertexIndices: number[] = [];
    const localBounds = box3.create();
    box3.empty(localBounds);

    for (const builderFace of faces) {
        const firstVertex = vertexIndices.length;
        let numVertices = 0;

        // loop over vertices in face
        let edge = builderFace.firstEdge;
        do {
            const originalIdx = edge!.mStartIdx;
            let newIdx = vertexMap.get(originalIdx);

            if (newIdx === undefined) {
                // store point in original shape space (not relative to center of mass)
                const src = inputPoints[originalIdx];

                // update local bounds
                box3.expandByPoint(localBounds, localBounds, src);

                // append vertex to the packed arrays (numFaces/faces filled in below)
                newIdx = numPoints++;
                pointPositions.push(src[0], src[1], src[2]);
                pointNumFaces.push(0);
                pointFaces.push(-1, -1, -1);
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
        const constant = -vec3.dot(normal, builderFace.centroid);
        planes.push({ normal, constant });
    }

    // validate number of points
    if (numPoints > MAX_POINTS_IN_HULL) {
        throw new Error(
            `Too many points in hull (${numPoints}), max allowed ${MAX_POINTS_IN_HULL}, try increasing hullTolerance`,
        );
    }

    // for each point, find neighboring faces for convex radius support
    for (let p = 0; p < numPoints; p++) {
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
        const fb = p * 3;
        if (best3[0] !== -1) {
            pointNumFaces[p] = 3;
            pointFaces[fb] = best3[0];
            pointFaces[fb + 1] = best3[1];
            pointFaces[fb + 2] = best3[2];
        } else if (best2[0] !== -1) {
            pointNumFaces[p] = 2;
            pointFaces[fb] = best2[0];
            pointFaces[fb + 1] = best2[1];
            pointFaces[fb + 2] = -1;
        } else {
            pointNumFaces[p] = 1;
            pointFaces[fb] = neighboringFaces[0];
            pointFaces[fb + 1] = -1;
            pointFaces[fb + 2] = -1;
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
            const [nx, ny, nz] = plane.normal;
            for (let i = 0; i < pointPositions.length; i += 3) {
                // point is always behind plane (hull is convex), so negate signed distance
                const dist = -(nx * pointPositions[i] + ny * pointPositions[i + 1] + nz * pointPositions[i + 2] + plane.constant);
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

        for (let p = 0; p < numPoints; p++) {
            const numFaces = pointNumFaces[p];
            // skip if only 1 face (no sharp edge, shifting back is simple)
            if (numFaces === 1) continue;

            const fb = p * 3;
            // get the 3 planes that define this vertex
            const p1 = planes[pointFaces[fb]];
            const p2 = planes[pointFaces[fb + 1]];
            let p3: ConvexHullPlane;
            let offsetMask: Vec3;

            if (numFaces === 3) {
                // all 3 neighboring planes are offset by convex radius
                p3 = planes[pointFaces[fb + 2]];
                offsetMask = [1, 1, 1];
            } else {
                // only 2 planes, create perpendicular plane through vertex
                const n1 = p1.normal;
                const n2 = p2.normal;
                vec3.cross(_perpendicularNormal, n1, n2);
                vec3.normalize(_perpendicularNormal, _perpendicularNormal);
                const perpConstant = -(_perpendicularNormal[0] * pointPositions[fb] +
                    _perpendicularNormal[1] * pointPositions[fb + 1] +
                    _perpendicularNormal[2] * pointPositions[fb + 2]);
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
        pointPositions,
        pointNumFaces,
        pointFaces,
        numPoints,
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
        computeMassProperties,
        getSurfaceNormal,
        getSupportingFace,
        getInnerRadius,
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

function computeMassProperties(out: MassProperties, shape: ConvexHullShape): void {
    // mass = volume * density
    out.mass = shape.volume * shape.density;

    // scale inertia tensor by density
    mat4.multiplyScalar(out.inertia, shape.inertia, shape.density);
}

function getSurfaceNormal(ioResult: SurfaceNormalResult, shape: ConvexHullShape, subShapeId: number): void {
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
}

function getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: ConvexHullShape, _subShapeId: number): void {
    const face = ioResult.face;
    const scale = ioResult.scale;
    const transform = ioResult.transform;

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
            const pbase = vtxIdx * 3;
            const base = face.numVertices * 3;
            face.vertices[base] = shape.pointPositions[pbase];
            face.vertices[base + 1] = shape.pointPositions[pbase + 1];
            face.vertices[base + 2] = shape.pointPositions[pbase + 2];
            face.numVertices++;
        }
    } else {
        // normal winding of supporting face
        for (let i = 0; i < numVertices; i += deltaVtx) {
            const vtxIdx = shape.vertexIndices[firstVtxIdx + i];
            const pbase = vtxIdx * 3;
            const base = face.numVertices * 3;
            face.vertices[base] = shape.pointPositions[pbase];
            face.vertices[base + 1] = shape.pointPositions[pbase + 1];
            face.vertices[base + 2] = shape.pointPositions[pbase + 2];
            face.numVertices++;
        }
    }

    transformFaceWithMat4Scale(face, transform, scale);
}

function getInnerRadius(shape: ConvexHullShape): number {
    // calculate the inner radius by getting the minimum distance from the shape origin to the planes of the hull
    let innerRadius = Number.MAX_VALUE;
    for (const plane of shape.planes) {
        // distance from shape origin (0,0,0) to plane
        innerRadius = Math.min(innerRadius, -plane.constant);
    }
    // clamp against zero for numerical stability (flat convex hulls may have round-off issues)
    return Math.max(0.0, innerRadius);
}
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
    const p = this.shape.pointPositions;
    const dx = direction[0];
    const dy = direction[1];
    const dz = direction[2];
    let bestDot = -Infinity;
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;

    for (let i = 0, L = p.length; i < L; i += 3) {
        const x = p[i];
        const y = p[i + 1];
        const z = p[i + 2];
        const dot = x * dx + y * dy + z * dz;
        if (dot > bestDot) {
            bestDot = dot;
            out[0] = x;
            out[1] = y;
            out[2] = z;
        }
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
    // find point with highest projection on direction, scaling each vertex on the fly
    const p = this.shape.pointPositions;
    const sx = this.scale[0];
    const sy = this.scale[1];
    const sz = this.scale[2];
    const dx = direction[0];
    const dy = direction[1];
    const dz = direction[2];
    let bestDot = -Infinity;
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;

    for (let i = 0, L = p.length; i < L; i += 3) {
        const scaledX = p[i] * sx;
        const scaledY = p[i + 1] * sy;
        const scaledZ = p[i + 2] * sz;
        const dot = scaledX * dx + scaledY * dy + scaledZ * dz;
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
    // find point with highest projection on direction
    const p = this.points;
    const dx = direction[0];
    const dy = direction[1];
    const dz = direction[2];
    let bestDot = -Infinity;
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;

    for (let i = 0, L = this.numPoints * 3; i < L; i += 3) {
        const x = p[i];
        const y = p[i + 1];
        const z = p[i + 2];
        const dot = x * dx + y * dy + z * dz;
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

/**
 * Compute the convex-radius-shrunk hull vertices (unscaled) into `dst` as a flat [x,y,z,...] array.
 * Each neighbouring face plane is offset inward by the convex radius (constant += r) and the up-to-3
 * planes are intersected (Cramer's rule). For a 2-face vertex the third plane is perpendicular to the
 * first two through the vertex; its `n1 × n2` normal is left unnormalized (the intersection is
 * invariant to per-plane scale).
 */
function computeShrunkHullPoints(shape: ConvexHullShape, dst: number[]): void {
    const convexRadius = shape.convexRadius;
    const numPoints = shape.numPoints;
    const positions = shape.pointPositions;
    const numFacesArr = shape.pointNumFaces;
    const facesArr = shape.pointFaces;
    const planes = shape.planes;

    const requiredLength = numPoints * 3;
    while (dst.length < requiredLength) {
        dst.push(0);
    }

    let w = 0;
    for (let pi = 0; pi < numPoints; pi++) {
        const pb = pi * 3;
        const px = positions[pb];
        const py = positions[pb + 1];
        const pz = positions[pb + 2];
        const numFaces = numFacesArr[pi];

        // first neighbouring face plane (normal is unit; offset inward → constant + r)
        const plane1 = planes[facesArr[pb]];
        const nrm1 = plane1.normal;
        const n1x = nrm1[0];
        const n1y = nrm1[1];
        const n1z = nrm1[2];

        let rx: number;
        let ry: number;
        let rz: number;

        if (numFaces === 1) {
            // simple case: shift back along the single plane normal
            rx = px - n1x * convexRadius;
            ry = py - n1y * convexRadius;
            rz = pz - n1z * convexRadius;
        } else {
            const plane2 = planes[facesArr[pb + 1]];
            const nrm2 = plane2.normal;
            const n2x = nrm2[0];
            const n2y = nrm2[1];
            const n2z = nrm2[2];

            // offset the two face planes inward by the convex radius (use the stored plane constants)
            const d1 = plane1.constant + convexRadius;
            const d2 = plane2.constant + convexRadius;

            // third plane: 3rd face plane (offset inward), or a perpendicular plane through the vertex
            let n3x: number;
            let n3y: number;
            let n3z: number;
            let d3: number;
            if (numFaces === 3) {
                const plane3v = planes[facesArr[pb + 2]];
                const nrm3 = plane3v.normal;
                n3x = nrm3[0];
                n3y = nrm3[1];
                n3z = nrm3[2];
                d3 = plane3v.constant + convexRadius;
            } else {
                // third plane perpendicular to the first two, through the vertex (unnormalized normal)
                n3x = n1y * n2z - n1z * n2y;
                n3y = n1z * n2x - n1x * n2z;
                n3z = n1x * n2y - n1y * n2x;
                d3 = -(n3x * px + n3y * py + n3z * pz);
            }

            // intersect the three planes (Cramer's rule; the cross products are the adj columns)
            const c1x = n2y * n3z - n2z * n3y;
            const c1y = n2z * n3x - n2x * n3z;
            const c1z = n2x * n3y - n2y * n3x;
            const denom = n1x * c1x + n1y * c1y + n1z * c1z;

            if (Math.abs(denom) < 0.000001) {
                // near-parallel planes: fall back to pushing back along the first plane
                rx = px - n1x * convexRadius;
                ry = py - n1y * convexRadius;
                rz = pz - n1z * convexRadius;
            } else {
                const c2x = n3y * n1z - n3z * n1y;
                const c2y = n3z * n1x - n3x * n1z;
                const c2z = n3x * n1y - n3y * n1x;
                const c3x = n1y * n2z - n1z * n2y;
                const c3y = n1z * n2x - n1x * n2z;
                const c3z = n1x * n2y - n1y * n2x;
                const s = -1 / denom;
                rx = (d1 * c1x + d2 * c2x + d3 * c3x) * s;
                ry = (d1 * c1y + d2 * c2y + d3 * c3y) * s;
                rz = (d1 * c1z + d2 * c2z + d3 * c3z) * s;
            }
        }

        dst[w++] = rx;
        dst[w++] = ry;
        dst[w++] = rz;
    }
}

export function setConvexHullNoConvexSupport(out: ConvexHullNoConvexSupport, shape: ConvexHullShape): void {
    // rebuild the shrunk hull into the support object's buffer (runs once per pair, reused across
    // all GJK/EPA iterations)
    out.numPoints = shape.numPoints;
    out.convexRadius = shape.convexRadius;
    computeShrunkHullPoints(shape, out.points);
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
    // scaled variant: positions are scaled, face-plane normals are transformed by the inverse scale
    // and renormalized, planes are rebuilt through the scaled vertex, offset inward by the scaled
    // convex radius, then intersected. The 2-face third plane uses the unnormalized cross of n1, n2.
    const scaledRadius = scaleConvexRadius(shape.convexRadius, scale);
    const numPoints = shape.numPoints;
    const positions = shape.pointPositions;
    const numFacesArr = shape.pointNumFaces;
    const facesArr = shape.pointFaces;
    const planes = shape.planes;

    const requiredLength = numPoints * 3;
    while (out.points.length < requiredLength) {
        out.points.push(0);
    }
    out.numPoints = numPoints;
    out.convexRadius = scaledRadius;

    const sx = scale[0];
    const sy = scale[1];
    const sz = scale[2];
    const isx = 1 / sx;
    const isy = 1 / sy;
    const isz = 1 / sz;

    const dst = out.points;
    let w = 0;
    for (let pi = 0; pi < numPoints; pi++) {
        const pb = pi * 3;
        // scaled vertex position
        const px = positions[pb] * sx;
        const py = positions[pb + 1] * sy;
        const pz = positions[pb + 2] * sz;
        const numFaces = numFacesArr[pi];

        // first face-plane normal, transformed by inverse scale and renormalized
        const m1 = planes[facesArr[pb]].normal;
        let n1x = m1[0] * isx;
        let n1y = m1[1] * isy;
        let n1z = m1[2] * isz;
        let l1 = n1x * n1x + n1y * n1y + n1z * n1z;
        if (l1 > 0) {
            l1 = 1 / Math.sqrt(l1);
            n1x *= l1;
            n1y *= l1;
            n1z *= l1;
        }

        let rx: number;
        let ry: number;
        let rz: number;

        if (numFaces === 1) {
            rx = px - n1x * scaledRadius;
            ry = py - n1y * scaledRadius;
            rz = pz - n1z * scaledRadius;
        } else {
            const m2 = planes[facesArr[pb + 1]].normal;
            let n2x = m2[0] * isx;
            let n2y = m2[1] * isy;
            let n2z = m2[2] * isz;
            let l2 = n2x * n2x + n2y * n2y + n2z * n2z;
            if (l2 > 0) {
                l2 = 1 / Math.sqrt(l2);
                n2x *= l2;
                n2y *= l2;
                n2z *= l2;
            }

            // planes rebuilt through the scaled vertex, offset inward by the scaled convex radius
            const d1 = -(n1x * px + n1y * py + n1z * pz) + scaledRadius;
            const d2 = -(n2x * px + n2y * py + n2z * pz) + scaledRadius;

            let n3x: number;
            let n3y: number;
            let n3z: number;
            let d3: number;
            if (numFaces === 3) {
                const m3 = planes[facesArr[pb + 2]].normal;
                let a = m3[0] * isx;
                let b = m3[1] * isy;
                let c = m3[2] * isz;
                let l3 = a * a + b * b + c * c;
                if (l3 > 0) {
                    l3 = 1 / Math.sqrt(l3);
                    a *= l3;
                    b *= l3;
                    c *= l3;
                }
                n3x = a;
                n3y = b;
                n3z = c;
                d3 = -(n3x * px + n3y * py + n3z * pz) + scaledRadius;
            } else {
                // third plane perpendicular to the first two, through the scaled vertex (unnormalized normal)
                n3x = n1y * n2z - n1z * n2y;
                n3y = n1z * n2x - n1x * n2z;
                n3z = n1x * n2y - n1y * n2x;
                d3 = -(n3x * px + n3y * py + n3z * pz);
            }

            const c1x = n2y * n3z - n2z * n3y;
            const c1y = n2z * n3x - n2x * n3z;
            const c1z = n2x * n3y - n2y * n3x;
            const denom = n1x * c1x + n1y * c1y + n1z * c1z;

            if (Math.abs(denom) < 0.000001) {
                rx = px - n1x * scaledRadius;
                ry = py - n1y * scaledRadius;
                rz = pz - n1z * scaledRadius;
            } else {
                const c2x = n3y * n1z - n3z * n1y;
                const c2y = n3z * n1x - n3x * n1z;
                const c2z = n3x * n1y - n3y * n1x;
                const c3x = n1y * n2z - n1z * n2y;
                const c3y = n1z * n2x - n1x * n2z;
                const c3z = n1x * n2y - n1y * n2x;
                const s = -1 / denom;
                rx = (d1 * c1x + d2 * c2x + d3 * c3x) * s;
                ry = (d1 * c1y + d2 * c2y + d3 * c3y) * s;
                rz = (d1 * c1z + d2 * c2z + d3 * c3z) * s;
            }
        }

        dst[w++] = rx;
        dst[w++] = ry;
        dst[w++] = rz;
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
