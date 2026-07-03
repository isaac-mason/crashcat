import { type Box3, box3, mat4, type Quat, quat, raycast3, triangle3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import * as activeEdges from '../collision/active-edges';
import {
    type CastRayCollector,
    type CastRayHit,
    type CastRaySettings,
    CastRayStatus,
    createCastRayHit,
    createDefaultCastRaySettings,
} from '../collision/cast-ray-vs-shape';
import {
    type CastShapeCollector,
    type CastShapeSettings,
    CastShapeStatus,
    createCastShapeHit,
    reversedCastShapeVsShape,
} from '../collision/cast-shape-vs-shape';
import { INITIAL_EARLY_OUT_FRACTION, rayDistanceToBox3 } from '../collision/cast-utils';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import { createCollidePointHit } from '../collision/collide-point-vs-shape';
import {
    type CollideShapeCollector,
    type CollideShapeSettings,
    createCollideShapeHit,
    reversedCollideShapeVsShape,
} from '../collision/collide-shape-vs-shape';
import { createGjkCastShapeResult } from '../collision/gjk';
import {
    createPenetrationDepth,
    PenetrationDepthStatus,
    penetrationCastShape,
    penetrationDepthStepEPA,
    penetrationDepthStepGJK,
} from '../collision/penetration';
import { createSimplex } from '../collision/simplex';
import { FEATURE_TO_ACTIVE_EDGES, rayCylinder, raySphereFromOrigin } from '../collision/sphere-triangle';
import { createSupport, SupportFunctionMode, setTriangleSupport } from '../collision/support';
import { createClosestPointOnTriangleResult, getClosestPointOnTriangle } from '../collision/triangle';
import { assert } from '../utils/assert';
import { isScaleInsideOut, transformFaceWithMat4RotationTranslation, transformFaceWithMat4Scale } from '../utils/face';
import {
    type ConvexShape,
    defineShape,
    getShapeSupportingFace,
    type Shape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
    setShapeSupport,
    shapeDefs,
} from './shapes';
import type { SphereShape } from './sphere';
import * as bvh from './utils/bvh';
import { buildTriangleMesh } from './utils/triangle-mesh-builder';
import type { BvhSplitStrategy, TriangleMeshBVH } from './utils/triangle-mesh-bvh';
import * as triangleMeshBvh from './utils/triangle-mesh-bvh';
import type { TriangleMeshData } from './utils/triangle-mesh-data';
import * as triangleMeshData from './utils/triangle-mesh-data';
import { getActiveEdges, getTriangleNormal, getTriangleVertices } from './utils/triangle-mesh-data';

export { BvhSplitStrategy } from './utils/triangle-mesh-bvh';

export type TriangleMeshShapeSettings = {
    /** flat array of vertex positions [x1, y1, z1, x2, y2, z2, ...] */
    positions: number[];
    /** flat array of triangle vertex indices [i1, i2, i3, i4, i5, i6, ...] */
    indices: number[];
    /**
     * Optional per-triangle material indices.
     * Length should match number of triangles (indices.length / 3).
     * Default: -1 for all triangles (no material).
     */
    materialIndices?: number[];
    /** bvh split strategy */
    bvhSplitStrategy?: BvhSplitStrategy;
    /** maximum triangles per leaf node in the bvh */
    bvhMaxLeafTris?: number;
    /** degenerate tolerance for triangles */
    degenerateTolerance?: number;
    /**
     * cosine threshold for active edge determination.
     * edges with cos(dihedral_angle) >= this value are considered smooth/inactive.
     * default: cos(5°) = 0.996195
     * set to -1.0 to disable active edge determination (all edges active)
     * set to 1.0 to make all edges inactive
     */
    activeEdgeCosThresholdAngle?: number;
};

export const DEFAULT_TRIANGLE_MESH_OPTIONS = {
    bvhSplitStrategy: triangleMeshBvh.BvhSplitStrategy.CENTER,
    bvhMaxLeafTris: 8,
    degenerateTolerance: 1e-6,
    activeEdgeCosThresholdAngle: 0.996195, // cos(5°)
};

export type TriangleMeshShape = {
    type: ShapeType.TRIANGLE_MESH;
    bvh: TriangleMeshBVH;
    data: TriangleMeshData;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

export function create(o: TriangleMeshShapeSettings): TriangleMeshShape {
    const result = buildTriangleMesh({
        positions: o.positions,
        indices: o.indices,
        materialIds: o.materialIndices,
        bvhSplitStrategy: o.bvhSplitStrategy ?? DEFAULT_TRIANGLE_MESH_OPTIONS.bvhSplitStrategy,
        bvhMaxLeafTris: o.bvhMaxLeafTris ?? DEFAULT_TRIANGLE_MESH_OPTIONS.bvhMaxLeafTris,
        degenerateTolerance: o.degenerateTolerance ?? DEFAULT_TRIANGLE_MESH_OPTIONS.degenerateTolerance,
        activeEdgeCosThresholdAngle: o.activeEdgeCosThresholdAngle ?? DEFAULT_TRIANGLE_MESH_OPTIONS.activeEdgeCosThresholdAngle,
    });

    let aabb: Box3;
    if (result.bvh.buffer.length > 0) {
        aabb = box3.create();
        bvh.nodeGetBounds(aabb, result.bvh.buffer, 0);
    } else {
        aabb = box3.create();
    }

    const shape: TriangleMeshShape = {
        type: ShapeType.TRIANGLE_MESH,
        bvh: result.bvh,
        data: result.data,
        aabb,
        centerOfMass: [0, 0, 0],
        volume: 0,
    };

    return shape;
}

const _subShapeIdPopResult = /* @__PURE__ */ subShape.popResult();
const _getSurfaceNormal_normal = /* @__PURE__ */ vec3.create();
const _getSupportingFace_a = /* @__PURE__ */ vec3.create();
const _getSupportingFace_b = /* @__PURE__ */ vec3.create();
const _getSupportingFace_c = /* @__PURE__ */ vec3.create();

export const def = /* @__PURE__ */ (() =>
    defineShape<TriangleMeshShape>({
        type: ShapeType.TRIANGLE_MESH,
        category: ShapeCategory.MESH,
        computeMassProperties,
        getSurfaceNormal,
        getSupportingFace,
        castRay: castRayVsTriangleMesh,
        collidePoint: collidePointVsTriangleMesh,
        register: () => {
            for (const shapeDef of Object.values(shapeDefs)) {
                if (shapeDef.category === ShapeCategory.CONVEX) {
                    setCollideShapeFn(shapeDef.type, ShapeType.TRIANGLE_MESH, collideConvexVsTriangleMesh);
                    setCollideShapeFn(ShapeType.TRIANGLE_MESH, shapeDef.type, collideTriangleMeshVsConvex);
                    setCastShapeFn(shapeDef.type, ShapeType.TRIANGLE_MESH, castConvexVsTriangleMesh);
                    setCastShapeFn(ShapeType.TRIANGLE_MESH, shapeDef.type, castTriangleMeshVsConvex);
                }
            }

            // specialized collision and cast functions for sphere (2-4x faster than generic convex)
            setCollideShapeFn(ShapeType.SPHERE, ShapeType.TRIANGLE_MESH, collideSphereVsTriangleMesh);
            setCollideShapeFn(ShapeType.TRIANGLE_MESH, ShapeType.SPHERE, collideTriangleMeshVsSphere);
            setCastShapeFn(ShapeType.SPHERE, ShapeType.TRIANGLE_MESH, castSphereVsTriangleMesh);
            setCastShapeFn(ShapeType.TRIANGLE_MESH, ShapeType.SPHERE, castTriangleMeshVsSphere);
        },
    }))();

function computeMassProperties(out: MassProperties, _shape: TriangleMeshShape): void {
    // triangle meshes are static collision geometry by default.
    // mass properties should be overridden at the body level if needed.
    out.mass = 0;
    mat4.identity(out.inertia);
}

function getSurfaceNormal(ioResult: SurfaceNormalResult, shape: TriangleMeshShape, subShapeId: number): void {
    subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.data.triangleCount);
    const triangleIndex = _subShapeIdPopResult.value;

    // if triangleIndex is a valid triangle index, return that triangle's normal
    if (triangleIndex >= 0 && triangleIndex < shape.data.triangleCount) {
        getTriangleNormal(_getSurfaceNormal_normal, shape.data, triangleIndex);
        vec3.copy(ioResult.normal, _getSurfaceNormal_normal);
        return;
    }

    assert(false, 'Invalid SubShapeID for TriangleMeshShape');
}

function getSupportingFace(ioResult: SupportingFaceResult, _direction: Vec3, shape: TriangleMeshShape, subShapeId: number): void {
    const face = ioResult.face;
    const { transform, scale } = ioResult;

    // extract triangle index from SubShapeID
    subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.data.triangleCount);
    const triangleIndex = _subShapeIdPopResult.value;

    const a = _getSupportingFace_a;
    const b = _getSupportingFace_b;
    const c = _getSupportingFace_c;
    getTriangleVertices(a, b, c, shape.data, triangleIndex);

    // check if scale inverts winding (negative determinant)
    const insideOut = isScaleInsideOut(scale);

    // return the 3 vertices of the triangle
    face.numVertices = 3;

    if (insideOut) {
        // reverse winding: a,b,c -> c,b,a
        face.vertices[0] = c[0];
        face.vertices[1] = c[1];
        face.vertices[2] = c[2];
        face.vertices[3] = b[0];
        face.vertices[4] = b[1];
        face.vertices[5] = b[2];
        face.vertices[6] = a[0];
        face.vertices[7] = a[1];
        face.vertices[8] = a[2];
    } else {
        face.vertices[0] = a[0];
        face.vertices[1] = a[1];
        face.vertices[2] = a[2];
        face.vertices[3] = b[0];
        face.vertices[4] = b[1];
        face.vertices[5] = b[2];
        face.vertices[6] = c[0];
        face.vertices[7] = c[1];
        face.vertices[8] = c[2];
    }

    transformFaceWithMat4Scale(face, transform, scale);
}

/* cast ray */

const _castRayVsTriangleMesh_pos = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_quat = /* @__PURE__ */ quat.create();
const _castRayVsTriangleMesh_scale = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_rayOriginLocal = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_rayDirectionLocal = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_invQuat = /* @__PURE__ */ quat.create();
const _castRayVsTriangleMesh_mat4_WorldToB = /* @__PURE__ */ mat4.create();
const _castRayVsTriangleMesh_negPos = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_rayForMathcat = /* @__PURE__ */ raycast3.create();
const _castRayVsTriangleMesh_hitResult = /* @__PURE__ */ raycast3.createIntersectsTriangleResult();
// parallel flat stacks for this cast traversal: node offsets (SMI array) + distances (double
// array) sharing one size counter — jolt keeps the same structure (a distance stack parallel
// to the node stack). separate arrays so each keeps its optimal element kind.
const _castRayVsTriangleMesh_stackNodes: number[] = [];
const _castRayVsTriangleMesh_stackDist: number[] = [];
const _castRayVsTriangleMesh_leftBounds = /* @__PURE__ */ box3.create();
const _castRayVsTriangleMesh_rightBounds = /* @__PURE__ */ box3.create();
const _castRayVsTriangleMesh_a = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_b = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_c = /* @__PURE__ */ vec3.create();
const _castRayVsTriangleMesh_hit = /* @__PURE__ */ createCastRayHit();
const _castRayVsTriangleMesh_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

function castRayVsTriangleMesh(
    collector: CastRayCollector,
    settings: CastRaySettings,
    originX: number,
    originY: number,
    originZ: number,
    directionX: number,
    directionY: number,
    directionZ: number,
    length: number,
    shape: TriangleMeshShape,
    subShapeId: number,
    subShapeIdBits: number,
    posX: number,
    posY: number,
    posZ: number,
    quatX: number,
    quatY: number,
    quatZ: number,
    quatW: number,
    scaleX: number,
    scaleY: number,
    scaleZ: number,
): void {
    vec3.set(_castRayVsTriangleMesh_pos, posX, posY, posZ);
    quat.set(_castRayVsTriangleMesh_quat, quatX, quatY, quatZ, quatW);
    vec3.set(_castRayVsTriangleMesh_scale, scaleX, scaleY, scaleZ);

    // pre-compute world-to-B matrix (inverse of B's transform)
    // inverse transform: conjugate rotation, then negate-rotated position
    quat.conjugate(_castRayVsTriangleMesh_invQuat, _castRayVsTriangleMesh_quat);
    vec3.negate(_castRayVsTriangleMesh_negPos, _castRayVsTriangleMesh_pos);
    vec3.transformQuat(_castRayVsTriangleMesh_negPos, _castRayVsTriangleMesh_negPos, _castRayVsTriangleMesh_invQuat);
    mat4.fromRotationTranslation(
        _castRayVsTriangleMesh_mat4_WorldToB,
        _castRayVsTriangleMesh_invQuat,
        _castRayVsTriangleMesh_negPos,
    );

    // set ray origin from parameters
    vec3.set(_castRayVsTriangleMesh_rayOriginLocal, originX, originY, originZ);
    // transform ray from world space to mesh local space using pre-computed matrix
    vec3.transformMat4(
        _castRayVsTriangleMesh_rayOriginLocal,
        _castRayVsTriangleMesh_rayOriginLocal,
        _castRayVsTriangleMesh_mat4_WorldToB,
    );

    // set ray direction from parameters
    vec3.set(_castRayVsTriangleMesh_rayDirectionLocal, directionX, directionY, directionZ);
    mat4.multiply3x3Vec(
        _castRayVsTriangleMesh_rayDirectionLocal,
        _castRayVsTriangleMesh_mat4_WorldToB,
        _castRayVsTriangleMesh_rayDirectionLocal,
    );

    // handle scale by dividing ray direction components by scale
    // (scale affects how far we need to go in local space to reach world distance)
    _castRayVsTriangleMesh_rayDirectionLocal[0] /= _castRayVsTriangleMesh_scale[0];
    _castRayVsTriangleMesh_rayDirectionLocal[1] /= _castRayVsTriangleMesh_scale[1];
    _castRayVsTriangleMesh_rayDirectionLocal[2] /= _castRayVsTriangleMesh_scale[2];

    // set up scratch ray for mathcat api calls
    // safe to use module-level scratch because triangle meshes don't nest
    vec3.copy(_castRayVsTriangleMesh_rayForMathcat.origin, _castRayVsTriangleMesh_rayOriginLocal);
    vec3.copy(_castRayVsTriangleMesh_rayForMathcat.direction, _castRayVsTriangleMesh_rayDirectionLocal);
    _castRayVsTriangleMesh_rayForMathcat.length = length;

    const buffer = shape.bvh.buffer;
    const meshData = shape.data;

    if (buffer.length === 0) {
        collector.addMiss();
        return;
    }

    let foundHit = false;
    let stackSize = 0;
    _castRayVsTriangleMesh_stackNodes[stackSize] = 0;
    _castRayVsTriangleMesh_stackDist[stackSize] = -Infinity; // root always visited
    stackSize++;

    while (stackSize > 0) {
        // early out: very close hit
        if (collector.earlyOutFraction <= 0) {
            break;
        }

        stackSize--;
        const nodeOffset = _castRayVsTriangleMesh_stackNodes[stackSize];
        const nodeDistance = _castRayVsTriangleMesh_stackDist[stackSize];

        // early out: if fraction to this node >= closest hit, skip it
        if (nodeDistance >= collector.earlyOutFraction) {
            continue;
        }

        // note: no need to test ray x triangle bounds intersection here - we already proved the ray
        // intersects this node's bounds when we computed the distance during push.
        // if the ray didn't intersect, rayDistanceToBox3 would have returned Infinity
        // and we wouldn't have pushed this node onto the stack.

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf: check triangles
            const triStart = triangleMeshBvh.nodeTriStart(buffer, nodeOffset);
            const triCount = triangleMeshBvh.nodeTriCount(buffer, nodeOffset);
            for (let i = 0; i < triCount; i++) {
                const triangleIndex = triStart + i;

                // get triangle vertices from interleaved buffer
                getTriangleVertices(
                    _castRayVsTriangleMesh_a,
                    _castRayVsTriangleMesh_b,
                    _castRayVsTriangleMesh_c,
                    meshData,
                    triangleIndex,
                );

                // apply scale to vertices (mesh is in local space)
                const a = vec3.mul(_castRayVsTriangleMesh_a, _castRayVsTriangleMesh_a, _castRayVsTriangleMesh_scale);
                const b = vec3.mul(_castRayVsTriangleMesh_b, _castRayVsTriangleMesh_b, _castRayVsTriangleMesh_scale);
                const c = vec3.mul(_castRayVsTriangleMesh_c, _castRayVsTriangleMesh_c, _castRayVsTriangleMesh_scale);

                // note: we don't do a per-triangle aabb test, the bvh node aabb already provides tight culling
                // and the ray x triangle test is not so expensive as gjk/epa collision or shapecast.

                // test ray vs triangle
                raycast3.intersectsTriangle(
                    _castRayVsTriangleMesh_hitResult,
                    _castRayVsTriangleMesh_rayForMathcat,
                    a,
                    b,
                    c,
                    !settings.collideWithBackfaces,
                );

                if (
                    _castRayVsTriangleMesh_hitResult.hit &&
                    _castRayVsTriangleMesh_hitResult.fraction < collector.earlyOutFraction
                ) {
                    foundHit = true;

                    _castRayVsTriangleMesh_subShapeIdBuilder.value = subShapeId;
                    _castRayVsTriangleMesh_subShapeIdBuilder.currentBit = subShapeIdBits;
                    subShape.pushIndex(
                        _castRayVsTriangleMesh_subShapeIdBuilder,
                        _castRayVsTriangleMesh_subShapeIdBuilder,
                        triangleIndex,
                        meshData.triangleCount,
                    );

                    _castRayVsTriangleMesh_hit.status = CastRayStatus.COLLIDING;
                    _castRayVsTriangleMesh_hit.fraction = _castRayVsTriangleMesh_hitResult.fraction;
                    _castRayVsTriangleMesh_hit.subShapeId = _castRayVsTriangleMesh_subShapeIdBuilder.value;
                    _castRayVsTriangleMesh_hit.materialId = triangleMeshData.getMaterialId(meshData, triangleIndex);
                    _castRayVsTriangleMesh_hit.bodyIdB = collector.bodyIdB;
                    collector.addHit(_castRayVsTriangleMesh_hit);
                }
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            // push farther child first so closer is popped first
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);

            bvh.nodeGetBounds(_castRayVsTriangleMesh_leftBounds, buffer, leftOffset);
            bvh.nodeGetBounds(_castRayVsTriangleMesh_rightBounds, buffer, rightOffset);

            const leftDist = rayDistanceToBox3(
                _castRayVsTriangleMesh_rayForMathcat.origin[0],
                _castRayVsTriangleMesh_rayForMathcat.origin[1],
                _castRayVsTriangleMesh_rayForMathcat.origin[2],
                _castRayVsTriangleMesh_rayForMathcat.direction[0],
                _castRayVsTriangleMesh_rayForMathcat.direction[1],
                _castRayVsTriangleMesh_rayForMathcat.direction[2],
                _castRayVsTriangleMesh_rayForMathcat.length,
                _castRayVsTriangleMesh_leftBounds,
            );
            const rightDist = rayDistanceToBox3(
                _castRayVsTriangleMesh_rayForMathcat.origin[0],
                _castRayVsTriangleMesh_rayForMathcat.origin[1],
                _castRayVsTriangleMesh_rayForMathcat.origin[2],
                _castRayVsTriangleMesh_rayForMathcat.direction[0],
                _castRayVsTriangleMesh_rayForMathcat.direction[1],
                _castRayVsTriangleMesh_rayForMathcat.direction[2],
                _castRayVsTriangleMesh_rayForMathcat.length,
                _castRayVsTriangleMesh_rightBounds,
            );

            // push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                // left is closer or equal - push right first
                if (rightDist < collector.earlyOutFraction) {
                    _castRayVsTriangleMesh_stackNodes[stackSize] = rightOffset;
                    _castRayVsTriangleMesh_stackDist[stackSize] = rightDist;
                    stackSize++;
                }
                if (leftDist < collector.earlyOutFraction) {
                    _castRayVsTriangleMesh_stackNodes[stackSize] = leftOffset;
                    _castRayVsTriangleMesh_stackDist[stackSize] = leftDist;
                    stackSize++;
                }
            } else {
                // right is closer - push left first
                if (leftDist < collector.earlyOutFraction) {
                    _castRayVsTriangleMesh_stackNodes[stackSize] = leftOffset;
                    _castRayVsTriangleMesh_stackDist[stackSize] = leftDist;
                    stackSize++;
                }
                if (rightDist < collector.earlyOutFraction) {
                    _castRayVsTriangleMesh_stackNodes[stackSize] = rightOffset;
                    _castRayVsTriangleMesh_stackDist[stackSize] = rightDist;
                    stackSize++;
                }
            }
        }
    }

    if (!foundHit) {
        collector.addMiss();
    }
}

/* collide point */

const hitCountCollector = {
    bodyIdB: -1,
    earlyOutFraction: INITIAL_EARLY_OUT_FRACTION,
    hitCount: 0,
    lastSubShapeId: EMPTY_SUB_SHAPE_ID,

    addHit(hit: CastRayHit): void {
        this.hitCount++;
        this.lastSubShapeId = hit.subShapeId;
    },

    addMiss(): void {
        // no-op
    },

    shouldEarlyOut(): boolean {
        return false; // never early out, count all hits
    },

    reset(): void {
        this.bodyIdB = -1;
        this.hitCount = 0;
        this.lastSubShapeId = EMPTY_SUB_SHAPE_ID;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    },
};

const _collidePointVsTriangleMesh_castRaySettings = /* @__PURE__ */ (() => {
    const s = createDefaultCastRaySettings();
    s.collideWithBackfaces = true; // backface collision required for odd-even rule
    return s;
})();

const _collidePointVsTriangleMesh_quatB = /* @__PURE__ */ quat.create();
const _collidePointVsTriangleMesh_localPoint = /* @__PURE__ */ vec3.create();
const _collidePointVsTriangleMesh_ray = /* @__PURE__ */ raycast3.create();
const _collidePointVsTriangleMesh_rayDirection = /* @__PURE__ */ vec3.create();
const _collidePointVsTriangleMesh_aabbSize = /* @__PURE__ */ vec3.create();
const _collidePointVsTriangleMesh_popResult = /* @__PURE__ */ subShape.popResult();
const _collidePointHit = /* @__PURE__ */ createCollidePointHit();

function collidePointVsTriangleMesh(
    collector: CollidePointCollector,
    _settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: TriangleMeshShape,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
): void {
    // transform point to mesh's local space
    const localX = pointX - posBX;
    const localY = pointY - posBY;
    const localZ = pointZ - posBZ;

    // apply inverse rotation
    quat.set(_collidePointVsTriangleMesh_quatB, quatBX, quatBY, quatBZ, quatBW);
    quat.conjugate(_collidePointVsTriangleMesh_quatB, _collidePointVsTriangleMesh_quatB);
    vec3.set(_collidePointVsTriangleMesh_localPoint, localX, localY, localZ);
    vec3.transformQuat(
        _collidePointVsTriangleMesh_localPoint,
        _collidePointVsTriangleMesh_localPoint,
        _collidePointVsTriangleMesh_quatB,
    );

    // apply inverse scale
    const invScaleX = 1.0 / scaleBX;
    const invScaleY = 1.0 / scaleBY;
    const invScaleZ = 1.0 / scaleBZ;
    _collidePointVsTriangleMesh_localPoint[0] *= invScaleX;
    _collidePointVsTriangleMesh_localPoint[1] *= invScaleY;
    _collidePointVsTriangleMesh_localPoint[2] *= invScaleZ;

    // early exit if point is outside AABB (point is already in unscaled local space)
    if (!box3.containsPoint(shapeB.aabb, _collidePointVsTriangleMesh_localPoint)) {
        return;
    }

    // construct ray from point in +Y direction
    // ray length = 10% longer than AABB height to ensure we exit the mesh
    box3.size(_collidePointVsTriangleMesh_aabbSize, shapeB.aabb);
    const aabbHeight = _collidePointVsTriangleMesh_aabbSize[1];
    const rayLength = aabbHeight * 1.1;

    vec3.set(_collidePointVsTriangleMesh_rayDirection, 0, 1, 0); // +Y axis
    raycast3.set(
        _collidePointVsTriangleMesh_ray,
        _collidePointVsTriangleMesh_localPoint,
        _collidePointVsTriangleMesh_rayDirection,
        rayLength,
    );

    // reset hit counter and perform raycast
    hitCountCollector.reset();
    hitCountCollector.bodyIdB = collector.bodyIdB;

    // cast ray through mesh, counting all triangle intersections
    // pass ray components directly from the local point and direction
    // biome-ignore format: readability
    castRayVsTriangleMesh(
        hitCountCollector,
        _collidePointVsTriangleMesh_castRaySettings,
        _collidePointVsTriangleMesh_localPoint[0],
        _collidePointVsTriangleMesh_localPoint[1],
        _collidePointVsTriangleMesh_localPoint[2],
        0, 1, 0,    // direction: +Y axis
        rayLength,
        shapeB,
        subShapeIdB,
        subShapeIdBitsB,
        0, 0, 0,    // mesh is already in local space
        0, 0, 0, 1, // identity rotation
        1, 1, 1,    // no additional scale
    );

    // apply odd-even rule (Jordan curve theorem)
    // odd number of hits = point is inside
    // even number of hits = point is outside
    const isInside = (hitCountCollector.hitCount & 1) !== 0;

    if (isInside) {
        _collidePointHit.subShapeIdB = hitCountCollector.lastSubShapeId;

        // extract triangle index from subShapeId to get material
        subShape.popIndex(_collidePointVsTriangleMesh_popResult, hitCountCollector.lastSubShapeId, shapeB.data.triangleCount);
        _collidePointHit.materialId = triangleMeshData.getMaterialId(shapeB.data, _collidePointVsTriangleMesh_popResult.value);

        _collidePointHit.bodyIdB = collector.bodyIdB;
        collector.addHit(_collidePointHit);
    }
}

/* cast shape */

const _castConvexVsTriangleMesh_castShapeHit = /* @__PURE__ */ createCastShapeHit();
const _castConvexVsTriangleMesh_displacementInB = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_triangleSupport = /* @__PURE__ */ createSupport();
const _castConvexVsTriangleMesh_sweptAABB: Box3 = /* @__PURE__ */ box3.create();

const _castConvexVsTriangleMesh_gjkResult = /* @__PURE__ */ createGjkCastShapeResult();

const _castConvexVsTriangleMesh_worldPointA = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_worldPointB = /* @__PURE__ */ vec3.create();

const _castConvexVsTriangleMesh_inverseQuaternionB = /* @__PURE__ */ quat.create();

const _castConvexVsTriangleMesh_faceNormal = /* @__PURE__ */ vec3.create();

const _castConvexVsTriangleMesh_activeEdgeMovementDirection = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_triangleNormalForFix = /* @__PURE__ */ vec3.create();

const _castConvexVsTriangleMesh_triangleA = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_triangleB = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_triangleC = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_getTriangleVertices_a = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_getTriangleVertices_b = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_getTriangleVertices_c = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_edgeA = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_edgeB = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_triangleNormal = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_penetrationDifference = /* @__PURE__ */ vec3.create();

const _castConvexVsTriangleMesh_posA = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_quatA = /* @__PURE__ */ quat.create();
const _castConvexVsTriangleMesh_scaleA = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_displacementA = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_posB = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_quatB = /* @__PURE__ */ quat.create();
const _castConvexVsTriangleMesh_scaleB = /* @__PURE__ */ vec3.create();

const _castConvexVsTriangleMesh_BtoWorld = /* @__PURE__ */ mat4.create();
const _castConvexVsTriangleMesh_AtoB = /* @__PURE__ */ mat4.create();
const _castConvexVsTriangleMesh_AtoWorld = /* @__PURE__ */ mat4.create();
const _castConvexVsTriangleMesh_invBtoWorld = /* @__PURE__ */ mat4.create();
const _castConvexVsTriangleMesh_AtoWorldAtContact = /* @__PURE__ */ mat4.create();

// parallel flat stacks for this cast traversal: node offsets (SMI array) + distances (double
// array) sharing one size counter — jolt keeps the same structure (a distance stack parallel
// to the node stack). separate arrays so each keeps its optimal element kind.
const _castConvexVsTriangleMesh_stackNodes: number[] = [];
const _castConvexVsTriangleMesh_stackDist: number[] = [];
const _castConvexVsTriangleMesh_raycast = /* @__PURE__ */ raycast3.create();
const _castConvexVsTriangleMesh_halfExtents = /* @__PURE__ */ vec3.create();
const _castConvexVsTriangleMesh_expandedBounds = /* @__PURE__ */ box3.create();
const _castConvexVsTriangleMesh_triExpandedBounds = /* @__PURE__ */ box3.create();

const _castConvexVsTriangleMesh_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

const _castConvexVsTriangleMesh_supportA = /* @__PURE__ */ createSupport();

function castConvexVsTriangleMesh(
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    _subShapeIdBitsA: number,
    posAX: number,
    posAY: number,
    posAZ: number,
    quatAX: number,
    quatAY: number,
    quatAZ: number,
    quatAW: number,
    scaleAX: number,
    scaleAY: number,
    scaleAZ: number,
    displacementAX: number,
    displacementAY: number,
    displacementAZ: number,
    shapeB: Shape,
    _subShapeIdB: number,
    _subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
): void {
    const meshShape = shapeB as TriangleMeshShape;

    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    // early out if buffer is empty (no triangles)
    if (buffer.length === 0) {
        return;
    }

    vec3.set(_castConvexVsTriangleMesh_posA, posAX, posAY, posAZ);
    quat.set(_castConvexVsTriangleMesh_quatA, quatAX, quatAY, quatAZ, quatAW);
    vec3.set(_castConvexVsTriangleMesh_scaleA, scaleAX, scaleAY, scaleAZ);
    vec3.set(_castConvexVsTriangleMesh_displacementA, displacementAX, displacementAY, displacementAZ);
    vec3.set(_castConvexVsTriangleMesh_posB, posBX, posBY, posBZ);
    quat.set(_castConvexVsTriangleMesh_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_castConvexVsTriangleMesh_scaleB, scaleBX, scaleBY, scaleBZ);

    // transform A into B's local space (aligned with castConvexVsConvex pattern)
    // transformA = A's transform in world space
    const transformA = mat4.fromRotationTranslationScale(
        _castConvexVsTriangleMesh_AtoWorld,
        _castConvexVsTriangleMesh_quatA,
        _castConvexVsTriangleMesh_posA,
        _castConvexVsTriangleMesh_scaleA,
    );

    // targetTransform = B's transform in world space
    const targetTransform = mat4.fromRotationTranslation(
        _castConvexVsTriangleMesh_BtoWorld,
        _castConvexVsTriangleMesh_quatB,
        _castConvexVsTriangleMesh_posB,
    );

    // castTransform = targetTransform^-1 * transformA (A's transform in B's space)
    mat4.invert(_castConvexVsTriangleMesh_invBtoWorld, targetTransform);
    const castTransform = mat4.multiply(_castConvexVsTriangleMesh_AtoB, _castConvexVsTriangleMesh_invBtoWorld, transformA);

    // transform displacement to B's space using the inverse transform's rotation (3x3)
    // _castConvexVsTriangleMesh_invBtoWorld is already B's inverse transform
    mat4.multiply3x3Vec(
        _castConvexVsTriangleMesh_displacementInB,
        _castConvexVsTriangleMesh_invBtoWorld,
        _castConvexVsTriangleMesh_displacementA,
    );

    // compute base AABB of shape A at t=0 in mesh local space
    box3.transformMat4(_castConvexVsTriangleMesh_sweptAABB, shapeA.aabb, castTransform);

    // determine if we want to use the actual shape or a shrunken shape with convex radius
    const supportMode = settings.useShrunkenShapeAndConvexRadius
        ? SupportFunctionMode.EXCLUDE_CONVEX_RADIUS
        : SupportFunctionMode.DEFAULT;

    // get transformed support function for convex shape
    // castTransform (passed to penetrationCastShape) folds A into B's space internally
    const supportA = _castConvexVsTriangleMesh_supportA;
    setShapeSupport(supportA, shapeA, supportMode, _castConvexVsTriangleMesh_scaleA);

    // determine if shape is inside out or not
    const scaleSign = vec3.isScaleInsideOut(_castConvexVsTriangleMesh_scaleB) ? -1 : 1;

    // targetTransform is already B-to-world from above
    const mat4_BtoWorld = targetTransform;

    // compute centroid of base AABB
    const ray = _castConvexVsTriangleMesh_raycast;
    ray.origin[0] = (_castConvexVsTriangleMesh_sweptAABB[0] + _castConvexVsTriangleMesh_sweptAABB[3]) * 0.5;
    ray.origin[1] = (_castConvexVsTriangleMesh_sweptAABB[1] + _castConvexVsTriangleMesh_sweptAABB[4]) * 0.5;
    ray.origin[2] = (_castConvexVsTriangleMesh_sweptAABB[2] + _castConvexVsTriangleMesh_sweptAABB[5]) * 0.5;

    // compute ray direction and length from displacement
    ray.length = vec3.length(_castConvexVsTriangleMesh_displacementInB);
    if (ray.length > 1e-10) {
        vec3.normalize(ray.direction, _castConvexVsTriangleMesh_displacementInB);
    } else {
        ray.direction[0] = 0;
        ray.direction[1] = 0;
        ray.direction[2] = 0;
    }

    // compute half-extents of the base AABB
    const halfExtents = _castConvexVsTriangleMesh_halfExtents;
    halfExtents[0] = (_castConvexVsTriangleMesh_sweptAABB[3] - _castConvexVsTriangleMesh_sweptAABB[0]) * 0.5;
    halfExtents[1] = (_castConvexVsTriangleMesh_sweptAABB[4] - _castConvexVsTriangleMesh_sweptAABB[1]) * 0.5;
    halfExtents[2] = (_castConvexVsTriangleMesh_sweptAABB[5] - _castConvexVsTriangleMesh_sweptAABB[2]) * 0.5;

    let stackSize = 0;
    _castConvexVsTriangleMesh_stackNodes[stackSize] = 0;
    _castConvexVsTriangleMesh_stackDist[stackSize] = 0; // root always visited
    stackSize++;

    const expandedBounds = _castConvexVsTriangleMesh_expandedBounds;

    while (stackSize > 0) {
        stackSize--;
        const nodeOffset = _castConvexVsTriangleMesh_stackNodes[stackSize];
        const nodeDistance = _castConvexVsTriangleMesh_stackDist[stackSize];

        // early out: if fraction to this node >= closest hit, skip it
        if (nodeDistance >= collector.earlyOutFraction) {
            continue;
        }

        // note: no need to test ray intersection here - we already proved the ray
        // intersects this node's expanded bounds when we computed the distance during push.
        // if the ray didn't intersect, rayDistanceToBox3 would have returned Infinity
        // and we wouldn't have pushed this node onto the stack.

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf: test triangles
            const triStart = triangleMeshBvh.nodeTriStart(buffer, nodeOffset);
            const triCount = triangleMeshBvh.nodeTriCount(buffer, nodeOffset);
            const triangleAABBs = meshData.triangleAABBs;

            for (let i = 0; i < triCount; i++) {
                const triangleIndex = triStart + i;

                // triangle aabb (precomputed at build) expanded by the cast shape's half-extents
                const aabbOffset = triangleIndex * 6;
                const triBounds = _castConvexVsTriangleMesh_triExpandedBounds;
                triBounds[0] = triangleAABBs[aabbOffset] - halfExtents[0];
                triBounds[1] = triangleAABBs[aabbOffset + 1] - halfExtents[1];
                triBounds[2] = triangleAABBs[aabbOffset + 2] - halfExtents[2];
                triBounds[3] = triangleAABBs[aabbOffset + 3] + halfExtents[0];
                triBounds[4] = triangleAABBs[aabbOffset + 4] + halfExtents[1];
                triBounds[5] = triangleAABBs[aabbOffset + 5] + halfExtents[2];

                // early out: ray x triangle expanded bounds
                if (!raycast3.intersectsBox3(ray, triBounds)) {
                    continue;
                }

                // early out if we've found a very close hit
                if (collector.earlyOutFraction <= 0) {
                    return;
                }

                // get triangle vertices
                getTriangleVertices(
                    _castConvexVsTriangleMesh_getTriangleVertices_a,
                    _castConvexVsTriangleMesh_getTriangleVertices_b,
                    _castConvexVsTriangleMesh_getTriangleVertices_c,
                    meshData,
                    triangleIndex,
                );

                // scale triangle
                const a = vec3.mul(
                    _castConvexVsTriangleMesh_triangleA,
                    _castConvexVsTriangleMesh_getTriangleVertices_a,
                    _castConvexVsTriangleMesh_scaleB,
                );
                const b = vec3.mul(
                    _castConvexVsTriangleMesh_triangleB,
                    _castConvexVsTriangleMesh_getTriangleVertices_b,
                    _castConvexVsTriangleMesh_scaleB,
                );
                const c = vec3.mul(
                    _castConvexVsTriangleMesh_triangleC,
                    _castConvexVsTriangleMesh_getTriangleVertices_c,
                    _castConvexVsTriangleMesh_scaleB,
                );

                // calculate scaled triangle normal
                const normal = vec3.scale(
                    _castConvexVsTriangleMesh_triangleNormal,
                    vec3.cross(
                        _castConvexVsTriangleMesh_triangleNormal,
                        vec3.sub(_castConvexVsTriangleMesh_edgeA, b, a),
                        vec3.sub(_castConvexVsTriangleMesh_edgeB, c, a),
                    ),
                    scaleSign,
                );

                // backface check
                if (!settings.collideWithBackfaces && vec3.dot(normal, _castConvexVsTriangleMesh_displacementInB) > 0) {
                    continue;
                }

                // set triangle support function
                setTriangleSupport(_castConvexVsTriangleMesh_triangleSupport, a, b, c);

                // gjk shapecast with epa fallback for deep penetration
                // castTransform already contains A in B's local space
                penetrationCastShape(
                    _castConvexVsTriangleMesh_gjkResult,
                    castTransform,
                    supportA,
                    _castConvexVsTriangleMesh_triangleSupport,
                    _castConvexVsTriangleMesh_displacementInB,
                    settings.collisionTolerance,
                    settings.penetrationTolerance,
                    supportA.convexRadius,
                    0, // triangle has no convex radius
                    collector.earlyOutFraction,
                    settings.returnDeepestPoint,
                );

                // check if hit found
                if (!_castConvexVsTriangleMesh_gjkResult.hit) {
                    continue;
                }

                const fraction = _castConvexVsTriangleMesh_gjkResult.lambda;
                const penetrationDepth = vec3.length(
                    vec3.sub(
                        _castConvexVsTriangleMesh_penetrationDifference,
                        _castConvexVsTriangleMesh_gjkResult.pointA,
                        _castConvexVsTriangleMesh_gjkResult.pointB,
                    ),
                );

                // early out: if this hit is deeper than the collector's early out value
                if (fraction === 0 && -penetrationDepth >= collector.earlyOutFraction) {
                    continue;
                }

                // active edge detection - correct normal if hitting inactive edge
                const triangleActiveEdges = getActiveEdges(meshData, triangleIndex);
                if (settings.collideOnlyWithActiveEdges && triangleActiveEdges !== 0b111) {
                    // transform movement direction from world space to mesh's local space (reuse pre-computed inverse)
                    vec3.transformQuat(
                        _castConvexVsTriangleMesh_activeEdgeMovementDirection,
                        settings.activeEdgeMovementDirection,
                        _castConvexVsTriangleMesh_inverseQuaternionB,
                    );

                    // prepare triangle normal for fixNormal
                    // back-facing check: if displacement dot normal > 0, we're approaching from back
                    const backFacing =
                        !settings.collideWithBackfaces || vec3.dot(normal, _castConvexVsTriangleMesh_displacementInB) > 0;
                    if (backFacing) {
                        vec3.copy(_castConvexVsTriangleMesh_triangleNormalForFix, normal);
                    } else {
                        vec3.negate(_castConvexVsTriangleMesh_triangleNormalForFix, normal);
                    }

                    // apply active edge correction (all parameters in mesh's local space)
                    const correctedAxis = activeEdges.fixNormal(
                        a,
                        b,
                        c,
                        _castConvexVsTriangleMesh_triangleNormalForFix,
                        triangleActiveEdges,
                        _castConvexVsTriangleMesh_gjkResult.pointB,
                        _castConvexVsTriangleMesh_gjkResult.separatingAxis,
                        _castConvexVsTriangleMesh_activeEdgeMovementDirection,
                    );

                    vec3.copy(_castConvexVsTriangleMesh_gjkResult.separatingAxis, correctedAxis);
                }

                // convert to world space using pre-computed matrix
                vec3.transformMat4(
                    _castConvexVsTriangleMesh_worldPointA,
                    _castConvexVsTriangleMesh_gjkResult.pointA,
                    mat4_BtoWorld,
                );
                vec3.transformMat4(
                    _castConvexVsTriangleMesh_worldPointB,
                    _castConvexVsTriangleMesh_gjkResult.pointB,
                    mat4_BtoWorld,
                );
                // transform direction (penetration axis) without translation
                mat4.multiply3x3Vec(
                    _castConvexVsTriangleMesh_castShapeHit.penetrationAxis,
                    mat4_BtoWorld,
                    _castConvexVsTriangleMesh_gjkResult.separatingAxis,
                );

                // store hit info
                _castConvexVsTriangleMesh_castShapeHit.status = CastShapeStatus.COLLIDING;
                _castConvexVsTriangleMesh_castShapeHit.fraction = _castConvexVsTriangleMesh_gjkResult.lambda;
                vec3.copy(_castConvexVsTriangleMesh_castShapeHit.pointA, _castConvexVsTriangleMesh_worldPointA);
                vec3.copy(_castConvexVsTriangleMesh_castShapeHit.pointB, _castConvexVsTriangleMesh_worldPointB);
                vec3.normalize(
                    _castConvexVsTriangleMesh_castShapeHit.normal,
                    _castConvexVsTriangleMesh_castShapeHit.penetrationAxis,
                );
                vec3.negate(_castConvexVsTriangleMesh_castShapeHit.normal, _castConvexVsTriangleMesh_castShapeHit.normal);
                _castConvexVsTriangleMesh_castShapeHit.penetrationDepth = penetrationDepth;
                _castConvexVsTriangleMesh_castShapeHit.subShapeIdA = subShapeIdA;

                _castConvexVsTriangleMesh_subShapeIdBuilder.value = _subShapeIdB;
                _castConvexVsTriangleMesh_subShapeIdBuilder.currentBit = _subShapeIdBitsB;
                subShape.pushIndex(
                    _castConvexVsTriangleMesh_subShapeIdBuilder,
                    _castConvexVsTriangleMesh_subShapeIdBuilder,
                    triangleIndex,
                    meshData.triangleCount,
                );

                _castConvexVsTriangleMesh_castShapeHit.subShapeIdB = _castConvexVsTriangleMesh_subShapeIdBuilder.value;
                _castConvexVsTriangleMesh_castShapeHit.materialIdA = (shapeA as ConvexShape).materialId;
                _castConvexVsTriangleMesh_castShapeHit.materialIdB = triangleMeshData.getMaterialId(meshData, triangleIndex);
                _castConvexVsTriangleMesh_castShapeHit.bodyIdB = collector.bodyIdB;

                // gather faces if requested
                if (settings.collectFaces) {
                    // face a: transform contact normal from B's local space to A's local space
                    // using transposed 3x3 of castTransform (inverse rotation)
                    const normalInA = vec3.negate(
                        _castConvexVsTriangleMesh_faceNormal,
                        _castConvexVsTriangleMesh_gjkResult.separatingAxis,
                    );
                    mat4.multiply3x3TransposedVec(normalInA, castTransform, normalInA);

                    // calculate transform for shape A at contact point (aligned with castConvexVsConvex)
                    // castTransform with translation += fraction * displacementInB
                    mat4.copy(_castConvexVsTriangleMesh_AtoWorldAtContact, castTransform);
                    _castConvexVsTriangleMesh_AtoWorldAtContact[12] +=
                        _castConvexVsTriangleMesh_gjkResult.lambda * _castConvexVsTriangleMesh_displacementInB[0];
                    _castConvexVsTriangleMesh_AtoWorldAtContact[13] +=
                        _castConvexVsTriangleMesh_gjkResult.lambda * _castConvexVsTriangleMesh_displacementInB[1];
                    _castConvexVsTriangleMesh_AtoWorldAtContact[14] +=
                        _castConvexVsTriangleMesh_gjkResult.lambda * _castConvexVsTriangleMesh_displacementInB[2];

                    // transform to world space: targetTransform * transformAAtContact
                    mat4.multiply(
                        _castConvexVsTriangleMesh_AtoWorldAtContact,
                        targetTransform,
                        _castConvexVsTriangleMesh_AtoWorldAtContact,
                    );

                    getShapeSupportingFace(
                        _castConvexVsTriangleMesh_castShapeHit.faceA,
                        shapeA,
                        subShapeIdA,
                        normalInA,
                        _castConvexVsTriangleMesh_AtoWorldAtContact,
                        _castConvexVsTriangleMesh_scaleA,
                    );

                    // face b: supporting face is the triangle vertices (already scaled and in B's local space)
                    _castConvexVsTriangleMesh_castShapeHit.faceB.numVertices = 3;
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[0] = a[0];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[1] = a[1];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[2] = a[2];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[3] = b[0];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[4] = b[1];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[5] = b[2];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[6] = c[0];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[7] = c[1];
                    _castConvexVsTriangleMesh_castShapeHit.faceB.vertices[8] = c[2];

                    // transform to world space (rotation + translation only, no scale - vertices are already scaled)
                    transformFaceWithMat4RotationTranslation(_castConvexVsTriangleMesh_castShapeHit.faceB, mat4_BtoWorld);
                } else {
                    // clear faces
                    _castConvexVsTriangleMesh_castShapeHit.faceA.numVertices = 0;
                    _castConvexVsTriangleMesh_castShapeHit.faceB.numVertices = 0;
                }

                // add hit
                collector.addHit(_castConvexVsTriangleMesh_castShapeHit);
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            // ray from swept AABB center along displacement
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);

            // expanded bounds for left child
            expandedBounds[0] = buffer[leftOffset + bvh.NODE_MIN_X] - halfExtents[0];
            expandedBounds[1] = buffer[leftOffset + bvh.NODE_MIN_Y] - halfExtents[1];
            expandedBounds[2] = buffer[leftOffset + bvh.NODE_MIN_Z] - halfExtents[2];
            expandedBounds[3] = buffer[leftOffset + bvh.NODE_MAX_X] + halfExtents[0];
            expandedBounds[4] = buffer[leftOffset + bvh.NODE_MAX_Y] + halfExtents[1];
            expandedBounds[5] = buffer[leftOffset + bvh.NODE_MAX_Z] + halfExtents[2];

            // get distance
            const leftDist = rayDistanceToBox3(
                ray.origin[0],
                ray.origin[1],
                ray.origin[2],
                ray.direction[0],
                ray.direction[1],
                ray.direction[2],
                ray.length,
                expandedBounds,
            );

            // expanded bounds for right child
            expandedBounds[0] = buffer[rightOffset + bvh.NODE_MIN_X] - halfExtents[0];
            expandedBounds[1] = buffer[rightOffset + bvh.NODE_MIN_Y] - halfExtents[1];
            expandedBounds[2] = buffer[rightOffset + bvh.NODE_MIN_Z] - halfExtents[2];
            expandedBounds[3] = buffer[rightOffset + bvh.NODE_MAX_X] + halfExtents[0];
            expandedBounds[4] = buffer[rightOffset + bvh.NODE_MAX_Y] + halfExtents[1];
            expandedBounds[5] = buffer[rightOffset + bvh.NODE_MAX_Z] + halfExtents[2];

            // get distance
            const rightDist = rayDistanceToBox3(
                ray.origin[0],
                ray.origin[1],
                ray.origin[2],
                ray.direction[0],
                ray.direction[1],
                ray.direction[2],
                ray.length,
                expandedBounds,
            );

            // sort: push farther child first (so closer child is on top of stack), lifo traversal
            if (leftDist <= rightDist) {
                // left is closer or equal - push right first
                if (rightDist < collector.earlyOutFraction) {
                    _castConvexVsTriangleMesh_stackNodes[stackSize] = rightOffset;
                    _castConvexVsTriangleMesh_stackDist[stackSize] = rightDist;
                    stackSize++;
                }
                if (leftDist < collector.earlyOutFraction) {
                    _castConvexVsTriangleMesh_stackNodes[stackSize] = leftOffset;
                    _castConvexVsTriangleMesh_stackDist[stackSize] = leftDist;
                    stackSize++;
                }
            } else {
                // right is closer - push left first
                if (leftDist < collector.earlyOutFraction) {
                    _castConvexVsTriangleMesh_stackNodes[stackSize] = leftOffset;
                    _castConvexVsTriangleMesh_stackDist[stackSize] = leftDist;
                    stackSize++;
                }
                if (rightDist < collector.earlyOutFraction) {
                    _castConvexVsTriangleMesh_stackNodes[stackSize] = rightOffset;
                    _castConvexVsTriangleMesh_stackDist[stackSize] = rightDist;
                    stackSize++;
                }
            }
        }
    }
}

const castTriangleMeshVsConvex = /* @__PURE__ */ reversedCastShapeVsShape(castConvexVsTriangleMesh);

/* collide shape */

const _collideConvexVsTriangleMesh_collideShapeHit = /* @__PURE__ */ createCollideShapeHit();

const _collideConvexVsTriangleMesh_temp_faceDirA = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_simplex = /* @__PURE__ */ createSimplex();
const _collideConvexVsTriangleMesh_penetrationDepth = /* @__PURE__ */ createPenetrationDepth();

const _collideConvexVsTriangleMesh_supportA = /* @__PURE__ */ createSupport();

// separate struct for the EPA-inflated A, so `supportA` stays EXCLUDE across the BVH triangle loop
const _collideConvexVsTriangleMesh_supportAWithRadius = /* @__PURE__ */ createSupport();

const _collideConvexVsTriangleMesh_penetrationAxis = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_vectorAB = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_inverseQuatA = /* @__PURE__ */ quat.create();

const _collideConvexVsTriangleMesh_aabbShapeExpand = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_inverseQuatB = /* @__PURE__ */ quat.create();
const _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2 = /* @__PURE__ */ box3.create();
const _collideConvexVsTriangleMesh_boundsOf1 = /* @__PURE__ */ box3.create();
const _collideConvexVsTriangleMesh_transform2To1Pos = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_transform2To1Quat = /* @__PURE__ */ quat.create();

const _collideConvexVsTriangleMesh_triangleA_inA = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_triangleB_inA = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_triangleC_inA = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_triangleA = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_triangleB = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_triangleC = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_getTriangleVertices_a = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_getTriangleVertices_b = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_getTriangleVertices_c = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_edgeA = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_edgeB = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_triangleNormal = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_posA = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_quatA = /* @__PURE__ */ quat.create();
const _collideConvexVsTriangleMesh_scaleA = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_posB = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_quatB = /* @__PURE__ */ quat.create();
const _collideConvexVsTriangleMesh_scaleB = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_aabbTransform = /* @__PURE__ */ mat4.create();

const _collideConvexVsTriangleMesh_mat4_BtoA = /* @__PURE__ */ mat4.create();
const _collideConvexVsTriangleMesh_mat4_AtoWorld = /* @__PURE__ */ mat4.create();

const _collideConvexVsTriangleMesh_activeEdgeMovementDirection = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_triangleNormalForFix = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_conjugateQuat = /* @__PURE__ */ quat.create();

const _collideConvexVsTriangleMesh_worldPointA = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_worldPointB = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

const _collideConvexVsTriangleMesh_posAInB = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_quatAInB = /* @__PURE__ */ quat.create();
const _collideConvexVsTriangleMesh_positionDifference = /* @__PURE__ */ vec3.create();

const _collideConvexVsTriangleMesh_triangleSupport = /* @__PURE__ */ createSupport();

// flat SMI stack of node offsets for this shape-query traversal — children are still pushed
// farther-first (sorted by distance) but the sort distance itself isn't stored. grow-once with
// a size counter (no pop()/length churn, stays packed).
const _collideConvexVsTriangleMesh_stackNodes: number[] = [];
const _collideConvexVsTriangleMesh_queryCenter = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_nodeCenter = /* @__PURE__ */ vec3.create();
const _collideConvexVsTriangleMesh_triangleAABB = /* @__PURE__ */ box3.create();

function collideConvexVsTriangleMesh(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    _subShapeIdBitsA: number,
    posAX: number,
    posAY: number,
    posAZ: number,
    quatAX: number,
    quatAY: number,
    quatAZ: number,
    quatAW: number,
    scaleAX: number,
    scaleAY: number,
    scaleAZ: number,
    shapeB: Shape,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
): void {
    const meshShape = shapeB as TriangleMeshShape;

    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    if (buffer.length === 0) {
        return;
    }

    vec3.set(_collideConvexVsTriangleMesh_posA, posAX, posAY, posAZ);
    quat.set(_collideConvexVsTriangleMesh_quatA, quatAX, quatAY, quatAZ, quatAW);
    vec3.set(_collideConvexVsTriangleMesh_scaleA, scaleAX, scaleAY, scaleAZ);

    vec3.set(_collideConvexVsTriangleMesh_posB, posBX, posBY, posBZ);
    quat.set(_collideConvexVsTriangleMesh_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_collideConvexVsTriangleMesh_scaleB, scaleBX, scaleBY, scaleBZ);

    // calculate transforms
    // transform B (mesh) into A's (convex) local space for triangle tests
    quat.conjugate(_collideConvexVsTriangleMesh_inverseQuatA, _collideConvexVsTriangleMesh_quatA);
    quat.multiply(
        _collideConvexVsTriangleMesh_transform2To1Quat,
        _collideConvexVsTriangleMesh_inverseQuatA,
        _collideConvexVsTriangleMesh_quatB,
    );
    vec3.subtract(_collideConvexVsTriangleMesh_vectorAB, _collideConvexVsTriangleMesh_posB, _collideConvexVsTriangleMesh_posA);
    vec3.transformQuat(
        _collideConvexVsTriangleMesh_transform2To1Pos,
        _collideConvexVsTriangleMesh_vectorAB,
        _collideConvexVsTriangleMesh_inverseQuatA,
    );

    // transform A into B's space for BVH query
    quat.conjugate(_collideConvexVsTriangleMesh_inverseQuatB, _collideConvexVsTriangleMesh_quatB);
    quat.multiply(
        _collideConvexVsTriangleMesh_quatAInB,
        _collideConvexVsTriangleMesh_inverseQuatB,
        _collideConvexVsTriangleMesh_quatA,
    );
    vec3.subtract(
        _collideConvexVsTriangleMesh_positionDifference,
        _collideConvexVsTriangleMesh_posA,
        _collideConvexVsTriangleMesh_posB,
    );
    vec3.transformQuat(
        _collideConvexVsTriangleMesh_posAInB,
        _collideConvexVsTriangleMesh_positionDifference,
        _collideConvexVsTriangleMesh_inverseQuatB,
    );

    // compute shape A's bounds in its own local space
    const boundsOf1 = box3.copy(_collideConvexVsTriangleMesh_boundsOf1, shapeA.aabb);
    box3.scale(boundsOf1, boundsOf1, _collideConvexVsTriangleMesh_scaleA);
    box3.expandByExtents(
        boundsOf1,
        boundsOf1,
        vec3.setScalar(_collideConvexVsTriangleMesh_aabbShapeExpand, settings.maxSeparationDistance),
    );

    // compute shape A's bounds in shape B's space for BVH culling
    const aabbMatrix = mat4.fromRotationTranslationScale(
        _collideConvexVsTriangleMesh_aabbTransform,
        _collideConvexVsTriangleMesh_quatAInB,
        _collideConvexVsTriangleMesh_posAInB,
        _collideConvexVsTriangleMesh_scaleA,
    );
    box3.transformMat4(_collideConvexVsTriangleMesh_boundsOf1InSpaceOf2, shapeA.aabb, aabbMatrix);
    box3.expandByExtents(
        _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2,
        _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2,
        vec3.setScalar(_collideConvexVsTriangleMesh_aabbShapeExpand, settings.maxSeparationDistance),
    );

    // pre-compute transformation matrices (align with joltphysics pattern)
    // A-to-world matrix (rotation + translation only, no scale)
    const mat4_AtoWorld = mat4.fromRotationTranslation(
        _collideConvexVsTriangleMesh_mat4_AtoWorld,
        _collideConvexVsTriangleMesh_quatA,
        _collideConvexVsTriangleMesh_posA,
    );

    // B-to-A matrix (rotation + translation only, no scale - vertices are scaled separately)
    const mat4_BtoA = mat4.fromRotationTranslation(
        _collideConvexVsTriangleMesh_mat4_BtoA,
        _collideConvexVsTriangleMesh_transform2To1Quat,
        _collideConvexVsTriangleMesh_transform2To1Pos,
    );

    // determine if mesh is inside-out
    const scaleSign = vec3.isScaleInsideOut(_collideConvexVsTriangleMesh_scaleB) ? -1 : 1;

    // get support function for shape A (shrunk core; filled once, reused across the triangle loop)
    const supportA = _collideConvexVsTriangleMesh_supportA;
    setShapeSupport(supportA, shapeA, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, _collideConvexVsTriangleMesh_scaleA);

    // bvh traversal
    let stackSize = 0;

    // compute query shape center in B's space for distance-based sorting
    // use center-to-center distance heuristic
    box3.center(_collideConvexVsTriangleMesh_queryCenter, _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2);

    _collideConvexVsTriangleMesh_stackNodes[stackSize++] = 0; // root always visited

    while (stackSize > 0) {
        const nodeOffset = _collideConvexVsTriangleMesh_stackNodes[--stackSize];
        // distance not used for filtering in shape queries

        // skip if node bounds don't intersect query
        if (
            !bvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2[0],
                _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2[1],
                _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2[2],
                _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2[3],
                _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2[4],
                _collideConvexVsTriangleMesh_boundsOf1InSpaceOf2[5],
            )
        ) {
            continue;
        }

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf node: check triangles
            const triStart = triangleMeshBvh.nodeTriStart(buffer, nodeOffset);
            const triCount = triangleMeshBvh.nodeTriCount(buffer, nodeOffset);
            for (let i = 0; i < triCount; i++) {
                const triangleIndex = triStart + i;

                // get triangle vertices
                getTriangleVertices(
                    _collideConvexVsTriangleMesh_getTriangleVertices_a,
                    _collideConvexVsTriangleMesh_getTriangleVertices_b,
                    _collideConvexVsTriangleMesh_getTriangleVertices_c,
                    meshData,
                    triangleIndex,
                );

                // scale triangle in mesh local space, then transform to shape A's local space
                // using pre-computed mat4_BtoA matrix (joltphysics pattern)
                const a = vec3.mul(
                    _collideConvexVsTriangleMesh_triangleA,
                    _collideConvexVsTriangleMesh_getTriangleVertices_a,
                    _collideConvexVsTriangleMesh_scaleB,
                );
                const b = vec3.mul(
                    _collideConvexVsTriangleMesh_triangleB,
                    _collideConvexVsTriangleMesh_getTriangleVertices_b,
                    _collideConvexVsTriangleMesh_scaleB,
                );
                const c = vec3.mul(
                    _collideConvexVsTriangleMesh_triangleC,
                    _collideConvexVsTriangleMesh_getTriangleVertices_c,
                    _collideConvexVsTriangleMesh_scaleB,
                );

                // transform scaled triangle vertices to shape A's local space using mat4
                vec3.transformMat4(_collideConvexVsTriangleMesh_triangleA_inA, a, mat4_BtoA);
                vec3.transformMat4(_collideConvexVsTriangleMesh_triangleB_inA, b, mat4_BtoA);
                vec3.transformMat4(_collideConvexVsTriangleMesh_triangleC_inA, c, mat4_BtoA);

                // compute triangle AABB in shape A's local space
                const triangleAABB = _collideConvexVsTriangleMesh_triangleAABB;
                triangle3.bounds(
                    triangleAABB,
                    _collideConvexVsTriangleMesh_triangleA_inA,
                    _collideConvexVsTriangleMesh_triangleB_inA,
                    _collideConvexVsTriangleMesh_triangleC_inA,
                );

                // early out: if triangle AABB doesn't overlap shape AABB, skip this triangle
                if (!box3.intersectsBox3(triangleAABB, boundsOf1)) {
                    continue;
                }

                vec3.sub(
                    _collideConvexVsTriangleMesh_edgeA,
                    _collideConvexVsTriangleMesh_triangleB_inA,
                    _collideConvexVsTriangleMesh_triangleA_inA,
                );
                vec3.sub(
                    _collideConvexVsTriangleMesh_edgeB,
                    _collideConvexVsTriangleMesh_triangleC_inA,
                    _collideConvexVsTriangleMesh_triangleA_inA,
                );

                // calculate triangle normal in A's local space
                const normal = vec3.scale(
                    _collideConvexVsTriangleMesh_triangleNormal,
                    vec3.cross(
                        _collideConvexVsTriangleMesh_triangleNormal,
                        _collideConvexVsTriangleMesh_edgeA,
                        _collideConvexVsTriangleMesh_edgeB,
                    ),
                    scaleSign,
                );

                // back-face check
                // check if triangle normal and first vertex both point in same direction from origin
                // (shape A is at origin in its local space)
                const backFacing = vec3.dot(normal, _collideConvexVsTriangleMesh_triangleA_inA) > 0.0;
                if (!settings.collideWithBackfaces && backFacing) {
                    continue;
                }

                // create triangle support function
                setTriangleSupport(
                    _collideConvexVsTriangleMesh_triangleSupport,
                    _collideConvexVsTriangleMesh_triangleA_inA,
                    _collideConvexVsTriangleMesh_triangleB_inA,
                    _collideConvexVsTriangleMesh_triangleC_inA,
                );

                // run GJK with negative triangle normal as initial penetration axis
                // (likely that shape A is in front of triangle B)
                const penetrationAxis = vec3.negate(_collideConvexVsTriangleMesh_penetrationAxis, normal);

                // ensure non-zero penetration axis
                if (vec3.squaredLength(penetrationAxis) < 1e-10) {
                    vec3.set(penetrationAxis, 1, 0, 0);
                } else {
                    vec3.normalize(penetrationAxis, penetrationAxis);
                }

                // perform GJK step with inflated shape (convex radius + max separation distance)
                let maxSeparationDistance = settings.maxSeparationDistance;
                penetrationDepthStepGJK(
                    _collideConvexVsTriangleMesh_penetrationDepth,
                    _collideConvexVsTriangleMesh_simplex,
                    supportA,
                    _collideConvexVsTriangleMesh_triangleSupport,
                    supportA.convexRadius + maxSeparationDistance,
                    0, // triangle has no convex radius
                    penetrationAxis,
                    settings.collisionTolerance,
                );

                // check result of collision detection
                if (_collideConvexVsTriangleMesh_penetrationDepth.status === PenetrationDepthStatus.NOT_COLLIDING) {
                    continue;
                }

                if (_collideConvexVsTriangleMesh_penetrationDepth.status === PenetrationDepthStatus.INDETERMINATE) {
                    // need to run expensive EPA algorithm
                    // clamp max separation distance to avoid excessive inflation
                    maxSeparationDistance = Math.min(maxSeparationDistance, 1.0);

                    // fill the inflated A (include convex radius + separation distance) for EPA
                    const supportAWithRadius = _collideConvexVsTriangleMesh_supportAWithRadius;
                    setShapeSupport(
                        supportAWithRadius,
                        shapeA,
                        SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
                        _collideConvexVsTriangleMesh_scaleA,
                    );
                    supportAWithRadius.addRadius = maxSeparationDistance;

                    // perform EPA step
                    if (
                        !penetrationDepthStepEPA(
                            _collideConvexVsTriangleMesh_penetrationDepth,
                            supportAWithRadius,
                            _collideConvexVsTriangleMesh_triangleSupport,
                            settings.penetrationTolerance,
                            _collideConvexVsTriangleMesh_simplex,
                        )
                    ) {
                        continue;
                    }
                }

                // calculate penetration depth (subtract the inflation from the distance)
                const penetration =
                    vec3.distance(
                        _collideConvexVsTriangleMesh_penetrationDepth.pointA,
                        _collideConvexVsTriangleMesh_penetrationDepth.pointB,
                    ) - maxSeparationDistance;

                // check if penetration exceeds early-out threshold
                if (-penetration >= collector.earlyOutFraction) {
                    continue;
                }

                // correct point A by moving it back along penetration axis to account for the max separation distance
                const penetrationAxisLen = vec3.length(_collideConvexVsTriangleMesh_penetrationDepth.penetrationAxis);
                if (penetrationAxisLen > 0.0) {
                    const correction = maxSeparationDistance / penetrationAxisLen;
                    vec3.scaleAndAdd(
                        _collideConvexVsTriangleMesh_penetrationDepth.pointA,
                        _collideConvexVsTriangleMesh_penetrationDepth.pointA,
                        _collideConvexVsTriangleMesh_penetrationDepth.penetrationAxis,
                        -correction,
                    );
                }

                // active edge detection - correct normal if hitting inactive edge
                const triangleActiveEdges = getActiveEdges(meshData, triangleIndex);
                if (settings.collideOnlyWithActiveEdges && triangleActiveEdges !== 0b111) {
                    // transform movement direction from world space to shape A's local space
                    vec3.transformQuat(
                        _collideConvexVsTriangleMesh_activeEdgeMovementDirection,
                        settings.activeEdgeMovementDirection,
                        quat.conjugate(_collideConvexVsTriangleMesh_conjugateQuat, _collideConvexVsTriangleMesh_quatA),
                    );

                    // prepare triangle normal for fixNormal
                    if (backFacing) {
                        vec3.copy(_collideConvexVsTriangleMesh_triangleNormalForFix, normal);
                    } else {
                        vec3.negate(_collideConvexVsTriangleMesh_triangleNormalForFix, normal);
                    }

                    // apply active edge correction (all parameters in shape A's local space)
                    const correctedAxis = activeEdges.fixNormal(
                        _collideConvexVsTriangleMesh_triangleA_inA,
                        _collideConvexVsTriangleMesh_triangleB_inA,
                        _collideConvexVsTriangleMesh_triangleC_inA,
                        _collideConvexVsTriangleMesh_triangleNormalForFix,
                        triangleActiveEdges,
                        _collideConvexVsTriangleMesh_penetrationDepth.pointB,
                        _collideConvexVsTriangleMesh_penetrationDepth.penetrationAxis,
                        _collideConvexVsTriangleMesh_activeEdgeMovementDirection,
                    );

                    vec3.copy(_collideConvexVsTriangleMesh_penetrationDepth.penetrationAxis, correctedAxis);
                }

                // transform results to world space using pre-computed mat4_AtoWorld
                vec3.transformMat4(
                    _collideConvexVsTriangleMesh_worldPointA,
                    _collideConvexVsTriangleMesh_penetrationDepth.pointA,
                    mat4_AtoWorld,
                );
                vec3.transformMat4(
                    _collideConvexVsTriangleMesh_worldPointB,
                    _collideConvexVsTriangleMesh_penetrationDepth.pointB,
                    mat4_AtoWorld,
                );
                mat4.multiply3x3Vec(
                    _collideConvexVsTriangleMesh_collideShapeHit.penetrationAxis,
                    mat4_AtoWorld,
                    _collideConvexVsTriangleMesh_penetrationDepth.penetrationAxis,
                );

                // push sub shape id for triangle
                _collideConvexVsTriangleMesh_subShapeIdBuilder.value = subShapeIdB;
                _collideConvexVsTriangleMesh_subShapeIdBuilder.currentBit = subShapeIdBitsB;
                subShape.pushIndex(
                    _collideConvexVsTriangleMesh_subShapeIdBuilder,
                    _collideConvexVsTriangleMesh_subShapeIdBuilder,
                    triangleIndex,
                    meshShape.data.triangleCount,
                );

                // report collision
                vec3.copy(_collideConvexVsTriangleMesh_collideShapeHit.pointA, _collideConvexVsTriangleMesh_worldPointA);
                vec3.copy(_collideConvexVsTriangleMesh_collideShapeHit.pointB, _collideConvexVsTriangleMesh_worldPointB);
                _collideConvexVsTriangleMesh_collideShapeHit.penetration = penetration;
                _collideConvexVsTriangleMesh_collideShapeHit.subShapeIdA = subShapeIdA;
                _collideConvexVsTriangleMesh_collideShapeHit.subShapeIdB = _collideConvexVsTriangleMesh_subShapeIdBuilder.value;
                _collideConvexVsTriangleMesh_collideShapeHit.materialIdA = (shapeA as ConvexShape).materialId;
                _collideConvexVsTriangleMesh_collideShapeHit.materialIdB = triangleMeshData.getMaterialId(
                    meshShape.data,
                    triangleIndex,
                );
                _collideConvexVsTriangleMesh_collideShapeHit.bodyIdB = collector.bodyIdB;

                // collect faces if requested
                if (settings.collectFaces) {
                    // direction for shape A: opposite of penetration axis (local space)
                    const faceDirectionA = vec3.negate(
                        _collideConvexVsTriangleMesh_temp_faceDirA,
                        _collideConvexVsTriangleMesh_penetrationDepth.penetrationAxis,
                    );
                    getShapeSupportingFace(
                        _collideConvexVsTriangleMesh_collideShapeHit.faceA,
                        shapeA,
                        subShapeIdA,
                        faceDirectionA,
                        _collideConvexVsTriangleMesh_mat4_AtoWorld,
                        _collideConvexVsTriangleMesh_scaleA,
                    );

                    // shape B face: triangle has 3 vertices in shape A's local space
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.numVertices = 3;
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[0] =
                        _collideConvexVsTriangleMesh_triangleA_inA[0];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[1] =
                        _collideConvexVsTriangleMesh_triangleA_inA[1];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[2] =
                        _collideConvexVsTriangleMesh_triangleA_inA[2];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[3] =
                        _collideConvexVsTriangleMesh_triangleB_inA[0];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[4] =
                        _collideConvexVsTriangleMesh_triangleB_inA[1];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[5] =
                        _collideConvexVsTriangleMesh_triangleB_inA[2];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[6] =
                        _collideConvexVsTriangleMesh_triangleC_inA[0];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[7] =
                        _collideConvexVsTriangleMesh_triangleC_inA[1];
                    _collideConvexVsTriangleMesh_collideShapeHit.faceB.vertices[8] =
                        _collideConvexVsTriangleMesh_triangleC_inA[2];

                    // transform from shape A's local space to world space
                    // note: vertices are already scaled (by scaleB), so we use mat4 without scale
                    transformFaceWithMat4RotationTranslation(_collideConvexVsTriangleMesh_collideShapeHit.faceB, mat4_AtoWorld);
                }

                collector.addHit(_collideConvexVsTriangleMesh_collideShapeHit);
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            // use center-to-center distance heuristic
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);

            // compute squared distances from query center to child node centers
            bvh.nodeGetCenter(_collideConvexVsTriangleMesh_nodeCenter, buffer, leftOffset);
            const leftDist = vec3.squaredDistance(
                _collideConvexVsTriangleMesh_queryCenter,
                _collideConvexVsTriangleMesh_nodeCenter,
            );

            bvh.nodeGetCenter(_collideConvexVsTriangleMesh_nodeCenter, buffer, rightOffset);
            const rightDist = vec3.squaredDistance(
                _collideConvexVsTriangleMesh_queryCenter,
                _collideConvexVsTriangleMesh_nodeCenter,
            );

            // sort: push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                // left is closer or equal - push right first
                _collideConvexVsTriangleMesh_stackNodes[stackSize++] = rightOffset;
                _collideConvexVsTriangleMesh_stackNodes[stackSize++] = leftOffset;
            } else {
                // right is closer - push left first
                _collideConvexVsTriangleMesh_stackNodes[stackSize++] = leftOffset;
                _collideConvexVsTriangleMesh_stackNodes[stackSize++] = rightOffset;
            }
        }
    }
}

const collideTriangleMeshVsConvex = /* @__PURE__ */ reversedCollideShapeVsShape(collideConvexVsTriangleMesh);

/* collide sphere vs triangle mesh */

const _collideSphereVsTriangleMesh_sphereCenterInMesh = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_posB = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_quatB = /* @__PURE__ */ quat.create();
const _collideSphereVsTriangleMesh_scaleB = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_inverseQuatB = /* @__PURE__ */ quat.create();
const _collideSphereVsTriangleMesh_positionDifference = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_boundsOfSphere = /* @__PURE__ */ box3.create();
const _collideSphereVsTriangleMesh_queryCenter = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_nodeCenter = /* @__PURE__ */ vec3.create();
// flat SMI stack of node offsets for this shape-query traversal — children are still pushed
// farther-first (sorted by distance) but the sort distance itself isn't stored. grow-once with
// a size counter (no pop()/length churn, stays packed).
const _collideSphereVsTriangleMesh_stackNodes: number[] = [];
const _collideSphereVsTriangleMesh_v0 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_v1 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_v2 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_sv0 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_sv1 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_sv2 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_rv0 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_rv1 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_rv2 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_edge1 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_edge2 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_triangleNormal = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_closestPointResult = /* @__PURE__ */ createClosestPointOnTriangleResult();
const _collideSphereVsTriangleMesh_penetrationAxis = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_point1 = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_point1World = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_point2World = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_penetrationAxisWorld = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_activeEdgeMovementDir = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_newPenetrationAxis = /* @__PURE__ */ vec3.create();
const _collideSphereVsTriangleMesh_hit = /* @__PURE__ */ createCollideShapeHit();
const _collideSphereVsTriangleMesh_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

function collideSphereVsTriangleMesh(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    _subShapeIdBitsA: number,
    posAX: number,
    posAY: number,
    posAZ: number,
    _quatAX: number,
    _quatAY: number,
    _quatAZ: number,
    _quatAW: number,
    scaleAX: number,
    _scaleAY: number,
    _scaleAZ: number,
    shapeB: Shape,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
): void {
    const sphereShape = shapeA as SphereShape;
    const meshShape = shapeB as TriangleMeshShape;

    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    if (buffer.length === 0) {
        return;
    }

    // calculate scaled sphere radius (assuming uniform scale)
    const sphereRadius = Math.abs(scaleAX) * sphereShape.radius;
    const maxSeparationSq = (sphereRadius + settings.maxSeparationDistance) ** 2;

    // transform sphere center to mesh local space
    const posA = vec3.set(_collideSphereVsTriangleMesh_sphereCenterInMesh, posAX, posAY, posAZ);
    const posB = vec3.set(_collideSphereVsTriangleMesh_posB, posBX, posBY, posBZ);
    const quatB = quat.set(_collideSphereVsTriangleMesh_quatB, quatBX, quatBY, quatBZ, quatBW);
    const scaleB = vec3.set(_collideSphereVsTriangleMesh_scaleB, scaleBX, scaleBY, scaleBZ);

    quat.conjugate(_collideSphereVsTriangleMesh_inverseQuatB, quatB);
    vec3.subtract(_collideSphereVsTriangleMesh_positionDifference, posA, posB);
    vec3.transformQuat(
        _collideSphereVsTriangleMesh_sphereCenterInMesh,
        _collideSphereVsTriangleMesh_positionDifference,
        _collideSphereVsTriangleMesh_inverseQuatB,
    );

    const sphereCenterInMesh = _collideSphereVsTriangleMesh_sphereCenterInMesh;

    // detect inside-out scaling
    const scaleSign = vec3.isScaleInsideOut(scaleB) ? -1 : 1;

    // create sphere AABB for BVH culling
    const sphereBounds = _collideSphereVsTriangleMesh_boundsOfSphere;
    const expandedRadius = sphereRadius + settings.maxSeparationDistance;
    sphereBounds[0] = sphereCenterInMesh[0] - expandedRadius;
    sphereBounds[1] = sphereCenterInMesh[1] - expandedRadius;
    sphereBounds[2] = sphereCenterInMesh[2] - expandedRadius;
    sphereBounds[3] = sphereCenterInMesh[0] + expandedRadius;
    sphereBounds[4] = sphereCenterInMesh[1] + expandedRadius;
    sphereBounds[5] = sphereCenterInMesh[2] + expandedRadius;

    // BVH traversal
    let stackSize = 0;

    // use sphere center for distance-based sorting
    vec3.copy(_collideSphereVsTriangleMesh_queryCenter, sphereCenterInMesh);

    _collideSphereVsTriangleMesh_stackNodes[stackSize++] = 0; // root

    while (stackSize > 0) {
        const nodeOffset = _collideSphereVsTriangleMesh_stackNodes[--stackSize];

        // skip if node bounds don't intersect sphere
        if (
            !bvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                sphereBounds[0],
                sphereBounds[1],
                sphereBounds[2],
                sphereBounds[3],
                sphereBounds[4],
                sphereBounds[5],
            )
        ) {
            continue;
        }

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf node: test triangles
            const triStart = triangleMeshBvh.nodeTriStart(buffer, nodeOffset);
            const triCount = triangleMeshBvh.nodeTriCount(buffer, nodeOffset);

            for (let i = 0; i < triCount; i++) {
                const triangleIndex = triStart + i;

                // get triangle vertices
                getTriangleVertices(
                    _collideSphereVsTriangleMesh_v0,
                    _collideSphereVsTriangleMesh_v1,
                    _collideSphereVsTriangleMesh_v2,
                    meshData,
                    triangleIndex,
                );

                const v0 = _collideSphereVsTriangleMesh_v0;
                const v1 = _collideSphereVsTriangleMesh_v1;
                const v2 = _collideSphereVsTriangleMesh_v2;

                // scale triangle vertices
                const sv0 = vec3.multiply(_collideSphereVsTriangleMesh_sv0, v0, scaleB);
                const sv1 = vec3.multiply(_collideSphereVsTriangleMesh_sv1, v1, scaleB);
                const sv2 = vec3.multiply(_collideSphereVsTriangleMesh_sv2, v2, scaleB);

                // make relative to sphere center (sphere at origin)
                const rv0 = vec3.subtract(_collideSphereVsTriangleMesh_rv0, sv0, sphereCenterInMesh);
                const rv1 = vec3.subtract(_collideSphereVsTriangleMesh_rv1, sv1, sphereCenterInMesh);
                const rv2 = vec3.subtract(_collideSphereVsTriangleMesh_rv2, sv2, sphereCenterInMesh);

                // calculate triangle normal
                const edge1 = vec3.subtract(_collideSphereVsTriangleMesh_edge1, rv1, rv0);
                const edge2 = vec3.subtract(_collideSphereVsTriangleMesh_edge2, rv2, rv0);
                const triangleNormal = vec3.cross(_collideSphereVsTriangleMesh_triangleNormal, edge1, edge2);
                vec3.scale(triangleNormal, triangleNormal, scaleSign);

                // backface check
                const backFacing = vec3.dot(triangleNormal, rv0) > 0;
                if (!settings.collideWithBackfaces && backFacing) {
                    continue;
                }

                // get closest point on triangle to sphere center (origin)
                getClosestPointOnTriangle(_collideSphereVsTriangleMesh_closestPointResult, rv0, rv1, rv2);

                const point2 = _collideSphereVsTriangleMesh_closestPointResult.point;
                const point2LenSq = _collideSphereVsTriangleMesh_closestPointResult.distanceSq;

                // early out if too far
                if (point2LenSq > maxSeparationSq) {
                    continue;
                }

                // calculate penetration depth
                const penetrationDepth = sphereRadius - Math.sqrt(point2LenSq);

                // early out if worse than current best
                if (-penetrationDepth >= collector.earlyOutFraction) {
                    continue;
                }

                // calculate penetration axis (away from sphere center)
                const penetrationAxis = _collideSphereVsTriangleMesh_penetrationAxis;
                if (point2LenSq > 0) {
                    vec3.normalize(penetrationAxis, point2);
                } else {
                    // sphere center exactly on triangle - use arbitrary axis
                    vec3.set(penetrationAxis, 0, 1, 0);
                }

                // point on sphere surface
                const point1 = vec3.scale(_collideSphereVsTriangleMesh_point1, penetrationAxis, sphereRadius);

                // active edge handling
                const feature = _collideSphereVsTriangleMesh_closestPointResult.feature;
                const triangleActiveEdges = getActiveEdges(meshData, triangleIndex);

                if (
                    settings.collideOnlyWithActiveEdges &&
                    feature !== 0b111 && // not interior hit
                    triangleActiveEdges !== 0b111 // triangle has some inactive edges
                ) {
                    const requiredEdges = FEATURE_TO_ACTIVE_EDGES[feature];

                    // check if the feature we hit requires an active edge that isn't active
                    if ((triangleActiveEdges & requiredEdges) === 0) {
                        // transform movement direction to mesh space
                        vec3.transformQuat(
                            _collideSphereVsTriangleMesh_activeEdgeMovementDir,
                            settings.activeEdgeMovementDirection,
                            _collideSphereVsTriangleMesh_inverseQuatB,
                        );

                        // apply simplified active edge correction
                        const newPenetrationAxis = backFacing
                            ? triangleNormal
                            : vec3.negate(_collideSphereVsTriangleMesh_newPenetrationAxis, triangleNormal);
                        const newPenetrationAxisLen = vec3.length(newPenetrationAxis);

                        // if penetration_axis affects movement less than triangle normal, use triangle normal
                        if (
                            vec3.dot(_collideSphereVsTriangleMesh_activeEdgeMovementDir, penetrationAxis) *
                                newPenetrationAxisLen >=
                            vec3.dot(_collideSphereVsTriangleMesh_activeEdgeMovementDir, newPenetrationAxis)
                        ) {
                            vec3.copy(penetrationAxis, newPenetrationAxis);
                        }
                    }
                }

                // transform to world space
                vec3.add(_collideSphereVsTriangleMesh_point1World, sphereCenterInMesh, point1);
                vec3.transformQuat(_collideSphereVsTriangleMesh_point1World, _collideSphereVsTriangleMesh_point1World, quatB);
                vec3.add(_collideSphereVsTriangleMesh_point1World, _collideSphereVsTriangleMesh_point1World, posB);

                vec3.add(_collideSphereVsTriangleMesh_point2World, sphereCenterInMesh, point2);
                vec3.transformQuat(_collideSphereVsTriangleMesh_point2World, _collideSphereVsTriangleMesh_point2World, quatB);
                vec3.add(_collideSphereVsTriangleMesh_point2World, _collideSphereVsTriangleMesh_point2World, posB);

                vec3.transformQuat(_collideSphereVsTriangleMesh_penetrationAxisWorld, penetrationAxis, quatB);

                // build sub shape id
                _collideSphereVsTriangleMesh_subShapeIdBuilder.value = subShapeIdB;
                _collideSphereVsTriangleMesh_subShapeIdBuilder.currentBit = subShapeIdBitsB;
                subShape.pushIndex(
                    _collideSphereVsTriangleMesh_subShapeIdBuilder,
                    _collideSphereVsTriangleMesh_subShapeIdBuilder,
                    triangleIndex,
                    meshShape.data.triangleCount,
                );

                // create collision result
                const hit = _collideSphereVsTriangleMesh_hit;
                vec3.copy(hit.pointA, _collideSphereVsTriangleMesh_point1World);
                vec3.copy(hit.pointB, _collideSphereVsTriangleMesh_point2World);
                vec3.copy(hit.penetrationAxis, _collideSphereVsTriangleMesh_penetrationAxisWorld);
                hit.penetration = penetrationDepth;
                hit.subShapeIdA = subShapeIdA;
                hit.subShapeIdB = _collideSphereVsTriangleMesh_subShapeIdBuilder.value;
                hit.materialIdA = sphereShape.materialId;
                hit.materialIdB = triangleMeshData.getMaterialId(meshShape.data, triangleIndex);
                hit.bodyIdB = collector.bodyIdB;

                // collect faces if requested
                if (settings.collectFaces) {
                    // sphere has no supporting face (point contact)
                    hit.faceA.numVertices = 0;

                    // triangle face in world space
                    hit.faceB.numVertices = 3;

                    // transform scaled triangle vertices to world space
                    vec3.transformQuat(sv0, sv0, quatB);
                    vec3.add(sv0, sv0, posB);
                    vec3.transformQuat(sv1, sv1, quatB);
                    vec3.add(sv1, sv1, posB);
                    vec3.transformQuat(sv2, sv2, quatB);
                    vec3.add(sv2, sv2, posB);

                    hit.faceB.vertices[0] = sv0[0];
                    hit.faceB.vertices[1] = sv0[1];
                    hit.faceB.vertices[2] = sv0[2];
                    hit.faceB.vertices[3] = sv1[0];
                    hit.faceB.vertices[4] = sv1[1];
                    hit.faceB.vertices[5] = sv1[2];
                    hit.faceB.vertices[6] = sv2[0];
                    hit.faceB.vertices[7] = sv2[1];
                    hit.faceB.vertices[8] = sv2[2];
                }

                collector.addHit(hit);
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);

            // compute squared distances from sphere center to child node centers
            bvh.nodeGetCenter(_collideSphereVsTriangleMesh_nodeCenter, buffer, leftOffset);
            const leftDist = vec3.squaredDistance(
                _collideSphereVsTriangleMesh_queryCenter,
                _collideSphereVsTriangleMesh_nodeCenter,
            );

            bvh.nodeGetCenter(_collideSphereVsTriangleMesh_nodeCenter, buffer, rightOffset);
            const rightDist = vec3.squaredDistance(
                _collideSphereVsTriangleMesh_queryCenter,
                _collideSphereVsTriangleMesh_nodeCenter,
            );

            // sort: push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                _collideSphereVsTriangleMesh_stackNodes[stackSize++] = rightOffset;
                _collideSphereVsTriangleMesh_stackNodes[stackSize++] = leftOffset;
            } else {
                _collideSphereVsTriangleMesh_stackNodes[stackSize++] = leftOffset;
                _collideSphereVsTriangleMesh_stackNodes[stackSize++] = rightOffset;
            }
        }
    }
}

const collideTriangleMeshVsSphere = /* @__PURE__ */ reversedCollideShapeVsShape(collideSphereVsTriangleMesh);

/* cast sphere vs triangle mesh */

const _castSphereVsTriangleMesh_start = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_direction = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_posB = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_quatB = /* @__PURE__ */ quat.create();
const _castSphereVsTriangleMesh_scaleB = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_inverseQuatB = /* @__PURE__ */ quat.create();
const _castSphereVsTriangleMesh_positionDifference = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_displacement = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_ray = /* @__PURE__ */ raycast3.create();
const _castSphereVsTriangleMesh_expandedBounds = /* @__PURE__ */ box3.create();
// parallel flat stacks for this cast traversal: node offsets (SMI array) + distances (double
// array) sharing one size counter — jolt keeps the same structure (a distance stack parallel
// to the node stack). separate arrays so each keeps its optimal element kind.
const _castSphereVsTriangleMesh_stackNodes: number[] = [];
const _castSphereVsTriangleMesh_stackDist: number[] = [];
const _castSphereVsTriangleMesh_v0 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v1 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v2 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_sv0 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_sv1 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_sv2 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_rv0 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_rv1 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_rv2 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_edge1 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_edge2 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_triangleNormal = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_closestPointResult = /* @__PURE__ */ createClosestPointOnTriangleResult();
const _castSphereVsTriangleMesh_hit = /* @__PURE__ */ createCastShapeHit();
const _castSphereVsTriangleMesh_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();
const _castSphereVsTriangleMesh_activeEdgeMovementDir = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_origin = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_triangleNormalForFix = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_contactPointAWorld = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_contactPointBWorld = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_contactNormalWorld = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_planeIntersectionTemp = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_interiorContactNormal = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_sphereCenterAtHit = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v0RelativeToHit = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v1RelativeToHit = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v2RelativeToHit = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_edgeVertexContactNormal = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_edgeVertexContactPoint = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_contactNormal = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_contactPointA = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_d = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v0Rel = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v1Rel = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_v2Rel = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_n = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_n0 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_n1 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_n2 = /* @__PURE__ */ vec3.create();
const _castSphereVsTriangleMesh_p = /* @__PURE__ */ vec3.create();

/** helper to add a cast hit with active edge detection for sphere vs triangle mesh */
function castSphereVsTriangleMeshAddHit(
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    sphereShape: any,
    meshShape: TriangleMeshShape,
    subShapeIdA: number,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    sphereRadius: number,
    start: Vec3,
    posB: Vec3,
    quatB: Quat,
    inverseQuatB: Quat,
    backFacing: boolean,
    triangleNormal: Vec3,
    activeEdges: number,
    triangleIndex: number,
    fraction: number,
    contactPointA: Vec3,
    contactPointB: Vec3,
    contactNormal: Vec3,
): void {
    let finalContactNormal = contactNormal;

    if (settings.collideOnlyWithActiveEdges && activeEdges !== 0b111) {
        // transform movement direction to mesh space
        vec3.transformQuat(_castSphereVsTriangleMesh_activeEdgeMovementDir, settings.activeEdgeMovementDirection, inverseQuatB);

        // apply simplified active edge correction
        const triangleNormalForFix = backFacing
            ? triangleNormal
            : vec3.negate(_castSphereVsTriangleMesh_triangleNormalForFix, triangleNormal);
        const triangleNormalLen = vec3.length(triangleNormalForFix);
        const contactNormalLen = vec3.length(contactNormal);

        if (
            vec3.dot(_castSphereVsTriangleMesh_activeEdgeMovementDir, contactNormal) * triangleNormalLen <
            vec3.dot(_castSphereVsTriangleMesh_activeEdgeMovementDir, triangleNormalForFix) * contactNormalLen
        ) {
            finalContactNormal = triangleNormalForFix;
        }
    }

    // transform to world space
    const contactPointAWorld = vec3.add(_castSphereVsTriangleMesh_contactPointAWorld, start, contactPointA);
    vec3.transformQuat(contactPointAWorld, contactPointAWorld, quatB);
    vec3.add(contactPointAWorld, contactPointAWorld, posB);

    const contactPointBWorld = vec3.add(_castSphereVsTriangleMesh_contactPointBWorld, start, contactPointB);
    vec3.transformQuat(contactPointBWorld, contactPointBWorld, quatB);
    vec3.add(contactPointBWorld, contactPointBWorld, posB);

    const contactNormalWorld = vec3.transformQuat(_castSphereVsTriangleMesh_contactNormalWorld, finalContactNormal, quatB);
    vec3.normalize(contactNormalWorld, contactNormalWorld);

    // build sub shape id
    _castSphereVsTriangleMesh_subShapeIdBuilder.value = subShapeIdB;
    _castSphereVsTriangleMesh_subShapeIdBuilder.currentBit = subShapeIdBitsB;
    subShape.pushIndex(
        _castSphereVsTriangleMesh_subShapeIdBuilder,
        _castSphereVsTriangleMesh_subShapeIdBuilder,
        triangleIndex,
        meshShape.data.triangleCount,
    );

    // create cast result
    const hit = _castSphereVsTriangleMesh_hit;
    hit.status = CastShapeStatus.COLLIDING;
    hit.fraction = fraction;
    vec3.copy(hit.pointA, contactPointAWorld);
    vec3.copy(hit.pointB, contactPointBWorld);
    vec3.copy(hit.normal, contactNormalWorld);
    vec3.negate(hit.penetrationAxis, contactNormalWorld);
    hit.penetrationDepth = fraction === 0 ? sphereRadius - vec3.distance(contactPointA, contactPointB) : 0;
    hit.subShapeIdA = subShapeIdA;
    hit.subShapeIdB = _castSphereVsTriangleMesh_subShapeIdBuilder.value;
    hit.materialIdA = sphereShape.materialId;
    hit.materialIdB = triangleMeshData.getMaterialId(meshShape.data, triangleIndex);
    hit.bodyIdB = collector.bodyIdB;
    hit.faceA.numVertices = 0; // sphere has no face
    hit.faceB.numVertices = 0; // don't collect faces for swept tests

    collector.addHit(hit);
}

/**
 * Specialized cast function for sphere vs triangle mesh.
 * Direct geometric tests instead of GJK for 3-5x performance improvement.
 */
function castSphereVsTriangleMesh(
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    _subShapeIdBitsA: number,
    posAX: number,
    posAY: number,
    posAZ: number,
    _quatAX: number,
    _quatAY: number,
    _quatAZ: number,
    _quatAW: number,
    scaleAX: number,
    _scaleAY: number,
    _scaleAZ: number,
    displacementAX: number,
    displacementAY: number,
    displacementAZ: number,
    shapeB: Shape,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
): void {
    const sphereShape = shapeA as SphereShape;
    const meshShape = shapeB as TriangleMeshShape;

    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    if (buffer.length === 0) {
        return;
    }

    // calculate scaled sphere radius (assuming uniform scale)
    const sphereRadius = Math.abs(scaleAX) * sphereShape.radius;

    // setup sphere sweep in mesh local space
    const posA = vec3.set(_castSphereVsTriangleMesh_start, posAX, posAY, posAZ);
    const posB = vec3.set(_castSphereVsTriangleMesh_posB, posBX, posBY, posBZ);
    const quatB = quat.set(_castSphereVsTriangleMesh_quatB, quatBX, quatBY, quatBZ, quatBW);
    const scaleB = vec3.set(_castSphereVsTriangleMesh_scaleB, scaleBX, scaleBY, scaleBZ);
    const displacement = vec3.set(_castSphereVsTriangleMesh_displacement, displacementAX, displacementAY, displacementAZ);

    // transform sphere start position to mesh local space
    quat.conjugate(_castSphereVsTriangleMesh_inverseQuatB, quatB);
    vec3.subtract(_castSphereVsTriangleMesh_positionDifference, posA, posB);
    vec3.transformQuat(
        _castSphereVsTriangleMesh_start,
        _castSphereVsTriangleMesh_positionDifference,
        _castSphereVsTriangleMesh_inverseQuatB,
    );

    // transform displacement to mesh local space
    vec3.transformQuat(_castSphereVsTriangleMesh_direction, displacement, _castSphereVsTriangleMesh_inverseQuatB);

    const start = _castSphereVsTriangleMesh_start;
    const direction = _castSphereVsTriangleMesh_direction;

    // detect inside-out scaling
    const scaleSign = vec3.isScaleInsideOut(scaleB) ? -1 : 1;

    // ordered front-to-back traversal (matches castConvexVsTriangleMesh): ray from the
    // sphere center at t=0 along the displacement, node bounds expanded by the sphere
    // radius, children pushed nearest-on-top and pruned against the early-out fraction
    const ray = _castSphereVsTriangleMesh_ray;
    vec3.copy(ray.origin, start);
    ray.length = vec3.length(direction);
    if (ray.length > 1e-10) {
        vec3.normalize(ray.direction, direction);
    } else {
        ray.direction[0] = 0;
        ray.direction[1] = 0;
        ray.direction[2] = 0;
    }

    let stackSize = 0;
    _castSphereVsTriangleMesh_stackNodes[stackSize] = 0;
    _castSphereVsTriangleMesh_stackDist[stackSize] = 0; // root always visited
    stackSize++;

    const expandedBounds = _castSphereVsTriangleMesh_expandedBounds;

    while (stackSize > 0) {
        stackSize--;
        const nodeOffset = _castSphereVsTriangleMesh_stackNodes[stackSize];
        const nodeDistance = _castSphereVsTriangleMesh_stackDist[stackSize];

        // early out: if fraction to this node >= closest hit, skip it
        if (nodeDistance >= collector.earlyOutFraction) {
            continue;
        }

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf node: test triangles
            const triStart = triangleMeshBvh.nodeTriStart(buffer, nodeOffset);
            const triCount = triangleMeshBvh.nodeTriCount(buffer, nodeOffset);

            for (let i = 0; i < triCount; i++) {
                const triangleIndex = triStart + i;

                // get triangle vertices
                getTriangleVertices(
                    _castSphereVsTriangleMesh_v0,
                    _castSphereVsTriangleMesh_v1,
                    _castSphereVsTriangleMesh_v2,
                    meshData,
                    triangleIndex,
                );

                const v0 = _castSphereVsTriangleMesh_v0;
                const v1 = _castSphereVsTriangleMesh_v1;
                const v2 = _castSphereVsTriangleMesh_v2;

                // scale triangle vertices
                const sv0 = vec3.multiply(_castSphereVsTriangleMesh_sv0, v0, scaleB);
                const sv1 = vec3.multiply(_castSphereVsTriangleMesh_sv1, v1, scaleB);
                const sv2 = vec3.multiply(_castSphereVsTriangleMesh_sv2, v2, scaleB);

                // make relative to sweep start
                const rv0 = vec3.subtract(_castSphereVsTriangleMesh_rv0, sv0, start);
                const rv1 = vec3.subtract(_castSphereVsTriangleMesh_rv1, sv1, start);
                const rv2 = vec3.subtract(_castSphereVsTriangleMesh_rv2, sv2, start);

                // calculate triangle normal
                const edge1 = vec3.subtract(_castSphereVsTriangleMesh_edge1, rv1, rv0);
                const edge2 = vec3.subtract(_castSphereVsTriangleMesh_edge2, rv2, rv0);
                const triangleNormal = vec3.cross(_castSphereVsTriangleMesh_triangleNormal, edge1, edge2);
                const triangleNormalLen = vec3.length(triangleNormal);

                if (triangleNormalLen === 0) {
                    continue; // degenerate triangle
                }

                vec3.scale(triangleNormal, triangleNormal, scaleSign / triangleNormalLen);

                // backface check
                const normalDotDirection = vec3.dot(triangleNormal, direction);
                const backFacing = normalDotDirection > 0;

                if (!settings.collideWithBackfaces && backFacing) {
                    continue;
                }

                const triangleActiveEdges = getActiveEdges(meshData, triangleIndex);

                // CASE 1: Check if already penetrating at t=0
                if (Math.abs(vec3.dot(rv0, triangleNormal)) <= sphereRadius) {
                    getClosestPointOnTriangle(_castSphereVsTriangleMesh_closestPointResult, rv0, rv1, rv2);
                    const q = _castSphereVsTriangleMesh_closestPointResult.point;
                    const qLenSq = _castSphereVsTriangleMesh_closestPointResult.distanceSq;

                    if (qLenSq <= sphereRadius * sphereRadius) {
                        const qLen = Math.sqrt(qLenSq);
                        const penetrationDepth = sphereRadius - qLen;

                        if (-penetrationDepth >= collector.earlyOutFraction) {
                            continue;
                        }

                        const contactNormal =
                            qLen > 0
                                ? vec3.scale(_castSphereVsTriangleMesh_contactNormal, q, 1 / qLen)
                                : vec3.fromValues(0, 1, 0);
                        const contactPointA = vec3.scaleAndAdd(
                            _castSphereVsTriangleMesh_contactPointA,
                            q,
                            contactNormal,
                            penetrationDepth,
                        );
                        const contactPointB = q;

                        castSphereVsTriangleMeshAddHit(
                            collector,
                            settings,
                            sphereShape,
                            meshShape,
                            subShapeIdA,
                            subShapeIdB,
                            subShapeIdBitsB,
                            sphereRadius,
                            start,
                            posB,
                            quatB,
                            _castSphereVsTriangleMesh_inverseQuatB,
                            backFacing,
                            triangleNormal,
                            triangleActiveEdges,
                            triangleIndex,
                            0.0,
                            contactPointA,
                            contactPointB,
                            contactNormal,
                        );
                        continue;
                    }
                }

                // CASE 2: Plane intersection test
                const absNormalDotDirection = Math.abs(normalDotDirection);
                if (absNormalDotDirection > 1e-6) {
                    // calculate point on sphere that hits plane first
                    const d = vec3.scale(
                        _castSphereVsTriangleMesh_d,
                        triangleNormal,
                        Math.sign(normalDotDirection) * sphereRadius,
                    );

                    const planeIntersection =
                        vec3.dot(vec3.subtract(_castSphereVsTriangleMesh_planeIntersectionTemp, rv0, d), triangleNormal) /
                        normalDotDirection;

                    // check if hit is within sweep range
                    if (
                        planeIntersection * absNormalDotDirection >= -sphereRadius &&
                        planeIntersection < collector.earlyOutFraction
                    ) {
                        if (planeIntersection >= 0) {
                            // calculate contact point on plane
                            const p = vec3.scaleAndAdd(_castSphereVsTriangleMesh_p, d, direction, planeIntersection);

                            // check if interior point using barycentric coordinates
                            const bary = { u: 0, v: 0, w: 0 };
                            const v0Rel = vec3.subtract(_castSphereVsTriangleMesh_v0Rel, rv0, p);
                            const v1Rel = vec3.subtract(_castSphereVsTriangleMesh_v1Rel, rv1, p);
                            const v2Rel = vec3.subtract(_castSphereVsTriangleMesh_v2Rel, rv2, p);

                            // simplified barycentric test
                            const n = vec3.cross(_castSphereVsTriangleMesh_n, v1Rel, v2Rel);
                            const nLen = vec3.length(n);
                            if (nLen > 0) {
                                const totalArea = vec3.dot(n, triangleNormal) / nLen;
                                if (totalArea > 0) {
                                    const n0 = vec3.cross(_castSphereVsTriangleMesh_n0, v1Rel, v2Rel);
                                    const n1 = vec3.cross(_castSphereVsTriangleMesh_n1, v2Rel, v0Rel);
                                    const n2 = vec3.cross(_castSphereVsTriangleMesh_n2, v0Rel, v1Rel);

                                    bary.u = vec3.dot(n0, triangleNormal) / (totalArea * nLen);
                                    bary.v = vec3.dot(n1, triangleNormal) / (totalArea * nLen);
                                    bary.w = vec3.dot(n2, triangleNormal) / (totalArea * nLen);

                                    if (bary.u >= 0 && bary.v >= 0 && bary.w >= 0) {
                                        // interior hit
                                        const contactNormal = backFacing
                                            ? triangleNormal
                                            : vec3.negate(_castSphereVsTriangleMesh_interiorContactNormal, triangleNormal);
                                        castSphereVsTriangleMeshAddHit(
                                            collector,
                                            settings,
                                            sphereShape,
                                            meshShape,
                                            subShapeIdA,
                                            subShapeIdB,
                                            subShapeIdBitsB,
                                            sphereRadius,
                                            start,
                                            posB,
                                            quatB,
                                            _castSphereVsTriangleMesh_inverseQuatB,
                                            backFacing,
                                            triangleNormal,
                                            0b111, // interior hit, all edges active
                                            triangleIndex,
                                            planeIntersection,
                                            p,
                                            p,
                                            contactNormal,
                                        );
                                        continue;
                                    }
                                }
                            }
                        }
                    }
                }

                // CASE 3: Edge and vertex tests
                let fraction = Infinity;

                // test 3 edges (swept sphere vs cylinder)
                fraction = Math.min(fraction, rayCylinder(direction, rv0, rv1, sphereRadius));
                fraction = Math.min(fraction, rayCylinder(direction, rv1, rv2, sphereRadius));
                fraction = Math.min(fraction, rayCylinder(direction, rv2, rv0, sphereRadius));

                // test 3 vertices (swept sphere vs sphere)
                vec3.set(_castSphereVsTriangleMesh_origin, 0, 0, 0);
                const origin = _castSphereVsTriangleMesh_origin;
                fraction = Math.min(fraction, raySphereFromOrigin(direction, rv0, sphereRadius));
                fraction = Math.min(fraction, raySphereFromOrigin(direction, rv1, sphereRadius));
                fraction = Math.min(fraction, raySphereFromOrigin(direction, rv2, sphereRadius));

                // check if we have a valid hit
                if (fraction >= 0 && fraction < collector.earlyOutFraction) {
                    // calculate sphere center at collision point
                    const p = vec3.scaleAndAdd(_castSphereVsTriangleMesh_sphereCenterAtHit, origin, direction, fraction);

                    // get closest point on triangle
                    getClosestPointOnTriangle(
                        _castSphereVsTriangleMesh_closestPointResult,
                        vec3.subtract(_castSphereVsTriangleMesh_v0RelativeToHit, rv0, p),
                        vec3.subtract(_castSphereVsTriangleMesh_v1RelativeToHit, rv1, p),
                        vec3.subtract(_castSphereVsTriangleMesh_v2RelativeToHit, rv2, p),
                    );

                    const q = _castSphereVsTriangleMesh_closestPointResult.point;
                    const contactNormal = vec3.normalize(_castSphereVsTriangleMesh_edgeVertexContactNormal, q);
                    const contactPointAB = vec3.add(_castSphereVsTriangleMesh_edgeVertexContactPoint, p, q);

                    castSphereVsTriangleMeshAddHit(
                        collector,
                        settings,
                        sphereShape,
                        meshShape,
                        subShapeIdA,
                        subShapeIdB,
                        subShapeIdBitsB,
                        sphereRadius,
                        start,
                        posB,
                        quatB,
                        _castSphereVsTriangleMesh_inverseQuatB,
                        backFacing,
                        triangleNormal,
                        triangleActiveEdges,
                        triangleIndex,
                        fraction,
                        contactPointAB,
                        contactPointAB,
                        contactNormal,
                    );
                }
            }
        } else {
            // internal node: distance-order both children along the cast ray,
            // node bounds expanded by the sphere radius
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);

            expandedBounds[0] = buffer[leftOffset + bvh.NODE_MIN_X] - sphereRadius;
            expandedBounds[1] = buffer[leftOffset + bvh.NODE_MIN_Y] - sphereRadius;
            expandedBounds[2] = buffer[leftOffset + bvh.NODE_MIN_Z] - sphereRadius;
            expandedBounds[3] = buffer[leftOffset + bvh.NODE_MAX_X] + sphereRadius;
            expandedBounds[4] = buffer[leftOffset + bvh.NODE_MAX_Y] + sphereRadius;
            expandedBounds[5] = buffer[leftOffset + bvh.NODE_MAX_Z] + sphereRadius;

            const leftDist = rayDistanceToBox3(
                ray.origin[0],
                ray.origin[1],
                ray.origin[2],
                ray.direction[0],
                ray.direction[1],
                ray.direction[2],
                ray.length,
                expandedBounds,
            );

            expandedBounds[0] = buffer[rightOffset + bvh.NODE_MIN_X] - sphereRadius;
            expandedBounds[1] = buffer[rightOffset + bvh.NODE_MIN_Y] - sphereRadius;
            expandedBounds[2] = buffer[rightOffset + bvh.NODE_MIN_Z] - sphereRadius;
            expandedBounds[3] = buffer[rightOffset + bvh.NODE_MAX_X] + sphereRadius;
            expandedBounds[4] = buffer[rightOffset + bvh.NODE_MAX_Y] + sphereRadius;
            expandedBounds[5] = buffer[rightOffset + bvh.NODE_MAX_Z] + sphereRadius;

            const rightDist = rayDistanceToBox3(
                ray.origin[0],
                ray.origin[1],
                ray.origin[2],
                ray.direction[0],
                ray.direction[1],
                ray.direction[2],
                ray.length,
                expandedBounds,
            );

            // sort: push farther child first (so closer child is on top of stack), lifo traversal
            if (leftDist <= rightDist) {
                if (rightDist < collector.earlyOutFraction) {
                    _castSphereVsTriangleMesh_stackNodes[stackSize] = rightOffset;
                    _castSphereVsTriangleMesh_stackDist[stackSize] = rightDist;
                    stackSize++;
                }
                if (leftDist < collector.earlyOutFraction) {
                    _castSphereVsTriangleMesh_stackNodes[stackSize] = leftOffset;
                    _castSphereVsTriangleMesh_stackDist[stackSize] = leftDist;
                    stackSize++;
                }
            } else {
                if (leftDist < collector.earlyOutFraction) {
                    _castSphereVsTriangleMesh_stackNodes[stackSize] = leftOffset;
                    _castSphereVsTriangleMesh_stackDist[stackSize] = leftDist;
                    stackSize++;
                }
                if (rightDist < collector.earlyOutFraction) {
                    _castSphereVsTriangleMesh_stackNodes[stackSize] = rightOffset;
                    _castSphereVsTriangleMesh_stackDist[stackSize] = rightDist;
                    stackSize++;
                }
            }
        }
    }
}

const castTriangleMeshVsSphere = /* @__PURE__ */ reversedCastShapeVsShape(castSphereVsTriangleMesh);
