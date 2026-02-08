import { type Box3, box3, mat4, type Quat, quat, type Raycast3, raycast3, triangle3, type Vec3, vec3 } from 'mathcat';
import { CastRayStatus, createCastRayHit, createDefaultCastRaySettings } from 'src/collision/cast-ray-vs-shape';
import type { MassProperties } from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import * as activeEdges from '../collision/active-edges';
import type { CastRayCollector, CastRayHit, CastRaySettings } from '../collision/cast-ray-vs-shape';
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
import {
    createAddConvexRadiusSupport,
    createShapeSupportPool,
    createTransformedSupport,
    createTriangleSupport,
    getShapeSupportFunction,
    SupportFunctionMode,
    setAddConvexRadiusSupport,
    setTransformedSupport,
    setTriangleSupport,
} from '../collision/support';
import { createClosestPointOnTriangleResult, getClosestPointOnTriangle } from '../collision/triangle';
import { assert } from '../utils/assert';
import * as bvhStack from '../utils/bvh-stack';
import { isScaleInsideOut, transformFace } from '../utils/face';
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
    shapeDefs,
} from './shapes';
import { buildTriangleMesh } from './utils/triangle-mesh-builder';
import type { BvhSplitStrategy, TriangleMeshBVH } from './utils/triangle-mesh-bvh';
import * as triangleMeshBvh from './utils/triangle-mesh-bvh';
import type { TriangleMeshData } from './utils/triangle-mesh-data';
import * as triangleMeshData from './utils/triangle-mesh-data';
import { calculateTriangleAABB, getActiveEdges, getTriangleNormal, getTriangleVertices } from './utils/triangle-mesh-data';

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
    const { bvh, data } = buildTriangleMesh({
        positions: o.positions,
        indices: o.indices,
        materialIds: o.materialIndices,
        bvhSplitStrategy: o.bvhSplitStrategy ?? DEFAULT_TRIANGLE_MESH_OPTIONS.bvhSplitStrategy,
        bvhMaxLeafTris: o.bvhMaxLeafTris ?? DEFAULT_TRIANGLE_MESH_OPTIONS.bvhMaxLeafTris,
        degenerateTolerance: o.degenerateTolerance ?? DEFAULT_TRIANGLE_MESH_OPTIONS.degenerateTolerance,
        activeEdgeCosThresholdAngle: o.activeEdgeCosThresholdAngle ?? DEFAULT_TRIANGLE_MESH_OPTIONS.activeEdgeCosThresholdAngle,
    });

    let aabb: Box3;
    if (bvh.buffer.length > 0) {
        aabb = box3.create();
        triangleMeshBvh.nodeGetBounds(bvh.buffer, 0, aabb);
    } else {
        aabb = box3.create();
    }

    const shape: TriangleMeshShape = {
        type: ShapeType.TRIANGLE_MESH,
        bvh,
        data,
        aabb,
        centerOfMass: [0, 0, 0],
        volume: 0,
    };

    return shape;
}

const _subShapeIdPopResult = subShape.popResult();
const _getSurfaceNormal_normal: Vec3 = [0, 0, 0];
const _getSupportingFace_a: Vec3 = [0, 0, 0];
const _getSupportingFace_b: Vec3 = [0, 0, 0];
const _getSupportingFace_c: Vec3 = [0, 0, 0];

export const def = defineShape<TriangleMeshShape>({
    type: ShapeType.TRIANGLE_MESH,
    category: ShapeCategory.MESH,
    computeMassProperties(out: MassProperties, _shape: TriangleMeshShape): void {
        // triangle meshes are static collision geometry by default.
        // mass properties should be overridden at the body level if needed.
        out.mass = 0;
        mat4.identity(out.inertia);
    },
    getSurfaceNormal(ioResult: SurfaceNormalResult, shape: TriangleMeshShape, subShapeId: number): void {
        subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.data.triangleCount);
        const triangleIndex = _subShapeIdPopResult.value;

        // if triangleIndex is a valid triangle index, return that triangle's normal
        if (triangleIndex >= 0 && triangleIndex < shape.data.triangleCount) {
            getTriangleNormal(_getSurfaceNormal_normal, shape.data, triangleIndex);
            vec3.copy(ioResult.normal, _getSurfaceNormal_normal);
            return;
        }

        assert(false, 'Invalid SubShapeID for TriangleMeshShape');
    },
    getSupportingFace(ioResult: SupportingFaceResult, _direction: Vec3, shape: TriangleMeshShape, subShapeId: number): void {
        const face = ioResult.face;
        const { position, quaternion, scale } = ioResult;

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

        transformFace(face, position, quaternion, scale);
    },
    getInnerRadius(_shape: TriangleMeshShape): number {
        return 0.0;
    },
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
});

/* cast ray */

const _castRayVsTriangleMesh_pos = vec3.create();
const _castRayVsTriangleMesh_quat = quat.create();
const _castRayVsTriangleMesh_scale = vec3.create();
const _castRayVsTriangleMesh_rayOriginLocal = vec3.create();
const _castRayVsTriangleMesh_rayDirectionLocal = vec3.create();
const _castRayVsTriangleMesh_invQuat = quat.create();
const _castRayVsTriangleMesh_ray = raycast3.create();
const _castRayVsTriangleMesh_hitResult = raycast3.createIntersectsTriangleResult();
const _castRayVsTriangleMesh_stack = bvhStack.create();
const _castRayVsTriangleMesh_leftBounds = box3.create();
const _castRayVsTriangleMesh_rightBounds = box3.create();
const _castRayVsTriangleMesh_a = vec3.create();
const _castRayVsTriangleMesh_b = vec3.create();
const _castRayVsTriangleMesh_c = vec3.create();
const _castRayVsTriangleMesh_hit = createCastRayHit();
const _castRayVsTriangleMesh_subShapeIdBuilder = subShape.builder();

function castRayVsTriangleMesh(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
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

    // transform ray from world space to mesh local space
    quat.conjugate(_castRayVsTriangleMesh_invQuat, _castRayVsTriangleMesh_quat);

    // transform ray origin: (origin - position) rotated by inverse quaternion
    vec3.subtract(_castRayVsTriangleMesh_rayOriginLocal, ray.origin, _castRayVsTriangleMesh_pos);
    vec3.transformQuat(
        _castRayVsTriangleMesh_rayOriginLocal,
        _castRayVsTriangleMesh_rayOriginLocal,
        _castRayVsTriangleMesh_invQuat,
    );

    // transform ray direction: direction rotated by inverse quaternion
    vec3.copy(_castRayVsTriangleMesh_rayDirectionLocal, ray.direction);
    vec3.transformQuat(
        _castRayVsTriangleMesh_rayDirectionLocal,
        _castRayVsTriangleMesh_rayDirectionLocal,
        _castRayVsTriangleMesh_invQuat,
    );

    // handle scale by dividing ray direction components by scale
    // (scale affects how far we need to go in local space to reach world distance)
    _castRayVsTriangleMesh_rayDirectionLocal[0] /= _castRayVsTriangleMesh_scale[0];
    _castRayVsTriangleMesh_rayDirectionLocal[1] /= _castRayVsTriangleMesh_scale[1];
    _castRayVsTriangleMesh_rayDirectionLocal[2] /= _castRayVsTriangleMesh_scale[2];

    // prepare local ray for bvh traversal
    vec3.copy(_castRayVsTriangleMesh_ray.origin, _castRayVsTriangleMesh_rayOriginLocal);
    vec3.copy(_castRayVsTriangleMesh_ray.direction, _castRayVsTriangleMesh_rayDirectionLocal);
    _castRayVsTriangleMesh_ray.length = ray.length;

    const buffer = shape.bvh.buffer;
    const meshData = shape.data;

    if (buffer.length === 0) {
        collector.addMiss();
        return;
    }

    let foundHit = false;
    bvhStack.reset(_castRayVsTriangleMesh_stack);
    bvhStack.push(_castRayVsTriangleMesh_stack, 0, -Infinity); // root always visited

    while (_castRayVsTriangleMesh_stack.size > 0) {
        // early out: very close hit
        if (collector.earlyOutFraction <= 0) {
            break;
        }

        const entry = bvhStack.pop(_castRayVsTriangleMesh_stack)!;

        // early out: if fraction to this node >= closest hit, skip it
        if (entry.distance >= collector.earlyOutFraction) {
            continue;
        }

        const nodeOffset = entry.nodeIndex;

        // note: no need to test ray x triangle bounds intersection here - we already proved the ray
        // intersects this node's bounds when we computed the distance during push.
        // if the ray didn't intersect, rayDistanceToBox3 would have returned Infinity
        // and we wouldn't have pushed this node onto the stack.

        if (triangleMeshBvh.nodeIsLeaf(buffer, nodeOffset)) {
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
                    _castRayVsTriangleMesh_ray,
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
            const leftOffset = triangleMeshBvh.nodeLeft(nodeOffset);
            const rightOffset = triangleMeshBvh.nodeRight(buffer, nodeOffset);

            triangleMeshBvh.nodeGetBounds(buffer, leftOffset, _castRayVsTriangleMesh_leftBounds);
            triangleMeshBvh.nodeGetBounds(buffer, rightOffset, _castRayVsTriangleMesh_rightBounds);

            const leftDist = rayDistanceToBox3(_castRayVsTriangleMesh_ray, _castRayVsTriangleMesh_leftBounds);
            const rightDist = rayDistanceToBox3(_castRayVsTriangleMesh_ray, _castRayVsTriangleMesh_rightBounds);

            // push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                // left is closer or equal - push right first
                if (rightDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsTriangleMesh_stack, rightOffset, rightDist);
                }
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsTriangleMesh_stack, leftOffset, leftDist);
                }
            } else {
                // right is closer - push left first
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsTriangleMesh_stack, leftOffset, leftDist);
                }
                if (rightDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsTriangleMesh_stack, rightOffset, rightDist);
                }
            }
        }
    }

    if (!foundHit) {
        collector.addMiss();
    }
}

/* collide point */

class HitCountCollector {
    bodyIdB = -1;
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hitCount = 0;
    lastSubShapeId = EMPTY_SUB_SHAPE_ID;

    addHit(hit: CastRayHit): void {
        this.hitCount++;
        this.lastSubShapeId = hit.subShapeId;
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return false; // never early out, count all hits
    }

    reset(): void {
        this.bodyIdB = -1;
        this.hitCount = 0;
        this.lastSubShapeId = EMPTY_SUB_SHAPE_ID;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

const _collidePointVsTriangleMesh_hitCountCollector = new HitCountCollector();
const _collidePointVsTriangleMesh_castRaySettings = createDefaultCastRaySettings();
_collidePointVsTriangleMesh_castRaySettings.collideWithBackfaces = true; // backface collision required for odd-even rule

const _collidePointVsTriangleMesh_quatB = quat.create();
const _collidePointVsTriangleMesh_localPoint = vec3.create();
const _collidePointVsTriangleMesh_ray = raycast3.create();
const _collidePointVsTriangleMesh_rayDirection = vec3.create();
const _collidePointVsTriangleMesh_aabbSize = vec3.create();
const _collidePointVsTriangleMesh_popResult: subShape.PopResult = { value: 0, remainder: subShape.EMPTY_SUB_SHAPE_ID };
const _collidePointHit = createCollidePointHit();

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
    _collidePointVsTriangleMesh_hitCountCollector.reset();
    _collidePointVsTriangleMesh_hitCountCollector.bodyIdB = collector.bodyIdB;

    // cast ray through mesh, counting all triangle intersections
    // biome-ignore format: readability
    castRayVsTriangleMesh(
        _collidePointVsTriangleMesh_hitCountCollector,
        _collidePointVsTriangleMesh_castRaySettings,
        _collidePointVsTriangleMesh_ray,
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
    const isInside = (_collidePointVsTriangleMesh_hitCountCollector.hitCount & 1) !== 0;

    if (isInside) {
        _collidePointHit.subShapeIdB = _collidePointVsTriangleMesh_hitCountCollector.lastSubShapeId;

        // extract triangle index from subShapeId to get material
        subShape.popIndex(
            _collidePointVsTriangleMesh_popResult,
            _collidePointVsTriangleMesh_hitCountCollector.lastSubShapeId,
            shapeB.data.triangleCount,
        );
        _collidePointHit.materialId = triangleMeshData.getMaterialId(shapeB.data, _collidePointVsTriangleMesh_popResult.value);

        _collidePointHit.bodyIdB = collector.bodyIdB;
        collector.addHit(_collidePointHit);
    }
}

/* cast shape */

const _castShapeHit = createCastShapeHit();
const _displacementInB = vec3.create();
const _transformedSupportA = createTransformedSupport();
const _triangleSupport = createTriangleSupport();
const _sweptAABB: Box3 = box3.create();

const _gjkResult = createGjkCastShapeResult();

const _worldPointA = vec3.create();
const _worldPointB = vec3.create();
const _displacementScaled = vec3.create();

const _posAInB = vec3.create();
const _quatAInB = quat.create();
const _positionDifference = vec3.create();
const _inverseQuaternionB = quat.create();
const _positionAtHitTime = vec3.create();
const _inverseQuatAInB = quat.create();

const _faceNormal = vec3.create();

const _activeEdgeMovementDirection = vec3.create();
const _triangleNormalForFix = vec3.create();
const _conjugateQuat = quat.create();

const _triangleA = vec3.create();
const _triangleB = vec3.create();
const _triangleC = vec3.create();
const _getTriangleVertices_a = vec3.create();
const _getTriangleVertices_b = vec3.create();
const _getTriangleVertices_c = vec3.create();
const _computeTriangleAABB_result = box3.create();
const _edgeA = vec3.create();
const _edgeB = vec3.create();
const _triangleNormal = vec3.create();
const _penetrationDifference = vec3.create();

const _posA = vec3.create();
const _quatA = quat.create();
const _scaleA = vec3.create();
const _displacementA = vec3.create();
const _posB = vec3.create();
const _quatB = quat.create();
const _scaleB = vec3.create();

const _mat4 = mat4.create();

const _bvhStack = bvhStack.create();
const _raycast = raycast3.create();
const _halfExtents = vec3.create();
const _expandedBounds = box3.create();
const _triExpandedBounds = box3.create();

const _subShapeIdBuilder = subShape.builder();

const supportPoolA = createShapeSupportPool();

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

    vec3.set(_posA, posAX, posAY, posAZ);
    quat.set(_quatA, quatAX, quatAY, quatAZ, quatAW);
    vec3.set(_scaleA, scaleAX, scaleAY, scaleAZ);
    vec3.set(_displacementA, displacementAX, displacementAY, displacementAZ);
    vec3.set(_posB, posBX, posBY, posBZ);
    quat.set(_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_scaleB, scaleBX, scaleBY, scaleBZ);

    // transform A into B's local space
    quat.conjugate(_inverseQuaternionB, _quatB);

    vec3.sub(_positionDifference, _posA, _posB);
    vec3.transformQuat(_posAInB, _positionDifference, _inverseQuaternionB);

    quat.multiply(_quatAInB, _inverseQuaternionB, _quatA);

    vec3.transformQuat(_displacementInB, _displacementA, _inverseQuaternionB);

    // compute base AABB of shape A at t=0 in mesh local space
    const aabbMatrix = mat4.fromRotationTranslationScale(_mat4, _quatAInB, _posAInB, _scaleA);
    box3.transformMat4(_sweptAABB, shapeA.aabb, aabbMatrix);

    // determine if we want to use the actual shape or a shrunken shape with convex radius
    const supportMode = settings.useShrunkenShapeAndConvexRadius
        ? SupportFunctionMode.EXCLUDE_CONVEX_RADIUS
        : SupportFunctionMode.DEFAULT;

    // get transformed support function for convex shape
    const supportA = getShapeSupportFunction(supportPoolA, shapeA, supportMode, _scaleA);
    setTransformedSupport(_transformedSupportA, _posAInB, _quatAInB, supportA);

    // determine if shape is inside out or not
    const scaleSign = vec3.isScaleInsideOut(_scaleB) ? -1 : 1;

    // inline BVH traversal for performance
    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    if (buffer.length === 0) {
        return;
    }

    // compute centroid of base AABB
    const ray = _raycast;
    ray.origin[0] = (_sweptAABB[0][0] + _sweptAABB[1][0]) * 0.5;
    ray.origin[1] = (_sweptAABB[0][1] + _sweptAABB[1][1]) * 0.5;
    ray.origin[2] = (_sweptAABB[0][2] + _sweptAABB[1][2]) * 0.5;

    // compute ray direction and length from displacement
    ray.length = vec3.length(_displacementInB);
    if (ray.length > 1e-10) {
        vec3.normalize(ray.direction, _displacementInB);
    } else {
        ray.direction[0] = 0;
        ray.direction[1] = 0;
        ray.direction[2] = 0;
    }

    // compute half-extents of the base AABB
    const halfExtents = _halfExtents;
    halfExtents[0] = (_sweptAABB[1][0] - _sweptAABB[0][0]) * 0.5;
    halfExtents[1] = (_sweptAABB[1][1] - _sweptAABB[0][1]) * 0.5;
    halfExtents[2] = (_sweptAABB[1][2] - _sweptAABB[0][2]) * 0.5;

    bvhStack.reset(_bvhStack);
    bvhStack.push(_bvhStack, 0, 0); // root always visited

    const expandedBounds = _expandedBounds;

    // let nodesVisited = 0;
    // let gjkCastsPerformed = 0;

    while (_bvhStack.size > 0) {
        const entry = bvhStack.pop(_bvhStack)!;

        // early out: if fraction to this node >= closest hit, skip it
        if (entry.distance >= collector.earlyOutFraction) {
            continue;
        }

        const nodeOffset = entry.nodeIndex;

        // nodesVisited++;

        // note: no need to test ray intersection here - we already proved the ray
        // intersects this node's expanded bounds when we computed the distance during push.
        // if the ray didn't intersect, rayDistanceToBox3 would have returned Infinity
        // and we wouldn't have pushed this node onto the stack.

        if (triangleMeshBvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf: test triangles
            const triStart = triangleMeshBvh.nodeTriStart(buffer, nodeOffset);
            const triCount = triangleMeshBvh.nodeTriCount(buffer, nodeOffset);
            for (let i = 0; i < triCount; i++) {
                const triangleIndex = triStart + i;

                // compute triangle aabb
                calculateTriangleAABB(_computeTriangleAABB_result, meshData, triangleIndex);
                const triBounds = _triExpandedBounds;
                box3.copy(triBounds, _computeTriangleAABB_result);

                // expand by half-extents
                triBounds[0][0] -= halfExtents[0];
                triBounds[0][1] -= halfExtents[1];
                triBounds[0][2] -= halfExtents[2];
                triBounds[1][0] += halfExtents[0];
                triBounds[1][1] += halfExtents[1];
                triBounds[1][2] += halfExtents[2];

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
                    _getTriangleVertices_a,
                    _getTriangleVertices_b,
                    _getTriangleVertices_c,
                    meshData,
                    triangleIndex,
                );

                // scale triangle
                const a = vec3.mul(_triangleA, _getTriangleVertices_a, _scaleB);
                const b = vec3.mul(_triangleB, _getTriangleVertices_b, _scaleB);
                const c = vec3.mul(_triangleC, _getTriangleVertices_c, _scaleB);

                // calculate scaled triangle normal
                const normal = vec3.scale(
                    _triangleNormal,
                    vec3.cross(_triangleNormal, vec3.sub(_edgeA, b, a), vec3.sub(_edgeB, c, a)),
                    scaleSign,
                );

                // backface check
                if (!settings.collideWithBackfaces && vec3.dot(normal, _displacementInB) > 0) {
                    continue;
                }

                // set triangle support function
                setTriangleSupport(_triangleSupport, a, b, c);

                // gjk shapecast with epa fallback for deep penetration
                // gjkCastsPerformed++;
                penetrationCastShape(
                    _gjkResult,
                    _posAInB,
                    _quatAInB,
                    supportA,
                    _triangleSupport,
                    _displacementInB,
                    settings.collisionTolerance,
                    settings.penetrationTolerance,
                    supportA.convexRadius,
                    0, // triangle has no convex radius
                    collector.earlyOutFraction,
                    settings.returnDeepestPoint,
                );

                // check if hit found
                if (!_gjkResult.hit) {
                    continue;
                }

                const fraction = _gjkResult.lambda;
                const penetrationDepth = vec3.length(vec3.sub(_penetrationDifference, _gjkResult.pointA, _gjkResult.pointB));

                // early out: if this hit is deeper than the collector's early out value
                if (fraction === 0 && -penetrationDepth >= collector.earlyOutFraction) {
                    continue;
                }

                // active edge detection - correct normal if hitting inactive edge
                const triangleActiveEdges = getActiveEdges(meshData, triangleIndex);
                if (settings.collideOnlyWithActiveEdges && triangleActiveEdges !== 0b111) {
                    // transform movement direction from world space to mesh's local space
                    vec3.transformQuat(
                        _activeEdgeMovementDirection,
                        settings.activeEdgeMovementDirection,
                        quat.conjugate(_conjugateQuat, _quatB),
                    );

                    // prepare triangle normal for fixNormal
                    // back-facing check: if displacement dot normal > 0, we're approaching from back
                    const backFacing = !settings.collideWithBackfaces || vec3.dot(normal, _displacementInB) > 0;
                    if (backFacing) {
                        vec3.copy(_triangleNormalForFix, normal);
                    } else {
                        vec3.negate(_triangleNormalForFix, normal);
                    }

                    // apply active edge correction (all parameters in mesh's local space)
                    const correctedAxis = activeEdges.fixNormal(
                        a,
                        b,
                        c,
                        _triangleNormalForFix,
                        triangleActiveEdges,
                        _gjkResult.pointB,
                        _gjkResult.separatingAxis,
                        _activeEdgeMovementDirection,
                    );

                    vec3.copy(_gjkResult.separatingAxis, correctedAxis);
                }

                // convert to world space
                vec3.transformQuat(_worldPointA, _gjkResult.pointA, _quatB);
                vec3.add(_worldPointA, _worldPointA, _posB);
                vec3.transformQuat(_worldPointB, _gjkResult.pointB, _quatB);
                vec3.add(_worldPointB, _worldPointB, _posB);
                vec3.transformQuat(_castShapeHit.penetrationAxis, _gjkResult.separatingAxis, _quatB);

                // store hit info
                _castShapeHit.status = CastShapeStatus.COLLIDING;
                _castShapeHit.fraction = _gjkResult.lambda;
                vec3.copy(_castShapeHit.pointA, _worldPointA);
                vec3.copy(_castShapeHit.pointB, _worldPointB);
                vec3.normalize(_castShapeHit.normal, _castShapeHit.penetrationAxis);
                vec3.negate(_castShapeHit.normal, _castShapeHit.normal);
                _castShapeHit.penetrationDepth = penetrationDepth;
                _castShapeHit.subShapeIdA = subShapeIdA;

                _subShapeIdBuilder.value = _subShapeIdB;
                _subShapeIdBuilder.currentBit = _subShapeIdBitsB;
                subShape.pushIndex(_subShapeIdBuilder, _subShapeIdBuilder, triangleIndex, meshData.triangleCount);

                _castShapeHit.subShapeIdB = _subShapeIdBuilder.value;
                _castShapeHit.materialIdA = (shapeA as ConvexShape).materialId;
                _castShapeHit.materialIdB = triangleMeshData.getMaterialId(meshData, triangleIndex);
                _castShapeHit.bodyIdB = collector.bodyIdB;

                // gather faces if requested
                if (settings.collectFaces) {
                    // face a: transform contact normal from B's local space to A's local space
                    const scaledDisplacement = vec3.scale(_displacementScaled, _displacementA, _gjkResult.lambda);
                    const position = vec3.add(_positionAtHitTime, _posA, scaledDisplacement);
                    const normalInA = vec3.negate(_faceNormal, _gjkResult.separatingAxis);
                    quat.conjugate(_inverseQuatAInB, _quatAInB);
                    vec3.transformQuat(normalInA, normalInA, _inverseQuatAInB);
                    getShapeSupportingFace(_castShapeHit.faceA, shapeA, subShapeIdA, normalInA, position, _quatA, _scaleA);

                    // face b: supporting face is the triangle vertices (already in B's local space)
                    _castShapeHit.faceB.numVertices = 3;
                    _castShapeHit.faceB.vertices[0] = a[0];
                    _castShapeHit.faceB.vertices[1] = a[1];
                    _castShapeHit.faceB.vertices[2] = a[2];
                    _castShapeHit.faceB.vertices[3] = b[0];
                    _castShapeHit.faceB.vertices[4] = b[1];
                    _castShapeHit.faceB.vertices[5] = b[2];
                    _castShapeHit.faceB.vertices[6] = c[0];
                    _castShapeHit.faceB.vertices[7] = c[1];
                    _castShapeHit.faceB.vertices[8] = c[2];
                    transformFace(_castShapeHit.faceB, _posB, _quatB, _scaleB);
                } else {
                    // clear faces
                    _castShapeHit.faceA.numVertices = 0;
                    _castShapeHit.faceB.numVertices = 0;
                }

                // add hit
                collector.addHit(_castShapeHit);
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            // ray from swept AABB center along displacement
            const leftOffset = triangleMeshBvh.nodeLeft(nodeOffset);
            const rightOffset = triangleMeshBvh.nodeRight(buffer, nodeOffset);

            // expand bounds by half-extents (same as triangle test)
            expandedBounds[0][0] = buffer[leftOffset + triangleMeshBvh.NODE_MIN_X] - halfExtents[0];
            expandedBounds[0][1] = buffer[leftOffset + triangleMeshBvh.NODE_MIN_Y] - halfExtents[1];
            expandedBounds[0][2] = buffer[leftOffset + triangleMeshBvh.NODE_MIN_Z] - halfExtents[2];
            expandedBounds[1][0] = buffer[leftOffset + triangleMeshBvh.NODE_MAX_X] + halfExtents[0];
            expandedBounds[1][1] = buffer[leftOffset + triangleMeshBvh.NODE_MAX_Y] + halfExtents[1];
            expandedBounds[1][2] = buffer[leftOffset + triangleMeshBvh.NODE_MAX_Z] + halfExtents[2];
            const leftDist = rayDistanceToBox3(ray, expandedBounds);

            expandedBounds[0][0] = buffer[rightOffset + triangleMeshBvh.NODE_MIN_X] - halfExtents[0];
            expandedBounds[0][1] = buffer[rightOffset + triangleMeshBvh.NODE_MIN_Y] - halfExtents[1];
            expandedBounds[0][2] = buffer[rightOffset + triangleMeshBvh.NODE_MIN_Z] - halfExtents[2];
            expandedBounds[1][0] = buffer[rightOffset + triangleMeshBvh.NODE_MAX_X] + halfExtents[0];
            expandedBounds[1][1] = buffer[rightOffset + triangleMeshBvh.NODE_MAX_Y] + halfExtents[1];
            expandedBounds[1][2] = buffer[rightOffset + triangleMeshBvh.NODE_MAX_Z] + halfExtents[2];
            const rightDist = rayDistanceToBox3(ray, expandedBounds);

            // sort: push farther child first (so closer child is on top of stack), lifo traversal
            if (leftDist <= rightDist) {
                // left is closer or equal - push right first
                if (rightDist < collector.earlyOutFraction) {
                    bvhStack.push(_bvhStack, rightOffset, rightDist);
                }
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_bvhStack, leftOffset, leftDist);
                }
            } else {
                // right is closer - push left first
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_bvhStack, leftOffset, leftDist);
                }
                if (rightDist < collector.earlyOutFraction) {
                    bvhStack.push(_bvhStack, rightOffset, rightDist);
                }
            }
        }
    }

    // console.log(`[castConvexVsTriangleMesh] Nodes visited: ${nodesVisited}, GJK casts performed: ${gjkCastsPerformed}`);
}

const castTriangleMeshVsConvex = reversedCastShapeVsShape(castConvexVsTriangleMesh);

/* collide shape */

const _collideShapeHit = createCollideShapeHit();

const _temp_faceDirA = vec3.create();

const _simplex = createSimplex();
const _penetrationDepth = createPenetrationDepth();

const _supportPoolA = createShapeSupportPool();

const _addRadiusSupport = createAddConvexRadiusSupport();

const _penetrationAxis = vec3.create();
const _vectorAB = vec3.create();

const _inverseQuatA = quat.create();

const _aabbShapeExpand = vec3.create();

const _inverseQuatB = quat.create();
const _boundsOf1InSpaceOf2 = box3.create();
const _boundsOf1 = box3.create();
const _transform2To1Pos = vec3.create();
const _transform2To1Quat = quat.create();

const _triangleA_inA = vec3.create();
const _triangleB_inA = vec3.create();
const _triangleC_inA = vec3.create();

const _collideConvexVsTriangleMesh_stack = bvhStack.create();
const _collideConvexVsTriangleMesh_queryCenter = vec3.create();
const _collideConvexVsTriangleMesh_nodeCenter = vec3.create();
const _collideConvexVsTriangleMesh_triangleAABB = box3.create();

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

    vec3.set(_posA, posAX, posAY, posAZ);
    quat.set(_quatA, quatAX, quatAY, quatAZ, quatAW);
    vec3.set(_scaleA, scaleAX, scaleAY, scaleAZ);

    vec3.set(_posB, posBX, posBY, posBZ);
    quat.set(_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_scaleB, scaleBX, scaleBY, scaleBZ);

    // calculate transforms
    // transform B (mesh) into A's (convex) local space for triangle tests
    quat.conjugate(_inverseQuatA, _quatA);
    quat.multiply(_transform2To1Quat, _inverseQuatA, _quatB);
    vec3.subtract(_vectorAB, _posB, _posA);
    vec3.transformQuat(_transform2To1Pos, _vectorAB, _inverseQuatA);

    // transform A into B's space for BVH query
    quat.conjugate(_inverseQuatB, _quatB);
    quat.multiply(_quatAInB, _inverseQuatB, _quatA);
    vec3.subtract(_positionDifference, _posA, _posB);
    vec3.transformQuat(_posAInB, _positionDifference, _inverseQuatB);

    // compute shape A's bounds in its own local space
    const boundsOf1 = box3.copy(_boundsOf1, shapeA.aabb);
    box3.scale(boundsOf1, boundsOf1, _scaleA);
    box3.expandByExtents(boundsOf1, boundsOf1, vec3.setScalar(_aabbShapeExpand, settings.maxSeparationDistance));

    // compute shape A's bounds in shape B's space for BVH culling
    const aabbMatrix = mat4.fromRotationTranslationScale(_mat4, _quatAInB, _posAInB, _scaleA);
    box3.transformMat4(_boundsOf1InSpaceOf2, shapeA.aabb, aabbMatrix);
    box3.expandByExtents(
        _boundsOf1InSpaceOf2,
        _boundsOf1InSpaceOf2,
        vec3.setScalar(_aabbShapeExpand, settings.maxSeparationDistance),
    );

    // determine if mesh is inside-out
    const scaleSign = vec3.isScaleInsideOut(_scaleB) ? -1 : 1;

    // get support function for shape A
    const supportA = getShapeSupportFunction(_supportPoolA, shapeA, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, _scaleA);

    // inline BVH traversal for performance
    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    if (buffer.length === 0) {
        return;
    }

    bvhStack.reset(_collideConvexVsTriangleMesh_stack);

    // compute query shape center in B's space for distance-based sorting
    // use center-to-center distance heuristic
    box3.center(_collideConvexVsTriangleMesh_queryCenter, _boundsOf1InSpaceOf2);

    bvhStack.push(_collideConvexVsTriangleMesh_stack, 0, 0); // root always visited (distance 0)

    while (_collideConvexVsTriangleMesh_stack.size > 0) {
        const entry = bvhStack.pop(_collideConvexVsTriangleMesh_stack)!;
        const nodeOffset = entry.nodeIndex;
        // distance not used for filtering in shape queries

        // skip if node bounds don't intersect query
        if (
            !triangleMeshBvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                _boundsOf1InSpaceOf2[0][0],
                _boundsOf1InSpaceOf2[0][1],
                _boundsOf1InSpaceOf2[0][2],
                _boundsOf1InSpaceOf2[1][0],
                _boundsOf1InSpaceOf2[1][1],
                _boundsOf1InSpaceOf2[1][2],
            )
        ) {
            continue;
        }

        if (triangleMeshBvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf node: check triangles
            const triStart = triangleMeshBvh.nodeTriStart(buffer, nodeOffset);
            const triCount = triangleMeshBvh.nodeTriCount(buffer, nodeOffset);
            for (let i = 0; i < triCount; i++) {
                const triangleIndex = triStart + i;

                // get triangle vertices
                getTriangleVertices(
                    _getTriangleVertices_a,
                    _getTriangleVertices_b,
                    _getTriangleVertices_c,
                    meshData,
                    triangleIndex,
                );

                // scale triangle in mesh local space
                const a = vec3.mul(_triangleA, _getTriangleVertices_a, _scaleB);
                const b = vec3.mul(_triangleB, _getTriangleVertices_b, _scaleB);
                const c = vec3.mul(_triangleC, _getTriangleVertices_c, _scaleB);

                // transform triangle to shape A's local space
                vec3.transformQuat(_triangleA_inA, a, _transform2To1Quat);
                vec3.add(_triangleA_inA, _triangleA_inA, _transform2To1Pos);
                vec3.transformQuat(_triangleB_inA, b, _transform2To1Quat);
                vec3.add(_triangleB_inA, _triangleB_inA, _transform2To1Pos);
                vec3.transformQuat(_triangleC_inA, c, _transform2To1Quat);
                vec3.add(_triangleC_inA, _triangleC_inA, _transform2To1Pos);

                // compute triangle AABB in shape A's local space
                const triangleAABB = _collideConvexVsTriangleMesh_triangleAABB;
                triangle3.bounds(triangleAABB, _triangleA_inA, _triangleB_inA, _triangleC_inA);

                // early out: if triangle AABB doesn't overlap shape AABB, skip this triangle
                if (!box3.intersectsBox3(triangleAABB, _boundsOf1)) {
                    continue;
                }

                vec3.sub(_edgeA, _triangleB_inA, _triangleA_inA);
                vec3.sub(_edgeB, _triangleC_inA, _triangleA_inA);

                // calculate triangle normal in A's local space
                const normal = vec3.scale(_triangleNormal, vec3.cross(_triangleNormal, _edgeA, _edgeB), scaleSign);

                // back-face check
                // check if triangle normal and first vertex both point in same direction from origin
                // (shape A is at origin in its local space)
                const backFacing = vec3.dot(normal, _triangleA_inA) > 0.0;
                if (!settings.collideWithBackfaces && backFacing) {
                    continue;
                }

                // create triangle support function
                setTriangleSupport(_triangleSupport, _triangleA_inA, _triangleB_inA, _triangleC_inA);

                // run GJK with negative triangle normal as initial penetration axis
                // (likely that shape A is in front of triangle B)
                const penetrationAxis = vec3.negate(_penetrationAxis, normal);

                // ensure non-zero penetration axis
                if (vec3.squaredLength(penetrationAxis) < 1e-10) {
                    vec3.set(penetrationAxis, 1, 0, 0);
                } else {
                    vec3.normalize(penetrationAxis, penetrationAxis);
                }

                // perform GJK step with inflated shape (convex radius + max separation distance)
                let maxSeparationDistance = settings.maxSeparationDistance;
                penetrationDepthStepGJK(
                    _penetrationDepth,
                    _simplex,
                    supportA,
                    _triangleSupport,
                    supportA.convexRadius + maxSeparationDistance,
                    0, // triangle has no convex radius
                    penetrationAxis,
                    settings.collisionTolerance,
                );

                // check result of collision detection
                if (_penetrationDepth.status === PenetrationDepthStatus.NOT_COLLIDING) {
                    continue;
                }

                if (_penetrationDepth.status === PenetrationDepthStatus.INDETERMINATE) {
                    // need to run expensive EPA algorithm
                    // clamp max separation distance to avoid excessive inflation
                    maxSeparationDistance = Math.min(maxSeparationDistance, 1.0);

                    // get support function including convex radius for EPA
                    const supportAWithRadius = getShapeSupportFunction(
                        _supportPoolA,
                        shapeA,
                        SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
                        _scaleA,
                    );

                    // add separation distance
                    setAddConvexRadiusSupport(_addRadiusSupport, maxSeparationDistance, supportAWithRadius);

                    // perform EPA step
                    if (
                        !penetrationDepthStepEPA(
                            _penetrationDepth,
                            _addRadiusSupport,
                            _triangleSupport,
                            settings.penetrationTolerance,
                            _simplex,
                        )
                    ) {
                        continue;
                    }
                }

                // calculate penetration depth (subtract the inflation from the distance)
                const penetration = vec3.distance(_penetrationDepth.pointA, _penetrationDepth.pointB) - maxSeparationDistance;

                // check if penetration exceeds early-out threshold
                if (-penetration >= collector.earlyOutFraction) {
                    continue;
                }

                // correct point A by moving it back along penetration axis to account for the max separation distance
                const penetrationAxisLen = vec3.length(_penetrationDepth.penetrationAxis);
                if (penetrationAxisLen > 0.0) {
                    const correction = maxSeparationDistance / penetrationAxisLen;
                    vec3.scaleAndAdd(
                        _penetrationDepth.pointA,
                        _penetrationDepth.pointA,
                        _penetrationDepth.penetrationAxis,
                        -correction,
                    );
                }

                // active edge detection - correct normal if hitting inactive edge
                const triangleActiveEdges = getActiveEdges(meshData, triangleIndex);
                if (settings.collideOnlyWithActiveEdges && triangleActiveEdges !== 0b111) {
                    // transform movement direction from world space to shape A's local space
                    vec3.transformQuat(
                        _activeEdgeMovementDirection,
                        settings.activeEdgeMovementDirection,
                        quat.conjugate(_conjugateQuat, _quatA),
                    );

                    // prepare triangle normal for fixNormal
                    if (backFacing) {
                        vec3.copy(_triangleNormalForFix, normal);
                    } else {
                        vec3.negate(_triangleNormalForFix, normal);
                    }

                    // apply active edge correction (all parameters in shape A's local space)
                    const correctedAxis = activeEdges.fixNormal(
                        _triangleA_inA,
                        _triangleB_inA,
                        _triangleC_inA,
                        _triangleNormalForFix,
                        triangleActiveEdges,
                        _penetrationDepth.pointB,
                        _penetrationDepth.penetrationAxis,
                        _activeEdgeMovementDirection,
                    );

                    vec3.copy(_penetrationDepth.penetrationAxis, correctedAxis);
                }

                // transform results to world space
                vec3.transformQuat(_worldPointA, _penetrationDepth.pointA, _quatA);
                vec3.add(_worldPointA, _worldPointA, _posA);
                vec3.transformQuat(_worldPointB, _penetrationDepth.pointB, _quatA);
                vec3.add(_worldPointB, _worldPointB, _posA);
                vec3.transformQuat(_collideShapeHit.penetrationAxis, _penetrationDepth.penetrationAxis, _quatA);

                // push sub shape id for triangle
                _subShapeIdBuilder.value = subShapeIdB;
                _subShapeIdBuilder.currentBit = subShapeIdBitsB;
                subShape.pushIndex(_subShapeIdBuilder, _subShapeIdBuilder, triangleIndex, meshShape.data.triangleCount);

                // report collision
                vec3.copy(_collideShapeHit.pointA, _worldPointA);
                vec3.copy(_collideShapeHit.pointB, _worldPointB);
                _collideShapeHit.penetration = penetration;
                _collideShapeHit.subShapeIdA = subShapeIdA;
                _collideShapeHit.subShapeIdB = _subShapeIdBuilder.value;
                _collideShapeHit.materialIdA = (shapeA as ConvexShape).materialId;
                _collideShapeHit.materialIdB = triangleMeshData.getMaterialId(meshShape.data, triangleIndex);
                _collideShapeHit.bodyIdB = collector.bodyIdB;

                // collect faces if requested
                if (settings.collectFaces) {
                    // direction for shape A: opposite of penetration axis (local space)
                    const faceDirectionA = vec3.negate(_temp_faceDirA, _penetrationDepth.penetrationAxis);
                    getShapeSupportingFace(_collideShapeHit.faceA, shapeA, subShapeIdA, faceDirectionA, _posA, _quatA, _scaleA);

                    // shape B face: triangle has 3 vertices in shape A's local space
                    _collideShapeHit.faceB.numVertices = 3;
                    _collideShapeHit.faceB.vertices[0] = _triangleA_inA[0];
                    _collideShapeHit.faceB.vertices[1] = _triangleA_inA[1];
                    _collideShapeHit.faceB.vertices[2] = _triangleA_inA[2];
                    _collideShapeHit.faceB.vertices[3] = _triangleB_inA[0];
                    _collideShapeHit.faceB.vertices[4] = _triangleB_inA[1];
                    _collideShapeHit.faceB.vertices[5] = _triangleB_inA[2];
                    _collideShapeHit.faceB.vertices[6] = _triangleC_inA[0];
                    _collideShapeHit.faceB.vertices[7] = _triangleC_inA[1];
                    _collideShapeHit.faceB.vertices[8] = _triangleC_inA[2];

                    // transform from shape A's local space to world space
                    transformFace(_collideShapeHit.faceB, _posA, _quatA, _scaleA);
                }

                collector.addHit(_collideShapeHit);
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            // use center-to-center distance heuristic
            const leftOffset = triangleMeshBvh.nodeLeft(nodeOffset);
            const rightOffset = triangleMeshBvh.nodeRight(buffer, nodeOffset);

            // compute squared distances from query center to child node centers
            triangleMeshBvh.nodeGetCenter(buffer, leftOffset, _collideConvexVsTriangleMesh_nodeCenter);
            const leftDist = vec3.squaredDistance(
                _collideConvexVsTriangleMesh_queryCenter,
                _collideConvexVsTriangleMesh_nodeCenter,
            );

            triangleMeshBvh.nodeGetCenter(buffer, rightOffset, _collideConvexVsTriangleMesh_nodeCenter);
            const rightDist = vec3.squaredDistance(
                _collideConvexVsTriangleMesh_queryCenter,
                _collideConvexVsTriangleMesh_nodeCenter,
            );

            // sort: push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                // left is closer or equal - push right first
                bvhStack.push(_collideConvexVsTriangleMesh_stack, rightOffset, rightDist);
                bvhStack.push(_collideConvexVsTriangleMesh_stack, leftOffset, leftDist);
            } else {
                // right is closer - push left first
                bvhStack.push(_collideConvexVsTriangleMesh_stack, leftOffset, leftDist);
                bvhStack.push(_collideConvexVsTriangleMesh_stack, rightOffset, rightDist);
            }
        }
    }
}

const collideTriangleMeshVsConvex = reversedCollideShapeVsShape(collideConvexVsTriangleMesh);

/* collide sphere vs triangle mesh */

const _collideSphereVsTriangleMesh_sphereCenterInMesh = vec3.create();
const _collideSphereVsTriangleMesh_posB = vec3.create();
const _collideSphereVsTriangleMesh_quatB = quat.create();
const _collideSphereVsTriangleMesh_scaleB = vec3.create();
const _collideSphereVsTriangleMesh_inverseQuatB = quat.create();
const _collideSphereVsTriangleMesh_positionDifference = vec3.create();
const _collideSphereVsTriangleMesh_boundsOfSphere = box3.create();
const _collideSphereVsTriangleMesh_queryCenter = vec3.create();
const _collideSphereVsTriangleMesh_nodeCenter = vec3.create();
const _collideSphereVsTriangleMesh_stack = bvhStack.create();
const _collideSphereVsTriangleMesh_v0 = vec3.create();
const _collideSphereVsTriangleMesh_v1 = vec3.create();
const _collideSphereVsTriangleMesh_v2 = vec3.create();
const _collideSphereVsTriangleMesh_sv0 = vec3.create();
const _collideSphereVsTriangleMesh_sv1 = vec3.create();
const _collideSphereVsTriangleMesh_sv2 = vec3.create();
const _collideSphereVsTriangleMesh_rv0 = vec3.create();
const _collideSphereVsTriangleMesh_rv1 = vec3.create();
const _collideSphereVsTriangleMesh_rv2 = vec3.create();
const _collideSphereVsTriangleMesh_edge1 = vec3.create();
const _collideSphereVsTriangleMesh_edge2 = vec3.create();
const _collideSphereVsTriangleMesh_triangleNormal = vec3.create();
const _collideSphereVsTriangleMesh_closestPointResult = createClosestPointOnTriangleResult();
const _collideSphereVsTriangleMesh_penetrationAxis = vec3.create();
const _collideSphereVsTriangleMesh_point1 = vec3.create();
const _collideSphereVsTriangleMesh_point1World = vec3.create();
const _collideSphereVsTriangleMesh_point2World = vec3.create();
const _collideSphereVsTriangleMesh_penetrationAxisWorld = vec3.create();
const _collideSphereVsTriangleMesh_activeEdgeMovementDir = vec3.create();
const _collideSphereVsTriangleMesh_newPenetrationAxis = vec3.create();
const _collideSphereVsTriangleMesh_hit = createCollideShapeHit();
const _collideSphereVsTriangleMesh_subShapeIdBuilder = subShape.builder();

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
    const sphereShape = shapeA as any; // SphereShape
    const meshShape = shapeB as TriangleMeshShape;

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
    sphereBounds[0][0] = sphereCenterInMesh[0] - expandedRadius;
    sphereBounds[0][1] = sphereCenterInMesh[1] - expandedRadius;
    sphereBounds[0][2] = sphereCenterInMesh[2] - expandedRadius;
    sphereBounds[1][0] = sphereCenterInMesh[0] + expandedRadius;
    sphereBounds[1][1] = sphereCenterInMesh[1] + expandedRadius;
    sphereBounds[1][2] = sphereCenterInMesh[2] + expandedRadius;

    // BVH traversal
    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    if (buffer.length === 0) {
        return;
    }

    bvhStack.reset(_collideSphereVsTriangleMesh_stack);

    // use sphere center for distance-based sorting
    vec3.copy(_collideSphereVsTriangleMesh_queryCenter, sphereCenterInMesh);

    bvhStack.push(_collideSphereVsTriangleMesh_stack, 0, 0); // root

    while (_collideSphereVsTriangleMesh_stack.size > 0) {
        const entry = bvhStack.pop(_collideSphereVsTriangleMesh_stack)!;
        const nodeOffset = entry.nodeIndex;

        // skip if node bounds don't intersect sphere
        if (
            !triangleMeshBvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                sphereBounds[0][0],
                sphereBounds[0][1],
                sphereBounds[0][2],
                sphereBounds[1][0],
                sphereBounds[1][1],
                sphereBounds[1][2],
            )
        ) {
            continue;
        }

        if (triangleMeshBvh.nodeIsLeaf(buffer, nodeOffset)) {
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
            const leftOffset = triangleMeshBvh.nodeLeft(nodeOffset);
            const rightOffset = triangleMeshBvh.nodeRight(buffer, nodeOffset);

            // compute squared distances from sphere center to child node centers
            triangleMeshBvh.nodeGetCenter(buffer, leftOffset, _collideSphereVsTriangleMesh_nodeCenter);
            const leftDist = vec3.squaredDistance(
                _collideSphereVsTriangleMesh_queryCenter,
                _collideSphereVsTriangleMesh_nodeCenter,
            );

            triangleMeshBvh.nodeGetCenter(buffer, rightOffset, _collideSphereVsTriangleMesh_nodeCenter);
            const rightDist = vec3.squaredDistance(
                _collideSphereVsTriangleMesh_queryCenter,
                _collideSphereVsTriangleMesh_nodeCenter,
            );

            // sort: push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                bvhStack.push(_collideSphereVsTriangleMesh_stack, rightOffset, rightDist);
                bvhStack.push(_collideSphereVsTriangleMesh_stack, leftOffset, leftDist);
            } else {
                bvhStack.push(_collideSphereVsTriangleMesh_stack, leftOffset, leftDist);
                bvhStack.push(_collideSphereVsTriangleMesh_stack, rightOffset, rightDist);
            }
        }
    }
}

const collideTriangleMeshVsSphere = reversedCollideShapeVsShape(collideSphereVsTriangleMesh);

/* cast sphere vs triangle mesh */

const _castSphereVsTriangleMesh_start = vec3.create();
const _castSphereVsTriangleMesh_direction = vec3.create();
const _castSphereVsTriangleMesh_posB = vec3.create();
const _castSphereVsTriangleMesh_quatB = quat.create();
const _castSphereVsTriangleMesh_scaleB = vec3.create();
const _castSphereVsTriangleMesh_inverseQuatB = quat.create();
const _castSphereVsTriangleMesh_positionDifference = vec3.create();
const _castSphereVsTriangleMesh_displacement = vec3.create();
const _castSphereVsTriangleMesh_sweptBounds = box3.create();
const _castSphereVsTriangleMesh_queryCenter = vec3.create();
const _castSphereVsTriangleMesh_nodeCenter = vec3.create();
const _castSphereVsTriangleMesh_stack = bvhStack.create();
const _castSphereVsTriangleMesh_v0 = vec3.create();
const _castSphereVsTriangleMesh_v1 = vec3.create();
const _castSphereVsTriangleMesh_v2 = vec3.create();
const _castSphereVsTriangleMesh_sv0 = vec3.create();
const _castSphereVsTriangleMesh_sv1 = vec3.create();
const _castSphereVsTriangleMesh_sv2 = vec3.create();
const _castSphereVsTriangleMesh_rv0 = vec3.create();
const _castSphereVsTriangleMesh_rv1 = vec3.create();
const _castSphereVsTriangleMesh_rv2 = vec3.create();
const _castSphereVsTriangleMesh_edge1 = vec3.create();
const _castSphereVsTriangleMesh_edge2 = vec3.create();
const _castSphereVsTriangleMesh_triangleNormal = vec3.create();
const _castSphereVsTriangleMesh_closestPointResult = createClosestPointOnTriangleResult();
const _castSphereVsTriangleMesh_hit = createCastShapeHit();
const _castSphereVsTriangleMesh_subShapeIdBuilder = subShape.builder();
const _castSphereVsTriangleMesh_activeEdgeMovementDir = vec3.create();
const _castSphereVsTriangleMesh_origin = vec3.create();
const _castSphereVsTriangleMesh_endPoint = vec3.create();
const _castSphereVsTriangleMesh_triangleNormalForFix = vec3.create();
const _castSphereVsTriangleMesh_contactPointAWorld = vec3.create();
const _castSphereVsTriangleMesh_contactPointBWorld = vec3.create();
const _castSphereVsTriangleMesh_contactNormalWorld = vec3.create();
const _castSphereVsTriangleMesh_planeIntersectionTemp = vec3.create();
const _castSphereVsTriangleMesh_interiorContactNormal = vec3.create();
const _castSphereVsTriangleMesh_sphereCenterAtHit = vec3.create();
const _castSphereVsTriangleMesh_v0RelativeToHit = vec3.create();
const _castSphereVsTriangleMesh_v1RelativeToHit = vec3.create();
const _castSphereVsTriangleMesh_v2RelativeToHit = vec3.create();
const _castSphereVsTriangleMesh_edgeVertexContactNormal = vec3.create();
const _castSphereVsTriangleMesh_edgeVertexContactPoint = vec3.create();
const _castSphereVsTriangleMesh_contactNormal = vec3.create();
const _castSphereVsTriangleMesh_contactPointA = vec3.create();
const _castSphereVsTriangleMesh_d = vec3.create();
const _castSphereVsTriangleMesh_v0Rel = vec3.create();
const _castSphereVsTriangleMesh_v1Rel = vec3.create();
const _castSphereVsTriangleMesh_v2Rel = vec3.create();
const _castSphereVsTriangleMesh_n = vec3.create();
const _castSphereVsTriangleMesh_n0 = vec3.create();
const _castSphereVsTriangleMesh_n1 = vec3.create();
const _castSphereVsTriangleMesh_n2 = vec3.create();
const _castSphereVsTriangleMesh_p = vec3.create();

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
    const sphereShape = shapeA as any; // SphereShape
    const meshShape = shapeB as TriangleMeshShape;

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

    // create swept AABB for BVH culling
    const sweptBounds = _castSphereVsTriangleMesh_sweptBounds;
    const endPoint = vec3.add(_castSphereVsTriangleMesh_endPoint, start, direction);

    sweptBounds[0][0] = Math.min(start[0], endPoint[0]) - sphereRadius;
    sweptBounds[0][1] = Math.min(start[1], endPoint[1]) - sphereRadius;
    sweptBounds[0][2] = Math.min(start[2], endPoint[2]) - sphereRadius;
    sweptBounds[1][0] = Math.max(start[0], endPoint[0]) + sphereRadius;
    sweptBounds[1][1] = Math.max(start[1], endPoint[1]) + sphereRadius;
    sweptBounds[1][2] = Math.max(start[2], endPoint[2]) + sphereRadius;

    // BVH traversal
    const buffer = meshShape.bvh.buffer;
    const meshData = meshShape.data;

    if (buffer.length === 0) {
        return;
    }

    bvhStack.reset(_castSphereVsTriangleMesh_stack);
    box3.center(_castSphereVsTriangleMesh_queryCenter, sweptBounds);
    bvhStack.push(_castSphereVsTriangleMesh_stack, 0, 0); // root

    while (_castSphereVsTriangleMesh_stack.size > 0) {
        const entry = bvhStack.pop(_castSphereVsTriangleMesh_stack)!;
        const nodeOffset = entry.nodeIndex;

        // skip if node bounds don't intersect swept sphere
        if (
            !triangleMeshBvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                sweptBounds[0][0],
                sweptBounds[0][1],
                sweptBounds[0][2],
                sweptBounds[1][0],
                sweptBounds[1][1],
                sweptBounds[1][2],
            )
        ) {
            continue;
        }

        if (triangleMeshBvh.nodeIsLeaf(buffer, nodeOffset)) {
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
            // internal node: compute distances to both children and sort by distance
            const leftOffset = triangleMeshBvh.nodeLeft(nodeOffset);
            const rightOffset = triangleMeshBvh.nodeRight(buffer, nodeOffset);

            // compute squared distances from query center to child node centers
            triangleMeshBvh.nodeGetCenter(buffer, leftOffset, _castSphereVsTriangleMesh_nodeCenter);
            const leftDist = vec3.squaredDistance(_castSphereVsTriangleMesh_queryCenter, _castSphereVsTriangleMesh_nodeCenter);

            triangleMeshBvh.nodeGetCenter(buffer, rightOffset, _castSphereVsTriangleMesh_nodeCenter);
            const rightDist = vec3.squaredDistance(_castSphereVsTriangleMesh_queryCenter, _castSphereVsTriangleMesh_nodeCenter);

            // sort: push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                bvhStack.push(_castSphereVsTriangleMesh_stack, rightOffset, rightDist);
                bvhStack.push(_castSphereVsTriangleMesh_stack, leftOffset, leftDist);
            } else {
                bvhStack.push(_castSphereVsTriangleMesh_stack, leftOffset, leftDist);
                bvhStack.push(_castSphereVsTriangleMesh_stack, rightOffset, rightDist);
            }
        }
    }
}

const castTriangleMeshVsSphere = reversedCastShapeVsShape(castSphereVsTriangleMesh);
