import { type Box3, box3, type Mat4, mat4, type Plane3, plane3, quat, type Raycast3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import { type CastRayCollector, type CastRaySettings, CastRayStatus, createCastRayHit } from '../collision/cast-ray-vs-shape';
import {
    type CastShapeCollector,
    type CastShapeSettings,
    CastShapeStatus,
    createCastShapeHit,
    reversedCastShapeVsShape,
} from '../collision/cast-shape-vs-shape';
import {
    type CollidePointCollector,
    type CollidePointSettings,
    createCollidePointHit,
} from '../collision/collide-point-vs-shape';
import {
    type CollideShapeCollector,
    type CollideShapeSettings,
    createCollideShapeHit,
    reversedCollideShapeVsShape,
} from '../collision/collide-shape-vs-shape';
import { createShapeSupportPool, getShapeSupportFunction, SupportFunctionMode } from '../collision/support';
import type { Face } from '../utils/face';
import { isScaleInsideOut, transformFace } from '../utils/face';
import {
    type ConvexShape,
    defineShape,
    getShapeSupportingFace,
    setCastShapeFn,
    setCollideShapeFn,
    type Shape,
    ShapeCategory,
    shapeDefs,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
} from './shapes';

export const DEFAULT_PLANE_HALF_EXTENT = 1000.0;

/* shape definition */

/** settings for creating a plane shape */
export type PlaneShapeSettings = {
    /** plane definition (normal + constant) */
    plane: Plane3;
    /** half-extent of the plane's bounded representation @default 1000 */
    halfExtent?: number;
    /** material identifier @default -1 */
    materialId?: number;
};

/**
 * a plane shape - infinite plane but with bounded representation for broad-phase.
 * the negative half-space (where signedDistance < 0) is considered solid.
 *
 * planes have no mass properties and should only be used on static bodies.
 */
export type PlaneShape = {
    /** plane shape type */
    type: ShapeType.PLANE;
    /** plane equation: point · normal + constant = 0 */
    plane: Plane3;
    /** half-extent for bounded representation */
    halfExtent: number;
    /** material identifier */
    materialId: number;
    /** shape local bounds (computed) */
    aabb: Box3;
    /** shape center of mass (always at origin) */
    centerOfMass: Vec3;
    /** shape volume (always 0 - infinite/static) */
    volume: number;
};

/** create a plane shape from settings */
export function create(o: PlaneShapeSettings): PlaneShape {
    const plane = plane3.create();
    vec3.normalize(plane.normal, o.plane.normal);
    plane.constant = o.plane.constant;

    const halfExtent = o.halfExtent ?? DEFAULT_PLANE_HALF_EXTENT;

    const shape: PlaneShape = {
        type: ShapeType.PLANE,
        plane,
        halfExtent,
        materialId: o.materialId ?? -1,
        aabb: box3.create(),
        centerOfMass: [0, 0, 0],
        volume: 0,
    };

    update(shape);

    return shape;
}

/** updates a plane shape after it's properties have changed */
export function update(shape: PlaneShape): void {
    computePlaneLocalBounds(shape.aabb, shape);
    shape.volume = 0;
}

/* shape def */

export const def = defineShape<PlaneShape>({
    type: ShapeType.PLANE,
    category: ShapeCategory.OTHER,
    computeMassProperties,
    getSurfaceNormal,
    getSupportingFace,
    getInnerRadius,
    castRay: castRayVsPlane,
    collidePoint: collidePointVsPlane,
    register,
});

function computeMassProperties(out: MassProperties, _shape: PlaneShape): void {
    out.mass = 0;
}

function getSurfaceNormal(ioResult: SurfaceNormalResult, shape: PlaneShape, _subShapeId: number): void {
    // plane normal is always the surface normal
    vec3.copy(ioResult.normal, shape.plane.normal);
}

const _getSupportingFace_vertices: [Vec3, Vec3, Vec3, Vec3] = [vec3.create(), vec3.create(), vec3.create(), vec3.create()];

function getSupportingFace(ioResult: SupportingFaceResult, _direction: Vec3, shape: PlaneShape, _subShapeId: number): void {
    const { position, quaternion, scale } = ioResult;
    const face = ioResult.face;

    // get the 4 vertices of the plane quad in local space
    getPlaneVertices(_getSupportingFace_vertices, shape);

    // check if scale inverts winding
    const insideOut = isScaleInsideOut(scale);

    face.numVertices = 4;
    if (insideOut) {
        // reverse winding: 0,1,2,3 -> 3,2,1,0
        face.vertices[0] = _getSupportingFace_vertices[3][0]; face.vertices[1] = _getSupportingFace_vertices[3][1]; face.vertices[2] = _getSupportingFace_vertices[3][2];
        face.vertices[3] = _getSupportingFace_vertices[2][0]; face.vertices[4] = _getSupportingFace_vertices[2][1]; face.vertices[5] = _getSupportingFace_vertices[2][2];
        face.vertices[6] = _getSupportingFace_vertices[1][0]; face.vertices[7] = _getSupportingFace_vertices[1][1]; face.vertices[8] = _getSupportingFace_vertices[1][2];
        face.vertices[9] = _getSupportingFace_vertices[0][0]; face.vertices[10] = _getSupportingFace_vertices[0][1]; face.vertices[11] = _getSupportingFace_vertices[0][2];
    } else {
        face.vertices[0] = _getSupportingFace_vertices[0][0]; face.vertices[1] = _getSupportingFace_vertices[0][1]; face.vertices[2] = _getSupportingFace_vertices[0][2];
        face.vertices[3] = _getSupportingFace_vertices[1][0]; face.vertices[4] = _getSupportingFace_vertices[1][1]; face.vertices[5] = _getSupportingFace_vertices[1][2];
        face.vertices[6] = _getSupportingFace_vertices[2][0]; face.vertices[7] = _getSupportingFace_vertices[2][1]; face.vertices[8] = _getSupportingFace_vertices[2][2];
        face.vertices[9] = _getSupportingFace_vertices[3][0]; face.vertices[10] = _getSupportingFace_vertices[3][1]; face.vertices[11] = _getSupportingFace_vertices[3][2];
    }

    transformFace(face, position, quaternion, scale);
}

function getInnerRadius(_shape: PlaneShape): number {
    return 0; // planes are infinite, no meaningful inner radius
}

/** scale a plane by a non-uniform scale vector. handles non-uniform and negative scaling correctly */
function scalePlane(out: Plane3, plane: Plane3, scale: Vec3): Plane3 {
    // scaled_normal = normal / scale (component-wise division)
    const scaledNormal: Vec3 = [plane.normal[0] / scale[0], plane.normal[1] / scale[1], plane.normal[2] / scale[2]];

    const scaledNormalLength = vec3.length(scaledNormal);
    vec3.scale(out.normal, scaledNormal, 1 / scaledNormalLength);
    out.constant = plane.constant / scaledNormalLength;

    return out;
}

/** transform a plane by a rigid body transform (rotation + translation only). does NOT normalize - assumes orthogonal transform */
function transformPlane(out: Plane3, plane: Plane3, transform: Mat4): Plane3 {
    // transform normal by rotation part (3x3 multiply)
    out.normal[0] = transform[0] * plane.normal[0] + transform[4] * plane.normal[1] + transform[8] * plane.normal[2];
    out.normal[1] = transform[1] * plane.normal[0] + transform[5] * plane.normal[1] + transform[9] * plane.normal[2];
    out.normal[2] = transform[2] * plane.normal[0] + transform[6] * plane.normal[1] + transform[10] * plane.normal[2];

    // update constant: constant - translation · transformed_normal
    out.constant =
        plane.constant - (transform[12] * out.normal[0] + transform[13] * out.normal[1] + transform[14] * out.normal[2]);

    return out;
}

/** create orthogonal basis vectors for a plane normal, used for generating the quad representation */
function getOrthogonalBasis(normal: Vec3, outPerp1: Vec3, outPerp2: Vec3): void {
    const axisY: Vec3 = [0, 1, 0];
    vec3.cross(outPerp1, normal, axisY);

    const lenSq = vec3.squaredLength(outPerp1);
    if (lenSq < 1e-6) {
        // parallel to Y, use X instead
        vec3.set(outPerp1, 1, 0, 0);
    } else {
        vec3.scale(outPerp1, outPerp1, 1 / Math.sqrt(lenSq));
    }

    vec3.cross(outPerp2, outPerp1, normal);
    vec3.normalize(outPerp2, outPerp2);

    // recompute perp1 for stability
    vec3.cross(outPerp1, normal, outPerp2);
    vec3.normalize(outPerp1, outPerp1);
}

const _getPlaneVertices_perp1 = vec3.create();
const _getPlaneVertices_perp2 = vec3.create();
const _getPlaneVertices_point = vec3.create();
const _getPlaneVertices_temp = vec3.create();

function getPlaneVertices(out: [Vec3, Vec3, Vec3, Vec3], shape: PlaneShape): void {
    const normal = shape.plane.normal;

    // create orthogonal basis
    getOrthogonalBasis(normal, _getPlaneVertices_perp1, _getPlaneVertices_perp2);

    // scale by half-extent
    vec3.scale(_getPlaneVertices_perp1, _getPlaneVertices_perp1, shape.halfExtent);
    vec3.scale(_getPlaneVertices_perp2, _getPlaneVertices_perp2, shape.halfExtent);

    // calculate point on plane: -normal * constant
    vec3.scale(_getPlaneVertices_point, normal, -shape.plane.constant);

    // calculate corners
    vec3.add(_getPlaneVertices_temp, _getPlaneVertices_perp1, _getPlaneVertices_perp2);
    vec3.add(out[0], _getPlaneVertices_point, _getPlaneVertices_temp);

    vec3.subtract(_getPlaneVertices_temp, _getPlaneVertices_perp1, _getPlaneVertices_perp2);
    vec3.add(out[1], _getPlaneVertices_point, _getPlaneVertices_temp);

    vec3.add(_getPlaneVertices_temp, _getPlaneVertices_perp1, _getPlaneVertices_perp2);
    vec3.subtract(out[2], _getPlaneVertices_point, _getPlaneVertices_temp);

    // vertex[3] = point + (-perp1 + perp2)
    vec3.negate(_getPlaneVertices_temp, _getPlaneVertices_perp1);
    vec3.add(_getPlaneVertices_temp, _getPlaneVertices_temp, _getPlaneVertices_perp2);
    vec3.add(out[3], _getPlaneVertices_point, _getPlaneVertices_temp);
}

const _computePlaneLocalBounds_vertices: [Vec3, Vec3, Vec3, Vec3] = [vec3.create(), vec3.create(), vec3.create(), vec3.create()];
const _computePlaneLocalBounds_offset = vec3.create();
const _computePlaneLocalBounds_behind = vec3.create();

function computePlaneLocalBounds(out: Box3, shape: PlaneShape): void {
    getPlaneVertices(_computePlaneLocalBounds_vertices, shape);

    // start with empty bounds - box3 is [min, max]
    out[0][0] = Number.POSITIVE_INFINITY;
    out[0][1] = Number.POSITIVE_INFINITY;
    out[0][2] = Number.POSITIVE_INFINITY;
    out[1][0] = Number.NEGATIVE_INFINITY;
    out[1][1] = Number.NEGATIVE_INFINITY;
    out[1][2] = Number.NEGATIVE_INFINITY;

    const normal = shape.plane.normal;
    vec3.scale(_computePlaneLocalBounds_offset, normal, shape.halfExtent);

    // encapsulate all vertices and points behind the plane
    for (const v of _computePlaneLocalBounds_vertices) {
        box3.expandByPoint(out, out, v);

        // also include point halfExtent behind the plane
        vec3.subtract(_computePlaneLocalBounds_behind, v, _computePlaneLocalBounds_offset);
        box3.expandByPoint(out, out, _computePlaneLocalBounds_behind);
    }
}

function register(): void {
    // register collision and casting with all convex shapes
    for (const shapeDef of Object.values(shapeDefs)) {
        if (shapeDef.category === ShapeCategory.CONVEX) {
            // convex vs plane (direct - shapeA=convex, shapeB=plane)
            setCollideShapeFn(shapeDef.type, ShapeType.PLANE, collideConvexVsPlane);
            setCastShapeFn(shapeDef.type, ShapeType.PLANE, castConvexVsPlane);

            // plane vs convex (reversed - swaps arguments)
            setCollideShapeFn(ShapeType.PLANE, shapeDef.type, reversedCollideShapeVsShape(collideConvexVsPlane));
            setCastShapeFn(ShapeType.PLANE, shapeDef.type, reversedCastShapeVsShape(castConvexVsPlane));
        }
    }
}

const _castRayVsPlane_worldPlane = plane3.create();
const _castRayVsPlane_scaledPlane = plane3.create();
const _castRayVsPlane_scale = vec3.create();
const _castRayVsPlane_transform = mat4.create();
const _castRayVsPlane_quat = quat.create();
const _castRayVsPlane_pos = vec3.create();
const _castRayVsPlane_hit = createCastRayHit();

function castRayVsPlane(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: PlaneShape,
    subShapeId: number,
    _subShapeIdBits: number,
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
    // transform plane to world space

    // first scale the plane
    vec3.set(_castRayVsPlane_scale, scaleX, scaleY, scaleZ);
    scalePlane(_castRayVsPlane_scaledPlane, shape.plane, _castRayVsPlane_scale);

    // then transform by position + rotation
    vec3.set(_castRayVsPlane_pos, posX, posY, posZ);
    quat.set(_castRayVsPlane_quat, quatX, quatY, quatZ, quatW);
    mat4.fromRotationTranslation(_castRayVsPlane_transform, _castRayVsPlane_quat, _castRayVsPlane_pos);
    transformPlane(_castRayVsPlane_worldPlane, _castRayVsPlane_scaledPlane, _castRayVsPlane_transform);

    // calculate signed distance from ray origin to plane
    const distance = plane3.distanceToPoint(_castRayVsPlane_worldPlane, ray.origin);

    // inside solid half-space (distance <= 0)?
    if (settings.treatConvexAsSolid && distance <= 0.0 && collector.earlyOutFraction > 0.0) {
        _castRayVsPlane_hit.status = CastRayStatus.COLLIDING;
        _castRayVsPlane_hit.fraction = 0.0;
        _castRayVsPlane_hit.subShapeId = subShapeId;
        _castRayVsPlane_hit.materialId = (shape as PlaneShape).materialId;
        _castRayVsPlane_hit.bodyIdB = collector.bodyIdB;
        collector.addHit(_castRayVsPlane_hit);
        return;
    }

    // calculate dot product for ray direction with plane normal
    const dot = vec3.dot(ray.direction, _castRayVsPlane_worldPlane.normal);

    // parallel to plane?
    if (Math.abs(dot) < 1e-10) {
        return;
    }

    // back-face culling check
    const isBackFacing = dot > 0.0;
    if (!settings.collideWithBackfaces && isBackFacing) {
        return;
    }

    // calculate hit fraction
    // normalize by ray length to get fraction in [0,1]
    const fraction = -distance / dot / ray.length;

    // valid hit?
    if (fraction >= 0.0 && fraction < collector.earlyOutFraction) {
        _castRayVsPlane_hit.status = CastRayStatus.COLLIDING;
        _castRayVsPlane_hit.fraction = fraction;
        _castRayVsPlane_hit.subShapeId = subShapeId;
        _castRayVsPlane_hit.materialId = (shape as PlaneShape).materialId;
        _castRayVsPlane_hit.bodyIdB = collector.bodyIdB;
        collector.addHit(_castRayVsPlane_hit);
    } else {
        collector.addMiss();
    }
}

const _collidePointVsPlane_worldPlane = plane3.create();
const _collidePointVsPlane_scaledPlane = plane3.create();
const _collidePointVsPlane_scale = vec3.create();
const _collidePointVsPlane_transform = mat4.create();
const _collidePointVsPlane_quat = quat.create();
const _collidePointVsPlane_pos = vec3.create();
const _collidePointVsPlane_point = vec3.create();
const _collidePointVsPlane_hit = createCollidePointHit();

function collidePointVsPlane(
    collector: CollidePointCollector,
    _settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shape: PlaneShape,
    subShapeId: number,
    _subShapeIdBits: number,
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
    // transform plane to world space

    // scale plane
    vec3.set(_collidePointVsPlane_scale, scaleX, scaleY, scaleZ);
    scalePlane(_collidePointVsPlane_scaledPlane, shape.plane, _collidePointVsPlane_scale);

    // transform by position + rotation
    vec3.set(_collidePointVsPlane_pos, posX, posY, posZ);
    quat.set(_collidePointVsPlane_quat, quatX, quatY, quatZ, quatW);
    mat4.fromRotationTranslation(_collidePointVsPlane_transform, _collidePointVsPlane_quat, _collidePointVsPlane_pos);
    transformPlane(_collidePointVsPlane_worldPlane, _collidePointVsPlane_scaledPlane, _collidePointVsPlane_transform);

    // test if point is in negative half-space (solid)
    vec3.set(_collidePointVsPlane_point, pointX, pointY, pointZ);
    const distance = plane3.distanceToPoint(_collidePointVsPlane_worldPlane, _collidePointVsPlane_point);

    if (distance < 0.0) {
        _collidePointVsPlane_hit.subShapeIdB = subShapeId;
        _collidePointVsPlane_hit.bodyIdB = collector.bodyIdB;
        collector.addHit(_collidePointVsPlane_hit);
    }
}

/* convex vs plane collision */

const _collideConvexVsPlane_supportPool = createShapeSupportPool();
const _collideConvexVsPlane_hit = createCollideShapeHit();
const _collideConvexVsPlane_scaledPlane = plane3.create();
const _collideConvexVsPlane_localPlane = plane3.create();
const _collideConvexVsPlane_transform = mat4.create();
const _collideConvexVsPlane_invTransform = mat4.create();
const _collideConvexVsPlane_scaleA = vec3.create();
const _collideConvexVsPlane_scaleB = vec3.create();
const _collideConvexVsPlane_supportPoint = vec3.create();
const _collideConvexVsPlane_normal = vec3.create();
const _collideConvexVsPlane_point1 = vec3.create();
const _collideConvexVsPlane_point2 = vec3.create();
const _collideConvexVsPlane_transformA = mat4.create();
const _collideConvexVsPlane_quatA = quat.create();
const _collideConvexVsPlane_posA = vec3.create();
const _collideConvexVsPlane_quatB = quat.create();
const _collideConvexVsPlane_posB = vec3.create();
const _collideConvexVsPlane_combinedTransform = mat4.create();
const _collideConvexVsPlane_scaleVec = vec3.create();
const _collideConvexVsPlane_offsetByRadius = vec3.create();
const _collideConvexVsPlane_offsetByDistance = vec3.create();
const _collideConvexVsPlane_point1World = vec3.create();
const _collideConvexVsPlane_point2World = vec3.create();
const _collideConvexVsPlane_penetrationAxisWorld = vec3.create();

function collideConvexVsPlane(
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
    const planeShape = shapeB as PlaneShape;

    // scale plane B
    vec3.set(_collideConvexVsPlane_scaleB, scaleBX, scaleBY, scaleBZ);
    scalePlane(_collideConvexVsPlane_scaledPlane, planeShape.plane, _collideConvexVsPlane_scaleB);

    // build transform matrices
    // transform A: convex shape world transform
    vec3.set(_collideConvexVsPlane_posA, posAX, posAY, posAZ);
    quat.set(_collideConvexVsPlane_quatA, quatAX, quatAY, quatAZ, quatAW);

    // transform B: plane world transform (no scale, already applied to plane)
    vec3.set(_collideConvexVsPlane_posB, posBX, posBY, posBZ);
    quat.set(_collideConvexVsPlane_quatB, quatBX, quatBY, quatBZ, quatBW);
    mat4.fromRotationTranslation(_collideConvexVsPlane_transform, _collideConvexVsPlane_quatB, _collideConvexVsPlane_posB);

    // compute inverse of transform A (rotation + translation only, NOT scale)
    mat4.fromRotationTranslation(_collideConvexVsPlane_invTransform, _collideConvexVsPlane_quatA, _collideConvexVsPlane_posA);
    mat4.invert(_collideConvexVsPlane_invTransform, _collideConvexVsPlane_invTransform);

    // transform plane to convex shape's local space (invA * transformB)
    mat4.multiply(_collideConvexVsPlane_combinedTransform, _collideConvexVsPlane_invTransform, _collideConvexVsPlane_transform);
    transformPlane(_collideConvexVsPlane_localPlane, _collideConvexVsPlane_scaledPlane, _collideConvexVsPlane_combinedTransform);

    const normal = _collideConvexVsPlane_localPlane.normal;

    // get support function for convex shape with scale applied
    vec3.set(_collideConvexVsPlane_scaleA, scaleAX, scaleAY, scaleAZ);
    const supportFn = getShapeSupportFunction(
        _collideConvexVsPlane_supportPool,
        shapeA,
        SupportFunctionMode.DEFAULT,
        _collideConvexVsPlane_scaleA,
    );

    if (!supportFn) {
        throw new Error('collideConvexVsPlane: shape A must be convex');
    }

    // get support point in direction opposite to plane normal
    vec3.negate(_collideConvexVsPlane_normal, normal);
    supportFn.getSupport(_collideConvexVsPlane_normal, _collideConvexVsPlane_supportPoint);

    // calculate penetration
    const signedDistance = plane3.distanceToPoint(_collideConvexVsPlane_localPlane, _collideConvexVsPlane_supportPoint);
    const convexRadius = supportFn.convexRadius;
    const penetration = -signedDistance + convexRadius;

    // check if penetration is within tolerance
    if (penetration > -settings.maxSeparationDistance) {
        // calculate contact points in convex local space
        vec3.scale(_collideConvexVsPlane_offsetByRadius, normal, convexRadius);
        vec3.subtract(_collideConvexVsPlane_point1, _collideConvexVsPlane_supportPoint, _collideConvexVsPlane_offsetByRadius);

        vec3.scale(_collideConvexVsPlane_offsetByDistance, normal, signedDistance);
        vec3.subtract(_collideConvexVsPlane_point2, _collideConvexVsPlane_supportPoint, _collideConvexVsPlane_offsetByDistance);

        // build transform A (rotation + translation only, NO scale)
        // scale is already applied in the support function
        mat4.fromRotationTranslation(_collideConvexVsPlane_transformA, _collideConvexVsPlane_quatA, _collideConvexVsPlane_posA);

        // transform contact points to world space
        vec3.transformMat4(_collideConvexVsPlane_point1World, _collideConvexVsPlane_point1, _collideConvexVsPlane_transformA);
        vec3.transformMat4(_collideConvexVsPlane_point2World, _collideConvexVsPlane_point2, _collideConvexVsPlane_transformA);

        // transform penetration axis to world space (rotation only)
        vec3.negate(_collideConvexVsPlane_normal, normal);
        vec3.transformQuat(_collideConvexVsPlane_penetrationAxisWorld, _collideConvexVsPlane_normal, _collideConvexVsPlane_quatA);

        // create hit
        vec3.copy(_collideConvexVsPlane_hit.pointA, _collideConvexVsPlane_point1World);
        vec3.copy(_collideConvexVsPlane_hit.pointB, _collideConvexVsPlane_point2World);
        vec3.copy(_collideConvexVsPlane_hit.penetrationAxis, _collideConvexVsPlane_penetrationAxisWorld);
        _collideConvexVsPlane_hit.penetration = penetration;
        _collideConvexVsPlane_hit.subShapeIdA = subShapeIdA;
        _collideConvexVsPlane_hit.subShapeIdB = subShapeIdB;
        _collideConvexVsPlane_hit.materialIdA = (shapeA as ConvexShape).materialId;
        _collideConvexVsPlane_hit.materialIdB = planeShape.materialId;
        _collideConvexVsPlane_hit.bodyIdB = collector.bodyIdB;

        // gather supporting faces if requested
        if (settings.collectFaces) {
            // get supporting face of convex shape in world space
            vec3.set(_collideConvexVsPlane_scaleVec, scaleAX, scaleAY, scaleAZ);
            getShapeSupportingFace(
                _collideConvexVsPlane_hit.faceA,
                shapeA,
                subShapeIdA,
                normal, // direction in local space
                _collideConvexVsPlane_posA,
                _collideConvexVsPlane_quatA,
                _collideConvexVsPlane_scaleVec,
            );

            // get adaptive supporting face for plane
            if (_collideConvexVsPlane_hit.faceA.numVertices > 0) {
                getAdaptivePlaneSupportingFace(
                    _collideConvexVsPlane_hit.faceB,
                    shapeA,
                    _collideConvexVsPlane_posA,
                    _collideConvexVsPlane_scaledPlane,
                    _collideConvexVsPlane_transform,
                );
            }
        }

        collector.addHit(_collideConvexVsPlane_hit);
    } else {
        collector.addMiss();
    }
}

const _getAdaptivePlaneSupportingFace_worldPlane = plane3.create();
const _getAdaptivePlaneSupportingFace_center = vec3.create();
const _getAdaptivePlaneSupportingFace_perp1 = vec3.create();
const _getAdaptivePlaneSupportingFace_perp2 = vec3.create();
const _getAdaptivePlaneSupportingFace_bboxSize = vec3.create();
const _getAdaptivePlaneSupportingFace_temp = vec3.create();
const _getAdaptivePlaneSupportingFace_v0 = vec3.create();
const _getAdaptivePlaneSupportingFace_v1 = vec3.create();
const _getAdaptivePlaneSupportingFace_v2 = vec3.create();
const _getAdaptivePlaneSupportingFace_v3 = vec3.create();

/**
 * get an adaptive-sized supporting face for the plane.
 * size is based on the convex shape's bounding box to avoid numerical issues.
 */
function getAdaptivePlaneSupportingFace(
    outFace: Face,
    convexShape: Shape,
    convexWorldPos: Vec3,
    plane: Plane3,
    planeToWorld: Mat4,
): void {
    // transform plane to world space
    transformPlane(_getAdaptivePlaneSupportingFace_worldPlane, plane, planeToWorld);

    // project convex position onto plane
    plane3.projectPoint(_getAdaptivePlaneSupportingFace_center, _getAdaptivePlaneSupportingFace_worldPlane, convexWorldPos);

    // create orthogonal basis
    getOrthogonalBasis(
        _getAdaptivePlaneSupportingFace_worldPlane.normal,
        _getAdaptivePlaneSupportingFace_perp1,
        _getAdaptivePlaneSupportingFace_perp2,
    );

    // size based on convex shape's bounding box diagonal
    box3.size(_getAdaptivePlaneSupportingFace_bboxSize, convexShape.aabb);
    const size = vec3.length(_getAdaptivePlaneSupportingFace_bboxSize);

    vec3.scale(_getAdaptivePlaneSupportingFace_perp1, _getAdaptivePlaneSupportingFace_perp1, size);
    vec3.scale(_getAdaptivePlaneSupportingFace_perp2, _getAdaptivePlaneSupportingFace_perp2, size);

    // generate quad vertices
    outFace.numVertices = 4;

    vec3.add(_getAdaptivePlaneSupportingFace_temp, _getAdaptivePlaneSupportingFace_perp1, _getAdaptivePlaneSupportingFace_perp2);
    vec3.add(_getAdaptivePlaneSupportingFace_v0, _getAdaptivePlaneSupportingFace_center, _getAdaptivePlaneSupportingFace_temp);

    vec3.subtract(
        _getAdaptivePlaneSupportingFace_temp,
        _getAdaptivePlaneSupportingFace_perp1,
        _getAdaptivePlaneSupportingFace_perp2,
    );
    vec3.add(_getAdaptivePlaneSupportingFace_v1, _getAdaptivePlaneSupportingFace_center, _getAdaptivePlaneSupportingFace_temp);

    vec3.add(_getAdaptivePlaneSupportingFace_temp, _getAdaptivePlaneSupportingFace_perp1, _getAdaptivePlaneSupportingFace_perp2);
    vec3.subtract(
        _getAdaptivePlaneSupportingFace_v2,
        _getAdaptivePlaneSupportingFace_center,
        _getAdaptivePlaneSupportingFace_temp,
    );

    // vertex[3] = center + (-perp1 + perp2)
    vec3.negate(_getAdaptivePlaneSupportingFace_temp, _getAdaptivePlaneSupportingFace_perp1);
    vec3.add(_getAdaptivePlaneSupportingFace_temp, _getAdaptivePlaneSupportingFace_temp, _getAdaptivePlaneSupportingFace_perp2);
    vec3.add(_getAdaptivePlaneSupportingFace_v3, _getAdaptivePlaneSupportingFace_center, _getAdaptivePlaneSupportingFace_temp);

    outFace.vertices[0] = _getAdaptivePlaneSupportingFace_v0[0];
    outFace.vertices[1] = _getAdaptivePlaneSupportingFace_v0[1];
    outFace.vertices[2] = _getAdaptivePlaneSupportingFace_v0[2];
    outFace.vertices[3] = _getAdaptivePlaneSupportingFace_v1[0];
    outFace.vertices[4] = _getAdaptivePlaneSupportingFace_v1[1];
    outFace.vertices[5] = _getAdaptivePlaneSupportingFace_v1[2];
    outFace.vertices[6] = _getAdaptivePlaneSupportingFace_v2[0];
    outFace.vertices[7] = _getAdaptivePlaneSupportingFace_v2[1];
    outFace.vertices[8] = _getAdaptivePlaneSupportingFace_v2[2];
    outFace.vertices[9] = _getAdaptivePlaneSupportingFace_v3[0];
    outFace.vertices[10] = _getAdaptivePlaneSupportingFace_v3[1];
    outFace.vertices[11] = _getAdaptivePlaneSupportingFace_v3[2];
}

/* cast convex vs plane */

const _castConvexVsPlane_supportPool = createShapeSupportPool();
const _castConvexVsPlane_hit = createCastShapeHit();
const _castConvexVsPlane_scaledPlane = plane3.create();
const _castConvexVsPlane_scaleA = vec3.create();
const _castConvexVsPlane_scaleB = vec3.create();
const _castConvexVsPlane_startTransform = mat4.create();
const _castConvexVsPlane_normalInShapeSpace = vec3.create();
const _castConvexVsPlane_supportPoint = vec3.create();
const _castConvexVsPlane_direction = vec3.create();
const _castConvexVsPlane_invQuat = quat.create();
const _castConvexVsPlane_quatA = quat.create();
const _castConvexVsPlane_posA = vec3.create();
const _castConvexVsPlane_quatB = quat.create();
const _castConvexVsPlane_posB = vec3.create();
const _castConvexVsPlane_supportPointWorld = vec3.create();
const _castConvexVsPlane_planeToWorld = mat4.create();
const _castConvexVsPlane_offsetByRadius = vec3.create();
const _castConvexVsPlane_offsetByDistance = vec3.create();
const _castConvexVsPlane_point1 = vec3.create();
const _castConvexVsPlane_point2 = vec3.create();
const _castConvexVsPlane_offset = vec3.create();
const _castConvexVsPlane_comHit = mat4.create();
const _castConvexVsPlane_contactLocal = vec3.create();
const _castConvexVsPlane_penetrationAxisWorld = vec3.create();
const _castConvexVsPlane_shapeToWorld = mat4.create();
const _castConvexVsPlane_quatFromMat4 = quat.create();
const _castConvexVsPlane_posFromMat4 = vec3.create();

export function castConvexVsPlane(
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
    dispAX: number,
    dispAY: number,
    dispAZ: number,
    shapeB: Shape,
    subShapeIdB: number,
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
    const planeShape = shapeB as PlaneShape;

    // scale the plane
    vec3.set(_castConvexVsPlane_scaleB, scaleBX, scaleBY, scaleBZ);
    scalePlane(_castConvexVsPlane_scaledPlane, planeShape.plane, _castConvexVsPlane_scaleB);

    const normal = _castConvexVsPlane_scaledPlane.normal;

    // build start transform for convex shape
    vec3.set(_castConvexVsPlane_posA, posAX, posAY, posAZ);
    quat.set(_castConvexVsPlane_quatA, quatAX, quatAY, quatAZ, quatAW);
    mat4.fromRotationTranslation(_castConvexVsPlane_startTransform, _castConvexVsPlane_quatA, _castConvexVsPlane_posA);

    // get support function
    vec3.set(_castConvexVsPlane_scaleA, scaleAX, scaleAY, scaleAZ);
    const supportFn = getShapeSupportFunction(
        _castConvexVsPlane_supportPool,
        shapeA,
        SupportFunctionMode.DEFAULT,
        _castConvexVsPlane_scaleA,
    );

    if (!supportFn) {
        throw new Error('castConvexVsPlane: shape A must be convex');
    }

    // transform normal to convex shape's local space
    quat.conjugate(_castConvexVsPlane_invQuat, _castConvexVsPlane_quatA);
    vec3.transformQuat(_castConvexVsPlane_normalInShapeSpace, normal, _castConvexVsPlane_invQuat);

    // get support point in opposite direction
    vec3.negate(_castConvexVsPlane_normalInShapeSpace, _castConvexVsPlane_normalInShapeSpace);
    supportFn.getSupport(_castConvexVsPlane_normalInShapeSpace, _castConvexVsPlane_supportPoint);

    // transform support point to world space
    vec3.transformMat4(_castConvexVsPlane_supportPointWorld, _castConvexVsPlane_supportPoint, _castConvexVsPlane_startTransform);

    // calculate initial penetration
    const signedDistance = plane3.distanceToPoint(_castConvexVsPlane_scaledPlane, _castConvexVsPlane_supportPointWorld);
    const convexRadius = supportFn.convexRadius;
    const penetrationDepth = -signedDistance + convexRadius;

    // direction of cast
    vec3.set(_castConvexVsPlane_direction, dispAX, dispAY, dispAZ);
    const dot = vec3.dot(_castConvexVsPlane_direction, normal);

    let fraction: number;

    // starting in collision?
    if (penetrationDepth > 0.0) {
        // back-face culling?
        if (!settings.collideWithBackfaces && dot > 0.0) {
            return;
        }

        // check if this is a shallower hit than what we already have
        if (penetrationDepth <= -collector.earlyOutFraction) {
            return;
        }

        // hit at fraction 0
        fraction = 0.0;

        // transform plane to world space for contact points
        vec3.set(_castConvexVsPlane_posB, posBX, posBY, posBZ);
        quat.set(_castConvexVsPlane_quatB, quatBX, quatBY, quatBZ, quatBW);
        mat4.fromRotationTranslation(_castConvexVsPlane_planeToWorld, _castConvexVsPlane_quatB, _castConvexVsPlane_posB);
        mat4.copy(_castConvexVsPlane_comHit, _castConvexVsPlane_planeToWorld);

        vec3.scale(_castConvexVsPlane_offsetByRadius, normal, convexRadius);
        vec3.subtract(_castConvexVsPlane_contactLocal, _castConvexVsPlane_supportPointWorld, _castConvexVsPlane_offsetByRadius);
        vec3.transformMat4(_castConvexVsPlane_point1, _castConvexVsPlane_contactLocal, _castConvexVsPlane_planeToWorld);

        vec3.scale(_castConvexVsPlane_offsetByDistance, normal, signedDistance);
        vec3.subtract(_castConvexVsPlane_contactLocal, _castConvexVsPlane_supportPointWorld, _castConvexVsPlane_offsetByDistance);
        vec3.transformMat4(_castConvexVsPlane_point2, _castConvexVsPlane_contactLocal, _castConvexVsPlane_planeToWorld);
    } else if (dot < 0.0) {
        // moving towards the plane - calculate time of impact
        fraction = penetrationDepth / dot;

        // further than early out?
        if (fraction >= collector.earlyOutFraction) {
            return;
        }

        // transform plane to world and offset by cast direction
        vec3.set(_castConvexVsPlane_posB, posBX, posBY, posBZ);
        quat.set(_castConvexVsPlane_quatB, quatBX, quatBY, quatBZ, quatBW);
        mat4.fromRotationTranslation(_castConvexVsPlane_planeToWorld, _castConvexVsPlane_quatB, _castConvexVsPlane_posB);
        vec3.scale(_castConvexVsPlane_offset, _castConvexVsPlane_direction, fraction);
        mat4.translate(_castConvexVsPlane_comHit, _castConvexVsPlane_planeToWorld, _castConvexVsPlane_offset);

        // contact point at time of impact
        vec3.scale(_castConvexVsPlane_offsetByRadius, normal, convexRadius);
        vec3.subtract(_castConvexVsPlane_contactLocal, _castConvexVsPlane_supportPointWorld, _castConvexVsPlane_offsetByRadius);
        vec3.transformMat4(_castConvexVsPlane_point1, _castConvexVsPlane_contactLocal, _castConvexVsPlane_comHit);
        vec3.copy(_castConvexVsPlane_point2, _castConvexVsPlane_point1);
    } else {
        // moving away from plane or parallel
        return;
    }

    // transform penetration axis to world space
    vec3.negate(_castConvexVsPlane_penetrationAxisWorld, normal);
    vec3.transformMat4(
        _castConvexVsPlane_penetrationAxisWorld,
        _castConvexVsPlane_penetrationAxisWorld,
        _castConvexVsPlane_comHit,
    );

    // create hit
    _castConvexVsPlane_hit.status = CastShapeStatus.COLLIDING;
    _castConvexVsPlane_hit.fraction = fraction;
    vec3.copy(_castConvexVsPlane_hit.pointA, _castConvexVsPlane_point1);
    vec3.copy(_castConvexVsPlane_hit.pointB, _castConvexVsPlane_point2);
    _castConvexVsPlane_hit.penetrationDepth = penetrationDepth;
    vec3.copy(_castConvexVsPlane_hit.penetrationAxis, _castConvexVsPlane_penetrationAxisWorld);
    vec3.negate(_castConvexVsPlane_hit.normal, _castConvexVsPlane_penetrationAxisWorld);
    vec3.normalize(_castConvexVsPlane_hit.normal, _castConvexVsPlane_hit.normal);
    _castConvexVsPlane_hit.subShapeIdA = subShapeIdA;
    _castConvexVsPlane_hit.subShapeIdB = subShapeIdB;
    _castConvexVsPlane_hit.materialIdA = (shapeA as ConvexShape).materialId;
    _castConvexVsPlane_hit.materialIdB = planeShape.materialId;
    _castConvexVsPlane_hit.bodyIdB = collector.bodyIdB;

    // gather faces if requested
    if (settings.collectFaces) {
        // transform convex to world at impact
        mat4.multiply(_castConvexVsPlane_shapeToWorld, _castConvexVsPlane_comHit, _castConvexVsPlane_startTransform);

        vec3.set(
            _castConvexVsPlane_posFromMat4,
            _castConvexVsPlane_shapeToWorld[12],
            _castConvexVsPlane_shapeToWorld[13],
            _castConvexVsPlane_shapeToWorld[14],
        );
        quat.fromMat4(_castConvexVsPlane_quatFromMat4, _castConvexVsPlane_shapeToWorld);
        vec3.set(_castConvexVsPlane_scaleA, scaleAX, scaleAY, scaleAZ);

        getShapeSupportingFace(
            _castConvexVsPlane_hit.faceA,
            shapeA,
            subShapeIdA,
            _castConvexVsPlane_normalInShapeSpace,
            _castConvexVsPlane_posFromMat4,
            _castConvexVsPlane_quatFromMat4,
            _castConvexVsPlane_scaleA,
        );

        if (_castConvexVsPlane_hit.faceA.numVertices > 0) {
            getAdaptivePlaneSupportingFace(
                _castConvexVsPlane_hit.faceB,
                shapeA,
                _castConvexVsPlane_posFromMat4,
                _castConvexVsPlane_scaledPlane,
                _castConvexVsPlane_comHit,
            );
        }
    }

    collector.addHit(_castConvexVsPlane_hit);
}
