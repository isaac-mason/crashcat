import type { Quat, Raycast3, Vec3 } from 'mathcat';
import { box3, obb3, quat, vec3 } from 'mathcat';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape';
import { CastRayStatus, createCastRayHit } from '../collision/cast-ray-vs-shape';
import type { CastShapeCollector, CastShapeSettings } from '../collision/cast-shape-vs-shape';
import { CastShapeStatus, createCastShapeHit } from '../collision/cast-shape-vs-shape';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import { createCollidePointHit } from '../collision/collide-point-vs-shape';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape';
import { createCollideShapeHit } from '../collision/collide-shape-vs-shape';
import {
    createGjkCastRayResult,
    createGjkCastShapeResult,
    createGjkClosestPoints,
    gjkCastRay,
    gjkClosestPoints,
} from '../collision/gjk';
import {
    createPenetrationDepth,
    PenetrationDepthStatus,
    penetrationCastShape,
    penetrationDepthStepEPA,
    penetrationDepthStepGJK,
} from '../collision/penetration';
import * as simplex from '../collision/simplex';
import {
    type AddConvexRadiusSupport,
    createAddConvexRadiusSupport,
    createPointSupport,
    createShapeSupportPool,
    createTransformedSupport,
    getShapeSupportFunction,
    SupportFunctionMode,
    setAddConvexRadiusSupport,
    setPointSupport,
    setTransformedSupport,
} from '../collision/support';
import type { ConvexShape, Shape } from './shapes';
import { getShapeSupportingFace } from './shapes';

/* cast ray */

const _castRayVsConvex_supportPool = /* @__PURE__ */ createShapeSupportPool();
const _castRayVsConvex_hit = /* @__PURE__ */ createCastRayHit();
const _castRayVsConvex_pos = /* @__PURE__ */ vec3.create();
const _castRayVsConvex_quat = /* @__PURE__ */ quat.create();
const _castRayVsConvex_scale = /* @__PURE__ */ vec3.create();
const _castRayVsConvex_rayOriginLocal = /* @__PURE__ */ vec3.create();
const _castRayVsConvex_rayDirectionLocal = /* @__PURE__ */ vec3.create();
const _castRayVsConvex_invQuat = /* @__PURE__ */ quat.create();
const _castRayVsConvex_gjkResult = /* @__PURE__ */ createGjkCastRayResult();

/** cast ray implementation for convex shapes */
export function castRayVsConvex(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: Shape,
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
    vec3.set(_castRayVsConvex_pos, posX, posY, posZ);
    quat.set(_castRayVsConvex_quat, quatX, quatY, quatZ, quatW);
    vec3.set(_castRayVsConvex_scale, scaleX, scaleY, scaleZ);

    const supportFunction = getShapeSupportFunction(
        _castRayVsConvex_supportPool,
        shape,
        SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
        _castRayVsConvex_scale,
    );

    // transform ray from world space to shape local space
    // first, compute inverse quaternion
    quat.conjugate(_castRayVsConvex_invQuat, _castRayVsConvex_quat);

    // transform ray origin: (origin - position) rotated by inverse quaternion
    vec3.subtract(_castRayVsConvex_rayOriginLocal, ray.origin, _castRayVsConvex_pos);
    vec3.transformQuat(_castRayVsConvex_rayOriginLocal, _castRayVsConvex_rayOriginLocal, _castRayVsConvex_invQuat);

    // transform ray direction: direction rotated by inverse quaternion
    vec3.copy(_castRayVsConvex_rayDirectionLocal, ray.direction);
    vec3.transformQuat(_castRayVsConvex_rayDirectionLocal, _castRayVsConvex_rayDirectionLocal, _castRayVsConvex_invQuat);

    // scale direction by ray length for GJK
    vec3.scale(_castRayVsConvex_rayDirectionLocal, _castRayVsConvex_rayDirectionLocal, ray.length);

    // cast ray using GJK
    gjkCastRay(
        _castRayVsConvex_gjkResult,
        _castRayVsConvex_rayOriginLocal,
        _castRayVsConvex_rayDirectionLocal,
        1e-3,
        supportFunction,
        collector.earlyOutFraction,
    );

    if (_castRayVsConvex_gjkResult.isHitFound) {
        const fraction = _castRayVsConvex_gjkResult.lambda;

        // if treatConvexAsSolid OR fraction > 0 when treatConvexAsSolid=false and fraction=0,
        // ray starts inside but we don't report it
        if (settings.treatConvexAsSolid || fraction > 0.0) {
            _castRayVsConvex_hit.status = CastRayStatus.COLLIDING;
            _castRayVsConvex_hit.fraction = Math.max(0.0, fraction);
            _castRayVsConvex_hit.subShapeId = subShapeId;
            _castRayVsConvex_hit.materialId = (shape as ConvexShape).materialId;
            _castRayVsConvex_hit.bodyIdB = collector.bodyIdB;
            collector.addHit(_castRayVsConvex_hit);
        } else {
            collector.addMiss();
        }
    } else {
        collector.addMiss();
    }
}

/* collide point */

const _collidePointVsConvex_quatB = /* @__PURE__ */ quat.create();
const _collidePointVsConvex_scaleB = /* @__PURE__ */ vec3.create();
const _collidePointVsConvex_localPoint = /* @__PURE__ */ vec3.create();
const _collidePointVsConvex_pointSupport = /* @__PURE__ */ createPointSupport();
const _collidePointVsConvex_convexSupportPool = /* @__PURE__ */ createShapeSupportPool();
const _collidePointVsConvex_gjkResult = /* @__PURE__ */ createGjkClosestPoints();
const _collidePointVsConvex_initialDirection = /* @__PURE__ */ vec3.create();
const _collidePointVsConvex_scaledAABB = /* @__PURE__ */ box3.create();
const _collidePointVsConvex_scaleVec = /* @__PURE__ */ vec3.create();
const _collidePointHit = /* @__PURE__ */ createCollidePointHit();

/** collide point implementation for convex shapes */
export function collidePointVsConvex(
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
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
    // transform point to shape's local space
    const localX = pointX - posBX;
    const localY = pointY - posBY;
    const localZ = pointZ - posBZ;

    // apply inverse rotation
    quat.set(_collidePointVsConvex_quatB, quatBX, quatBY, quatBZ, quatBW);
    quat.conjugate(_collidePointVsConvex_quatB, _collidePointVsConvex_quatB);
    vec3.set(_collidePointVsConvex_localPoint, localX, localY, localZ);
    vec3.transformQuat(_collidePointVsConvex_localPoint, _collidePointVsConvex_localPoint, _collidePointVsConvex_quatB);

    // early-out: AABB test
    vec3.set(_collidePointVsConvex_scaleVec, scaleBX, scaleBY, scaleBZ);
    box3.scale(_collidePointVsConvex_scaledAABB, shapeB.aabb, _collidePointVsConvex_scaleVec);
    if (!box3.containsPoint(_collidePointVsConvex_scaledAABB, _collidePointVsConvex_localPoint)) {
        return;
    }

    // point support (always returns the same point)
    setPointSupport(_collidePointVsConvex_pointSupport, _collidePointVsConvex_localPoint);

    // get shape support function with scale applied
    vec3.set(_collidePointVsConvex_scaleB, scaleBX, scaleBY, scaleBZ);
    const shapeSupport = getShapeSupportFunction(
        _collidePointVsConvex_convexSupportPool,
        shapeB,
        SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
        _collidePointVsConvex_scaleB,
    );

    // use point position as initial separating axis guess
    vec3.copy(_collidePointVsConvex_initialDirection, _collidePointVsConvex_localPoint);
    if (vec3.squaredLength(_collidePointVsConvex_initialDirection) < 1e-10) {
        // if point is at origin, use arbitrary direction
        vec3.set(_collidePointVsConvex_initialDirection, 0, 1, 0);
    }

    // run gjk to test intersection
    gjkClosestPoints(
        _collidePointVsConvex_gjkResult,
        shapeSupport,
        _collidePointVsConvex_pointSupport,
        settings.collisionTolerance,
        _collidePointVsConvex_initialDirection,
        0, // maxDistanceSquared = 0 (only care about intersection)
    );

    // if squared distance is very small, point is inside
    const toleranceSq = settings.collisionTolerance * settings.collisionTolerance;
    if (_collidePointVsConvex_gjkResult.squaredDistance <= toleranceSq) {
        _collidePointHit.subShapeIdB = subShapeIdB;
        _collidePointHit.materialId = (shapeB as ConvexShape).materialId;
        _collidePointHit.bodyIdB = collector.bodyIdB;
        collector.addHit(_collidePointHit);
    }
}

/* collide convex vs convex */

const _collideConvex_supportPoolA = /* @__PURE__ */ createShapeSupportPool();
const _collideConvex_supportPoolB = /* @__PURE__ */ createShapeSupportPool();

const _collideConvex_simplex = /* @__PURE__ */ simplex.createSimplex();
const _collideConvex_penetrationDepth = /* @__PURE__ */ createPenetrationDepth();

const _collideConvex_transformedSupportB = /* @__PURE__ */ createTransformedSupport();

const _collideConvex_addRadiusSupport: AddConvexRadiusSupport = /* @__PURE__ */ createAddConvexRadiusSupport();
const _collideConvex_transformedSupport = /* @__PURE__ */ createTransformedSupport();

const _collideConvex_penetrationAxis = /* @__PURE__ */ vec3.create();
const _collideConvex_vectorAB = /* @__PURE__ */ vec3.create();

const _collideConvex_inverseQuatA = /* @__PURE__ */ quat.create();
const _collideConvex_relativePos = /* @__PURE__ */ vec3.create();
const _collideConvex_relativeRot = /* @__PURE__ */ quat.create();

const _collideConvex_aabbShapeExpand = /* @__PURE__ */ vec3.create();
const _collideConvex_aabbShapeA = /* @__PURE__ */ box3.create();
const _collideConvex_aabbShapeB = /* @__PURE__ */ box3.create();
const _collideConvex_obb3ShapeB = /* @__PURE__ */ obb3.create();

const _collideConvex_posA = /* @__PURE__ */ vec3.create();
const _collideConvex_quatA = /* @__PURE__ */ quat.create();
const _collideConvex_scaleA = /* @__PURE__ */ vec3.create();
const _collideConvex_posB = /* @__PURE__ */ vec3.create();
const _collideConvex_quatB = /* @__PURE__ */ quat.create();
const _collideConvex_scaleB = /* @__PURE__ */ vec3.create();

const _collideConvex_contactA = /* @__PURE__ */ vec3.create();
const _collideConvex_contactB = /* @__PURE__ */ vec3.create();

const _collideConvex_temp_faceDirA = /* @__PURE__ */ vec3.create();
const _collideConvex_temp_faceDirB = /* @__PURE__ */ vec3.create();
const _collideConvex_temp_invRelativeRot = /* @__PURE__ */ quat.create();

const _collideConvex_hit = /* @__PURE__ */ createCollideShapeHit();

/**
 * World-space entry point for convex vs convex collision detection.
 * Transforms shapes into local space and delegates to collideConvexVsConvexLocal.
 */
export function collideConvexVsConvex(
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
    vec3.set(_collideConvex_posA, posAX, posAY, posAZ);
    quat.set(_collideConvex_quatA, quatAX, quatAY, quatAZ, quatAW);
    vec3.set(_collideConvex_scaleA, scaleAX, scaleAY, scaleAZ);

    vec3.set(_collideConvex_posB, posBX, posBY, posBZ);
    quat.set(_collideConvex_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_collideConvex_scaleB, scaleBX, scaleBY, scaleBZ);

    // transform B into A's local space
    quat.conjugate(_collideConvex_inverseQuatA, _collideConvex_quatA);
    quat.multiply(_collideConvex_relativeRot, _collideConvex_inverseQuatA, _collideConvex_quatB);
    vec3.subtract(_collideConvex_vectorAB, _collideConvex_posB, _collideConvex_posA);
    vec3.transformQuat(_collideConvex_relativePos, _collideConvex_vectorAB, _collideConvex_inverseQuatA);

    // delegate to local-space function
    collideConvexVsConvexLocal(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        shapeB,
        subShapeIdB,
        _collideConvex_relativePos,
        _collideConvex_relativeRot,
        _collideConvex_scaleA,
        _collideConvex_scaleB,
        _collideConvex_posA,
        _collideConvex_quatA,
    );
}

/**
 * Local-space collision detection for convex vs convex.
 * Operates in shape A's local space. Shape B's transform is relative to A.
 *
 * @param posBInA shape B's position in A's local space
 * @param quatBInA shape B's orientation in A's local space
 * @param scaleA shape A's scale
 * @param scaleB shape B's scale
 * @param positionA shape A's world position (for transforming results back)
 * @param quaternionA shape A's world quaternion (for transforming results back)
 */
export function collideConvexVsConvexLocal(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    shapeB: Shape,
    subShapeIdB: number,
    posBInA: Vec3,
    quatBInA: Quat,
    scaleA: Vec3,
    scaleB: Vec3,
    positionA: Vec3,
    quaternionA: Quat,
): void {
    const { maxSeparationDistance, collisionTolerance, penetrationTolerance } = settings;

    // TODO: OBB3 early out?
    // aabb and obb intersection early out test
    const aabbShapeA = box3.copy(_collideConvex_aabbShapeA, shapeA.aabb);
    box3.scale(aabbShapeA, aabbShapeA, scaleA);
    box3.expandByExtents(aabbShapeA, aabbShapeA, vec3.setScalar(_collideConvex_aabbShapeExpand, maxSeparationDistance));

    const aabbShapeB = box3.copy(_collideConvex_aabbShapeB, shapeB.aabb);
    box3.scale(aabbShapeB, aabbShapeB, scaleB);

    const obb3ShapeB = _collideConvex_obb3ShapeB;
    box3.center(obb3ShapeB.center, aabbShapeB);
    box3.extents(obb3ShapeB.halfExtents, aabbShapeB);
    quat.copy(obb3ShapeB.quaternion, quatBInA);

    if (!obb3.intersectsBox3(obb3ShapeB, aabbShapeA)) {
        // early out - no collision
        return;
    }

    // get supports
    const supportA = getShapeSupportFunction(
        _collideConvex_supportPoolA,
        shapeA,
        SupportFunctionMode.EXCLUDE_CONVEX_RADIUS,
        scaleA,
    );
    const supportB = getShapeSupportFunction(
        _collideConvex_supportPoolB,
        shapeB,
        SupportFunctionMode.EXCLUDE_CONVEX_RADIUS,
        scaleB,
    );

    // wrap support objects in TransformedSupport with accumulated positions/rotations
    // shape B in A's local space
    setTransformedSupport(_collideConvex_transformedSupportB, posBInA, quatBInA, supportB);

    // initial search direction (vector from A to B in A's local space)
    // note: As we don't remember the penetration axis from the last iteration, and it is likely that shape2 is pushed out of
    // collision relative to shape1 by comparing their COM's, we use that as an initial penetration axis: shape2.com - shape1.com
    // This has been seen to improve performance by approx. 1% over using a fixed axis like (1, 0, 0).
    const penetrationAxis = vec3.copy(_collideConvex_penetrationAxis, posBInA);

    // ensure that we do not pass in a near zero penetration axis
    if (vec3.squaredLength(penetrationAxis) <= 1e-12) {
        vec3.set(penetrationAxis, 1, 0, 0);
    }

    const penetrationDepth = _collideConvex_penetrationDepth;
    const simplex = _collideConvex_simplex;

    let maxSeparationDistanceToUse = maxSeparationDistance;

    penetrationDepthStepGJK(
        penetrationDepth,
        simplex,
        supportA,
        _collideConvex_transformedSupportB,
        supportA.convexRadius + maxSeparationDistanceToUse,
        supportB.convexRadius,
        penetrationAxis,
        collisionTolerance,
    );

    // check result of collision detection
    switch (penetrationDepth.status) {
        case PenetrationDepthStatus.NOT_COLLIDING: {
            // definitive no collision
            return;
        }

        case PenetrationDepthStatus.COLLIDING: {
            // gjk found shallow penetration - use those results
            break;
        }

        case PenetrationDepthStatus.INDETERMINATE: {
            // need to run expensive EPA algorithm

            // we know we're overlapping at this point, so we can set the max separation distance to 0.
            // numerically it is possible that GJK finds that the shapes are overlapping but EPA finds that they're separated.
            // in order to avoid this, we clamp the max separation distance to 1 so that we don't excessively inflate the shape,
            // but we still inflate it enough to avoid the case where EPA misses the collision.
            maxSeparationDistanceToUse = Math.min(maxSeparationDistanceToUse, 1.0);

            // create support functions that include convex radius for EPA
            const supportAWithRadius = getShapeSupportFunction(
                _collideConvex_supportPoolA,
                shapeA,
                SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
                scaleA,
            );
            const supportBWithRadius = getShapeSupportFunction(
                _collideConvex_supportPoolB,
                shapeB,
                SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
                scaleB,
            );

            // add separation distance
            setAddConvexRadiusSupport(_collideConvex_addRadiusSupport, maxSeparationDistanceToUse, supportAWithRadius);

            // shape B in A's local space
            setTransformedSupport(_collideConvex_transformedSupport, posBInA, quatBInA, supportBWithRadius);

            // perform EPA step
            if (
                !penetrationDepthStepEPA(
                    penetrationDepth,
                    _collideConvex_addRadiusSupport,
                    _collideConvex_transformedSupport,
                    penetrationTolerance,
                    simplex,
                )
            ) {
                return;
            }
            break;
        }

        default: {
            throw new Error(`Invalid penetration depth status: ${penetrationDepth.status}`);
        }
    }

    // check if the penetration is bigger than the early out fraction
    const penetration = vec3.distance(penetrationDepth.pointA, penetrationDepth.pointB) - maxSeparationDistanceToUse;
    if (-penetration >= collector.earlyOutFraction) {
        return;
    }

    // correct point1 for the added separation distance
    const penetrationAxisLen = vec3.length(penetrationDepth.penetrationAxis);
    if (penetrationAxisLen > 0.0) {
        const correction = maxSeparationDistanceToUse / penetrationAxisLen;
        vec3.scaleAndAdd(penetrationDepth.pointA, penetrationDepth.pointA, penetrationDepth.penetrationAxis, -correction);
    }

    // convert to world space
    vec3.transformQuat(_collideConvex_contactA, penetrationDepth.pointA, quaternionA);
    vec3.add(_collideConvex_contactA, _collideConvex_contactA, positionA);

    vec3.transformQuat(_collideConvex_contactB, penetrationDepth.pointB, quaternionA);
    vec3.add(_collideConvex_contactB, _collideConvex_contactB, positionA);

    vec3.transformQuat(_collideConvex_hit.penetrationAxis, penetrationDepth.penetrationAxis, quaternionA);

    // create collision result
    vec3.copy(_collideConvex_hit.pointA, _collideConvex_contactA);
    vec3.copy(_collideConvex_hit.pointB, _collideConvex_contactB);
    _collideConvex_hit.penetration = penetration;
    _collideConvex_hit.subShapeIdA = subShapeIdA;
    _collideConvex_hit.subShapeIdB = subShapeIdB;
    _collideConvex_hit.materialIdA = (shapeA as ConvexShape).materialId;
    _collideConvex_hit.materialIdB = (shapeB as ConvexShape).materialId;
    _collideConvex_hit.bodyIdB = collector.bodyIdB;

    // gather faces
    if (settings.collectFaces) {
        // get supporting face of shape A
        vec3.negate(_collideConvex_temp_faceDirA, penetrationDepth.penetrationAxis);
        getShapeSupportingFace(
            _collideConvex_hit.faceA,
            shapeA,
            subShapeIdA,
            _collideConvex_temp_faceDirA,
            positionA,
            quaternionA,
            scaleA,
        );

        // get supporting face of shape B
        // transform penetration axis from A's local to B's local: quatBInA^-1 * penetrationAxis
        quat.conjugate(_collideConvex_temp_invRelativeRot, quatBInA);
        vec3.transformQuat(_collideConvex_temp_faceDirB, penetrationDepth.penetrationAxis, _collideConvex_temp_invRelativeRot);

        // transform B to world space for face query: positionB = positionA + quaternionA * posBInA
        vec3.transformQuat(_collideConvex_posB, posBInA, quaternionA);
        vec3.add(_collideConvex_posB, _collideConvex_posB, positionA);
        quat.multiply(_collideConvex_quatB, quaternionA, quatBInA);

        getShapeSupportingFace(
            _collideConvex_hit.faceB,
            shapeB,
            subShapeIdB,
            _collideConvex_temp_faceDirB,
            _collideConvex_posB,
            _collideConvex_quatB,
            scaleB,
        );
    }

    // notify the collector
    collector.addHit(_collideConvex_hit);
}

/* cast convex vs convex */

const castConvex_supportPoolA = /* @__PURE__ */ createShapeSupportPool();
const castConvex_supportPoolB = /* @__PURE__ */ createShapeSupportPool();

const _castConvex_gjkResult = /* @__PURE__ */ createGjkCastShapeResult();

const _castConvex_posAInB = /* @__PURE__ */ vec3.create();
const _castConvex_quatAInB = /* @__PURE__ */ quat.create();
const _castConvex_positionDifference = /* @__PURE__ */ vec3.create();
const _castConvex_inverseQuaternionB = /* @__PURE__ */ quat.create();

const _castConvex_posA = /* @__PURE__ */ vec3.create();
const _castConvex_quatA = /* @__PURE__ */ quat.create();
const _castConvex_scaleA = /* @__PURE__ */ vec3.create();
const _castConvex_displacementA = /* @__PURE__ */ vec3.create();
const _castConvex_posB = /* @__PURE__ */ vec3.create();
const _castConvex_quatB = /* @__PURE__ */ quat.create();
const _castConvex_scaleB = /* @__PURE__ */ vec3.create();
const _castConvex_displacementInB = /* @__PURE__ */ vec3.create();

const _castConvex_contactPointA = /* @__PURE__ */ vec3.create();
const _castConvex_contactPointWorldB = /* @__PURE__ */ vec3.create();
const _castConvex_convexWorldPositionA = /* @__PURE__ */ vec3.create();
const _castConvex_convexWorldQuaternionA = /* @__PURE__ */ quat.create();
const _castConvex_convexQueryNormal = /* @__PURE__ */ vec3.create();
const _castConvex_convexDisplacementScaled = /* @__PURE__ */ vec3.create();
const _castConvex_convexPosAInBAtContact = /* @__PURE__ */ vec3.create();
const _castConvex_convexInvQuatAInB = /* @__PURE__ */ quat.create();

const _castConvex_castShapeHit = /* @__PURE__ */ createCastShapeHit();

export function castConvexVsConvex(
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
    vec3.set(_castConvex_posA, posAX, posAY, posAZ);
    quat.set(_castConvex_quatA, quatAX, quatAY, quatAZ, quatAW);
    vec3.set(_castConvex_scaleA, scaleAX, scaleAY, scaleAZ);
    vec3.set(_castConvex_displacementA, dispAX, dispAY, dispAZ);
    vec3.set(_castConvex_posB, posBX, posBY, posBZ);
    quat.set(_castConvex_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_castConvex_scaleB, scaleBX, scaleBY, scaleBZ);

    // transform A into B's local space
    quat.conjugate(_castConvex_inverseQuaternionB, _castConvex_quatB);

    vec3.sub(_castConvex_positionDifference, _castConvex_posA, _castConvex_posB);
    vec3.transformQuat(_castConvex_posAInB, _castConvex_positionDifference, _castConvex_inverseQuaternionB);

    quat.multiply(_castConvex_quatAInB, _castConvex_inverseQuaternionB, _castConvex_quatA);

    vec3.transformQuat(_castConvex_displacementInB, _castConvex_displacementA, _castConvex_inverseQuaternionB);

    // delegate to local-space function with world space info for contact points
    castConvexVsConvexLocal(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        shapeB,
        subShapeIdB,
        _castConvex_posAInB,
        _castConvex_quatAInB,
        _castConvex_scaleA,
        _castConvex_displacementInB,
        _castConvex_scaleB,
        _castConvex_posB,
        _castConvex_quatB,
    );
}

export function castConvexVsConvexLocal(
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    shapeB: Shape,
    subShapeIdB: number,
    posAInB: Vec3,
    quatAInB: Quat,
    scaleA: Vec3,
    displacementInB: Vec3,
    scaleB: Vec3,
    positionB: Vec3,
    quaternionB: Quat,
): void {
    // get support functions for both shapes WITHOUT convex radius
    // (radius will be added in EPA if needed)
    const supportA = getShapeSupportFunction(castConvex_supportPoolA, shapeA, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, scaleA);
    const supportB = getShapeSupportFunction(castConvex_supportPoolB, shapeB, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, scaleB);

    // run GJK casting with EPA fallback for deep penetration in B's local space
    // penetrationCastShape will handle transform wrapping internally
    const tolerance = 1e-4;
    _castConvex_gjkResult.lambda = collector.earlyOutFraction;
    penetrationCastShape(
        _castConvex_gjkResult,
        posAInB,
        quatAInB,
        supportA,
        supportB,
        displacementInB,
        tolerance,
        settings.penetrationTolerance,
        supportA.convexRadius,
        supportB.convexRadius,
        collector.earlyOutFraction,
        settings.returnDeepestPoint,
    );

    // check if hit found
    if (!_castConvex_gjkResult.hit) {
        collector.addMiss();
        return;
    }

    // lambda must be within valid sweep range
    if (_castConvex_gjkResult.lambda > 1.0) {
        collector.addMiss();
        return;
    }

    // back-face culling: reject if shapes are separating (moving apart)
    if (!settings.collideWithBackfaces) {
        const dotProduct = vec3.dot(_castConvex_gjkResult.separatingAxis, displacementInB);
        if (dotProduct <= 0) {
            collector.addMiss();
            return;
        }
    }

    // early-out check: reject if this hit is further than already found
    if (_castConvex_gjkResult.lambda >= collector.earlyOutFraction) {
        collector.addMiss();
        return;
    }

    // transform contact points from B's local space to world space
    vec3.transformQuat(_castConvex_contactPointA, _castConvex_gjkResult.pointA, quaternionB);
    vec3.add(_castConvex_castShapeHit.pointA, _castConvex_contactPointA, positionB);

    vec3.transformQuat(_castConvex_contactPointWorldB, _castConvex_gjkResult.pointB, quaternionB);
    vec3.add(_castConvex_castShapeHit.pointB, _castConvex_contactPointWorldB, positionB);

    // transform penetrationAxis from B's local space to world space (keep unnormalized)
    vec3.transformQuat(_castConvex_castShapeHit.penetrationAxis, _castConvex_gjkResult.separatingAxis, quaternionB);

    // compute contact normal as -penetrationAxis.normalized()
    vec3.normalize(_castConvex_castShapeHit.normal, _castConvex_castShapeHit.penetrationAxis);
    vec3.negate(_castConvex_castShapeHit.normal, _castConvex_castShapeHit.normal);

    // update result object
    _castConvex_castShapeHit.status = CastShapeStatus.COLLIDING;
    _castConvex_castShapeHit.fraction = _castConvex_gjkResult.lambda;
    _castConvex_castShapeHit.subShapeIdA = subShapeIdA;
    _castConvex_castShapeHit.subShapeIdB = subShapeIdB;
    _castConvex_castShapeHit.materialIdA = (shapeA as ConvexShape).materialId;
    _castConvex_castShapeHit.materialIdB = (shapeB as ConvexShape).materialId;
    _castConvex_castShapeHit.bodyIdB = collector.bodyIdB;

    // extract supporting faces if requested
    if (settings.collectFaces) {
        // calculate transform for shape A at contact point
        // transform_1_to_2 = inShapeCast.mCenterOfMassStart with translation += fraction * inShapeCast.mDirection
        vec3.scale(_castConvex_convexDisplacementScaled, displacementInB, _castConvex_gjkResult.lambda);
        vec3.add(_castConvex_convexPosAInBAtContact, posAInB, _castConvex_convexDisplacementScaled);

        // world space transforms for shape A at contact: positionA = positionB + quaternionB * posAInB, quaternionA = quaternionB * quatAInB
        vec3.transformQuat(_castConvex_convexWorldPositionA, _castConvex_convexPosAInBAtContact, quaternionB);
        vec3.add(_castConvex_convexWorldPositionA, _castConvex_convexWorldPositionA, positionB);
        quat.multiply(_castConvex_convexWorldQuaternionA, quaternionB, quatAInB);

        // shape a: contact_normal is in B's local space, transform to A's local: conjugate(quatAInB) * (-separatingAxis)
        vec3.negate(_castConvex_convexQueryNormal, _castConvex_gjkResult.separatingAxis);
        quat.conjugate(_castConvex_convexInvQuatAInB, quatAInB);
        vec3.transformQuat(_castConvex_convexQueryNormal, _castConvex_convexQueryNormal, _castConvex_convexInvQuatAInB);
        getShapeSupportingFace(
            _castConvex_castShapeHit.faceA,
            shapeA,
            subShapeIdA,
            _castConvex_convexQueryNormal,
            _castConvex_convexWorldPositionA,
            _castConvex_convexWorldQuaternionA,
            scaleA,
        );

        // shape b: contact_normal (separatingAxis) is already in B's local space
        getShapeSupportingFace(
            _castConvex_castShapeHit.faceB,
            shapeB,
            subShapeIdB,
            _castConvex_gjkResult.separatingAxis,
            positionB,
            quaternionB,
            scaleB,
        );
    } else {
        _castConvex_castShapeHit.faceA.numVertices = 0;
        _castConvex_castShapeHit.faceB.numVertices = 0;
    }

    collector.addHit(_castConvex_castShapeHit);
}
