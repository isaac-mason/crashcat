import type { Mat4, Vec3 } from 'mathcat';
import { box3, mat4, quat, vec3 } from 'mathcat';
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
    originX: number,
    originY: number,
    originZ: number,
    directionX: number,
    directionY: number,
    directionZ: number,
    length: number,
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

    // set ray origin from parameters
    vec3.set(_castRayVsConvex_rayOriginLocal, originX, originY, originZ);
    // transform ray origin: (origin - position) rotated by inverse quaternion
    vec3.subtract(_castRayVsConvex_rayOriginLocal, _castRayVsConvex_rayOriginLocal, _castRayVsConvex_pos);
    vec3.transformQuat(_castRayVsConvex_rayOriginLocal, _castRayVsConvex_rayOriginLocal, _castRayVsConvex_invQuat);

    // set ray direction from parameters
    vec3.set(_castRayVsConvex_rayDirectionLocal, directionX, directionY, directionZ);
    // transform ray direction: direction rotated by inverse quaternion
    vec3.transformQuat(_castRayVsConvex_rayDirectionLocal, _castRayVsConvex_rayDirectionLocal, _castRayVsConvex_invQuat);

    // scale direction by ray length for GJK
    vec3.scale(_castRayVsConvex_rayDirectionLocal, _castRayVsConvex_rayDirectionLocal, length);

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

const _collideConvex_BtoA = /* @__PURE__ */ mat4.create();
const _collideConvex_AtoWorld = /* @__PURE__ */ mat4.create();
const _collideConvex_BtoWorld = /* @__PURE__ */ mat4.create();

const _collideConvex_scaleA = /* @__PURE__ */ vec3.create();
const _collideConvex_scaleB = /* @__PURE__ */ vec3.create();

const _collideConvex_faceDirA = /* @__PURE__ */ vec3.create();
const _collideConvex_faceDirB = /* @__PURE__ */ vec3.create();

const _collideConvex_hit = /* @__PURE__ */ createCollideShapeHit();

const _collideConvex_quatA = /* @__PURE__ */ quat.create();
const _collideConvex_quatB = /* @__PURE__ */ quat.create();
const _collideConvex_invQuatA = /* @__PURE__ */ quat.create();
const _collideConvex_relativeRot = /* @__PURE__ */ quat.create();

const _collideConvex_relativePos = /* @__PURE__ */ vec3.create();
const _collideConvex_rotatedRelativePos = /* @__PURE__ */ vec3.create();
const _collideConvex_posA = /* @__PURE__ */ vec3.create();

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
    // to transform shape B into A's local space, we need to compute:
    // 1. the relative rotation: inv(quatA) * quatB
    // 2. the relative position: inv(quatA) * (posB - posA)

    // set up quaternions and position from parameters
    quat.set(_collideConvex_quatA, quatAX, quatAY, quatAZ, quatAW);
    quat.set(_collideConvex_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_collideConvex_posA, posAX, posAY, posAZ);

    // compute the conjugate (inverse) of quatA
    quat.conjugate(_collideConvex_invQuatA, _collideConvex_quatA);

    // compute relative rotation: inv(quatA) * quatB
    quat.multiply(_collideConvex_relativeRot, _collideConvex_invQuatA, _collideConvex_quatB);

    // compute the relative position vector: posB - posA
    vec3.set(_collideConvex_relativePos, posBX - posAX, posBY - posAY, posBZ - posAZ);

    // rotate the relative position by inv(quatA) to bring it into A's local space
    vec3.transformQuat(_collideConvex_rotatedRelativePos, _collideConvex_relativePos, _collideConvex_invQuatA);

    // store scales in scratch arrays (needed for the local function call)
    vec3.set(_collideConvex_scaleA, scaleAX, scaleAY, scaleAZ);
    vec3.set(_collideConvex_scaleB, scaleBX, scaleBY, scaleBZ);

    // build the transformation matrix for A in world space (rotation + translation only, no scale)
    const transformAInWorld = _collideConvex_AtoWorld;
    mat4.fromRotationTranslation(transformAInWorld, _collideConvex_quatA, _collideConvex_posA);

    // build the transformation matrix for B relative to A's local space
    const transformBInA = _collideConvex_BtoA;
    mat4.fromRotationTranslation(transformBInA, _collideConvex_relativeRot, _collideConvex_rotatedRelativePos);

    // delegate to local-space collision function
    collideConvexVsConvexLocal(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        shapeB,
        subShapeIdB,
        transformBInA,
        transformAInWorld,
        _collideConvex_scaleA,
        _collideConvex_scaleB,
    );
}

/**
 * Local-space collision detection for convex vs convex.
 * Operates in shape A's local space. Shape B's transform is relative to A.
 */
export function collideConvexVsConvexLocal(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    shapeB: Shape,
    subShapeIdB: number,
    transformBInA: Mat4,
    transformAInWorld: Mat4,
    scaleA: Vec3,
    scaleB: Vec3,
): void {
    const { maxSeparationDistance, collisionTolerance, penetrationTolerance } = settings;

    // TODO: OBB3 early out?

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

    // wrap support objects in TransformedSupport
    // shape B in A's local space
    setTransformedSupport(_collideConvex_transformedSupportB, transformBInA, supportB);

    // initial search direction (vector from A to B in A's local space)
    // note: As we don't remember the penetration axis from the last iteration, and it is likely that shape2 is pushed out of
    // collision relative to shape1 by comparing their COM's, we use that as an initial penetration axis: shape2.com - shape1.com
    // This has been seen to improve performance by approx. 1% over using a fixed axis like (1, 0, 0).
    // extract translation from transformBInA
    const penetrationAxis = _collideConvex_penetrationAxis;
    penetrationAxis[0] = transformBInA[12];
    penetrationAxis[1] = transformBInA[13];
    penetrationAxis[2] = transformBInA[14];

    // ensure that we do not pass in a near zero penetration axis
    const [penetrationAxisX, penetrationAxisY, penetrationAxisZ] = penetrationAxis;
    const paLenSq =
        penetrationAxisX * penetrationAxisX + penetrationAxisY * penetrationAxisY + penetrationAxisZ * penetrationAxisZ;
    if (paLenSq <= 1e-12) {
        penetrationAxis[0] = 1;
        penetrationAxis[1] = 0;
        penetrationAxis[2] = 0;
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

            // shape B in A's local space (reuse transformBInA computed earlier)
            setTransformedSupport(_collideConvex_transformedSupport, transformBInA, supportBWithRadius);

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
    const [pointAX, pointAY, pointAZ] = penetrationDepth.pointA;
    const [pointBX, pointBY, pointBZ] = penetrationDepth.pointB;
    const dx = pointAX - pointBX;
    const dy = pointAY - pointBY;
    const dz = pointAZ - pointBZ;
    const penetration = Math.sqrt(dx * dx + dy * dy + dz * dz) - maxSeparationDistanceToUse;
    if (-penetration >= collector.earlyOutFraction) {
        return;
    }

    // correct point1 for the added separation distance
    // sqrt(x² + y² + z²)
    const [penetrationAxisNormalX, penetrationAxisNormalY, penetrationAxisNormalZ] = penetrationDepth.penetrationAxis;
    const penetrationAxisLen = Math.sqrt(
        penetrationAxisNormalX * penetrationAxisNormalX +
            penetrationAxisNormalY * penetrationAxisNormalY +
            penetrationAxisNormalZ * penetrationAxisNormalZ,
    );
    if (penetrationAxisLen > 0.0) {
        const correction = maxSeparationDistanceToUse / penetrationAxisLen;
        // inline scaleAndAdd: pointA += penetrationAxis * (-correction)
        penetrationDepth.pointA[0] += penetrationAxisNormalX * -correction;
        penetrationDepth.pointA[1] += penetrationAxisNormalY * -correction;
        penetrationDepth.pointA[2] += penetrationAxisNormalZ * -correction;
    }

    // convert to world space using transformAInWorld
    vec3.transformMat4(_collideConvex_hit.pointA, penetrationDepth.pointA, transformAInWorld);
    vec3.transformMat4(_collideConvex_hit.pointB, penetrationDepth.pointB, transformAInWorld);
    mat4.multiply3x3Vec(_collideConvex_hit.penetrationAxis, transformAInWorld, penetrationDepth.penetrationAxis);

    // populate hit info
    _collideConvex_hit.penetration = penetration;
    _collideConvex_hit.subShapeIdA = subShapeIdA;
    _collideConvex_hit.subShapeIdB = subShapeIdB;
    _collideConvex_hit.materialIdA = (shapeA as ConvexShape).materialId;
    _collideConvex_hit.materialIdB = (shapeB as ConvexShape).materialId;
    _collideConvex_hit.bodyIdB = collector.bodyIdB;

    // gather faces
    if (settings.collectFaces) {
        // get supporting face of shape A (negate penetration axis)
        _collideConvex_faceDirA[0] = -penetrationDepth.penetrationAxis[0];
        _collideConvex_faceDirA[1] = -penetrationDepth.penetrationAxis[1];
        _collideConvex_faceDirA[2] = -penetrationDepth.penetrationAxis[2];
        getShapeSupportingFace(_collideConvex_hit.faceA, shapeA, subShapeIdA, _collideConvex_faceDirA, transformAInWorld, scaleA);

        // get supporting face of shape B
        // transform penetration axis from A's local to B's local using inverse rotation of transformBInA
        // (transposed 3x3 portion = inverse rotation for orthonormal matrices)
        mat4.multiply3x3TransposedVec(_collideConvex_faceDirB, transformBInA, penetrationDepth.penetrationAxis);

        // compute B's world transform: mat4_BtoWorld = transformAInWorld * transformBInA
        mat4.multiply(_collideConvex_BtoWorld, transformAInWorld, transformBInA);

        getShapeSupportingFace(
            _collideConvex_hit.faceB,
            shapeB,
            subShapeIdB,
            _collideConvex_faceDirB,
            _collideConvex_BtoWorld,
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

const _castConvex_inverseQuaternionB = /* @__PURE__ */ quat.create();

const _castConvex_posA = /* @__PURE__ */ vec3.create();
const _castConvex_quatA = /* @__PURE__ */ quat.create();
const _castConvex_scaleA = /* @__PURE__ */ vec3.create();
const _castConvex_displacementA = /* @__PURE__ */ vec3.create();
const _castConvex_posB = /* @__PURE__ */ vec3.create();
const _castConvex_quatB = /* @__PURE__ */ quat.create();
const _castConvex_scaleB = /* @__PURE__ */ vec3.create();
const _castConvex_displacementInB = /* @__PURE__ */ vec3.create();

const _castConvex_convexQueryNormal = /* @__PURE__ */ vec3.create();
const _castConvex_AtoB = /* @__PURE__ */ mat4.create();
const _castConvex_AtoWorldAtContact = /* @__PURE__ */ mat4.create();
const _castConvex_BtoWorld = /* @__PURE__  */ mat4.create();
const _castConvex_invBtoWorld = /* @__PURE__  */ mat4.create();

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

    // compute inverse of B's quaternion for transforming displacement
    quat.conjugate(_castConvex_inverseQuaternionB, _castConvex_quatB);

    // transform A into B's local space using matrices
    // castTransform = inverse(targetTransform) * transformA
    const transformA = mat4.fromRotationTranslation(_castConvex_AtoB, _castConvex_quatA, _castConvex_posA);
    const targetTransform = mat4.fromRotationTranslation(_castConvex_BtoWorld, _castConvex_quatB, _castConvex_posB);

    // inverse target transform (use separate temp to avoid overwriting targetTransform)
    mat4.invert(_castConvex_invBtoWorld, targetTransform);

    // castTransform = targetTransform^-1 * transformA (A's start transform in B's space)
    mat4.multiply(_castConvex_AtoB, _castConvex_invBtoWorld, transformA);

    // transform displacement to B's space
    vec3.transformQuat(_castConvex_displacementInB, _castConvex_displacementA, _castConvex_inverseQuaternionB);

    // delegate to local-space function with world space info for contact points
    castConvexVsConvexLocal(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        shapeB,
        subShapeIdB,
        _castConvex_AtoB,
        _castConvex_scaleA,
        _castConvex_displacementInB,
        _castConvex_scaleB,
        targetTransform,
    );
}

export function castConvexVsConvexLocal(
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    shapeB: Shape,
    subShapeIdB: number,
    castTransform: Mat4,
    scaleA: Vec3,
    displacementInB: Vec3,
    scaleB: Vec3,
    targetTransform: Mat4,
): void {
    // get support functions for both shapes WITHOUT convex radius
    // (radius will be added in EPA if needed)
    const supportA = getShapeSupportFunction(castConvex_supportPoolA, shapeA, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, scaleA);
    const supportB = getShapeSupportFunction(castConvex_supportPoolB, shapeB, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, scaleB);

    // run GJK casting with EPA fallback for deep penetration in B's local space
    const tolerance = 1e-4;
    _castConvex_gjkResult.lambda = collector.earlyOutFraction;
    penetrationCastShape(
        _castConvex_gjkResult,
        castTransform,
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

    // transform contact points from B's local space to world space using targetTransform
    vec3.transformMat4(_castConvex_castShapeHit.pointA, _castConvex_gjkResult.pointA, targetTransform);
    vec3.transformMat4(_castConvex_castShapeHit.pointB, _castConvex_gjkResult.pointB, targetTransform);

    // transform penetrationAxis from B's local space to world space (keep unnormalized)
    mat4.multiply3x3Vec(_castConvex_castShapeHit.penetrationAxis, targetTransform, _castConvex_gjkResult.separatingAxis);

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
        // transform_1_to_2 = castTransform with translation += fraction * displacementInB
        mat4.copy(_castConvex_AtoWorldAtContact, castTransform);
        _castConvex_AtoWorldAtContact[12] += _castConvex_gjkResult.lambda * displacementInB[0];
        _castConvex_AtoWorldAtContact[13] += _castConvex_gjkResult.lambda * displacementInB[1];
        _castConvex_AtoWorldAtContact[14] += _castConvex_gjkResult.lambda * displacementInB[2];

        // shape A's world transform at contact = targetTransform * transformAAtContact
        mat4.multiply(_castConvex_AtoWorldAtContact, targetTransform, _castConvex_AtoWorldAtContact);

        // shape A: contact_normal is in B's local space, transform to A's local using transposed rotation
        mat4.multiply3x3TransposedVec(_castConvex_convexQueryNormal, castTransform, _castConvex_gjkResult.separatingAxis);
        vec3.negate(_castConvex_convexQueryNormal, _castConvex_convexQueryNormal);
        getShapeSupportingFace(
            _castConvex_castShapeHit.faceA,
            shapeA,
            subShapeIdA,
            _castConvex_convexQueryNormal,
            _castConvex_AtoWorldAtContact,
            scaleA,
        );

        // shape b: contact_normal (separatingAxis) is already in B's local space
        getShapeSupportingFace(
            _castConvex_castShapeHit.faceB,
            shapeB,
            subShapeIdB,
            _castConvex_gjkResult.separatingAxis,
            targetTransform,
            scaleB,
        );
    } else {
        _castConvex_castShapeHit.faceA.numVertices = 0;
        _castConvex_castShapeHit.faceB.numVertices = 0;
    }

    collector.addHit(_castConvex_castShapeHit);
}
