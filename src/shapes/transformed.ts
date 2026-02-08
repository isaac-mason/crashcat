import { type Box3, box3, mat4, type Quat, quat, type Raycast3, raycast3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as massProperties from '../body/mass-properties';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape';
import type { CastShapeCollector, CastShapeSettings } from '../collision/cast-shape-vs-shape';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape';
import {
    collisionDispatch,
    computeMassProperties,
    defineShape,
    getShapeInnerRadius,
    shapeDefs,
    type Shape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
} from './shapes';

/** settings for creating a transformed shape */
export type TransformedShapeSettings = {
    shape: Shape;
    position: Vec3;
    quaternion: Quat;
};

/** transformed shape - applies a rigid transform to an inner shape */
export type TransformedShape = {
    type: ShapeType.TRANSFORMED;
    shape: Shape;
    position: Vec3;
    quaternion: Quat;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

/** create a transformed shape */
export function create(o: TransformedShapeSettings): TransformedShape {
    const shape: TransformedShape = {
        type: ShapeType.TRANSFORMED,
        shape: o.shape,
        position: o.position,
        quaternion: o.quaternion,
        aabb: box3.create(),
        centerOfMass: vec3.create(),
        volume: 0,
    };

    update(shape);

    return shape;
}

function computeTransformedVolume(shape: TransformedShape): number {
    // translation and rotation don't affect volume
    return shape.shape.volume;
}

const _computeTransformedLocalBounds_corner = /* @__PURE__ */ vec3.create();

function computeTransformedLocalBounds(out: Box3, shape: TransformedShape): void {
    // start with empty bounds
    out[0][0] = Infinity;
    out[0][1] = Infinity;
    out[0][2] = Infinity;
    out[1][0] = -Infinity;
    out[1][1] = -Infinity;
    out[1][2] = -Infinity;

    const childAABB = shape.shape.aabb;

    // transform all 8 corners of the child AABB
    for (let x = 0; x < 2; x++) {
        for (let y = 0; y < 2; y++) {
            for (let z = 0; z < 2; z++) {
                _computeTransformedLocalBounds_corner[0] = childAABB[x][0];
                _computeTransformedLocalBounds_corner[1] = childAABB[y][1];
                _computeTransformedLocalBounds_corner[2] = childAABB[z][2];

                // rotate and translate corner
                vec3.transformQuat(
                    _computeTransformedLocalBounds_corner,
                    _computeTransformedLocalBounds_corner,
                    shape.quaternion,
                );
                vec3.add(_computeTransformedLocalBounds_corner, _computeTransformedLocalBounds_corner, shape.position);

                // expand transformed AABB
                box3.expandByPoint(out, out, _computeTransformedLocalBounds_corner);
            }
        }
    }
}

function computeTransformedCenterOfMass(out: Vec3, shape: TransformedShape): void {
    // get the inner shape's center of mass
    vec3.copy(out, shape.shape.centerOfMass);
    // rotate by quaternion
    vec3.transformQuat(out, out, shape.quaternion);
    // add translation
    vec3.add(out, out, shape.position);
}

/** updates a transformed shape after it's properties have changed */
export function update(shape: TransformedShape): void {
    computeTransformedLocalBounds(shape.aabb, shape);
    computeTransformedCenterOfMass(shape.centerOfMass, shape);
    shape.volume = computeTransformedVolume(shape);
}

const _rotationMat = /* @__PURE__ */ mat4.create();
const _childMassProperties = /* @__PURE__ */ massProperties.create();

const _surfaceNormal_invRotation = /* @__PURE__ */ quat.create();
const _surfaceNormal_forwardRotation = /* @__PURE__ */ quat.create();

const _supportingFace_vec3 = /* @__PURE__ */ vec3.create();
const _supportingFace_quat = /* @__PURE__ */ quat.create();
const _supportingFace_localDirection = /* @__PURE__ */ vec3.create();

export const def = defineShape<TransformedShape>({
    type: ShapeType.TRANSFORMED,
    category: ShapeCategory.DECORATOR,
    computeMassProperties(out: MassProperties, shape: TransformedShape): void {
        // only rotates the inertia (translation doesn't affect inertia about center of mass)
        computeMassProperties(_childMassProperties, shape.shape);

        // convert quaternion to rotation matrix
        mat4.fromQuat(_rotationMat, shape.quaternion);

        // only apply rotation to inertia
        massProperties.rotate(out, _childMassProperties, _rotationMat);
    },
    getSurfaceNormal(ioResult: SurfaceNormalResult, shape: TransformedShape, subShapeId: number): void {
        // accumulate transform: position = position - translation (inverse transform)
        vec3.subtract(ioResult.position, ioResult.position, shape.position);

        // accumulate rotation: quaternion = conjugate(shape.rotation) * quaternion (inverse transform)
        const invRotation = quat.conjugate(_surfaceNormal_invRotation, shape.quaternion);
        quat.multiply(ioResult.quaternion, invRotation, ioResult.quaternion);

        // transform position to local space
        vec3.transformQuat(ioResult.position, ioResult.position, invRotation);

        // get normal from inner shape in its local space
        shapeDefs[shape.shape.type].getSurfaceNormal(ioResult, shape.shape, subShapeId);

        // transform normal back to outer space using accumulated rotation.
        // the accumulated quaternion contains the inverse transforms, so we need to apply its conjugate
        // to get the forward transformation for the normal.
        const forwardAccumulatedRotation = quat.conjugate(_surfaceNormal_forwardRotation, ioResult.quaternion);
        vec3.transformQuat(ioResult.normal, ioResult.normal, forwardAccumulatedRotation);
    },
    getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: TransformedShape, subShapeId: number): void {
        // accumulate transform: position += rotation * translation
        vec3.transformQuat(_supportingFace_vec3, shape.position, ioResult.quaternion);
        vec3.add(ioResult.position, ioResult.position, _supportingFace_vec3);

        // accumulate rotation: rotation = rotation * shape.rotation
        quat.multiply(ioResult.quaternion, ioResult.quaternion, shape.quaternion);

        // transform direction to local space (rotate by inverse quaternion)
        const invRotation = quat.conjugate(_supportingFace_quat, shape.quaternion);
        vec3.transformQuat(_supportingFace_localDirection, direction, invRotation);

        // compute face in local space - pass SubShapeID unchanged (decorator shapes don't consume bits)
        shapeDefs[shape.shape.type].getSupportingFace(ioResult, _supportingFace_localDirection, shape.shape, subShapeId);
    },
    getInnerRadius(shape: TransformedShape): number {
        return getShapeInnerRadius(shape.shape);
    },
    getLeafShape(out, shape, subShapeId): void {
        // pass through to inner shape
        const innerShapeDef = shapeDefs[shape.shape.type];
        innerShapeDef.getLeafShape(out, shape.shape, subShapeId);
    },
    getSubShapeTransformedShape(outResult, shape, subShapeId): void {
        // apply decorated transform: pos = pos + rotate(rot, shapePos), rot = rot * shapeRot
        const rotatedPos = vec3.create();
        vec3.transformQuat(rotatedPos, shape.position, outResult.rotation);
        vec3.add(outResult.position, outResult.position, rotatedPos);
        quat.multiply(outResult.rotation, outResult.rotation, shape.quaternion);

        // pass through to inner shape
        const innerShapeDef = shapeDefs[shape.shape.type];
        innerShapeDef.getSubShapeTransformedShape(outResult, shape.shape, subShapeId);
    },
    castRay: castRayVsTransformed,
    collidePoint: collidePointVsTransformed,
    register: () => {
        for (const shapeDef of Object.values(shapeDefs)) {
            setCollideShapeFn(ShapeType.TRANSFORMED, shapeDef.type, collideTransformedVsShape);
            setCollideShapeFn(shapeDef.type, ShapeType.TRANSFORMED, collideShapeVsTransformed);

            setCastShapeFn(ShapeType.TRANSFORMED, shapeDef.type, castTransformedVsShape);
            setCastShapeFn(shapeDef.type, ShapeType.TRANSFORMED, castShapeVsTransformed);
        }
    },
});

/* cast ray */

const _castRayVsTransformed_pos = /* @__PURE__ */ vec3.create();
const _castRayVsTransformed_quat = /* @__PURE__ */ quat.create();
const _castRayVsTransformed_worldPos = /* @__PURE__ */ vec3.create();
const _castRayVsTransformed_worldQuat = /* @__PURE__ */ quat.create();
const _castRayVsTransformed_rayOriginLocal = /* @__PURE__ */ vec3.create();
const _castRayVsTransformed_rayDirectionLocal = /* @__PURE__ */ vec3.create();
const _castRayVsTransformed_invQuat = /* @__PURE__ */ quat.create();
const _castRayVsTransformed_tempRay = /* @__PURE__ */ raycast3.create();

function castRayVsTransformed(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: TransformedShape,
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
    const transformedShape = shape as TransformedShape;

    vec3.set(_castRayVsTransformed_pos, posX, posY, posZ);
    quat.set(_castRayVsTransformed_quat, quatX, quatY, quatZ, quatW);

    // accumulate transform: world = parent * child
    quat.multiply(_castRayVsTransformed_worldQuat, _castRayVsTransformed_quat, transformedShape.quaternion);
    vec3.transformQuat(_castRayVsTransformed_worldPos, transformedShape.position, _castRayVsTransformed_quat);
    vec3.add(_castRayVsTransformed_worldPos, _castRayVsTransformed_worldPos, _castRayVsTransformed_pos);

    // transform ray from world space to decorated shape's local space
    // first, compute inverse quaternion
    quat.conjugate(_castRayVsTransformed_invQuat, _castRayVsTransformed_worldQuat);

    // transform ray origin: (origin - position) rotated by inverse quaternion
    vec3.subtract(_castRayVsTransformed_rayOriginLocal, ray.origin, _castRayVsTransformed_worldPos);
    vec3.transformQuat(_castRayVsTransformed_rayOriginLocal, _castRayVsTransformed_rayOriginLocal, _castRayVsTransformed_invQuat);

    // transform ray direction: direction rotated by inverse quaternion
    vec3.copy(_castRayVsTransformed_rayDirectionLocal, ray.direction);
    vec3.transformQuat(
        _castRayVsTransformed_rayDirectionLocal,
        _castRayVsTransformed_rayDirectionLocal,
        _castRayVsTransformed_invQuat,
    );

    // prepare transformed ray for inner shape
    vec3.copy(_castRayVsTransformed_tempRay.origin, _castRayVsTransformed_rayOriginLocal);
    vec3.copy(_castRayVsTransformed_tempRay.direction, _castRayVsTransformed_rayDirectionLocal);
    _castRayVsTransformed_tempRay.length = ray.length;

    // cast against the inner shape with identity transform (already accumulated)
    const innerShapeDef = shapeDefs[transformedShape.shape.type];
    innerShapeDef.castRay(
        collector,
        settings,
        _castRayVsTransformed_tempRay,
        transformedShape.shape,
        subShapeId,
        subShapeIdBits,
        0,
        0,
        0, // identity position
        0,
        0,
        0,
        1, // identity quaternion
        scaleX,
        scaleY,
        scaleZ, // pass through scale
    );
}

/* collide point */

const _collidePointVsTransformed_posB = /* @__PURE__ */ vec3.create();
const _collidePointVsTransformed_quatB = /* @__PURE__ */ quat.create();
const _collidePointVsTransformed_transformedTranslation = /* @__PURE__ */ vec3.create();
const _collidePointVsTransformed_worldPos = /* @__PURE__ */ vec3.create();
const _collidePointVsTransformed_worldRot = /* @__PURE__ */ quat.create();

function collidePointVsTransformed(
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: TransformedShape,
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
    const transformed = shapeB as TransformedShape;

    vec3.set(_collidePointVsTransformed_posB, posBX, posBY, posBZ);
    quat.set(_collidePointVsTransformed_quatB, quatBX, quatBY, quatBZ, quatBW);

    // accumulate transform
    vec3.transformQuat(_collidePointVsTransformed_transformedTranslation, transformed.position, _collidePointVsTransformed_quatB);
    vec3.add(
        _collidePointVsTransformed_worldPos,
        _collidePointVsTransformed_posB,
        _collidePointVsTransformed_transformedTranslation,
    );
    quat.multiply(_collidePointVsTransformed_worldRot, _collidePointVsTransformed_quatB, transformed.quaternion);

    const innerShapeDef = shapeDefs[transformed.shape.type];
    innerShapeDef.collidePoint(
        collector,
        settings,
        pointX,
        pointY,
        pointZ,
        transformed.shape,
        subShapeIdB,
        subShapeIdBitsB,
        _collidePointVsTransformed_worldPos[0],
        _collidePointVsTransformed_worldPos[1],
        _collidePointVsTransformed_worldPos[2],
        _collidePointVsTransformed_worldRot[0],
        _collidePointVsTransformed_worldRot[1],
        _collidePointVsTransformed_worldRot[2],
        _collidePointVsTransformed_worldRot[3],
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}

/* collide shape */

const _collideTransformedVsShape_posA = /* @__PURE__ */ vec3.create();
const _collideTransformedVsShape_quatA = /* @__PURE__ */ quat.create();
const _collideTransformedVsShape_transformedTranslation = /* @__PURE__ */ vec3.create();
const _collideTransformedVsShape_worldPos = /* @__PURE__ */ vec3.create();
const _collideTransformedVsShape_worldRot = /* @__PURE__ */ quat.create();

function collideTransformedVsShape(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    subShapeIdBitsA: number,
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
    const transformed = shapeA as TransformedShape;

    vec3.set(_collideTransformedVsShape_posA, posAX, posAY, posAZ);
    quat.set(_collideTransformedVsShape_quatA, quatAX, quatAY, quatAZ, quatAW);

    // accumulate transform
    vec3.transformQuat(_collideTransformedVsShape_transformedTranslation, transformed.position, _collideTransformedVsShape_quatA);
    vec3.add(
        _collideTransformedVsShape_worldPos,
        _collideTransformedVsShape_posA,
        _collideTransformedVsShape_transformedTranslation,
    );
    quat.multiply(_collideTransformedVsShape_worldRot, _collideTransformedVsShape_quatA, transformed.quaternion);

    const fn = collisionDispatch.collideFns.get(transformed.shape.type)?.get(shapeB.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        transformed.shape,
        subShapeIdA,
        subShapeIdBitsA,
        _collideTransformedVsShape_worldPos[0],
        _collideTransformedVsShape_worldPos[1],
        _collideTransformedVsShape_worldPos[2],
        _collideTransformedVsShape_worldRot[0],
        _collideTransformedVsShape_worldRot[1],
        _collideTransformedVsShape_worldRot[2],
        _collideTransformedVsShape_worldRot[3],
        scaleAX,
        scaleAY,
        scaleAZ,
        shapeB,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}

const _collideShapeVsTransformed_posB = /* @__PURE__ */ vec3.create();
const _collideShapeVsTransformed_quatB = /* @__PURE__ */ quat.create();
const _collideShapeVsTransformed_transformedTranslation = /* @__PURE__ */ vec3.create();
const _collideShapeVsTransformed_worldPos = /* @__PURE__ */ vec3.create();
const _collideShapeVsTransformed_worldRot = /* @__PURE__ */ quat.create();

function collideShapeVsTransformed(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    subShapeIdBitsA: number,
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
    const transformed = shapeB as TransformedShape;

    vec3.set(_collideShapeVsTransformed_posB, posBX, posBY, posBZ);
    quat.set(_collideShapeVsTransformed_quatB, quatBX, quatBY, quatBZ, quatBW);

    // accumulate transform
    vec3.transformQuat(_collideShapeVsTransformed_transformedTranslation, transformed.position, _collideShapeVsTransformed_quatB);
    vec3.add(
        _collideShapeVsTransformed_worldPos,
        _collideShapeVsTransformed_posB,
        _collideShapeVsTransformed_transformedTranslation,
    );
    quat.multiply(_collideShapeVsTransformed_worldRot, _collideShapeVsTransformed_quatB, transformed.quaternion);

    const fn = collisionDispatch.collideFns.get(shapeA.type)?.get(transformed.shape.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        subShapeIdBitsA,
        posAX,
        posAY,
        posAZ,
        quatAX,
        quatAY,
        quatAZ,
        quatAW,
        scaleAX,
        scaleAY,
        scaleAZ,
        transformed.shape,
        subShapeIdB,
        subShapeIdBitsB,
        _collideShapeVsTransformed_worldPos[0],
        _collideShapeVsTransformed_worldPos[1],
        _collideShapeVsTransformed_worldPos[2],
        _collideShapeVsTransformed_worldRot[0],
        _collideShapeVsTransformed_worldRot[1],
        _collideShapeVsTransformed_worldRot[2],
        _collideShapeVsTransformed_worldRot[3],
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}

/* cast shape */

const _castTransformedVsShape_castDecorated_temp = /* @__PURE__ */ vec3.create();
const _castTransformedVsShape_worldPos = /* @__PURE__ */ vec3.create();
const _castTransformedVsShape_worldRot = /* @__PURE__ */ quat.create();
const _castTransformedVsShape_transformDisplacementA = /* @__PURE__ */ vec3.create();
const _castTransformedVsShape_posA = /* @__PURE__ */ vec3.create();
const _castTransformedVsShape_quatA = /* @__PURE__ */ quat.create();
const _castTransformedVsShape_displacementA = /* @__PURE__ */ vec3.create();

function castTransformedVsShape(
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    subShapeIdBitsA: number,
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
    const transformed = shapeA as TransformedShape;

    vec3.set(_castTransformedVsShape_posA, posAX, posAY, posAZ);
    quat.set(_castTransformedVsShape_quatA, quatAX, quatAY, quatAZ, quatAW);

    // accumulate transform
    vec3.transformQuat(_castTransformedVsShape_castDecorated_temp, transformed.position, _castTransformedVsShape_quatA);
    vec3.add(_castTransformedVsShape_worldPos, _castTransformedVsShape_posA, _castTransformedVsShape_castDecorated_temp);

    quat.multiply(_castTransformedVsShape_worldRot, _castTransformedVsShape_quatA, transformed.quaternion);

    // transform displacement - rotate by accumulated quaternion
    vec3.set(_castTransformedVsShape_displacementA, dispAX, dispAY, dispAZ);
    vec3.transformQuat(
        _castTransformedVsShape_transformDisplacementA,
        _castTransformedVsShape_displacementA,
        _castTransformedVsShape_quatA,
    );

    const fn = collisionDispatch.castFns.get(transformed.shape.type)?.get(shapeB.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        transformed.shape,
        subShapeIdA,
        subShapeIdBitsA,
        _castTransformedVsShape_worldPos[0],
        _castTransformedVsShape_worldPos[1],
        _castTransformedVsShape_worldPos[2],
        _castTransformedVsShape_worldRot[0],
        _castTransformedVsShape_worldRot[1],
        _castTransformedVsShape_worldRot[2],
        _castTransformedVsShape_worldRot[3],
        scaleAX,
        scaleAY,
        scaleAZ,
        _castTransformedVsShape_transformDisplacementA[0],
        _castTransformedVsShape_transformDisplacementA[1],
        _castTransformedVsShape_transformDisplacementA[2],
        shapeB,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}

const _castShapeVsTransformed_castDecorated_temp = /* @__PURE__ */ vec3.create();
const _castShapeVsTransformed_worldPos = /* @__PURE__ */ vec3.create();
const _castShapeVsTransformed_worldRot = /* @__PURE__ */ quat.create();
const _castShapeVsTransformed_posB = /* @__PURE__ */ vec3.create();
const _castShapeVsTransformed_quatB = /* @__PURE__ */ quat.create();

function castShapeVsTransformed(
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    subShapeIdBitsA: number,
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
    const transformed = shapeB as TransformedShape;

    vec3.set(_castShapeVsTransformed_posB, posBX, posBY, posBZ);
    quat.set(_castShapeVsTransformed_quatB, quatBX, quatBY, quatBZ, quatBW);

    vec3.transformQuat(_castShapeVsTransformed_castDecorated_temp, transformed.position, _castShapeVsTransformed_quatB);
    vec3.add(_castShapeVsTransformed_worldPos, _castShapeVsTransformed_posB, _castShapeVsTransformed_castDecorated_temp);

    quat.multiply(_castShapeVsTransformed_worldRot, _castShapeVsTransformed_quatB, transformed.quaternion);

    const fn = collisionDispatch.castFns.get(shapeA.type)?.get(transformed.shape.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        subShapeIdBitsA,
        posAX,
        posAY,
        posAZ,
        quatAX,
        quatAY,
        quatAZ,
        quatAW,
        scaleAX,
        scaleAY,
        scaleAZ,
        dispAX,
        dispAY,
        dispAZ,
        transformed.shape,
        subShapeIdB,
        subShapeIdBitsB,
        _castShapeVsTransformed_worldPos[0],
        _castShapeVsTransformed_worldPos[1],
        _castShapeVsTransformed_worldPos[2],
        _castShapeVsTransformed_worldRot[0],
        _castShapeVsTransformed_worldRot[1],
        _castShapeVsTransformed_worldRot[2],
        _castShapeVsTransformed_worldRot[3],
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}
