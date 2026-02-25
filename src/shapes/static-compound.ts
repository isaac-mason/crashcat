import { type Box3, box3, mat3, mat4, quat, raycast3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as massProperties from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape';
import type { CastShapeCollector, CastShapeSettings } from '../collision/cast-shape-vs-shape';
import { rayDistanceToBox3 } from '../collision/cast-utils';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape';
import { assert } from '../utils/assert';
import * as bvhStack from '../utils/bvh-stack';
import type { CompoundShapeChild } from './compound';
import {
    collisionDispatch,
    defineShape,
    type GetLeafShapeResult,
    type GetSubShapeTransformedShapeResult,
    getShapeInnerRadius,
    type Shape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
    shapeDefs,
} from './shapes';
import * as bvh from './utils/bvh';
import type { BvhBuildSettings, StaticCompoundBVH } from './utils/static-compound-bvh';
import * as staticCompoundBvh from './utils/static-compound-bvh';

export type StaticCompoundShapeSettings = {
    children: CompoundShapeChild[];
    /** maximum children per leaf node (default: 4) */
    bvhMaxLeafChildren?: number;
};

export const DEFAULT_STATIC_COMPOUND_OPTIONS = {
    bvhMaxLeafChildren: 4,
};

export type StaticCompoundShape = {
    type: ShapeType.STATIC_COMPOUND;
    children: CompoundShapeChild[];
    bvh: StaticCompoundBVH;
    bvhSettings: BvhBuildSettings;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

export function create(o: StaticCompoundShapeSettings): StaticCompoundShape {
    const bvhSettings: BvhBuildSettings = {
        maxLeafChildren: o.bvhMaxLeafChildren ?? DEFAULT_STATIC_COMPOUND_OPTIONS.bvhMaxLeafChildren,
    };

    const shape: StaticCompoundShape = {
        type: ShapeType.STATIC_COMPOUND,
        children: [...o.children],
        bvh: { buffer: [] },
        bvhSettings,
        aabb: box3.create(),
        centerOfMass: vec3.create(),
        volume: 0,
    };

    update(shape);

    return shape;
}

function computeVolume(shape: StaticCompoundShape): number {
    let totalVolume = 0;
    for (const child of shape.children) {
        totalVolume += child.shape.volume;
    }
    return totalVolume;
}

const _computeLocalBounds_transformed = /* @__PURE__ */ vec3.create();

function computeLocalBounds(out: Box3, shape: StaticCompoundShape): void {
    box3.empty(out);

    for (const child of shape.children) {
        const childAABB = child.shape.aabb;
        const minX = childAABB[0], minY = childAABB[1], minZ = childAABB[2];
        const maxX = childAABB[3], maxY = childAABB[4], maxZ = childAABB[5];

        for (let x = 0; x < 2; x++) {
            for (let y = 0; y < 2; y++) {
                for (let z = 0; z < 2; z++) {
                    _computeLocalBounds_transformed[0] = x === 0 ? minX : maxX;
                    _computeLocalBounds_transformed[1] = y === 0 ? minY : maxY;
                    _computeLocalBounds_transformed[2] = z === 0 ? minZ : maxZ;

                    vec3.transformQuat(_computeLocalBounds_transformed, _computeLocalBounds_transformed, child.quaternion);
                    vec3.add(_computeLocalBounds_transformed, _computeLocalBounds_transformed, child.position);

                    box3.expandByPoint(out, out, _computeLocalBounds_transformed);
                }
            }
        }
    }
}

const _computeCenterOfMass_childCOM = /* @__PURE__ */ vec3.create();
const _computeCenterOfMass_worldChildCOM = /* @__PURE__ */ vec3.create();
const _computeMassProperties_childMass = /* @__PURE__ */ massProperties.create();

function computeCenterOfMass(out: Vec3, shape: StaticCompoundShape): void {
    vec3.zero(out);
    let totalMass = 0;

    const childMass = _computeMassProperties_childMass;

    for (const child of shape.children) {
        const shapeDef = shapeDefs[child.shape.type];
        shapeDef.computeMassProperties(childMass, child.shape);

        vec3.transformQuat(_computeCenterOfMass_childCOM, child.shape.centerOfMass, child.quaternion);
        vec3.add(_computeCenterOfMass_worldChildCOM, child.position, _computeCenterOfMass_childCOM);

        out[0] += _computeCenterOfMass_worldChildCOM[0] * childMass.mass;
        out[1] += _computeCenterOfMass_worldChildCOM[1] * childMass.mass;
        out[2] += _computeCenterOfMass_worldChildCOM[2] * childMass.mass;

        totalMass += childMass.mass;
    }

    if (totalMass > 0) {
        out[0] /= totalMass;
        out[1] /= totalMass;
        out[2] /= totalMass;
    }
}

/**
 * updates a static compound shape after properties have changed.
 * recomputes bounds, center of mass, volume, and rebuilds BVH.
 * note: this reorders the children array and invalidates any existing SubShapeIDs.
 */
export function update(shape: StaticCompoundShape): void {
    // rebuild BVH (reorders the children array)
    shape.bvh = staticCompoundBvh.build(shape.children, shape.bvhSettings);

    // recompute derived properties
    computeLocalBounds(shape.aabb, shape);
    computeCenterOfMass(shape.centerOfMass, shape);
    shape.volume = computeVolume(shape);
}

export const def = /* @__PURE__ */ (() =>
    defineShape<StaticCompoundShape>({
        type: ShapeType.STATIC_COMPOUND,
        category: ShapeCategory.COMPOSITE,
        computeMassProperties,
        getSurfaceNormal,
        getSupportingFace,
        getInnerRadius,
        getLeafShape,
        getSubShapeTransformedShape,
        castRay,
        collidePoint,
        register,
    }))();

function register(): void {
    for (const shapeDef of Object.values(shapeDefs)) {
        setCollideShapeFn(ShapeType.STATIC_COMPOUND, shapeDef.type, collideStaticCompoundVsShape);
        setCollideShapeFn(shapeDef.type, ShapeType.STATIC_COMPOUND, collideShapeVsStaticCompound);

        setCastShapeFn(ShapeType.STATIC_COMPOUND, shapeDef.type, castStaticCompoundVsShape);
        setCastShapeFn(shapeDef.type, ShapeType.STATIC_COMPOUND, castShapeVsStaticCompound);
    }
}

const _computeMassProperties_rotatedInertia = /* @__PURE__ */ massProperties.create();
const _computeMassProperties_childCOM = /* @__PURE__ */ vec3.create();
const _computeMassProperties_childCOMRelative = /* @__PURE__ */ vec3.create();
const _computeMassProperties_childRotMat3 = /* @__PURE__ */ mat3.create();
const _computeMassProperties_childRotMat4 = /* @__PURE__ */ mat4.create();

const _subShapeIdPopResult = /* @__PURE__ */ subShape.popResult();

function computeMassProperties(out: MassProperties, shape: StaticCompoundShape): void {
    out.mass = 0;

    for (let i = 0; i < 16; i++) {
        out.inertia[i] = 0;
    }

    if (shape.children.length === 0) {
        out.inertia[15] = 1.0;
        return;
    }

    const childMass = _computeMassProperties_childMass;
    const rotatedInertia = _computeMassProperties_rotatedInertia;
    const childRotMat3 = _computeMassProperties_childRotMat3;
    const childRotMat4 = _computeMassProperties_childRotMat4;

    for (const child of shape.children) {
        const shapeDef = shapeDefs[child.shape.type];
        shapeDef.computeMassProperties(childMass, child.shape);

        vec3.transformQuat(_computeMassProperties_childCOM, child.shape.centerOfMass, child.quaternion);
        vec3.add(_computeMassProperties_childCOM, _computeMassProperties_childCOM, child.position);
        vec3.subtract(_computeMassProperties_childCOMRelative, _computeMassProperties_childCOM, shape.centerOfMass);

        out.mass += childMass.mass;

        mat3.fromQuat(childRotMat3, child.quaternion);
        mat4.identity(childRotMat4);
        for (let i = 0; i < 3; i++) {
            for (let j = 0; j < 3; j++) {
                childRotMat4[i + j * 4] = childRotMat3[i + j * 3];
            }
        }

        massProperties.rotate(rotatedInertia, childMass, childRotMat4);
        massProperties.translate(rotatedInertia, rotatedInertia, _computeMassProperties_childCOMRelative);

        for (let i = 0; i < 15; i++) {
            out.inertia[i] += rotatedInertia.inertia[i];
        }
    }

    out.inertia[15] = 1.0;
}

const _getSurfaceNormal_invRotation = /* @__PURE__ */ quat.create();
const _getSurfaceNormal_forwardRotation = /* @__PURE__ */ quat.create();

function getSurfaceNormal(ioResult: SurfaceNormalResult, shape: StaticCompoundShape, subShapeId: number): void {
    subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);

    const childIndex = _subShapeIdPopResult.value;
    const remainder = _subShapeIdPopResult.remainder;

    if (childIndex < 0 || childIndex >= shape.children.length) {
        assert(false, 'Invalid SubShapeID for StaticCompoundShape');
        return;
    }

    const child = shape.children[childIndex];

    vec3.subtract(ioResult.position, ioResult.position, child.position);

    const invRotation = quat.conjugate(_getSurfaceNormal_invRotation, child.quaternion);
    quat.multiply(ioResult.quaternion, invRotation, ioResult.quaternion);

    vec3.transformQuat(ioResult.position, ioResult.position, invRotation);

    shapeDefs[child.shape.type].getSurfaceNormal(ioResult, child.shape, remainder);

    const forwardAccumulatedRotation = quat.conjugate(_getSurfaceNormal_forwardRotation, ioResult.quaternion);
    vec3.transformQuat(ioResult.normal, ioResult.normal, forwardAccumulatedRotation);
}

const _getSupportingFace_childTransform = /* @__PURE__ */ mat4.create();
const _getSupportingFace_localDirection = /* @__PURE__ */ vec3.create();

function getSupportingFace(
    ioResult: SupportingFaceResult,
    direction: Vec3,
    shape: StaticCompoundShape,
    subShapeId: number,
): void {
    subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);
    const childIndex = _subShapeIdPopResult.value;
    const remainder = _subShapeIdPopResult.remainder;

    if (childIndex < 0 || childIndex >= shape.children.length) {
        assert(false, 'Invalid SubShapeID for StaticCompoundShape');
        return;
    }

    const child = shape.children[childIndex];

    // build child transform matrix and accumulate: transform = transform * childTransform
    mat4.fromRotationTranslation(_getSupportingFace_childTransform, child.quaternion, child.position);
    mat4.multiply(ioResult.transform, ioResult.transform, _getSupportingFace_childTransform);

    // transform direction to child local space using inverse child rotation
    // inverse rotation = conjugate for unit quaternions, same as transposed 3x3 portion
    mat4.multiply3x3TransposedVec(_getSupportingFace_localDirection, _getSupportingFace_childTransform, direction);

    shapeDefs[child.shape.type].getSupportingFace(ioResult, _getSupportingFace_localDirection, child.shape, remainder);
}

function getInnerRadius(shape: StaticCompoundShape): number {
    let innerRadius = Number.MAX_VALUE;
    for (const child of shape.children) {
        innerRadius = Math.min(innerRadius, getShapeInnerRadius(child.shape));
    }
    return innerRadius;
}

function getLeafShape(out: GetLeafShapeResult, shape: StaticCompoundShape, subShapeId: number): void {
    if (subShape.isEmpty(subShapeId)) {
        out.shape = null;
        out.remainder = subShape.EMPTY_SUB_SHAPE_ID;
        return;
    }

    subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);
    const childShape = shape.children[_subShapeIdPopResult.value].shape;

    const childShapeDef = shapeDefs[childShape.type];
    childShapeDef.getLeafShape(out, childShape, _subShapeIdPopResult.remainder);
}

const _getStaticCompoundSubShapeTransformedShape_rotatedChildPos = /* @__PURE__ */ vec3.create();

function getSubShapeTransformedShape(
    out: GetSubShapeTransformedShapeResult,
    shape: StaticCompoundShape,
    subShapeId: number,
): void {
    if (subShape.isEmpty(subShapeId)) {
        out.shape = null;
        out.remainder = subShape.EMPTY_SUB_SHAPE_ID;
        return;
    }

    subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);
    const child = shape.children[_subShapeIdPopResult.value];

    vec3.transformQuat(_getStaticCompoundSubShapeTransformedShape_rotatedChildPos, child.position, out.rotation);
    vec3.add(out.position, out.position, _getStaticCompoundSubShapeTransformedShape_rotatedChildPos);
    quat.multiply(out.rotation, out.rotation, child.quaternion);

    const childShapeDef = shapeDefs[child.shape.type];
    childShapeDef.getSubShapeTransformedShape(out, child.shape, _subShapeIdPopResult.remainder);
}

/* cast ray */

const _castRayVsStaticCompound_stack = /* @__PURE__ */ bvhStack.create();

const _castRayVsStaticCompound_pos = /* @__PURE__ */ vec3.create();
const _castRayVsStaticCompound_quat = /* @__PURE__ */ quat.create();
const _castRayVsStaticCompound_invQuat = /* @__PURE__ */ quat.create();

const _castRayVsStaticCompound_localRayOrigin = /* @__PURE__ */ vec3.create();
const _castRayVsStaticCompound_localRayDir = /* @__PURE__ */ vec3.create();

const _castRayVsStaticCompound_transformedTranslation = /* @__PURE__ */ vec3.create();
const _castRayVsStaticCompound_worldPos = /* @__PURE__ */ vec3.create();
const _castRayVsStaticCompound_worldRot = /* @__PURE__ */ quat.create();

const _castRayVsStaticCompound_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

const _castRayVsStaticCompound_nodeBounds = /* @__PURE__ */ box3.create();

function castRay(
    collector: CastRayCollector,
    settings: CastRaySettings,
    originX: number,
    originY: number,
    originZ: number,
    directionX: number,
    directionY: number,
    directionZ: number,
    length: number,
    shape: StaticCompoundShape,
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
    const buffer = shape.bvh.buffer;

    if (buffer.length === 0) {
        return;
    }

    vec3.set(_castRayVsStaticCompound_pos, posX, posY, posZ);
    quat.set(_castRayVsStaticCompound_quat, quatX, quatY, quatZ, quatW);

    // transform ray from world space to compound local space
    quat.conjugate(_castRayVsStaticCompound_invQuat, _castRayVsStaticCompound_quat);

    // set ray origin from parameters
    vec3.set(_castRayVsStaticCompound_localRayOrigin, originX, originY, originZ);
    // transform ray origin: (origin - position) rotated by inverse quaternion
    vec3.subtract(_castRayVsStaticCompound_localRayOrigin, _castRayVsStaticCompound_localRayOrigin, _castRayVsStaticCompound_pos);
    vec3.transformQuat(
        _castRayVsStaticCompound_localRayOrigin,
        _castRayVsStaticCompound_localRayOrigin,
        _castRayVsStaticCompound_invQuat,
    );

    // set ray direction from parameters
    vec3.set(_castRayVsStaticCompound_localRayDir, directionX, directionY, directionZ);
    // transform ray direction: direction rotated by inverse quaternion
    vec3.transformQuat(
        _castRayVsStaticCompound_localRayDir,
        _castRayVsStaticCompound_localRayDir,
        _castRayVsStaticCompound_invQuat,
    );

    // store transformed ray components for bvh traversal
    const localOriginX = _castRayVsStaticCompound_localRayOrigin[0];
    const localOriginY = _castRayVsStaticCompound_localRayOrigin[1];
    const localOriginZ = _castRayVsStaticCompound_localRayOrigin[2];
    const localDirX = _castRayVsStaticCompound_localRayDir[0];
    const localDirY = _castRayVsStaticCompound_localRayDir[1];
    const localDirZ = _castRayVsStaticCompound_localRayDir[2];

    bvhStack.reset(_castRayVsStaticCompound_stack);
    bvhStack.push(_castRayVsStaticCompound_stack, 0, -Infinity); // root always visited

    while (_castRayVsStaticCompound_stack.size > 0) {
        // early out: very close hit
        if (collector.earlyOutFraction <= 0) {
            break;
        }

        const entry = bvhStack.pop(_castRayVsStaticCompound_stack)!;

        // early out: if fraction to this node >= closest hit, skip it
        if (entry.distance >= collector.earlyOutFraction) {
            continue;
        }

        const nodeOffset = entry.nodeIndex;

        // note: no need to test ray x child bounds intersection here - we already proved the ray
        // intersects this node's bounds when we computed the distance during push.
        // if the ray didn't intersect, rayDistanceToBox3 would have returned Infinity
        // and we wouldn't have pushed this node onto the stack.

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf: check children
            const childStart = staticCompoundBvh.nodeChildStart(buffer, nodeOffset);
            const childCount = staticCompoundBvh.nodeChildCount(buffer, nodeOffset);

            for (let i = 0; i < childCount; i++) {
                const childIndex = childStart + i;
                const child = shape.children[childIndex];

                _castRayVsStaticCompound_subShapeIdBuilder.value = subShapeId;
                _castRayVsStaticCompound_subShapeIdBuilder.currentBit = subShapeIdBits;
                subShape.pushIndex(
                    _castRayVsStaticCompound_subShapeIdBuilder,
                    _castRayVsStaticCompound_subShapeIdBuilder,
                    childIndex,
                    shape.children.length,
                );

                vec3.transformQuat(
                    _castRayVsStaticCompound_transformedTranslation,
                    child.position,
                    _castRayVsStaticCompound_quat,
                );
                vec3.add(
                    _castRayVsStaticCompound_worldPos,
                    _castRayVsStaticCompound_pos,
                    _castRayVsStaticCompound_transformedTranslation,
                );
                quat.multiply(_castRayVsStaticCompound_worldRot, _castRayVsStaticCompound_quat, child.quaternion);

                const childShapeDef = shapeDefs[child.shape.type];
                childShapeDef.castRay(
                    collector,
                    settings,
                    localOriginX,
                    localOriginY,
                    localOriginZ,
                    localDirX,
                    localDirY,
                    localDirZ,
                    length,
                    child.shape,
                    _castRayVsStaticCompound_subShapeIdBuilder.value,
                    _castRayVsStaticCompound_subShapeIdBuilder.currentBit,
                    _castRayVsStaticCompound_worldPos[0],
                    _castRayVsStaticCompound_worldPos[1],
                    _castRayVsStaticCompound_worldPos[2],
                    _castRayVsStaticCompound_worldRot[0],
                    _castRayVsStaticCompound_worldRot[1],
                    _castRayVsStaticCompound_worldRot[2],
                    _castRayVsStaticCompound_worldRot[3],
                    scaleX,
                    scaleY,
                    scaleZ,
                );
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            // push farther child first so closer is popped first
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);

            bvh.nodeGetBounds(_castRayVsStaticCompound_nodeBounds, buffer, leftOffset);
            const leftDist = rayDistanceToBox3(
                localOriginX,
                localOriginY,
                localOriginZ,
                localDirX,
                localDirY,
                localDirZ,
                length,
                _castRayVsStaticCompound_nodeBounds,
            );

            bvh.nodeGetBounds(_castRayVsStaticCompound_nodeBounds, buffer, rightOffset);
            const rightDist = rayDistanceToBox3(
                localOriginX,
                localOriginY,
                localOriginZ,
                localDirX,
                localDirY,
                localDirZ,
                length,
                _castRayVsStaticCompound_nodeBounds,
            );

            // push farther child first (so closer child is on top of stack)
            if (leftDist <= rightDist) {
                // left is closer or equal - push right first
                if (rightDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsStaticCompound_stack, rightOffset, rightDist);
                }
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsStaticCompound_stack, leftOffset, leftDist);
                }
            } else {
                // right is closer - push left first
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsStaticCompound_stack, leftOffset, leftDist);
                }
                if (rightDist < collector.earlyOutFraction) {
                    bvhStack.push(_castRayVsStaticCompound_stack, rightOffset, rightDist);
                }
            }
        }
    }
}

/* collide point */

const _collidePointVsStaticCompound_stack = /* @__PURE__ */ bvhStack.create();

const _collidePointVsStaticCompound_posB = /* @__PURE__ */ vec3.create();
const _collidePointVsStaticCompound_quatB = /* @__PURE__ */ quat.create();
const _collidePointVsStaticCompound_invQuatB = /* @__PURE__ */ quat.create();

const _collidePointVsStaticCompound_localPoint = /* @__PURE__ */ vec3.create();

const _collidePointVsStaticCompound_transformedTranslation = /* @__PURE__ */ vec3.create();
const _collidePointVsStaticCompound_worldPos = /* @__PURE__ */ vec3.create();
const _collidePointVsStaticCompound_worldRot = /* @__PURE__ */ quat.create();

const _collidePointVsStaticCompound_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

function collidePoint(
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: StaticCompoundShape,
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
    const buffer = shapeB.bvh.buffer;
    if (buffer.length === 0) return;

    // transform point into compound's local space for BVH traversal
    vec3.set(_collidePointVsStaticCompound_posB, posBX, posBY, posBZ);
    quat.set(_collidePointVsStaticCompound_quatB, quatBX, quatBY, quatBZ, quatBW);
    quat.conjugate(_collidePointVsStaticCompound_invQuatB, _collidePointVsStaticCompound_quatB);

    // localPoint = invQuatB * (worldPoint - posB)
    vec3.set(_collidePointVsStaticCompound_localPoint, pointX - posBX, pointY - posBY, pointZ - posBZ);
    vec3.transformQuat(
        _collidePointVsStaticCompound_localPoint,
        _collidePointVsStaticCompound_localPoint,
        _collidePointVsStaticCompound_invQuatB,
    );
    const localPointX = _collidePointVsStaticCompound_localPoint[0];
    const localPointY = _collidePointVsStaticCompound_localPoint[1];
    const localPointZ = _collidePointVsStaticCompound_localPoint[2];

    bvhStack.reset(_collidePointVsStaticCompound_stack);
    bvhStack.push(_collidePointVsStaticCompound_stack, 0, 0);

    while (_collidePointVsStaticCompound_stack.size > 0) {
        if (collector.shouldEarlyOut()) break;

        const entry = bvhStack.pop(_collidePointVsStaticCompound_stack)!;
        const nodeOffset = entry.nodeIndex;

        // point-AABB test using LOCAL point
        if (
            !bvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                localPointX,
                localPointY,
                localPointZ,
                localPointX,
                localPointY,
                localPointZ,
            )
        ) {
            continue;
        }

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            const childStart = staticCompoundBvh.nodeChildStart(buffer, nodeOffset);
            const childCount = staticCompoundBvh.nodeChildCount(buffer, nodeOffset);

            for (let i = 0; i < childCount; i++) {
                if (collector.shouldEarlyOut()) break;

                const childIndex = childStart + i;
                const child = shapeB.children[childIndex];

                _collidePointVsStaticCompound_subShapeIdBuilder.value = subShapeIdB;
                _collidePointVsStaticCompound_subShapeIdBuilder.currentBit = subShapeIdBitsB;
                subShape.pushIndex(
                    _collidePointVsStaticCompound_subShapeIdBuilder,
                    _collidePointVsStaticCompound_subShapeIdBuilder,
                    childIndex,
                    shapeB.children.length,
                );

                vec3.set(_collidePointVsStaticCompound_posB, posBX, posBY, posBZ);
                quat.set(_collidePointVsStaticCompound_quatB, quatBX, quatBY, quatBZ, quatBW);
                vec3.transformQuat(
                    _collidePointVsStaticCompound_transformedTranslation,
                    child.position,
                    _collidePointVsStaticCompound_quatB,
                );
                vec3.add(
                    _collidePointVsStaticCompound_worldPos,
                    _collidePointVsStaticCompound_posB,
                    _collidePointVsStaticCompound_transformedTranslation,
                );
                quat.multiply(_collidePointVsStaticCompound_worldRot, _collidePointVsStaticCompound_quatB, child.quaternion);

                const childShapeDef = shapeDefs[child.shape.type];
                childShapeDef.collidePoint(
                    collector,
                    settings,
                    pointX,
                    pointY,
                    pointZ,
                    child.shape,
                    _collidePointVsStaticCompound_subShapeIdBuilder.value,
                    _collidePointVsStaticCompound_subShapeIdBuilder.currentBit,
                    _collidePointVsStaticCompound_worldPos[0],
                    _collidePointVsStaticCompound_worldPos[1],
                    _collidePointVsStaticCompound_worldPos[2],
                    _collidePointVsStaticCompound_worldRot[0],
                    _collidePointVsStaticCompound_worldRot[1],
                    _collidePointVsStaticCompound_worldRot[2],
                    _collidePointVsStaticCompound_worldRot[3],
                    scaleBX,
                    scaleBY,
                    scaleBZ,
                );
            }
        } else {
            // internal node: push both children
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);
            bvhStack.push(_collidePointVsStaticCompound_stack, leftOffset, 0);
            bvhStack.push(_collidePointVsStaticCompound_stack, rightOffset, 0);
        }
    }
}

/* collide shape */

const _collideStaticCompoundVsShape_stack = /* @__PURE__ */ bvhStack.create();
const _collideStaticCompoundVsShape_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

const _collideStaticCompoundVsShape_posA = /* @__PURE__ */ vec3.create();
const _collideStaticCompoundVsShape_quatA = /* @__PURE__ */ quat.create();
const _collideStaticCompoundVsShape_invQuatA = /* @__PURE__ */ quat.create();

const _collideStaticCompoundVsShape_worldPos = /* @__PURE__ */ vec3.create();
const _collideStaticCompoundVsShape_worldRot = /* @__PURE__ */ quat.create();
const _collideStaticCompoundVsShape_transformedTranslation = /* @__PURE__ */ vec3.create();

const _collideStaticCompoundVsShape_queryBounds = /* @__PURE__ */ box3.create();
const _collideStaticCompoundVsShape_aabbTransform = /* @__PURE__ */ mat4.create();

const _collideStaticCompoundVsShape_localPosB = /* @__PURE__ */ vec3.create();
const _collideStaticCompoundVsShape_localQuatB = /* @__PURE__ */ quat.create();

function collideStaticCompoundVsShape(
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
    const compound = shapeA as StaticCompoundShape;
    const buffer = compound.bvh.buffer;

    if (buffer.length === 0) return;

    // compute shapeB world bounds and transform to compound A's local space for BVH culling
    vec3.set(_collideStaticCompoundVsShape_posA, posAX, posAY, posAZ);
    quat.set(_collideStaticCompoundVsShape_quatA, quatAX, quatAY, quatAZ, quatAW);
    quat.conjugate(_collideStaticCompoundVsShape_invQuatA, _collideStaticCompoundVsShape_quatA);

    // compute shapeB's position in compound A's local space
    vec3.set(_collideStaticCompoundVsShape_localPosB, posBX - posAX, posBY - posAY, posBZ - posAZ);
    vec3.transformQuat(
        _collideStaticCompoundVsShape_localPosB,
        _collideStaticCompoundVsShape_localPosB,
        _collideStaticCompoundVsShape_invQuatA,
    );

    // compute shapeB's rotation in compound A's local space
    quat.set(_collideStaticCompoundVsShape_localQuatB, quatBX, quatBY, quatBZ, quatBW);
    quat.multiply(
        _collideStaticCompoundVsShape_localQuatB,
        _collideStaticCompoundVsShape_invQuatA,
        _collideStaticCompoundVsShape_localQuatB,
    );

    // transform shapeB's local AABB to compound A's local space
    const queryBounds = _collideStaticCompoundVsShape_queryBounds;
    mat4.fromRotationTranslation(
        _collideStaticCompoundVsShape_aabbTransform,
        _collideStaticCompoundVsShape_localQuatB,
        _collideStaticCompoundVsShape_localPosB,
    );
    box3.transformMat4(queryBounds, shapeB.aabb, _collideStaticCompoundVsShape_aabbTransform);

    // expand by max separation distance
    queryBounds[0] -= settings.maxSeparationDistance;
    queryBounds[1] -= settings.maxSeparationDistance;
    queryBounds[2] -= settings.maxSeparationDistance;
    queryBounds[3] += settings.maxSeparationDistance;
    queryBounds[4] += settings.maxSeparationDistance;
    queryBounds[5] += settings.maxSeparationDistance;

    bvhStack.reset(_collideStaticCompoundVsShape_stack);
    bvhStack.push(_collideStaticCompoundVsShape_stack, 0, 0);

    while (_collideStaticCompoundVsShape_stack.size > 0) {
        if (collector.shouldEarlyOut()) break;

        const entry = bvhStack.pop(_collideStaticCompoundVsShape_stack)!;
        const nodeOffset = entry.nodeIndex;

        // AABB-AABB test using transformed bounds
        if (
            !bvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                queryBounds[0],
                queryBounds[1],
                queryBounds[2],
                queryBounds[3],
                queryBounds[4],
                queryBounds[5],
            )
        ) {
            continue;
        }

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            const childStart = staticCompoundBvh.nodeChildStart(buffer, nodeOffset);
            const childCount = staticCompoundBvh.nodeChildCount(buffer, nodeOffset);

            for (let i = 0; i < childCount; i++) {
                if (collector.shouldEarlyOut()) break;

                const childIndex = childStart + i;
                const child = compound.children[childIndex];

                _collideStaticCompoundVsShape_subShapeIdBuilder.value = subShapeIdA;
                _collideStaticCompoundVsShape_subShapeIdBuilder.currentBit = subShapeIdBitsA;
                subShape.pushIndex(
                    _collideStaticCompoundVsShape_subShapeIdBuilder,
                    _collideStaticCompoundVsShape_subShapeIdBuilder,
                    childIndex,
                    compound.children.length,
                );

                vec3.set(_collideStaticCompoundVsShape_posA, posAX, posAY, posAZ);
                quat.set(_collideStaticCompoundVsShape_quatA, quatAX, quatAY, quatAZ, quatAW);
                vec3.transformQuat(
                    _collideStaticCompoundVsShape_transformedTranslation,
                    child.position,
                    _collideStaticCompoundVsShape_quatA,
                );
                vec3.add(
                    _collideStaticCompoundVsShape_worldPos,
                    _collideStaticCompoundVsShape_posA,
                    _collideStaticCompoundVsShape_transformedTranslation,
                );
                quat.multiply(_collideStaticCompoundVsShape_worldRot, _collideStaticCompoundVsShape_quatA, child.quaternion);

                const fn = collisionDispatch.collideFns.get(child.shape.type)?.get(shapeB.type);
                if (fn) {
                    fn(
                        collector,
                        settings,
                        child.shape,
                        _collideStaticCompoundVsShape_subShapeIdBuilder.value,
                        _collideStaticCompoundVsShape_subShapeIdBuilder.currentBit,
                        _collideStaticCompoundVsShape_worldPos[0],
                        _collideStaticCompoundVsShape_worldPos[1],
                        _collideStaticCompoundVsShape_worldPos[2],
                        _collideStaticCompoundVsShape_worldRot[0],
                        _collideStaticCompoundVsShape_worldRot[1],
                        _collideStaticCompoundVsShape_worldRot[2],
                        _collideStaticCompoundVsShape_worldRot[3],
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
            }
        } else {
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);
            bvhStack.push(_collideStaticCompoundVsShape_stack, leftOffset, 0);
            bvhStack.push(_collideStaticCompoundVsShape_stack, rightOffset, 0);
        }
    }
}

const _collideShapeVsStaticCompound_stack = /* @__PURE__ */ bvhStack.create();
const _collideShapeVsStaticCompound_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();
const _collideShapeVsStaticCompound_posB = /* @__PURE__ */ vec3.create();
const _collideShapeVsStaticCompound_quatB = /* @__PURE__ */ quat.create();
const _collideShapeVsStaticCompound_invQuatB = /* @__PURE__ */ quat.create();
const _collideShapeVsStaticCompound_worldPos = /* @__PURE__ */ vec3.create();
const _collideShapeVsStaticCompound_worldRot = /* @__PURE__ */ quat.create();
const _collideShapeVsStaticCompound_transformedTranslation = /* @__PURE__ */ vec3.create();
const _collideShapeVsStaticCompound_queryBounds = /* @__PURE__ */ box3.create();
const _collideShapeVsStaticCompound_localPosA = /* @__PURE__ */ vec3.create();
const _collideShapeVsStaticCompound_localQuatA = /* @__PURE__ */ quat.create();
const _collideShapeVsStaticCompound_aabbTransform = /* @__PURE__ */ mat4.create();
const _collideShapeVsStaticCompound_scaleA = /* @__PURE__ */ vec3.create();

function collideShapeVsStaticCompound(
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
    const compound = shapeB as StaticCompoundShape;
    const buffer = compound.bvh.buffer;

    if (buffer.length === 0) return;

    // compute shapeA world bounds and transform to compound B's local space for BVH culling
    // first get the world transform of shapeA
    vec3.set(_collideShapeVsStaticCompound_posB, posBX, posBY, posBZ);
    quat.set(_collideShapeVsStaticCompound_quatB, quatBX, quatBY, quatBZ, quatBW);
    quat.conjugate(_collideShapeVsStaticCompound_invQuatB, _collideShapeVsStaticCompound_quatB);

    // compute shapeA's position in compound B's local space
    vec3.set(_collideShapeVsStaticCompound_localPosA, posAX - posBX, posAY - posBY, posAZ - posBZ);
    vec3.transformQuat(
        _collideShapeVsStaticCompound_localPosA,
        _collideShapeVsStaticCompound_localPosA,
        _collideShapeVsStaticCompound_invQuatB,
    );

    // compute shapeA's rotation in compound B's local space
    quat.set(_collideShapeVsStaticCompound_localQuatA, quatAX, quatAY, quatAZ, quatAW);
    quat.multiply(
        _collideShapeVsStaticCompound_localQuatA,
        _collideShapeVsStaticCompound_invQuatB,
        _collideShapeVsStaticCompound_localQuatA,
    );

    vec3.set(_collideShapeVsStaticCompound_scaleA, scaleAX, scaleAY, scaleAZ);

    // transform shapeA's local AABB to compound B's local space
    const queryBounds = _collideShapeVsStaticCompound_queryBounds;
    const aabbMatrix = mat4.fromRotationTranslationScale(
        _collideShapeVsStaticCompound_aabbTransform,
        _collideShapeVsStaticCompound_localQuatA,
        _collideShapeVsStaticCompound_localPosA,
        _collideShapeVsStaticCompound_scaleA,
    );
    box3.transformMat4(queryBounds, shapeA.aabb, aabbMatrix);

    // expand by max separation distance
    queryBounds[0] -= settings.maxSeparationDistance;
    queryBounds[1] -= settings.maxSeparationDistance;
    queryBounds[2] -= settings.maxSeparationDistance;
    queryBounds[3] += settings.maxSeparationDistance;
    queryBounds[4] += settings.maxSeparationDistance;
    queryBounds[5] += settings.maxSeparationDistance;

    bvhStack.reset(_collideShapeVsStaticCompound_stack);
    bvhStack.push(_collideShapeVsStaticCompound_stack, 0, 0);

    while (_collideShapeVsStaticCompound_stack.size > 0) {
        if (collector.shouldEarlyOut()) break;

        const entry = bvhStack.pop(_collideShapeVsStaticCompound_stack)!;
        const nodeOffset = entry.nodeIndex;

        if (
            !bvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                queryBounds[0],
                queryBounds[1],
                queryBounds[2],
                queryBounds[3],
                queryBounds[4],
                queryBounds[5],
            )
        ) {
            continue;
        }

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            const childStart = staticCompoundBvh.nodeChildStart(buffer, nodeOffset);
            const childCount = staticCompoundBvh.nodeChildCount(buffer, nodeOffset);

            for (let i = 0; i < childCount; i++) {
                if (collector.shouldEarlyOut()) break;

                const childIndex = childStart + i;
                const child = compound.children[childIndex];

                _collideShapeVsStaticCompound_subShapeIdBuilder.value = subShapeIdB;
                _collideShapeVsStaticCompound_subShapeIdBuilder.currentBit = subShapeIdBitsB;
                subShape.pushIndex(
                    _collideShapeVsStaticCompound_subShapeIdBuilder,
                    _collideShapeVsStaticCompound_subShapeIdBuilder,
                    childIndex,
                    compound.children.length,
                );

                vec3.set(_collideShapeVsStaticCompound_posB, posBX, posBY, posBZ);
                quat.set(_collideShapeVsStaticCompound_quatB, quatBX, quatBY, quatBZ, quatBW);
                vec3.transformQuat(
                    _collideShapeVsStaticCompound_transformedTranslation,
                    child.position,
                    _collideShapeVsStaticCompound_quatB,
                );
                vec3.add(
                    _collideShapeVsStaticCompound_worldPos,
                    _collideShapeVsStaticCompound_posB,
                    _collideShapeVsStaticCompound_transformedTranslation,
                );
                quat.multiply(_collideShapeVsStaticCompound_worldRot, _collideShapeVsStaticCompound_quatB, child.quaternion);

                const fn = collisionDispatch.collideFns.get(shapeA.type)?.get(child.shape.type);
                if (fn) {
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
                        child.shape,
                        _collideShapeVsStaticCompound_subShapeIdBuilder.value,
                        _collideShapeVsStaticCompound_subShapeIdBuilder.currentBit,
                        _collideShapeVsStaticCompound_worldPos[0],
                        _collideShapeVsStaticCompound_worldPos[1],
                        _collideShapeVsStaticCompound_worldPos[2],
                        _collideShapeVsStaticCompound_worldRot[0],
                        _collideShapeVsStaticCompound_worldRot[1],
                        _collideShapeVsStaticCompound_worldRot[2],
                        _collideShapeVsStaticCompound_worldRot[3],
                        scaleBX,
                        scaleBY,
                        scaleBZ,
                    );
                }
            }
        } else {
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);
            bvhStack.push(_collideShapeVsStaticCompound_stack, leftOffset, 0);
            bvhStack.push(_collideShapeVsStaticCompound_stack, rightOffset, 0);
        }
    }
}

/* cast shape */

const _castStaticCompoundVsShape_stack = /* @__PURE__ */ bvhStack.create();
const _castStaticCompoundVsShape_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();

const _castStaticCompoundVsShape_posA = /* @__PURE__ */ vec3.create();
const _castStaticCompoundVsShape_quatA = /* @__PURE__ */ quat.create();
const _castStaticCompoundVsShape_invQuatA = /* @__PURE__ */ quat.create();

const _castStaticCompoundVsShape_worldPos = /* @__PURE__ */ vec3.create();
const _castStaticCompoundVsShape_worldRot = /* @__PURE__ */ quat.create();
const _castStaticCompoundVsShape_transformedTranslation = /* @__PURE__ */ vec3.create();
const _castStaticCompoundVsShape_displacementA = /* @__PURE__ */ vec3.create();

const _castStaticCompoundVsShape_queryBounds = /* @__PURE__ */ box3.create();
const _castStaticCompoundVsShape_endBounds = /* @__PURE__ */ box3.create();
const _castStaticCompoundVsShape_aabbTransform = /* @__PURE__ */ mat4.create();

const _castStaticCompoundVsShape_localPosB = /* @__PURE__ */ vec3.create();
const _castStaticCompoundVsShape_localQuatB = /* @__PURE__ */ quat.create();
const _castStaticCompoundVsShape_localDispA = /* @__PURE__ */ vec3.create();

function castStaticCompoundVsShape(
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
    const compound = shapeA as StaticCompoundShape;
    const buffer = compound.bvh.buffer;

    if (buffer.length === 0) return;

    // compute shapeB's swept bounds in compound A's local space for BVH culling
    // note: we're checking which compound children could be hit during the cast
    // the compound is moving with displacement, so from compound's perspective,
    // shapeB is moving with -displacement
    vec3.set(_castStaticCompoundVsShape_posA, posAX, posAY, posAZ);
    quat.set(_castStaticCompoundVsShape_quatA, quatAX, quatAY, quatAZ, quatAW);
    quat.conjugate(_castStaticCompoundVsShape_invQuatA, _castStaticCompoundVsShape_quatA);

    // transform shapeB position to compound A's local space
    vec3.set(_castStaticCompoundVsShape_localPosB, posBX - posAX, posBY - posAY, posBZ - posAZ);
    vec3.transformQuat(
        _castStaticCompoundVsShape_localPosB,
        _castStaticCompoundVsShape_localPosB,
        _castStaticCompoundVsShape_invQuatA,
    );

    // transform shapeB rotation to compound A's local space
    quat.set(_castStaticCompoundVsShape_localQuatB, quatBX, quatBY, quatBZ, quatBW);
    quat.multiply(
        _castStaticCompoundVsShape_localQuatB,
        _castStaticCompoundVsShape_invQuatA,
        _castStaticCompoundVsShape_localQuatB,
    );

    // transform displacement to compound A's local space (negated because from compound's frame, B moves opposite)
    vec3.set(_castStaticCompoundVsShape_localDispA, -dispAX, -dispAY, -dispAZ);
    vec3.transformQuat(
        _castStaticCompoundVsShape_localDispA,
        _castStaticCompoundVsShape_localDispA,
        _castStaticCompoundVsShape_invQuatA,
    );

    // compute swept bounds of shapeB in compound A's local space:
    // transform shapeB AABB by localQuatB+localPosB to get start bounds,
    // then translate by localDispA to get end bounds, and union them.
    const queryBounds = _castStaticCompoundVsShape_queryBounds;
    const endBounds = _castStaticCompoundVsShape_endBounds;
    const localDisp = _castStaticCompoundVsShape_localDispA;
    mat4.fromRotationTranslation(
        _castStaticCompoundVsShape_aabbTransform,
        _castStaticCompoundVsShape_localQuatB,
        _castStaticCompoundVsShape_localPosB,
    );
    box3.transformMat4(queryBounds, shapeB.aabb, _castStaticCompoundVsShape_aabbTransform);
    // end bounds = start bounds translated by displacement
    endBounds[0] = queryBounds[0] + localDisp[0];
    endBounds[1] = queryBounds[1] + localDisp[1];
    endBounds[2] = queryBounds[2] + localDisp[2];
    endBounds[3] = queryBounds[3] + localDisp[0];
    endBounds[4] = queryBounds[4] + localDisp[1];
    endBounds[5] = queryBounds[5] + localDisp[2];
    box3.union(queryBounds, queryBounds, endBounds);

    bvhStack.reset(_castStaticCompoundVsShape_stack);
    bvhStack.push(_castStaticCompoundVsShape_stack, 0, 0);

    while (_castStaticCompoundVsShape_stack.size > 0) {
        if (collector.earlyOutFraction <= -Infinity) break;

        const entry = bvhStack.pop(_castStaticCompoundVsShape_stack)!;
        const nodeOffset = entry.nodeIndex;

        // BVH culling using swept bounds
        if (
            !bvh.nodeIntersectsBox(
                buffer,
                nodeOffset,
                queryBounds[0],
                queryBounds[1],
                queryBounds[2],
                queryBounds[3],
                queryBounds[4],
                queryBounds[5],
            )
        ) {
            continue;
        }

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            const childStart = staticCompoundBvh.nodeChildStart(buffer, nodeOffset);
            const childCount = staticCompoundBvh.nodeChildCount(buffer, nodeOffset);

            for (let i = 0; i < childCount; i++) {
                if (collector.earlyOutFraction <= -Infinity) break;

                const childIndex = childStart + i;
                const child = compound.children[childIndex];

                _castStaticCompoundVsShape_subShapeIdBuilder.value = subShapeIdA;
                _castStaticCompoundVsShape_subShapeIdBuilder.currentBit = subShapeIdBitsA;
                subShape.pushIndex(
                    _castStaticCompoundVsShape_subShapeIdBuilder,
                    _castStaticCompoundVsShape_subShapeIdBuilder,
                    childIndex,
                    compound.children.length,
                );

                vec3.set(_castStaticCompoundVsShape_posA, posAX, posAY, posAZ);
                quat.set(_castStaticCompoundVsShape_quatA, quatAX, quatAY, quatAZ, quatAW);
                vec3.transformQuat(
                    _castStaticCompoundVsShape_transformedTranslation,
                    child.position,
                    _castStaticCompoundVsShape_quatA,
                );
                vec3.add(
                    _castStaticCompoundVsShape_worldPos,
                    _castStaticCompoundVsShape_posA,
                    _castStaticCompoundVsShape_transformedTranslation,
                );
                quat.multiply(_castStaticCompoundVsShape_worldRot, _castStaticCompoundVsShape_quatA, child.quaternion);

                vec3.set(_castStaticCompoundVsShape_displacementA, dispAX, dispAY, dispAZ);

                const fn = collisionDispatch.castFns.get(child.shape.type)?.get(shapeB.type);
                if (fn) {
                    fn(
                        collector,
                        settings,
                        child.shape,
                        _castStaticCompoundVsShape_subShapeIdBuilder.value,
                        _castStaticCompoundVsShape_subShapeIdBuilder.currentBit,
                        _castStaticCompoundVsShape_worldPos[0],
                        _castStaticCompoundVsShape_worldPos[1],
                        _castStaticCompoundVsShape_worldPos[2],
                        _castStaticCompoundVsShape_worldRot[0],
                        _castStaticCompoundVsShape_worldRot[1],
                        _castStaticCompoundVsShape_worldRot[2],
                        _castStaticCompoundVsShape_worldRot[3],
                        scaleAX,
                        scaleAY,
                        scaleAZ,
                        dispAX,
                        dispAY,
                        dispAZ,
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
            }
        } else {
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);
            bvhStack.push(_castStaticCompoundVsShape_stack, leftOffset, 0);
            bvhStack.push(_castStaticCompoundVsShape_stack, rightOffset, 0);
        }
    }
}

/* cast shape vs static compound */

const _computeChildAABB_result = /* @__PURE__ */ box3.create();
const _computeChildAABB_mat4 = /* @__PURE__ */ mat4.create();

function calculateChildAABB(out: Box3, compound: StaticCompoundShape, childIndex: number): void {
    const child = compound.children[childIndex];
    const childAABB = child.shape.aabb;

    // transform child AABB to compound local space using rotation and translation
    const matrix = mat4.fromRotationTranslation(_computeChildAABB_mat4, child.quaternion, child.position);
    box3.transformMat4(out, childAABB, matrix);
}

const _castShapeVsStaticCompound_posA = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_quatA = /* @__PURE__ */ quat.create();
const _castShapeVsStaticCompound_scaleA = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_displacementA = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_posB = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_quatB = /* @__PURE__ */ quat.create();
const _castShapeVsStaticCompound_inverseQuaternionB = /* @__PURE__ */ quat.create();
const _castShapeVsStaticCompound_positionDifference = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_posAInB = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_quatAInB = /* @__PURE__ */ quat.create();
const _castShapeVsStaticCompound_displacementInB = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_mat4 = /* @__PURE__ */ mat4.create();
const _castShapeVsStaticCompound_sweptAABB = /* @__PURE__ */ box3.create();
const _castShapeVsStaticCompound_raycast = /* @__PURE__ */ raycast3.create();
const _castShapeVsStaticCompound_halfExtents = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_expandedBounds = /* @__PURE__ */ box3.create();
const _castShapeVsStaticCompound_childExpandedBounds = /* @__PURE__ */ box3.create();
const _castShapeVsStaticCompound_stack = /* @__PURE__ */ bvhStack.create();
const _castShapeVsStaticCompound_subShapeIdBuilder = /* @__PURE__ */ subShape.builder();
const _castShapeVsStaticCompound_worldPos = /* @__PURE__ */ vec3.create();
const _castShapeVsStaticCompound_worldRot = /* @__PURE__ */ quat.create();
const _castShapeVsStaticCompound_transformedTranslation = /* @__PURE__ */ vec3.create();

function castShapeVsStaticCompound(
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
    const compound = shapeB as StaticCompoundShape;

    const buffer = compound.bvh.buffer;

    // early out if buffer is empty (no children)
    if (buffer.length === 0) {
        return;
    }

    vec3.set(_castShapeVsStaticCompound_posA, posAX, posAY, posAZ);
    quat.set(_castShapeVsStaticCompound_quatA, quatAX, quatAY, quatAZ, quatAW);
    vec3.set(_castShapeVsStaticCompound_scaleA, scaleAX, scaleAY, scaleAZ);
    vec3.set(_castShapeVsStaticCompound_displacementA, displacementAX, displacementAY, displacementAZ);
    vec3.set(_castShapeVsStaticCompound_posB, posBX, posBY, posBZ);
    quat.set(_castShapeVsStaticCompound_quatB, quatBX, quatBY, quatBZ, quatBW);

    // transform A into B's local space
    quat.conjugate(_castShapeVsStaticCompound_inverseQuaternionB, _castShapeVsStaticCompound_quatB);

    vec3.sub(_castShapeVsStaticCompound_positionDifference, _castShapeVsStaticCompound_posA, _castShapeVsStaticCompound_posB);
    vec3.transformQuat(
        _castShapeVsStaticCompound_posAInB,
        _castShapeVsStaticCompound_positionDifference,
        _castShapeVsStaticCompound_inverseQuaternionB,
    );

    quat.multiply(
        _castShapeVsStaticCompound_quatAInB,
        _castShapeVsStaticCompound_inverseQuaternionB,
        _castShapeVsStaticCompound_quatA,
    );

    vec3.transformQuat(
        _castShapeVsStaticCompound_displacementInB,
        _castShapeVsStaticCompound_displacementA,
        _castShapeVsStaticCompound_inverseQuaternionB,
    );

    // compute base AABB of shape A at t=0 in compound local space
    const aabbMatrix = mat4.fromRotationTranslationScale(
        _castShapeVsStaticCompound_mat4,
        _castShapeVsStaticCompound_quatAInB,
        _castShapeVsStaticCompound_posAInB,
        _castShapeVsStaticCompound_scaleA,
    );
    box3.transformMat4(_castShapeVsStaticCompound_sweptAABB, shapeA.aabb, aabbMatrix);

    // compute centroid of base AABB
    const ray = _castShapeVsStaticCompound_raycast;
    ray.origin[0] = (_castShapeVsStaticCompound_sweptAABB[0] + _castShapeVsStaticCompound_sweptAABB[3]) * 0.5;
    ray.origin[1] = (_castShapeVsStaticCompound_sweptAABB[1] + _castShapeVsStaticCompound_sweptAABB[4]) * 0.5;
    ray.origin[2] = (_castShapeVsStaticCompound_sweptAABB[2] + _castShapeVsStaticCompound_sweptAABB[5]) * 0.5;

    // compute ray direction and length from displacement
    ray.length = vec3.length(_castShapeVsStaticCompound_displacementInB);
    if (ray.length > 1e-10) {
        vec3.normalize(ray.direction, _castShapeVsStaticCompound_displacementInB);
    } else {
        ray.direction[0] = 0;
        ray.direction[1] = 0;
        ray.direction[2] = 0;
    }

    // compute half-extents of the base AABB
    const halfExtents = _castShapeVsStaticCompound_halfExtents;
    halfExtents[0] = (_castShapeVsStaticCompound_sweptAABB[3] - _castShapeVsStaticCompound_sweptAABB[0]) * 0.5;
    halfExtents[1] = (_castShapeVsStaticCompound_sweptAABB[4] - _castShapeVsStaticCompound_sweptAABB[1]) * 0.5;
    halfExtents[2] = (_castShapeVsStaticCompound_sweptAABB[5] - _castShapeVsStaticCompound_sweptAABB[2]) * 0.5;

    bvhStack.reset(_castShapeVsStaticCompound_stack);
    bvhStack.push(_castShapeVsStaticCompound_stack, 0, 0); // root always visited

    const expandedBounds = _castShapeVsStaticCompound_expandedBounds;

    while (_castShapeVsStaticCompound_stack.size > 0) {
        const entry = bvhStack.pop(_castShapeVsStaticCompound_stack)!;

        // early out: if fraction to this node >= closest hit, skip it
        if (entry.distance >= collector.earlyOutFraction) {
            continue;
        }

        const nodeOffset = entry.nodeIndex;

        // note: no need to test ray intersection here - we already proved the ray
        // intersects this node's expanded bounds when we computed the distance during push.
        // if the ray didn't intersect, rayDistanceToBox3 would have returned Infinity
        // and we wouldn't have pushed this node onto the stack.

        if (bvh.nodeIsLeaf(buffer, nodeOffset)) {
            // leaf: test child shapes
            const childStart = staticCompoundBvh.nodeChildStart(buffer, nodeOffset);
            const childCount = staticCompoundBvh.nodeChildCount(buffer, nodeOffset);

            for (let i = 0; i < childCount; i++) {
                const childIndex = childStart + i;

                // compute child aabb
                calculateChildAABB(_computeChildAABB_result, compound, childIndex);
                const childBounds = _castShapeVsStaticCompound_childExpandedBounds;
                box3.copy(childBounds, _computeChildAABB_result);

                // expand by half-extents
                childBounds[0] -= halfExtents[0];
                childBounds[1] -= halfExtents[1];
                childBounds[2] -= halfExtents[2];
                childBounds[3] += halfExtents[0];
                childBounds[4] += halfExtents[1];
                childBounds[5] += halfExtents[2];

                // early out: ray x child expanded bounds
                if (!raycast3.intersectsBox3(ray, childBounds)) {
                    continue;
                }

                // early out if we've found a very close hit
                if (collector.earlyOutFraction <= 0) {
                    return;
                }

                const child = compound.children[childIndex];

                _castShapeVsStaticCompound_subShapeIdBuilder.value = subShapeIdB;
                _castShapeVsStaticCompound_subShapeIdBuilder.currentBit = subShapeIdBitsB;
                subShape.pushIndex(
                    _castShapeVsStaticCompound_subShapeIdBuilder,
                    _castShapeVsStaticCompound_subShapeIdBuilder,
                    childIndex,
                    compound.children.length,
                );

                vec3.transformQuat(
                    _castShapeVsStaticCompound_transformedTranslation,
                    child.position,
                    _castShapeVsStaticCompound_quatB,
                );
                vec3.add(
                    _castShapeVsStaticCompound_worldPos,
                    _castShapeVsStaticCompound_posB,
                    _castShapeVsStaticCompound_transformedTranslation,
                );
                quat.multiply(_castShapeVsStaticCompound_worldRot, _castShapeVsStaticCompound_quatB, child.quaternion);

                const fn = collisionDispatch.castFns.get(shapeA.type)?.get(child.shape.type);

                if (fn) {
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
                        displacementAX,
                        displacementAY,
                        displacementAZ,
                        child.shape,
                        _castShapeVsStaticCompound_subShapeIdBuilder.value,
                        _castShapeVsStaticCompound_subShapeIdBuilder.currentBit,
                        _castShapeVsStaticCompound_worldPos[0],
                        _castShapeVsStaticCompound_worldPos[1],
                        _castShapeVsStaticCompound_worldPos[2],
                        _castShapeVsStaticCompound_worldRot[0],
                        _castShapeVsStaticCompound_worldRot[1],
                        _castShapeVsStaticCompound_worldRot[2],
                        _castShapeVsStaticCompound_worldRot[3],
                        scaleBX,
                        scaleBY,
                        scaleBZ,
                    );
                }
            }
        } else {
            // internal node: compute distances to both children and sort by distance
            // ray from swept AABB center along displacement
            const leftOffset = bvh.nodeLeft(nodeOffset);
            const rightOffset = bvh.nodeRight(buffer, nodeOffset);

            // expand bounds by half-extents (same as child test)
            expandedBounds[0] = buffer[leftOffset + bvh.NODE_MIN_X] - halfExtents[0];
            expandedBounds[1] = buffer[leftOffset + bvh.NODE_MIN_Y] - halfExtents[1];
            expandedBounds[2] = buffer[leftOffset + bvh.NODE_MIN_Z] - halfExtents[2];
            expandedBounds[3] = buffer[leftOffset + bvh.NODE_MAX_X] + halfExtents[0];
            expandedBounds[4] = buffer[leftOffset + bvh.NODE_MAX_Y] + halfExtents[1];
            expandedBounds[5] = buffer[leftOffset + bvh.NODE_MAX_Z] + halfExtents[2];
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

            expandedBounds[0] = buffer[rightOffset + bvh.NODE_MIN_X] - halfExtents[0];
            expandedBounds[1] = buffer[rightOffset + bvh.NODE_MIN_Y] - halfExtents[1];
            expandedBounds[2] = buffer[rightOffset + bvh.NODE_MIN_Z] - halfExtents[2];
            expandedBounds[3] = buffer[rightOffset + bvh.NODE_MAX_X] + halfExtents[0];
            expandedBounds[4] = buffer[rightOffset + bvh.NODE_MAX_Y] + halfExtents[1];
            expandedBounds[5] = buffer[rightOffset + bvh.NODE_MAX_Z] + halfExtents[2];
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
                    bvhStack.push(_castShapeVsStaticCompound_stack, rightOffset, rightDist);
                }
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_castShapeVsStaticCompound_stack, leftOffset, leftDist);
                }
            } else {
                // right is closer - push left first
                if (leftDist < collector.earlyOutFraction) {
                    bvhStack.push(_castShapeVsStaticCompound_stack, leftOffset, leftDist);
                }
                if (rightDist < collector.earlyOutFraction) {
                    bvhStack.push(_castShapeVsStaticCompound_stack, rightOffset, rightDist);
                }
            }
        }
    }
}
