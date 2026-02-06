import { type Box3, box3, mat3, mat4, type Quat, quat, type Raycast3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as massProperties from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape';
import type { CastShapeCollector, CastShapeSettings } from '../collision/cast-shape-vs-shape';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape';
import { assert } from '../utils/assert';
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

export type CompoundShapeChild = {
    position: Vec3;
    quaternion: Quat;
    shape: Shape;
};

export type CompoundShape = {
    type: ShapeType.COMPOUND;
    children: CompoundShapeChild[];
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

export type CompoundShapeSettings = {
    children: CompoundShapeChild[];
};

export function create(o: CompoundShapeSettings): CompoundShape {
    assert(o.children.length > 0, 'CompoundShape must have at least one child');

    const shape: CompoundShape = {
        type: ShapeType.COMPOUND,
        children: o.children,
        aabb: box3.create(),
        centerOfMass: vec3.create(),
        volume: 0,
    };

    update(shape);

    return shape;
}

function computeCompoundVolume(shape: CompoundShape): number {
    // sum volumes of all children
    let totalVolume = 0;
    for (const child of shape.children) {
        totalVolume += child.shape.volume;
    }
    return totalVolume;
}

const _computeCompoundLocalBounds_transformed = vec3.create();

function computeCompoundLocalBounds(out: Box3, shape: CompoundShape): void {
    // start with empty bounds
    out[0][0] = Infinity;
    out[0][1] = Infinity;
    out[0][2] = Infinity;
    out[1][0] = -Infinity;
    out[1][1] = -Infinity;
    out[1][2] = -Infinity;

    for (const child of shape.children) {
        // Transform child AABB corners by child position and rotation
        const childAABB = child.shape.aabb;

        // Transform all 8 corners of the child AABB
        for (let x = 0; x < 2; x++) {
            for (let y = 0; y < 2; y++) {
                for (let z = 0; z < 2; z++) {
                    _computeCompoundLocalBounds_transformed[0] = childAABB[x][0];
                    _computeCompoundLocalBounds_transformed[1] = childAABB[y][1];
                    _computeCompoundLocalBounds_transformed[2] = childAABB[z][2];

                    // Rotate and translate corner
                    vec3.transformQuat(
                        _computeCompoundLocalBounds_transformed,
                        _computeCompoundLocalBounds_transformed,
                        child.quaternion,
                    );
                    vec3.add(_computeCompoundLocalBounds_transformed, _computeCompoundLocalBounds_transformed, child.position);

                    // Expand compound AABB
                    box3.expandByPoint(out, out, _computeCompoundLocalBounds_transformed);
                }
            }
        }
    }
}

const _computeCompoundCenterOfMass_childCOM = vec3.create();
const _computeCompoundCenterOfMass_worldChildCOM = vec3.create();

function computeCompoundCenterOfMass(out: Vec3, shape: CompoundShape): void {
    // Compute weighted center of mass based on child mass and transforms
    vec3.zero(out);
    let totalMass = 0;

    const childMass = _computeCompoundMassProperties_childMass;

    for (const child of shape.children) {
        // Get child mass properties to weight COM calculation
        computeMassProperties(childMass, child.shape);

        // Transform child COM to compound space:
        // worldChildCOM = child.position + rotate(child.quaternion, child.shape.centerOfMass)
        vec3.transformQuat(_computeCompoundCenterOfMass_childCOM, child.shape.centerOfMass, child.quaternion);
        vec3.add(_computeCompoundCenterOfMass_worldChildCOM, child.position, _computeCompoundCenterOfMass_childCOM);

        // Accumulate weighted position
        out[0] += _computeCompoundCenterOfMass_worldChildCOM[0] * childMass.mass;
        out[1] += _computeCompoundCenterOfMass_worldChildCOM[1] * childMass.mass;
        out[2] += _computeCompoundCenterOfMass_worldChildCOM[2] * childMass.mass;

        totalMass += childMass.mass;
    }

    // Divide by total mass to get weighted average
    if (totalMass > 0) {
        out[0] /= totalMass;
        out[1] /= totalMass;
        out[2] /= totalMass;
    }
}

/** updates a compound shape after it's properties have changed or children have changed */
export function update(shape: CompoundShape): void {
    computeCompoundLocalBounds(shape.aabb, shape);
    computeCompoundCenterOfMass(shape.centerOfMass, shape);
    shape.volume = computeCompoundVolume(shape);
}

const _computeCompoundMassProperties_childMass = massProperties.create();
const _computeCompoundMassProperties_childCOM = vec3.create();
const _computeCompoundMassProperties_childCOMRelative = vec3.create();
const _computeCompoundMassProperties_rotatedInertia = massProperties.create();
const _computeCompoundMassProperties_childRotMat3 = mat3.create();
const _computeCompoundMassProperties_childRotMat4 = mat4.create();

const _getSurfaceNormal_invRotation = quat.create();
const _getSurfaceNormal_forwardRotation = quat.create();

const _subShapeIdPopResult = subShape.popResult();

const _getSupportingFace_childPos = vec3.create();
const _getSupportingFace_invChildRot = quat.create();
const _getSupportingFace_localDirection = vec3.create();

export const def = defineShape<CompoundShape>({
    type: ShapeType.COMPOUND,
    category: ShapeCategory.COMPOSITE,
    computeMassProperties(out: MassProperties, shape: CompoundShape): void {
        out.mass = 0;

        // initialize inertia to zero
        for (let i = 0; i < 16; i++) {
            out.inertia[i] = 0;
        }

        if (shape.children.length === 0) {
            out.inertia[15] = 1.0;
            return;
        }

        const childMass = _computeCompoundMassProperties_childMass;
        const rotatedInertia = _computeCompoundMassProperties_rotatedInertia;
        const childRotMat3 = _computeCompoundMassProperties_childRotMat3;
        const childRotMat4 = _computeCompoundMassProperties_childRotMat4;

        // accumulate mass properties from all children with proper transforms
        for (const child of shape.children) {
            // get child mass properties
            computeMassProperties(childMass, child.shape);

            // calculate child COM position relative to compound COM:
            vec3.transformQuat(_computeCompoundMassProperties_childCOM, child.shape.centerOfMass, child.quaternion);
            vec3.add(_computeCompoundMassProperties_childCOM, _computeCompoundMassProperties_childCOM, child.position);
            vec3.subtract(
                _computeCompoundMassProperties_childCOMRelative,
                _computeCompoundMassProperties_childCOM,
                shape.centerOfMass,
            );

            // accumulate mass
            out.mass += childMass.mass;

            // transform child inertia to compound space:
            // 1. convert child rotation to mat4
            mat3.fromQuat(childRotMat3, child.quaternion);
            mat4.identity(childRotMat4);
            for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                    childRotMat4[i + j * 4] = childRotMat3[i + j * 3];
                }
            }

            // 2. rotate inertia by child rotation
            massProperties.rotate(rotatedInertia, childMass, childRotMat4);

            // 3. apply parallel axis theorem to translate inertia
            massProperties.translate(rotatedInertia, rotatedInertia, _computeCompoundMassProperties_childCOMRelative);

            // add to compound inertia
            for (let i = 0; i < 15; i++) {
                out.inertia[i] += rotatedInertia.inertia[i];
            }
        }

        out.inertia[15] = 1.0;
    },
    getSurfaceNormal(ioResult: SurfaceNormalResult, shape: CompoundShape, subShapeId: number): void {
        subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);

        const childIndex = _subShapeIdPopResult.value;
        const remainder = _subShapeIdPopResult.remainder;

        if (childIndex < 0 || childIndex >= shape.children.length) {
            assert(false, 'Invalid SubShapeID for CompoundShape');
            return;
        }

        const child = shape.children[childIndex];

        // accumulate transform: position = position - child.position (inverse transform)
        vec3.subtract(ioResult.position, ioResult.position, child.position);

        // accumulate rotation: quaternion = conjugate(child.quaternion) * quaternion (inverse transform)
        const invRotation = quat.conjugate(_getSurfaceNormal_invRotation, child.quaternion);
        quat.multiply(ioResult.quaternion, invRotation, ioResult.quaternion);

        // transform position to local space
        vec3.transformQuat(ioResult.position, ioResult.position, invRotation);

        // get normal from child shape in its local space
        shapeDefs[child.shape.type].getSurfaceNormal(ioResult, child.shape, remainder);

        // transform normal back to compound space using accumulated rotation.
        // the accumulated quaternion contains the inverse transforms, so we need to apply its conjugate
        // to get the forward transformation for the normal.
        const forwardAccumulatedRotation = quat.conjugate(_getSurfaceNormal_forwardRotation, ioResult.quaternion);
        vec3.transformQuat(ioResult.normal, ioResult.normal, forwardAccumulatedRotation);
    },
    getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: CompoundShape, subShapeId: number): void {
        subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);
        const childIndex = _subShapeIdPopResult.value;
        const remainder = _subShapeIdPopResult.remainder;

        if (childIndex < 0 || childIndex >= shape.children.length) {
            assert(false, 'Invalid SubShapeID for CompoundShape');
            return;
        }

        const child = shape.children[childIndex];

        // accumulate transform: position += rotation * child.position
        vec3.transformQuat(_getSupportingFace_childPos, child.position, ioResult.quaternion);
        vec3.add(ioResult.position, ioResult.position, _getSupportingFace_childPos);

        // accumulate rotation: rotation = rotation * child.quaternion
        quat.multiply(ioResult.quaternion, ioResult.quaternion, child.quaternion);

        // transform direction to child local space
        quat.conjugate(_getSupportingFace_invChildRot, child.quaternion);
        vec3.transformQuat(_getSupportingFace_localDirection, direction, _getSupportingFace_invChildRot);

        shapeDefs[child.shape.type].getSupportingFace(ioResult, _getSupportingFace_localDirection, child.shape, remainder);
    },
    getInnerRadius(shape: CompoundShape): number {
        let innerRadius = Number.MAX_VALUE;
        for (const child of shape.children) {
            innerRadius = Math.min(innerRadius, getShapeInnerRadius(child.shape));
        }
        return innerRadius;
    },
    getLeafShape(out, shape, subShapeId): void {
        // navigate to child shape
        if (subShape.isEmpty(subShapeId)) {
            out.shape = null;
            out.remainder = subShape.EMPTY_SUB_SHAPE_ID;
            return;
        }

        subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);
        const childShape = shape.children[_subShapeIdPopResult.value].shape;

        // get leaf shape from child
        const childShapeDef = shapeDefs[childShape.type];
        childShapeDef.getLeafShape(out, childShape, _subShapeIdPopResult.remainder);
    },
    getSubShapeTransformedShape(out, shape, subShapeId): void {
        // navigate to child shape
        if (subShape.isEmpty(subShapeId)) {
            out.shape = null;
            out.remainder = subShape.EMPTY_SUB_SHAPE_ID;
            return;
        }

        subShape.popIndex(_subShapeIdPopResult, subShapeId, shape.children.length);
        const child = shape.children[_subShapeIdPopResult.value];

        // apply child's transform: pos = pos + rotate(rot, childPos), rot = rot * childRot
        const rotatedChildPos = vec3.create();
        vec3.transformQuat(rotatedChildPos, child.position, out.rotation);
        vec3.add(out.position, out.position, rotatedChildPos);
        quat.multiply(out.rotation, out.rotation, child.quaternion);

        // recursively get transformed shape from child
        const childShapeDef = shapeDefs[child.shape.type];
        childShapeDef.getSubShapeTransformedShape(out, child.shape, _subShapeIdPopResult.remainder);
    },
    castRay: castRayVsCompound,
    collidePoint: collidePointVsCompound,
    register: () => {
        for (const shapeDef of Object.values(shapeDefs)) {
            setCollideShapeFn(ShapeType.COMPOUND, shapeDef.type, collideCompoundVsShape);
            setCollideShapeFn(shapeDef.type, ShapeType.COMPOUND, collideShapeVsCompound);

            setCastShapeFn(ShapeType.COMPOUND, shapeDef.type, castCompoundVsShape);
            setCastShapeFn(shapeDef.type, ShapeType.COMPOUND, castShapeVsCompound);
        }
    },
});

/* cast ray */

const _castRayVsCompound_pos = vec3.create();
const _castRayVsCompound_quat = quat.create();
const _castRayVsCompound_transformedTranslation = vec3.create();
const _castRayVsCompound_worldPos = vec3.create();
const _castRayVsCompound_worldRot = quat.create();
const _castRayVsCompound_subShapeIdBuilder = subShape.builder();

function castRayVsCompound(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: CompoundShape,
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
    vec3.set(_castRayVsCompound_pos, posX, posY, posZ);
    quat.set(_castRayVsCompound_quat, quatX, quatY, quatZ, quatW);

    for (let childIndex = 0; childIndex < shape.children.length; childIndex++) {
        if (collector.earlyOutFraction <= 0) {
            break;
        }

        const child = shape.children[childIndex];

        _castRayVsCompound_subShapeIdBuilder.value = subShapeId;
        _castRayVsCompound_subShapeIdBuilder.currentBit = subShapeIdBits;
        subShape.pushIndex(
            _castRayVsCompound_subShapeIdBuilder,
            _castRayVsCompound_subShapeIdBuilder,
            childIndex,
            shape.children.length,
        );

        // accumulate transform
        vec3.transformQuat(_castRayVsCompound_transformedTranslation, child.position, _castRayVsCompound_quat);
        vec3.add(_castRayVsCompound_worldPos, _castRayVsCompound_pos, _castRayVsCompound_transformedTranslation);
        quat.multiply(_castRayVsCompound_worldRot, _castRayVsCompound_quat, child.quaternion);

        const childShapeDef = shapeDefs[child.shape.type];
        childShapeDef.castRay(
            collector,
            settings,
            ray,
            child.shape,
            _castRayVsCompound_subShapeIdBuilder.value,
            _castRayVsCompound_subShapeIdBuilder.currentBit,
            _castRayVsCompound_worldPos[0],
            _castRayVsCompound_worldPos[1],
            _castRayVsCompound_worldPos[2],
            _castRayVsCompound_worldRot[0],
            _castRayVsCompound_worldRot[1],
            _castRayVsCompound_worldRot[2],
            _castRayVsCompound_worldRot[3],
            scaleX,
            scaleY,
            scaleZ,
        );
    }
}

/* collide point */

const _collidePointVsCompound_posB = vec3.create();
const _collidePointVsCompound_quatB = quat.create();
const _collidePointVsCompound_transformedTranslation = vec3.create();
const _collidePointVsCompound_worldPos = vec3.create();
const _collidePointVsCompound_worldRot = quat.create();
const _collidePointVsCompound_subShapeIdBuilder = subShape.builder();

function collidePointVsCompound(
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: CompoundShape,
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
    for (let childIndex = 0; childIndex < shapeB.children.length; childIndex++) {
        if (collector.shouldEarlyOut()) {
            break;
        }

        const child = shapeB.children[childIndex];

        _collidePointVsCompound_subShapeIdBuilder.value = subShapeIdB;
        _collidePointVsCompound_subShapeIdBuilder.currentBit = subShapeIdBitsB;
        subShape.pushIndex(
            _collidePointVsCompound_subShapeIdBuilder,
            _collidePointVsCompound_subShapeIdBuilder,
            childIndex,
            shapeB.children.length,
        );

        // accumulate transform
        vec3.set(_collidePointVsCompound_posB, posBX, posBY, posBZ);
        quat.set(_collidePointVsCompound_quatB, quatBX, quatBY, quatBZ, quatBW);
        vec3.transformQuat(_collidePointVsCompound_transformedTranslation, child.position, _collidePointVsCompound_quatB);
        vec3.add(_collidePointVsCompound_worldPos, _collidePointVsCompound_posB, _collidePointVsCompound_transformedTranslation);
        quat.multiply(_collidePointVsCompound_worldRot, _collidePointVsCompound_quatB, child.quaternion);

        const childShapeDef = shapeDefs[child.shape.type];
        childShapeDef.collidePoint(
            collector,
            settings,
            pointX,
            pointY,
            pointZ,
            child.shape,
            _collidePointVsCompound_subShapeIdBuilder.value,
            _collidePointVsCompound_subShapeIdBuilder.currentBit,
            _collidePointVsCompound_worldPos[0],
            _collidePointVsCompound_worldPos[1],
            _collidePointVsCompound_worldPos[2],
            _collidePointVsCompound_worldRot[0],
            _collidePointVsCompound_worldRot[1],
            _collidePointVsCompound_worldRot[2],
            _collidePointVsCompound_worldRot[3],
            scaleBX,
            scaleBY,
            scaleBZ,
        );
    }
}

/* collide shape */

const _transformedTranslation = vec3.create();

function collideCompoundVsShape(
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
    const compound = shapeA as CompoundShape;

    for (let childIndex = 0; childIndex < compound.children.length; childIndex++) {
        if (collector.shouldEarlyOut()) {
            break;
        }

        const child = compound.children[childIndex];

        _subShapeIdBuilder.value = subShapeIdA;
        _subShapeIdBuilder.currentBit = subShapeIdBitsA;
        subShape.pushIndex(_subShapeIdBuilder, _subShapeIdBuilder, childIndex, compound.children.length);

        // accumulate transform
        vec3.set(_posA, posAX, posAY, posAZ);
        quat.set(_quatA, quatAX, quatAY, quatAZ, quatAW);
        vec3.transformQuat(_transformedTranslation, child.position, _quatA);
        vec3.add(_worldPos, _posA, _transformedTranslation);
        quat.multiply(_worldRot, _quatA, child.quaternion);

        const fn = collisionDispatch.collideFns.get(child.shape.type)?.get(shapeB.type);
        if (fn) {
            fn(
                collector,
                settings,
                child.shape,
                _subShapeIdBuilder.value,
                _subShapeIdBuilder.currentBit,
                _worldPos[0],
                _worldPos[1],
                _worldPos[2],
                _worldRot[0],
                _worldRot[1],
                _worldRot[2],
                _worldRot[3],
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
}

function collideShapeVsCompound(
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
    const compound = shapeB as CompoundShape;

    for (let childIndex = 0; childIndex < compound.children.length; childIndex++) {
        if (collector.shouldEarlyOut()) {
            break;
        }

        const child = compound.children[childIndex];

        _subShapeIdBuilder.value = subShapeIdB;
        _subShapeIdBuilder.currentBit = subShapeIdBitsB;
        subShape.pushIndex(_subShapeIdBuilder, _subShapeIdBuilder, childIndex, compound.children.length);

        // accumulate transform
        vec3.set(_posB, posBX, posBY, posBZ);
        quat.set(_quatB, quatBX, quatBY, quatBZ, quatBW);
        vec3.transformQuat(_transformedTranslation, child.position, _quatB);
        vec3.add(_worldPos, _posB, _transformedTranslation);
        quat.multiply(_worldRot, _quatB, child.quaternion);

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
                _subShapeIdBuilder.value,
                _subShapeIdBuilder.currentBit,
                _worldPos[0],
                _worldPos[1],
                _worldPos[2],
                _worldRot[0],
                _worldRot[1],
                _worldRot[2],
                _worldRot[3],
                scaleBX,
                scaleBY,
                scaleBZ,
            );
        }
    }
}

/* cast shape */

const _subShapeIdBuilder = subShape.builder();
const _worldPos = vec3.create();
const _worldRot = quat.create();
const _castDecorated_temp = vec3.create();
const _transformDisplacementA = vec3.create();

const _posA = vec3.create();
const _quatA = quat.create();
const _displacementA = vec3.create();
const _posB = vec3.create();
const _quatB = quat.create();

function castCompoundVsShape(
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
    const compound = shapeA as CompoundShape;

    for (let childIndex = 0; childIndex < compound.children.length; childIndex++) {
        if (collector.earlyOutFraction <= -Infinity) {
            break;
        }

        const child = compound.children[childIndex];

        _subShapeIdBuilder.value = subShapeIdA;
        _subShapeIdBuilder.currentBit = subShapeIdBitsA;
        subShape.pushIndex(_subShapeIdBuilder, _subShapeIdBuilder, childIndex, compound.children.length);

        // accumulate transform
        vec3.set(_posA, posAX, posAY, posAZ);
        quat.set(_quatA, quatAX, quatAY, quatAZ, quatAW);
        vec3.transformQuat(_castDecorated_temp, child.position, _quatA);
        vec3.add(_worldPos, _posA, _castDecorated_temp);
        quat.multiply(_worldRot, _quatA, child.quaternion);

        // transform displacement - rotate by accumulated quaternion
        vec3.set(_displacementA, dispAX, dispAY, dispAZ);
        vec3.transformQuat(_transformDisplacementA, _displacementA, _quatA);

        const fn = collisionDispatch.castFns.get(child.shape.type)?.get(shapeB.type);
        if (fn) {
            fn(
                collector,
                settings,
                child.shape,
                _subShapeIdBuilder.value,
                _subShapeIdBuilder.currentBit,
                _worldPos[0],
                _worldPos[1],
                _worldPos[2],
                _worldRot[0],
                _worldRot[1],
                _worldRot[2],
                _worldRot[3],
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
}

function castShapeVsCompound(
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
    const compound = shapeB as CompoundShape;

    for (let childIndex = 0; childIndex < compound.children.length; childIndex++) {
        if (collector.earlyOutFraction <= -Infinity) {
            break;
        }

        const child = compound.children[childIndex];

        _subShapeIdBuilder.value = subShapeIdB;
        _subShapeIdBuilder.currentBit = subShapeIdBitsB;
        subShape.pushIndex(_subShapeIdBuilder, _subShapeIdBuilder, childIndex, compound.children.length);

        // accumulate transform
        vec3.set(_posB, posBX, posBY, posBZ);
        quat.set(_quatB, quatBX, quatBY, quatBZ, quatBW);
        vec3.transformQuat(_castDecorated_temp, child.position, _quatB);
        vec3.add(_worldPos, _posB, _castDecorated_temp);
        quat.multiply(_worldRot, _quatB, child.quaternion);

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
                dispAX,
                dispAY,
                dispAZ,
                child.shape,
                _subShapeIdBuilder.value,
                _subShapeIdBuilder.currentBit,
                _worldPos[0],
                _worldPos[1],
                _worldPos[2],
                _worldRot[0],
                _worldRot[1],
                _worldRot[2],
                _worldRot[3],
                scaleBX,
                scaleBY,
                scaleBZ,
            );
        }
    }
}
