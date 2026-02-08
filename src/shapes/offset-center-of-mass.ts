import { type Box3, box3, type Raycast3, type Vec3, vec3 } from 'mathcat';
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
    type Shape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
    shapeDefs,
} from './shapes';

/** settings for creating an offset center of mass shape */
export type OffsetCenterOfMassShapeSettings = {
    shape: Shape;
    offset: Vec3;
};

/**
 * offset center of mass shape - shifts the center of mass of a child shape
 * without affecting its collision geometry.
 *
 * use case: stabilizing unstable objects (e.g., cars, boats) by lowering their center of mass.
 *
 * in crashcat's coordinate system:
 * - collision geometry stays at origin
 * - centerOfMass property is offset by the specified amount
 * - only affects physics calculations (inertia, stability), not collision detection
 */
export type OffsetCenterOfMassShape = {
    type: ShapeType.OFFSET_CENTER_OF_MASS;
    shape: Shape;
    offset: Vec3;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

/** create an offset center of mass shape */
export function create(o: OffsetCenterOfMassShapeSettings): OffsetCenterOfMassShape {
    const shape: OffsetCenterOfMassShape = {
        type: ShapeType.OFFSET_CENTER_OF_MASS,
        shape: o.shape,
        offset: vec3.clone(o.offset),
        aabb: box3.create(),
        centerOfMass: vec3.create(),
        volume: 0,
    };

    update(shape);

    return shape;
}

/** update an offset center of mass shape after its properties have changed */
export function update(shape: OffsetCenterOfMassShape): void {
    computeOffsetCenterOfMassLocalBounds(shape.aabb, shape);
    computeOffsetCenterOfMassCenterOfMass(shape.centerOfMass, shape);
    shape.volume = shape.shape.volume;
}

/** compute local bounds for offset center of mass shape */
function computeOffsetCenterOfMassLocalBounds(out: Box3, shape: OffsetCenterOfMassShape): void {
    // geometry is at local origin - bounds are unchanged from inner shape
    // only the COM is offset, not the collision geometry
    box3.copy(out, shape.shape.aabb);
}

/** compute center of mass for offset center of mass shape */
function computeOffsetCenterOfMassCenterOfMass(out: Vec3, shape: OffsetCenterOfMassShape): void {
    // add offset to inner shape's center of mass
    vec3.add(out, shape.shape.centerOfMass, shape.offset);
}

const _childMassProperties = /* @__PURE__ */ massProperties.create();
const _supportingFace_scaledOffset = /* @__PURE__ */ vec3.create();
const _supportingFace_transformedOffset = /* @__PURE__ */ vec3.create();

export const def = /* @__PURE__ */ (() =>
    defineShape<OffsetCenterOfMassShape>({
        type: ShapeType.OFFSET_CENTER_OF_MASS,
        category: ShapeCategory.DECORATOR,

        computeMassProperties(out: MassProperties, shape: OffsetCenterOfMassShape): void {
            // get inner shape mass properties
            computeMassProperties(_childMassProperties, shape.shape);

            // translate inertia by offset using parallel axis theorem
            // this shifts the inertia tensor from the inner shape's COM to the new COM
            massProperties.translate(out, _childMassProperties, shape.offset);
        },

        getSurfaceNormal(ioResult: SurfaceNormalResult, shape: OffsetCenterOfMassShape, subShapeId: number): void {
            // in crashcat, collision geometry is at origin, not COM-centered
            // so we just pass through to the inner shape
            shapeDefs[shape.shape.type].getSurfaceNormal(ioResult, shape.shape, subShapeId);
        },

        getSupportingFace(
            ioResult: SupportingFaceResult,
            direction: Vec3,
            shape: OffsetCenterOfMassShape,
            subShapeId: number,
        ): void {
            // compute -scale * offset
            vec3.multiply(_supportingFace_scaledOffset, ioResult.scale, shape.offset);
            vec3.negate(_supportingFace_scaledOffset, _supportingFace_scaledOffset);

            // transform by rotation
            vec3.transformQuat(_supportingFace_transformedOffset, _supportingFace_scaledOffset, ioResult.quaternion);

            // pre-translate the position
            vec3.add(ioResult.position, ioResult.position, _supportingFace_transformedOffset);

            // delegate to inner shape
            shapeDefs[shape.shape.type].getSupportingFace(ioResult, direction, shape.shape, subShapeId);
        },

        getInnerRadius(shape: OffsetCenterOfMassShape): number {
            // inner radius doesn't change with COM offset
            return getShapeInnerRadius(shape.shape);
        },

        castRay: castRayVsOffsetCenterOfMass,
        collidePoint: collidePointVsOffsetCenterOfMass,

        register: () => {
            // register collision dispatch for all shape types
            for (const shapeDef of Object.values(shapeDefs)) {
                setCollideShapeFn(ShapeType.OFFSET_CENTER_OF_MASS, shapeDef.type, collideOffsetCenterOfMassVsShape);
                setCollideShapeFn(shapeDef.type, ShapeType.OFFSET_CENTER_OF_MASS, collideShapeVsOffsetCenterOfMass);

                setCastShapeFn(ShapeType.OFFSET_CENTER_OF_MASS, shapeDef.type, castOffsetCenterOfMassVsShape);
                setCastShapeFn(shapeDef.type, ShapeType.OFFSET_CENTER_OF_MASS, castShapeVsOffsetCenterOfMass);
            }
        },
    }))();

// in crashcat, collision geometry is at origin, not COM-centered
// so ray casting is a simple passthrough - no transformation needed
function castRayVsOffsetCenterOfMass(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: OffsetCenterOfMassShape,
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
    // geometry doesn't move when COM changes - just pass through
    const innerShapeDef = shapeDefs[shape.shape.type];
    innerShapeDef.castRay(
        collector,
        settings,
        ray,
        shape.shape,
        subShapeId,
        subShapeIdBits,
        posX,
        posY,
        posZ,
        quatX,
        quatY,
        quatZ,
        quatW,
        scaleX,
        scaleY,
        scaleZ,
    );
}

// in crashcat, collision geometry is at origin, not COM-centered
// so point collision is a simple passthrough - no transformation needed
function collidePointVsOffsetCenterOfMass(
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shape: OffsetCenterOfMassShape,
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
    // geometry doesn't move when COM changes - just pass through
    const innerShapeDef = shapeDefs[shape.shape.type];
    innerShapeDef.collidePoint(
        collector,
        settings,
        pointX,
        pointY,
        pointZ,
        shape.shape,
        subShapeId,
        subShapeIdBits,
        posX,
        posY,
        posZ,
        quatX,
        quatY,
        quatZ,
        quatW,
        scaleX,
        scaleY,
        scaleZ,
    );
}

/* collide shape vs shape */

// in crashcat, position represents shape origin (where geometry is), not COM
// so collision dispatch is a simple passthrough - no transformation needed
function collideOffsetCenterOfMassVsShape(
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
    const offsetShape = shapeA as OffsetCenterOfMassShape;

    // dispatch to inner shape vs shapeB
    const fn = collisionDispatch.collideFns.get(offsetShape.shape.type)?.get(shapeB.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        offsetShape.shape,
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

function collideShapeVsOffsetCenterOfMass(
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
    const offsetShape = shapeB as OffsetCenterOfMassShape;

    // dispatch to shapeA vs inner shape
    const fn = collisionDispatch.collideFns.get(shapeA.type)?.get(offsetShape.shape.type);

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
        offsetShape.shape,
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

/* cast shape vs shape */

// in crashcat, position represents shape origin (where geometry is), not COM
// so cast dispatch is a simple passthrough - no transformation needed
function castOffsetCenterOfMassVsShape(
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
    const offsetShape = shapeA as OffsetCenterOfMassShape;

    // cast inner shape vs shapeB
    const fn = collisionDispatch.castFns.get(offsetShape.shape.type)?.get(shapeB.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        offsetShape.shape,
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

function castShapeVsOffsetCenterOfMass(
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
    const offsetShape = shapeB as OffsetCenterOfMassShape;

    // cast shapeA vs inner shape
    const fn = collisionDispatch.castFns.get(shapeA.type)?.get(offsetShape.shape.type);

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
        offsetShape.shape,
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
