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
    shapeDefs,
    type Shape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
} from './shapes';

/**
 * settings for creating a scaled shape.
 *
 * note that some shapes only support uniform scaling:
 * - sphere
 */
export type ScaledShapeSettings = {
    shape: Shape;
    scale: Vec3;
};

/** a scaled shape */
export type ScaledShape = {
    type: ShapeType.SCALED;
    shape: Shape;
    scale: Vec3;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

/** create a scaled shape */
export function create(o: ScaledShapeSettings): ScaledShape {
    const shape: ScaledShape = {
        type: ShapeType.SCALED,
        shape: o.shape,
        scale: o.scale,
        aabb: box3.create(),
        centerOfMass: vec3.create(),
        volume: 0,
    };
    update(shape);
    return shape;
}

function computeScaledVolume(shape: ScaledShape): number {
    const innerVolume = shape.shape.volume;
    const scaleX = Math.abs(shape.scale[0]);
    const scaleY = Math.abs(shape.scale[1]);
    const scaleZ = Math.abs(shape.scale[2]);
    return innerVolume * scaleX * scaleY * scaleZ;
}

function computeScaledLocalBounds(out: Box3, shape: ScaledShape): void {
    const innerAABB = shape.shape.aabb;

    const scaleX = Math.abs(shape.scale[0]);
    const scaleY = Math.abs(shape.scale[1]);
    const scaleZ = Math.abs(shape.scale[2]);

    out[0][0] = innerAABB[0][0] * scaleX;
    out[0][1] = innerAABB[0][1] * scaleY;
    out[0][2] = innerAABB[0][2] * scaleZ;

    out[1][0] = innerAABB[1][0] * scaleX;
    out[1][1] = innerAABB[1][1] * scaleY;
    out[1][2] = innerAABB[1][2] * scaleZ;
}

function computeScaledCenterOfMass(out: Vec3, shape: ScaledShape): void {
    out[0] = shape.centerOfMass[0] * shape.scale[0];
    out[1] = shape.centerOfMass[1] * shape.scale[1];
    out[2] = shape.centerOfMass[2] * shape.scale[2];
}

/** updates a scaled shape after it's properties have changed */
export function update(shape: ScaledShape): void {
    computeScaledLocalBounds(shape.aabb, shape);
    computeScaledCenterOfMass(shape.centerOfMass, shape);
    shape.volume = computeScaledVolume(shape);
}

const _childMassProperties = /* @__PURE__ */ massProperties.create();

export const def = /* @__PURE__ */ (() =>
    defineShape<ScaledShape>({
        type: ShapeType.SCALED,
        category: ShapeCategory.DECORATOR,
        computeMassProperties(out: MassProperties, shape: ScaledShape): void {
            computeMassProperties(_childMassProperties, shape.shape);
            massProperties.scale(out, _childMassProperties, shape.scale);
        },
        getSurfaceNormal(ioResult: SurfaceNormalResult, shape: ScaledShape, subShapeId: number): void {
            // accumulate scale
            vec3.multiply(ioResult.scale, ioResult.scale, shape.scale);

            // transform surface position to local space: divide by scale (inverse transform)
            vec3.divide(ioResult.position, ioResult.position, shape.scale);

            // get normal from inner shape
            shapeDefs[shape.shape.type].getSurfaceNormal(ioResult, shape.shape, subShapeId);

            // transform normal: divide by scale and renormalize
            // this handles the (M^-1)^T transformation for plane normals
            vec3.divide(ioResult.normal, ioResult.normal, shape.scale);
            vec3.normalize(ioResult.normal, ioResult.normal);
        },
        getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: ScaledShape, subShapeId: number): void {
            // accumulate scale: scale = scale * shape.scale
            vec3.multiply(ioResult.scale, ioResult.scale, shape.scale);

            // compute face in inner shape space - pass SubShapeID unchanged (decorator shapes don't consume bits)
            shapeDefs[shape.shape.type].getSupportingFace(ioResult, direction, shape.shape, subShapeId);
        },
        getInnerRadius(shape: ScaledShape): number {
            const minScale = Math.min(Math.abs(shape.scale[0]), Math.abs(shape.scale[1]), Math.abs(shape.scale[2]));
            return minScale * getShapeInnerRadius(shape.shape);
        },
        getLeafShape(outResult, shape, subShapeId): void {
            // pass through to inner shape
            const innerShapeDef = shapeDefs[shape.shape.type];
            innerShapeDef.getLeafShape(outResult, shape.shape, subShapeId);
        },
        getSubShapeTransformedShape(out, shape, subShapeId): void {
            // apply scale
            vec3.multiply(out.scale, out.scale, shape.scale);

            // pass through to inner shape
            const innerShapeDef = shapeDefs[shape.shape.type];
            innerShapeDef.getSubShapeTransformedShape(out, shape.shape, subShapeId);
        },
        castRay: castRayVsScaled,
        collidePoint: collidePointVsScaled,
        register: () => {
            for (const shapeDef of Object.values(shapeDefs)) {
                setCollideShapeFn(ShapeType.SCALED, shapeDef.type, collideScaledVsShape);
                setCollideShapeFn(shapeDef.type, ShapeType.SCALED, collideShapeVsScaled);

                setCastShapeFn(ShapeType.SCALED, shapeDef.type, castScaledVsShape);
                setCastShapeFn(shapeDef.type, ShapeType.SCALED, castShapeVsScaled);
            }
        },
    }))();

/* cast ray */

function castRayVsScaled(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: ScaledShape,
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
    // accumulate scale
    const newScaleX = scaleX * shape.scale[0];
    const newScaleY = scaleY * shape.scale[1];
    const newScaleZ = scaleZ * shape.scale[2];

    // cast against the inner shape with accumulated scale
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
        newScaleX,
        newScaleY,
        newScaleZ,
    );
}

/* collide point */

function collidePointVsScaled(
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: ScaledShape,
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
    // accumulate scale
    const newScaleBX = scaleBX * shapeB.scale[0];
    const newScaleBY = scaleBY * shapeB.scale[1];
    const newScaleBZ = scaleBZ * shapeB.scale[2];

    const innerShapeDef = shapeDefs[shapeB.shape.type];
    innerShapeDef.collidePoint(
        collector,
        settings,
        pointX,
        pointY,
        pointZ,
        shapeB.shape,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        newScaleBX,
        newScaleBY,
        newScaleBZ,
    );
}

/* collide shape */

function collideScaledVsShape(
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
    const scaled = shapeA as ScaledShape;

    // accumulate scale
    const newScaleAX = scaleAX * scaled.scale[0];
    const newScaleAY = scaleAY * scaled.scale[1];
    const newScaleAZ = scaleAZ * scaled.scale[2];

    const fn = collisionDispatch.collideFns.get(scaled.shape.type)?.get(shapeB.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        scaled.shape,
        subShapeIdA,
        subShapeIdBitsA,
        posAX,
        posAY,
        posAZ,
        quatAX,
        quatAY,
        quatAZ,
        quatAW,
        newScaleAX,
        newScaleAY,
        newScaleAZ,
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

function collideShapeVsScaled(
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
    const scaled = shapeB as ScaledShape;

    // accumulate scale
    const newScaleBX = scaleBX * scaled.scale[0];
    const newScaleBY = scaleBY * scaled.scale[1];
    const newScaleBZ = scaleBZ * scaled.scale[2];

    const fn = collisionDispatch.collideFns.get(shapeA.type)?.get(scaled.shape.type);

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
        scaled.shape,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        newScaleBX,
        newScaleBY,
        newScaleBZ,
    );
}

/* cast shape */

function castScaledVsShape(
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
    const scaled = shapeA as ScaledShape;

    // accumulate scale
    const newScaleX = scaleAX * scaled.scale[0];
    const newScaleY = scaleAY * scaled.scale[1];
    const newScaleZ = scaleAZ * scaled.scale[2];

    const fn = collisionDispatch.castFns.get(scaled.shape.type)?.get(shapeB.type);

    if (!fn) return;

    fn(
        collector,
        settings,
        scaled.shape,
        subShapeIdA,
        subShapeIdBitsA,
        posAX,
        posAY,
        posAZ,
        quatAX,
        quatAY,
        quatAZ,
        quatAW,
        newScaleX,
        newScaleY,
        newScaleZ,
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

function castShapeVsScaled(
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
    const scaled = shapeB as ScaledShape;

    // accumulate scale
    const newScaleX = scaleBX * scaled.scale[0];
    const newScaleY = scaleBY * scaled.scale[1];
    const newScaleZ = scaleBZ * scaled.scale[2];

    const fn = collisionDispatch.castFns.get(shapeA.type)?.get(scaled.shape.type);

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
        scaled.shape,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        newScaleX,
        newScaleY,
        newScaleZ,
    );
}
