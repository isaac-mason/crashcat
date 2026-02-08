import { type Box3, box3, quat, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as massProperties from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import { createCollidePointHit } from '../collision/collide-point-vs-shape';
import {
    type BoxSupport,
    createBoxSupport,
    DEFAULT_CONVEX_RADIUS,
    type Support,
    type SupportFunctionMode,
    setBoxSupport,
} from '../collision/support';
import { assert } from '../utils/assert';
import { isScaleInsideOut, transformFace } from '../utils/face';
import * as convex from './convex';
import {
    DEFAULT_SHAPE_DENSITY,
    defineShape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
    shapeDefs,
} from './shapes';

/** settings for creating a box shape */
export type BoxShapeSettings = {
    /** half extents of the box */
    halfExtents: Vec3;
    /** @default 0.05 @see DEFAULT_CONVEX_RADIUS */
    convexRadius?: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};

/** a box shape */
export type BoxShape = {
    type: ShapeType.BOX;
    halfExtents: Vec3;
    convexRadius: number;
    density: number;
    materialId: number;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

/** create a box shape from settings */
export function create(o: BoxShapeSettings): BoxShape {
    const shape: BoxShape = {
        type: ShapeType.BOX,
        halfExtents: o.halfExtents,
        convexRadius: o.convexRadius ?? DEFAULT_CONVEX_RADIUS,
        density: o.density ?? DEFAULT_SHAPE_DENSITY,
        materialId: o.materialId ?? -1,
        aabb: box3.create(),
        // always at origin
        centerOfMass: [0, 0, 0],
        volume: 0,
    };
    update(shape);
    return shape;
}

function computeBoxVolume(halfExtents: Vec3): number {
    // V = width * height * depth
    // halfExtents are already half the full dimensions, so multiply by 8
    return 8 * halfExtents[0] * halfExtents[1] * halfExtents[2];
}

function computeBoxLocalBounds(out: Box3, halfExtents: Vec3): void {
    out[0][0] = -halfExtents[0];
    out[0][1] = -halfExtents[1];
    out[0][2] = -halfExtents[2];
    out[1][0] = halfExtents[0];
    out[1][1] = halfExtents[1];
    out[1][2] = halfExtents[2];
}

/** updates a box shape after it's properties have changed */
export function update(shape: BoxShape): void {
    computeBoxLocalBounds(shape.aabb, shape.halfExtents);
    shape.volume = computeBoxVolume(shape.halfExtents);
}

/* shape def */

const _computeBoxMassProperties_fullExtents = /* @__PURE__ */ vec3.create();

export const def = defineShape<BoxShape>({
    type: ShapeType.BOX,
    category: ShapeCategory.CONVEX,
    computeMassProperties(out: MassProperties, shape: BoxShape): void {
        const fullExtents = vec3.scale(_computeBoxMassProperties_fullExtents, shape.halfExtents, 2);
        massProperties.setMassAndInertiaOfSolidBox(out, fullExtents, shape.density);
    },
    getSurfaceNormal(ioResult: SurfaceNormalResult, shape: BoxShape, subShapeId: number): void {
        assert(subShape.isEmpty(subShapeId), 'Invalid subshape ID for BoxShape');

        // get absolute distance from center to each face
        const diffX = Math.abs(Math.abs(ioResult.position[0]) - shape.halfExtents[0]);
        const diffY = Math.abs(Math.abs(ioResult.position[1]) - shape.halfExtents[1]);
        const diffZ = Math.abs(Math.abs(ioResult.position[2]) - shape.halfExtents[2]);

        // find axis closest to box surface
        let dominantAxis = 0;
        let minDist = diffX;
        if (diffY < minDist) {
            dominantAxis = 1;
            minDist = diffY;
        }
        if (diffZ < minDist) {
            dominantAxis = 2;
        }

        // return axis normal with sign based on position
        ioResult.normal[0] = 0;
        ioResult.normal[1] = 0;
        ioResult.normal[2] = 0;
        ioResult.normal[dominantAxis] = ioResult.position[dominantAxis] > 0.0 ? 1.0 : -1.0;
    },
    getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: BoxShape, _subShapeId: number): void {
        const hx = shape.halfExtents[0];
        const hy = shape.halfExtents[1];
        const hz = shape.halfExtents[2];
        const face = ioResult.face;
        const { position, quaternion, scale } = ioResult;

        // check if scale inverts winding
        const insideOut = isScaleInsideOut(scale);

        // find dominant axis
        const absX = Math.abs(direction[0]);
        const absY = Math.abs(direction[1]);
        const absZ = Math.abs(direction[2]);

        face.numVertices = 4;

        if (absX >= absY && absX >= absZ) {
            // face perpendicular to X axis
            if (direction[0] < 0) {
                // positive X face (direction points toward -X, so face is on +X side)
                if (insideOut) {
                    face.vertices[0] = hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = -hz;
                    face.vertices[9] = hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = -hz;
                } else {
                    face.vertices[0] = hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = -hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = hz;
                }
            } else {
                // negative X face
                if (insideOut) {
                    face.vertices[0] = -hx;
                    face.vertices[1] = hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = -hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = -hx;
                    face.vertices[7] = -hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = -hz;
                } else {
                    face.vertices[0] = -hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = -hx;
                    face.vertices[4] = -hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = -hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = hy;
                    face.vertices[11] = -hz;
                }
            }
        } else if (absY >= absX && absY >= absZ) {
            // face perpendicular to Y axis
            if (direction[1] < 0) {
                // positive Y face
                if (insideOut) {
                    face.vertices[0] = hx;
                    face.vertices[1] = hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = -hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = hy;
                    face.vertices[11] = -hz;
                } else {
                    face.vertices[0] = -hx;
                    face.vertices[1] = hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = -hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = hx;
                    face.vertices[10] = hy;
                    face.vertices[11] = -hz;
                }
            } else {
                // negative Y face
                if (insideOut) {
                    face.vertices[0] = -hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = -hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = -hy;
                    face.vertices[8] = -hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = -hz;
                } else {
                    face.vertices[0] = -hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = -hy;
                    face.vertices[5] = -hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = -hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = hz;
                }
            }
        } else {
            // face perpendicular to Z axis
            if (direction[2] < 0) {
                // positive Z face
                if (insideOut) {
                    face.vertices[0] = -hx;
                    face.vertices[1] = hy;
                    face.vertices[2] = hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = -hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = hz;
                } else {
                    face.vertices[0] = -hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = -hy;
                    face.vertices[5] = hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = hy;
                    face.vertices[11] = hz;
                }
            } else {
                // negative Z face
                if (insideOut) {
                    face.vertices[0] = hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = -hz;
                    face.vertices[6] = -hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = -hz;
                    face.vertices[9] = -hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = -hz;
                } else {
                    face.vertices[0] = -hx;
                    face.vertices[1] = -hy;
                    face.vertices[2] = -hz;
                    face.vertices[3] = -hx;
                    face.vertices[4] = hy;
                    face.vertices[5] = -hz;
                    face.vertices[6] = hx;
                    face.vertices[7] = hy;
                    face.vertices[8] = -hz;
                    face.vertices[9] = hx;
                    face.vertices[10] = -hy;
                    face.vertices[11] = -hz;
                }
            }
        }

        transformFace(face, position, quaternion, scale);
    },
    getInnerRadius(shape: BoxShape): number {
        return Math.min(shape.halfExtents[0], shape.halfExtents[1], shape.halfExtents[2]);
    },
    castRay: convex.castRayVsConvex,
    collidePoint: collidePointVsBox,
    createSupportPool: createBoxSupportPool,
    getSupportFunction: getBoxSupportFunction,
    register: () => {
        for (const shapeDef of Object.values(shapeDefs)) {
            if (shapeDef.category === ShapeCategory.CONVEX) {
                setCollideShapeFn(ShapeType.BOX, shapeDef.type, convex.collideConvexVsConvex);
                setCollideShapeFn(shapeDef.type, ShapeType.BOX, convex.collideConvexVsConvex);

                setCastShapeFn(ShapeType.BOX, shapeDef.type, convex.castConvexVsConvex);
                setCastShapeFn(shapeDef.type, ShapeType.BOX, convex.castConvexVsConvex);
            }
        }
    },
});

type BoxSupportPool = BoxSupport;

function createBoxSupportPool(): BoxSupportPool {
    return createBoxSupport();
}

function getBoxSupportFunction(pool: BoxSupportPool, shape: BoxShape, mode: SupportFunctionMode, scale: Vec3): Support {
    setBoxSupport(pool, shape.halfExtents, shape.convexRadius, mode, scale);
    return pool;
}

/* collide point */

const _collidePointVsBox_posB = /* @__PURE__ */ vec3.create();
const _collidePointVsBox_quatB = /* @__PURE__ */ quat.create();
const _collidePointVsBox_hit = /* @__PURE__ */ createCollidePointHit();

function collidePointVsBox(
    collector: CollidePointCollector,
    _settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: BoxShape,
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
    // transform point to box's local space
    const localX = pointX - posBX;
    const localY = pointY - posBY;
    const localZ = pointZ - posBZ;

    // rotate point by inverse of box rotation
    quat.set(_collidePointVsBox_quatB, quatBX, quatBY, quatBZ, quatBW);
    quat.conjugate(_collidePointVsBox_quatB, _collidePointVsBox_quatB);
    vec3.set(_collidePointVsBox_posB, localX, localY, localZ);
    vec3.transformQuat(_collidePointVsBox_posB, _collidePointVsBox_posB, _collidePointVsBox_quatB);

    // apply accumulated scale to half extents
    const scaledHalfX = shapeB.halfExtents[0] * Math.abs(scaleBX);
    const scaledHalfY = shapeB.halfExtents[1] * Math.abs(scaleBY);
    const scaledHalfZ = shapeB.halfExtents[2] * Math.abs(scaleBZ);

    // test if absolute value of each component is within scaled half extents
    if (
        Math.abs(_collidePointVsBox_posB[0]) <= scaledHalfX &&
        Math.abs(_collidePointVsBox_posB[1]) <= scaledHalfY &&
        Math.abs(_collidePointVsBox_posB[2]) <= scaledHalfZ
    ) {
        _collidePointVsBox_hit.subShapeIdB = subShapeIdB;
        _collidePointVsBox_hit.bodyIdB = collector.bodyIdB;
        collector.addHit(_collidePointVsBox_hit);
    }
}
