import type { Vec3 } from 'math';
import { type Box3, box3 } from 'math/shapes';
import type { MassProperties } from '../body/mass-properties';
import { DEFAULT_CONVEX_RADIUS, setCylinderSupport } from '../collision/support';
import { isScaleInsideOut, transformFaceWithMat4Scale } from '../utils/face';
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

/** settings for creating a cylinder shape */
export type CylinderShapeSettings = {
    halfHeight: number;
    radius: number;
    /** @default 0.05 @see DEFAULT_CONVEX_RADIUS */
    convexRadius?: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};

/** cylinder shape aligned with Y-axis */
export type CylinderShape = {
    type: ShapeType.CYLINDER;
    halfHeight: number;
    radius: number;
    convexRadius: number;
    density: number;
    materialId: number;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

// octagon template for cap faces
// y component is always 1.0 (unit circle at Y=1), will be scaled to ±halfHeight
const CYLINDER_CAP_OCTAGON = [
    [0.0, 1.0, 1.0],
    [0.707106769, 1.0, 0.707106769], // sqrt(2)/2 ≈ 0.707
    [1.0, 1.0, 0.0],
    [0.707106769, 1.0, -0.707106769],
    [0.0, 1.0, -1.0],
    [-0.707106769, 1.0, -0.707106769],
    [-1.0, 1.0, 0.0],
    [-0.707106769, 1.0, 0.707106769],
];

/** create cylinder shape from settings */
export function create(o: CylinderShapeSettings): CylinderShape {
    // auto-clamp convex radius to fit the smallest dimension
    const requestedConvexRadius = o.convexRadius ?? DEFAULT_CONVEX_RADIUS;
    const convexRadius = Math.min(requestedConvexRadius, Math.min(o.halfHeight, o.radius));
    const density = o.density ?? DEFAULT_SHAPE_DENSITY;

    const shape: CylinderShape = {
        type: ShapeType.CYLINDER,
        halfHeight: o.halfHeight,
        radius: o.radius,
        convexRadius,
        density,
        materialId: o.materialId ?? -1,
        aabb: box3.create(),
        centerOfMass: [0, 0, 0],
        volume: 0,
    };

    update(shape);

    return shape;
}

/** calculate cylinder volume - V = π × r² × (2h) */
function calculateVolume(halfHeight: number, radius: number): number {
    return Math.PI * radius * radius * 2 * halfHeight;
}

/** calculate AABB for cylinder */
function calculateAABB(out: Box3, halfHeight: number, radius: number): void {
    out[0] = -radius;
    out[1] = -halfHeight;
    out[2] = -radius;
    out[3] = radius;
    out[4] = halfHeight;
    out[5] = radius;
}

/** update cylinder shape's derived properties */
export function update(shape: CylinderShape): void {
    // validation: dimensions must be non-negative
    if (shape.halfHeight < 0) {
        throw new Error('cylinder halfHeight must be >= 0');
    }
    if (shape.radius < 0) {
        throw new Error('cylinder radius must be >= 0');
    }
    if (shape.convexRadius < 0) {
        throw new Error('cylinder convexRadius must be >= 0');
    }

    // update
    calculateAABB(shape.aabb, shape.halfHeight, shape.radius);
    shape.centerOfMass[0] = 0;
    shape.centerOfMass[1] = 0;
    shape.centerOfMass[2] = 0;
    shape.volume = calculateVolume(shape.halfHeight, shape.radius);
}

/* shape def */

export const def = /* @__PURE__ */ (() =>
    defineShape<CylinderShape>({
        type: ShapeType.CYLINDER,
        category: ShapeCategory.CONVEX,
        computeMassProperties,
        getSurfaceNormal,
        getSupportingFace,
        getInnerRadius,
        castRay: convex.castRayVsConvex,
        collidePoint: convex.collidePointVsConvex,
        setSupport: setCylinderSupport,
        register: () => {
            // cylinder vs all convex shapes
            for (const shapeDef of Object.values(shapeDefs)) {
                if (shapeDef.category === ShapeCategory.CONVEX) {
                    setCollideShapeFn(ShapeType.CYLINDER, shapeDef.type, convex.collideConvexVsConvex);
                    setCollideShapeFn(shapeDef.type, ShapeType.CYLINDER, convex.collideConvexVsConvex);

                    setCastShapeFn(ShapeType.CYLINDER, shapeDef.type, convex.castConvexVsConvex);
                    setCastShapeFn(shapeDef.type, ShapeType.CYLINDER, convex.castConvexVsConvex);
                }
            }
        },
    }))();

function computeMassProperties(out: MassProperties, shape: CylinderShape): void {
    // mass = density * volume
    out.mass = shape.volume * shape.density;

    // inertia tensor for solid cylinder aligned with Y-axis:
    // I_yy = 0.5 × mass × r²
    // I_xx = I_zz = 0.5 × I_yy + mass × (2h)² / 12
    const r = shape.radius;
    const h = shape.halfHeight;
    const mass = out.mass;

    const iyy = 0.5 * mass * r * r;
    const ixx = 0.5 * iyy + (mass * 4 * h * h) / 12;
    const izz = ixx;

    // set diagonal inertia tensor (column-major mat4)
    out.inertia[0] = ixx;
    out.inertia[1] = 0;
    out.inertia[2] = 0;
    out.inertia[3] = 0;
    out.inertia[4] = 0;
    out.inertia[5] = iyy;
    out.inertia[6] = 0;
    out.inertia[7] = 0;
    out.inertia[8] = 0;
    out.inertia[9] = 0;
    out.inertia[10] = izz;
    out.inertia[11] = 0;
    out.inertia[12] = 0;
    out.inertia[13] = 0;
    out.inertia[14] = 0;
    out.inertia[15] = 1.0;
}

function getSurfaceNormal(ioResult: SurfaceNormalResult, shape: CylinderShape): void {
    const x = ioResult.position[0];
    const y = ioResult.position[1];
    const z = ioResult.position[2];

    // calculate distance to curved surface and cap surfaces
    const radialDist = Math.sqrt(x * x + z * z);
    const distCurved = Math.abs(radialDist - shape.radius);
    const distCaps = Math.abs(Math.abs(y) - shape.halfHeight);

    if (distCaps > distCurved) {
        // closer to curved surface - return radial normal
        if (radialDist > 0) {
            ioResult.normal[0] = x / radialDist;
            ioResult.normal[1] = 0;
            ioResult.normal[2] = z / radialDist;
        } else {
            // point on central axis - default to X axis
            ioResult.normal[0] = 1;
            ioResult.normal[1] = 0;
            ioResult.normal[2] = 0;
        }
    } else {
        // closer to cap - return ±Y normal
        ioResult.normal[0] = 0;
        ioResult.normal[1] = y >= 0 ? 1 : -1;
        ioResult.normal[2] = 0;
    }
}

function getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: CylinderShape): void {
    const face = ioResult.face;
    const { transform, scale } = ioResult;
    const halfHeight = shape.halfHeight;
    const radius = shape.radius;

    // check if scale inverts winding
    const insideOut = isScaleInsideOut(scale);

    // calculate squared components to determine if hitting side or cap
    const xzSq = direction[0] * direction[0] + direction[2] * direction[2];
    const ySq = direction[1] * direction[1];

    if (xzSq > ySq) {
        // case 1: side face - direction is more horizontal than vertical
        // returns 2 vertices forming a vertical edge on the curved surface

        face.numVertices = 2;

        const horizontalLen = Math.sqrt(xzSq);

        // point on curved surface in penetration direction (negative)
        const f = -radius / horizontalLen;
        const vx = direction[0] * f;
        const vz = direction[2] * f;

        // store local vertices
        face.vertices[0] = vx;
        face.vertices[1] = halfHeight;
        face.vertices[2] = vz;
        face.vertices[3] = vx;
        face.vertices[4] = -halfHeight;
        face.vertices[5] = vz;
    } else {
        // case 2: cap face - direction is more vertical than horizontal
        // returns 8 vertices (octagon approximation of circular cap)

        face.numVertices = 8;

        // bottom cap (y < 0): (radius, halfHeight, radius)
        // top cap (y >= 0): (-radius, -halfHeight, radius)
        const scaleX = direction[1] < 0 ? radius : -radius;
        const scaleY = direction[1] < 0 ? halfHeight : -halfHeight;
        const scaleZ = radius;

        // when direction is more than 5 degrees from vertical, rotate octagon so one vertex
        // points toward max penetration
        // 0.00765427 ≈ tan²(5°)
        let rotationCos = 1;
        let rotationSin = 0;
        if (xzSq > 0.00765427 * ySq) {
            // normalize XZ direction to get rotation angle
            const horizontalLen = Math.sqrt(xzSq);
            rotationCos = direction[0] / horizontalLen;
            rotationSin = direction[2] / horizontalLen;
        }

        // write octagon vertices with winding reversal for inside-out scales
        for (let i = 0; i < 8; i++) {
            const idx = insideOut ? 7 - i : i;
            const template = CYLINDER_CAP_OCTAGON[idx];

            // first apply scale
            const scaledX = scaleX * template[0];
            const scaledY = scaleY * template[1];
            const scaledZ = scaleZ * template[2];

            // then apply 2D rotation in XZ plane
            const rotatedX = scaledX * rotationCos - scaledZ * rotationSin;
            const rotatedZ = scaledX * rotationSin + scaledZ * rotationCos;

            const base = i * 3;
            face.vertices[base] = rotatedX;
            face.vertices[base + 1] = scaledY;
            face.vertices[base + 2] = rotatedZ;
        }
    }

    transformFaceWithMat4Scale(face, transform, scale);
}

function getInnerRadius(shape: CylinderShape): number {
    return Math.min(shape.halfHeight, shape.radius);
}
