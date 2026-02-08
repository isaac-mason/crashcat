import { type Box3, box3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import { SupportFunctionMode, type Support } from '../collision/support';
import { assert } from '../utils/assert';
import { transformFace } from '../utils/face';
import * as convex from './convex';
import {
    DEFAULT_SHAPE_DENSITY,
    defineShape,
    ShapeCategory,
    shapeDefs,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
} from './shapes';
import { setCastShapeFn, setCollideShapeFn } from './shapes';

/** settings for creating a capsule shape */
export type CapsuleShapeSettings = {
    /** half height of the central cylinder (excluding hemisphere caps) */
    halfHeightOfCylinder: number;
    /** radius of the capsule */
    radius: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};

/** a capsule shape */
export type CapsuleShape = {
    /** capsule shape type */
    type: ShapeType.CAPSULE;
    /** half height of the central cylinder (excluding hemisphere caps) */
    halfHeightOfCylinder: number;
    /** radius of the capsule (the convex radius) */
    radius: number;
    /** the shape density */
    density: number;
    /** material identifier */
    materialId: number;
    /** local bounds */
    aabb: Box3;
    /** center of mass */
    centerOfMass: Vec3;
    /** volume */
    volume: number;
};

/** create a capsule shape from settings */
export function create(o: CapsuleShapeSettings): CapsuleShape {
    if (o.radius <= 0) {
        throw new Error('Invalid radius, must be > 0');
    }

    if (o.halfHeightOfCylinder <= 0) {
        throw new Error('Invalid height, must be > 0');
    }

    const shape: CapsuleShape = {
        type: ShapeType.CAPSULE,
        halfHeightOfCylinder: o.halfHeightOfCylinder,
        radius: o.radius,
        density: o.density ?? DEFAULT_SHAPE_DENSITY,
        materialId: o.materialId ?? -1,
        aabb: box3.create(),
        centerOfMass: vec3.create(),
        volume: 0,
    };

    update(shape);

    return shape;
}

function computeCapsuleVolume(halfHeightOfCylinder: number, radius: number): number {
    // volume = cylinder volume + sphere volume
    // v_cylinder = π * r² * h (where h = 2 * halfHeight)
    // v_sphere = (4/3) * π * r³
    const cylinderVolume = Math.PI * radius * radius * (2 * halfHeightOfCylinder);
    const sphereVolume = (4.0 / 3.0) * Math.PI * radius * radius * radius;
    return cylinderVolume + sphereVolume;
}

function computeCapsuleLocalBounds(out: Box3, halfHeightOfCylinder: number, radius: number): void {
    // capsule extends from -halfHeight-radius to +halfHeight+radius along Yand ±radius in X and Z
    const totalHalfHeight = halfHeightOfCylinder + radius;
    out[0][0] = -radius;
    out[0][1] = -totalHalfHeight;
    out[0][2] = -radius;
    out[1][0] = radius;
    out[1][1] = totalHalfHeight;
    out[1][2] = radius;
}

function computeCapsuleCenterOfMass(out: Vec3): void {
    // center of mass is at origin
    vec3.zero(out);
}

/** updates a capsule shape after it's properties have changed */
export function update(shape: CapsuleShape): void {
    computeCapsuleLocalBounds(shape.aabb, shape.halfHeightOfCylinder, shape.radius);
    computeCapsuleCenterOfMass(shape.centerOfMass);
    shape.volume = computeCapsuleVolume(shape.halfHeightOfCylinder, shape.radius);
}

/* shape def */

export const def = /* @__PURE__ */ (() =>
    defineShape<CapsuleShape>({
        type: ShapeType.CAPSULE,
        category: ShapeCategory.CONVEX,
        computeMassProperties(out: MassProperties, shape: CapsuleShape): void {
            const r = shape.radius;
            const h = shape.halfHeightOfCylinder;

            // mass = density * volume
            out.mass = shape.volume * shape.density;

            // calculate inertia
            const radius_sq = r * r;
            const height = 2.0 * h;
            const cylinder_mass = Math.PI * height * radius_sq * shape.density;
            const hemisphere_mass = ((2.0 * Math.PI) / 3.0) * radius_sq * r * shape.density;

            // from cylinder
            const height_sq = height * height;
            const inertia_y = radius_sq * cylinder_mass * 0.5;
            const inertia_xz = cylinder_mass * ((1.0 / 12.0) * height_sq + (1.0 / 4.0) * radius_sq);

            // from hemispheres
            const hemisphere_ixx_izz = hemisphere_mass * ((2.0 / 5.0) * radius_sq + h * h + (3.0 / 8.0) * h * r);
            const hemisphere_iyy = hemisphere_mass * (2.0 / 5.0) * radius_sq;

            // total inertia
            const i_xx_zz = inertia_xz + hemisphere_ixx_izz;
            const i_yy = inertia_y + hemisphere_iyy;

            // set diagonal inertia tensor (column-major mat4)
            out.inertia[0] = i_xx_zz;
            out.inertia[1] = 0;
            out.inertia[2] = 0;
            out.inertia[3] = 0;
            out.inertia[4] = 0;
            out.inertia[5] = i_yy;
            out.inertia[6] = 0;
            out.inertia[7] = 0;
            out.inertia[8] = 0;
            out.inertia[9] = 0;
            out.inertia[10] = i_xx_zz;
            out.inertia[11] = 0;
            out.inertia[12] = 0;
            out.inertia[13] = 0;
            out.inertia[14] = 0;
            out.inertia[15] = 1.0;
        },
        getSurfaceNormal(ioResult: SurfaceNormalResult, shape: CapsuleShape, subShapeId: number): void {
            assert(subShape.isEmpty(subShapeId), 'Invalid subshape ID for CapsuleShape');

            // capsule aligned along Y-axis from (0, -h, 0) to (0, h, 0) with radius r
            // clamp Y to cylinder range, then compute normal from clamped point
            const clampedY = Math.max(-shape.halfHeightOfCylinder, Math.min(shape.halfHeightOfCylinder, ioResult.position[1]));

            const toPointX = ioResult.position[0];
            const toPointY = ioResult.position[1] - clampedY;
            const toPointZ = ioResult.position[2];

            const len = Math.sqrt(toPointX * toPointX + toPointY * toPointY + toPointZ * toPointZ);
            if (len !== 0.0) {
                ioResult.normal[0] = toPointX / len;
                ioResult.normal[1] = toPointY / len;
                ioResult.normal[2] = toPointZ / len;
                return;
            }

            // fallback: if on central axis, use radial direction from X-Z plane
            // if exactly at center, default to Y axis
            const radialLen = Math.sqrt(
                ioResult.position[0] * ioResult.position[0] + ioResult.position[2] * ioResult.position[2],
            );
            if (radialLen > 0) {
                ioResult.normal[0] = ioResult.position[0] / radialLen;
                ioResult.normal[1] = 0;
                ioResult.normal[2] = ioResult.position[2] / radialLen;
            } else {
                ioResult.normal[0] = 1; // arbitrary radial direction
                ioResult.normal[1] = 0;
                ioResult.normal[2] = 0;
            }
        },
        getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: CapsuleShape, _subShapeId: number): void {
            const face = ioResult.face;
            const { position, quaternion, scale } = ioResult;
            const halfHeightOfCylinder = shape.halfHeightOfCylinder;
            const radius = shape.radius;

            // capsule is aligned along Y-axis: line segment from (0, -h, 0) to (0, h, 0) with radius r

            // get direction in horizontal plane (zero out Y component)
            const horizontalDirX = direction[0];
            const horizontalDirZ = direction[2];

            // check zero vector, in this case we're hitting from top/bottom so there's no supporting face
            const len = Math.sqrt(horizontalDirX * horizontalDirX + horizontalDirZ * horizontalDirZ);
            if (len === 0.0) {
                face.numVertices = 0;
                return;
            }

            // get support point for top and bottom sphere in the opposite of 'direction' (including convex radius)
            // support = (radius / len) * horizontal_direction
            const supportX = (radius / len) * horizontalDirX;
            const supportZ = (radius / len) * horizontalDirZ;

            // support_top = (0, halfHeight, 0) - support
            const supportTopX = -supportX;
            const supportTopY = halfHeightOfCylinder;
            const supportTopZ = -supportZ;

            // support_bottom = (0, -halfHeight, 0) - support
            const supportBottomX = -supportX;
            const supportBottomY = -halfHeightOfCylinder;
            const supportBottomZ = -supportZ;

            // get projection on inDirection.
            // note that inDirection is not normalized, so we need to divide by inDirection.Length() to get the actual projection.
            // we've multiplied both sides of the if below with inDirection.Length().
            const projTop = supportTopX * direction[0] + supportTopY * direction[1] + supportTopZ * direction[2];
            const projBottom = supportBottomX * direction[0] + supportBottomY * direction[1] + supportBottomZ * direction[2];

            // cCapsuleProjectionSlop = 0.02f (from PhysicsSettings.h)
            const capsuleProjectionSlop = 0.02;
            const directionLength = Math.sqrt(
                direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2],
            );

            // if projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
            if (Math.abs(projTop - projBottom) < capsuleProjectionSlop * directionLength) {
                face.numVertices = 2;
                face.vertices[0] = supportTopX;
                face.vertices[1] = supportTopY;
                face.vertices[2] = supportTopZ;
                face.vertices[3] = supportBottomX;
                face.vertices[4] = supportBottomY;
                face.vertices[5] = supportBottomZ;
                transformFace(face, position, quaternion, scale);
            } else {
                // only one point is relevant
                face.numVertices = 0;
            }
        },
        getInnerRadius(shape: CapsuleShape): number {
            return shape.radius;
        },
        castRay: convex.castRayVsConvex,
        collidePoint: convex.collidePointVsConvex,
        createSupportPool: createCapsuleSupportPool,
        getSupportFunction: getCapsuleSupportFunction,
        register: () => {
            // capsule vs all convex shapes
            for (const shapeDef of Object.values(shapeDefs)) {
                if (shapeDef.category === ShapeCategory.CONVEX) {
                    setCollideShapeFn(ShapeType.CAPSULE, shapeDef.type, convex.collideConvexVsConvex);
                    setCollideShapeFn(shapeDef.type, ShapeType.CAPSULE, convex.collideConvexVsConvex);

                    setCastShapeFn(ShapeType.CAPSULE, shapeDef.type, convex.castConvexVsConvex);
                    setCastShapeFn(shapeDef.type, ShapeType.CAPSULE, convex.castConvexVsConvex);
                }
            }
        },
    }))();

/* support functions */

/**
 * Capsule support for EXCLUDE_CONVEX_RADIUS mode.
 * Used by GJK - returns line segment endpoints, stores radius in convexRadius.
 */
export type CapsuleNoConvexSupport = {
    halfHeightOfCylinder: Vec3; // (0, halfHeight, 0)
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

function capsuleNoConvexGetSupport(this: CapsuleNoConvexSupport, direction: Vec3, out: Vec3): void {
    // a line segment along Y axis from (0, -halfHeight, 0) to (0, halfHeight, 0)
    // return top or bottom endpoint based on Y component of direction
    if (direction[1] > 0) {
        vec3.copy(out, this.halfHeightOfCylinder);
    } else {
        vec3.negate(out, this.halfHeightOfCylinder);
    }
}

export function createCapsuleNoConvexSupport(): CapsuleNoConvexSupport {
    return {
        halfHeightOfCylinder: vec3.create(),
        convexRadius: 0,
        getSupport: capsuleNoConvexGetSupport,
    };
}

export function setCapsuleNoConvexSupport(
    out: CapsuleNoConvexSupport,
    halfHeightOfCylinder: number,
    radius: number,
    scale: Vec3,
): void {
    // uniform scale only - take absolute value of first component
    const absScale = Math.abs(scale[0]);
    const scaledHalfHeight = absScale * halfHeightOfCylinder;
    const scaledRadius = absScale * radius;

    vec3.set(out.halfHeightOfCylinder, 0, scaledHalfHeight, 0);
    out.convexRadius = scaledRadius;
}

/**
 * Capsule support for INCLUDE_CONVEX_RADIUS mode.
 * Used by EPA and raycasting - returns surface points, convexRadius is 0.
 */
export type CapsuleWithConvexSupport = {
    halfHeightOfCylinder: Vec3; // (0, halfHeight, 0)
    radius: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

const _capsuleWithConvexGetSupport_radiusVec = /* @__PURE__ */ vec3.create();

function capsuleWithConvexGetSupport(this: CapsuleWithConvexSupport, direction: Vec3, out: Vec3): void {
    const length = vec3.length(direction);
    const radiusVec =
        length > 0
            ? vec3.scale(_capsuleWithConvexGetSupport_radiusVec, direction, this.radius / length)
            : vec3.zero(_capsuleWithConvexGetSupport_radiusVec);

    // add radius in direction to the appropriate endpoint
    if (direction[1] > 0) {
        vec3.add(out, radiusVec, this.halfHeightOfCylinder);
    } else {
        vec3.subtract(out, radiusVec, this.halfHeightOfCylinder);
    }
}

export function createCapsuleWithConvexSupport(): CapsuleWithConvexSupport {
    return {
        halfHeightOfCylinder: vec3.create(),
        radius: 0,
        convexRadius: 0,
        getSupport: capsuleWithConvexGetSupport,
    };
}

export function setCapsuleWithConvexSupport(
    out: CapsuleWithConvexSupport,
    halfHeightOfCylinder: number,
    radius: number,
    scale: Vec3,
): void {
    // uniform scale only - take absolute value of first component
    const absScale = Math.abs(scale[0]);
    const scaledHalfHeight = absScale * halfHeightOfCylinder;
    const scaledRadius = absScale * radius;

    vec3.set(out.halfHeightOfCylinder, 0, scaledHalfHeight, 0);
    out.radius = scaledRadius;
    out.convexRadius = 0;
}

type CapsuleSupportPool = {
    noConvex: CapsuleNoConvexSupport;
    withConvex: CapsuleWithConvexSupport;
};

function createCapsuleSupportPool(): CapsuleSupportPool {
    return {
        noConvex: createCapsuleNoConvexSupport(),
        withConvex: createCapsuleWithConvexSupport(),
    };
}

function getCapsuleSupportFunction(
    pool: CapsuleSupportPool,
    shape: CapsuleShape,
    mode: SupportFunctionMode,
    scale: Vec3,
): Support {
    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS) {
        setCapsuleWithConvexSupport(pool.withConvex, shape.halfHeightOfCylinder, shape.radius, scale);
        return pool.withConvex;
    } else {
        // EXCLUDE_CONVEX_RADIUS or DEFAULT
        setCapsuleNoConvexSupport(pool.noConvex, shape.halfHeightOfCylinder, shape.radius, scale);
        return pool.noConvex;
    }
}
