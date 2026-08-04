import { mat4, type Quat, quat, type Vec3, vec3 } from 'mathcat';
import { type Box3, box3 } from 'mathcat/shapes';
import type { MassProperties } from '../body/mass-properties';
import * as massProperties from '../body/mass-properties';
import * as subShape from '../body/sub-shape';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape';
import { CastRayStatus, createCastRayHit } from '../collision/cast-ray-vs-shape';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import { createCollidePointHit } from '../collision/collide-point-vs-shape';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape';
import { createCollideShapeHit, reversedCollideShapeVsShape } from '../collision/collide-shape-vs-shape';
import { DEFAULT_CONVEX_RADIUS, setBoxSupport } from '../collision/support';
import { assert } from '../utils/assert';
import { isScaleInsideOut, transformFaceWithMat4Scale } from '../utils/face';
import * as convex from './convex';
import {
    DEFAULT_SHAPE_DENSITY,
    defineShape,
    getShapeSupportingFace,
    type Shape,
    ShapeCategory,
    ShapeType,
    type SupportingFaceResult,
    type SurfaceNormalResult,
    setCastShapeFn,
    setCollideShapeFn,
    shapeDefs,
} from './shapes';
import type { SphereShape } from './sphere';

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
    // auto-clamp convex radius to fit the smallest half extent
    const requestedConvexRadius = o.convexRadius ?? DEFAULT_CONVEX_RADIUS;
    const minHalfExtent = Math.min(o.halfExtents[0], o.halfExtents[1], o.halfExtents[2]);
    const convexRadius = Math.min(requestedConvexRadius, minHalfExtent);

    const shape: BoxShape = {
        type: ShapeType.BOX,
        halfExtents: o.halfExtents,
        convexRadius,
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
    out[0] = -halfExtents[0];
    out[1] = -halfExtents[1];
    out[2] = -halfExtents[2];
    out[3] = halfExtents[0];
    out[4] = halfExtents[1];
    out[5] = halfExtents[2];
}

/** updates a box shape after it's properties have changed */
export function update(shape: BoxShape): void {
    // validation: dimensions must be non-negative
    if (shape.halfExtents[0] < 0 || shape.halfExtents[1] < 0 || shape.halfExtents[2] < 0) {
        throw new Error('box halfExtents must be >= 0');
    }
    if (shape.convexRadius < 0) {
        throw new Error('box convexRadius must be >= 0');
    }

    computeBoxLocalBounds(shape.aabb, shape.halfExtents);
    shape.volume = computeBoxVolume(shape.halfExtents);
}

/* shape def */

const _computeBoxMassProperties_fullExtents = /* @__PURE__ */ vec3.create();

export const def = /* @__PURE__ */ (() =>
    defineShape<BoxShape>({
        type: ShapeType.BOX,
        category: ShapeCategory.CONVEX,
        computeMassProperties,
        getSurfaceNormal,
        getSupportingFace,
        getInnerRadius,
        castRay: castRayVsBox,
        collidePoint: collidePointVsBox,
        setSupport: setBoxSupport,
        register: () => {
            for (const shapeDef of Object.values(shapeDefs)) {
                if (shapeDef.category === ShapeCategory.CONVEX) {
                    setCollideShapeFn(ShapeType.BOX, shapeDef.type, convex.collideConvexVsConvex);
                    setCollideShapeFn(shapeDef.type, ShapeType.BOX, convex.collideConvexVsConvex);

                    setCastShapeFn(ShapeType.BOX, shapeDef.type, convex.castConvexVsConvex);
                    setCastShapeFn(shapeDef.type, ShapeType.BOX, convex.castConvexVsConvex);
                }
            }

            // analytical sphere vs box (closed-form clamp, skips GJK/EPA).
            // must be registered after the generic convex loop above: box (ShapeType.BOX = 1)
            // registers after sphere (ShapeType.SPHERE = 0), and the loop sets these pairs to
            // the generic handler, so the override has to be applied here to win.
            setCollideShapeFn(ShapeType.SPHERE, ShapeType.BOX, collideSphereVsBox);
            setCollideShapeFn(ShapeType.BOX, ShapeType.SPHERE, reversedCollideShapeVsShape(collideSphereVsBox));
        },
    }))();

/* collide sphere vs box (analytical, A = sphere, B = box) */

const _collideSphereVsBox_hit = /* @__PURE__ */ createCollideShapeHit();
const _collideSphereVsBox_boxRotation: Quat = /* @__PURE__ */ quat.create();
const _collideSphereVsBox_invBoxRotation: Quat = /* @__PURE__ */ quat.create();
const _collideSphereVsBox_localCenter: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_coreHalf: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_negCoreHalf: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_closest: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_delta: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_normal: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_face: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_boxPosition: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_worldScratch: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_boxToWorld = /* @__PURE__ */ mat4.create();
const _collideSphereVsBox_boxScale: Vec3 = /* @__PURE__ */ vec3.create();
const _collideSphereVsBox_faceDirection: Vec3 = /* @__PURE__ */ vec3.create();

/**
 * Analytical sphere vs box collision. A is the sphere, B is the box.
 *
 * Closed-form clamp of the sphere centre to the box's shrunk core (half-extents minus convex
 * radius, mirroring setBoxSupport EXCLUDE_CONVEX_RADIUS), with the combined radius handling the
 * rounded shell. Skips GJK/EPA entirely; the deep (centre-inside-core) case degrades to a per-axis
 * SAT scan rather than EPA. Bit-equivalent to convex.collideConvexVsConvex on shallow contacts.
 *
 * The mathcat frame transforms are written idiomatically; compilecat's `@optimize` (flatten +
 * SROA) inlines the vec3/quat calls and localises the literal-initialised scratch, so the hot
 * path compiles to straight-line scalar arithmetic with no module-array round-trips or calls.
 * (The faces branch keeps its scratch arrays — they feed the un-inlined getShapeSupportingFace.)
 *
 * @optimize
 */
export function collideSphereVsBox(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    _subShapeIdBitsA: number,
    posAX: number,
    posAY: number,
    posAZ: number,
    _quatAX: number,
    _quatAY: number,
    _quatAZ: number,
    _quatAW: number,
    scaleAX: number,
    _scaleAY: number,
    _scaleAZ: number,
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
    const sphereShape = shapeA as SphereShape;
    const boxShape = shapeB as BoxShape;

    const sphereRadius = sphereShape.radius * Math.abs(scaleAX);

    // scaled half-extents and shrunk core (mirror setBoxSupport EXCLUDE_CONVEX_RADIUS)
    const scaledHalfX = Math.abs(scaleBX) * boxShape.halfExtents[0];
    const scaledHalfY = Math.abs(scaleBY) * boxShape.halfExtents[1];
    const scaledHalfZ = Math.abs(scaleBZ) * boxShape.halfExtents[2];
    const minBoxScale = Math.min(Math.abs(scaleBX), Math.abs(scaleBY), Math.abs(scaleBZ));
    const scaledConvexRadius = Math.min(boxShape.convexRadius * minBoxScale, DEFAULT_CONVEX_RADIUS);
    const coreHalfX = Math.max(0, scaledHalfX - scaledConvexRadius);
    const coreHalfY = Math.max(0, scaledHalfY - scaledConvexRadius);
    const coreHalfZ = Math.max(0, scaledHalfZ - scaledConvexRadius);
    const combinedRadius = sphereRadius + scaledConvexRadius;

    // box rotation (box-local -> world) and its inverse (world -> box-local). scale is folded into
    // the half-extents above, matching the gjk path. rotation is isometric, so distances in the
    // box-local frame equal world distances.
    quat.set(_collideSphereVsBox_boxRotation, quatBX, quatBY, quatBZ, quatBW);
    quat.conjugate(_collideSphereVsBox_invBoxRotation, _collideSphereVsBox_boxRotation);

    // sphere centre into box-local: rotate (posA - posB) by the inverse box rotation
    vec3.set(_collideSphereVsBox_localCenter, posAX - posBX, posAY - posBY, posAZ - posBZ);
    vec3.transformQuat(_collideSphereVsBox_localCenter, _collideSphereVsBox_localCenter, _collideSphereVsBox_invBoxRotation);

    // closest point on the core box: clamp the centre to [-coreHalf, +coreHalf]
    vec3.set(_collideSphereVsBox_coreHalf, coreHalfX, coreHalfY, coreHalfZ);
    vec3.negate(_collideSphereVsBox_negCoreHalf, _collideSphereVsBox_coreHalf);
    vec3.min(_collideSphereVsBox_closest, _collideSphereVsBox_localCenter, _collideSphereVsBox_coreHalf);
    vec3.max(_collideSphereVsBox_closest, _collideSphereVsBox_closest, _collideSphereVsBox_negCoreHalf);

    // delta = centre - closest; distance² is its squared length
    vec3.subtract(_collideSphereVsBox_delta, _collideSphereVsBox_localCenter, _collideSphereVsBox_closest);
    const distanceSq = vec3.squaredLength(_collideSphereVsBox_delta);
    const contactDistance = combinedRadius + settings.maxSeparationDistance;
    if (distanceSq > contactDistance * contactDistance) {
        return; // separated beyond radius (+ speculative margin)
    }

    // outward normal (box surface -> sphere centre) and the core-face contact point, in box-local
    const normal = _collideSphereVsBox_normal;
    const face = _collideSphereVsBox_face;
    let penetration: number;

    if (distanceSq > 1e-12) {
        // shallow: sphere centre outside the core box — normal is the normalised delta
        const distance = Math.sqrt(distanceSq);
        vec3.scale(normal, _collideSphereVsBox_delta, 1 / distance);
        vec3.copy(face, _collideSphereVsBox_closest);
        penetration = combinedRadius - distance;
    } else {
        // deep: sphere centre inside the core box -> nearest face per axis (SAT, no EPA). per axis
        // the nearest-face depth is coreHalfExtent - |centre|, its outward normal is sign(centre);
        // pick the smallest-depth axis (ties X > Y > Z, +face over -face — a straight six-face scan).
        const localX = _collideSphereVsBox_localCenter[0];
        const localY = _collideSphereVsBox_localCenter[1];
        const localZ = _collideSphereVsBox_localCenter[2];
        const depthX = coreHalfX - Math.abs(localX);
        const depthY = coreHalfY - Math.abs(localY);
        const depthZ = coreHalfZ - Math.abs(localZ);
        let nx = 0;
        let ny = 0;
        let nz = 0;
        let fx = localX;
        let fy = localY;
        let fz = localZ;
        let depth: number;
        if (depthX <= depthY && depthX <= depthZ) {
            depth = depthX;
            nx = localX < 0 ? -1 : 1;
            fx = nx * coreHalfX;
        } else if (depthY <= depthZ) {
            depth = depthY;
            ny = localY < 0 ? -1 : 1;
            fy = ny * coreHalfY;
        } else {
            depth = depthZ;
            nz = localZ < 0 ? -1 : 1;
            fz = nz * coreHalfZ;
        }
        vec3.set(normal, nx, ny, nz);
        vec3.set(face, fx, fy, fz);
        penetration = combinedRadius + depth;
    }

    // early-out on penetration vs the collector budget (matches the convex path)
    if (-penetration >= collector.earlyOutFraction) {
        return;
    }

    const hit = _collideSphereVsBox_hit;
    vec3.set(_collideSphereVsBox_boxPosition, posBX, posBY, posBZ);

    // contact points, box-local -> world:
    //   box surface  = core face + convexRadius along the normal
    //   sphere point = centre - sphereRadius along the (box -> sphere) normal
    vec3.scaleAndAdd(_collideSphereVsBox_worldScratch, face, normal, scaledConvexRadius);
    vec3.transformQuat(_collideSphereVsBox_worldScratch, _collideSphereVsBox_worldScratch, _collideSphereVsBox_boxRotation);
    vec3.add(hit.pointB, _collideSphereVsBox_worldScratch, _collideSphereVsBox_boxPosition);

    vec3.scaleAndAdd(_collideSphereVsBox_worldScratch, _collideSphereVsBox_localCenter, normal, -sphereRadius);
    vec3.transformQuat(_collideSphereVsBox_worldScratch, _collideSphereVsBox_worldScratch, _collideSphereVsBox_boxRotation);
    vec3.add(hit.pointA, _collideSphereVsBox_worldScratch, _collideSphereVsBox_boxPosition);

    // penetration axis: A -> B = -(box-local normal, which points box -> sphere = B -> A) in world
    vec3.transformQuat(_collideSphereVsBox_worldScratch, normal, _collideSphereVsBox_boxRotation);
    vec3.negate(hit.penetrationAxis, _collideSphereVsBox_worldScratch);

    hit.penetration = penetration;
    hit.subShapeIdA = subShapeIdA;
    hit.subShapeIdB = subShapeIdB;
    hit.materialIdA = sphereShape.materialId;
    hit.materialIdB = boxShape.materialId;
    hit.bodyIdB = collector.bodyIdB;

    if (settings.collectFaces) {
        // box supporting face: getSupportingFace picks the face by dominant axis + sign of the
        // passed direction; -normal selects the face whose outward normal is +normal.
        vec3.negate(_collideSphereVsBox_faceDirection, normal);
        vec3.set(_collideSphereVsBox_boxScale, scaleBX, scaleBY, scaleBZ);
        mat4.fromRotationTranslation(
            _collideSphereVsBox_boxToWorld,
            _collideSphereVsBox_boxRotation,
            _collideSphereVsBox_boxPosition,
        );
        getShapeSupportingFace(
            hit.faceB,
            boxShape,
            subShapeIdB,
            _collideSphereVsBox_faceDirection,
            _collideSphereVsBox_boxToWorld,
            _collideSphereVsBox_boxScale,
        );

        // sphere has no supporting face (single point contact)
        hit.faceA.numVertices = 0;
    }

    collector.addHit(hit);
}

function computeMassProperties(out: MassProperties, shape: BoxShape): void {
    const fullExtents = vec3.scale(_computeBoxMassProperties_fullExtents, shape.halfExtents, 2);
    massProperties.setMassAndInertiaOfSolidBox(out, fullExtents, shape.density);
}

function getSurfaceNormal(ioResult: SurfaceNormalResult, shape: BoxShape, subShapeId: number): void {
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
}

function getSupportingFace(ioResult: SupportingFaceResult, direction: Vec3, shape: BoxShape, _subShapeId: number): void {
    const hx = shape.halfExtents[0];
    const hy = shape.halfExtents[1];
    const hz = shape.halfExtents[2];
    const face = ioResult.face;
    const { transform, scale } = ioResult;

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

    transformFaceWithMat4Scale(face, transform, scale);
}
function getInnerRadius(shape: BoxShape): number {
    return Math.min(shape.halfExtents[0], shape.halfExtents[1], shape.halfExtents[2]);
}

/* cast ray */

const _castRayVsBox_invQuat = /* @__PURE__ */ quat.create();
const _castRayVsBox_origin = /* @__PURE__ */ vec3.create();
const _castRayVsBox_dir = /* @__PURE__ */ vec3.create();
const _castRayVsBox_hit = /* @__PURE__ */ createCastRayHit();

/**
 * Analytic ray-vs-box (slab test), replacing the generic GJK convex cast for boxes. Not an
 * approximation: the gjk path already casts against a sharp box of |scale|·halfExtents with zero
 * convex radius (setBoxSupport under INCLUDE_CONVEX_RADIUS), so this is bit-equivalent geometry,
 * exact rather than iterated to a 1e-3 tolerance. Reporting matches castRayVsConvex (entry hit,
 * treatConvexAsSolid gate); the hit carries no normal, matching CastRayHit.
 */
export function castRayVsBox(
    collector: CastRayCollector,
    settings: CastRaySettings,
    originX: number,
    originY: number,
    originZ: number,
    directionX: number,
    directionY: number,
    directionZ: number,
    length: number,
    shape: BoxShape,
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
    // transform the ray into the box's rotation-local frame (translation + rotation removed). scale
    // is folded into the half extents below rather than inverted out of the ray — matching the gjk
    // path's convention, so the geometry is identical.
    quat.set(_castRayVsBox_invQuat, quatX, quatY, quatZ, quatW);
    quat.conjugate(_castRayVsBox_invQuat, _castRayVsBox_invQuat);

    vec3.set(_castRayVsBox_origin, originX - posX, originY - posY, originZ - posZ);
    vec3.transformQuat(_castRayVsBox_origin, _castRayVsBox_origin, _castRayVsBox_invQuat);
    vec3.set(_castRayVsBox_dir, directionX, directionY, directionZ);
    vec3.transformQuat(_castRayVsBox_dir, _castRayVsBox_dir, _castRayVsBox_invQuat);

    const ox = _castRayVsBox_origin[0];
    const oy = _castRayVsBox_origin[1];
    const oz = _castRayVsBox_origin[2];
    // scale the direction by ray length so the slab fractions are along [0, 1] of the ray (matching
    // the gjk lambda convention in castRayVsConvex)
    const dx = _castRayVsBox_dir[0] * length;
    const dy = _castRayVsBox_dir[1] * length;
    const dz = _castRayVsBox_dir[2] * length;

    const hx = Math.abs(scaleX) * shape.halfExtents[0];
    const hy = Math.abs(scaleY) * shape.halfExtents[1];
    const hz = Math.abs(scaleZ) * shape.halfExtents[2];

    // slab test against the centered box [-h, h]; tmin/tmax bound the ray-fraction hit interval
    let tmin = -Infinity;
    let tmax = Infinity;

    if (Math.abs(dx) < 1e-10) {
        if (ox < -hx || ox > hx) {
            collector.addMiss();
            return;
        }
    } else {
        const inv = 1 / dx;
        let t0 = (-hx - ox) * inv;
        let t1 = (hx - ox) * inv;
        if (t0 > t1) {
            const t = t0;
            t0 = t1;
            t1 = t;
        }
        if (t0 > tmin) tmin = t0;
        if (t1 < tmax) tmax = t1;
        if (tmin > tmax) {
            collector.addMiss();
            return;
        }
    }

    if (Math.abs(dy) < 1e-10) {
        if (oy < -hy || oy > hy) {
            collector.addMiss();
            return;
        }
    } else {
        const inv = 1 / dy;
        let t0 = (-hy - oy) * inv;
        let t1 = (hy - oy) * inv;
        if (t0 > t1) {
            const t = t0;
            t0 = t1;
            t1 = t;
        }
        if (t0 > tmin) tmin = t0;
        if (t1 < tmax) tmax = t1;
        if (tmin > tmax) {
            collector.addMiss();
            return;
        }
    }

    if (Math.abs(dz) < 1e-10) {
        if (oz < -hz || oz > hz) {
            collector.addMiss();
            return;
        }
    } else {
        const inv = 1 / dz;
        let t0 = (-hz - oz) * inv;
        let t1 = (hz - oz) * inv;
        if (t0 > t1) {
            const t = t0;
            t0 = t1;
            t1 = t;
        }
        if (t0 > tmin) tmin = t0;
        if (t1 < tmax) tmax = t1;
        if (tmin > tmax) {
            collector.addMiss();
            return;
        }
    }

    // the box is hit over ray fractions [tmin, tmax]; reject if that interval misses the ray segment
    // [0, 1] (box entirely behind the origin, or entirely beyond the ray's end)
    if (tmax < 0 || tmin > 1) {
        collector.addMiss();
        return;
    }

    // entry fraction (gjk lambda); tmin < 0 means the origin is inside the box
    const fraction = tmin;
    if (settings.treatConvexAsSolid || fraction > 0.0) {
        _castRayVsBox_hit.status = CastRayStatus.COLLIDING;
        _castRayVsBox_hit.fraction = Math.max(0.0, fraction);
        _castRayVsBox_hit.subShapeId = subShapeId;
        _castRayVsBox_hit.materialId = shape.materialId;
        _castRayVsBox_hit.bodyIdB = collector.bodyIdB;
        collector.addHit(_castRayVsBox_hit);
    } else {
        collector.addMiss();
    }
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
