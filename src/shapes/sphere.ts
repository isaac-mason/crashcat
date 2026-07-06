import { type Box3, box3, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import {
    type CollidePointCollector,
    type CollidePointSettings,
    createCollidePointHit,
} from '../collision/collide-point-vs-shape';
import {
    type CollideShapeCollector,
    type CollideShapeSettings,
    createCollideShapeHit,
} from '../collision/collide-shape-vs-shape';
import { setSphereSupport } from '../collision/support';
import * as convex from './convex';
import type { Shape } from './shapes';
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

/** settings for creating a sphere shape */
export type SphereShapeSettings = {
    /** the radius of the sphere */
    radius: number;
    /** @default 1000 @see DEFAULT_SHAPE_DENSITY */
    density?: number;
    /** material identifier @default -1 */
    materialId?: number;
};

/** sphere shape */
export type SphereShape = {
    type: ShapeType.SPHERE;
    /** the radius of the sphere */
    radius: number;
    /** the shape density */
    density: number;
    /** material identifier */
    materialId: number;
    /** shape local bounds */
    aabb: Box3;
    /** shape center of mass */
    centerOfMass: Vec3;
    /** shape volume */
    volume: number;
};

/** create a sphere shape */
export function create(o: SphereShapeSettings): SphereShape {
    const shape: SphereShape = {
        type: ShapeType.SPHERE,
        radius: o.radius,
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

function computeSphereVolume(radius: number): number {
    // V = (4/3) * π * r³
    return (4 / 3) * Math.PI * radius * radius * radius;
}

function computeSphereLocalBounds(out: Box3, radius: number): void {
    out[0] = -radius;
    out[1] = -radius;
    out[2] = -radius;
    out[3] = radius;
    out[4] = radius;
    out[5] = radius;
}

/** updates a sphere shape after it's properties have changed */
export function update(shape: SphereShape): void {
    computeSphereLocalBounds(shape.aabb, shape.radius);
    shape.volume = computeSphereVolume(shape.radius);
}

/* shape def */

export const def = /* @__PURE__ */ (() =>
    defineShape<SphereShape>({
        type: ShapeType.SPHERE,
        category: ShapeCategory.CONVEX,
        computeMassProperties,
        getSurfaceNormal,
        getSupportingFace,
        getInnerRadius,
        castRay: convex.castRayVsConvex,
        collidePoint: collidePointVsSphere,
        setSupport: setSphereSupport,
        register: () => {
            // sphere vs convex shapes
            for (const shapeDef of Object.values(shapeDefs)) {
                if (shapeDef.category === ShapeCategory.CONVEX) {
                    setCollideShapeFn(ShapeType.SPHERE, shapeDef.type, convex.collideConvexVsConvex);
                    setCollideShapeFn(shapeDef.type, ShapeType.SPHERE, convex.collideConvexVsConvex);

                    setCastShapeFn(ShapeType.SPHERE, shapeDef.type, convex.castConvexVsConvex);
                    setCastShapeFn(shapeDef.type, ShapeType.SPHERE, convex.castConvexVsConvex);
                }
            }

            // optimized sphere vs sphere handler
            setCollideShapeFn(ShapeType.SPHERE, ShapeType.SPHERE, collideSphereVsSphere);
        },
    }))();

function computeMassProperties(out: MassProperties, shape: SphereShape): void {
    const r2 = shape.radius * shape.radius;
    out.mass = (4.0 / 3.0) * Math.PI * shape.radius * r2 * shape.density;

    // calculate inertia: I = (2/5) * m * r²
    const inertia = (2.0 / 5.0) * out.mass * r2;

    // set diagonal inertia tensor using mat4::sScale pattern
    // diagonal matrix in column-major order: [I, 0, 0, 0, 0, I, 0, 0, 0, 0, I, 0, 0, 0, 0, 1]
    out.inertia[0] = inertia;
    out.inertia[1] = 0;
    out.inertia[2] = 0;
    out.inertia[3] = 0;
    out.inertia[4] = 0;
    out.inertia[5] = inertia;
    out.inertia[6] = 0;
    out.inertia[7] = 0;
    out.inertia[8] = 0;
    out.inertia[9] = 0;
    out.inertia[10] = inertia;
    out.inertia[11] = 0;
    out.inertia[12] = 0;
    out.inertia[13] = 0;
    out.inertia[14] = 0;
    out.inertia[15] = 1.0;
}

function getSurfaceNormal(ioResult: SurfaceNormalResult, _shape: SphereShape, _subShapeId: number): void {
    const len = vec3.length(ioResult.position);

    if (len !== 0.0) {
        ioResult.normal[0] = ioResult.position[0] / len;
        ioResult.normal[1] = ioResult.position[1] / len;
        ioResult.normal[2] = ioResult.position[2] / len;
        return;
    }

    // fallback to y axis if position is at origin
    ioResult.normal[0] = 0;
    ioResult.normal[1] = 1;
    ioResult.normal[2] = 0;
}

function getSupportingFace(ioResult: SupportingFaceResult, _direction: Vec3, _shape: SphereShape, _subShapeId: number): void {
    // sphere has no supporting face (single point)
    ioResult.face.numVertices = 0;
}

function getInnerRadius(shape: SphereShape): number {
    return shape.radius;
}

/* collide point */

const _collidePointHit = /* @__PURE__ */ createCollidePointHit();

function collidePointVsSphere(
    collector: CollidePointCollector,
    _settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: SphereShape,
    subShapeIdB: number,
    _subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    _quatBX: number,
    _quatBY: number,
    _quatBZ: number,
    _quatBW: number,
    scaleBX: number,
    _scaleBY: number,
    _scaleBZ: number,
): void {
    // transform point to sphere's local space
    const localX = pointX - posBX;
    const localY = pointY - posBY;
    const localZ = pointZ - posBZ;

    // apply accumulated scale to radius (uniform scale, using X component)
    const radius = shapeB.radius * Math.abs(scaleBX);

    // sphere is centered at origin in local space, test distance squared
    const distSq = localX * localX + localY * localY + localZ * localZ;
    const radiusSq = radius * radius;

    if (distSq <= radiusSq) {
        _collidePointHit.subShapeIdB = subShapeIdB;
        _collidePointHit.materialId = shapeB.materialId;
        _collidePointHit.bodyIdB = collector.bodyIdB;
        collector.addHit(_collidePointHit);
    }
}

/* collide shape */

const _collideSphereVsSphere_hit = /* @__PURE__ */ createCollideShapeHit();

export function collideSphereVsSphere(
    collector: CollideShapeCollector,
    _settings: CollideShapeSettings,
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
    _quatBX: number,
    _quatBY: number,
    _quatBZ: number,
    _quatBW: number,
    scaleBX: number,
    _scaleBY: number,
    _scaleBZ: number,
): void {
    // optimized case for sphere vs sphere
    // simpler than GJK - just check distance between centers

    // at this point, decorators have been unwrapped by dispatch
    // we can safely cast to SphereShape
    const sphereA = shapeA as SphereShape;
    const sphereB = shapeB as SphereShape;

    // apply accumulated scale to radii (uniform scale, using X component)
    const radiusA = sphereA.radius * Math.abs(scaleAX);
    const radiusB = sphereB.radius * Math.abs(scaleBX);

    // distance between centers (inline vector math)
    const dx = posBX - posAX;
    const dy = posBY - posAY;
    const dz = posBZ - posAZ;
    const distSq = dx * dx + dy * dy + dz * dz;
    const radiusSum = radiusA + radiusB;

    // no collision if distance > sum of radii
    if (distSq >= radiusSum * radiusSum) {
        return;
    }

    const dist = Math.sqrt(distSq);
    const penetration = radiusSum - dist;

    // normal from A to B
    let normalX: number, normalY: number, normalZ: number;

    if (dist > 0) {
        const invDist = 1 / dist;
        normalX = dx * invDist;
        normalY = dy * invDist;
        normalZ = dz * invDist;
    } else {
        // spheres at same position, use arbitrary normal
        normalX = 0;
        normalY = 1;
        normalZ = 0;
    }

    // report hit
    const hit = _collideSphereVsSphere_hit;

    hit.pointA[0] = posAX + normalX * radiusA;
    hit.pointA[1] = posAY + normalY * radiusA;
    hit.pointA[2] = posAZ + normalZ * radiusA;

    hit.pointB[0] = posBX - normalX * radiusB;
    hit.pointB[1] = posBY - normalY * radiusB;
    hit.pointB[2] = posBZ - normalZ * radiusB;

    hit.penetrationAxis[0] = normalX;
    hit.penetrationAxis[1] = normalY;
    hit.penetrationAxis[2] = normalZ;

    hit.penetration = penetration;
    hit.subShapeIdA = subShapeIdA;
    hit.subShapeIdB = subShapeIdB;
    hit.materialIdA = sphereA.materialId;
    hit.materialIdB = sphereB.materialId;
    hit.bodyIdB = collector.bodyIdB;

    collector.addHit(hit);
}
