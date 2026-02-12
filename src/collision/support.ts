import { type Mat4, mat4, type Quat, type Vec3, vec3 } from 'mathcat';
import { type Shape, type ShapeDef, type ShapeType, shapeDefs } from '../shapes/shapes';
import type { Face } from '../utils/face';

export const DEFAULT_CONVEX_RADIUS = 0.05;

export type ShapeSupportPool = {
    shapes: Partial<Record<ShapeType, unknown>>;
    dispose: () => void;
};

export enum SupportFunctionMode {
    INCLUDE_CONVEX_RADIUS,
    EXCLUDE_CONVEX_RADIUS,
    DEFAULT,
}

export type Support = {
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

/* shape support pool - pre-allocated support objects for hot paths */

const allSupportPools = /* @__PURE__ */ new Set<ShapeSupportPool>();

/** creates a new shape support pool */
export function createShapeSupportPool(): ShapeSupportPool {
    function dispose() {
        allSupportPools.delete(pool);
    }

    const pool: ShapeSupportPool = { shapes: {}, dispose };

    for (const def of Object.values(shapeDefs)) {
        const shapePool = def.createSupportPool();
        pool.shapes[def.type] = shapePool;
    }

    allSupportPools.add(pool);

    return pool;
}

export function allocateShapeSupportPools(def: ShapeDef<any>): void {
    for (const pool of allSupportPools) {
        // create a separate support pool instance for each existing support pool
        const shapePool = def.createSupportPool();
        if (shapePool !== undefined) {
            pool.shapes[def.type] = shapePool;
        }
    }
}

/* shape + mode + scale -> support function */

export function getShapeSupportFunction(pool: ShapeSupportPool, shape: Shape, mode: SupportFunctionMode, scale: Vec3): Support {
    const shapeDef = shapeDefs[shape.type];

    // pool entry may not exist for non-convex shapes
    const shapePool = pool.shapes[shape.type];
    const support = shapeDef.getSupportFunction(shapePool, shape, mode, scale) as Support;

    // if (support === undefined) {
    //     throw new Error(`Support function not implemented for shape type ${shape?.type}`);
    // }

    return support;
}

/* non-shape-specific support utilities for gjk/epa */

/* triangle support - for triangle vs convex shape collisions with gjk */

export type TriangleSupport = {
    a: Vec3;
    b: Vec3;
    c: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

function triangleGetSupport(this: TriangleSupport, direction: Vec3, out: Vec3): void {
    const { a, b, c } = this;
    const [dx, dy, dz] = direction;

    // project vertices on direction
    const d1 = a[0] * dx + a[1] * dy + a[2] * dz;
    const d2 = b[0] * dx + b[1] * dy + b[2] * dz;
    const d3 = c[0] * dx + c[1] * dy + c[2] * dz;

    // return vertex with biggest projection
    if (d1 > d2) {
        if (d1 > d3) {
            out[0] = a[0];
            out[1] = a[1];
            out[2] = a[2];
        } else {
            out[0] = c[0];
            out[1] = c[1];
            out[2] = c[2];
        }
    } else {
        if (d2 > d3) {
            out[0] = b[0];
            out[1] = b[1];
            out[2] = b[2];
        } else {
            out[0] = c[0];
            out[1] = c[1];
            out[2] = c[2];
        }
    }
}

export function createTriangleSupport(): TriangleSupport {
    return {
        a: vec3.create(),
        b: vec3.create(),
        c: vec3.create(),
        convexRadius: 0,
        getSupport: triangleGetSupport,
    };
}

export function setTriangleSupport(out: TriangleSupport, a: Vec3, b: Vec3, c: Vec3): void {
    out.a[0] = a[0];
    out.a[1] = a[1];
    out.a[2] = a[2];

    out.b[0] = b[0];
    out.b[1] = b[1];
    out.b[2] = b[2];

    out.c[0] = c[0];
    out.c[1] = c[1];
    out.c[2] = c[2];
}

/* point - lets us do point vs convex shape collisions with gjk */

export type PointSupport = {
    point: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

function pointGetSupport(this: PointSupport, _direction: Vec3, out: Vec3): void {
    vec3.copy(out, this.point);
}

export function createPointSupport(): PointSupport {
    return {
        point: vec3.create(),
        convexRadius: 0,
        getSupport: pointGetSupport,
    };
}

export function setPointSupport(out: PointSupport, point: Vec3): void {
    vec3.copy(out.point, point);
}

/* polygon support */

export type PolygonSupport = {
    vertices: number[]; // flat array [x1, y1, z1, x2, y2, z2, ...]
    numVertices: number;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

function polygonGetSupport(this: PolygonSupport, direction: Vec3, out: Vec3): void {
    // find vertex with maximum dot product in the given direction

    // initialize with first vertex
    if (this.numVertices === 0) {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        return;
    }

    let bestDot = this.vertices[0] * direction[0] + this.vertices[1] * direction[1] + this.vertices[2] * direction[2];
    let bestIndex = 0;

    // check remaining vertices
    for (let i = 1; i < this.numVertices; i++) {
        const vx = this.vertices[i * 3];
        const vy = this.vertices[i * 3 + 1];
        const vz = this.vertices[i * 3 + 2];
        const dot = vx * direction[0] + vy * direction[1] + vz * direction[2];

        if (dot > bestDot) {
            bestDot = dot;
            bestIndex = i;
        }
    }

    out[0] = this.vertices[bestIndex * 3];
    out[1] = this.vertices[bestIndex * 3 + 1];
    out[2] = this.vertices[bestIndex * 3 + 2];
}

export function createPolygonSupport(): PolygonSupport {
    return {
        vertices: [],
        numVertices: 0,
        convexRadius: 0,
        getSupport: polygonGetSupport,
    };
}

export function setPolygonSupport(out: PolygonSupport, face: Face): void {
    out.vertices = face.vertices;
    out.numVertices = face.numVertices;
    out.convexRadius = 0; // face geometry doesn't include convex radius
}

/* transformed support - applies a position + rotation to an inner support function */

// uses mat4 approach like jolt's TransformedConvexObject for efficiency:
// - direction to local: multiply3x3Transposed (inverse rotation for orthonormal)
// - support to world: multiply3x3 + translate
// this is ~2x fewer operations than quaternion approach per getSupport call

const _transformedSupport_localDirection = /* @__PURE__ */ vec3.create();

export type TransformedSupport = {
    convexRadius: number;
    support: Support | null;
    transform: Mat4; // rotation + translation matrix
    getSupport(direction: Vec3, out: Vec3): void;
    innerSupport: Support | null;
};

function transformedGetSupport(this: TransformedSupport, direction: Vec3, out: Vec3): void {
    const { support: inner, transform: m } = this;

    // transform direction to local space using transposed 3x3 (inverse rotation)
    // mat4.multiply3x3TransposedVec inlined for performance
    const dx = direction[0];
    const dy = direction[1];
    const dz = direction[2];

    _transformedSupport_localDirection[0] = m[0] * dx + m[1] * dy + m[2] * dz;
    _transformedSupport_localDirection[1] = m[4] * dx + m[5] * dy + m[6] * dz;
    _transformedSupport_localDirection[2] = m[8] * dx + m[9] * dy + m[10] * dz;

    // get support point in local space
    inner!.getSupport(_transformedSupport_localDirection, out);

    // transform support point to world space: rotation + translation
    // mat4.multiply3x3Vec + translation inlined for performance
    const sx = out[0];
    const sy = out[1];
    const sz = out[2];

    out[0] = m[0] * sx + m[4] * sy + m[8] * sz + m[12];
    out[1] = m[1] * sx + m[5] * sy + m[9] * sz + m[13];
    out[2] = m[2] * sx + m[6] * sy + m[10] * sz + m[14];
}

export function createTransformedSupport(): TransformedSupport {
    return {
        convexRadius: 0,
        support: null,
        transform: mat4.create(),
        getSupport: transformedGetSupport,
        innerSupport: null,
    };
}

export function setTransformedSupport(out: TransformedSupport, position: Vec3, quaternion: Quat, innerSupport: Support): void {
    out.support = innerSupport;
    out.innerSupport = innerSupport;
    out.convexRadius = innerSupport.convexRadius;
    mat4.fromRotationTranslation(out.transform, quaternion, position);
}

/* add convex radius to support function */

export type AddConvexRadiusSupport = {
    innerSupport: Support;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

function addConvexRadiusGetSupport(this: AddConvexRadiusSupport, direction: Vec3, out: Vec3): void {
    this.innerSupport.getSupport(direction, out);

    const dx = direction[0];
    const dy = direction[1];
    const dz = direction[2];
    const lengthSq = dx * dx + dy * dy + dz * dz;

    if (lengthSq > 0) {
        const scale = this.convexRadius / Math.sqrt(lengthSq);
        out[0] += dx * scale;
        out[1] += dy * scale;
        out[2] += dz * scale;
    }
}

export function createAddConvexRadiusSupport(): AddConvexRadiusSupport {
    return {
        innerSupport: null!,
        convexRadius: 0,
        getSupport: addConvexRadiusGetSupport,
    };
}

export function setAddConvexRadiusSupport(out: AddConvexRadiusSupport, convexRadius: number, innerSupport: Support): void {
    out.innerSupport = innerSupport;
    out.convexRadius = convexRadius;
}

/* box support */

export type BoxSupport = {
    halfExtents: Vec3;
    convexRadius: number;
    getSupport(direction: Vec3, out: Vec3): void;
};

function boxGetSupport(this: BoxSupport, direction: Vec3, out: Vec3): void {
    out[0] = direction[0] >= 0 ? this.halfExtents[0] : -this.halfExtents[0];
    out[1] = direction[1] >= 0 ? this.halfExtents[1] : -this.halfExtents[1];
    out[2] = direction[2] >= 0 ? this.halfExtents[2] : -this.halfExtents[2];
}

export function createBoxSupport(): BoxSupport {
    return {
        halfExtents: vec3.create(),
        convexRadius: 0,
        getSupport: boxGetSupport,
    };
}

export function setBoxSupport(
    out: BoxSupport,
    halfExtents: Vec3,
    convexRadius: number,
    mode: SupportFunctionMode,
    scale: Vec3
): void {
    // scale half extents component-wise with absolute scale
    const scaledX = Math.abs(scale[0]) * halfExtents[0];
    const scaledY = Math.abs(scale[1]) * halfExtents[1];
    const scaledZ = Math.abs(scale[2]) * halfExtents[2];

    if (mode === SupportFunctionMode.EXCLUDE_CONVEX_RADIUS) {
        // scale convex radius using minimum scale component
        const minScale = Math.min(Math.abs(scale[0]), Math.abs(scale[1]), Math.abs(scale[2]));
        // jolt clamps scaled convex radius to BOX_DEFAULT_CONVEX_RADIUS
        // TODO: is this intended?
        const scaledConvexRadius = Math.min(convexRadius * minScale, DEFAULT_CONVEX_RADIUS);

        // reduce geometry by the convex radius, then report the excluded amount
        out.halfExtents[0] = Math.max(0, scaledX - scaledConvexRadius);
        out.halfExtents[1] = Math.max(0, scaledY - scaledConvexRadius);
        out.halfExtents[2] = Math.max(0, scaledZ - scaledConvexRadius);
        out.convexRadius = scaledConvexRadius;
    } else {
        // DEFAULT or INCLUDE_CONVEX_RADIUS: convex radius is baked into the geometry, so report zero additional radius
        out.halfExtents[0] = scaledX;
        out.halfExtents[1] = scaledY;
        out.halfExtents[2] = scaledZ;
        out.convexRadius = 0;
    }
}
