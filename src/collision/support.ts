import { type Mat4, mat4, type Vec3, vec3 } from 'math';
import type { BoxShape } from '../shapes/box';
import type { CapsuleShape } from '../shapes/capsule';
import type { ConvexHullShape } from '../shapes/convex-hull';
import type { CylinderShape } from '../shapes/cylinder';
import type { SphereShape } from '../shapes/sphere';

export const DEFAULT_CONVEX_RADIUS = 0.05;

export enum SupportFunctionMode {
    INCLUDE_CONVEX_RADIUS,
    EXCLUDE_CONVEX_RADIUS,
    DEFAULT,
}

const EMPTY_VERTICES: number[] = [];

/**
 * Support evaluation through the struct.
 *
 * A single {@link Support} struct (one hidden class) is filled once per collision pair. the fill
 * installs the shape's evaluator on the struct next to its parameters, and gjk/epa call it many
 * times per pair as `support.getSupport(out, support, direction)`. there is no kind tag and no
 * switch: the evaluator *is* the kind.
 *
 * Every evaluator is self-contained — it rotates the direction into local space, finds the core
 * support, adds the radius along the local direction and transforms the point back — so each
 * evaluation is one straight-line optimisation unit with nothing shared across a call boundary.
 *
 * Radius contract:
 *  - `convexRadius` is the *reported* radius. no evaluator adds it; the collision driver reads it and
 *    passes it to `gjkClosestPoints`/EPA for the shrunk-core-plus-radius distance math.
 *  - `addRadius` is an extra radius added along the (local) direction by the evaluator itself (the
 *    EPA speculative-separation / cast convex radius). 0 on the GJK path.
 *  - "mode" (include vs exclude convex radius) is baked into the params by the fill: exclude uses the
 *    shrunk core + reports `convexRadius`; include uses the full/rounded core + `convexRadius = 0`.
 */

/** evaluate the support point of `support` in direction `direction`, writing it to `out` */
export type SupportFunction = (out: Vec3, support: Support, direction: Vec3) => void;

export type Support = {
    /** the shape's evaluator, installed by the fill. the hot call site is `support.getSupport(out, support, direction)` */
    getSupport: SupportFunction;

    /** reported convex radius — read by the driver, never added by the evaluator (0 in include mode) */
    convexRadius: number;

    /** extra radius added along the local direction by the evaluator (EPA separation / cast radius) */
    addRadius: number;

    /** whether the support has a transform */
    hasTransform: boolean;

    /** B-in-A transform, applied when hasTransform is true (identity otherwise) */
    transform: Mat4;

    /** axis-aligned box: support is the corner picked by the sign of the direction on each axis (±halfExtents) */
    box: { halfExtents: Vec3 };

    /** sphere core: the origin, rounded by `radius`. 0 in exclude mode, where the radius is reported instead. */
    sphere: { radius: number };

    /** capsule core: a segment of half-length `halfHeight` along local Y, rounded by `radius` (0 in exclude mode) */
    capsule: { halfHeight: number; radius: number };

    /**
     * cylinder: `radius` is the radial extent in the local XZ plane, `halfHeight` the axial extent along
     * local Y. support = the radial extreme (`radius·dir̂ₓ_z`) combined with the near/far axial cap.
     */
    cylinder: { radius: number; halfHeight: number };

    /** triangle (mesh operand): three vertices as a flat `[ax,ay,az, bx,by,bz, cx,cy,cz]` buffer, owned and copied into by the fill */
    triangle: { vertices: number[] };

    /** single point (collidePoint operand), owned and copied into by the fill */
    point: Vec3;

    /** convex vertex set (a hull, or a borrowed polygon face) — support is the vertex with the greatest dot product against the direction */
    hull: {
        /** flat `[x,y,z,...]` vertices scanned by the evaluator; read-only borrow valid for the current pair */
        vertices: number[];
        /** number of vertices in `vertices` (it may be longer than `vertexCount * 3`) */
        vertexCount: number;
        /** per-vertex output scale — the winning vertex is multiplied by this (uniform-scale fast path); 1 when `vertices` are already baked */
        outputScale: number;
        /** owned grow-once buffer that `vertices` points at for the non-uniform scaled slow path */
        scratch: number[];
        /** borrowed CSR neighbour prefix offsets (shape-owned, length numPoints+1); empty ⇔ brute scan */
        neighborsStart: number[];
        /** borrowed CSR flat neighbour indices (shape-owned), indexed via `neighborsStart` */
        neighbors: number[];
        /** warm-start hint: the last winning vertex index, carried across support calls within one pair; -1 = cold */
        lastVertex: number;
    };
};

/**
 * Allocate a reusable {@link Support}. A driver holds a small fixed number of these (e.g. one per
 * operand slot) and refills them per pair via the fill functions. The sub-objects, transform, and
 * scratch buffer are pre-allocated so filling never allocates. starts as a zero-radius sphere at the origin.
 */
export function createSupport(): Support {
    return {
        getSupport: sphereSupport,
        convexRadius: 0,
        addRadius: 0,
        hasTransform: false,
        transform: mat4.create(),
        box: { halfExtents: vec3.create() },
        sphere: { radius: 0 },
        capsule: { halfHeight: 0, radius: 0 },
        cylinder: { radius: 0, halfHeight: 0 },
        triangle: { vertices: [0, 0, 0, 0, 0, 0, 0, 0, 0] },
        point: vec3.create(),
        hull: {
            vertices: EMPTY_VERTICES,
            vertexCount: 0,
            outputScale: 1,
            scratch: [],
            neighborsStart: EMPTY_VERTICES,
            neighbors: EMPTY_VERTICES,
            lastVertex: -1,
        },
    };
}

/**
 * sphere evaluator. a sphere's support is rotation invariant: R·(r·dir̂_local) where
 * dir̂_local = Rᵀ·dir̂ is just r·dir̂, so the direction transform and the rotation half of the
 * transform-back cancel and only the translation survives. addRadius is applied along that same
 * direction, so it folds into the radius.
 */
export function sphereSupport(out: Vec3, support: Support, direction: Vec3): void {
    const radius = support.sphere.radius + support.addRadius;
    const directionX = direction[0];
    const directionY = direction[1];
    const directionZ = direction[2];
    let supportX = 0;
    let supportY = 0;
    let supportZ = 0;
    if (radius > 0) {
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = radius / Math.sqrt(lengthSq);
            supportX = directionX * scale;
            supportY = directionY * scale;
            supportZ = directionZ * scale;
        }
    }
    if (support.hasTransform) {
        const m = support.transform;
        out[0] = supportX + m[12];
        out[1] = supportY + m[13];
        out[2] = supportZ + m[14];
    } else {
        out[0] = supportX;
        out[1] = supportY;
        out[2] = supportZ;
    }
}

/** box evaluator: the corner picked by the sign of the local direction on each axis */
export function boxSupport(out: Vec3, support: Support, direction: Vec3): void {
    // direction into local space (inverse rotation = transposed 3x3)
    let directionX = direction[0];
    let directionY = direction[1];
    let directionZ = direction[2];
    if (support.hasTransform) {
        const m = support.transform;
        const localX = m[0] * directionX + m[1] * directionY + m[2] * directionZ;
        const localY = m[4] * directionX + m[5] * directionY + m[6] * directionZ;
        const localZ = m[8] * directionX + m[9] * directionY + m[10] * directionZ;
        directionX = localX;
        directionY = localY;
        directionZ = localZ;
    }

    const halfExtents = support.box.halfExtents;
    let supportX = directionX >= 0 ? halfExtents[0] : -halfExtents[0];
    let supportY = directionY >= 0 ? halfExtents[1] : -halfExtents[1];
    let supportZ = directionZ >= 0 ? halfExtents[2] : -halfExtents[2];

    // rounding along the local direction, then back to the caller's space
    const radius = support.addRadius;
    if (radius > 0) {
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = radius / Math.sqrt(lengthSq);
            supportX += directionX * scale;
            supportY += directionY * scale;
            supportZ += directionZ * scale;
        }
    }
    if (support.hasTransform) {
        const m = support.transform;
        out[0] = m[0] * supportX + m[4] * supportY + m[8] * supportZ + m[12];
        out[1] = m[1] * supportX + m[5] * supportY + m[9] * supportZ + m[13];
        out[2] = m[2] * supportX + m[6] * supportY + m[10] * supportZ + m[14];
    } else {
        out[0] = supportX;
        out[1] = supportY;
        out[2] = supportZ;
    }
}

/** capsule evaluator: the near end of the core segment, rounded by the capsule radius */
export function capsuleSupport(out: Vec3, support: Support, direction: Vec3): void {
    let directionX = direction[0];
    let directionY = direction[1];
    let directionZ = direction[2];
    if (support.hasTransform) {
        const m = support.transform;
        const localX = m[0] * directionX + m[1] * directionY + m[2] * directionZ;
        const localY = m[4] * directionX + m[5] * directionY + m[6] * directionZ;
        const localZ = m[8] * directionX + m[9] * directionY + m[10] * directionZ;
        directionX = localX;
        directionY = localY;
        directionZ = localZ;
    }

    const capsule = support.capsule;
    let supportX = 0;
    let supportY = directionY > 0 ? capsule.halfHeight : -capsule.halfHeight;
    let supportZ = 0;

    // the rounding that makes the segment a capsule, plus the driver's
    const radius = capsule.radius + support.addRadius;
    if (radius > 0) {
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = radius / Math.sqrt(lengthSq);
            supportX += directionX * scale;
            supportY += directionY * scale;
            supportZ += directionZ * scale;
        }
    }
    if (support.hasTransform) {
        const m = support.transform;
        out[0] = m[0] * supportX + m[4] * supportY + m[8] * supportZ + m[12];
        out[1] = m[1] * supportX + m[5] * supportY + m[9] * supportZ + m[13];
        out[2] = m[2] * supportX + m[6] * supportY + m[10] * supportZ + m[14];
    } else {
        out[0] = supportX;
        out[1] = supportY;
        out[2] = supportZ;
    }
}

/** cylinder evaluator: the radial extreme in the local XZ plane on the near/far cap */
export function cylinderSupport(out: Vec3, support: Support, direction: Vec3): void {
    let directionX = direction[0];
    let directionY = direction[1];
    let directionZ = direction[2];
    if (support.hasTransform) {
        const m = support.transform;
        const localX = m[0] * directionX + m[1] * directionY + m[2] * directionZ;
        const localY = m[4] * directionX + m[5] * directionY + m[6] * directionZ;
        const localZ = m[8] * directionX + m[9] * directionY + m[10] * directionZ;
        directionX = localX;
        directionY = localY;
        directionZ = localZ;
    }

    const cylinder = support.cylinder;
    let supportX: number;
    let supportZ: number;
    const horizontalLen = Math.sqrt(directionX * directionX + directionZ * directionZ);
    if (horizontalLen > 0) {
        const scale = cylinder.radius / horizontalLen;
        supportX = directionX * scale;
        supportZ = directionZ * scale;
    } else {
        supportX = 0;
        supportZ = 0;
    }
    let supportY = directionY >= 0 ? cylinder.halfHeight : -cylinder.halfHeight;

    const radius = support.addRadius;
    if (radius > 0) {
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = radius / Math.sqrt(lengthSq);
            supportX += directionX * scale;
            supportY += directionY * scale;
            supportZ += directionZ * scale;
        }
    }
    if (support.hasTransform) {
        const m = support.transform;
        out[0] = m[0] * supportX + m[4] * supportY + m[8] * supportZ + m[12];
        out[1] = m[1] * supportX + m[5] * supportY + m[9] * supportZ + m[13];
        out[2] = m[2] * supportX + m[6] * supportY + m[10] * supportZ + m[14];
    } else {
        out[0] = supportX;
        out[1] = supportY;
        out[2] = supportZ;
    }
}

/** triangle evaluator: three points with a last-maximal tie-break — mesh contact quality depends on it */
export function triangleSupport(out: Vec3, support: Support, direction: Vec3): void {
    let directionX = direction[0];
    let directionY = direction[1];
    let directionZ = direction[2];
    if (support.hasTransform) {
        const m = support.transform;
        const localX = m[0] * directionX + m[1] * directionY + m[2] * directionZ;
        const localY = m[4] * directionX + m[5] * directionY + m[6] * directionZ;
        const localZ = m[8] * directionX + m[9] * directionY + m[10] * directionZ;
        directionX = localX;
        directionY = localY;
        directionZ = localZ;
    }

    const v = support.triangle.vertices;
    const dotA = v[0] * directionX + v[1] * directionY + v[2] * directionZ;
    const dotB = v[3] * directionX + v[4] * directionY + v[5] * directionZ;
    const dotC = v[6] * directionX + v[7] * directionY + v[8] * directionZ;
    const base = dotA > dotB ? (dotA > dotC ? 0 : 6) : dotB > dotC ? 3 : 6;
    let supportX = v[base];
    let supportY = v[base + 1];
    let supportZ = v[base + 2];

    const radius = support.addRadius;
    if (radius > 0) {
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = radius / Math.sqrt(lengthSq);
            supportX += directionX * scale;
            supportY += directionY * scale;
            supportZ += directionZ * scale;
        }
    }
    if (support.hasTransform) {
        const m = support.transform;
        out[0] = m[0] * supportX + m[4] * supportY + m[8] * supportZ + m[12];
        out[1] = m[1] * supportX + m[5] * supportY + m[9] * supportZ + m[13];
        out[2] = m[2] * supportX + m[6] * supportY + m[10] * supportZ + m[14];
    } else {
        out[0] = supportX;
        out[1] = supportY;
        out[2] = supportZ;
    }
}

/** point evaluator: the point itself, rounded by addRadius along the local direction */
export function pointSupport(out: Vec3, support: Support, direction: Vec3): void {
    const point = support.point;
    let supportX = point[0];
    let supportY = point[1];
    let supportZ = point[2];

    const radius = support.addRadius;
    if (radius > 0) {
        let directionX = direction[0];
        let directionY = direction[1];
        let directionZ = direction[2];
        if (support.hasTransform) {
            const m = support.transform;
            const localX = m[0] * directionX + m[1] * directionY + m[2] * directionZ;
            const localY = m[4] * directionX + m[5] * directionY + m[6] * directionZ;
            const localZ = m[8] * directionX + m[9] * directionY + m[10] * directionZ;
            directionX = localX;
            directionY = localY;
            directionZ = localZ;
        }
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = radius / Math.sqrt(lengthSq);
            supportX += directionX * scale;
            supportY += directionY * scale;
            supportZ += directionZ * scale;
        }
    }
    if (support.hasTransform) {
        const m = support.transform;
        out[0] = m[0] * supportX + m[4] * supportY + m[8] * supportZ + m[12];
        out[1] = m[1] * supportX + m[5] * supportY + m[9] * supportZ + m[13];
        out[2] = m[2] * supportX + m[6] * supportY + m[10] * supportZ + m[14];
    } else {
        out[0] = supportX;
        out[1] = supportY;
        out[2] = supportZ;
    }
}

/**
 * convex point-set evaluator: a hull or a polygon face. brute scan when no adjacency is baked or
 * the pair is cold; otherwise a warm hill-climb over the baked 1-ring.
 */
export function hullSupport(out: Vec3, support: Support, direction: Vec3): void {
    let directionX = direction[0];
    let directionY = direction[1];
    let directionZ = direction[2];
    if (support.hasTransform) {
        const m = support.transform;
        const localX = m[0] * directionX + m[1] * directionY + m[2] * directionZ;
        const localY = m[4] * directionX + m[5] * directionY + m[6] * directionZ;
        const localZ = m[8] * directionX + m[9] * directionY + m[10] * directionZ;
        directionX = localX;
        directionY = localY;
        directionZ = localZ;
    }

    const hull = support.hull;
    const vertices = hull.vertices;
    const neighborsStart = hull.neighborsStart;
    let supportX = 0;
    let supportY = 0;
    let supportZ = 0;

    if (neighborsStart.length === 0 || hull.lastVertex === -1) {
        // brute scan — no adjacency baked, or cold pair-fill. when accelerated, this first
        // call seeds the warm-start hint with the exact argmax.
        const length = hull.vertexCount * 3;
        let bestDot = -Infinity;
        let bestBase = 0;
        for (let i = 0; i < length; i += 3) {
            const vertexX = vertices[i];
            const vertexY = vertices[i + 1];
            const vertexZ = vertices[i + 2];
            const dot = vertexX * directionX + vertexY * directionY + vertexZ * directionZ;
            if (dot > bestDot) {
                bestDot = dot;
                bestBase = i;
                supportX = vertexX;
                supportY = vertexY;
                supportZ = vertexZ;
            }
        }
        if (neighborsStart.length !== 0) {
            hull.lastVertex = bestBase / 3;
        }
    } else {
        // warm hill-climb over the baked 1-ring: take the best neighbour of the current
        // vertex, repeat until none improves. a local max over a convex hull's vertex graph
        // is the global max, so this returns a true support vertex. steps cap guards
        // non-termination.
        const neighbors = hull.neighbors;
        const vertexCount = hull.vertexCount;
        let cur = hull.lastVertex;
        let curBase = cur * 3;
        let bestDot = vertices[curBase] * directionX + vertices[curBase + 1] * directionY + vertices[curBase + 2] * directionZ;
        let steps = 0;
        let prev: number;
        do {
            prev = cur;
            const end = neighborsStart[cur + 1];
            for (let k = neighborsStart[cur]; k < end; k++) {
                const n = neighbors[k];
                const nb = n * 3;
                const d = vertices[nb] * directionX + vertices[nb + 1] * directionY + vertices[nb + 2] * directionZ;
                if (d > bestDot) {
                    bestDot = d;
                    cur = n;
                }
            }
        } while (cur !== prev && ++steps <= vertexCount);
        hull.lastVertex = cur;
        curBase = cur * 3;
        supportX = vertices[curBase];
        supportY = vertices[curBase + 1];
        supportZ = vertices[curBase + 2];
    }

    // uniform-scale fast path: scale the winning local vertex (must precede addRadius + transform-back)
    const outputScale = hull.outputScale;
    supportX *= outputScale;
    supportY *= outputScale;
    supportZ *= outputScale;

    const radius = support.addRadius;
    if (radius > 0) {
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = radius / Math.sqrt(lengthSq);
            supportX += directionX * scale;
            supportY += directionY * scale;
            supportZ += directionZ * scale;
        }
    }
    if (support.hasTransform) {
        const m = support.transform;
        out[0] = m[0] * supportX + m[4] * supportY + m[8] * supportZ + m[12];
        out[1] = m[1] * supportX + m[5] * supportY + m[9] * supportZ + m[13];
        out[2] = m[2] * supportX + m[6] * supportY + m[10] * supportZ + m[14];
    } else {
        out[0] = supportX;
        out[1] = supportY;
        out[2] = supportZ;
    }
}

export function setBoxSupport(out: Support, shape: BoxShape, mode: SupportFunctionMode, scale: Vec3): void {
    const scaledX = Math.abs(scale[0]) * shape.halfExtents[0];
    const scaledY = Math.abs(scale[1]) * shape.halfExtents[1];
    const scaledZ = Math.abs(scale[2]) * shape.halfExtents[2];

    out.getSupport = boxSupport;
    out.hasTransform = false;
    out.addRadius = 0;

    const halfExtents = out.box.halfExtents;
    if (mode === SupportFunctionMode.EXCLUDE_CONVEX_RADIUS) {
        const minScale = Math.min(Math.abs(scale[0]), Math.abs(scale[1]), Math.abs(scale[2]));
        const scaledConvexRadius = Math.min(shape.convexRadius * minScale, DEFAULT_CONVEX_RADIUS);
        halfExtents[0] = Math.max(0, scaledX - scaledConvexRadius);
        halfExtents[1] = Math.max(0, scaledY - scaledConvexRadius);
        halfExtents[2] = Math.max(0, scaledZ - scaledConvexRadius);
        out.convexRadius = scaledConvexRadius;
    } else {
        halfExtents[0] = scaledX;
        halfExtents[1] = scaledY;
        halfExtents[2] = scaledZ;
        out.convexRadius = 0;
    }
}

export function setSphereSupport(out: Support, shape: SphereShape, mode: SupportFunctionMode, scale: Vec3): void {
    const absScale = Math.abs(scale[0]); // uniform scale only
    out.getSupport = sphereSupport;
    out.hasTransform = false;
    out.addRadius = 0;

    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS) {
        out.sphere.radius = shape.radius * absScale; // a sphere is a rounded point
        out.convexRadius = 0;
    } else {
        out.sphere.radius = 0; // core = origin, the radius is reported instead
        out.convexRadius = shape.radius * absScale;
    }
}

export function setCapsuleSupport(out: Support, shape: CapsuleShape, mode: SupportFunctionMode, scale: Vec3): void {
    const absScale = Math.abs(scale[0]); // uniform scale only
    const scaledHalfHeight = absScale * shape.halfHeightOfCylinder;
    const scaledRadius = absScale * shape.radius;

    out.getSupport = capsuleSupport;
    out.hasTransform = false;
    out.addRadius = 0;
    out.capsule.halfHeight = scaledHalfHeight;

    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS) {
        out.capsule.radius = scaledRadius; // a capsule is a rounded segment
        out.convexRadius = 0;
    } else {
        out.capsule.radius = 0; // segment only, the radius is reported instead
        out.convexRadius = scaledRadius;
    }
}

export function setCylinderSupport(out: Support, shape: CylinderShape, mode: SupportFunctionMode, scale: Vec3): void {
    const absScale = Math.abs(scale[0]); // uniform scale only
    out.getSupport = cylinderSupport;
    out.hasTransform = false;
    out.addRadius = 0;

    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS || mode === SupportFunctionMode.DEFAULT) {
        out.cylinder.halfHeight = absScale * shape.halfHeight;
        out.cylinder.radius = absScale * shape.radius;
        out.convexRadius = 0;
    } else {
        const scaledHalfHeight = absScale * shape.halfHeight;
        const scaledRadius = absScale * shape.radius;
        const scaledConvexRadius = absScale * shape.convexRadius;
        out.cylinder.halfHeight = scaledHalfHeight - scaledConvexRadius;
        out.cylinder.radius = scaledRadius - scaledConvexRadius;
        out.convexRadius = scaledConvexRadius;
    }
}

/** triangle operand (mesh) — copies the 3 verts */
export function setTriangleSupport(out: Support, a: Vec3, b: Vec3, c: Vec3): void {
    out.getSupport = triangleSupport;
    out.hasTransform = false;
    out.addRadius = 0;
    out.convexRadius = 0;
    const v = out.triangle.vertices;
    v[0] = a[0];
    v[1] = a[1];
    v[2] = a[2];
    v[3] = b[0];
    v[4] = b[1];
    v[5] = b[2];
    v[6] = c[0];
    v[7] = c[1];
    v[8] = c[2];
}

/** polygon face (KCC) — borrows the face's vertex array (read-only, valid for this pair) */
export function setPolygonSupport(out: Support, vertices: number[], vertexCount: number): void {
    out.getSupport = hullSupport; // a face is just a convex vertex set
    out.hasTransform = false;
    out.addRadius = 0;
    out.convexRadius = 0;
    out.hull.vertices = vertices; // read-only borrow, valid for this pair
    out.hull.vertexCount = vertexCount;
    out.hull.outputScale = 1;
    // a borrowed polygon face carries no adjacency: force the brute scan
    out.hull.neighborsStart = EMPTY_VERTICES;
    out.hull.neighbors = EMPTY_VERTICES;
    out.hull.lastVertex = -1;
}

/** point operand (collidePoint) — copies the point */
export function setPointSupport(out: Support, point: Vec3): void {
    out.getSupport = pointSupport;
    out.hasTransform = false;
    out.addRadius = 0;
    out.convexRadius = 0;
    out.point[0] = point[0];
    out.point[1] = point[1];
    out.point[2] = point[2];
}

/**
 * Compute the convex-radius-shrunk hull vertices (unscaled) into `dst` as a flat [x,y,z,...] array.
 * Each neighbouring face plane is offset inward by the convex radius (constant += r) and the up-to-3
 * planes are intersected (Cramer's rule). For a 2-face vertex the third plane is perpendicular to the
 * first two through the vertex; its `n1 × n2` normal is left unnormalized (the intersection is
 * invariant to per-plane scale).
 *
 * Computed once per shape at create time (see convex-hull.ts) and borrowed by the exclude-mode fill,
 * so `convexRadius` is passed in explicitly rather than read off the (not-yet-built) shape.
 */
export function computeShrunkHullPoints(
    shape: Pick<ConvexHullShape, 'numPoints' | 'pointPositions' | 'pointNumFaces' | 'pointFaces' | 'planes'>,
    convexRadius: number,
    dst: number[],
): void {
    const numPoints = shape.numPoints;
    const positions = shape.pointPositions;
    const numFacesArr = shape.pointNumFaces;
    const facesArr = shape.pointFaces;
    const planes = shape.planes;

    const requiredLength = numPoints * 3;
    while (dst.length < requiredLength) {
        dst.push(0);
    }

    let w = 0;
    for (let pi = 0; pi < numPoints; pi++) {
        const pb = pi * 3;
        const px = positions[pb];
        const py = positions[pb + 1];
        const pz = positions[pb + 2];
        const numFaces = numFacesArr[pi];

        // first neighbouring face plane (normal is unit; offset inward → constant + r)
        const plane1 = planes[facesArr[pb]];
        const nrm1 = plane1.normal;
        const n1x = nrm1[0];
        const n1y = nrm1[1];
        const n1z = nrm1[2];

        let rx: number;
        let ry: number;
        let rz: number;

        if (numFaces === 1) {
            // simple case: shift back along the single plane normal
            rx = px - n1x * convexRadius;
            ry = py - n1y * convexRadius;
            rz = pz - n1z * convexRadius;
        } else {
            const plane2 = planes[facesArr[pb + 1]];
            const nrm2 = plane2.normal;
            const n2x = nrm2[0];
            const n2y = nrm2[1];
            const n2z = nrm2[2];

            // offset the two face planes inward by the convex radius (use the stored plane constants)
            const d1 = plane1.constant + convexRadius;
            const d2 = plane2.constant + convexRadius;

            // third plane: 3rd face plane (offset inward), or a perpendicular plane through the vertex
            let n3x: number;
            let n3y: number;
            let n3z: number;
            let d3: number;
            if (numFaces === 3) {
                const plane3v = planes[facesArr[pb + 2]];
                const nrm3 = plane3v.normal;
                n3x = nrm3[0];
                n3y = nrm3[1];
                n3z = nrm3[2];
                d3 = plane3v.constant + convexRadius;
            } else {
                // third plane perpendicular to the first two, through the vertex (unnormalized normal)
                n3x = n1y * n2z - n1z * n2y;
                n3y = n1z * n2x - n1x * n2z;
                n3z = n1x * n2y - n1y * n2x;
                d3 = -(n3x * px + n3y * py + n3z * pz);
            }

            // intersect the three planes (Cramer's rule; the cross products are the adj columns)
            const c1x = n2y * n3z - n2z * n3y;
            const c1y = n2z * n3x - n2x * n3z;
            const c1z = n2x * n3y - n2y * n3x;
            const denom = n1x * c1x + n1y * c1y + n1z * c1z;

            if (Math.abs(denom) < 0.000001) {
                // near-parallel planes: fall back to pushing back along the first plane
                rx = px - n1x * convexRadius;
                ry = py - n1y * convexRadius;
                rz = pz - n1z * convexRadius;
            } else {
                const c2x = n3y * n1z - n3z * n1y;
                const c2y = n3z * n1x - n3x * n1z;
                const c2z = n3x * n1y - n3y * n1x;
                const c3x = n1y * n2z - n1z * n2y;
                const c3y = n1z * n2x - n1x * n2z;
                const c3z = n1x * n2y - n1y * n2x;
                const s = -1 / denom;
                rx = (d1 * c1x + d2 * c2x + d3 * c3x) * s;
                ry = (d1 * c1y + d2 * c2y + d3 * c3y) * s;
                rz = (d1 * c1z + d2 * c2z + d3 * c3z) * s;
            }
        }

        dst[w++] = rx;
        dst[w++] = ry;
        dst[w++] = rz;
    }
}

function scaleConvexRadius(radius: number, scale: Vec3): number {
    // use minimum absolute scale component
    const minScale = Math.min(Math.abs(scale[0]), Math.abs(scale[1]), Math.abs(scale[2]));
    return radius * minScale;
}

/**
 * Compute the scaled convex-radius-shrunk hull vertices into `dst` as a flat [x,y,z,...] array.
 * Positions are scaled, face-plane normals transformed by the inverse scale and renormalized, planes
 * rebuilt through the scaled vertex, offset inward by the scaled convex radius, then intersected.
 * The 2-face third plane uses the unnormalized cross of n1, n2.
 */
function computeScaledShrunkHullPoints(shape: ConvexHullShape, scale: Vec3, dst: number[]): void {
    const scaledRadius = scaleConvexRadius(shape.convexRadius, scale);
    const numPoints = shape.numPoints;
    const positions = shape.pointPositions;
    const numFacesArr = shape.pointNumFaces;
    const facesArr = shape.pointFaces;
    const planes = shape.planes;

    const requiredLength = numPoints * 3;
    while (dst.length < requiredLength) {
        dst.push(0);
    }

    const sx = scale[0];
    const sy = scale[1];
    const sz = scale[2];
    const isx = 1 / sx;
    const isy = 1 / sy;
    const isz = 1 / sz;

    let w = 0;
    for (let pi = 0; pi < numPoints; pi++) {
        const pb = pi * 3;
        // scaled vertex position
        const px = positions[pb] * sx;
        const py = positions[pb + 1] * sy;
        const pz = positions[pb + 2] * sz;
        const numFaces = numFacesArr[pi];

        // first face-plane normal, transformed by inverse scale and renormalized
        const m1 = planes[facesArr[pb]].normal;
        let n1x = m1[0] * isx;
        let n1y = m1[1] * isy;
        let n1z = m1[2] * isz;
        let l1 = n1x * n1x + n1y * n1y + n1z * n1z;
        if (l1 > 0) {
            l1 = 1 / Math.sqrt(l1);
            n1x *= l1;
            n1y *= l1;
            n1z *= l1;
        }

        let rx: number;
        let ry: number;
        let rz: number;

        if (numFaces === 1) {
            rx = px - n1x * scaledRadius;
            ry = py - n1y * scaledRadius;
            rz = pz - n1z * scaledRadius;
        } else {
            const m2 = planes[facesArr[pb + 1]].normal;
            let n2x = m2[0] * isx;
            let n2y = m2[1] * isy;
            let n2z = m2[2] * isz;
            let l2 = n2x * n2x + n2y * n2y + n2z * n2z;
            if (l2 > 0) {
                l2 = 1 / Math.sqrt(l2);
                n2x *= l2;
                n2y *= l2;
                n2z *= l2;
            }

            // planes rebuilt through the scaled vertex, offset inward by the scaled convex radius
            const d1 = -(n1x * px + n1y * py + n1z * pz) + scaledRadius;
            const d2 = -(n2x * px + n2y * py + n2z * pz) + scaledRadius;

            let n3x: number;
            let n3y: number;
            let n3z: number;
            let d3: number;
            if (numFaces === 3) {
                const m3 = planes[facesArr[pb + 2]].normal;
                let a = m3[0] * isx;
                let b = m3[1] * isy;
                let c = m3[2] * isz;
                let l3 = a * a + b * b + c * c;
                if (l3 > 0) {
                    l3 = 1 / Math.sqrt(l3);
                    a *= l3;
                    b *= l3;
                    c *= l3;
                }
                n3x = a;
                n3y = b;
                n3z = c;
                d3 = -(n3x * px + n3y * py + n3z * pz) + scaledRadius;
            } else {
                // third plane perpendicular to the first two, through the scaled vertex (unnormalized normal)
                n3x = n1y * n2z - n1z * n2y;
                n3y = n1z * n2x - n1x * n2z;
                n3z = n1x * n2y - n1y * n2x;
                d3 = -(n3x * px + n3y * py + n3z * pz);
            }

            const c1x = n2y * n3z - n2z * n3y;
            const c1y = n2z * n3x - n2x * n3z;
            const c1z = n2x * n3y - n2y * n3x;
            const denom = n1x * c1x + n1y * c1y + n1z * c1z;

            if (Math.abs(denom) < 0.000001) {
                rx = px - n1x * scaledRadius;
                ry = py - n1y * scaledRadius;
                rz = pz - n1z * scaledRadius;
            } else {
                const c2x = n3y * n1z - n3z * n1y;
                const c2y = n3z * n1x - n3x * n1z;
                const c2z = n3x * n1y - n3y * n1x;
                const c3x = n1y * n2z - n1z * n2y;
                const c3y = n1z * n2x - n1x * n2z;
                const c3z = n1x * n2y - n1y * n2x;
                const s = -1 / denom;
                rx = (d1 * c1x + d2 * c2x + d3 * c3x) * s;
                ry = (d1 * c1y + d2 * c2y + d3 * c3y) * s;
                rz = (d1 * c1z + d2 * c2z + d3 * c3z) * s;
            }
        }

        dst[w++] = rx;
        dst[w++] = ry;
        dst[w++] = rz;
    }
}

/**
 * Fill a HULL support for the given mode + scale. Include (or zero-radius) uses the raw vertices;
 * exclude uses the convex-radius-shrunk vertices. Uniform positive scale borrows the shape-owned
 * arrays and scales the support point in the evaluator (fast path); non-uniform / mirrored scale bakes
 * scaled vertices into scratch per pair (slow path). `vertices` is a read-only borrow valid for the
 * current pair.
 */
export function setHullSupport(out: Support, shape: ConvexHullShape, mode: SupportFunctionMode, scale: Vec3): void {
    out.getSupport = hullSupport;
    out.hasTransform = false;
    out.addRadius = 0;

    const hull = out.hull;
    hull.vertexCount = shape.numPoints;

    // borrow the shape's CSR adjacency (empty ⇔ not baked → brute scan). valid for the shrunk set (same
    // count/order) and every scale path (affine transforms preserve hull vertex adjacency). reset the
    // warm-start hint so this pair-fill starts cold (first evaluation brute-seeds it).
    hull.neighborsStart = shape.pointNeighborsStart;
    hull.neighbors = shape.pointNeighbors;
    hull.lastVertex = -1;

    // uniform positive scale (identity is just s=1): borrow the shape-owned vertex arrays and scale the
    // winning support point in the evaluator — exact, and no per-pair bake. non-uniform / mirrored scale:
    // fall back to baking scaled vertices into scratch per pair (slow path).
    const uniform = scale[0] === scale[1] && scale[1] === scale[2] && scale[0] > 0;

    if (uniform) {
        const s = scale[0];
        hull.outputScale = s;
        if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS || shape.convexRadius === 0) {
            out.convexRadius = 0;
            hull.vertices = shape.pointPositions; // read-only borrow, scaled via outputScale
        } else {
            out.convexRadius = shape.convexRadius * s;
            hull.vertices = shape.shrunkPointPositions; // read-only borrow, scaled via outputScale
        }
        return;
    }

    // slow path: non-uniform / mirrored scale bakes per-pair vertices into scratch
    hull.outputScale = 1;

    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS || shape.convexRadius === 0) {
        out.convexRadius = 0;
        // bake scaled raw vertices into scratch (same per-vertex scaling as withConvexScaled)
        const positions = shape.pointPositions;
        const scratch = hull.scratch;
        const requiredLength = shape.numPoints * 3;
        while (scratch.length < requiredLength) {
            scratch.push(0);
        }
        const sx = scale[0];
        const sy = scale[1];
        const sz = scale[2];
        for (let i = 0; i < requiredLength; i += 3) {
            scratch[i] = positions[i] * sx;
            scratch[i + 1] = positions[i + 1] * sy;
            scratch[i + 2] = positions[i + 2] * sz;
        }
        hull.vertices = scratch;
    } else {
        // EXCLUDE convex radius (nonzero) → scaled shrunk vertices
        computeScaledShrunkHullPoints(shape, scale, hull.scratch);
        out.convexRadius = scaleConvexRadius(shape.convexRadius, scale);
        hull.vertices = hull.scratch;
    }
}
