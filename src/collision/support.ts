import { type Mat4, mat4, type Vec3, vec3 } from 'mathcat';
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
 * Monomorphic support evaluation.
 *
 * A single {@link Support} struct (one hidden class) is filled once per collision pair, then
 * {@link getSupport} — a single, monomorphic function — is called many times per pair by GJK/EPA.
 * The per-shape polymorphism lives entirely in the fill (cold, once per pair); the hot path is one
 * function with a `switch` on `kind`.
 *
 * Radius contract:
 *  - `convexRadius` is the *reported* radius. `getSupport` never adds it; the collision driver
 *    reads it and passes it to `gjkClosestPoints`/EPA for the shrunk-core-plus-radius distance math.
 *  - `addRadius` is an extra radius added along the (local) direction by `getSupport` itself (the EPA
 *    speculative-separation / cast convex radius). 0 on the GJK path.
 *  - "mode" (include vs exclude convex radius) is baked into the params by the fill: exclude uses the
 *    shrunk core + reports `convexRadius`; include uses the full/rounded core + `convexRadius = 0`.
 */

export enum SupportKind {
    BOX,
    SPHERE,
    CAPSULE,
    CYLINDER,
    HULL,
    TRIANGLE,
    POINT,
}

export type Support = {
    /** which sub-object holds this support's parameters; selects the branch taken in {@link getSupport} */
    kind: SupportKind;

    /** reported convex radius — read by the driver, never added by getSupport (0 in include mode) */
    convexRadius: number;

    /** extra radius added along the local direction by getSupport (EPA separation / cast radius) */
    addRadius: number;

    /** B-in-A transform, applied when hasTransform is true (identity otherwise) */
    hasTransform: boolean;
    transform: Mat4;

    /** axis-aligned box: support is the corner picked by the sign of the direction on each axis (±halfExtents) */
    box: { halfExtents: Vec3 };

    /**
     * sphere as a single radius. 0 → the core is the origin (exclude mode: a sphere is pure convex radius);
     * r → the rounded surface point `r·dir̂` (include mode).
     */
    sphere: { radius: number };

    /**
     * capsule as a segment of half-length `halfHeight` along local Y, optionally rounded by `radius`.
     * radius 0 → the bare segment endpoint (exclude); r → segment endpoint + `r·dir̂` (include).
     */
    capsule: { halfHeight: number; radius: number };

    /**
     * cylinder: `radius` is the radial extent in the local XZ plane, `halfHeight` the axial extent along
     * local Y. support = the radial extreme (`radius·dir̂ₓ_z`) combined with the near/far axial cap.
     */
    cylinder: { radius: number; halfHeight: number };

    /** convex vertex set — support is the vertex with the greatest dot product against the direction */
    hull: {
        /** flat `[x,y,z,...]` vertices scanned by getSupport; read-only borrow valid for the current pair */
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

    /** triangle (mesh face) — support is whichever of the three vertices has the greatest dot with the direction */
    triangle: { a: Vec3; b: Vec3; c: Vec3 };

    /** single point — the support is always this point, regardless of direction */
    point: { position: Vec3 };
};

/**
 * Allocate a reusable {@link Support}. A driver holds a small fixed number of these (e.g. one per
 * operand slot) and refills them per pair via the fill functions. The sub-objects, transform, and
 * scratch buffer are pre-allocated so filling never allocates.
 */
export function createSupport(): Support {
    return {
        kind: SupportKind.SPHERE,
        convexRadius: 0,
        addRadius: 0,
        hasTransform: false,
        transform: mat4.create(),
        box: { halfExtents: vec3.create() },
        sphere: { radius: 0 },
        capsule: { halfHeight: 0, radius: 0 },
        cylinder: { radius: 0, halfHeight: 0 },
        hull: {
            vertices: EMPTY_VERTICES,
            vertexCount: 0,
            outputScale: 1,
            scratch: [],
            neighborsStart: EMPTY_VERTICES,
            neighbors: EMPTY_VERTICES,
            lastVertex: -1,
        },
        triangle: { a: vec3.create(), b: vec3.create(), c: vec3.create() },
        point: { position: vec3.create() },
    };
}

/**
 * Evaluate the support point of `support` in direction `direction`, writing it to `out`.
 * The single hot GJK/EPA call site — monomorphic.
 */
export function getSupport(out: Vec3, support: Support, direction: Vec3): void {
    // 1. transform the direction into the shape's local space (inverse rotation = transposed 3x3)
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

    // 2. core support in local space
    let supportX: number;
    let supportY: number;
    let supportZ: number;

    switch (support.kind) {
        case SupportKind.BOX: {
            const halfExtents = support.box.halfExtents;
            supportX = directionX >= 0 ? halfExtents[0] : -halfExtents[0];
            supportY = directionY >= 0 ? halfExtents[1] : -halfExtents[1];
            supportZ = directionZ >= 0 ? halfExtents[2] : -halfExtents[2];
            break;
        }
        case SupportKind.SPHERE: {
            // core is the origin (exclude); for include the radius produces r·dir̂
            const radius = support.sphere.radius;
            if (radius > 0) {
                const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
                if (lengthSq > 0) {
                    const scale = radius / Math.sqrt(lengthSq);
                    supportX = directionX * scale;
                    supportY = directionY * scale;
                    supportZ = directionZ * scale;
                } else {
                    supportX = 0;
                    supportY = 0;
                    supportZ = 0;
                }
            } else {
                supportX = 0;
                supportY = 0;
                supportZ = 0;
            }
            break;
        }
        case SupportKind.CAPSULE: {
            const capsule = support.capsule;
            const halfHeight = capsule.halfHeight;
            const radius = capsule.radius;
            if (radius > 0) {
                const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
                if (lengthSq > 0) {
                    const scale = radius / Math.sqrt(lengthSq);
                    supportX = directionX * scale;
                    supportY = directionY * scale + (directionY > 0 ? halfHeight : -halfHeight);
                    supportZ = directionZ * scale;
                } else {
                    supportX = 0;
                    supportY = halfHeight;
                    supportZ = 0;
                }
            } else {
                supportX = 0;
                supportY = directionY > 0 ? halfHeight : -halfHeight;
                supportZ = 0;
            }
            break;
        }
        case SupportKind.CYLINDER: {
            const cylinder = support.cylinder;
            const horizontalLen = Math.sqrt(directionX * directionX + directionZ * directionZ);
            if (horizontalLen > 0) {
                const scale = cylinder.radius / horizontalLen;
                supportX = directionX * scale;
                supportZ = directionZ * scale;
            } else {
                supportX = 0;
                supportZ = 0;
            }
            supportY = directionY >= 0 ? cylinder.halfHeight : -cylinder.halfHeight;
            break;
        }
        case SupportKind.HULL: {
            const hull = support.hull;
            const vertices = hull.vertices;
            const neighborsStart = hull.neighborsStart;
            supportX = 0;
            supportY = 0;
            supportZ = 0;

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
                let bestDot =
                    vertices[curBase] * directionX + vertices[curBase + 1] * directionY + vertices[curBase + 2] * directionZ;
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
            break;
        }
        case SupportKind.TRIANGLE: {
            const triangle = support.triangle;
            const a = triangle.a;
            const b = triangle.b;
            const c = triangle.c;
            const dotA = a[0] * directionX + a[1] * directionY + a[2] * directionZ;
            const dotB = b[0] * directionX + b[1] * directionY + b[2] * directionZ;
            const dotC = c[0] * directionX + c[1] * directionY + c[2] * directionZ;
            let best: Vec3;
            if (dotA > dotB) {
                best = dotA > dotC ? a : c;
            } else {
                best = dotB > dotC ? b : c;
            }
            supportX = best[0];
            supportY = best[1];
            supportZ = best[2];
            break;
        }
        default: {
            // POINT
            const position = support.point.position;
            supportX = position[0];
            supportY = position[1];
            supportZ = position[2];
            break;
        }
    }

    // 3. folded AddConvexRadius — add addRadius along the local direction
    if (support.addRadius > 0) {
        const lengthSq = directionX * directionX + directionY * directionY + directionZ * directionZ;
        if (lengthSq > 0) {
            const scale = support.addRadius / Math.sqrt(lengthSq);
            supportX += directionX * scale;
            supportY += directionY * scale;
            supportZ += directionZ * scale;
        }
    }

    // 4. transform the support point back to world space (rotation + translation)
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

/* -------------------------------------------------------------------------- */
/* setters — bake a shape (+ mode + scale) into a Support struct, once per pair. */
/* transform and addRadius default to identity/0; the driver sets them.       */
/* -------------------------------------------------------------------------- */

export function setBoxSupport(out: Support, shape: BoxShape, mode: SupportFunctionMode, scale: Vec3): void {
    const scaledX = Math.abs(scale[0]) * shape.halfExtents[0];
    const scaledY = Math.abs(scale[1]) * shape.halfExtents[1];
    const scaledZ = Math.abs(scale[2]) * shape.halfExtents[2];

    out.kind = SupportKind.BOX;
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
    out.kind = SupportKind.SPHERE;
    out.hasTransform = false;
    out.addRadius = 0;

    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS) {
        out.sphere.radius = shape.radius * absScale; // core = radius·dir̂
        out.convexRadius = 0;
    } else {
        out.sphere.radius = 0; // core = origin
        out.convexRadius = shape.radius * absScale;
    }
}

export function setCapsuleSupport(out: Support, shape: CapsuleShape, mode: SupportFunctionMode, scale: Vec3): void {
    const absScale = Math.abs(scale[0]); // uniform scale only
    const scaledHalfHeight = absScale * shape.halfHeightOfCylinder;
    const scaledRadius = absScale * shape.radius;

    out.kind = SupportKind.CAPSULE;
    out.hasTransform = false;
    out.addRadius = 0;
    out.capsule.halfHeight = scaledHalfHeight;

    if (mode === SupportFunctionMode.INCLUDE_CONVEX_RADIUS) {
        out.capsule.radius = scaledRadius; // segment + radius·dir̂
        out.convexRadius = 0;
    } else {
        out.capsule.radius = 0; // segment only
        out.convexRadius = scaledRadius;
    }
}

export function setCylinderSupport(out: Support, shape: CylinderShape, mode: SupportFunctionMode, scale: Vec3): void {
    const absScale = Math.abs(scale[0]); // uniform scale only
    out.kind = SupportKind.CYLINDER;
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
    out.kind = SupportKind.TRIANGLE;
    out.hasTransform = false;
    out.addRadius = 0;
    out.convexRadius = 0;
    const ta = out.triangle.a;
    const tb = out.triangle.b;
    const tc = out.triangle.c;
    ta[0] = a[0];
    ta[1] = a[1];
    ta[2] = a[2];
    tb[0] = b[0];
    tb[1] = b[1];
    tb[2] = b[2];
    tc[0] = c[0];
    tc[1] = c[1];
    tc[2] = c[2];
}

/** polygon face (KCC) — borrows the face's vertex array (read-only, valid for this pair) */
export function setPolygonSupport(out: Support, vertices: number[], vertexCount: number): void {
    out.kind = SupportKind.HULL; // a face is just a convex vertex set
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
    out.kind = SupportKind.POINT;
    out.hasTransform = false;
    out.addRadius = 0;
    out.convexRadius = 0;
    const position = out.point.position;
    position[0] = point[0];
    position[1] = point[1];
    position[2] = point[2];
}

/* -------------------------------------------------------------------------- */
/* convex hull — the fill computes the convex-radius-shrunk vertices (exclude mode) */
/* -------------------------------------------------------------------------- */

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
 * arrays and scales the support point in getSupport (fast path); non-uniform / mirrored scale bakes
 * scaled vertices into scratch per pair (slow path). `vertices` is a read-only borrow valid for the
 * current pair.
 */
export function setHullSupport(out: Support, shape: ConvexHullShape, mode: SupportFunctionMode, scale: Vec3): void {
    out.kind = SupportKind.HULL;
    out.hasTransform = false;
    out.addRadius = 0;

    const hull = out.hull;
    hull.vertexCount = shape.numPoints;

    // borrow the shape's CSR adjacency (empty ⇔ not baked → brute scan). valid for the shrunk set (same
    // count/order) and every scale path (affine transforms preserve hull vertex adjacency). reset the
    // warm-start hint so this pair-fill starts cold (first getSupport call brute-seeds it).
    hull.neighborsStart = shape.pointNeighborsStart;
    hull.neighbors = shape.pointNeighbors;
    hull.lastVertex = -1;

    // uniform positive scale (identity is just s=1): borrow the shape-owned vertex arrays and scale the
    // winning support point in getSupport — exact, and no per-pair bake. non-uniform / mirrored scale:
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
