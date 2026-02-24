import { type Quat, quat, type Vec3, vec3 } from 'mathcat';
import { MotionType } from './body/motion-type';
import type { RigidBody } from './body/rigid-body';
import { ConstraintType } from './constraints/constraint-id';
import type { ConeConstraint } from './constraints/cone-constraint';
import type { DistanceConstraint } from './constraints/distance-constraint';
import type { FixedConstraint } from './constraints/fixed-constraint';
import type { HingeConstraint } from './constraints/hinge-constraint';
import type { PointConstraint } from './constraints/point-constraint';
import type { SixDOFConstraint } from './constraints/six-dof-constraint';
import type { SliderConstraint } from './constraints/slider-constraint';
import type { SwingTwistConstraint } from './constraints/swing-twist-constraint';
import type { Shape } from './shapes/shapes';
import { ShapeType } from './shapes/shapes';
import type { World } from './world';

export enum BodyColorMode {
    MOTION_TYPE,
    INSTANCE,
    SLEEPING,
    ISLAND,
}

export type DebugRenderResult = {
    /** Flat XYZ positions for line segments. Every 6 floats = one line (two verts). */
    vertices: Float32Array;
    /** Flat RGB colors, one per vertex — same length as vertices. */
    colors: Float32Array;
    /** Number of line segments. vertices.length === numLines * 6. */
    numLines: number;
};

export type BodiesOptions = {
    colorMode: BodyColorMode;
    showLinearVelocity: boolean;
    showAngularVelocity: boolean;
};

export type ContactsOptions = Record<string, never>;

export type ContactConstraintsOptions = Record<string, never>;

export type JointsOptions = {
    /** Axis arm length. Default 0.5. */
    size: number;
    /** Draw angular / translation limit indicators. Default true. */
    drawLimits: boolean;
};

export function createBodiesOptions(): BodiesOptions {
    return { colorMode: BodyColorMode.MOTION_TYPE, showLinearVelocity: false, showAngularVelocity: false };
}

export function createContactsOptions(): ContactsOptions {
    return {};
}

export function createContactConstraintsOptions(): ContactConstraintsOptions {
    return {};
}

export function createJointsOptions(): JointsOptions {
    return { size: 0.5, drawLimits: true };
}

// ---------------------------------------------------------------------------
// Internal line buffer helpers
// ---------------------------------------------------------------------------

type LineBuffer = { v: number[]; c: number[] };

function createLineBuffer(): LineBuffer {
    return { v: [], c: [] };
}

function finalizeLineBuffer(out: LineBuffer): DebugRenderResult {
    const vertices = new Float32Array(out.v);
    const colors   = new Float32Array(out.c);
    return { vertices, colors, numLines: vertices.length / 6 };
}

function pushLine(
    out: LineBuffer,
    ax: number, ay: number, az: number,
    bx: number, by: number, bz: number,
    r: number, g: number, b: number,
): void {
    out.v.push(ax, ay, az, bx, by, bz);
    out.c.push(r, g, b, r, g, b);
}

/**
 * Circle in the plane defined by two perpendicular unit axes (u, v),
 * centred at (cx, cy, cz) with given radius.
 */
function pushCircle(
    out: LineBuffer,
    cx: number, cy: number, cz: number,
    ux: number, uy: number, uz: number,
    vx: number, vy: number, vz: number,
    radius: number, segments: number,
    r: number, g: number, b: number,
): void {
    const step = (2 * Math.PI) / segments;
    let prevX = cx + ux * radius;
    let prevY = cy + uy * radius;
    let prevZ = cz + uz * radius;
    for (let i = 1; i <= segments; i++) {
        const a = i * step;
        const cos = Math.cos(a);
        const sin = Math.sin(a);
        const nx = cx + (ux * cos + vx * sin) * radius;
        const ny = cy + (uy * cos + vy * sin) * radius;
        const nz = cz + (uz * cos + vz * sin) * radius;
        pushLine(out, prevX, prevY, prevZ, nx, ny, nz, r, g, b);
        prevX = nx; prevY = ny; prevZ = nz;
    }
}

/** 3-axis cross/marker at a point. */
function pushCross(
    out: LineBuffer,
    x: number, y: number, z: number,
    size: number,
    r: number, g: number, b: number,
): void {
    pushLine(out, x - size, y, z, x + size, y, z, r, g, b);
    pushLine(out, x, y - size, z, x, y + size, z, r, g, b);
    pushLine(out, x, y, z - size, x, y, z + size, r, g, b);
}

function bodyColor(body: RigidBody, mode: BodyColorMode): [number, number, number] {
    switch (mode) {
        case BodyColorMode.MOTION_TYPE:
            switch (body.motionType) {
                case MotionType.DYNAMIC:   return [0.2, 1.0, 0.2];  // green
                case MotionType.KINEMATIC: return [1.0, 1.0, 0.0];  // yellow
                case MotionType.STATIC:    return [0.2, 0.2, 1.0];  // blue
                default:                   return [1, 1, 1];
            }

        case BodyColorMode.INSTANCE: {
            const hash = body.id * 137.5;
            if (body.motionType === MotionType.STATIC) {
                const l = 0.22 + ((hash % 100) / 100) * 0.2;
                return hslToRgb(0, 0, l);
            }
            const hue = (hash % 360) / 360;
            return hslToRgb(hue, 0.7, 0.6);
        }

        case BodyColorMode.SLEEPING:
            return body.sleeping ? [0.2, 0.2, 1.0] : [1.0, 0.2, 0.2];

        case BodyColorMode.ISLAND: {
            const idx = body.islandIndex;
            if (idx === -1) return [1, 1, 1];
            return hslToRgb(((idx * 137.5) % 360) / 360, 0.8, 0.6);
        }
    }
}

function hslToRgb(h: number, s: number, l: number): [number, number, number] {
    if (s === 0) return [l, l, l];
    const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    const p = 2 * l - q;
    return [hue2rgb(p, q, h + 1 / 3), hue2rgb(p, q, h), hue2rgb(p, q, h - 1 / 3)];
}

function hue2rgb(p: number, q: number, t: number): number {
    if (t < 0) t += 1;
    if (t > 1) t -= 1;
    if (t < 1 / 6) return p + (q - p) * 6 * t;
    if (t < 1 / 2) return q;
    if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
    return p;
}

// persistent scratch vec for shape traversal
const _transformScratch  = /* @__PURE__ */ vec3.create();

/**
 * Compute a pair of unit axes perpendicular to `(nx, ny, nz)`.
 * Returns [ux, uy, uz, vx, vy, vz].
 */
function perpendicularAxes(nx: number, ny: number, nz: number): [number, number, number, number, number, number] {
    // pick an axis not parallel to n
    let ux: number, uy: number, uz: number;
    if (Math.abs(nx) < 0.9) { ux = 0; uy = -nz; uz = ny; }
    else                     { ux = -ny; uy = nx; uz = 0; }
    const ul = Math.sqrt(ux * ux + uy * uy + uz * uz);
    ux /= ul; uy /= ul; uz /= ul;
    // v = n × u
    const vx = ny * uz - nz * uy;
    const vy = nz * ux - nx * uz;
    const vz = nx * uy - ny * ux;
    return [ux, uy, uz, vx, vy, vz];
}

/** Transform local Vec3 into world space given a body position + quaternion. */
function transformPoint(out: Vec3, local: Vec3, pos: Vec3, q: Quat): void {
    vec3.transformQuat(_transformScratch, local, q);
    vec3.add(out, _transformScratch, pos);
}

/** Rotate local direction by quaternion into world space (no translation). */
function transformDir(out: Vec3, local: Vec3, q: Quat): void {
    vec3.transformQuat(out, local, q);
}

/**
 * Recursively emit wireframe lines for a shape and all its children,
 * accumulating the local→world transform as we descend.
 */
function drawShape(
    out: LineBuffer,
    shape: Shape,
    // world-space position and quaternion of this node
    px: number, py: number, pz: number,
    qx: number, qy: number, qz: number, qw: number,
    r: number, g: number, b: number,
): void {
    const q: Quat = [qx, qy, qz, qw];

    switch (shape.type) {

        case ShapeType.SPHERE: {
            const rad = shape.radius;
            // XY, XZ, YZ great circles
            pushCircle(out, px, py, pz,  1,0,0, 0,1,0, rad, 32, r, g, b);
            pushCircle(out, px, py, pz,  1,0,0, 0,0,1, rad, 32, r, g, b);
            pushCircle(out, px, py, pz,  0,1,0, 0,0,1, rad, 32, r, g, b);
            break;
        }

        case ShapeType.BOX: {
            const [hx, hy, hz] = shape.halfExtents;
            // transform all 8 corners into world space then draw 12 edges
            const corners: [number, number, number][] = [
                [-hx,-hy,-hz], [hx,-hy,-hz], [hx,-hy,hz], [-hx,-hy,hz],
                [-hx, hy,-hz], [hx, hy,-hz], [hx, hy,hz], [-hx, hy,hz],
            ];
            const wc = corners.map(([lx, ly, lz]) => {
                const v: Vec3 = [lx, ly, lz];
                const out = vec3.create();
                transformPoint(out, v, [px, py, pz], q);
                return out as [number, number, number];
            });
            const edges = [
                [0,1],[1,2],[2,3],[3,0],  // bottom
                [4,5],[5,6],[6,7],[7,4],  // top
                [0,4],[1,5],[2,6],[3,7],  // verticals
            ] as const;
            for (const [a, b] of edges) {
                pushLine(out, wc[a][0],wc[a][1],wc[a][2], wc[b][0],wc[b][1],wc[b][2], r,g,b);
            }
            break;
        }

        case ShapeType.CAPSULE: {
            const rad = shape.radius;
            const hh  = shape.halfHeightOfCylinder;
            // axis is local Y, rotated to world
            const localY: Vec3 = [0, 1, 0];
            const axisOut = vec3.create();
            transformDir(axisOut, localY, q);
            const [ax, ay, az] = axisOut;
            const [ux, uy, uz, vx, vy, vz] = perpendicularAxes(ax, ay, az);

            // top & bottom cylinder cap circles
            const topCx  = px + ax * hh;
            const topCy  = py + ay * hh;
            const topCz  = pz + az * hh;
            const botCx  = px - ax * hh;
            const botCy  = py - ay * hh;
            const botCz  = pz - az * hh;
            pushCircle(out, topCx, topCy, topCz, ux,uy,uz, vx,vy,vz, rad, 32, r,g,b);
            pushCircle(out, botCx, botCy, botCz, ux,uy,uz, vx,vy,vz, rad, 32, r,g,b);

            // 4 longitude arcs connecting caps (semicircles rotated 0°, 90°, 180°, 270° around axis)
            const arcAxes: [number, number, number][] = [
                [ux, uy, uz],
                [vx, vy, vz],
                [-ux, -uy, -uz],
                [-vx, -vy, -vz],
            ];
            for (const [arcU_x, arcU_y, arcU_z] of arcAxes) {
                // semicircle from top to bottom in the plane of (axis, arcU)
                const segs = 16;
                let prevX = topCx + arcU_x * rad;
                let prevY = topCy + arcU_y * rad;
                let prevZ = topCz + arcU_z * rad;
                for (let i = 1; i <= segs; i++) {
                    const angle = (Math.PI * i) / segs;
                    const t = angle / Math.PI; // 0..1
                    const cx2 = px + ax * (hh - 2 * hh * t) + arcU_x * rad * Math.sin(angle);
                    const cy2 = py + ay * (hh - 2 * hh * t) + arcU_y * rad * Math.sin(angle);
                    const cz2 = pz + az * (hh - 2 * hh * t) + arcU_z * rad * Math.sin(angle);
                    pushLine(out, prevX, prevY, prevZ, cx2, cy2, cz2, r, g, b);
                    prevX = cx2; prevY = cy2; prevZ = cz2;
                }
            }
            break;
        }

        case ShapeType.CYLINDER: {
            const rad = shape.radius;
            const hh  = shape.halfHeight;
            const localY: Vec3 = [0, 1, 0];
            const axisOut = vec3.create();
            transformDir(axisOut, localY, q);
            const [ax, ay, az] = axisOut;
            const [ux, uy, uz, vx, vy, vz] = perpendicularAxes(ax, ay, az);

            const topCx = px + ax * hh, topCy = py + ay * hh, topCz = pz + az * hh;
            const botCx = px - ax * hh, botCy = py - ay * hh, botCz = pz - az * hh;
            pushCircle(out, topCx, topCy, topCz, ux,uy,uz, vx,vy,vz, rad, 32, r,g,b);
            pushCircle(out, botCx, botCy, botCz, ux,uy,uz, vx,vy,vz, rad, 32, r,g,b);
            // 4 vertical lines
            for (const [cx, cy, cz] of [[ux,uy,uz],[-ux,-uy,-uz],[vx,vy,vz],[-vx,-vy,-vz]] as [number,number,number][]) {
                pushLine(out, 
                    topCx + cx * rad, topCy + cy * rad, topCz + cz * rad,
                    botCx + cx * rad, botCy + cy * rad, botCz + cz * rad,
                    r, g, b,
                );
            }
            break;
        }

        case ShapeType.PLANE: {
            // infinite plane: draw a grid scaled by halfExtent
            const [nx, ny, nz] = shape.plane.normal;
            const dist   = -shape.plane.constant;
            const size   = shape.halfExtent;
            const [ux, uy, uz, vx, vy, vz] = perpendicularAxes(nx, ny, nz);
            const ox = nx * dist, oy = ny * dist, oz = nz * dist;
            const steps = 5;
            for (let i = -steps; i <= steps; i++) {
                const t = (i / steps) * size;
                // lines along U direction
                pushLine(out, 
                    ox + ux * t - vx * size, oy + uy * t - vy * size, oz + uz * t - vz * size,
                    ox + ux * t + vx * size, oy + uy * t + vy * size, oz + uz * t + vz * size,
                    r, g, b,
                );
                // lines along V direction
                pushLine(out, 
                    ox + vx * t - ux * size, oy + vy * t - uy * size, oz + vz * t - uz * size,
                    ox + vx * t + ux * size, oy + vy * t + uy * size, oz + vz * t + uz * size,
                    r, g, b,
                );
            }
            break;
        }

        case ShapeType.CONVEX_HULL: {
            // emit all face edges (may duplicate, fine for debug use)
            for (const face of shape.faces) {
                const n = face.numVertices;
                for (let i = 0; i < n; i++) {
                    const ia = shape.vertexIndices[face.firstVertex + i];
                    const ib = shape.vertexIndices[face.firstVertex + (i + 1) % n];
                    const pa = shape.points[ia].position;
                    const pb = shape.points[ib].position;
                    const wa = vec3.create();
                    const wb = vec3.create();
                    transformPoint(wa, pa, [px, py, pz], q);
                    transformPoint(wb, pb, [px, py, pz], q);
                    pushLine(out, wa[0], wa[1], wa[2], wb[0], wb[1], wb[2], r, g, b);
                }
            }
            break;
        }

        case ShapeType.TRIANGLE_MESH: {
            const TRIANGLE_STRIDE = 8;
            for (let i = 0; i < shape.data.triangleCount; i++) {
                const off = i * TRIANGLE_STRIDE;
                const ia = shape.data.triangleBuffer[off + 0];
                const ib = shape.data.triangleBuffer[off + 1];
                const ic = shape.data.triangleBuffer[off + 2];
                const posArr = shape.data.positions;
                const pa: Vec3 = [posArr[ia * 3], posArr[ia * 3 + 1], posArr[ia * 3 + 2]];
                const pb: Vec3 = [posArr[ib * 3], posArr[ib * 3 + 1], posArr[ib * 3 + 2]];
                const pc: Vec3 = [posArr[ic * 3], posArr[ic * 3 + 1], posArr[ic * 3 + 2]];
                const wa = vec3.create(), wb = vec3.create(), wc2 = vec3.create();
                transformPoint(wa, pa, [px, py, pz], q);
                transformPoint(wb, pb, [px, py, pz], q);
                transformPoint(wc2, pc, [px, py, pz], q);
                pushLine(out, wa[0],wa[1],wa[2], wb[0],wb[1],wb[2], r,g,b);
                pushLine(out, wb[0],wb[1],wb[2], wc2[0],wc2[1],wc2[2], r,g,b);
                pushLine(out, wc2[0],wc2[1],wc2[2], wa[0],wa[1],wa[2], r,g,b);
            }
            break;
        }

        case ShapeType.COMPOUND:
        case ShapeType.STATIC_COMPOUND: {
            for (const child of shape.children) {
                const [lx, ly, lz] = child.position;
                const [cqx, cqy, cqz, cqw] = child.quaternion;
                // combine parent quat with child local quat
                const parentQ: Quat = [qx, qy, qz, qw];
                const childLocalQ: Quat = [cqx, cqy, cqz, cqw];
                const combined = quat.create();
                quat.multiply(combined, parentQ, childLocalQ);
                // rotate child local position by parent quat, add to parent world pos
                const localPos: Vec3 = [lx, ly, lz];
                const rotatedPos = vec3.create();
                vec3.transformQuat(rotatedPos, localPos, parentQ);
                drawShape(
                    out, child.shape,
                    px + rotatedPos[0], py + rotatedPos[1], pz + rotatedPos[2],
                    combined[0], combined[1], combined[2], combined[3],
                    r, g, b,
                );
            }
            break;
        }

        case ShapeType.TRANSFORMED: {
            const [lx, ly, lz] = shape.position;
            const [cqx, cqy, cqz, cqw] = shape.quaternion;
            const parentQ: Quat = [qx, qy, qz, qw];
            const childLocalQ: Quat = [cqx, cqy, cqz, cqw];
            const combined = quat.create();
            quat.multiply(combined, parentQ, childLocalQ);
            const localPos: Vec3 = [lx, ly, lz];
            const rotatedPos = vec3.create();
            vec3.transformQuat(rotatedPos, localPos, parentQ);
            drawShape(
                out, shape.shape,
                px + rotatedPos[0], py + rotatedPos[1], pz + rotatedPos[2],
                combined[0], combined[1], combined[2], combined[3],
                r, g, b,
            );
            break;
        }

        case ShapeType.SCALED: {
            // SCALED doesn't have its own shape type exposed cleanly to draw,
            // so just recurse and the leaf shape will apply scale from the vertex data.
            // For primitive shapes the scale is baked into the half-extents/radius of the inner shape,
            // so we pass it through unchanged. For mesh shapes, vertex positions are pre-scaled.
            drawShape(out, shape.shape, px, py, pz, qx, qy, qz, qw, r, g, b);
            break;
        }

        case ShapeType.OFFSET_CENTER_OF_MASS: {
            // offset only affects COM, not geometry position
            drawShape(out, shape.shape, px, py, pz, qx, qy, qz, qw, r, g, b);
            break;
        }

        // EMPTY: nothing
    }
}

type BodyTransform = {
    centerOfMassPosition: Vec3;
    quaternion: Quat;
};

const _constraintPointA  = /* @__PURE__ */ vec3.create();
const _constraintPointB = /* @__PURE__ */ vec3.create();
const _constraintDirA  = /* @__PURE__ */ vec3.create();
const _constraintDirB = /* @__PURE__ */ vec3.create();

function transformPointToWorld(out: Vec3, local: Vec3, xf: BodyTransform): void {
    vec3.transformQuat(out, local, xf.quaternion);
    vec3.add(out, out, xf.centerOfMassPosition);
}

function transformDirectionToWorld(out: Vec3, local: Vec3, xf: BodyTransform): void {
    vec3.transformQuat(out, local, xf.quaternion);
}

function pushConstraintMarker(out: LineBuffer, pos: Vec3, size: number, r: number, g: number, b: number): void {
    pushCross(out, pos[0], pos[1], pos[2], size, r, g, b);
}

function pushConstraintLine(out: LineBuffer, from: Vec3, to: Vec3, r: number, g: number, b: number): void {
    pushLine(out, from[0], from[1], from[2], to[0], to[1], to[2], r, g, b);
}

const _pieRotationQuat = /* @__PURE__ */ quat.create();
const _pieRotatedAxis  = /* @__PURE__ */ vec3.create();

function drawPie(
    out: LineBuffer,
    center: Vec3,
    radius: number,
    normal: Vec3,
    axis: Vec3,
    minAngle: number,
    maxAngle: number,
    r: number, g: number, b: number,
): void {
    if (minAngle >= maxAngle) return;
    const segs = 32;
    const step = (maxAngle - minAngle) / segs;
    let prevX = 0, prevY = 0, prevZ = 0;
    let hasPrev = false;
    for (let i = 0; i <= segs; i++) {
        const angle = minAngle + i * step;
        quat.setAxisAngle(_pieRotationQuat, normal, angle);
        vec3.transformQuat(_pieRotatedAxis, axis, _pieRotationQuat);
        const px = center[0] + _pieRotatedAxis[0] * radius;
        const py = center[1] + _pieRotatedAxis[1] * radius;
        const pz = center[2] + _pieRotatedAxis[2] * radius;
        // radial spoke from centre
        pushLine(out, center[0], center[1], center[2], px, py, pz, r, g, b);
        if (hasPrev) {
            pushLine(out, prevX, prevY, prevZ, px, py, pz, r, g, b);
        }
        prevX = px; prevY = py; prevZ = pz;
        hasPrev = true;
    }
}

function drawSwingConeLimits(
    out: LineBuffer,
    center: Vec3,
    twistAxis: Vec3, planeAxis: Vec3, normalAxis: Vec3,
    planeHalfAngle: number, normalHalfAngle: number,
    radius: number,
    r: number, g: number, b: number,
): void {
    const maxAngle = Math.max(planeHalfAngle, normalHalfAngle);
    if (maxAngle <= 0) return;
    const segs = 32;
    const step = (2 * Math.PI) / segs;
    let prevX = 0, prevY = 0, prevZ = 0;
    let hasPrev = false;
    for (let i = 0; i <= segs; i++) {
        const theta = i * step;
        const cosT = Math.cos(theta);
        const sinT = Math.sin(theta);
        const cosM = Math.cos(maxAngle);
        const sinM = Math.sin(maxAngle);
        const dx = twistAxis[0] * cosM + (planeAxis[0] * cosT + normalAxis[0] * sinT) * sinM;
        const dy = twistAxis[1] * cosM + (planeAxis[1] * cosT + normalAxis[1] * sinT) * sinM;
        const dz = twistAxis[2] * cosM + (planeAxis[2] * cosT + normalAxis[2] * sinT) * sinM;
        const px = center[0] + dx * radius;
        const py = center[1] + dy * radius;
        const pz = center[2] + dz * radius;
        if (i % 4 === 0) pushLine(out, center[0], center[1], center[2], px, py, pz, r, g, b);
        if (hasPrev) pushLine(out, prevX, prevY, prevZ, px, py, pz, r, g, b);
        prevX = px; prevY = py; prevZ = pz;
        hasPrev = true;
    }
}

/**
 * Render body wireframes.
 *
 * @example
 * const { vertices, colors } = debug.bodies(world);
 */
export function bodies(world: World, options?: Partial<BodiesOptions>): DebugRenderResult {
    const opts: BodiesOptions = { ...createBodiesOptions(), ...options };
    const out = createLineBuffer();

    for (const body of world.bodies.pool) {
        if ((body as unknown as { _pooled: boolean })._pooled) continue;

        const [r, g, b] = bodyColor(body, opts.colorMode);
        const [px, py, pz] = body.position;
        const [qx, qy, qz, qw] = body.quaternion;
        drawShape(out, body.shape, px, py, pz, qx, qy, qz, qw, r, g, b);

        if (opts.showLinearVelocity && body.motionType !== MotionType.STATIC) {
            const mp = body.motionProperties;
            const [vx, vy, vz] = mp.linearVelocity;
            const mag = Math.sqrt(vx * vx + vy * vy + vz * vz);
            if (mag > 0.01) {
                const len = Math.min(mag, 5.0);
                const inv = 1 / mag;
                const [cx, cy, cz] = body.centerOfMassPosition;
                pushLine(out, cx, cy, cz, cx + vx * inv * len, cy + vy * inv * len, cz + vz * inv * len, 1, 1, 0);
            }
        }

        if (opts.showAngularVelocity && body.motionType !== MotionType.STATIC) {
            const mp = body.motionProperties;
            const [vx, vy, vz] = mp.angularVelocity;
            const mag = Math.sqrt(vx * vx + vy * vy + vz * vz);
            if (mag > 0.01) {
                const len = Math.min(mag * 0.5, 3.0);
                const inv = 1 / mag;
                const [cx, cy, cz] = body.centerOfMassPosition;
                pushLine(out, cx, cy, cz, cx + vx * inv * len, cy + vy * inv * len, cz + vz * inv * len, 1, 0.5, 0);
            }
        }
    }

    return finalizeLineBuffer(out);
}

/**
 * Render contact points and normals.
 *
 * @example
 * const { vertices, colors } = debug.contacts(world);
 */
export function contacts(world: World, _options?: Partial<ContactsOptions>): DebugRenderResult {
    const out = createLineBuffer();

    for (let i = 0; i < world.contactConstraints.count; i++) {
        const constraint = world.contactConstraints.pool[i];
        if (constraint.numContactPoints === 0) continue;

        const [nx, ny, nz] = constraint.normal;

        for (let j = 0; j < constraint.numContactPoints; j++) {
            const cp = constraint.contactPoints[j];
            const [ax, ay, az] = cp.positionA;

            // cross at contact point A — red
            pushCross(out, ax, ay, az, 0.05, 1, 0, 0);

            // normal line from contact point A — magenta
            pushLine(out, ax, ay, az, ax + nx * 0.15, ay + ny * 0.15, az + nz * 0.15, 1, 0, 1);
        }
    }

    return finalizeLineBuffer(out);
}

/**
 * Render detailed contact constraint manifolds (points, edges, normal, tangents).
 *
 * @example
 * const { vertices, colors } = debug.contactConstraints(world);
 */
export function contactConstraints(world: World, _options?: Partial<ContactConstraintsOptions>): DebugRenderResult {
    const out = createLineBuffer();

    for (let i = 0; i < world.contactConstraints.count; i++) {
        const constraint = world.contactConstraints.pool[i];
        if (constraint.numContactPoints === 0) continue;

        const [nx, ny, nz] = constraint.normal;
        const [t1x, t1y, t1z] = constraint.tangent1;
        const [t2x, t2y, t2z] = constraint.tangent2;

        const firstPt = constraint.contactPoints[0];
        const [fpx, fpy, fpz] = firstPt.positionA;

        let prevAx = 0, prevAy = 0, prevAz = 0;
        let hasPrev = false;

        for (let j = 0; j < constraint.numContactPoints; j++) {
            const cp = constraint.contactPoints[j];
            const [ax, ay, az] = cp.positionA;
            const [bx, by, bz] = cp.positionB;

            const hasImpulse =
                cp.normalConstraint.totalLambda !== 0 ||
                cp.tangentConstraint1.totalLambda !== 0 ||
                cp.tangentConstraint2.totalLambda !== 0;

            // point A — cyan (bright if active)
            const ar = hasImpulse ? 0 : 0;
            const ag = hasImpulse ? 1 : 0.4;
            const ab = hasImpulse ? 1 : 0.6;
            pushCross(out, ax, ay, az, hasImpulse ? 0.04 : 0.02, ar, ag, ab);

            // point B — magenta
            const br2 = hasImpulse ? 1 : 0.5;
            const bg2 = hasImpulse ? 0 : 0;
            const bb2 = hasImpulse ? 1 : 0.5;
            pushCross(out, bx, by, bz, hasImpulse ? 0.04 : 0.02, br2, bg2, bb2);

            // edge between consecutive A points — yellow
            if (hasPrev) {
                pushLine(out, prevAx, prevAy, prevAz, ax, ay, az, 0.8, 0.8, 0);
            }
            prevAx = ax; prevAy = ay; prevAz = az;
            hasPrev = true;
        }

        // close loop if 3+ points
        if (constraint.numContactPoints > 2) {
            const [ax, ay, az] = constraint.contactPoints[0].positionA;
            pushLine(out, prevAx, prevAy, prevAz, ax, ay, az, 0.8, 0.8, 0);
        }

        // normal arrow from first point — magenta
        const normalLen = 0.3;
        pushLine(out, fpx, fpy, fpz, fpx + nx * normalLen, fpy + ny * normalLen, fpz + nz * normalLen, 1, 0, 1);

        // tangent1 — green
        const tangentLen = 0.2;
        pushLine(out, fpx, fpy, fpz, fpx + t1x * tangentLen, fpy + t1y * tangentLen, fpz + t1z * tangentLen, 0, 1, 0);

        // tangent2 — blue
        pushLine(out, fpx, fpy, fpz, fpx + t2x * tangentLen, fpy + t2y * tangentLen, fpz + t2z * tangentLen, 0, 0.4, 1);
    }

    return finalizeLineBuffer(out);
}

const _worldXAxis: Vec3 = /* @__PURE__ */ [1, 0, 0];
const _worldYAxis: Vec3 = /* @__PURE__ */ [0, 1, 0];
const _worldZAxis: Vec3 = /* @__PURE__ */ [0, 0, 1];
const _constraintNormalAxis  = /* @__PURE__ */ vec3.create();
const _constraintSpaceQuat     = /* @__PURE__ */ quat.create();

/**
 * Render joint/constraint axes and limits.
 *
 * @example
 * const { vertices, colors } = debug.joints(world);
 */
export function joints(world: World, options?: Partial<JointsOptions>): DebugRenderResult {
    const opts: JointsOptions = { ...createJointsOptions(), ...options };
    const out = createLineBuffer();
    const { size, drawLimits } = opts;

    const pool = world.bodies.pool;

    function getXf(bodyIndex: number): BodyTransform | null {
        const body = pool[bodyIndex] as (RigidBody & { _pooled: boolean }) | undefined;
        if (!body || body._pooled) return null;
        return { centerOfMassPosition: body.centerOfMassPosition, quaternion: body.quaternion };
    }

    // Hinge
    const hingePool = world.constraints.pools[ConstraintType.HINGE];
    if (hingePool) {
        for (const constraint of hingePool.constraints as HingeConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePosition1, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePosition2, xfB);
            transformDirectionToWorld(_constraintDirA,  constraint.localSpaceHingeAxis1, xfA);

            pushConstraintMarker(out, _constraintPointA, size * 0.1, 1, 0, 0);
            pushLine(out, 
                _constraintPointA[0], _constraintPointA[1], _constraintPointA[2],
                _constraintPointA[0] + _constraintDirA[0] * size, _constraintPointA[1] + _constraintDirA[1] * size, _constraintPointA[2] + _constraintDirA[2] * size,
                1, 0, 0,
            );
            pushConstraintMarker(out, _constraintPointB, size * 0.1, 0, 1, 0);

            // normal axis ref line + optional pie limits
            vec3.transformQuat(_constraintNormalAxis, constraint.localSpaceNormalAxis1 as Vec3, xfA.quaternion);
            pushLine(out, 
                _constraintPointB[0], _constraintPointB[1], _constraintPointB[2],
                _constraintPointB[0] + _constraintNormalAxis[0] * size, _constraintPointB[1] + _constraintNormalAxis[1] * size, _constraintPointB[2] + _constraintNormalAxis[2] * size,
                1, 1, 1,
            );
            if (drawLimits && (constraint as HingeConstraint & { hasLimits?: boolean }).hasLimits) {
                const c = constraint as HingeConstraint & { limitsMin: number; limitsMax: number };
                if (c.limitsMax > c.limitsMin) {
                    drawPie(out, _constraintPointA, size, _constraintDirA, _constraintNormalAxis, c.limitsMin, c.limitsMax, 0.5, 0, 0.5);
                }
            }
        }
    }

    // Swing-Twist
    const swingTwistPool = world.constraints.pools[ConstraintType.SWING_TWIST];
    if (swingTwistPool) {
        for (const constraint of swingTwistPool.constraints as SwingTwistConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePosition1, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePosition2, xfB);

            pushConstraintMarker(out, _constraintPointA,  size * 0.1, 1, 0, 0);
            pushConstraintMarker(out, _constraintPointB, size * 0.1, 0, 1, 0);
            pushConstraintLine(out, _constraintPointA, _constraintPointB, 1, 1, 1);

            if (drawLimits) {
                const c2b1 = constraint.constraintToBody1 as Quat;
                quat.multiply(_constraintSpaceQuat, xfA.quaternion, c2b1);
                const ctf: BodyTransform = { centerOfMassPosition: xfA.centerOfMassPosition, quaternion: _constraintSpaceQuat };
                const twistAxis = vec3.create(), planeAxis = vec3.create(), normalAxis = vec3.create();
                transformDirectionToWorld(twistAxis,  _worldXAxis, ctf);
                transformDirectionToWorld(planeAxis,  _worldYAxis, ctf);
                transformDirectionToWorld(normalAxis, _worldZAxis, ctf);

                const c = constraint as SwingTwistConstraint & {
                    planeHalfConeAngle: number; normalHalfConeAngle: number;
                    twistMinAngle: number; twistMaxAngle: number;
                };
                if (c.planeHalfConeAngle > 0 || c.normalHalfConeAngle > 0) {
                    drawSwingConeLimits(out, _constraintPointA, twistAxis, planeAxis, normalAxis,
                        c.planeHalfConeAngle, c.normalHalfConeAngle, size, 0, 1, 0);
                }
                if (c.twistMaxAngle > c.twistMinAngle) {
                    drawPie(out, _constraintPointA, size, twistAxis, planeAxis, c.twistMinAngle, c.twistMaxAngle, 0.5, 0, 0.5);
                }
            }
        }
    }

    // Distance
    const distPool = world.constraints.pools[ConstraintType.DISTANCE];
    if (distPool) {
        for (const constraint of distPool.constraints as DistanceConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePosition1, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePosition2, xfB);
            pushConstraintLine(out, _constraintPointA, _constraintPointB, 0, 1, 0);

            if (drawLimits) {
                const c = constraint as DistanceConstraint & { minDistance: number; maxDistance: number };
                const dx = _constraintPointB[0] - _constraintPointA[0];
                const dy = _constraintPointB[1] - _constraintPointA[1];
                const dz = _constraintPointB[2] - _constraintPointA[2];
                const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
                if (dist > 0.001) {
                    const inv = 1 / dist;
                    const [dirX, dirY, dirZ] = [dx * inv, dy * inv, dz * inv];
                    const [ux, uy, uz, vx, vy, vz] = perpendicularAxes(dirX, dirY, dirZ);
                    if (c.minDistance > 0) {
                        pushCircle(out, 
                            _constraintPointA[0], _constraintPointA[1], _constraintPointA[2],
                            ux, uy, uz, vx, vy, vz,
                            c.minDistance, 16, 0, 0.4, 1,
                        );
                    }
                    if (c.maxDistance < Infinity) {
                        pushCircle(out, 
                            _constraintPointA[0], _constraintPointA[1], _constraintPointA[2],
                            ux, uy, uz, vx, vy, vz,
                            c.maxDistance, 16, 1, 0.2, 0.2,
                        );
                    }
                }
            }
        }
    }

    // Cone
    const conePool = world.constraints.pools[ConstraintType.CONE];
    if (conePool) {
        for (const constraint of conePool.constraints as ConeConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePosition1, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePosition2, xfB);
            transformDirectionToWorld(_constraintDirA,  constraint.localSpaceTwistAxis1 as Vec3, xfA);
            transformDirectionToWorld(_constraintDirB, constraint.localSpaceTwistAxis2 as Vec3, xfB);

            pushConstraintMarker(out, _constraintPointA,  size * 0.1, 1, 0, 0);
            pushLine(out, _constraintPointA[0],_constraintPointA[1],_constraintPointA[2], _constraintPointA[0]+_constraintDirA[0]*size,_constraintPointA[1]+_constraintDirA[1]*size,_constraintPointA[2]+_constraintDirA[2]*size, 1,0,0);
            pushConstraintMarker(out, _constraintPointB, size * 0.1, 0, 1, 0);
            pushLine(out, _constraintPointB[0],_constraintPointB[1],_constraintPointB[2], _constraintPointB[0]+_constraintDirB[0]*size,_constraintPointB[1]+_constraintDirB[1]*size,_constraintPointB[2]+_constraintDirB[2]*size, 0,1,0);

            if (drawLimits) {
                const c = constraint as ConeConstraint & { cosHalfConeAngle: number };
                if (c.cosHalfConeAngle < 1) {
                    const halfAngle = Math.acos(c.cosHalfConeAngle);
                    const [ux, uy, uz, vx, vy, vz] = perpendicularAxes(_constraintDirA[0], _constraintDirA[1], _constraintDirA[2]);
                    const pA: Vec3 = [ux, uy, uz];
                    const pB: Vec3 = [vx, vy, vz];
                    drawSwingConeLimits(out, _constraintPointA, _constraintDirA, pA, pB, halfAngle, halfAngle, size, 1, 1, 0);
                }
            }
        }
    }

    // Fixed
    const fixedPool = world.constraints.pools[ConstraintType.FIXED];
    if (fixedPool) {
        for (const constraint of fixedPool.constraints as FixedConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePosition1, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePosition2, xfB);
            pushConstraintMarker(out, _constraintPointA,  size * 0.1, 0, 0, 1);
            pushConstraintMarker(out, _constraintPointB, size * 0.1, 0, 0, 1);
        }
    }

    // Point
    const pointPool = world.constraints.pools[ConstraintType.POINT];
    if (pointPool) {
        for (const constraint of pointPool.constraints as PointConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePosition1, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePosition2, xfB);
            pushConstraintMarker(out, _constraintPointA,  size * 0.1, 1, 1, 1);
            pushConstraintMarker(out, _constraintPointB, size * 0.1, 1, 1, 1);
            pushConstraintLine(out, _constraintPointA, _constraintPointB, 1, 1, 1);
        }
    }

    // Slider
    const sliderPool = world.constraints.pools[ConstraintType.SLIDER];
    if (sliderPool) {
        for (const constraint of sliderPool.constraints as SliderConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePositionA, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePositionB, xfB);
            transformDirectionToWorld(_constraintDirA,  constraint.localSpaceSliderAxisA as Vec3, xfA);

            pushConstraintMarker(out, _constraintPointA,  size * 0.1, 1, 0, 0);
            pushLine(out, _constraintPointA[0],_constraintPointA[1],_constraintPointA[2], _constraintPointA[0]+_constraintDirA[0]*size,_constraintPointA[1]+_constraintDirA[1]*size,_constraintPointA[2]+_constraintDirA[2]*size, 1,0,0);
            pushConstraintMarker(out, _constraintPointB, size * 0.1, 0, 1, 0);

            if (drawLimits) {
                const c = constraint as SliderConstraint & { hasLimits: boolean; limitsMin: number; limitsMax: number };
                if (c.hasLimits) {
                    if (c.limitsMin > -Infinity) {
                        const mp: Vec3 = [
                            _constraintPointA[0] + _constraintDirA[0] * c.limitsMin,
                            _constraintPointA[1] + _constraintDirA[1] * c.limitsMin,
                            _constraintPointA[2] + _constraintDirA[2] * c.limitsMin,
                        ];
                        pushConstraintMarker(out, mp, size * 0.15, 0, 0, 1);
                    }
                    if (c.limitsMax < Infinity) {
                        const mp: Vec3 = [
                            _constraintPointA[0] + _constraintDirA[0] * c.limitsMax,
                            _constraintPointA[1] + _constraintDirA[1] * c.limitsMax,
                            _constraintPointA[2] + _constraintDirA[2] * c.limitsMax,
                        ];
                        pushConstraintMarker(out, mp, size * 0.15, 1, 0, 0);
                    }
                    if (c.limitsMin > -Infinity && c.limitsMax < Infinity) {
                        const mn: Vec3 = [_constraintPointA[0]+_constraintDirA[0]*c.limitsMin, _constraintPointA[1]+_constraintDirA[1]*c.limitsMin, _constraintPointA[2]+_constraintDirA[2]*c.limitsMin];
                        const mx: Vec3 = [_constraintPointA[0]+_constraintDirA[0]*c.limitsMax, _constraintPointA[1]+_constraintDirA[1]*c.limitsMax, _constraintPointA[2]+_constraintDirA[2]*c.limitsMax];
                        pushConstraintLine(out, mn, mx, 0.5, 0, 0.5);
                    }
                }
            }
        }
    }

    // Six DOF
    const sixDOFPool = world.constraints.pools[ConstraintType.SIX_DOF];
    if (sixDOFPool) {
        for (const constraint of sixDOFPool.constraints as SixDOFConstraint[]) {
            if ((constraint as unknown as { _pooled: boolean })._pooled || !constraint.enabled) continue;
            const xfA = getXf(constraint.bodyIndexA);
            const xfB = getXf(constraint.bodyIndexB);
            if (!xfA || !xfB) continue;

            transformPointToWorld(_constraintPointA,  constraint.localSpacePosition1, xfA);
            transformPointToWorld(_constraintPointB, constraint.localSpacePosition2, xfB);
            pushConstraintMarker(out, _constraintPointA,  size * 0.1, 0.5, 0, 0.5);
            pushConstraintMarker(out, _constraintPointB, size * 0.1, 0.5, 0, 0.5);

            if (drawLimits) {
                const c2b1 = constraint.constraintToBody1 as Quat;
                quat.multiply(_constraintSpaceQuat, xfA.quaternion, c2b1);
                const ctf: BodyTransform = { centerOfMassPosition: xfA.centerOfMassPosition, quaternion: _constraintSpaceQuat };
                const axisX = vec3.create(), axisY = vec3.create(), axisZ = vec3.create();
                transformDirectionToWorld(axisX, _worldXAxis, ctf);
                transformDirectionToWorld(axisY, _worldYAxis, ctf);
                transformDirectionToWorld(axisZ, _worldZAxis, ctf);

                const axes  = [axisX, axisY, axisZ];
                const axClr: [number, number, number][] = [[1,0,0],[0,1,0],[0,0,1]];

                const c = constraint as SixDOFConstraint & { limitMin: number[]; limitMax: number[] };
                const BIG = 3.4e38;

                for (let i = 0; i < 3; i++) {
                    const minL = c.limitMin[i], maxL = c.limitMax[i];
                    const axis = axes[i], [cr, cg, cb] = axClr[i];
                    const isFree  = minL <= -BIG && maxL >= BIG;
                    const isFixed = minL >= BIG  && maxL <= -BIG;
                    if (!isFree && !isFixed && minL < maxL) {
                        if (minL > -BIG) {
                            const mp: Vec3 = [_constraintPointA[0]+axis[0]*minL, _constraintPointA[1]+axis[1]*minL, _constraintPointA[2]+axis[2]*minL];
                            pushConstraintMarker(out, mp, size * 0.08, cr, cg, cb);
                        }
                        if (maxL < BIG) {
                            const mp: Vec3 = [_constraintPointA[0]+axis[0]*maxL, _constraintPointA[1]+axis[1]*maxL, _constraintPointA[2]+axis[2]*maxL];
                            pushConstraintMarker(out, mp, size * 0.08, cr, cg, cb);
                        }
                        if (minL > -BIG && maxL < BIG) {
                            const mn: Vec3 = [_constraintPointA[0]+axis[0]*minL, _constraintPointA[1]+axis[1]*minL, _constraintPointA[2]+axis[2]*minL];
                            const mx: Vec3 = [_constraintPointA[0]+axis[0]*maxL, _constraintPointA[1]+axis[1]*maxL, _constraintPointA[2]+axis[2]*maxL];
                            pushConstraintLine(out, mn, mx, cr, cg, cb);
                        }
                    }
                }

                for (let i = 0; i < 3; i++) {
                    const minL = c.limitMin[3 + i], maxL = c.limitMax[3 + i];
                    const axis = axes[i], [cr, cg, cb] = axClr[i];
                    const perpAxis = axes[(i + 1) % 3];
                    const isFree  = minL <= -BIG && maxL >= BIG;
                    const isFixed = minL >= BIG  && maxL <= -BIG;
                    if (!isFree && !isFixed && minL < maxL && Math.abs(maxL - minL) > 0.001) {
                        drawPie(out, _constraintPointA, size * 0.5, axis, perpAxis, minL, maxL, cr, cg, cb);
                    }
                }
            }
        }
    }

    return finalizeLineBuffer(out);
}
