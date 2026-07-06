import type { RigidBody } from './body/rigid-body.js';
import type { Shape } from './shapes/shapes.js';
import type { World } from './world.js';
export declare enum BodyColorMode {
    MOTION_TYPE = 0,
    INSTANCE = 1,
    SLEEPING = 2,
    ISLAND = 3
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
export type BodyOptions = {
    /** explicit [r,g,b] color override. when omitted, bodyColor() is used based on colorMode */
    color: [number, number, number] | null;
    colorMode: BodyColorMode;
    showLinearVelocity: boolean;
    showAngularVelocity: boolean;
};
export type ShapeOptions = {
    /** explicit [r,g,b] color. defaults to white [1,1,1] */
    color: [number, number, number];
};
export type ContactsOptions = Record<string, never>;
export type ContactConstraintsOptions = Record<string, never>;
export type JointsOptions = {
    /** Axis arm length. Default 0.5. */
    size: number;
    /** Draw angular / translation limit indicators. Default true. */
    drawLimits: boolean;
};
export declare function createBodiesOptions(): BodiesOptions;
export declare function createBodyOptions(): BodyOptions;
export declare function createShapeOptions(): ShapeOptions;
export declare function createContactsOptions(): ContactsOptions;
export declare function createContactConstraintsOptions(): ContactConstraintsOptions;
export declare function createJointsOptions(): JointsOptions;
/**
 * Render body wireframes.
 *
 * @example
 * const { vertices, colors } = debug.bodies(world);
 */
export declare function bodies(world: World, options?: Partial<BodiesOptions>): DebugRenderResult;
/**
 * Render debug wireframe for a single body.
 *
 * @example
 * const { vertices, colors } = debug.body(myBody);
 * const { vertices, colors } = debug.body(myBody, { color: [1, 0, 0] });
 */
export declare function body(b: RigidBody, options?: Partial<BodyOptions>): DebugRenderResult;
/**
 * Render debug wireframe for a single shape at a given world-space transform.
 *
 * @example
 * const { vertices, colors } = debug.shape(myShape, [0, 0, 0], [0, 0, 0, 1]);
 * const { vertices, colors } = debug.shape(myShape, pos, quat, { color: [0.2, 1.0, 0.2] });
 */
export declare function shape(s: Shape, position: [number, number, number], quaternion: [number, number, number, number], options?: Partial<ShapeOptions>): DebugRenderResult;
/**
 * Render contact points and normals.
 *
 * @example
 * const { vertices, colors } = debug.contacts(world);
 */
export declare function contacts(world: World, _options?: Partial<ContactsOptions>): DebugRenderResult;
/**
 * Render detailed contact constraint manifolds (points, edges, normal, tangents).
 *
 * @example
 * const { vertices, colors } = debug.contactConstraints(world);
 */
export declare function contactConstraints(world: World, _options?: Partial<ContactConstraintsOptions>): DebugRenderResult;
/**
 * Render joint/constraint axes and limits.
 *
 * @example
 * const { vertices, colors } = debug.joints(world);
 */
export declare function joints(world: World, options?: Partial<JointsOptions>): DebugRenderResult;
