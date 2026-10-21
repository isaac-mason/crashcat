import { type Vec3 } from 'math';
import type { CollideShapeVsShapeFn } from '../shapes/shapes.js';
import { type Face } from '../utils/face.js';
export type CollideShapeHit = {
    /** contact position on body A (world space) */
    pointA: Vec3;
    /** contact position on body B (world space) */
    pointB: Vec3;
    /** penetration axis: direction to move shape B out of collision along shortest path, magnitude is meaningless, in world space */
    penetrationAxis: Vec3;
    /** penetration depth (positive = overlapping, can be negative if maxSeparationDistance > 0) */
    penetration: number;
    /** sub shape id of shape A, EMPTY_SUB_SHAPE_ID if not a compound shape */
    subShapeIdA: number;
    /** sub shape id of shape B, EMPTY_SUB_SHAPE_ID if not a compound shape */
    subShapeIdB: number;
    /** supporting face on shape A (world space, flat array of vertices) */
    faceA: Face;
    /** supporting face on shape B (world space, flat array of vertices) */
    faceB: Face;
    /** the body id for body B */
    bodyIdB: number;
};
export declare function createCollideShapeHit(): CollideShapeHit;
export declare function copyCollideShapeHit(out: CollideShapeHit, source: CollideShapeHit): void;
export type CollideShapeCollector = {
    bodyIdB: number;
    earlyOutFraction: number;
    addHit(hit: CollideShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    onBody?(bodyId: number): void;
    onBodyEnd?(): void;
    reset?(): void;
};
export declare class AllCollideShapeCollector implements CollideShapeCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hitPool: {
        request: () => CollideShapeHit;
        release: (item: CollideShapeHit) => void;
        reset: () => void;
    };
    hits: CollideShapeHit[];
    addHit(h: CollideShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
    /** orders hits deepest first */
    sort(): void;
}
export declare function createAllCollideShapeCollector(): AllCollideShapeCollector;
export declare class AnyCollideShapeCollector implements CollideShapeCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hit: CollideShapeHit | null;
    private _hit;
    addHit(h: CollideShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAnyCollideShapeCollector(): AnyCollideShapeCollector;
export declare class ClosestCollideShapeCollector implements CollideShapeCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hit: CollideShapeHit | null;
    _hit: CollideShapeHit;
    addHit(h: CollideShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createClosestCollideShapeCollector(): ClosestCollideShapeCollector;
export type CollideShapeSettings = {
    /** maximum separation distance for finding contacts when shapes are slightly separated, allows finding contacts even when shapes don't overlap. range: [0, 1], clamped for EPA use. */
    maxSeparationDistance: number;
    /** if objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter) */
    collisionTolerance: number;
    /** a factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless) */
    penetrationTolerance: number;
    /** if true, back facing triangles (triangle meshes) are collided with, otherwise they are ignored */
    collideWithBackfaces: boolean;
    /** how active edges (edges that a moving object should bump into) are handled */
    collideOnlyWithActiveEdges: boolean;
    /** when collideOnlyWithActiveEdges is true a movement direction can be provided. When hitting an inactive edge, the system will select the triangle normal as penetration depth only if it impedes the movement less than with the calculated penetration depth */
    activeEdgeMovementDirection: Vec3;
    /** if true, supporting faces will be collected and stored in the collision result */
    collectFaces: boolean;
    /** max squared distance to consider a vertex to be the same as another vertex, used by the internal edge removal algorithm to determine if two edges are shared (unit: meter^2) */
    internalEdgeRemovalVertexToleranceSq: number;
};
export declare const DEFAULT_INTERNAL_EDGE_REMOVAL_VERTEX_TOLERANCE_SQ = 1e-8;
export declare function createDefaultCollideShapeSettings(): CollideShapeSettings;
export declare function copyCollideShapeSettings(out: CollideShapeSettings, source: CollideShapeSettings): void;
/**
 * Wraps a collision function to swap shape A and B arguments.
 * Used when registering bidirectional collision handlers.
 *
 * @param fn the collision function to wrap
 * @returns A new function that calls fn with A and B swapped
 */
export declare function reversedCollideShapeVsShape(fn: CollideShapeVsShapeFn): CollideShapeVsShapeFn;
