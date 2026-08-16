import { type Vec3 } from 'math';
import type { CastShapeVsShapeFn } from '../shapes/shapes.js';
import { type Face } from '../utils/face.js';
export declare enum CastShapeStatus {
    NOT_COLLIDING = 0,
    COLLIDING = 1
}
export type CastShapeHit = {
    /** status of the cast */
    status: CastShapeStatus;
    /** fraction along the sweep (0=start, 1=end) */
    fraction: number;
    /** point on shape A (world space) */
    pointA: Vec3;
    /** point on shape B (world space) */
    pointB: Vec3;
    /** penetration depth */
    penetrationDepth: number;
    /**
     * penetration axis: direction to move shape B out of collision along shortest path.
     * magnitude is meaningless, in world space.
     * when fraction > 0: this is the contact normal (A → B)
     * when fraction = 0: this is the penetration direction
     */
    penetrationAxis: Vec3;
    /**
     * contact normal (world space, normalized).
     * computed as -penetrationAxis.normalized().
     * points from shape B towards shape A (opposite of penetration axis).
     */
    normal: Vec3;
    /** sub-shape ID on shape A */
    subShapeIdA: number;
    /** sub-shape ID on shape B */
    subShapeIdB: number;
    /** material ID of sub-shape A */
    materialIdA: number;
    /** material ID of sub-shape B */
    materialIdB: number;
    /** supporting face on shape A (world space, flat array of vertices) */
    faceA: Face;
    /** supporting face on shape B (world space, flat array of vertices) */
    faceB: Face;
    /** body ID of shape B */
    bodyIdB: number;
};
export declare function createCastShapeHit(): CastShapeHit;
export declare function copyCastShapeHit(out: CastShapeHit, hit: CastShapeHit): void;
export type CastShapeSettings = {
    /** if true, back-faces are considered for collision (otherwise they are ignored) */
    collideWithBackfaces: boolean;
    /** if true, the supporting faces at the contact points will be computed and returned in the CastShapeHit objects */
    collectFaces: boolean;
    /** if objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter) */
    collisionTolerance: number;
    /** a factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless) */
    penetrationTolerance: number;
    /** when true, and the shape is intersecting at the beginning of the cast (fraction = 0) then this will calculate the deepest penetration point (costing additional CPU time) */
    returnDeepestPoint: boolean;
    /** indicates if we want to shrink the shape by the convex radius and then expand it again. This speeds up collision detection and gives a more accurate normal at the cost of a more 'rounded' shape */
    useShrunkenShapeAndConvexRadius: boolean;
    /** how active edges (edges that a moving object should bump into) are handled */
    collideOnlyWithActiveEdges: boolean;
    /** when collideOnlyWithActiveEdges is true a movement direction can be provided. When hitting an inactive edge, the system will select the triangle normal as penetration depth only if it impedes the movement less than with the calculated penetration depth */
    activeEdgeMovementDirection: Vec3;
};
export declare function createDefaultCastShapeSettings(): CastShapeSettings;
export type CastShapeCollector = {
    bodyIdB: number;
    earlyOutFraction: number;
    addHit(hit: CastShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
};
export declare class AllCastShapeCollector implements CastShapeCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hitPool: {
        request: () => CastShapeHit;
        release: (item: CastShapeHit) => void;
        reset: () => void;
    };
    hits: CastShapeHit[];
    addHit(h: CastShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAllCastShapeCollector(): AllCastShapeCollector;
export declare class AnyCastShapeCollector implements CastShapeCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hit: CastShapeHit;
    addHit(h: CastShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAnyCastShapeCollector(): AnyCastShapeCollector;
export declare class ClosestCastShapeCollector implements CastShapeCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hit: CastShapeHit;
    addHit(h: CastShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createClosestCastShapeCollector(): ClosestCastShapeCollector;
export declare function reversedCastShapeVsShape(fn: CastShapeVsShapeFn): CastShapeVsShapeFn;
