export declare enum CastRayStatus {
    NOT_COLLIDING = 0,
    COLLIDING = 1
}
export type CastRayHit = {
    /** status of the ray cast */
    status: CastRayStatus;
    /** fraction along the ray where the hit occurred (0 = start, 1 = end) */
    fraction: number;
    /** sub shape id of the hit shape */
    subShapeId: number;
    /** id of the body that was hit */
    bodyIdB: number;
    /** material id of the hit sub-shape */
    materialId: number;
};
export declare function createCastRayHit(): CastRayHit;
export declare function copyCastRayHit(out: CastRayHit, src: CastRayHit): void;
export type CastRaySettings = {
    /** if true, back facing triangles (triangle meshes) are hit, otherwise they are ignored */
    collideWithBackfacesTriangles: boolean;
    /** if true, convex shapes (and planes) are also hit where the ray exits them, otherwise only where it enters */
    collideWithBackfacesConvex: boolean;
    /** if true, ray starting inside convex shape returns hit at fraction 0. when false, only surface hits are reported */
    treatConvexAsSolid: boolean;
};
export declare function createDefaultCastRaySettings(): CastRaySettings;
export type CastRayCollector = {
    bodyIdB: number;
    earlyOutFraction: number;
    addHit(hit: CastRayHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
};
export declare class AllCastRayCollector implements CastRayCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hitPool: {
        request: () => CastRayHit;
        release: (item: CastRayHit) => void;
        reset: () => void;
    };
    hits: CastRayHit[];
    addHit(hit: CastRayHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
    /** orders hits closest first */
    sort(): void;
}
export declare function createAllCastRayCollector(): AllCastRayCollector;
export declare class AnyCastRayCollector implements CastRayCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hit: CastRayHit;
    addHit(h: CastRayHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAnyCastRayCollector(): AnyCastRayCollector;
export declare class ClosestCastRayCollector implements CastRayCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hit: CastRayHit;
    addHit(h: CastRayHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createClosestCastRayCollector(): ClosestCastRayCollector;
