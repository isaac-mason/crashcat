export type CollidePointHit = {
    /** sub shape id of shape b, EMPTY_SUB_SHAPE_ID if not a compound shape */
    subShapeIdB: number;
    /** body id of shape b */
    bodyIdB: number;
    /** material id of the hit sub-shape */
    materialId: number;
};
export declare function createCollidePointHit(): CollidePointHit;
export declare function copyCollidePointHit(out: CollidePointHit, source: CollidePointHit): void;
export type CollidePointSettings = {
    collisionTolerance: number;
};
export declare function createDefaultCollidePointSettings(): CollidePointSettings;
export type CollidePointCollector = {
    bodyIdB: number;
    earlyOutFraction: number;
    addHit(hit: CollidePointHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
};
export declare class AllCollidePointCollector implements CollidePointCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hitPool: {
        request: () => CollidePointHit;
        release: (item: CollidePointHit) => void;
        reset: () => void;
    };
    hits: CollidePointHit[];
    addHit(h: CollidePointHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAllCollidePointCollector(): AllCollidePointCollector;
export declare class AnyCollidePointCollector implements CollidePointCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    hit: CollidePointHit | null;
    _hit: CollidePointHit;
    addHit(h: CollidePointHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAnyCollidePointCollector(): AnyCollidePointCollector;
