import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import { pool } from '../utils/pool';

export type CollidePointHit = {
    /** sub shape id of shape b, EMPTY_SUB_SHAPE_ID if not a compound shape */
    subShapeIdB: number;
    /** body id of shape b */
    bodyIdB: number;
    /** material id of the hit sub-shape */
    materialId: number;
};

export function createCollidePointHit(): CollidePointHit {
    return {
        subShapeIdB: EMPTY_SUB_SHAPE_ID,
        bodyIdB: -1,
        materialId: -1,
    };
}

export function copyCollidePointHit(out: CollidePointHit, source: CollidePointHit): void {
    out.subShapeIdB = source.subShapeIdB;
    out.bodyIdB = source.bodyIdB;
    out.materialId = source.materialId;
}

export type CollidePointSettings = {
    collisionTolerance: number;
};

export function createDefaultCollidePointSettings(): CollidePointSettings {
    return {
        collisionTolerance: 1e-4,
    };
}

export type CollidePointCollector = {
    bodyIdB: number;
    earlyOutFraction: number;
    addHit(hit: CollidePointHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
};

export class AllCollidePointCollector implements CollidePointCollector {
    bodyIdB = -1;
    earlyOutFraction = Number.MAX_VALUE;
    hitPool = pool(createCollidePointHit);
    hits: CollidePointHit[] = [];

    addHit(h: CollidePointHit): void {
        const hit = this.hitPool.request();
        copyCollidePointHit(hit, h);
        this.hits.push(hit);
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return false;
    }

    reset(): void {
        this.bodyIdB = -1;
        this.hitPool.reset();
        this.hits.length = 0;
        this.earlyOutFraction = Number.MAX_VALUE;
    }
}

export function createAllCollidePointCollector() {
    return new AllCollidePointCollector();
}

export class AnyCollidePointCollector implements CollidePointCollector {
    bodyIdB = -1;
    earlyOutFraction = Number.MAX_VALUE;
    hit: CollidePointHit | null = null;
    _hit: CollidePointHit = createCollidePointHit();

    addHit(h: CollidePointHit): void {
        copyCollidePointHit(this._hit, h);
        this.hit = this._hit;
        this.earlyOutFraction = 0; // early out immediately
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return this.hit !== null;
    }

    reset(): void {
        this.bodyIdB = -1;
        this.hit = null;
        this.earlyOutFraction = Number.MAX_VALUE;
    }
}

export function createAnyCollidePointCollector() {
    return new AnyCollidePointCollector();
}
