import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import { pool } from '../utils/pool';
import {
    INITIAL_EARLY_OUT_FRACTION,
    SHOULD_EARLY_OUT_FRACTION,
} from './cast-utils';

export enum CastRayStatus {
    NOT_COLLIDING,
    COLLIDING
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

export function createCastRayHit(): CastRayHit {
    return {
        status: CastRayStatus.NOT_COLLIDING,
        fraction: 1.0,
        subShapeId: EMPTY_SUB_SHAPE_ID,
        bodyIdB: -1,
        materialId: -1,
    };
}

export function copyCastRayHit(out: CastRayHit, src: CastRayHit): void {
    out.status = src.status;
    out.fraction = src.fraction;
    out.subShapeId = src.subShapeId;
    out.bodyIdB = src.bodyIdB;
    out.materialId = src.materialId;
}

export type CastRaySettings = {
    /** if true, back-faces are considered for collision (otherwise they are ignored) */
    collideWithBackfaces: boolean;
    /** if true, ray starting inside convex shape returns hit at fraction 0. when false, only surface hits are reported */
    treatConvexAsSolid: boolean;
};

export function createDefaultCastRaySettings(): CastRaySettings {
    return {
        collideWithBackfaces: false,
        treatConvexAsSolid: true,
    };
}

export type CastRayCollector = {
    bodyIdB: number;
    earlyOutFraction: number;
    addHit(hit: CastRayHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
};

export class AllCastRayCollector implements CastRayCollector {
    bodyIdB = -1;
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hitPool = pool(createCastRayHit);
    hits: CastRayHit[] = [];

    addHit(hit: CastRayHit): void {
        const newHit = this.hitPool.request();
        copyCastRayHit(newHit, hit);
        this.hits.push(newHit);
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
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createAllCastRayCollector() {
    return new AllCastRayCollector();
}

export class AnyCastRayCollector implements CastRayCollector {
    bodyIdB = -1;
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hit: CastRayHit = createCastRayHit();

    addHit(h: CastRayHit): void {
        copyCastRayHit(this.hit, h);
        this.earlyOutFraction = SHOULD_EARLY_OUT_FRACTION;
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return this.hit.status === CastRayStatus.COLLIDING;
    }

    reset(): void {
        this.bodyIdB = -1;
        this.hit.status = CastRayStatus.NOT_COLLIDING;
        this.hit.fraction = 1.0;
        this.hit.subShapeId = EMPTY_SUB_SHAPE_ID;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createAnyCastRayCollector() {
    return new AnyCastRayCollector();
}

export class ClosestCastRayCollector implements CastRayCollector {
    bodyIdB = -1;
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hit: CastRayHit = createCastRayHit();

    addHit(h: CastRayHit): void {
        if (this.hit.status === CastRayStatus.NOT_COLLIDING || h.fraction < this.hit.fraction) {
            this.earlyOutFraction = h.fraction;
            copyCastRayHit(this.hit, h);
        }
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return false;
    }

    reset(): void {
        this.bodyIdB = -1;
        this.hit.status = CastRayStatus.NOT_COLLIDING;
        this.hit.fraction = 1.0;
        this.hit.subShapeId = EMPTY_SUB_SHAPE_ID;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createClosestCastRayCollector() {
    return new ClosestCastRayCollector();
}
