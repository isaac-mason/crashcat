import type { Vec3 } from 'math';
import type { Box3 } from 'math/shapes';
import { type BodyId } from './body/body-id.js';
import type { OrientedBox } from './broadphase/broadphase.js';
import type { Filter } from './filter.js';
import type { World } from './world.js';
export type { OrientedBox };
/** the bodies an overlap query meets; an overlap has no distance, so there is no closest, as jolt's CollideShapeBodyCollector */
export type BodyCollector = {
    addHit(bodyId: BodyId): void;
    shouldEarlyOut(): boolean;
};
export declare class AllBodyCollector implements BodyCollector {
    hits: BodyId[];
    addHit(bodyId: BodyId): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAllBodyCollector(): AllBodyCollector;
export declare class AnyBodyCollector implements BodyCollector {
    hit: BodyId;
    addHit(bodyId: BodyId): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAnyBodyCollector(): AnyBodyCollector;
/** a body whose bounds a cast entered, and the fraction of the cast where: jolt's BroadPhaseCastResult */
export type BodyCastHit = {
    bodyId: BodyId;
    fraction: number;
};
export declare function createBodyCastHit(): BodyCastHit;
/** the bodies a cast meets */
export type BodyCastCollector = {
    /** a hit at or past this is not reported, and the walk prunes nodes entered at or past it */
    earlyOutFraction: number;
    addHit(hit: BodyCastHit): void;
    shouldEarlyOut(): boolean;
};
export declare class AllBodyCastCollector implements BodyCastCollector {
    earlyOutFraction: number;
    hitPool: {
        request: () => BodyCastHit;
        release: (item: BodyCastHit) => void;
        reset: () => void;
    };
    hits: BodyCastHit[];
    addHit(hit: BodyCastHit): void;
    shouldEarlyOut(): boolean;
    reset(): void;
    /** orders hits nearest first */
    sort(): void;
}
export declare function createAllBodyCastCollector(): AllBodyCastCollector;
export declare class AnyBodyCastCollector implements BodyCastCollector {
    earlyOutFraction: number;
    hit: BodyCastHit;
    addHit(hit: BodyCastHit): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createAnyBodyCastCollector(): AnyBodyCastCollector;
export declare class ClosestBodyCastCollector implements BodyCastCollector {
    earlyOutFraction: number;
    hit: BodyCastHit;
    addHit(hit: BodyCastHit): void;
    shouldEarlyOut(): boolean;
    reset(): void;
}
export declare function createClosestBodyCastCollector(): ClosestBodyCastCollector;
/** the bodies whose bounds the ray's segment enters */
export declare function castRay(world: World, collector: BodyCastCollector, origin: Vec3, direction: Vec3, length: number, filter: Filter): void;
/** the bodies whose bounds overlap the box, faces included */
export declare function collideAABB(world: World, collector: BodyCollector, aabb: Box3, filter: Filter): void;
/** the bodies whose bounds are within radius of the centre */
export declare function collideSphere(world: World, collector: BodyCollector, center: Vec3, radius: number, filter: Filter): void;
/** the bodies whose bounds hold the point, faces included */
export declare function collidePoint(world: World, collector: BodyCollector, point: Vec3, filter: Filter): void;
/** the bodies whose bounds overlap the oriented box */
export declare function collideOrientedBox(world: World, collector: BodyCollector, box: OrientedBox, filter: Filter): void;
/** the bodies whose bounds the box meets as it moves by the displacement */
export declare function castAABB(world: World, collector: BodyCastCollector, aabb: Box3, displacement: Vec3, filter: Filter): void;
