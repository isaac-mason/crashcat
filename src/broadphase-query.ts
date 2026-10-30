import type { Vec3 } from 'math';
import type { Box3 } from 'math/shapes';
import { type BodyId, INVALID_BODY_ID } from './body/body-id';
import type { RigidBody } from './body/rigid-body';
import type { OrientedBox } from './broadphase/broadphase';
import * as broadphase from './broadphase/broadphase';
import { INITIAL_EARLY_OUT_FRACTION, SHOULD_EARLY_OUT_FRACTION } from './collision/cast-utils';
import type { Filter } from './filter';
import { pool } from './utils/pool';
import type { World } from './world';

// the bodies whose bounds a query meets, under the filter, without looking at their shapes: jolt's BroadPhaseQuery. for
// the shapes themselves, castRay, collideShape and the other queries

export type { OrientedBox };

/** the bodies an overlap query meets; an overlap has no distance, so there is no closest, as jolt's CollideShapeBodyCollector */
export type BodyCollector = {
    addHit(bodyId: BodyId): void;
    shouldEarlyOut(): boolean;
};

export class AllBodyCollector implements BodyCollector {
    hits: BodyId[] = [];

    addHit(bodyId: BodyId): void {
        this.hits.push(bodyId);
    }

    shouldEarlyOut(): boolean {
        return false;
    }

    reset(): void {
        this.hits.length = 0;
    }
}

export function createAllBodyCollector() {
    return new AllBodyCollector();
}

export class AnyBodyCollector implements BodyCollector {
    hit: BodyId = INVALID_BODY_ID;

    addHit(bodyId: BodyId): void {
        this.hit = bodyId;
    }

    shouldEarlyOut(): boolean {
        return this.hit !== INVALID_BODY_ID;
    }

    reset(): void {
        this.hit = INVALID_BODY_ID;
    }
}

export function createAnyBodyCollector() {
    return new AnyBodyCollector();
}

/** a body whose bounds a cast entered, and the fraction of the cast where: jolt's BroadPhaseCastResult */
export type BodyCastHit = {
    bodyId: BodyId;
    fraction: number;
};

export function createBodyCastHit(): BodyCastHit {
    return { bodyId: INVALID_BODY_ID, fraction: 1 };
}

function copyBodyCastHit(out: BodyCastHit, hit: BodyCastHit): void {
    out.bodyId = hit.bodyId;
    out.fraction = hit.fraction;
}

/** the bodies a cast meets */
export type BodyCastCollector = {
    /** a hit at or past this is not reported, and the walk prunes nodes entered at or past it */
    earlyOutFraction: number;
    addHit(hit: BodyCastHit): void;
    shouldEarlyOut(): boolean;
};

function compareBodyCastHits(a: BodyCastHit, b: BodyCastHit): number {
    return a.fraction < b.fraction ? -1 : a.fraction > b.fraction ? 1 : 0;
}

export class AllBodyCastCollector implements BodyCastCollector {
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hitPool = pool(createBodyCastHit);
    hits: BodyCastHit[] = [];

    addHit(hit: BodyCastHit): void {
        const newHit = this.hitPool.request();
        copyBodyCastHit(newHit, hit);
        this.hits.push(newHit);
    }

    shouldEarlyOut(): boolean {
        return false;
    }

    reset(): void {
        this.hitPool.reset();
        this.hits.length = 0;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }

    /** orders hits nearest first */
    sort(): void {
        this.hits.sort(compareBodyCastHits);
    }
}

export function createAllBodyCastCollector() {
    return new AllBodyCastCollector();
}

export class AnyBodyCastCollector implements BodyCastCollector {
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hit: BodyCastHit = createBodyCastHit();

    addHit(hit: BodyCastHit): void {
        copyBodyCastHit(this.hit, hit);
        this.earlyOutFraction = SHOULD_EARLY_OUT_FRACTION;
    }

    shouldEarlyOut(): boolean {
        return this.hit.bodyId !== INVALID_BODY_ID;
    }

    reset(): void {
        this.hit.bodyId = INVALID_BODY_ID;
        this.hit.fraction = 1;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createAnyBodyCastCollector() {
    return new AnyBodyCastCollector();
}

export class ClosestBodyCastCollector implements BodyCastCollector {
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hit: BodyCastHit = createBodyCastHit();

    addHit(hit: BodyCastHit): void {
        if (hit.fraction >= this.earlyOutFraction) return;
        copyBodyCastHit(this.hit, hit);
        this.earlyOutFraction = hit.fraction;
    }

    shouldEarlyOut(): boolean {
        return false;
    }

    reset(): void {
        this.hit.bodyId = INVALID_BODY_ID;
        this.hit.fraction = 1;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createClosestBodyCastCollector() {
    return new ClosestBodyCastCollector();
}

/** hands each body an overlap walk meets to the collector */
const OverlapVisitor = {
    shouldExit: false,
    collector: null as unknown as BodyCollector,
    set(collector: BodyCollector) {
        this.shouldExit = collector.shouldEarlyOut();
        this.collector = collector;
    },
    visit(body: RigidBody) {
        this.collector.addHit(body.id);
        this.shouldExit = this.collector.shouldEarlyOut();
    },
    reset() {
        this.collector = null as unknown as BodyCollector;
    },
};

const _castVisitor_hit = /* @__PURE__ */ createBodyCastHit();

/** hands each body a cast walk meets, and where, to the collector, and the collector's bound back to the walk */
const CastVisitor = {
    shouldExit: false,
    earlyOutFraction: INITIAL_EARLY_OUT_FRACTION,
    collector: null as unknown as BodyCastCollector,
    set(collector: BodyCastCollector) {
        this.shouldExit = collector.shouldEarlyOut();
        this.earlyOutFraction = collector.earlyOutFraction;
        this.collector = collector;
    },
    visit(body: RigidBody, fraction?: number) {
        const entry = fraction ?? 0;
        // the walk visits a body it could not cull, which may lie at or past the collector's bound
        if (entry >= this.collector.earlyOutFraction) return;
        _castVisitor_hit.bodyId = body.id;
        _castVisitor_hit.fraction = entry;
        this.collector.addHit(_castVisitor_hit);
        this.shouldExit = this.collector.shouldEarlyOut();
        this.earlyOutFraction = this.collector.earlyOutFraction;
    },
    reset() {
        this.collector = null as unknown as BodyCastCollector;
    },
};

/** the bodies whose bounds the ray's segment enters */
export function castRay(
    world: World,
    collector: BodyCastCollector,
    origin: Vec3,
    direction: Vec3,
    length: number,
    filter: Filter,
): void {
    CastVisitor.set(collector);
    if (!CastVisitor.shouldExit) broadphase.castRay(world, origin, direction, length, filter, CastVisitor);
    CastVisitor.reset();
}

/** the bodies whose bounds overlap the box, faces included */
export function collideAABB(world: World, collector: BodyCollector, aabb: Box3, filter: Filter): void {
    OverlapVisitor.set(collector);
    if (!OverlapVisitor.shouldExit) broadphase.intersectAABB(world, aabb, filter, OverlapVisitor);
    OverlapVisitor.reset();
}

/** the bodies whose bounds are within radius of the centre */
export function collideSphere(world: World, collector: BodyCollector, center: Vec3, radius: number, filter: Filter): void {
    OverlapVisitor.set(collector);
    if (!OverlapVisitor.shouldExit) broadphase.intersectSphere(world, center, radius, filter, OverlapVisitor);
    OverlapVisitor.reset();
}

/** the bodies whose bounds hold the point, faces included */
export function collidePoint(world: World, collector: BodyCollector, point: Vec3, filter: Filter): void {
    OverlapVisitor.set(collector);
    if (!OverlapVisitor.shouldExit) broadphase.intersectPoint(world, point, filter, OverlapVisitor);
    OverlapVisitor.reset();
}

/** the bodies whose bounds overlap the oriented box */
export function collideOrientedBox(world: World, collector: BodyCollector, box: OrientedBox, filter: Filter): void {
    OverlapVisitor.set(collector);
    if (!OverlapVisitor.shouldExit) broadphase.intersectOrientedBox(world, box, filter, OverlapVisitor);
    OverlapVisitor.reset();
}

/** the bodies whose bounds the box meets as it moves by the displacement */
export function castAABB(world: World, collector: BodyCastCollector, aabb: Box3, displacement: Vec3, filter: Filter): void {
    CastVisitor.set(collector);
    if (!CastVisitor.shouldExit) broadphase.castAABB(world, aabb, displacement, filter, CastVisitor);
    CastVisitor.reset();
}
