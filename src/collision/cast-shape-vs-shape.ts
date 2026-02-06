import { type Vec3, vec3 } from 'mathcat';
import type { CastShapeVsShapeFn, Shape } from '../shapes/shapes';
import { createFace, type Face } from '../utils/face';
import { pool } from '../utils/pool';
import { INITIAL_EARLY_OUT_FRACTION, SHOULD_EARLY_OUT_FRACTION } from './cast-utils';

export enum CastShapeStatus {
    NOT_COLLIDING,
    COLLIDING,
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

export function createCastShapeHit(): CastShapeHit {
    return {
        status: CastShapeStatus.NOT_COLLIDING,
        fraction: 0,
        pointA: vec3.create(),
        pointB: vec3.create(),
        penetrationDepth: 0,
        penetrationAxis: vec3.create(),
        normal: vec3.create(),
        subShapeIdA: 0,
        subShapeIdB: 0,
        materialIdA: -1,
        materialIdB: -1,
        faceA: createFace(),
        faceB: createFace(),
        bodyIdB: -1,
    };
}

export function copyCastShapeHit(out: CastShapeHit, hit: CastShapeHit) {
    out.status = hit.status;
    out.fraction = hit.fraction;
    vec3.copy(out.pointA, hit.pointA);
    vec3.copy(out.pointB, hit.pointB);
    out.penetrationDepth = hit.penetrationDepth;
    vec3.copy(out.penetrationAxis, hit.penetrationAxis);
    vec3.copy(out.normal, hit.normal);
    out.subShapeIdA = hit.subShapeIdA;
    out.subShapeIdB = hit.subShapeIdB;
    out.materialIdA = hit.materialIdA;
    out.materialIdB = hit.materialIdB;

    out.faceA.numVertices = hit.faceA.numVertices;
    for (let i = 0; i < hit.faceA.vertices.length; i++) {
        out.faceA.vertices[i] = hit.faceA.vertices[i];
    }

    out.faceB.numVertices = hit.faceB.numVertices;
    for (let i = 0; i < hit.faceB.vertices.length; i++) {
        out.faceB.vertices[i] = hit.faceB.vertices[i];
    }

    out.bodyIdB = hit.bodyIdB;
}

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

export function createDefaultCastShapeSettings(): CastShapeSettings {
    return {
        collideWithBackfaces: false,
        collectFaces: false,
        collisionTolerance: 0.01,
        penetrationTolerance: 0.01,
        returnDeepestPoint: false,
        useShrunkenShapeAndConvexRadius: false,
        collideOnlyWithActiveEdges: true,
        activeEdgeMovementDirection: [0, 0, 0],
    };
}

export type CastShapeCollector = {
    bodyIdB: number;
    earlyOutFraction: number;
    addHit(hit: CastShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
};

export class AllCastShapeCollector implements CastShapeCollector {
    bodyIdB = -1;
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hitPool = pool(createCastShapeHit);
    hits: CastShapeHit[] = [];

    addHit(h: CastShapeHit): void {
        const hit = this.hitPool.request();
        copyCastShapeHit(hit, h);
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
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createAllCastShapeCollector() {
    return new AllCastShapeCollector();
}

export class AnyCastShapeCollector implements CastShapeCollector {
    bodyIdB = -1;
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hit: CastShapeHit = createCastShapeHit();

    addHit(h: CastShapeHit): void {
        copyCastShapeHit(this.hit, h);
        this.earlyOutFraction = SHOULD_EARLY_OUT_FRACTION;
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return this.earlyOutFraction <= SHOULD_EARLY_OUT_FRACTION;
    }

    reset(): void {
        this.bodyIdB = -1;
        this.hit.status = CastShapeStatus.NOT_COLLIDING;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createAnyCastShapeCollector() {
    return new AnyCastShapeCollector();
}

export class ClosestCastShapeCollector implements CastShapeCollector {
    bodyIdB = -1;
    earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    hit: CastShapeHit = createCastShapeHit();

    addHit(h: CastShapeHit): void {
        if (this.hit.status === CastShapeStatus.NOT_COLLIDING || h.fraction < this.hit.fraction) {
            this.earlyOutFraction = h.fraction;
            copyCastShapeHit(this.hit, h);
        }
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return this.earlyOutFraction <= SHOULD_EARLY_OUT_FRACTION;
    }

    reset(): void {
        this.bodyIdB = -1;
        this.hit.status = CastShapeStatus.NOT_COLLIDING;
        this.earlyOutFraction = INITIAL_EARLY_OUT_FRACTION;
    }
}

export function createClosestCastShapeCollector() {
    return new ClosestCastShapeCollector();
}

function createInvertedCollector() {
    const collector = {
        base: null! as CastShapeCollector,
        bodyIdB: -1,
        get earlyOutFraction() {
            return this.base.earlyOutFraction;
        },
        addHit(hit: CastShapeHit) {
            const tmpPoint = hit.pointA;
            hit.pointA = hit.pointB;
            hit.pointB = tmpPoint;

            vec3.negate(hit.normal, hit.normal);

            const tmpFace = hit.faceA;
            hit.faceA = hit.faceB;
            hit.faceB = tmpFace;

            const tmpSubShapeId = hit.subShapeIdA;
            hit.subShapeIdA = hit.subShapeIdB;
            hit.subShapeIdB = tmpSubShapeId;

            this.base.addHit(hit);
        },
        addMiss() {
            this.base.addMiss();
        },
        shouldEarlyOut() {
            return this.base.shouldEarlyOut();
        },
    } satisfies CastShapeCollector & {
        base: CastShapeCollector;
    };

    return collector;
}

export function reversedCastShapeVsShape(fn: CastShapeVsShapeFn): CastShapeVsShapeFn {
    const invertedCollector = createInvertedCollector();

    return (
        collector: CastShapeCollector,
        settings: CastShapeSettings,
        shapeA: Shape,
        subShapeIdA: number,
        subShapeIdBitsA: number,
        posAX: number,
        posAY: number,
        posAZ: number,
        quatAX: number,
        quatAY: number,
        quatAZ: number,
        quatAW: number,
        scaleAX: number,
        scaleAY: number,
        scaleAZ: number,
        dispAX: number,
        dispAY: number,
        dispAZ: number,
        shapeB: Shape,
        subShapeIdB: number,
        subShapeIdBitsB: number,
        posBX: number,
        posBY: number,
        posBZ: number,
        quatBX: number,
        quatBY: number,
        quatBZ: number,
        quatBW: number,
        scaleBX: number,
        scaleBY: number,
        scaleBZ: number,
    ) => {
        invertedCollector.base = collector;
        invertedCollector.bodyIdB = collector.bodyIdB;

        const result = fn(
            invertedCollector,
            settings,
            shapeB,
            subShapeIdB,
            subShapeIdBitsB,
            posBX,
            posBY,
            posBZ,
            quatBX,
            quatBY,
            quatBZ,
            quatBW,
            scaleBX,
            scaleBY,
            scaleBZ,
            -dispAX,
            -dispAY,
            -dispAZ,
            shapeA,
            subShapeIdA,
            subShapeIdBitsA,
            posAX,
            posAY,
            posAZ,
            quatAX,
            quatAY,
            quatAZ,
            quatAW,
            scaleAX,
            scaleAY,
            scaleAZ,
        );

        invertedCollector.base = null!;

        return result;
    };
}
