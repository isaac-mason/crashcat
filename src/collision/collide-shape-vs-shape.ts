import { type Vec3, vec3 } from 'mathcat';
import { createFace, type Face } from '../utils/face';
import { EMPTY_SUB_SHAPE_ID } from '../body/sub-shape';
import type { Shape } from '../shapes/shapes';
import { pool } from '../utils/pool';
import type { CollideShapeVsShapeFn } from '../shapes/shapes';

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
    /** material id of sub-shape A */
    materialIdA: number;
    /** material id of sub-shape B */
    materialIdB: number;
    /** supporting face on shape A (world space, flat array of vertices) */
    faceA: Face;
    /** supporting face on shape B (world space, flat array of vertices) */
    faceB: Face;
    /** the body id for body B */
    bodyIdB: number;
};

export function createCollideShapeHit(): CollideShapeHit {
    return {
        pointA: vec3.create(),
        pointB: vec3.create(),
        penetrationAxis: vec3.create(),
        penetration: 0,
        subShapeIdA: EMPTY_SUB_SHAPE_ID,
        subShapeIdB: EMPTY_SUB_SHAPE_ID,
        materialIdA: -1,
        materialIdB: -1,
        faceA: createFace(),
        faceB: createFace(),
        bodyIdB: -1,
    };
}

export function copyCollideShapeHit(out: CollideShapeHit, source: CollideShapeHit): void {
    vec3.copy(out.pointA, source.pointA);
    vec3.copy(out.pointB, source.pointB);
    vec3.copy(out.penetrationAxis, source.penetrationAxis);
    out.penetration = source.penetration;
    out.subShapeIdA = source.subShapeIdA;
    out.subShapeIdB = source.subShapeIdB;
    out.materialIdA = source.materialIdA;
    out.materialIdB = source.materialIdB;

    out.faceA.numVertices = source.faceA.numVertices;
    for (let i = 0; i < source.faceA.numVertices * 3; i++) {
        out.faceA.vertices[i] = source.faceA.vertices[i];
    }

    out.faceB.numVertices = source.faceB.numVertices;
    for (let i = 0; i < source.faceB.numVertices * 3; i++) {
        out.faceB.vertices[i] = source.faceB.vertices[i];
    }

    out.bodyIdB = source.bodyIdB;
}

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

export class AllCollideShapeCollector implements CollideShapeCollector {
    bodyIdB = -1;
    earlyOutFraction = Number.MAX_VALUE;
    hitPool = pool(createCollideShapeHit);
    hits: CollideShapeHit[] = [];

    addHit(h: CollideShapeHit): void {
        const hit = this.hitPool.request();
        copyCollideShapeHit(hit, h);
        this.hits.push(hit);
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return false;
    }

    reset(): void {
        this.hitPool.reset();
        this.hits.length = 0;
        this.earlyOutFraction = Number.MAX_VALUE;
        this.bodyIdB = -1;
    }
}

export function createAllCollideShapeCollector() {
    return new AllCollideShapeCollector();
}

export class AnyCollideShapeCollector implements CollideShapeCollector {
    bodyIdB = -1;
    earlyOutFraction = Number.MAX_VALUE;
    hit: CollideShapeHit | null = null;
    private _hit: CollideShapeHit = createCollideShapeHit();

    addHit(h: CollideShapeHit): void {
        copyCollideShapeHit(this._hit, h);
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
        this.hit = null;
        this.earlyOutFraction = Number.MAX_VALUE;
        this.bodyIdB = -1;
    }
}

export function createAnyCollideShapeCollector() {
    return new AnyCollideShapeCollector();
}

export class ClosestCollideShapeCollector implements CollideShapeCollector {
    bodyIdB = -1;
    earlyOutFraction = Infinity;
    hit: CollideShapeHit | null = null;
    _hit: CollideShapeHit = createCollideShapeHit();

    addHit(h: CollideShapeHit): void {
        if (this.hit === null || h.penetration > this.hit.penetration) {
            this.earlyOutFraction = h.penetration;
            copyCollideShapeHit(this._hit, h);
            this.hit = this._hit;
        }
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return false;
    }

    reset(): void {
        this.hit = null;
        this.earlyOutFraction = Infinity;
        this.bodyIdB = -1;
    }
}

export function createClosestCollideShapeCollector() {
    return new ClosestCollideShapeCollector();
}

export type CollideShapeSettings = {
    /** maximum separation distance for finding contacts when shapes are slightly separated, allows finding contacts even when shapes don't overlap. range: [0, 1], clamped for EPA use. */
    maxSeparationDistance: number;

    /** if objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter) */
    collisionTolerance: number;

    /** a factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless) */
    penetrationTolerance: number;

    /** when true, and the shape is intersecting at the beginning of the cast (fraction = 0) then this will calculate the deepest penetration point (costing additional CPU time) */
    returnDeepestPoint: boolean;

    /** if true, back-faces are considered for collision (otherwise they are ignored) */
    collideWithBackfaces: boolean;

    /** how active edges (edges that a moving object should bump into) are handled */
    collideOnlyWithActiveEdges: boolean;

    /** when collideOnlyWithActiveEdges is true a movement direction can be provided. When hitting an inactive edge, the system will select the triangle normal as penetration depth only if it impedes the movement less than with the calculated penetration depth */
    activeEdgeMovementDirection: Vec3;

    /** if true, supporting faces will be collected and stored in the collision result */
    collectFaces: boolean;
};

export function createDefaultCollideShapeSettings(): CollideShapeSettings {
    return {
        maxSeparationDistance: 0,
        collisionTolerance: 1e-4,
        penetrationTolerance: 1e-4,
        returnDeepestPoint: true,
        collideWithBackfaces: true,
        collideOnlyWithActiveEdges: true,
        activeEdgeMovementDirection: vec3.create(),
        collectFaces: false,
    };
}

export function copyCollideShapeSettings(
    out: CollideShapeSettings,
    source: CollideShapeSettings,
): void {
    out.maxSeparationDistance = source.maxSeparationDistance;
    out.collisionTolerance = source.collisionTolerance;
    out.penetrationTolerance = source.penetrationTolerance;
    out.returnDeepestPoint = source.returnDeepestPoint;
    out.collideWithBackfaces = source.collideWithBackfaces;
    out.collideOnlyWithActiveEdges = source.collideOnlyWithActiveEdges;
    vec3.copy(out.activeEdgeMovementDirection, source.activeEdgeMovementDirection);
    out.collectFaces = source.collectFaces;
}

function createInvertedCollector() {
    const collector: CollideShapeCollector & {
        base: CollideShapeCollector;
        bodyIdB: number;
    } = {
        base: null! as CollideShapeCollector,
        bodyIdB: -1,
        get earlyOutFraction() {
            return this.base.earlyOutFraction;
        },
        addHit(hit: CollideShapeHit) {
            // swap pointA and pointB
            const tmpX = hit.pointA[0];
            const tmpY = hit.pointA[1];
            const tmpZ = hit.pointA[2];
            hit.pointA[0] = hit.pointB[0];
            hit.pointA[1] = hit.pointB[1];
            hit.pointA[2] = hit.pointB[2];
            hit.pointB[0] = tmpX;
            hit.pointB[1] = tmpY;
            hit.pointB[2] = tmpZ;

            // negate penetrationAxis
            hit.penetrationAxis[0] = -hit.penetrationAxis[0];
            hit.penetrationAxis[1] = -hit.penetrationAxis[1];
            hit.penetrationAxis[2] = -hit.penetrationAxis[2];

            const tmpSubShapeId = hit.subShapeIdA;
            hit.subShapeIdA = hit.subShapeIdB;
            hit.subShapeIdB = tmpSubShapeId;

            const tmpFace = hit.faceA;
            hit.faceA = hit.faceB;
            hit.faceB = tmpFace;

            this.base.addHit(hit);
        },
        addMiss() {
            this.base.addMiss();
        },
        shouldEarlyOut() {
            return this.base.shouldEarlyOut();
        },
    };

    return collector;
}

const _invertedCollector = /* @__PURE__ */ createInvertedCollector();

/**
 * Wraps a collision function to swap shape A and B arguments.
 * Used when registering bidirectional collision handlers.
 *
 * @param fn the collision function to wrap
 * @returns A new function that calls fn with A and B swapped
 */
export function reversedCollideShapeVsShape(fn: CollideShapeVsShapeFn): CollideShapeVsShapeFn {
    return (
        collector: CollideShapeCollector,
        settings: CollideShapeSettings,
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
        _invertedCollector.bodyIdB = collector.bodyIdB;
        _invertedCollector.base = collector;

        const result = fn(
            _invertedCollector,
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

        _invertedCollector.base = null!;
        _invertedCollector.bodyIdB = -1;

        return result;
    };
}
