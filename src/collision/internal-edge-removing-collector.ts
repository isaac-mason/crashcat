import { vec3, type Vec3 } from 'mathcat';
import {
    type CollideShapeCollector,
    type CollideShapeHit,
    createCollideShapeHit,
    copyCollideShapeHit,
} from './collide-shape-vs-shape';
import type { Face } from '../utils/face';
import { pool } from '../utils/pool';

// cosine of 1 degree (for face contact detection threshold)
const COS_1_DEGREE = Math.cos((1 * Math.PI) / 180);

// epsilon for floating point comparisons
const FLT_EPSILON = 1e-6;
const FLT_EPSILON_SQ = FLT_EPSILON * FLT_EPSILON;

const _faceV0 = vec3.create();
const _faceV1 = vec3.create();
const _faceV2 = vec3.create();
const _vertexA = vec3.create();
const _vertexB = vec3.create();
const _triangleNormal = vec3.create();
const _contactNormal = vec3.create();
const _v1 = vec3.create();
const _v2 = vec3.create();
const _v1_v2 = vec3.create();
const _closest = vec3.create();
const _edge01 = vec3.create();
const _edge12 = vec3.create();

type ClosestFeatureResult = {
    vertexIndex1: number;
    vertexIndex2: number;
    distanceSq: number;
};

const _closestFeatureResult: ClosestFeatureResult = {
    vertexIndex1: 0,
    vertexIndex2: 0,
    distanceSq: 0,
};

export type VoidedFeature = {
    /** feature position (world space) */
    feature: Vec3;
    /** sub-shape id of the convex shape (shape A) colliding against this feature */
    subShapeId: number;
};

const delayedResultsPool = pool(createCollideShapeHit);

const voidedFeaturesPool = pool<VoidedFeature>(() => ({
    feature: vec3.create(),
    subShapeId: 0,
}));

/**
 * internal edge removing collector - eliminates "ghost collisions" that occur when a convex object
 * slides across internal edges of a triangle mesh or compound shape.
 *
 * this collector wraps another collector and delays processing of edge/vertex contacts to determine
 * if they are on internal edges based on the full contact set. face contacts are forwarded immediately.
 *
 * the algorithm:
 * 1. collect all hits during addHit calls
 * 2. forward face contacts immediately (contact normal aligns with face normal within 1 degree)
 * 3. delay edge/vertex contacts for later processing
 * 4. on flush (typically called at end of body collision):
 *    - sort delayed results by penetration depth (deepest first)
 *    - for each result, find closest feature (vertex or edge)
 *    - if feature is already "voided" by a deeper contact, skip it
 *    - otherwise forward to chained collector and void all vertices of this face
 *
 * this ensures that internal edges between coplanar/near-coplanar faces are ignored,
 * preventing stuttering when objects slide across multi-primitive surfaces.
 */
export class InternalEdgeRemovingCollector implements CollideShapeCollector {
    bodyIdB = -1;
    earlyOutFraction = Number.MAX_VALUE;

    private chainedCollector: CollideShapeCollector = null!;
    private voidedFeatures: VoidedFeature[] = [];
    private delayedResults: CollideShapeHit[] = [];

    addHit(hit: CollideShapeHit): void {
        // 1. degenerate check - faces with < 3 vertices can't have internal edges
        if (hit.faceB.numVertices < 3) {
            this.chainAndVoid(hit);
            return;
        }

        // 2. calculate face normal
        this.calculateFaceNormal(hit.faceB, _triangleNormal);
        const normalLengthSq = vec3.squaredLength(_triangleNormal);

        if (normalLengthSq < 1e-12) {
            // degenerate face, forward immediately
            this.chainAndVoid(hit);
            return;
        }

        // 3. check if contact normal aligns with face normal (within 1 degree)
        // contact normal is opposite of penetration axis
        vec3.negate(_contactNormal, hit.penetrationAxis);

        const dotProduct = vec3.dot(_triangleNormal, _contactNormal);
        const contactNormalLength = vec3.length(_contactNormal);
        const faceNormalLength = Math.sqrt(normalLengthSq);
        const threshold = COS_1_DEGREE * faceNormalLength * contactNormalLength;

        if (dotProduct > threshold) {
            // face contact - process immediately
            this.chainAndVoid(hit);
            return;
        }

        // 4. edge/vertex contact - delay for processing
        const delayedHit = delayedResultsPool.request();
        copyCollideShapeHit(delayedHit, hit);
        this.delayedResults.push(delayedHit);
    }

    addMiss(): void {
        this.chainedCollector.addMiss();
    }

    shouldEarlyOut(): boolean {
        return this.chainedCollector.shouldEarlyOut();
    }

    /**
     * process all delayed results, checking for voided features and forwarding non-voided hits.
     * this is typically called automatically at the end of processing all hits for a body.
     */
    flush(): void {
        // 1. sort by penetration depth (deepest first)
        this.delayedResults.sort((a, b) => b.penetration - a.penetration);

        // 2. process each result
        for (const result of this.delayedResults) {
            // 3. find closest feature (vertex or edge)
            this.findClosestFeature(_closestFeatureResult, result.faceB, result.pointB);
            const { vertexIndex1, vertexIndex2 } = _closestFeatureResult;

            // 4. check if voided
            const v1 = this.getFaceVertex(result.faceB, vertexIndex1, _vertexA);
            let voided = this.isVoided(result.subShapeIdA, v1);

            if (vertexIndex1 !== vertexIndex2) {
                // edge contact - both vertices must be voided
                const v2 = this.getFaceVertex(result.faceB, vertexIndex2, _vertexB);
                if (!this.isVoided(result.subShapeIdA, v2)) {
                    voided = false;
                }
            }

            // 5. if not voided, forward to chained collector
            if (!voided) {
                this.chain(result);
            }

            // 6. void all features of this face
            this.voidFeatures(result);
        }

        // 7. clear for next body
        for (const result of this.delayedResults) {
            delayedResultsPool.release(result);
        }
        for (const vf of this.voidedFeatures) {
            voidedFeaturesPool.release(vf);
        }
        this.voidedFeatures.length = 0;
        this.delayedResults.length = 0;
    }

    /**
     * reset the collector state
     */
    reset(): void {
        this.voidedFeatures.length = 0;
        this.delayedResults.length = 0;
        this.earlyOutFraction = Number.MAX_VALUE;
        this.bodyIdB = -1;
    }

    /**
     * set the chained collector (for reusing the wrapper)
     */
    set(chainedCollector: CollideShapeCollector): void {
        this.chainedCollector = chainedCollector;
        this.earlyOutFraction = chainedCollector.earlyOutFraction;
        this.bodyIdB = chainedCollector.bodyIdB;
    }

    /**
     * calculate the face normal from the first 3 vertices
     */
    private calculateFaceNormal(face: Face, out: Vec3): void {
        if (face.numVertices < 3) {
            vec3.set(out, 0, 0, 0);
            return;
        }

        // get first 3 vertices
        const v0 = vec3.set(_faceV0, face.vertices[0], face.vertices[1], face.vertices[2]);
        const v1 = vec3.set(_faceV1, face.vertices[3], face.vertices[4], face.vertices[5]);
        const v2 = vec3.set(_faceV2, face.vertices[6], face.vertices[7], face.vertices[8]);

        // calculate edges
        vec3.subtract(_edge01, v1, v0);
        vec3.subtract(_edge12, v2, v0);

        // cross product
        vec3.cross(out, _edge01, _edge12);
    }

    /**
     * get a vertex from a face by index
     */
    private getFaceVertex(face: Face, index: number, out: Vec3): Vec3 {
        const i = index * 3;
        return vec3.set(out, face.vertices[i], face.vertices[i + 1], face.vertices[i + 2]);
    }

    /**
     * find the closest feature (vertex or edge) on a face to a point.
     * returns indices of the vertices that form the feature.
     * if vertexIndex1 === vertexIndex2, it's a vertex contact.
     * if vertexIndex1 !== vertexIndex2, it's an edge contact.
     */
    private findClosestFeature(
        out: ClosestFeatureResult,
        face: Face,
        point: Vec3,
    ): void {
        let bestDistSq = Number.MAX_VALUE;
        let bestV1 = 0;
        let bestV2 = 0;

        const numVertices = face.numVertices;
        let v1Idx = numVertices - 1;
        vec3.subtract(_v1, this.getFaceVertex(face, v1Idx, _vertexA), point);

        for (let v2Idx = 0; v2Idx < numVertices; v2Idx++) {
            vec3.subtract(_v2, this.getFaceVertex(face, v2Idx, _vertexB), point);
            vec3.subtract(_v1_v2, _v2, _v1);

            const denominator = vec3.squaredLength(_v1_v2);

            if (denominator < FLT_EPSILON_SQ) {
                // degenerate edge, test v1 only
                const v1LenSq = vec3.squaredLength(_v1);
                if (v1LenSq < bestDistSq) {
                    bestDistSq = v1LenSq;
                    bestV1 = v1Idx;
                    bestV2 = v1Idx;
                }
            } else {
                // project point onto line segment
                const fraction = -vec3.dot(_v1, _v1_v2) / denominator;

                if (fraction < 1e-6) {
                    // closest to v1
                    const v1LenSq = vec3.squaredLength(_v1);
                    if (v1LenSq < bestDistSq) {
                        bestDistSq = v1LenSq;
                        bestV1 = v1Idx;
                        bestV2 = v1Idx;
                    }
                } else if (fraction < 1.0 - 1e-6) {
                    // closest to edge
                    vec3.scaleAndAdd(_closest, _v1, _v1_v2, fraction);
                    const closestLenSq = vec3.squaredLength(_closest);
                    if (closestLenSq < bestDistSq) {
                        bestDistSq = closestLenSq;
                        bestV1 = v1Idx;
                        bestV2 = v2Idx;
                    }
                }
                // else closest to v2, will be tested in next iteration
            }

            v1Idx = v2Idx;
            vec3.copy(_v1, _v2);
        }

        out.vertexIndex1 = bestV1;
        out.vertexIndex2 = bestV2;
        out.distanceSq = bestDistSq;
    }

    /** check if a feature (vertex) is voided */
    private isVoided(subShapeId: number, feature: Vec3): boolean {
        for (const vf of this.voidedFeatures) {
            if (vf.subShapeId === subShapeId && vec3.squaredDistance(vf.feature, feature) < 1e-16) {
                return true;
            }
        }
        return false;
    }

    /** void all vertices of a face */
    private voidFeatures(hit: CollideShapeHit): void {
        const face = hit.faceB;
        for (let i = 0; i < face.numVertices; i++) {
            const vertex = this.getFaceVertex(face, i, _vertexA);

            // don't duplicate voided features
            if (!this.isVoided(hit.subShapeIdA, vertex)) {
                const vf = voidedFeaturesPool.request();
                vec3.copy(vf.feature, vertex);
                vf.subShapeId = hit.subShapeIdA;
                this.voidedFeatures.push(vf);
            }
        }
    }

    /** forward a hit to the chained collector */
    private chain(hit: CollideShapeHit): void {
        this.chainedCollector.bodyIdB = hit.bodyIdB;
        this.chainedCollector.addHit(hit);
        // sync early out fraction from chained collector
        this.earlyOutFraction = this.chainedCollector.earlyOutFraction;
    }

    /** forward a hit to the chained collector and void its features */
    private chainAndVoid(hit: CollideShapeHit): void {
        this.chain(hit);
        this.voidFeatures(hit);
    }
}
