import { type Vec3 } from 'math';
import { type CollideShapeCollector, type CollideShapeHit } from './collide-shape-vs-shape.js';
export type VoidedFeature = {
    /** feature position (world space) */
    feature: Vec3;
    /** sub-shape id of the convex shape (shape A) colliding against this feature */
    subShapeId: number;
};
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
export declare class InternalEdgeRemovingCollector implements CollideShapeCollector {
    bodyIdB: number;
    earlyOutFraction: number;
    private chainedCollector;
    private voidedFeatures;
    private delayedResults;
    addHit(hit: CollideShapeHit): void;
    addMiss(): void;
    shouldEarlyOut(): boolean;
    /**
     * process all delayed results, checking for voided features and forwarding non-voided hits.
     * this is typically called automatically at the end of processing all hits for a body.
     */
    flush(): void;
    /**
     * reset the collector state
     */
    reset(): void;
    /**
     * set the chained collector (for reusing the wrapper)
     */
    set(chainedCollector: CollideShapeCollector): void;
    /**
     * calculate the face normal from the first 3 vertices
     */
    private calculateFaceNormal;
    /**
     * get a vertex from a face by index
     */
    private getFaceVertex;
    /**
     * find the closest feature (vertex or edge) on a face to a point.
     * returns indices of the vertices that form the feature.
     * if vertexIndex1 === vertexIndex2, it's a vertex contact.
     * if vertexIndex1 !== vertexIndex2, it's an edge contact.
     */
    private findClosestFeature;
    /** check if a feature (vertex) is voided */
    private isVoided;
    /** void all vertices of a face */
    private voidFeatures;
    /** forward a hit to the chained collector */
    private chain;
    /** forward a hit to the chained collector and void its features */
    private chainAndVoid;
}
