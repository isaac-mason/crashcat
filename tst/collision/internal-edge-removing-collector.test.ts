import { describe, expect, it } from 'vitest';
import { vec3 } from 'mathcat';
import {
    InternalEdgeRemovingCollector,
    createCollideShapeHit,
    createAllCollideShapeCollector,
    type CollideShapeHit,
} from '../../src/collision/narrowphase';
import { createFace } from '../../src/utils/face';
import { EMPTY_SUB_SHAPE_ID } from '../../src/body/sub-shape';

describe('InternalEdgeRemovingCollector', () => {
    describe('face contact detection', () => {
        it('should forward face contacts immediately', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            // create a hit with face normal aligned with contact normal (within 1 degree)
            const hit = createCollideShapeHit();
            hit.bodyIdB = 1;
            hit.penetration = 1.0;
            hit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            hit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            // create a triangle face (equilateral, lying on xy plane)
            const face = createFace();
            face.numVertices = 3;
            face.vertices[0] = 0;
            face.vertices[1] = 0;
            face.vertices[2] = 0;
            face.vertices[3] = 1;
            face.vertices[4] = 0;
            face.vertices[5] = 0;
            face.vertices[6] = 0.5;
            face.vertices[7] = Math.sqrt(3) / 2;
            face.vertices[8] = 0;
            hit.faceB = face;

            // set contact normal aligned with face normal (pointing up in z)
            vec3.set(hit.penetrationAxis, 0, 0, -1); // penetration axis is opposite of contact normal
            vec3.set(hit.pointB, 0.5, 0.3, 0);

            collector.addHit(hit);

            // should be forwarded immediately, not delayed
            expect(chainedCollector.hits.length).toBe(1);
            expect(chainedCollector.hits[0].penetration).toBe(1.0);
        });

        it('should delay edge/vertex contacts', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            const hit = createCollideShapeHit();
            hit.bodyIdB = 1;
            hit.penetration = 1.0;
            hit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            hit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            // create a triangle face
            const face = createFace();
            face.numVertices = 3;
            face.vertices[0] = 0;
            face.vertices[1] = 0;
            face.vertices[2] = 0;
            face.vertices[3] = 1;
            face.vertices[4] = 0;
            face.vertices[5] = 0;
            face.vertices[6] = 0.5;
            face.vertices[7] = Math.sqrt(3) / 2;
            face.vertices[8] = 0;
            hit.faceB = face;

            // set contact normal NOT aligned with face normal (edge contact)
            vec3.set(hit.penetrationAxis, 0, -1, 0);
            vec3.set(hit.pointB, 0.5, 0, 0); // on edge

            collector.addHit(hit);

            // should be delayed, not forwarded yet
            expect(chainedCollector.hits.length).toBe(0);

            // flush to process delayed results
            collector.flush();

            // now should be forwarded
            expect(chainedCollector.hits.length).toBe(1);
        });

        it('should forward degenerate faces immediately', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            const hit = createCollideShapeHit();
            hit.bodyIdB = 1;
            hit.penetration = 1.0;
            hit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            hit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            // degenerate face with < 3 vertices
            const face = createFace();
            face.numVertices = 2;
            face.vertices[0] = 0;
            face.vertices[1] = 0;
            face.vertices[2] = 0;
            face.vertices[1] = 0;
            face.vertices[2] = 0;
            face.vertices[3] = 1;
            face.vertices[4] = 0;
            face.vertices[5] = 0;
            hit.faceB = face;

            vec3.set(hit.penetrationAxis, 0, 0, -1);
            vec3.set(hit.pointB, 0.5, 0, 0);

            collector.addHit(hit);

            // should be forwarded immediately
            expect(chainedCollector.hits.length).toBe(1);
        });
    });

    describe('voiding algorithm', () => {
        it('should void edge contacts when face contact exists at same location', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            // first hit: face contact (deeper penetration)
            const faceHit = createCollideShapeHit();
            faceHit.bodyIdB = 1;
            faceHit.penetration = 2.0;
            faceHit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            faceHit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            const face1 = createFace();
            face1.numVertices = 4;
            // square face
            face1.vertices[0] = 0;
            face1.vertices[1] = 0;
            face1.vertices[2] = 0;
            face1.vertices[3] = 1;
            face1.vertices[4] = 0;
            face1.vertices[5] = 0;
            face1.vertices[6] = 1;
            face1.vertices[7] = 1;
            face1.vertices[8] = 0;
            face1.vertices[9] = 0;
            face1.vertices[10] = 1;
            face1.vertices[11] = 0;
            faceHit.faceB = face1;

            vec3.set(faceHit.penetrationAxis, 0, 0, -1);
            vec3.set(faceHit.pointB, 0.5, 0.5, 0);

            // second hit: edge contact at shared edge (shallower penetration)
            const edgeHit = createCollideShapeHit();
            edgeHit.bodyIdB = 1;
            edgeHit.penetration = 1.0;
            edgeHit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            edgeHit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            const face2 = createFace();
            face2.numVertices = 4;
            // adjacent square face sharing edge at x=1
            face2.vertices[0] = 1;
            face2.vertices[1] = 0;
            face2.vertices[2] = 0;
            face2.vertices[3] = 2;
            face2.vertices[4] = 0;
            face2.vertices[5] = 0;
            face2.vertices[6] = 2;
            face2.vertices[7] = 1;
            face2.vertices[8] = 0;
            face2.vertices[9] = 1;
            face2.vertices[10] = 1;
            face2.vertices[11] = 0;
            edgeHit.faceB = face2;

            vec3.set(edgeHit.penetrationAxis, 1, 0, 0); // edge contact normal
            vec3.set(edgeHit.pointB, 1, 0.5, 0); // on shared edge

            collector.addHit(faceHit);
            collector.addHit(edgeHit);

            collector.flush();

            // face contact should be forwarded
            // edge contact should be voided because its vertices were voided by face contact
            expect(chainedCollector.hits.length).toBe(1);
            expect(chainedCollector.hits[0].penetration).toBe(2.0);
        });

        it('should process delayed results in penetration depth order', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            // create three edge contacts with different penetration depths
            const createEdgeHit = (penetration: number, edgeX: number): CollideShapeHit => {
                const hit = createCollideShapeHit();
                hit.bodyIdB = 1;
                hit.penetration = penetration;
                hit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
                hit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

                const face = createFace();
                face.numVertices = 3;
                face.vertices[0] = edgeX;
                face.vertices[1] = 0;
                face.vertices[2] = 0;
                face.vertices[3] = edgeX + 1;
                face.vertices[4] = 0;
                face.vertices[5] = 0;
                face.vertices[6] = edgeX + 0.5;
                face.vertices[7] = 1;
                face.vertices[8] = 0;
                hit.faceB = face;

                vec3.set(hit.penetrationAxis, 0, -1, 0);
                vec3.set(hit.pointB, edgeX + 0.5, 0, 0);

                return hit;
            };

            // add in random order
            collector.addHit(createEdgeHit(1.0, 0)); // shallow
            collector.addHit(createEdgeHit(3.0, 2)); // deepest
            collector.addHit(createEdgeHit(2.0, 1)); // middle

            collector.flush();

            // all should be forwarded (no voiding since no shared vertices)
            expect(chainedCollector.hits.length).toBe(3);

            // verify they were processed deepest first by checking voiding order
            // (the deepest one should establish the voiding context)
            expect(chainedCollector.hits[0].penetration).toBe(3.0);
            expect(chainedCollector.hits[1].penetration).toBe(2.0);
            expect(chainedCollector.hits[2].penetration).toBe(1.0);
        });

        it('should handle vertex voiding correctly', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            // first triangle touching at a vertex
            const hit1 = createCollideShapeHit();
            hit1.bodyIdB = 1;
            hit1.penetration = 2.0;
            hit1.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            hit1.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            const face1 = createFace();
            face1.numVertices = 3;
            face1.vertices[0] = 0;
            face1.vertices[1] = 0;
            face1.vertices[2] = 0;
            face1.vertices[3] = 1;
            face1.vertices[4] = 0;
            face1.vertices[5] = 0;
            face1.vertices[6] = 0.5;
            face1.vertices[7] = 1;
            face1.vertices[8] = 0;
            hit1.faceB = face1;

            vec3.set(hit1.penetrationAxis, 0, -1, 0);
            vec3.set(hit1.pointB, 1, 0, 0); // contact at vertex (1,0,0)

            // second triangle sharing the same vertex
            const hit2 = createCollideShapeHit();
            hit2.bodyIdB = 1;
            hit2.penetration = 1.0;
            hit2.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            hit2.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            const face2 = createFace();
            face2.numVertices = 3;
            face2.vertices[0] = 1;
            face2.vertices[1] = 0;
            face2.vertices[2] = 0; // shared vertex
            face2.vertices[3] = 2;
            face2.vertices[4] = 0;
            face2.vertices[5] = 0;
            face2.vertices[6] = 1.5;
            face2.vertices[7] = 1;
            face2.vertices[8] = 0;
            hit2.faceB = face2;

            vec3.set(hit2.penetrationAxis, 0, -1, 0);
            vec3.set(hit2.pointB, 1, 0, 0); // contact at same vertex

            collector.addHit(hit1);
            collector.addHit(hit2);

            collector.flush();

            // first hit should be forwarded, second should be voided
            expect(chainedCollector.hits.length).toBe(1);
            expect(chainedCollector.hits[0].penetration).toBe(2.0);
        });
    });

    describe('sub-shape tracking', () => {
        it('should track voiding per sub-shape id', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            // two hits with same vertex but different sub-shape ids
            const createHit = (subShapeId: number, penetration: number): CollideShapeHit => {
                const hit = createCollideShapeHit();
                hit.bodyIdB = 1;
                hit.penetration = penetration;
                hit.subShapeIdA = subShapeId; // different sub-shapes
                hit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

                const face = createFace();
                face.numVertices = 3;
                face.vertices[0] = 0;
                face.vertices[1] = 0;
                face.vertices[2] = 0;
                face.vertices[3] = 1;
                face.vertices[4] = 0;
                face.vertices[5] = 0;
                face.vertices[6] = 0.5;
                face.vertices[7] = 1;
                face.vertices[8] = 0;
                hit.faceB = face;

                vec3.set(hit.penetrationAxis, 0, -1, 0);
                vec3.set(hit.pointB, 0.5, 0, 0);

                return hit;
            };

            collector.addHit(createHit(1, 2.0));
            collector.addHit(createHit(2, 1.0));

            collector.flush();

            // both should be forwarded (different sub-shapes, so voiding doesn't apply)
            expect(chainedCollector.hits.length).toBe(2);
        });
    });

    describe('reset', () => {
        it('should clear state on reset', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            const hit = createCollideShapeHit();
            hit.bodyIdB = 1;
            hit.penetration = 1.0;
            hit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            hit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            const face = createFace();
            face.numVertices = 3;
            face.vertices[0] = 0;
            face.vertices[1] = 0;
            face.vertices[2] = 0;
            face.vertices[3] = 1;
            face.vertices[4] = 0;
            face.vertices[5] = 0;
            face.vertices[6] = 0.5;
            face.vertices[7] = 1;
            face.vertices[8] = 0;
            hit.faceB = face;

            vec3.set(hit.penetrationAxis, 0, -1, 0);
            vec3.set(hit.pointB, 0.5, 0, 0);

            collector.addHit(hit);
            collector.reset();

            // delayed results should be cleared
            collector.flush();
            expect(chainedCollector.hits.length).toBe(0);
        });
    });

    describe('early out', () => {
        it('should propagate early out from chained collector', () => {
            const chainedCollector = createAllCollideShapeCollector();
            const collector = new InternalEdgeRemovingCollector();
            collector.set(chainedCollector);

            expect(collector.shouldEarlyOut()).toBe(false);

            // make chained collector want to early out
            chainedCollector.earlyOutFraction = 0;

            // update should happen on next chain call
            const hit = createCollideShapeHit();
            hit.bodyIdB = 1;
            hit.penetration = 1.0;
            hit.subShapeIdA = EMPTY_SUB_SHAPE_ID;
            hit.subShapeIdB = EMPTY_SUB_SHAPE_ID;

            const face = createFace();
            face.numVertices = 3;
            face.vertices[0] = 0;
            face.vertices[1] = 0;
            face.vertices[2] = 0;
            face.vertices[3] = 1;
            face.vertices[4] = 0;
            face.vertices[5] = 0;
            face.vertices[6] = 0.5;
            face.vertices[7] = 1;
            face.vertices[8] = 0;
            hit.faceB = face;

            vec3.set(hit.penetrationAxis, 0, 0, -1);
            vec3.set(hit.pointB, 0.5, 0.3, 0);

            collector.addHit(hit);

            // early out fraction should be synced
            expect(collector.earlyOutFraction).toBe(0);
        });
    });
});
