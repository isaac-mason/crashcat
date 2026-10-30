import { mat4, quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, capsule, EMPTY_SUB_SHAPE_ID } from '../../src';
import {
    type ContactManifold,
    createContactManifold,
    getWorldSpaceContactPointOnA,
    getWorldSpaceContactPointOnB,
    manifoldBetweenTwoFaces,
    pruneContactPoints,
} from '../../src/manifold/manifold';
import { getShapeSupportingFace } from '../../src/shapes/shapes';
import { createFace, type Face } from '../../src/utils/face';

function createFaceWithVertices(vertices: number[]): Face {
    const face = createFace();
    face.vertices = [...vertices];
    face.numVertices = vertices.length / 3;
    return face;
}

function expectRelativePoint(points: number[], index: number, x: number, y: number, z: number): void {
    expect(points[index * 3]).toBeCloseTo(x, 6);
    expect(points[index * 3 + 1]).toBeCloseTo(y, 6);
    expect(points[index * 3 + 2]).toBeCloseTo(z, 6);
}

/** whether some point of the manifold is (a, b) in world space, a on shape a and b on shape b */
function hasWorldPoint(manifold: ContactManifold, a: number[], b: number[]): boolean {
    const onA = vec3.create();
    const onB = vec3.create();
    for (let i = 0; i < manifold.numContactPoints; i++) {
        getWorldSpaceContactPointOnA(onA, manifold, i);
        getWorldSpaceContactPointOnB(onB, manifold, i);
        if (
            vec3.distance(onA, vec3.fromValues(a[0], a[1], a[2])) < 1e-6 &&
            vec3.distance(onB, vec3.fromValues(b[0], b[1], b[2])) < 1e-6
        ) {
            return true;
        }
    }
    return false;
}

describe('manifoldBetweenTwoFaces', () => {
    test('box face vs box face aligned produces 4 contact points', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: square at z=0 (reference face)
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // Face 2: square at z=0.1 (incident face, penetrating)
        const face2 = createFaceWithVertices([-0.5, -0.5, 0.1, 0.5, -0.5, 0.1, 0.5, 0.5, 0.1, -0.5, 0.5, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // Should produce 4 contact points at corners
        expect(manifold.numContactPoints).toBe(4);
    });

    test('box face vs box face rotated produces clipped polygon', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: square at z=0
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // Face 2: rotated diamond at z=0.1
        const s = 0.5;
        const face2 = createFaceWithVertices([0, -s, 0.1, s, 0, 0.1, 0, s, 0.1, -s, 0, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // Should produce 8 contact points (octagon from clipping)
        expect(manifold.numContactPoints).toBe(8);
    });

    test('box edge vs box face produces 2 contact points', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: edge (2 vertices)
        const face1 = createFaceWithVertices([0, -0.5, 0, 0, 0.5, 0]);

        // Face 2: square
        const face2 = createFaceWithVertices([-0.5, -0.5, 0.1, 0.5, -0.5, 0.1, 0.5, 0.5, 0.1, -0.5, 0.5, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // Edge vs face should produce 2 intersection points
        expect(manifold.numContactPoints).toBe(2);
    });

    test('fallback to original contact point when faces do not overlap', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: square at z=0
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // Face 2: square far away (no overlap after clipping)
        const face2 = createFaceWithVertices([10, 10, 0.1, 11, 10, 0.1, 11, 11, 0.1, 10, 11, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // Should fall back to original contact points
        expect(manifold.numContactPoints).toBe(1);
    });

    test('filters points beyond max contact distance', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: square at z=0
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // Face 2: square at z=0.1 (but we'll use small maxContactDistance to filter most points)
        const face2 = createFaceWithVertices([-0.5, -0.5, 0.1, 0.5, -0.5, 0.1, 0.5, 0.5, 0.1, -0.5, 0.5, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.05; // Only 0.05 units, but penetration is 0.1

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // Should filter out all points and fall back
        expect(manifold.numContactPoints).toBe(1);
    });

    test('fallback when face 1 has too few vertices', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: only 1 vertex (invalid)
        const face1 = createFaceWithVertices([0, 0, 0]);

        // Face 2: valid square
        const face2 = createFaceWithVertices([-0.5, -0.5, 0.1, 0.5, -0.5, 0.1, 0.5, 0.5, 0.1, -0.5, 0.5, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // Should fall back immediately
        expect(manifold.numContactPoints).toBe(1);
    });

    test('face 1 polygon vs face 2 edge swaps the faces and clips the edge', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: valid square
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // face 2: an edge from the square's centre to beyond its +x side
        const face2 = createFaceWithVertices([0, 0, 0.1, 1, 0, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // the part of the edge over the square, x in [0, 0.5]: its ends on the edge (b) and below them on the square (a)
        expect(manifold.numContactPoints).toBe(2);
        expect(hasWorldPoint(manifold, [0, 0, 0], [0, 0, 0.1])).toBe(true);
        expect(hasWorldPoint(manifold, [0.5, 0, 0], [0.5, 0, 0.1])).toBe(true);
    });

    test('capsule lying along a box top face gives the two ends of its resting line', () => {
        // a 2 m box at the origin, and a capsule (radius 0.5, cylinder half height 0.5) turned to lie along x,
        // its bottom line sunk 0.1 into the box's top face
        const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const capsuleShape = capsule.create({ radius: 0.5, halfHeightOfCylinder: 0.5 });
        const capsuleRotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);
        const capsuleTransform = mat4.fromRotationTranslation(mat4.create(), capsuleRotation, vec3.fromValues(0, 1.4, 0));
        const unitScale = vec3.fromValues(1, 1, 1);

        // penetration axis from the box (shape 1) into the capsule (shape 2): the box's face against -axis is its
        // top, the capsule's face against +axis (in its local space, +x) is its bottom line
        const penetrationAxis = vec3.fromValues(0, 1, 0);
        const boxFace = createFace();
        getShapeSupportingFace(boxFace, boxShape, EMPTY_SUB_SHAPE_ID, vec3.fromValues(0, -1, 0), mat4.create(), unitScale);
        const capsuleFace = createFace();
        getShapeSupportingFace(
            capsuleFace,
            capsuleShape,
            EMPTY_SUB_SHAPE_ID,
            vec3.fromValues(1, 0, 0),
            capsuleTransform,
            unitScale,
        );
        expect(boxFace.numVertices).toBe(4);
        expect(capsuleFace.numVertices).toBe(2);

        const manifold = createContactManifold();
        vec3.set(manifold.baseOffset, 0, 1, 0);
        manifoldBetweenTwoFaces(
            manifold,
            vec3.fromValues(0, 1, 0),
            vec3.fromValues(0, 0.9, 0),
            penetrationAxis,
            0.02,
            boxFace,
            capsuleFace,
        );

        // the resting line's ends x = +-0.5 on the capsule (y = 0.9), each with its point 0.1 above on the box's top
        expect(manifold.numContactPoints).toBe(2);
        expect(hasWorldPoint(manifold, [-0.5, 1, 0], [-0.5, 0.9, 0])).toBe(true);
        expect(hasWorldPoint(manifold, [0.5, 1, 0], [0.5, 0.9, 0])).toBe(true);
    });

    test('asserts the max contact distance is positive', () => {
        const manifold = createContactManifold();
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);
        const face2 = createFaceWithVertices([-0.5, -0.5, 0.1, 0.5, -0.5, 0.1, 0.5, 0.5, 0.1, -0.5, 0.5, 0.1]);
        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);

        expect(() => manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, 0, face1, face2)).toThrow();
    });

    test('contact points are stored relative to baseOffset', () => {
        const manifold = createContactManifold();
        const baseOffset = vec3.fromValues(10, 20, 30);
        manifold.baseOffset = baseOffset;

        // Face 1: square at z=0 (world space)
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // Face 2: square at z=0.1 (world space)
        const face2 = createFaceWithVertices([-0.5, -0.5, 0.1, 0.5, -0.5, 0.1, 0.5, 0.5, 0.1, -0.5, 0.5, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        const penetrationAxis = vec3.fromValues(0, 0, 0.1);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        expect(manifold.numContactPoints).toBe(4);

        // Verify all contact points are stored relative to baseOffset
        for (let i = 0; i < manifold.numContactPoints; i++) {
            const idx = i * 3;
            // Points should be relative (world - base)
            const relX1 = manifold.relativeContactPointsOnA[idx];
            const relY1 = manifold.relativeContactPointsOnA[idx + 1];
            const relZ1 = manifold.relativeContactPointsOnA[idx + 2];

            // World points are around [±0.5, ±0.5, 0]
            // baseOffset is [10, 20, 30]
            // So relative should be around [-10.5 to -9.5, -20.5 to -19.5, -30]
            expect(relX1).toBeGreaterThanOrEqual(-10.6);
            expect(relX1).toBeLessThanOrEqual(-9.4);
            expect(relY1).toBeGreaterThanOrEqual(-20.6);
            expect(relY1).toBeLessThanOrEqual(-19.4);
            expect(relZ1).toBeCloseTo(-30, 1);
        }
    });

    test('handles perpendicular penetration axis gracefully', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Face 1: square in XY plane
        const face1 = createFaceWithVertices([-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0]);

        // Face 2: square in XY plane
        const face2 = createFaceWithVertices([-0.5, -0.5, 0.1, 0.5, -0.5, 0.1, 0.5, 0.5, 0.1, -0.5, 0.5, 0.1]);

        const contactPoint1 = vec3.fromValues(0, 0, 0);
        const contactPoint2 = vec3.fromValues(0, 0, 0.1);
        // Penetration axis perpendicular to face normal (in XY plane)
        const penetrationAxis = vec3.fromValues(1, 0, 0);
        const maxContactDistance = 0.2;

        manifoldBetweenTwoFaces(manifold, contactPoint1, contactPoint2, penetrationAxis, maxContactDistance, face1, face2);

        // Should fall back to original contact point when axis perpendicular to normal
        expect(manifold.numContactPoints).toBe(1);
    });
});

describe('pruneContactPoints (separate function)', () => {
    test('asserts early if <= 4 points', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Create manifold with exactly 4 points
        for (let i = 0; i < 4; i++) {
            const p1 = vec3.fromValues(i - 1.5, i - 1.5, 0);
            const p2 = vec3.fromValues(i - 1.5, i - 1.5, 0.01);
            const idx = i * 3;
            manifold.relativeContactPointsOnA[idx] = p1[0];
            manifold.relativeContactPointsOnA[idx + 1] = p1[1];
            manifold.relativeContactPointsOnA[idx + 2] = p1[2];
            manifold.relativeContactPointsOnB[idx] = p2[0];
            manifold.relativeContactPointsOnB[idx + 1] = p2[1];
            manifold.relativeContactPointsOnB[idx + 2] = p2[2];
        }
        manifold.numContactPoints = 4;

        const penetrationAxis = vec3.fromValues(0, 0, 1);
        const beforeCount = manifold.numContactPoints;
        expect(() => {
            pruneContactPoints(manifold, penetrationAxis);
        }).toThrow();

        // Should not be modified (early exit)
        expect(manifold.numContactPoints).toBe(beforeCount);
    });

    test('reduces 8 points to 4 points', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Create 8 points arranged in a pattern
        const points = [
            vec3.fromValues(-1, -1, 0),
            vec3.fromValues(1, -1, 0),
            vec3.fromValues(1, 1, 0),
            vec3.fromValues(-1, 1, 0),
            vec3.fromValues(0, -1, 0),
            vec3.fromValues(1, 0, 0),
            vec3.fromValues(0, 1, 0),
            vec3.fromValues(-1, 0, 0),
        ];

        for (let i = 0; i < points.length; i++) {
            const p1 = points[i];
            const p2 = vec3.clone(p1);
            vec3.add(p2, p2, vec3.fromValues(0, 0, 0.02));
            const idx = i * 3;
            manifold.relativeContactPointsOnA[idx] = p1[0];
            manifold.relativeContactPointsOnA[idx + 1] = p1[1];
            manifold.relativeContactPointsOnA[idx + 2] = p1[2];
            manifold.relativeContactPointsOnB[idx] = p2[0];
            manifold.relativeContactPointsOnB[idx + 1] = p2[1];
            manifold.relativeContactPointsOnB[idx + 2] = p2[2];
        }
        manifold.numContactPoints = 8;

        const penetrationAxis = vec3.fromValues(0, 0, 1);
        pruneContactPoints(manifold, penetrationAxis);

        // the corners: p1 the first deepest-widest (-1, -1), p2 the one opposite, p3 and p4 either side of the
        // p1-p2 diagonal, kept in winding order [p1, p3, p2, p4]
        expect(manifold.numContactPoints).toBe(4);
        expectRelativePoint(manifold.relativeContactPointsOnA, 0, -1, -1, 0);
        expectRelativePoint(manifold.relativeContactPointsOnA, 1, -1, 1, 0);
        expectRelativePoint(manifold.relativeContactPointsOnA, 2, 1, 1, 0);
        expectRelativePoint(manifold.relativeContactPointsOnA, 3, 1, -1, 0);
        expectRelativePoint(manifold.relativeContactPointsOnB, 0, -1, -1, 0.02);
        expectRelativePoint(manifold.relativeContactPointsOnB, 1, -1, 1, 0.02);
        expectRelativePoint(manifold.relativeContactPointsOnB, 2, 1, 1, 0.02);
        expectRelativePoint(manifold.relativeContactPointsOnB, 3, 1, -1, 0.02);
    });

    test('asserts the penetration axis is normalized', () => {
        const manifold = createContactManifold();
        for (let i = 0; i < 5; i++) {
            manifold.relativeContactPointsOnA[i * 3] = i;
            manifold.relativeContactPointsOnB[i * 3] = i;
            manifold.relativeContactPointsOnB[i * 3 + 2] = 0.01;
        }
        manifold.numContactPoints = 5;

        expect(() => pruneContactPoints(manifold, vec3.fromValues(0, 0, 2))).toThrow();
    });

    // test('handles 1-3 points gracefully (no pruning needed)', () => {
    //     for (let count = 1; count <= 3; count++) {
    //         const manifold = createContactManifold();
    //         manifold.baseOffset = vec3.fromValues(0, 0, 0);

    //         for (let i = 0; i < count; i++) {
    //             const p1 = vec3.fromValues(i - 1, i - 1, 0);
    //             const p2 = vec3.fromValues(i - 1, i - 1, 0.01);
    //             const idx = i * 3;
    //             manifold.relativeContactPointsOnA[idx] = p1[0];
    //             manifold.relativeContactPointsOnA[idx + 1] = p1[1];
    //             manifold.relativeContactPointsOnA[idx + 2] = p1[2];
    //             manifold.relativeContactPointsOnB[idx] = p2[0];
    //             manifold.relativeContactPointsOnB[idx + 1] = p2[1];
    //             manifold.relativeContactPointsOnB[idx + 2] = p2[2];
    //         }
    //         manifold.numContactPoints = count;

    //         const penetrationAxis = vec3.fromValues(0, 0, 1);
    //         pruneContactPoints(manifold, penetrationAxis);

    //         // Verify no crash and correct count maintained (early exit)
    //         expect(manifold.numContactPoints).toBe(count);
    //     }
    // });

    test('preserves relative coordinates after pruning', () => {
        const manifold = createContactManifold();
        const baseOffset = vec3.fromValues(10, 20, 30);
        manifold.baseOffset = vec3.clone(baseOffset);

        // Set up 6 points with specific relative coordinates
        for (let i = 0; i < 6; i++) {
            const angle = (i / 6) * Math.PI * 2;
            const relP1 = vec3.fromValues(Math.cos(angle) * 0.5, Math.sin(angle) * 0.5, 0);
            const relP2 = vec3.clone(relP1);
            vec3.add(relP2, relP2, vec3.fromValues(0, 0, 0.01));

            const idx = i * 3;
            manifold.relativeContactPointsOnA[idx] = relP1[0];
            manifold.relativeContactPointsOnA[idx + 1] = relP1[1];
            manifold.relativeContactPointsOnA[idx + 2] = relP1[2];
            manifold.relativeContactPointsOnB[idx] = relP2[0];
            manifold.relativeContactPointsOnB[idx + 1] = relP2[1];
            manifold.relativeContactPointsOnB[idx + 2] = relP2[2];
        }
        manifold.numContactPoints = 6;

        const penetrationAxis = vec3.fromValues(0, 0, 1);
        pruneContactPoints(manifold, penetrationAxis);

        // baseOffset should remain unchanged
        expect(manifold.baseOffset).toEqual(baseOffset);
    });

    test('selects points that maximize metric (distance × depth)', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // 4 corners with high metric + 4 edge points with lower metric
        const points = [
            vec3.fromValues(-1, -1, 0), // corner - high metric
            vec3.fromValues(1, -1, 0), // corner - high metric
            vec3.fromValues(1, 1, 0), // corner - high metric
            vec3.fromValues(-1, 1, 0), // corner - high metric
            vec3.fromValues(0, -1, 0), // edge - lower metric
            vec3.fromValues(1, 0, 0), // edge - lower metric
            vec3.fromValues(0, 1, 0), // edge - lower metric
            vec3.fromValues(-1, 0, 0), // edge - lower metric
        ];

        for (let i = 0; i < points.length; i++) {
            const p1 = points[i];
            const p2 = vec3.clone(p1);
            vec3.add(p2, p2, vec3.fromValues(0, 0, 0.02));
            const idx = i * 3;
            manifold.relativeContactPointsOnA[idx] = p1[0];
            manifold.relativeContactPointsOnA[idx + 1] = p1[1];
            manifold.relativeContactPointsOnA[idx + 2] = p1[2];
            manifold.relativeContactPointsOnB[idx] = p2[0];
            manifold.relativeContactPointsOnB[idx + 1] = p2[1];
            manifold.relativeContactPointsOnB[idx + 2] = p2[2];
        }
        manifold.numContactPoints = 8;

        const penetrationAxis = vec3.fromValues(0, 0, 1);
        pruneContactPoints(manifold, penetrationAxis);

        // Should select 4 points (likely the corners for maximum area)
        expect(manifold.numContactPoints).toBe(4);
    });

    test('handles collinear points', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // 5 points on a line from (-1,0,0) to (1,0,0)
        const points = [
            vec3.fromValues(-1, 0, 0),
            vec3.fromValues(-0.5, 0, 0),
            vec3.fromValues(0, 0, 0),
            vec3.fromValues(0.5, 0, 0),
            vec3.fromValues(1, 0, 0),
        ];

        for (let i = 0; i < points.length; i++) {
            const p1 = points[i];
            const p2 = vec3.clone(p1);
            vec3.add(p2, p2, vec3.fromValues(0, 0, 0.01));
            const idx = i * 3;
            manifold.relativeContactPointsOnA[idx] = p1[0];
            manifold.relativeContactPointsOnA[idx + 1] = p1[1];
            manifold.relativeContactPointsOnA[idx + 2] = p1[2];
            manifold.relativeContactPointsOnB[idx] = p2[0];
            manifold.relativeContactPointsOnB[idx + 1] = p2[1];
            manifold.relativeContactPointsOnB[idx + 2] = p2[2];
        }
        manifold.numContactPoints = 5;

        const penetrationAxis = vec3.fromValues(0, 0, 1);
        pruneContactPoints(manifold, penetrationAxis);

        // When points are collinear, point3 and point4 are -1 (not found)
        // So only point1 and point2 are kept = 2 points
        expect(manifold.numContactPoints).toBe(2);
    });

    test('metric correctly weights distance and penetration depth', () => {
        const manifold = createContactManifold();
        manifold.baseOffset = vec3.fromValues(0, 0, 0);

        // Near point: distance=0.5, depth=0.5 → metric=(0.25 × 0.25) = 0.0625
        // Far point: distance=2, depth=2 → metric=(4 × 4) = 16
        // Algorithm should prefer the far point

        const points = [
            vec3.fromValues(0.5, 0, 0), // near point
            vec3.fromValues(2, 0, 0), // far point
            vec3.fromValues(0.5, 0.5, 0),
            vec3.fromValues(2, 2, 0),
            vec3.fromValues(-0.5, 0, 0),
            vec3.fromValues(-2, 0, 0),
            vec3.fromValues(-0.5, -0.5, 0),
            vec3.fromValues(-2, -2, 0),
        ];

        for (let i = 0; i < points.length; i++) {
            const p1 = points[i];
            const p2 = vec3.clone(p1);
            // Vary penetration depth based on distance
            const depth = vec3.length(p1) * 0.1;
            vec3.add(p2, p2, vec3.fromValues(0, 0, depth));
            const idx = i * 3;
            manifold.relativeContactPointsOnA[idx] = p1[0];
            manifold.relativeContactPointsOnA[idx + 1] = p1[1];
            manifold.relativeContactPointsOnA[idx + 2] = p1[2];
            manifold.relativeContactPointsOnB[idx] = p2[0];
            manifold.relativeContactPointsOnB[idx + 1] = p2[1];
            manifold.relativeContactPointsOnB[idx + 2] = p2[2];
        }
        manifold.numContactPoints = 8;

        const penetrationAxis = vec3.fromValues(0, 0, 1);
        pruneContactPoints(manifold, penetrationAxis);

        // Should reduce to 4 points
        expect(manifold.numContactPoints).toBe(4);
    });
});
