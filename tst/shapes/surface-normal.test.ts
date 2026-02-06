import { quat, vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { box, compound, convexHull, scaled, sphere, transformed, triangleMesh, subShape, getShapeSurfaceNormal } from '../../src';

describe('Surface Normal - Sphere', () => {
    test('should return normal pointing away from center', () => {
        const shape = sphere.create({ radius: 2.0 });
        const position = vec3.fromValues(1, 0, 0); // on surface
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // Should point in direction of position (outward)
        expect(normal[0]).toBeCloseTo(1);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(0);
    });

    test('should handle position at arbitrary location', () => {
        const shape = sphere.create({ radius: 1.0 });
        const position = vec3.fromValues(1, 1, 0);
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // Should be normalized and point in direction of position
        const expectedLength = Math.sqrt(2);
        expect(normal[0]).toBeCloseTo(1 / expectedLength, 5);
        expect(normal[1]).toBeCloseTo(1 / expectedLength, 5);
        expect(normal[2]).toBeCloseTo(0);
        
        // Should be unit length
        const length = vec3.length(normal);
        expect(length).toBeCloseTo(1, 5);
    });

    test('should handle fallback when position is at origin', () => {
        const shape = sphere.create({ radius: 1.0 });
        const position = vec3.fromValues(0, 0, 0);
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // Should fallback to Y axis
        expect(normal[0]).toBeCloseTo(0);
        expect(normal[1]).toBeCloseTo(1);
        expect(normal[2]).toBeCloseTo(0);
    });
});

describe('Surface Normal - Box', () => {
    test('should return X face normal when closest to X face', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(2, 1, 1) });
        const position = vec3.fromValues(2.1, 0, 0); // slightly outside +X face
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        expect(normal[0]).toBeCloseTo(1);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(0);
    });

    test('should return Y face normal when closest to Y face', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 2, 1) });
        const position = vec3.fromValues(0, -2.1, 0); // slightly outside -Y face
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        expect(normal[0]).toBeCloseTo(0);
        expect(normal[1]).toBeCloseTo(-1);
        expect(normal[2]).toBeCloseTo(0);
    });

    test('should return Z face normal when closest to Z face', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 2) });
        const position = vec3.fromValues(0, 0, 2.1); // slightly outside +Z face
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        expect(normal[0]).toBeCloseTo(0);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(1);
    });

    test('should handle corner positions correctly', () => {
        const shape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const position = vec3.fromValues(1.1, 0.5, 0.5); // closest to +X face
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // Should return X face normal since X distance is smallest
        expect(normal[0]).toBeCloseTo(1);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(0);
    });
});

describe('Surface Normal - Triangle Mesh', () => {
    test('should return triangle normal for valid triangle index', () => {
        // Create a simple triangle in XY plane facing +Z
        const shape = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });
        
        // Build SubShapeID for triangle 0
        const builder = subShape.builder();
        subShape.pushIndex(builder, builder, 0, 1); // triangle index 0 of 1 triangles
        
        const position = vec3.fromValues(0.3, 0.3, 0);
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, builder.value);

        // Should return the triangle's normal (pointing in +Z direction)
        expect(normal[0]).toBeCloseTo(0);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(1);
    });

    // test('should assert on invalid triangle index', () => {
    //     const shape = triangleMesh.create({
    //         positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
    //         indices: [0, 1, 2],
    //     });
        
    //     const position = vec3.fromValues(0.5, 0.5, 0);
    //     const normal = vec3.create();

    //     // Use invalid SubShapeID (empty) - should assert
    //     expect(() => {
    //         computeShapeSurfaceNormal(normal, shape, position, subShapeId.EMPTY_SUB_SHAPE_ID);
    //     }).toThrow();
    // });
});

describe('Surface Normal - Scaled Shape', () => {
    test('should return correct normal for scaled box', () => {
        const innerShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const shape = scaled.create({ shape: innerShape, scale: vec3.fromValues(2, 3, 1) });
        const position = vec3.fromValues(2.1, 1, 0); // outside +X face of scaled box
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // Should still point in +X direction (normals don't scale)
        expect(normal[0]).toBeCloseTo(1);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(0);
    });

    test('should work with scaled sphere', () => {
        const innerShape = sphere.create({ radius: 1.0 });
        const shape = scaled.create({ shape: innerShape, scale: vec3.fromValues(2, 2, 2) });
        const position = vec3.fromValues(2, 2, 0); // on surface of scaled sphere
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // Should be normalized and point outward
        const expectedLength = Math.sqrt(8);
        expect(normal[0]).toBeCloseTo(2 / expectedLength, 5);
        expect(normal[1]).toBeCloseTo(2 / expectedLength, 5);
        expect(normal[2]).toBeCloseTo(0);
        
        const length = vec3.length(normal);
        expect(length).toBeCloseTo(1, 5);
    });
});

describe('Surface Normal - Transformed Shape', () => {
    test('should return correct normal for translated box', () => {
        const innerShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const transformedPosition = vec3.fromValues(3, 2, 1);
        const transformedQuaternion = quat.create();
        const shape = transformed.create({ shape: innerShape, position: transformedPosition, quaternion: transformedQuaternion });
        
        const position = vec3.fromValues(4.1, 2, 1); // outside +X face in world space
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // Should point in +X direction in world space
        expect(normal[0]).toBeCloseTo(1);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(0);
    });

    test('should return correct normal for rotated box', () => {
        const innerShape = box.create({ halfExtents: vec3.fromValues(2, 1, 1) });
        const transformedPosition = vec3.fromValues(0, 0, 0);
        // 90 degree rotation around Z axis: +X face becomes +Y face
        const transformedQuaternion = quat.fromValues(0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4));
        const shape = transformed.create({ shape: innerShape, position: transformedPosition, quaternion: transformedQuaternion });
        
        const position = vec3.fromValues(0, 2.1, 0); // outside what is now the +Y face in world space
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);

        // The original +X face (normal [1,0,0]) is now rotated to be the +Y face
        // So we expect normal to point in +Y direction
        expect(normal[0]).toBeCloseTo(0, 5);
        expect(normal[1]).toBeCloseTo(1, 5);
        expect(normal[2]).toBeCloseTo(0);
    });
});

describe('Surface Normal - Compound Shape', () => {
    test('should return normal from correct child shape', () => {
        const box1 = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const box2 = box.create({ halfExtents: vec3.fromValues(2, 2, 2) });
        const shape = compound.create({ children: [
            { position: vec3.create(), quaternion: quat.create(), shape: box1 },
            { position: vec3.create(), quaternion: quat.create(), shape: box2 },
        ] });
        
        // Build SubShapeID for child 1 (the larger box)
        const builder = subShape.builder();
        subShape.pushIndex(builder, builder, 1, shape.children.length);
        
        const position = vec3.fromValues(2.1, 0, 0); // outside +X face of larger box
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, shape, position, builder.value);

        // Should get normal from the larger box
        expect(normal[0]).toBeCloseTo(1);
        expect(normal[1]).toBeCloseTo(0);
        expect(normal[2]).toBeCloseTo(0);
    });

    test('compound shape -> transformed shape -> box shape', () => {
        // Create the hierarchy: compound containing a transformed box
        const innerBox = box.create({ halfExtents: vec3.fromValues(1, 2, 1) });
        
        // Transform: translate and rotate the box
        const translation = vec3.fromValues(5, 3, 0);
        // 90 degree rotation around Z: +X becomes +Y, +Y becomes -X
        const rotation = quat.fromValues(0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4));
        
        // Add another shape to make it actually compound
        const sphere1 = sphere.create({ radius: 1.0 });
        const compoundShape = compound.create({ children: [
            { position: vec3.create(), quaternion: quat.create(), shape: sphere1 },
            { position: translation, quaternion: rotation, shape: innerBox },
        ] });
        
        // Build SubShapeID: compound child 1 (the transformed box)
        const builder = subShape.builder();
        subShape.pushIndex(builder, builder, 1, compoundShape.children.length);
        
        // Position outside the +Y face of the rotated box in world coordinates
        // Original box +X face (at x=1) becomes +Y face after rotation
        // After translation by (5,3,0), the +Y face is at world y = 3 + 1 = 4
        const position = vec3.fromValues(5, 4.1, 0);
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, compoundShape, position, builder.value);

        // The original +X face normal [1,0,0] rotated by 90° around Z becomes [0,1,0]
        expect(normal[0]).toBeCloseTo(0, 5);
        expect(normal[1]).toBeCloseTo(1, 5);
        expect(normal[2]).toBeCloseTo(0);
    });

    test('should work with valid empty SubShapeID for single child', () => {
        const box1 = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        const shape = compound.create({ children: [
            { position: vec3.create(), quaternion: quat.create(), shape: box1 },
        ] });
        
        const position = vec3.fromValues(1, 1, 1);
        const normal = vec3.create();

        // With 1 child, subShapeId.EMPTY_SUB_SHAPE_ID when popped with 0 bits gives index 0, which is valid
        getShapeSurfaceNormal(normal, shape, position, subShape.EMPTY_SUB_SHAPE_ID);
        
        // Should return a valid normal from the box
        const length = vec3.length(normal);
        expect(length).toBeCloseTo(1, 5); // Should be unit length
    });
});

// describe('Surface Normal - Complex Hierarchies', () => {
//     test('should handle nested transformations correctly', () => {
//         // Create: compound -> scaled shape -> transformed shape -> box
//         const innerBox = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        
//         // Transform: rotate 90° around Z
//         const rotation = quat.fromValues(0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4));
//         const transformedShape = transformed.create({ 
//             shape: innerBox, 
//             translation: vec3.fromValues(2, 0, 0),
//             rotation 
//         });
        
//         // Scale the transformed shape
//         const scaledShape = scaled.create({ 
//             shape: transformedShape, 
//             scale: vec3.fromValues(2, 2, 2) 
//         });
        
//         // Add to compound
//         const sphere1 = sphere.create({ radius: 0.5 });
//         const compoundShape = compound.create({ children: [
//             { position: vec3.create(), quaternion: quat.create(), shape: sphere1 },
//             { position: vec3.create(), quaternion: quat.create(), shape: scaledShape },
//         ] });
        
//         // Build SubShapeID for the scaled transformed box
//         const builder = subShapeId.builder();
//         subShapeId.pushIndex(builder, builder, 1, compoundShape.children.length);
        
//         // Let me trace this more carefully:
//         // 1. Box at origin with halfExtents (1,1,1)
//         // 2. Rotate 90° around Z: +X face becomes +Y face, -X becomes -Y, +Y becomes -X, -Y becomes +X  
//         // 3. Translate by (2,0,0): box center moves to (2,0,0)
//         // 4. Scale by (2,2,2): box halfExtents become (2,2,2), center stays at (2,0,0)
//         // 
//         // So final box faces are at:
//         // - +X face: x = 2 + 2 = 4 (was +Y face originally) 
//         // - -X face: x = 2 - 2 = 0 (was -Y face originally)
//         // - +Y face: y = 0 + 2 = 2 (was -X face originally)  
//         // - -Y face: y = 0 - 2 = -2 (was +X face originally)
//         // 
//         // Position (2, 2.1, 0) hits the +Y face (y=2.1 > 2)
//         // This face corresponds to the original -X face of the box
//         // So we should get the original -X normal rotated and scaled
        
//         const position = vec3.fromValues(2, 2.1, 0);
//         const normal = vec3.create();

//         getShapeSurfaceNormal(normal, compoundShape, position, builder.value);

//         // After all transforms:
//         // - Box gets rotated 90° around Z (+X → +Y, +Y → -X)
//         // - Then translated by (2, 0, 0)
//         // - Then scaled by 2
//         // 
//         // Position (2, 2.1, 0) after inverse transforms:
//         // - Unscale: (1, 1.05, 0)
//         // - Untranslate: (-1, 1.05, 0)
//         // - Unrotate by -90°Z: (1.05, 1, 0)
//         //
//         // In box space (1.05, 1, 0) is closest to +X face at x=1
//         // Box normal (+1, 0, 0) transformed back:
//         // - Rotate by +90°Z: (0, 1, 0)
        
//         expect(normal[0]).toBeCloseTo(0, 5);
//         expect(normal[1]).toBeCloseTo(1, 5);
//         expect(normal[2]).toBeCloseTo(0);
//     });

//     test('should handle compound with triangle mesh', () => {
//         // Create triangle mesh
//         const mesh = triangleMesh.create({
//             positions: [0, 0, 0, 1, 0, 0, 0, 1, 0], // triangle in XY plane, normal +Z
//             indices: [0, 1, 2],
//         });
        
//         const sphere1 = sphere.create({ radius: 1.0 });
//         const compoundShape = compound.create({ children: [
//             { position: vec3.create(), quaternion: quat.create(), shape: sphere1 },
//             { position: vec3.create(), quaternion: quat.create(), shape: mesh },
//         ] });
        
//         // Build SubShapeID: compound child 1 -> triangle 0
//         const builder1 = subShapeId.builder();
//         const builder2 = subShapeId.builder();
        
//         // First level: compound child 1 (the triangle mesh)
//         subShapeId.pushIndex(builder1, builder2, 1, compoundShape.children.length);
//         // Second level: triangle 0 within the mesh
//         subShapeId.pushIndex(builder2, builder1, 0, 1);
        
//         const position = vec3.fromValues(0.3, 0.3, 0.1);
//         const normal = vec3.create();

//         getShapeSurfaceNormal(normal, compoundShape, position, builder2.value);

//         // Should get the triangle's normal (+Z)
//         expect(normal[0]).toBeCloseTo(0);
//         expect(normal[1]).toBeCloseTo(0);
//         expect(normal[2]).toBeCloseTo(1);
//     });
// });


// describe('getSurfaceNormal', () => {
//     test('sphere: normal at surface point equals normalized position', () => {
//         const s = sphere.create({ radius: 1.0 });
//         const pos = vec3.fromValues(1, 0, 0);
//         const normal = computeShapeSurfaceNormal(vec3.create(), s, pos, subShapeId.EMPTY_SUB_SHAPE_ID);

//         // For sphere, normal = normalized position
//         expect(normal[0]).toBeCloseTo(1, 5);
//         expect(normal[1]).toBeCloseTo(0, 5);
//         expect(normal[2]).toBeCloseTo(0, 5);
//     });

//     test('sphere: normal at arbitrary surface point', () => {
//         const s = sphere.create({ radius: 1.0 });
//         const pos = vec3.fromValues(1, 1, 1);
//         const normal = computeShapeSurfaceNormal(vec3.create(), s, pos, subShapeId.EMPTY_SUB_SHAPE_ID);

//         // For sphere, normal = normalized position
//         const len = Math.sqrt(1 + 1 + 1);
//         expect(normal[0]).toBeCloseTo(1 / len, 5);
//         expect(normal[1]).toBeCloseTo(1 / len, 5);
//         expect(normal[2]).toBeCloseTo(1 / len, 5);
//     });

//     test('sphere: normal at origin returns fallback Y axis', () => {
//         const s = sphere.create({ radius: 1.0 });
//         const pos = vec3.fromValues(0, 0, 0);
//         const normal = computeShapeSurfaceNormal(vec3.create(), s, pos, subShapeId.EMPTY_SUB_SHAPE_ID);

//         expect(normal[0]).toBeCloseTo(0, 5);
//         expect(normal[1]).toBeCloseTo(1, 5);
//         expect(normal[2]).toBeCloseTo(0, 5);
//     });

//     test('box: normal on positive X face', () => {
//         const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
//         const pos = vec3.fromValues(1, 0, 0); // On X face
//         const normal = computeShapeSurfaceNormal(vec3.create(), b, pos, 0);

//         expect(normal[0]).toBeCloseTo(1, 5);
//         expect(normal[1]).toBeCloseTo(0, 5);
//         expect(normal[2]).toBeCloseTo(0, 5);
//     });

//     test('box: normal on negative Y face', () => {
//         const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
//         const pos = vec3.fromValues(0, -1, 0); // On -Y face
//         const normal = computeShapeSurfaceNormal(vec3.create(), b, pos, 0);

//         expect(normal[0]).toBeCloseTo(0, 5);
//         expect(normal[1]).toBeCloseTo(-1, 5);
//         expect(normal[2]).toBeCloseTo(0, 5);
//     });

//     test('box: normal at corner (dominant axis closest to surface)', () => {
//         const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
//         const pos = vec3.fromValues(1, 1, 1); // At corner
//         const normal = computeShapeSurfaceNormal(vec3.create(), b, pos, 0);

//         // Should pick one axis (all equally close), first one wins
//         const normalLen = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
//         expect(normalLen).toBeCloseTo(1, 5);
//     });

//     test('triangle mesh: with subShapeId returns specific triangle normal', () => {
//         const mesh = triangleMesh.create({
//             positions: [
//                 0, 0, 0,
//                 1, 0, 0,
//                 0, 1, 0,
//                 1, 0, 1,
//             ],
//             indices: [0, 1, 2, 1, 2, 3],
//         });

//         // Triangle 0: (0,0,0), (1,0,0), (0,1,0) - normal should be (0, 0, 1) or (0, 0, -1)
//         const normal0 = computeShapeSurfaceNormal(vec3.create(), mesh, vec3.create(), 0);
//         expect(Math.abs(normal0[2])).toBeCloseTo(1, 5);

//         // Triangle 1: different vertices
//         const normal1 = computeShapeSurfaceNormal(vec3.create(), mesh, vec3.create(), 1);
//         const len = vec3.length(normal1);
//         expect(len).toBeCloseTo(1, 5);
//     });

//     test('triangle mesh: without subShapeId returns average normal', () => {
//         const mesh = triangleMesh.create({
//             positions: [
//                 0, 0, 0,
//                 1, 0, 0,
//                 0, 1, 0,
//             ],
//             indices: [0, 1, 2],
//         });

//         const normalAvg = computeShapeSurfaceNormal(vec3.create(), mesh, vec3.create(), -1);
//         const len = vec3.length(normalAvg);
//         expect(len).toBeCloseTo(1, 5);
//     });

//     test('triangle mesh: invalid subShapeId returns average normal', () => {
//         const mesh = triangleMesh.create({
//             positions: [
//                 0, 0, 0,
//                 1, 0, 0,
//                 0, 1, 0,
//             ],
//             indices: [0, 1, 2],
//         });

//         const normalInvalid = computeShapeSurfaceNormal(vec3.create(), mesh, vec3.create(), 999);
//         const len = vec3.length(normalInvalid);
//         expect(len).toBeCloseTo(1, 5);
//     });

//     test('scaled shape: delegates to inner shape', () => {
//         const s = sphere.create({ radius: 1.0 });
//         const ss = scaled.create({ shape: s, scale: vec3.fromValues(2, 2, 2) });
//         const pos = vec3.fromValues(1, 0, 0);

//         const normalScaled = computeShapeSurfaceNormal(vec3.create(), ss, pos, 0);
//         const normalOrig = computeShapeSurfaceNormal(vec3.create(), s, pos, subShapeId.EMPTY_SUB_SHAPE_ID);

//         // Normal should be same (not affected by scale)
//         expect(normalScaled[0]).toBeCloseTo(normalOrig[0], 5);
//         expect(normalScaled[1]).toBeCloseTo(normalOrig[1], 5);
//         expect(normalScaled[2]).toBeCloseTo(normalOrig[2], 5);
//     });

//     test('transformed shape: transforms position and normal correctly', () => {
//         const s = sphere.create({ radius: 1.0 });
//         const ts = transformed.create({
//             shape: s,
//             translation: vec3.fromValues(0, 0, 0),
//             rotation: quat.create(), // Identity rotation
//         });
//         const pos = vec3.fromValues(1, 0, 0);

//         const normalTrans = computeShapeSurfaceNormal(vec3.create(), ts, pos, 0);
//         const normalOrig = computeShapeSurfaceNormal(vec3.create(), s, pos, subShapeId.EMPTY_SUB_SHAPE_ID);

//         // With identity rotation and zero translation, should be same
//         expect(normalTrans[0]).toBeCloseTo(normalOrig[0], 5);
//         expect(normalTrans[1]).toBeCloseTo(normalOrig[1], 5);
//         expect(normalTrans[2]).toBeCloseTo(normalOrig[2], 5);
//     });

//     test('compound shape: with valid subShapeId uses that child', () => {
//         const s = sphere.create({ radius: 1.0 });
//         const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
//         const cs = compound.create({ shapes: [s, b] });

describe('Surface Normal - ConvexHull', () => {
    test('should return plane normal for box hull - +X direction', () => {
        const boxPoints = [
            -1, -1, -1,
            1, -1, -1,
            -1, 1, -1,
            1, 1, -1,
            -1, -1, 1,
            1, -1, 1,
            -1, 1, 1,
            1, 1, 1,
        ];
        const hull = convexHull.create({ positions: boxPoints, convexRadius: 0.0 });
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, hull, vec3.fromValues(1, 0, 0), subShape.EMPTY_SUB_SHAPE_ID);

        // Should return a normalized normal
        const len = vec3.length(normal);
        expect(len).toBeCloseTo(1, 5);
        
        // Should be along one of the axes
        const absX = Math.abs(normal[0]);
        const absY = Math.abs(normal[1]);
        const absZ = Math.abs(normal[2]);
        const axisCount = (absX > 0.9 ? 1 : 0) + (absY > 0.9 ? 1 : 0) + (absZ > 0.9 ? 1 : 0);
        expect(axisCount).toBe(1);
    });

    test('should return plane normal for box hull - +Y direction', () => {
        const boxPoints = [
            -1, -1, -1,
            1, -1, -1,
            -1, 1, -1,
            1, 1, -1,
            -1, -1, 1,
            1, -1, 1,
            -1, 1, 1,
            1, 1, 1,
        ];
        const hull = convexHull.create({ positions: boxPoints, convexRadius: 0.0 });
        const normal = vec3.create();

        getShapeSurfaceNormal(normal, hull, vec3.fromValues(0, 1, 0), subShape.EMPTY_SUB_SHAPE_ID);

        // Should return a normalized normal
        const len = vec3.length(normal);
        expect(len).toBeCloseTo(1, 5);
    });

    test('should select best plane based on dot product', () => {
        const boxPoints = [
            -1, -1, -1,
            1, -1, -1,
            -1, 1, -1,
            1, 1, -1,
            -1, -1, 1,
            1, -1, 1,
            -1, 1, 1,
            1, 1, 1,
        ];
        const hull = convexHull.create({ positions: boxPoints, convexRadius: 0.0 });
        const normal = vec3.create();

        // Position at origin should select a plane
        getShapeSurfaceNormal(normal, hull, vec3.fromValues(0, 0, 0), subShape.EMPTY_SUB_SHAPE_ID);

        // Should still return a unit normal
        const len = vec3.length(normal);
        expect(len).toBeCloseTo(1, 5);
    });
});

// Commented out tests below

//         const normal0 = computeShapeSurfaceNormal(vec3.create(), cs, vec3.fromValues(1, 0, 0), 0);
//         expect(Math.abs(normal0[0])).toBeCloseTo(1, 5);

//         // subShapeId 1 = box
//         const normal1 = computeShapeSurfaceNormal(vec3.create(), cs, vec3.fromValues(1, 0, 0), 1);
//         expect(Math.abs(normal1[0])).toBeCloseTo(1, 5);
//     });

//     test('compound shape: without subShapeId finds closest child', () => {
//         const s = sphere.create({ radius: 1.0 });
//         const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
//         const cs = compound.create({ shapes: [s, b] });

//         const normal = computeShapeSurfaceNormal(vec3.create(), cs, vec3.fromValues(0.5, 0, 0), -1);
//         const len = vec3.length(normal);
//         expect(len).toBeCloseTo(1, 5);
//     });

//     // test('compound shape: invalid subShapeId finds closest child', () => {
//     //     const s = sphere.create({ radius: 1.0 });
//     //     const b = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
//     //     const cs = compound.create({ shapes: [s, b] });

//     //     const normal = computeShapeSurfaceNormal(vec3.create(), cs, vec3.fromValues(0.5, 0, 0), 999);
//     //     const len = vec3.length(normal);
//     //     expect(len).toBeCloseTo(1, 5);
//     // });
// });