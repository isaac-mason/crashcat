import { quat, vec3 } from 'mathcat';
import { describe, expect, it } from 'vitest';
import {
    box,
    capsule,
    compound,
    convexHull,
    getShapeInnerRadius,
    scaled,
    sphere,
    transformed,
    triangleMesh,
} from '../../src';

describe('getShapeInnerRadius', () => {
    it('returns radius for sphere', () => {
        const shape = sphere.create({ radius: 5 });
        const innerRadius = getShapeInnerRadius(shape);
        expect(innerRadius).toBe(5);
    });

    it('returns minimum half extent for box', () => {
        const shape = box.create({
            halfExtents: vec3.fromValues(2, 3, 4),
        });
        const innerRadius = getShapeInnerRadius(shape);
        expect(innerRadius).toBe(2); // min of 2, 3, 4
    });

    it('returns radius for capsule', () => {
        const shape = capsule.create({
            halfHeightOfCylinder: 10,
            radius: 3,
        });
        const innerRadius = getShapeInnerRadius(shape);
        expect(innerRadius).toBe(3);
    });

    it('calculates inner radius for convex hull', () => {
        // Create a simple box-like convex hull
        const shape = convexHull.create({
            positions: [
                -1, -1, -1,
                1, -1, -1,
                -1, 1, -1,
                1, 1, -1,
                -1, -1, 1,
                1, -1, 1,
                -1, 1, 1,
                1, 1, 1,
            ],
        });
        const innerRadius = getShapeInnerRadius(shape);
        // For a 2x2x2 box centered at origin, inner radius should be 1
        expect(innerRadius).toBeCloseTo(1, 2);
    });

    it('returns 0 for triangle mesh', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });
        const innerRadius = getShapeInnerRadius(mesh);
        expect(innerRadius).toBe(0);
    });

    it('returns minimum child inner radius for compound', () => {
        const shape1 = sphere.create({ radius: 5 });
        const shape2 = sphere.create({ radius: 3 });
        const compoundShape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: shape1 },
                { position: vec3.create(), quaternion: quat.create(), shape: shape2 },
            ],
        });
        const innerRadius = getShapeInnerRadius(compoundShape);
        expect(innerRadius).toBe(3); // min of 5 and 3
    });

    it('delegates to inner shape for transformed', () => {
        const shape = sphere.create({ radius: 7 });
        const transformedShape = transformed.create({
            shape: shape,
            position: vec3.fromValues(10, 20, 30),
            quaternion: [0, 0, 0, 1],
        });
        const innerRadius = getShapeInnerRadius(transformedShape);
        expect(innerRadius).toBe(7); // same as inner sphere
    });

    it('scales inner radius for scaled shape', () => {
        const shape = sphere.create({ radius: 10 });
        const scaledShape = scaled.create({
            shape: shape,
            scale: vec3.fromValues(2, 3, 4),
        });
        const innerRadius = getShapeInnerRadius(scaledShape);
        expect(innerRadius).toBe(20); // min scale (2) * sphere radius (10)
    });

    it('handles negative scale for scaled shape', () => {
        const shape = sphere.create({ radius: 10 });
        const scaledShape = scaled.create({
            shape: shape,
            scale: vec3.fromValues(-2, 3, 4),
        });
        const innerRadius = getShapeInnerRadius(scaledShape);
        expect(innerRadius).toBe(20); // abs(min scale) * radius = abs(-2) * 10
    });

    it('handles nested compound shapes', () => {
        const sphere1 = sphere.create({ radius: 10 });
        const sphere2 = sphere.create({ radius: 5 });
        const innerCompound = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere1 },
                { position: vec3.create(), quaternion: quat.create(), shape: sphere2 },
            ],
        });

        const sphere3 = sphere.create({ radius: 3 });
        const outerCompound = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: innerCompound },
                { position: vec3.create(), quaternion: quat.create(), shape: sphere3 },
            ],
        });

        const innerRadius = getShapeInnerRadius(outerCompound);
        expect(innerRadius).toBe(3); // min of all nested spheres
    });
});
