import { quat, vec3 } from 'math';
import { describe, expect, it } from 'vitest';
import { box, capsule, compound, convexHull, getShapeInnerRadius, scaled, sphere, transformed, triangleMesh } from '../../src';

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
            positions: [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1],
        });
        const innerRadius = getShapeInnerRadius(shape);
        // For a 2x2x2 box centered at origin, inner radius should be 1
        expect(innerRadius).toBeCloseTo(1, 2);
    });

    it('measures a convex hull from its center of mass, wherever the hull sits', () => {
        const cube = [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1];
        const shifted = cube.map((value, i) => (i % 3 === 0 ? value + 10 : value));
        const shape = convexHull.create({ positions: shifted });
        expect(shape.centerOfMass[0]).toBeCloseTo(10, 10);
        expect(getShapeInnerRadius(shape)).toBeCloseTo(1, 10);
    });

    it('is the shortest center of mass to plane distance of a skewed convex hull', () => {
        const vertices = [
            vec3.fromValues(3, 1, 2),
            vec3.fromValues(7, 1, 2),
            vec3.fromValues(4, 6, 2),
            vec3.fromValues(5, 3, 8),
        ];
        const shape = convexHull.create({ positions: vertices.flatMap((v) => [v[0], v[1], v[2]]) });

        // a tetrahedron's center of mass is the mean of its vertices
        const centerOfMass = vec3.create();
        for (const v of vertices) vec3.scaleAndAdd(centerOfMass, centerOfMass, v, 0.25);
        expect(vec3.distance(shape.centerOfMass, centerOfMass)).toBeLessThan(1e-10);

        let expected = Number.MAX_VALUE;
        for (let excluded = 0; excluded < 4; excluded++) {
            const [a, b, c] = vertices.filter((_, i) => i !== excluded);
            const normal = vec3.cross(vec3.create(), vec3.subtract(vec3.create(), b, a), vec3.subtract(vec3.create(), c, a));
            vec3.normalize(normal, normal);
            expected = Math.min(expected, Math.abs(vec3.dot(normal, vec3.subtract(vec3.create(), centerOfMass, a))));
        }
        expect(expected).toBeGreaterThan(0.5);

        expect(getShapeInnerRadius(shape)).toBeCloseTo(expected, 10);
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
