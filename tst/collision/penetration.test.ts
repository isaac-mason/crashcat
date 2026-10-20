import { mat4, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { box, sphere } from '../../src';
import { createGjkCastShapeResult, gjkCastShape } from '../../src/collision/gjk';
import { penetrationCastShape } from '../../src/collision/penetration';
import { createSupport, SupportFunctionMode } from '../../src/collision/support';
import { setShapeSupport } from '../../src/shapes/shapes';

const COLLISION_TOLERANCE = 1e-4;
const PENETRATION_TOLERANCE = 1e-4;

// unit sphere (a) cast against a 2x2x2 box (b) at b's origin, supports built the way castConvexVsConvexLocal builds them
function setup(sphereX: number) {
    const sphereShape = sphere.create({ radius: 1 });
    const boxShape = box.create({ halfExtents: [1, 1, 1] });

    const supportA = createSupport();
    const supportB = createSupport();
    const unitScale = vec3.fromValues(1, 1, 1);
    setShapeSupport(supportA, sphereShape, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, unitScale);
    setShapeSupport(supportB, boxShape, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS, unitScale);

    const transformAtoB = mat4.fromTranslation(mat4.create(), vec3.fromValues(sphereX, 0, 0));

    return { supportA, supportB, transformAtoB };
}

describe('gjkCastShape', () => {
    test('keeps its final simplex after a hit', () => {
        const { supportA, supportB, transformAtoB } = setup(3);
        const displacement = vec3.fromValues(-4, 0, 0);
        const out = createGjkCastShapeResult();

        gjkCastShape(
            out,
            transformAtoB,
            supportA,
            supportB,
            displacement,
            COLLISION_TOLERANCE,
            supportA.convexRadius,
            supportB.convexRadius,
            1,
        );

        expect(out.hit).toBe(true);
        // sphere surface at x = 2 reaches the box's +x face at x = 1 after moving 1 of 4
        expect(out.lambda).toBeCloseTo(0.25, 3);

        const simplex = out.simplex;
        expect(simplex.size).toBeGreaterThanOrEqual(1);
        expect(simplex.size).toBeLessThanOrEqual(4);

        // the cast leaves y = x - (q - p) with x = lambda * displacement, which is y = p - q only at lambda 0
        const x = vec3.scale(vec3.create(), displacement, out.lambda);
        for (let i = 0; i < simplex.size; i++) {
            for (let axis = 0; axis < 3; axis++) {
                const offset = i * 3 + axis;
                expect(simplex.y[offset]).toBeCloseTo(x[axis] + simplex.p[offset] - simplex.q[offset], 6);
            }
        }
    });
});

describe('penetrationCastShape', () => {
    // sphere centre at x = 0.8, 0.2 inside the box's +x face: the sphere spans x in [-0.2, 1.8], and the shortest
    // way out is pushing it +x until its -x extreme (-0.2) clears the face at x = 1, a depth of 1.2
    const SPHERE_X = 0.8;

    test('starting intersecting with returnDeepestPoint runs epa from the gjk simplex', () => {
        const { supportA, supportB, transformAtoB } = setup(SPHERE_X);
        const displacement = vec3.fromValues(-1, 0, 0);
        const out = createGjkCastShapeResult();

        penetrationCastShape(
            out,
            transformAtoB,
            supportA,
            supportB,
            displacement,
            COLLISION_TOLERANCE,
            PENETRATION_TOLERANCE,
            supportA.convexRadius,
            supportB.convexRadius,
            1,
            true,
        );

        expect(out.hit).toBe(true);
        expect(out.lambda).toBe(0);

        const depth = vec3.distance(out.pointA, out.pointB);
        expect(Math.abs(depth - 1.2)).toBeLessThan(1e-3);

        // the axis points from a into b, like the cast's separating axis, so the contact normal -axis is +x
        const axis = vec3.normalize(vec3.create(), out.separatingAxis);
        expect(axis[0]).toBeCloseTo(-1, 3);
        expect(axis[1]).toBeCloseTo(0, 3);
        expect(axis[2]).toBeCloseTo(0, 3);

        // deepest point of the sphere inside the box: its -x extreme
        expect(out.pointA[0]).toBeCloseTo(SPHERE_X - 1, 3);
        expect(out.pointA[1]).toBeCloseTo(0, 3);
        expect(out.pointA[2]).toBeCloseTo(0, 3);

        // deepest point of the box inside the sphere: on its +x face
        expect(out.pointB[0]).toBeCloseTo(1, 3);
        expect(Math.abs(out.pointB[1])).toBeLessThanOrEqual(1);
        expect(Math.abs(out.pointB[2])).toBeLessThanOrEqual(1);
    });

    test('starting intersecting without returnDeepestPoint still reports lambda 0', () => {
        const { supportA, supportB, transformAtoB } = setup(SPHERE_X);
        const displacement = vec3.fromValues(-1, 0, 0);
        const out = createGjkCastShapeResult();

        penetrationCastShape(
            out,
            transformAtoB,
            supportA,
            supportB,
            displacement,
            COLLISION_TOLERANCE,
            PENETRATION_TOLERANCE,
            supportA.convexRadius,
            supportB.convexRadius,
            1,
            false,
        );

        expect(out.hit).toBe(true);
        expect(out.lambda).toBe(0);
    });
});
