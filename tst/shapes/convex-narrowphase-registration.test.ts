import { describe, expect, test } from 'vitest';
import { collisionDispatch, ShapeType } from '../../src';
import { collideSphereVsBox } from '../../src/shapes/box';
import { castConvexVsConvex, collideConvexVsConvex } from '../../src/shapes/convex';
import { shapeDefs } from '../../src/shapes/shapes';
import { collideSphereVsSphere } from '../../src/shapes/sphere';

// a convex shape declares itself convex by carrying a `convex` descriptor, and each one wires its
// own pairs against every other convex shape, then overwrites individual pairs with specialised
// handlers. nothing else in the suite checks WHICH
// handler a pair resolves to — a pair silently falling back to the generic gjk/epa path still
// produces correct contacts, just slower, so every other test would keep passing. these assertions
// are the only thing standing between a registration-order mistake and a silent perf regression.

/** convexity is carried by the def, and nothing else: no category, no separate registry */
const isConvex = (t: ShapeType) => shapeDefs[t]?.convex !== undefined;

/** the convex types this test run actually has registered */
const registeredConvexTypes = (): ShapeType[] => (Object.keys(shapeDefs) as unknown as ShapeType[]).map(Number).filter(isConvex);

const collideFor = (a: ShapeType, b: ShapeType) => collisionDispatch.collideFns.get(a)?.get(b);
const castFor = (a: ShapeType, b: ShapeType) => collisionDispatch.castFns.get(a)?.get(b);

describe('convex narrowphase registration', () => {
    test('every built-in convex type is registered with the narrowphase', () => {
        const expected = [ShapeType.SPHERE, ShapeType.BOX, ShapeType.CAPSULE, ShapeType.CYLINDER, ShapeType.CONVEX_HULL];
        expect([...registeredConvexTypes()].sort((a, b) => a - b)).toEqual(expected.sort((a, b) => a - b));
        for (const t of expected) expect(isConvex(t)).toBe(true);

        // and nothing else claims to be convex
        expect(isConvex(ShapeType.TRIANGLE_MESH)).toBe(false);
        expect(isConvex(ShapeType.PLANE)).toBe(false);
        expect(isConvex(ShapeType.COMPOUND)).toBe(false);
    });

    test('the full convex cross product is wired in both directions', () => {
        const types = registeredConvexTypes();
        for (const a of types) {
            for (const b of types) {
                expect(collideFor(a, b), `collide ${ShapeType[a]} x ${ShapeType[b]}`).toBeDefined();
                expect(castFor(a, b), `cast ${ShapeType[a]} x ${ShapeType[b]}`).toBeDefined();
            }
        }
    });

    test('specialised handlers win over the generic one', () => {
        // these are the pairs that have a closed-form handler. if registration order regresses,
        // the generic handler overwrites them and they silently get slower, not wrong.
        expect(collideFor(ShapeType.SPHERE, ShapeType.SPHERE)).toBe(collideSphereVsSphere);
        expect(collideFor(ShapeType.SPHERE, ShapeType.BOX)).toBe(collideSphereVsBox);

        // box vs sphere is the reversed wrapper, so identity does not hold — but it must not be
        // the generic handler either
        expect(collideFor(ShapeType.BOX, ShapeType.SPHERE)).not.toBe(collideConvexVsConvex);
    });

    test('pairs without a specialisation resolve to the generic handler', () => {
        expect(collideFor(ShapeType.CAPSULE, ShapeType.CYLINDER)).toBe(collideConvexVsConvex);
        expect(collideFor(ShapeType.CONVEX_HULL, ShapeType.CONVEX_HULL)).toBe(collideConvexVsConvex);
        expect(collideFor(ShapeType.BOX, ShapeType.CONVEX_HULL)).toBe(collideConvexVsConvex);
        // box vs box has no specialised handler: the sat one is parked in llm/parked
        expect(collideFor(ShapeType.BOX, ShapeType.BOX)).toBe(collideConvexVsConvex);
        expect(castFor(ShapeType.BOX, ShapeType.BOX)).toBe(castConvexVsConvex);
    });

    test('convex shapes are wired against plane and triangle mesh', () => {
        for (const t of registeredConvexTypes()) {
            expect(collideFor(t, ShapeType.PLANE), `${ShapeType[t]} vs plane`).toBeDefined();
            expect(collideFor(t, ShapeType.TRIANGLE_MESH), `${ShapeType[t]} vs mesh`).toBeDefined();
            expect(collideFor(ShapeType.TRIANGLE_MESH, t), `mesh vs ${ShapeType[t]}`).toBeDefined();
        }
    });
});
