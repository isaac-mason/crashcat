import { describe, expect, test } from 'vitest';
import { rayDistanceToBox3, rayFractionToBox3, safeReciprocal } from '../../src/collision/cast-utils';

function fraction(
    origin: [number, number, number],
    direction: [number, number, number],
    length: number,
    box: [number, number, number, number, number, number],
) {
    return rayFractionToBox3(
        origin[0],
        origin[1],
        origin[2],
        safeReciprocal(direction[0] * length),
        safeReciprocal(direction[1] * length),
        safeReciprocal(direction[2] * length),
        box[0],
        box[1],
        box[2],
        box[3],
        box[4],
        box[5],
    );
}

const unit: [number, number, number, number, number, number] = [-1, -1, -1, 1, 1, 1];

describe('rayFractionToBox3', () => {
    test('entry fraction along the segment', () => {
        expect(fraction([-5, 0, 0], [1, 0, 0], 10, unit)).toBeCloseTo(0.4, 12);
        expect(fraction([0, 5, 0], [0, -1, 0], 20, unit)).toBeCloseTo(0.2, 12);
    });

    test('origin inside the box gives zero', () => {
        expect(fraction([0.2, -0.3, 0.1], [0, 0, 1], 5, unit)).toBe(0);
    });

    test('segment ending before the box misses', () => {
        expect(fraction([-5, 0, 0], [1, 0, 0], 3, unit)).toBe(Infinity);
    });

    test('box behind the origin misses', () => {
        expect(fraction([5, 0, 0], [1, 0, 0], 10, unit)).toBe(Infinity);
    });

    test('axis-parallel rays: inside the slab hit, outside the slab miss', () => {
        // moving along x with y and z components exactly zero
        expect(fraction([-5, 0.5, -0.5], [1, 0, 0], 10, unit)).toBeCloseTo(0.4, 12);
        expect(fraction([-5, 1.5, 0], [1, 0, 0], 10, unit)).toBe(Infinity);
        expect(fraction([-5, 0, -1.5], [1, 0, 0], 10, unit)).toBe(Infinity);
        // negative zero and tiny components behave the same
        expect(fraction([-5, 0.5, 0.5], [1, -0, 1e-40], 10, unit)).toBeCloseTo(0.4, 12);
        // origin exactly on a slab face counts as inside
        expect(fraction([-5, 1, 0], [1, 0, 0], 10, unit)).toBeCloseTo(0.4, 12);
    });

    test('agrees with rayDistanceToBox3 on random segments', () => {
        let seed = 12345;
        const rand = () => {
            seed = (seed * 1664525 + 1013904223) >>> 0;
            return seed / 4294967296;
        };
        for (let i = 0; i < 2000; i++) {
            const box: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
            for (let k = 0; k < 3; k++) {
                const a = rand() * 10 - 5;
                const b = a + rand() * 4;
                box[k] = a;
                box[k + 3] = b;
            }
            const origin: [number, number, number] = [rand() * 20 - 10, rand() * 20 - 10, rand() * 20 - 10];
            let dx = rand() * 2 - 1;
            let dy = rand() * 2 - 1;
            let dz = rand() * 2 - 1;
            if (i % 7 === 0) dx = 0;
            if (i % 11 === 0) dy = 0;
            const len = Math.hypot(dx, dy, dz) || 1;
            dx /= len;
            dy /= len;
            dz /= len;
            const length = rand() * 30;

            const expected = rayDistanceToBox3(
                origin[0],
                origin[1],
                origin[2],
                dx,
                dy,
                dz,
                length,
                box[0],
                box[1],
                box[2],
                box[3],
                box[4],
                box[5],
            );
            const actual = fraction(origin, [dx, dy, dz], length, box);

            if (expected === Infinity) {
                expect(actual).toBe(Infinity);
            } else {
                expect(actual).toBeCloseTo(expected, 9);
            }
        }
    });
});
