import { type Mat4, mat4 } from 'math';
import { describe, expect, test } from 'vitest';
import { getInverseInertiaForRotation } from '../src/body/motion-properties';

// `getInverseInertiaForRotation` is written out in scalars rather than as four `mat4` calls through
// module-scope scratch. That is a performance choice (the scratch is why the helper version was
// slower — a buffer the whole module can see cannot live in registers), and it is only a safe one
// while the arithmetic stays EXACTLY equivalent: crashcat asserts bit-identical determinism across
// two worlds, so a rewrite that is merely accurate to 1e-15 is a different engine.
//
// The helper chain no longer exists in src, so this test carries it as the reference. If someone
// edits the scalar math, this is what notices.

/** The original formulation: I_inv_world = (R * diag(d)) * R^T, via four mat4 calls. */
const _inertiaRotMat = mat4.create();
const _rotation = mat4.create();
const _scaled = mat4.create();

function reference(out: Mat4, inertiaRotation: number[], invInertiaDiagonal: number[], bodyRotation: Mat4): Mat4 {
    mat4.fromQuat(_inertiaRotMat, inertiaRotation as never);
    mat4.multiply3x3(_rotation, bodyRotation, _inertiaRotMat);
    mat4.scale(_scaled, _rotation, invInertiaDiagonal as never);
    mat4.multiply3x3RightTransposed(out, _scaled, _rotation);
    return out;
}

/** Deterministic PRNG, so a failure is reproducible rather than "it went red once in CI". */
function makeRandom(seed: number): () => number {
    let s = seed >>> 0;
    return () => {
        s = (s * 1664525 + 1013904223) >>> 0;
        return s / 0x100000000;
    };
}

describe('getInverseInertiaForRotation scalar form', () => {
    test('is bit-identical to the mat4 helper chain, for every rotation dof mask', () => {
        const rand = makeRandom(0x5bf03635);
        const spread = () => rand() * 4 - 2;
        const norm4 = (v: number[]) => {
            const l = Math.hypot(v[0], v[1], v[2], v[3]);
            return [v[0] / l, v[1] / l, v[2] / l, v[3] / l];
        };

        const masksSeen = new Set<number>();
        for (let i = 0; i < 4000; i++) {
            // The masked branch is where a rewrite is most likely to diverge, and it is the branch a
            // locked-axis body takes every step — so cycle all eight, not just the common one.
            const rotationDofs = i % 8;
            masksSeen.add(rotationDofs);

            const inertiaRotation = norm4([spread(), spread(), spread(), spread()]);
            const invInertiaDiagonal = [1 / (0.05 + rand() * 20), 1 / (0.05 + rand() * 20), 1 / (0.05 + rand() * 20)];
            const bodyRotation = mat4.create();
            mat4.fromQuat(bodyRotation, norm4([spread(), spread(), spread(), spread()]) as never);

            const motionProperties = {
                inertiaRotation,
                invInertiaDiagonal,
                allowedDegreesOfFreedom: rotationDofs << 3,
            } as never;

            const actual = getInverseInertiaForRotation(mat4.create(), motionProperties, bodyRotation);

            const expected = reference(mat4.create(), inertiaRotation, invInertiaDiagonal, bodyRotation);
            if (rotationDofs !== 0b111) {
                const mx = rotationDofs & 0b001 ? 1 : 0;
                const my = rotationDofs & 0b010 ? 1 : 0;
                const mz = rotationDofs & 0b100 ? 1 : 0;
                expected[0] *= mx * mx;
                expected[1] *= my * mx;
                expected[2] *= mz * mx;
                expected[4] *= mx * my;
                expected[5] *= my * my;
                expected[6] *= mz * my;
                expected[8] *= mx * mz;
                expected[9] *= my * mz;
                expected[10] *= mz * mz;
            }

            // All sixteen cells, exactly — not `toBeCloseTo`. The helper chain writes the whole
            // matrix including the zero column and the bottom row, so the rewrite must too.
            for (let c = 0; c < 16; c++) {
                expect(actual[c], `cell ${c}, dofs ${rotationDofs}, iteration ${i}`).toBe(expected[c]);
            }
        }
        expect(masksSeen.size).toBe(8);
    });
});
