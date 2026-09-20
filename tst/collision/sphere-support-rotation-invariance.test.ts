import { mat4, quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import { sphere } from '../../src';
import { createSupport, getSupport, SupportFunctionMode, setSphereSupport } from '../../src/collision/support';

// getSupport short-circuits spheres before the shared direction transform, on the grounds that a
// sphere's support is rotation invariant: the transform into local space and the rotation half of
// the transform back cancel exactly, and addRadius folds into the radius because it is applied
// along that same direction.
//
// this checks the claim against the long way round — transform the direction in, build r·dir̂ in
// local space, rotate and translate back — which is what the code did before. the two differ only
// in floating point ordering, so the gate is a tight tolerance rather than bit equality.

const UNIT: [number, number, number] = [1, 1, 1];

/** the pre-short-circuit computation, written out longhand */
function referenceSphereSupport(
    out: number[],
    transform: number[] | null,
    coreRadius: number,
    addRadius: number,
    dir: readonly number[],
): void {
    let dx = dir[0];
    let dy = dir[1];
    let dz = dir[2];
    if (transform) {
        const m = transform;
        const lx = m[0] * dx + m[1] * dy + m[2] * dz;
        const ly = m[4] * dx + m[5] * dy + m[6] * dz;
        const lz = m[8] * dx + m[9] * dy + m[10] * dz;
        dx = lx;
        dy = ly;
        dz = lz;
    }

    let sx = 0;
    let sy = 0;
    let sz = 0;
    if (coreRadius > 0) {
        const lengthSq = dx * dx + dy * dy + dz * dz;
        if (lengthSq > 0) {
            const scale = coreRadius / Math.sqrt(lengthSq);
            sx = dx * scale;
            sy = dy * scale;
            sz = dz * scale;
        }
    }
    if (addRadius > 0) {
        const lengthSq = dx * dx + dy * dy + dz * dz;
        if (lengthSq > 0) {
            const scale = addRadius / Math.sqrt(lengthSq);
            sx += dx * scale;
            sy += dy * scale;
            sz += dz * scale;
        }
    }

    if (transform) {
        const m = transform;
        out[0] = m[0] * sx + m[4] * sy + m[8] * sz + m[12];
        out[1] = m[1] * sx + m[5] * sy + m[9] * sz + m[13];
        out[2] = m[2] * sx + m[6] * sy + m[10] * sz + m[14];
    } else {
        out[0] = sx;
        out[1] = sy;
        out[2] = sz;
    }
}

function directions(): [number, number, number][] {
    const dirs: [number, number, number][] = [
        [0, 0, 0],
        [1, 0, 0],
        [-1, 0, 0],
        [0, 1, 0],
        [0, -1, 0],
        [0, 0, 1],
        [0, 0, -1],
    ];
    for (let i = 0; i < 60; i++) {
        const a = i * 0.7;
        const b = i * 1.31;
        dirs.push([Math.cos(a) * Math.sin(b), Math.sin(a), Math.cos(b)]);
    }
    // a deliberately unnormalised direction — gjk passes v, not v̂
    dirs.push([12.5, -3.25, 7.75]);
    return dirs;
}

describe('sphere support is rotation invariant', () => {
    test('matches the long way round under rotation, translation and addRadius', () => {
        const shape = sphere.create({ radius: 0.45 });
        const actual = vec3.create();
        const expected: number[] = [0, 0, 0];

        for (const mode of [SupportFunctionMode.INCLUDE_CONVEX_RADIUS, SupportFunctionMode.EXCLUDE_CONVEX_RADIUS]) {
            for (let i = 0; i < 6; i++) {
                const transform = i === 0 ? null : mat4.create();
                if (transform) {
                    const q = quat.create();
                    quat.fromDegrees(q, i * 41 + 13, i * 67 + 29, i * 23 + 5, 'xyz');
                    mat4.fromRotationTranslation(transform, q, [0.4 * i - 0.6, 0.35, -0.2 * i]);
                }

                for (const addRadius of [0, 0.125]) {
                    const support = createSupport();
                    setSphereSupport(support, shape, mode, UNIT);
                    support.addRadius = addRadius;
                    if (transform) {
                        support.hasTransform = true;
                        for (let k = 0; k < 16; k++) support.transform[k] = transform[k];
                    }

                    const coreRadius = support.coreRadius;
                    for (const dir of directions()) {
                        getSupport(actual, support, dir);
                        referenceSphereSupport(expected, transform ? [...transform] : null, coreRadius, addRadius, dir);
                        expect(actual[0]).toBeCloseTo(expected[0], 12);
                        expect(actual[1]).toBeCloseTo(expected[1], 12);
                        expect(actual[2]).toBeCloseTo(expected[2], 12);
                    }
                }
            }
        }
    });

    test('a translated sphere reports the rotated surface point at the expected distance', () => {
        // independent of the reference: whatever the rotation, the support must sit exactly
        // `radius` from the sphere centre, along the query direction.
        const shape = sphere.create({ radius: 0.45 });
        const support = createSupport();
        setSphereSupport(support, shape, SupportFunctionMode.INCLUDE_CONVEX_RADIUS, UNIT);
        const q = quat.create();
        quat.fromDegrees(q, 31, 57, 12, 'xyz');
        const transform = mat4.create();
        const centre: [number, number, number] = [1.25, -0.5, 0.75];
        mat4.fromRotationTranslation(transform, q, centre);
        support.hasTransform = true;
        for (let k = 0; k < 16; k++) support.transform[k] = transform[k];

        const actual = vec3.create();
        for (const dir of directions()) {
            const len = Math.hypot(dir[0], dir[1], dir[2]);
            if (len === 0) continue;
            getSupport(actual, support, dir);
            const offX = actual[0] - centre[0];
            const offY = actual[1] - centre[1];
            const offZ = actual[2] - centre[2];
            // distance from centre is the radius
            expect(Math.hypot(offX, offY, offZ)).toBeCloseTo(0.45, 12);
            // and it points along the query direction
            expect(offX / 0.45).toBeCloseTo(dir[0] / len, 12);
            expect(offY / 0.45).toBeCloseTo(dir[1] / len, 12);
            expect(offZ / 0.45).toBeCloseTo(dir[2] / len, 12);
        }
    });
});
