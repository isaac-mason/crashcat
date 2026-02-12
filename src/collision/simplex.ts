import type { Vec3 } from 'mathcat';
import { vec3 } from 'mathcat';

/** point in a simplex */
export type SimplexPoint = {
    /** minkowski difference (P - Q) */
    y: Vec3;
    /** support point on shape A */
    p: Vec3;
    /** support point on shape B */
    q: Vec3;
};

/** simplex used in GJK/EPA algorithms */
export type Simplex = {
    /** points in the simplex */
    points: [SimplexPoint, SimplexPoint, SimplexPoint, SimplexPoint];
    /** current number of points in the simplex */
    size: number;
};

/** creates a new simplex */
export const createSimplex = (): Simplex => ({
    points: [
        { y: vec3.create(), p: vec3.create(), q: vec3.create() },
        { y: vec3.create(), p: vec3.create(), q: vec3.create() },
        { y: vec3.create(), p: vec3.create(), q: vec3.create() },
        { y: vec3.create(), p: vec3.create(), q: vec3.create() },
    ],
    size: 0,
});

/** copies a simplex */
export const copySimplex = (out: Simplex, input: Simplex): Simplex => {
    out.size = input.size;
    for (let i = 0; i < input.size; i++) {
        const src = input.points[i];
        const dst = out.points[i];
        dst.y[0] = src.y[0];
        dst.y[1] = src.y[1];
        dst.y[2] = src.y[2];
        dst.p[0] = src.p[0];
        dst.p[1] = src.p[1];
        dst.p[2] = src.p[2];
        dst.q[0] = src.q[0];
        dst.q[1] = src.q[1];
        dst.q[2] = src.q[2];
    }
    return out;
};
