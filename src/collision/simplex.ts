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
        vec3.copy(out.points[i].y, input.points[i].y);
        vec3.copy(out.points[i].p, input.points[i].p);
        vec3.copy(out.points[i].q, input.points[i].q);
    }
    return out;
};
