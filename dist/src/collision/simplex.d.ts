/**
 * Simplex used in GJK/EPA algorithms.
 *
 * Storage is three flat `number[12]` arrays — `y`, `p`, `q` — with point `i`
 * living at offsets `i*3 .. i*3+2` in each. This keeps the gjk inner loop's
 * y-read sweep contiguous (one cache line) and avoided the nested-object
 * pointer chase that a `points[i].y[k]` layout pays on every access.
 *
 * Invariant: `y[i]`, `p[i]`, `q[i]` at the same base offset describe the same
 * logical support pair — `y = p - q`.
 */
export type Simplex = {
    /** minkowski difference (P - Q) per point, flat xyz, length 12 */
    y: [
        x1: number,
        y1: number,
        z1: number,
        x2: number,
        y2: number,
        z2: number,
        x3: number,
        y3: number,
        z3: number,
        x4: number,
        y4: number,
        z4: number
    ];
    /** support point on shape A per point, flat xyz, length 12 */
    p: [
        x1: number,
        y1: number,
        z1: number,
        x2: number,
        y2: number,
        z2: number,
        x3: number,
        y3: number,
        z3: number,
        x4: number,
        y4: number,
        z4: number
    ];
    /** support point on shape B per point, flat xyz, length 12 */
    q: [
        x1: number,
        y1: number,
        z1: number,
        x2: number,
        y2: number,
        z2: number,
        x3: number,
        y3: number,
        z3: number,
        x4: number,
        y4: number,
        z4: number
    ];
    /** current number of points in the simplex (0..4) */
    size: number;
};
/** creates a new simplex */
export declare const createSimplex: () => Simplex;
/** copies a simplex */
export declare const copySimplex: (out: Simplex, input: Simplex) => Simplex;
