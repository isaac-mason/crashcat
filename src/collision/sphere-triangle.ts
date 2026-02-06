import type { Vec3 } from 'mathcat';

/**
 * Test ray against infinite cylinder, returning smallest positive fraction in [0, 1].
 * Based on "Real Time Collision Detection" by Christer Ericson, Chapter 5.3.7.
 *
 * Note: The ray origin is assumed to be at the origin (0, 0, 0).
 * The cylinder is defined by two endpoints (cylinderA, cylinderB) and a radius.
 * This test only considers hits on the cylindrical surface, not the caps.
 *
 * @param direction ray direction (length defines max distance, ray starts at origin)
 * @param cylinderA first endpoint of cylinder axis
 * @param cylinderB second endpoint of cylinder axis
 * @param radius cylinder radius
 * @returns fraction [0, 1] if hit on cylinder surface, or Infinity if no hit
 */
export function rayCylinder(direction: Vec3, cylinderA: Vec3, cylinderB: Vec3, radius: number): number {
    // calculate cylinder axis
    const axisX = cylinderB[0] - cylinderA[0];
    const axisY = cylinderB[1] - cylinderA[1];
    const axisZ = cylinderB[2] - cylinderA[2];

    // make ray start relative to cylinder side A (moving cylinder A to the origin)
    // since ray origin is at (0,0,0), start = -cylinderA
    const startX = -cylinderA[0];
    const startY = -cylinderA[1];
    const startZ = -cylinderA[2];

    // test if segment is fully on the A side of the cylinder
    const startDotAxis = startX * axisX + startY * axisY + startZ * axisZ;
    const directionDotAxis = direction[0] * axisX + direction[1] * axisY + direction[2] * axisZ;
    const endDotAxis = startDotAxis + directionDotAxis;

    if (startDotAxis < 0 && endDotAxis < 0) {
        return Infinity;
    }

    // test if segment is fully on the B side of the cylinder
    const axisLenSq = axisX * axisX + axisY * axisY + axisZ * axisZ;
    if (startDotAxis > axisLenSq && endDotAxis > axisLenSq) {
        return Infinity;
    }

    // calculate a, b and c, the factors for quadratic equation
    // we're solving: |ray(t) - closest_point_on_axis|^2 = radius^2
    const dirLenSq = direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
    const a = axisLenSq * dirLenSq - directionDotAxis * directionDotAxis;

    if (Math.abs(a) < 1e-6) {
        // segment runs parallel to cylinder axis
        return Infinity;
    }

    const startDotDir = startX * direction[0] + startY * direction[1] + startZ * direction[2];
    const startLenSq = startX * startX + startY * startY + startZ * startZ;

    // note: b should be multiplied by 2, but we'll divide a and c by 2 when solving
    const b = axisLenSq * startDotDir - directionDotAxis * startDotAxis;
    const c = axisLenSq * (startLenSq - radius * radius) - startDotAxis * startDotAxis;

    // discriminant (normally 4*a*c but since both a and c are divided by 2, we lose the 4)
    const det = b * b - a * c;
    if (det < 0) {
        return Infinity; // no solution to quadratic equation
    }

    // solve fraction t where the ray hits the cylinder
    // normally divided by 2*a but since a should be divided by 2, we lose the 2
    const t = -(b + Math.sqrt(det)) / a;

    if (t < 0 || t > 1) {
        return Infinity; // intersection lies outside segment [0, 1]
    }

    // check if intersection is within cylinder segment (not past the caps)
    const hitDotAxis = startDotAxis + t * directionDotAxis;
    if (hitDotAxis < 0 || hitDotAxis > axisLenSq) {
        return Infinity; // intersection outside the cylinder segment
    }

    return t;
}

/**
 * Test ray against sphere, returning smallest positive fraction in [0, 1].
 * @param origin ray origin
 * @param direction ray direction (length defines max distance)
 * @param sphereCenter center of the sphere
 * @param sphereRadius radius of the sphere
 * @returns Fraction [0, 1] if hit, or Infinity if no hit. Returns 0 if ray starts inside sphere.
 */
export function raySphere(origin: Vec3, direction: Vec3, sphereCenter: Vec3, sphereRadius: number): number {
    // solve: |origin + t * direction - center|^2 = radius^2
    const centerOriginX = origin[0] - sphereCenter[0];
    const centerOriginY = origin[1] - sphereCenter[1];
    const centerOriginZ = origin[2] - sphereCenter[2];

    const a = direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
    const b = 2 * (direction[0] * centerOriginX + direction[1] * centerOriginY + direction[2] * centerOriginZ);
    const c =
        centerOriginX * centerOriginX +
        centerOriginY * centerOriginY +
        centerOriginZ * centerOriginZ -
        sphereRadius * sphereRadius;

    const discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        // no intersection - but check if we're inside the sphere
        return c <= 0 ? 0 : Infinity;
    }

    if (a < 1e-10) {
        // degenerate case: direction is zero-length
        return c <= 0 ? 0 : Infinity;
    }

    const sqrtD = Math.sqrt(discriminant);
    let t1 = (-b - sqrtD) / (2 * a);
    let t2 = (-b + sqrtD) / (2 * a);

    // sort so t1 <= t2
    if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
    }

    // test solution with lowest fraction (ray entering sphere)
    if (t1 >= 0 && t1 <= 1) {
        return t1;
    }

    // test solution with highest fraction (ray leaving sphere)
    if (t2 >= 0 && t2 <= 1) {
        // we start inside the sphere
        return 0;
    }

    // check if entire ray segment is inside sphere
    if (t1 < 0 && t2 > 1) {
        return 0;
    }

    return Infinity;
}

/**
 * Test ray (starting at origin) against sphere, returning smallest positive fraction in [0, 1].
 * Optimized version where ray origin is assumed to be at (0, 0, 0).
 * @param direction ray direction (length defines max distance)
 * @param sphereCenter center of the sphere
 * @param sphereRadius radius of the sphere
 * @returns Fraction [0, 1] if hit, or Infinity if no hit. Returns 0 if ray starts inside sphere.
 */
export function raySphereFromOrigin(direction: Vec3, sphereCenter: Vec3, sphereRadius: number): number {
    // solve: |t * direction - center|^2 = radius^2
    const a = direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
    const b = -2 * (direction[0] * sphereCenter[0] + direction[1] * sphereCenter[1] + direction[2] * sphereCenter[2]);
    const c =
        sphereCenter[0] * sphereCenter[0] +
        sphereCenter[1] * sphereCenter[1] +
        sphereCenter[2] * sphereCenter[2] -
        sphereRadius * sphereRadius;

    const discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        // no intersection - but check if we're inside the sphere
        return c <= 0 ? 0 : Infinity;
    }

    if (a < 1e-10) {
        // degenerate case: direction is zero-length
        return c <= 0 ? 0 : Infinity;
    }

    const sqrtD = Math.sqrt(discriminant);
    let t1 = (-b - sqrtD) / (2 * a);
    let t2 = (-b + sqrtD) / (2 * a);

    // sort so t1 <= t2
    if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
    }

    // test solution with lowest fraction (ray entering sphere)
    if (t1 >= 0 && t1 <= 1) {
        return t1;
    }

    // test solution with highest fraction (ray leaving sphere)
    if (t2 >= 0 && t2 <= 1) {
        // we start inside the sphere
        return 0;
    }

    // check if entire ray segment is inside sphere
    if (t1 < 0 && t2 > 1) {
        return 0;
    }

    return Infinity;
}

/**
 * Maps triangle feature (from getClosestPointOnTriangle) to required active edges bitmask.
 *
 * Feature bits: 0b001 = vertex A, 0b010 = vertex B, 0b100 = vertex C
 * Edge bits: 0b001 = edge AB, 0b010 = edge BC, 0b100 = edge CA
 *
 * When hitting a vertex, either of the adjacent edges must be active.
 * When hitting an edge, that specific edge must be active.
 * When hitting the face interior (0b111), no edge check is needed.
 */
export const FEATURE_TO_ACTIVE_EDGES: number[] = [
    0b000, // 0: invalid
    0b101, // 1 (0b001): vertex A -> needs edge AB (0) or edge CA (2) active
    0b011, // 2 (0b010): vertex B -> needs edge AB (0) or edge BC (1) active
    0b001, // 3 (0b011): edge AB -> needs edge AB (0) active
    0b110, // 4 (0b100): vertex C -> needs edge BC (1) or edge CA (2) active
    0b100, // 5 (0b101): edge CA -> needs edge CA (2) active
    0b010, // 6 (0b110): edge BC -> needs edge BC (1) active
    0b000, // 7 (0b111): face interior -> no edge check needed
];
