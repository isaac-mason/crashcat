const EPA_MAX_TRIANGLES = 256;
const EPA_MAX_EDGE_LENGTH = 128;
const EPA_MIN_TRIANGLE_AREA = 1e-10;
const EPA_BARYCENTRIC_EPSILON = 1e-3;

/**
 * triangle pool, flat double storage with stride TRI_FLOAT_STRIDE (9), indexed by triangle index:
 * [normalX, normalY, normalZ, centroidX, centroidY, centroidZ, closestLengthSq, lambda0, lambda1].
 * a triangle "is" its index into these arrays; -1 means null.
 */
export const TRI_FLOAT_STRIDE = 9;

/** offset of the triangle normal (3 slots) within a float record */
export const TRI_NORMAL = 0;

/** offset of the triangle centroid (3 slots) within a float record */
export const TRI_CENTROID = 3;

/** offset of the signed squared distance from origin to triangle plane within a float record */
export const TRI_CLOSEST_LENGTH_SQ = 6;

/** offset of barycentric coordinate lambda0 within a float record */
export const TRI_LAMBDA0 = 7;

/** offset of barycentric coordinate lambda1 within a float record */
export const TRI_LAMBDA1 = 8;

/**
 * triangle pool, flat integer storage with stride TRI_INT_STRIDE (11), indexed by triangle index.
 * the three edges live at slot `edge * 3`, each edge holding
 * [neighbourTriangle (-1 for null), neighbourEdge, startIndex] (slots 0-8); flags (9); nextFree (10).
 */
export const TRI_INT_STRIDE = 11;

/** per-edge offset (relative to `edge * 3`) of the neighbouring triangle index (-1 for null) */
export const EDGE_NEIGHBOUR_TRIANGLE = 0;

/** per-edge offset (relative to `edge * 3`) of the neighbouring edge index */
export const EDGE_NEIGHBOUR_EDGE = 1;

/** per-edge offset (relative to `edge * 3`) of the edge start vertex index */
export const EDGE_START_INDEX = 2;

/** offset of the packed flags word within an int record */
export const TRI_FLAGS = 9;

/** offset of the free-list link (next free triangle index, -1 if none) within an int record */
export const TRI_NEXT_FREE = 10;

/** flags bit: triangle has been removed from the hull */
export const TRI_FLAG_REMOVED = 1;
/** flags bit: triangle is currently in the priority queue */
export const TRI_FLAG_IN_QUEUE = 2;
/** flags bit: barycentric coordinates are relative to vertex 0 (else vertex 1) */
export const TRI_FLAG_LAMBDA_RELATIVE_TO_0 = 4;
/** flags bit: closest point to origin lies within the triangle interior */
export const TRI_FLAG_CLOSEST_POINT_INTERIOR = 8;

/**
 * silhouette edge list output by findEdge, flat with stride EDGE_STRIDE (3):
 * [neighbourTriangle, neighbourEdge, startIndex].
 */
export const EDGE_STRIDE = 3;

/**
 * DFS stack for the silhouette flood-fill, flat with stride STACK_STRIDE (3):
 * [triangleIndex, edge, iter]. `edge` is the entry edge, needed to compute the visited edge index.
 */
const STACK_STRIDE = 3;

export function triangleIsFacing(state: EpaConvexHullBuilderState, tri: number, points: number[], offset: number): boolean {
    const fBase = tri * TRI_FLOAT_STRIDE;
    // vectorAB = position - centroid
    const abx = points[offset] - state.triFloat[fBase + TRI_CENTROID];
    const aby = points[offset + 1] - state.triFloat[fBase + TRI_CENTROID + 1];
    const abz = points[offset + 2] - state.triFloat[fBase + TRI_CENTROID + 2];
    // dot(normal, vectorAB) > 0
    return (
        state.triFloat[fBase + TRI_NORMAL] * abx +
            state.triFloat[fBase + TRI_NORMAL + 1] * aby +
            state.triFloat[fBase + TRI_NORMAL + 2] * abz >
        0.0
    );
}

/** flat stride-3 point store: `values` packs [x, y, z] per point, `size` counts stored points */
export type Points = {
    values: number[];
    size: number;
};

export function createPoints(capacity: number): Points {
    return { values: new Array(capacity * 3).fill(0), size: 0 };
}

function pushTriangleToQueue(state: EpaConvexHullBuilderState, tri: number): void {
    const queue = state.queue;
    const triFloat = state.triFloat;
    queue.push(tri);
    state.triInt[tri * TRI_INT_STRIDE + TRI_FLAGS] |= TRI_FLAG_IN_QUEUE;

    // binary heap push: bubble up the new element
    let current = queue.length - 1;
    while (current > 0) {
        const currentElement = queue[current];
        const parent = (current - 1) >> 1;
        const parentElement = queue[parent];

        // min heap: parent should have smaller closestLengthSq than child
        if (
            triFloat[parentElement * TRI_FLOAT_STRIDE + TRI_CLOSEST_LENGTH_SQ] >
            triFloat[currentElement * TRI_FLOAT_STRIDE + TRI_CLOSEST_LENGTH_SQ]
        ) {
            queue[parent] = currentElement;
            queue[current] = parentElement;
            current = parent;
        } else {
            break;
        }
    }
}

/** new triangle indices produced by addPoint */
export type NewTriangles = number[];

export type EpaConvexHullBuilderState = {
    /** triangle pool double storage, stride TRI_FLOAT_STRIDE */
    triFloat: number[];
    /** triangle pool integer storage, stride TRI_INT_STRIDE */
    triInt: number[];
    // bump allocator: high watermark for never-used slots
    triangleHighWatermark: number;
    // linked free-list head index, -1 if empty
    triangleFreeHead: number;

    /** priority queue of triangle indices (binary heap keyed by closestLengthSq) */
    queue: number[];
    /** flat stride-3 vertex positions (aliases the support-point y store) */
    positions: number[];
    /** flat DFS stack, stride STACK_STRIDE */
    stack: number[];
    /** flat silhouette edge list, stride EDGE_STRIDE */
    edges: number[];
    /** number of edges currently stored in `edges` */
    edgesSize: number;
};

export function init(): EpaConvexHullBuilderState {
    return {
        // pre-allocate the full triangle pool upfront
        triFloat: new Array(EPA_MAX_TRIANGLES * TRI_FLOAT_STRIDE).fill(0),
        triInt: new Array(EPA_MAX_TRIANGLES * TRI_INT_STRIDE).fill(0),
        triangleHighWatermark: 0,
        triangleFreeHead: -1,
        queue: [],
        positions: [],
        stack: new Array(EPA_MAX_EDGE_LENGTH * STACK_STRIDE).fill(0),
        edges: new Array(EPA_MAX_EDGE_LENGTH * EDGE_STRIDE).fill(0),
        edgesSize: 0,
    };
}

export function linkTriangle(state: EpaConvexHullBuilderState, t1: number, edge1: number, t2: number, edge2: number) {
    const triInt = state.triInt;
    const b1 = t1 * TRI_INT_STRIDE + edge1 * 3;
    triInt[b1 + EDGE_NEIGHBOUR_TRIANGLE] = t2;
    triInt[b1 + EDGE_NEIGHBOUR_EDGE] = edge2;
    const b2 = t2 * TRI_INT_STRIDE + edge2 * 3;
    triInt[b2 + EDGE_NEIGHBOUR_TRIANGLE] = t1;
    triInt[b2 + EDGE_NEIGHBOUR_EDGE] = edge1;
}

export function createTriangle(state: EpaConvexHullBuilderState, idx1: number, idx2: number, idx3: number): number {
    const triInt = state.triInt;
    const triFloat = state.triFloat;

    let tri: number;
    const freeHead = state.triangleFreeHead;
    if (freeHead !== -1) {
        // take from free list
        tri = freeHead;
        state.triangleFreeHead = triInt[tri * TRI_INT_STRIDE + TRI_NEXT_FREE];
    } else {
        // take from never-used watermark
        const index = state.triangleHighWatermark;
        if (index >= EPA_MAX_TRIANGLES) return -1;
        state.triangleHighWatermark = index + 1;
        tri = index;
    }

    const fBase = tri * TRI_FLOAT_STRIDE;
    const iBase = tri * TRI_INT_STRIDE;

    // reset defaults (flags word clears removed/inQueue/lambdaRelativeTo0/closestPointInterior)
    triFloat[fBase + TRI_CLOSEST_LENGTH_SQ] = Infinity;
    triFloat[fBase + TRI_LAMBDA0] = 0.0;
    triFloat[fBase + TRI_LAMBDA1] = 0.0;
    triInt[iBase + TRI_FLAGS] = 0;

    // fill in indexes
    triInt[iBase + 0 * 3 + EDGE_START_INDEX] = idx1;
    triInt[iBase + 1 * 3 + EDGE_START_INDEX] = idx2;
    triInt[iBase + 2 * 3 + EDGE_START_INDEX] = idx3;

    // clear links
    triInt[iBase + 0 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
    triInt[iBase + 1 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
    triInt[iBase + 2 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;

    // get vertex positions
    const positions = state.positions;
    const o0 = idx1 * 3;
    const o1 = idx2 * 3;
    const o2 = idx3 * 3;
    const y0x = positions[o0];
    const y0y = positions[o0 + 1];
    const y0z = positions[o0 + 2];
    const y1x = positions[o1];
    const y1y = positions[o1 + 1];
    const y1z = positions[o1 + 2];
    const y2x = positions[o2];
    const y2y = positions[o2 + 1];
    const y2z = positions[o2 + 2];

    // calculate centroid
    const cx = (y0x + y1x + y2x) / 3.0;
    const cy = (y0y + y1y + y2y) / 3.0;
    const cz = (y0z + y1z + y2z) / 3.0;
    triFloat[fBase + TRI_CENTROID] = cx;
    triFloat[fBase + TRI_CENTROID + 1] = cy;
    triFloat[fBase + TRI_CENTROID + 2] = cz;

    // calculate edges
    // y10 = y1 - y0
    const y10x = y1x - y0x;
    const y10y = y1y - y0y;
    const y10z = y1z - y0z;

    // y20 = y2 - y0
    const y20x = y2x - y0x;
    const y20y = y2y - y0y;
    const y20z = y2z - y0z;

    // y21 = y2 - y1
    const y21x = y2x - y1x;
    const y21y = y2y - y1y;
    const y21z = y2z - y1z;

    // compute triangle normal
    // the most accurate normal is calculated by using the two shortest edges
    // y20DotY20 = dot(y20, y20)
    const y20DotY20 = y20x * y20x + y20y * y20y + y20z * y20z;
    // y21DotY21 = dot(y21, y21)
    const y21DotY21 = y21x * y21x + y21y * y21y + y21z * y21z;

    // both branches compute: normal, distance to origin, and barycentric coordinates
    // they differ only in which edge pair is selected for numerical stability
    if (y20DotY20 < y21DotY21) {
        // we select the edges y10 and y20
        // normal = cross(y10, y20)
        const nx = y10y * y20z - y10z * y20y;
        const ny = y10z * y20x - y10x * y20z;
        const nz = y10x * y20y - y10y * y20x;
        triFloat[fBase + TRI_NORMAL] = nx;
        triFloat[fBase + TRI_NORMAL + 1] = ny;
        triFloat[fBase + TRI_NORMAL + 2] = nz;

        // check if triangle is degenerate (area too small)
        const normalLenSq = nx * nx + ny * ny + nz * nz;
        if (normalLenSq > EPA_MIN_TRIANGLE_AREA) {
            // compute signed squared distance from origin to triangle plane
            // cDotN = dot(centroid, normal)
            const cDotN = cx * nx + cy * ny + cz * nz;
            // Math.abs(cDotN) * cDotN = sign(cDotN) * cDotN^2, preserves sign while squaring
            triFloat[fBase + TRI_CLOSEST_LENGTH_SQ] = (Math.abs(cDotN) * cDotN) / normalLenSq;

            // calculate barycentric coordinates of closest point on triangle to origin
            // y10DotY10 = dot(y10, y10)
            const y10DotY10 = y10x * y10x + y10y * y10y + y10z * y10z;
            // y10DotY20 = dot(y10, y20)
            const y10DotY20 = y10x * y20x + y10y * y20y + y10z * y20z;
            const determinant = y10DotY10 * y20DotY20 - y10DotY20 * y10DotY20;

            // determinant > 0 means non-degenerate triangle (edges not parallel)
            if (determinant > 0.0) {
                // y0DotY10 = dot(y0, y10)
                const y0DotY10 = y0x * y10x + y0y * y10y + y0z * y10z;
                // y0DotY20 = dot(y0, y20)
                const y0DotY20 = y0x * y20x + y0y * y20y + y0z * y20z;
                const l0 = (y10DotY20 * y0DotY20 - y20DotY20 * y0DotY10) / determinant;
                const l1 = (y10DotY20 * y0DotY10 - y10DotY10 * y0DotY20) / determinant;
                triFloat[fBase + TRI_LAMBDA0] = l0;
                triFloat[fBase + TRI_LAMBDA1] = l1;
                triInt[iBase + TRI_FLAGS] |= TRI_FLAG_LAMBDA_RELATIVE_TO_0;

                // check if closest point lies within triangle interior
                // point is interior if: l0 >= 0, l1 >= 0, and l0 + l1 <= 1 (barycentric bounds)
                if (l0 > -EPA_BARYCENTRIC_EPSILON && l1 > -EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0 + EPA_BARYCENTRIC_EPSILON) {
                    triInt[iBase + TRI_FLAGS] |= TRI_FLAG_CLOSEST_POINT_INTERIOR;
                }
            }
        }
    } else {
        // we select the edges y10 and y21
        // normal = cross(y10, y21)
        const nx = y10y * y21z - y10z * y21y;
        const ny = y10z * y21x - y10x * y21z;
        const nz = y10x * y21y - y10y * y21x;
        triFloat[fBase + TRI_NORMAL] = nx;
        triFloat[fBase + TRI_NORMAL + 1] = ny;
        triFloat[fBase + TRI_NORMAL + 2] = nz;

        // check if triangle is degenerate (area too small)
        const normalLenSq = nx * nx + ny * ny + nz * nz;
        if (normalLenSq > EPA_MIN_TRIANGLE_AREA) {
            // compute signed squared distance from origin to triangle plane
            // cDotN = dot(centroid, normal)
            const cDotN = cx * nx + cy * ny + cz * nz;
            // Math.abs(cDotN) * cDotN = sign(cDotN) * cDotN^2, preserves sign while squaring
            triFloat[fBase + TRI_CLOSEST_LENGTH_SQ] = (Math.abs(cDotN) * cDotN) / normalLenSq;

            // calculate barycentric coordinates of closest point on triangle to origin (y1 as reference)
            // y10DotY10 = dot(y10, y10)
            const y10DotY10 = y10x * y10x + y10y * y10y + y10z * y10z;
            // y10DotY21 = dot(y10, y21)
            const y10DotY21 = y10x * y21x + y10y * y21y + y10z * y21z;
            const determinant = y10DotY10 * y21DotY21 - y10DotY21 * y10DotY21;

            // determinant > 0 means non-degenerate triangle (edges not parallel)
            if (determinant > 0.0) {
                // y1DotY10 = dot(y1, y10)
                const y1DotY10 = y1x * y10x + y1y * y10y + y1z * y10z;
                // y1DotY21 = dot(y1, y21)
                const y1DotY21 = y1x * y21x + y1y * y21y + y1z * y21z;
                const l0 = (y21DotY21 * y1DotY10 - y10DotY21 * y1DotY21) / determinant;
                const l1 = (y10DotY21 * y1DotY10 - y10DotY10 * y1DotY21) / determinant;
                triFloat[fBase + TRI_LAMBDA0] = l0;
                triFloat[fBase + TRI_LAMBDA1] = l1;
                // lambdaRelativeTo0 stays false (flag bit not set)

                // check if closest point lies within triangle interior
                // point is interior if: l0 >= 0, l1 >= 0, and l0 + l1 <= 1 (barycentric bounds)
                if (l0 > -EPA_BARYCENTRIC_EPSILON && l1 > -EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0 + EPA_BARYCENTRIC_EPSILON) {
                    triInt[iBase + TRI_FLAGS] |= TRI_FLAG_CLOSEST_POINT_INTERIOR;
                }
            }
        }
    }

    return tri;
}

export function initialize(state: EpaConvexHullBuilderState, idx1: number, idx2: number, idx3: number) {
    // reset bump allocator — all triangles are freed en masse
    state.triangleHighWatermark = 0;
    state.triangleFreeHead = -1;
    state.queue.length = 0;

    // create triangles (back to back)
    const t1 = createTriangle(state, idx1, idx2, idx3);
    const t2 = createTriangle(state, idx1, idx3, idx2);

    if (t1 === -1 || t2 === -1) {
        throw new Error('Failed to create triangles');
    }

    // link triangle edges
    linkTriangle(state, t1, 0, t2, 2);
    linkTriangle(state, t1, 1, t2, 1);
    linkTriangle(state, t1, 2, t2, 0);

    // always add both triangles to the priority queue
    pushTriangleToQueue(state, t1);
    pushTriangleToQueue(state, t2);
}

export function hasNextTriangle(state: EpaConvexHullBuilderState): boolean {
    return state.queue.length > 0;
}

export function peekClosestTriangleInQueue(state: EpaConvexHullBuilderState): number {
    return state.queue.length === 0 ? -1 : state.queue[0];
}

export function popClosestTriangleFromQueue(state: EpaConvexHullBuilderState): number {
    const queue = state.queue;
    if (queue.length === 0) return -1;
    const triFloat = state.triFloat;

    // binary heap pop: move root to end, then bubble down
    const temp = queue[queue.length - 1];
    queue[queue.length - 1] = queue[0];
    queue[0] = temp;

    const count = queue.length - 1;
    let largest = 0;

    while (true) {
        let child = (largest << 1) + 1;
        if (child >= count) break;

        const prevLargest = largest;

        // max heap: find child with smallest closestLengthSq
        if (
            triFloat[queue[largest] * TRI_FLOAT_STRIDE + TRI_CLOSEST_LENGTH_SQ] >
            triFloat[queue[child] * TRI_FLOAT_STRIDE + TRI_CLOSEST_LENGTH_SQ]
        ) {
            largest = child;
        }

        ++child;

        if (
            child < count &&
            triFloat[queue[largest] * TRI_FLOAT_STRIDE + TRI_CLOSEST_LENGTH_SQ] >
                triFloat[queue[child] * TRI_FLOAT_STRIDE + TRI_CLOSEST_LENGTH_SQ]
        ) {
            largest = child;
        }

        if (prevLargest === largest) break;

        const tempElement = queue[prevLargest];
        queue[prevLargest] = queue[largest];
        queue[largest] = tempElement;
    }

    return queue.pop()!;
}

export function findFacingTriangle(
    state: EpaConvexHullBuilderState,
    points: number[],
    offset: number,
    outBestDistSq: { value: number },
): number {
    let best = -1;
    let bestDistSq = 0.0;
    const triFloat = state.triFloat;
    const triInt = state.triInt;

    for (let i = 0; i < state.queue.length; i++) {
        const t = state.queue[i];
        if ((triInt[t * TRI_INT_STRIDE + TRI_FLAGS] & TRI_FLAG_REMOVED) !== 0) continue;

        const fBase = t * TRI_FLOAT_STRIDE;
        const abx = points[offset] - triFloat[fBase + TRI_CENTROID];
        const aby = points[offset + 1] - triFloat[fBase + TRI_CENTROID + 1];
        const abz = points[offset + 2] - triFloat[fBase + TRI_CENTROID + 2];
        const nx = triFloat[fBase + TRI_NORMAL];
        const ny = triFloat[fBase + TRI_NORMAL + 1];
        const nz = triFloat[fBase + TRI_NORMAL + 2];
        const dot = nx * abx + ny * aby + nz * abz;
        if (dot > 0.0) {
            const normalLenSq = nx * nx + ny * ny + nz * nz;
            const distSq = (dot * dot) / normalLenSq;
            if (distSq > bestDistSq) {
                best = t;
                bestDistSq = distSq;
            }
        }
    }

    outBestDistSq.value = bestDistSq;
    return best;
}

export function freeTriangle(state: EpaConvexHullBuilderState, tri: number): void {
    state.triInt[tri * TRI_INT_STRIDE + TRI_NEXT_FREE] = state.triangleFreeHead;
    state.triangleFreeHead = tri;
}

export function unlinkTriangle(state: EpaConvexHullBuilderState, tri: number): void {
    const triInt = state.triInt;
    const base = tri * TRI_INT_STRIDE;

    // unlink edge 0
    const n0 = triInt[base + 0 * 3 + EDGE_NEIGHBOUR_TRIANGLE];
    if (n0 !== -1) {
        const ne0 = triInt[base + 0 * 3 + EDGE_NEIGHBOUR_EDGE];
        triInt[n0 * TRI_INT_STRIDE + ne0 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
        triInt[base + 0 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
    }
    // unlink edge 1
    const n1 = triInt[base + 1 * 3 + EDGE_NEIGHBOUR_TRIANGLE];
    if (n1 !== -1) {
        const ne1 = triInt[base + 1 * 3 + EDGE_NEIGHBOUR_EDGE];
        triInt[n1 * TRI_INT_STRIDE + ne1 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
        triInt[base + 1 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
    }
    // unlink edge 2
    const n2 = triInt[base + 2 * 3 + EDGE_NEIGHBOUR_TRIANGLE];
    if (n2 !== -1) {
        const ne2 = triInt[base + 2 * 3 + EDGE_NEIGHBOUR_EDGE];
        triInt[n2 * TRI_INT_STRIDE + ne2 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
        triInt[base + 2 * 3 + EDGE_NEIGHBOUR_TRIANGLE] = -1;
    }

    // if this triangle is not in the priority queue, we can delete it now
    if ((triInt[base + TRI_FLAGS] & TRI_FLAG_IN_QUEUE) === 0) {
        freeTriangle(state, tri);
    }
}

export function findEdge(state: EpaConvexHullBuilderState, facingTriangle: number, pointOffset: number): boolean {
    const triInt = state.triInt;
    const positions = state.positions;
    const edges = state.edges;

    // clear output edges
    state.edgesSize = 0;

    // flag as removed
    triInt[facingTriangle * TRI_INT_STRIDE + TRI_FLAGS] |= TRI_FLAG_REMOVED;

    // build our own stack
    const stack = state.stack;
    let curStackPos = 0;

    // start with the triangle/edge provided
    stack[0] = facingTriangle;
    stack[1] = 0;
    stack[2] = -1;

    // next index that we expect to find
    let nextExpectedStartIdx = -1;

    while (true) {
        const sBase = curStackPos * STACK_STRIDE;
        const curTriangle = stack[sBase];
        const curEdge = stack[sBase + 1];

        // next iteration
        const curIter = stack[sBase + 2] + 1;
        stack[sBase + 2] = curIter;

        if (curIter >= 3) {
            // this triangle needs to be removed, unlink it now
            unlinkTriangle(state, curTriangle);

            // pop from stack
            if (--curStackPos < 0) {
                break;
            }
        } else {
            // visit neighbour — compute edge index as (edge + iter) % 3
            const edgeSum = curEdge + curIter;
            const edgeIdx = edgeSum >= 3 ? edgeSum - 3 : edgeSum;
            const eBase = curTriangle * TRI_INT_STRIDE + edgeIdx * 3;
            const n = triInt[eBase + EDGE_NEIGHBOUR_TRIANGLE];
            const nEdge = triInt[eBase + EDGE_NEIGHBOUR_EDGE];
            const eStartIndex = triInt[eBase + EDGE_START_INDEX];

            if (n !== -1 && (triInt[n * TRI_INT_STRIDE + TRI_FLAGS] & TRI_FLAG_REMOVED) === 0) {
                // check if vertex is on the front side of this triangle
                if (triangleIsFacing(state, n, positions, pointOffset)) {
                    // vertex on front, this triangle needs to be removed
                    triInt[n * TRI_INT_STRIDE + TRI_FLAGS] |= TRI_FLAG_REMOVED;

                    // add element to the stack
                    curStackPos++;
                    const nBase = curStackPos * STACK_STRIDE;
                    stack[nBase] = n;
                    stack[nBase + 1] = nEdge;
                    stack[nBase + 2] = 0;
                } else {
                    // detect islands - if edge doesn't connect to previous edge
                    if (eStartIndex !== nextExpectedStartIdx && nextExpectedStartIdx !== -1) {
                        return false;
                    }

                    // next expected index is the start index of our neighbour's edge
                    nextExpectedStartIdx = triInt[n * TRI_INT_STRIDE + nEdge * 3 + EDGE_START_INDEX];

                    // vertex behind, keep edge — copy into output edge list
                    const oBase = state.edgesSize * EDGE_STRIDE;
                    edges[oBase + EDGE_NEIGHBOUR_TRIANGLE] = n;
                    edges[oBase + EDGE_NEIGHBOUR_EDGE] = nEdge;
                    edges[oBase + EDGE_START_INDEX] = eStartIndex;
                    state.edgesSize++;
                }
            }
        }
    }

    // assert that we have a fully connected loop
    if (state.edgesSize !== 0 && edges[EDGE_START_INDEX] !== nextExpectedStartIdx) {
        return false;
    }

    // need at least 3 edges to form a valid hull
    return state.edgesSize >= 3;
}

export function addPoint(
    state: EpaConvexHullBuilderState,
    facingTriangle: number,
    idx: number,
    closestDistSq: number,
    outTriangles: NewTriangles,
): boolean {
    const triFloat = state.triFloat;
    const triInt = state.triInt;

    // find edge of convex hull of triangles that are not facing the new vertex
    state.edgesSize = 0;

    if (!findEdge(state, facingTriangle, idx * 3)) {
        return false;
    }

    // create new triangles and link them in a single pass
    const numEdges = state.edgesSize;
    const edges = state.edges;

    for (let i = 0; i < numEdges; i++) {
        const iNext = i + 1 < numEdges ? i + 1 : 0;
        const startIndex = edges[i * EDGE_STRIDE + EDGE_START_INDEX];
        const startIndexNext = edges[iNext * EDGE_STRIDE + EDGE_START_INDEX];

        const nt = createTriangle(state, startIndex, startIndexNext, idx);
        if (nt === -1) {
            return false;
        }

        outTriangles.push(nt);

        // check if we need to put this triangle in the priority queue
        const closestLengthSq = triFloat[nt * TRI_FLOAT_STRIDE + TRI_CLOSEST_LENGTH_SQ];
        const closestPointInterior = (triInt[nt * TRI_INT_STRIDE + TRI_FLAGS] & TRI_FLAG_CLOSEST_POINT_INTERIOR) !== 0;
        if ((closestPointInterior && closestLengthSq < closestDistSq) || closestLengthSq < 0.0) {
            pushTriangleToQueue(state, nt);
        }
    }

    // link edges in second pass (can't do in first pass because we need all triangles created)
    for (let i = 0; i < numEdges; i++) {
        const iNext = i + 1 < numEdges ? i + 1 : 0;
        const t1 = outTriangles[i];
        const t2 = outTriangles[iNext];
        const eBase = i * EDGE_STRIDE;

        linkTriangle(state, t1, 0, edges[eBase + EDGE_NEIGHBOUR_TRIANGLE], edges[eBase + EDGE_NEIGHBOUR_EDGE]);
        linkTriangle(state, t1, 1, t2, 2);
    }

    return true;
}
