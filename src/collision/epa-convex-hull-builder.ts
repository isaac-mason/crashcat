import { type Vec3, vec3 } from 'mathcat';

const EPA_MAX_TRIANGLES = 256;
const EPA_MAX_EDGE_LENGTH = 128;
const EPA_MIN_TRIANGLE_AREA = 1e-10;
const EPA_BARYCENTRIC_EPSILON = 1e-3;

export type Edge = {
    neighbourTriangle: Triangle | null;
    neighbourEdge: number;
    startIndex: number;
};

export function createEdge(): Edge {
    return {
        neighbourTriangle: null,
        neighbourEdge: 0,
        startIndex: 0,
    };
}

export function copyEdge(dest: Edge, src: Edge): void {
    dest.neighbourTriangle = src.neighbourTriangle;
    dest.neighbourEdge = src.neighbourEdge;
    dest.startIndex = src.startIndex;
}

export type Triangle = {
    edge: [Edge, Edge, Edge];
    normal: Vec3;
    centroid: Vec3;
    closestLengthSq: number;
    lambda: [number, number];
    lambdaRelativeTo0: boolean;
    closestPointInterior: boolean;
    removed: boolean;
    inQueue: boolean;
    iteration: number;
    index: number;
};

export function allocateTriangle(): Triangle {
    return {
        edge: [createEdge(), createEdge(), createEdge()],
        normal: vec3.create(),
        centroid: vec3.create(),
        closestLengthSq: Infinity,
        lambda: [0, 0],
        lambdaRelativeTo0: false,
        closestPointInterior: false,
        removed: false,
        inQueue: false,
        iteration: 0,
        index: -1,
    };
}

const _vectorAB = /* @__PURE__ */ vec3.create();

export function triangleIsFacing(triangle: Triangle, position: Vec3): boolean {
    // vectorAB = position - centroid
    const abx = position[0] - triangle.centroid[0];
    const aby = position[1] - triangle.centroid[1];
    const abz = position[2] - triangle.centroid[2];
    // dot(normal, vectorAB) > 0
    return triangle.normal[0] * abx + triangle.normal[1] * aby + triangle.normal[2] * abz > 0.0;
}

export type Points = {
    values: Vec3[];
    size: number;
};

export function createPoints(capacity: number): Points {
    const values: Vec3[] = [];
    for (let i = 0; i < capacity; i++) {
        values.push(vec3.create());
    }
    return { values, size: 0 };
}

export type Edges = {
    values: Edge[];
    size: number;
};

export function createEdges(capacity: number): Edges {
    const values: Edge[] = [];
    for (let i = 0; i < capacity; i++) {
        values.push(createEdge());
    }
    return { values, size: 0 };
}

function pushTriangleToQueue(state: EpaConvexHullBuilderState, triangle: Triangle): void {
    const queue = state.queue;
    queue.push(triangle);
    triangle.inQueue = true;

    // binary heap push: bubble up the new element
    let current = queue.length - 1;
    while (current > 0) {
        const currentElement = queue[current];
        const parent = (current - 1) >> 1;
        const parentElement = queue[parent];

        // min heap: parent should have smaller closestLengthSq than child
        if (parentElement.closestLengthSq > currentElement.closestLengthSq) {
            queue[parent] = currentElement;
            queue[current] = parentElement;
            current = parent;
        } else {
            break;
        }
    }
}

type StackEntry = {
    triangle: Triangle | null;
    edge: number;
    iter: number;
};

function createStackEntry(): StackEntry {
    return {
        triangle: null,
        edge: 0,
        iter: -1,
    };
}

export type NewTriangles = Triangle[];

export type EpaConvexHullBuilderState = {
    triangles: Triangle[];
    freeTriangles: number[];

    queue: Triangle[];
    positions: Vec3[];
    stack: StackEntry[];
    edges: Edges;
};

export function init(): EpaConvexHullBuilderState {
    const stack: StackEntry[] = [];
    for (let i = 0; i < EPA_MAX_EDGE_LENGTH; i++) {
        stack.push(createStackEntry());
    }

    // initialize triangle pool
    const triangles: Triangle[] = [];
    for (let i = 0; i < EPA_MAX_TRIANGLES; i++) {
        const triangle = allocateTriangle();
        triangle.index = i;
        triangles.push(triangle);
    }

    return {
        triangles,
        freeTriangles: [],
        queue: [],
        positions: [],
        stack,
        edges: createEdges(EPA_MAX_EDGE_LENGTH),
    };
}

export function linkTriangle(t1: Triangle, edge1: number, t2: Triangle, edge2: number) {
    const e1 = t1.edge[edge1];
    const e2 = t2.edge[edge2];

    // Link up
    e1.neighbourTriangle = t2;
    e1.neighbourEdge = edge2;
    e2.neighbourTriangle = t1;
    e2.neighbourEdge = edge1;
}

export function createTriangle(state: EpaConvexHullBuilderState, idx1: number, idx2: number, idx3: number): Triangle | null {
    let triangle: Triangle;
    let index: number;
    if (state.freeTriangles.length > 0) {
        index = state.freeTriangles.pop()!;
        triangle = state.triangles[index];
    } else {
        index = state.triangles.length;
        if (index >= EPA_MAX_TRIANGLES) return null;
        triangle = allocateTriangle();
        triangle.index = index;
        state.triangles.push(triangle);
    }

    // reset defaults
    triangle.closestLengthSq = Infinity;
    triangle.lambda[0] = 0.0;
    triangle.lambda[1] = 0.0;
    triangle.lambdaRelativeTo0 = false;
    triangle.closestPointInterior = false;
    triangle.removed = false;
    triangle.inQueue = false;
    triangle.iteration = 0;

    // fill in indexes
    triangle.edge[0].startIndex = idx1;
    triangle.edge[1].startIndex = idx2;
    triangle.edge[2].startIndex = idx3;

    // clear links
    triangle.edge[0].neighbourTriangle = null;
    triangle.edge[1].neighbourTriangle = null;
    triangle.edge[2].neighbourTriangle = null;

    // get vertex positions
    const positions = state.positions;
    const y0 = positions[idx1];
    const y1 = positions[idx2];
    const y2 = positions[idx3];

    // calculate centroid
    const cx = (y0[0] + y1[0] + y2[0]) / 3.0;
    const cy = (y0[1] + y1[1] + y2[1]) / 3.0;
    const cz = (y0[2] + y1[2] + y2[2]) / 3.0;
    triangle.centroid[0] = cx;
    triangle.centroid[1] = cy;
    triangle.centroid[2] = cz;

    // calculate edges
    // y10 = y1 - y0
    const y10x = y1[0] - y0[0];
    const y10y = y1[1] - y0[1];
    const y10z = y1[2] - y0[2];

    // y20 = y2 - y0
    const y20x = y2[0] - y0[0];
    const y20y = y2[1] - y0[1];
    const y20z = y2[2] - y0[2];

    // y21 = y2 - y1
    const y21x = y2[0] - y1[0];
    const y21y = y2[1] - y1[1];
    const y21z = y2[2] - y1[2];

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
        triangle.normal[0] = nx;
        triangle.normal[1] = ny;
        triangle.normal[2] = nz;

        // check if triangle is degenerate (area too small)
        const normalLenSq = nx * nx + ny * ny + nz * nz;
        if (normalLenSq > EPA_MIN_TRIANGLE_AREA) {
            // compute signed squared distance from origin to triangle plane
            // cDotN = dot(centroid, normal)
            const cDotN = cx * nx + cy * ny + cz * nz;
            // Math.abs(cDotN) * cDotN = sign(cDotN) * cDotN^2, preserves sign while squaring
            triangle.closestLengthSq = (Math.abs(cDotN) * cDotN) / normalLenSq;

            // calculate barycentric coordinates of closest point on triangle to origin
            // y10DotY10 = dot(y10, y10)
            const y10DotY10 = y10x * y10x + y10y * y10y + y10z * y10z;
            // y10DotY20 = dot(y10, y20)
            const y10DotY20 = y10x * y20x + y10y * y20y + y10z * y20z;
            const determinant = y10DotY10 * y20DotY20 - y10DotY20 * y10DotY20;

            // determinant > 0 means non-degenerate triangle (edges not parallel)
            if (determinant > 0.0) {
                // y0DotY10 = dot(y0, y10)
                const y0DotY10 = y0[0] * y10x + y0[1] * y10y + y0[2] * y10z;
                // y0DotY20 = dot(y0, y20)
                const y0DotY20 = y0[0] * y20x + y0[1] * y20y + y0[2] * y20z;
                const l0 = (y10DotY20 * y0DotY20 - y20DotY20 * y0DotY10) / determinant;
                const l1 = (y10DotY20 * y0DotY10 - y10DotY10 * y0DotY20) / determinant;
                triangle.lambda[0] = l0;
                triangle.lambda[1] = l1;
                triangle.lambdaRelativeTo0 = true;

                // check if closest point lies within triangle interior
                // point is interior if: l0 >= 0, l1 >= 0, and l0 + l1 <= 1 (barycentric bounds)
                if (l0 > -EPA_BARYCENTRIC_EPSILON && l1 > -EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0 + EPA_BARYCENTRIC_EPSILON) {
                    triangle.closestPointInterior = true;
                }
            }
        }
    } else {
        // we select the edges y10 and y21
        // normal = cross(y10, y21)
        const nx = y10y * y21z - y10z * y21y;
        const ny = y10z * y21x - y10x * y21z;
        const nz = y10x * y21y - y10y * y21x;
        triangle.normal[0] = nx;
        triangle.normal[1] = ny;
        triangle.normal[2] = nz;

        // check if triangle is degenerate (area too small)
        const normalLenSq = nx * nx + ny * ny + nz * nz;
        if (normalLenSq > EPA_MIN_TRIANGLE_AREA) {
            // compute signed squared distance from origin to triangle plane
            // cDotN = dot(centroid, normal)
            const cDotN = cx * nx + cy * ny + cz * nz;
            // Math.abs(cDotN) * cDotN = sign(cDotN) * cDotN^2, preserves sign while squaring
            triangle.closestLengthSq = (Math.abs(cDotN) * cDotN) / normalLenSq;

            // calculate barycentric coordinates of closest point on triangle to origin (y1 as reference)
            // y10DotY10 = dot(y10, y10)
            const y10DotY10 = y10x * y10x + y10y * y10y + y10z * y10z;
            // y10DotY21 = dot(y10, y21)
            const y10DotY21 = y10x * y21x + y10y * y21y + y10z * y21z;
            const determinant = y10DotY10 * y21DotY21 - y10DotY21 * y10DotY21;

            // determinant > 0 means non-degenerate triangle (edges not parallel)
            if (determinant > 0.0) {
                // y1DotY10 = dot(y1, y10)
                const y1DotY10 = y1[0] * y10x + y1[1] * y10y + y1[2] * y10z;
                // y1DotY21 = dot(y1, y21)
                const y1DotY21 = y1[0] * y21x + y1[1] * y21y + y1[2] * y21z;
                const l0 = (y21DotY21 * y1DotY10 - y10DotY21 * y1DotY21) / determinant;
                const l1 = (y10DotY21 * y1DotY10 - y10DotY10 * y1DotY21) / determinant;
                triangle.lambda[0] = l0;
                triangle.lambda[1] = l1;
                triangle.lambdaRelativeTo0 = false;

                // check if closest point lies within triangle interior
                // point is interior if: l0 >= 0, l1 >= 0, and l0 + l1 <= 1 (barycentric bounds)
                if (l0 > -EPA_BARYCENTRIC_EPSILON && l1 > -EPA_BARYCENTRIC_EPSILON && l0 + l1 < 1.0 + EPA_BARYCENTRIC_EPSILON) {
                    triangle.closestPointInterior = true;
                }
            }
        }
    }

    return triangle;
}

export function initialize(state: EpaConvexHullBuilderState, idx1: number, idx2: number, idx3: number) {
    // release all triangles to free list (clear pool)
    state.freeTriangles.length = state.triangles.length;
    for (let i = 0; i < state.triangles.length; i++) {
        state.freeTriangles[i] = state.triangles.length - 1 - i;
    }
    state.queue.length = 0;

    // create triangles (back to back)
    const t1 = createTriangle(state, idx1, idx2, idx3);
    const t2 = createTriangle(state, idx1, idx3, idx2);

    if (!t1 || !t2) {
        throw new Error('Failed to create triangles');
    }

    // link triangle edges
    linkTriangle(t1, 0, t2, 2);
    linkTriangle(t1, 1, t2, 1);
    linkTriangle(t1, 2, t2, 0);

    // always add both triangles to the priority queue
    pushTriangleToQueue(state, t1);
    pushTriangleToQueue(state, t2);
}

export function hasNextTriangle(state: EpaConvexHullBuilderState): boolean {
    return state.queue.length > 0;
}

export function peekClosestTriangleInQueue(state: EpaConvexHullBuilderState): Triangle | null {
    return state.queue.length === 0 ? null : state.queue[0];
}

export function popClosestTriangleFromQueue(state: EpaConvexHullBuilderState): Triangle | null {
    const queue = state.queue;
    if (queue.length === 0) return null;

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
        if (queue[largest].closestLengthSq > queue[child].closestLengthSq) {
            largest = child;
        }

        ++child;

        if (child < count && queue[largest].closestLengthSq > queue[child].closestLengthSq) {
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
    position: Vec3,
    outBestDistSq: { value: number },
): Triangle | null {
    let best: Triangle | null = null;
    let bestDistSq = 0.0;

    for (let i = 0; i < state.queue.length; i++) {
        const t = state.queue[i];
        if (!t || t.removed) continue;

        vec3.subtract(_vectorAB, position, t.centroid);
        const dot = vec3.dot(t.normal, _vectorAB);
        if (dot > 0.0) {
            const distSq = (dot * dot) / vec3.squaredLength(t.normal);
            if (distSq > bestDistSq) {
                best = t;
                bestDistSq = distSq;
            }
        }
    }

    outBestDistSq.value = bestDistSq;
    return best;
}

export function freeTriangle(state: EpaConvexHullBuilderState, triangle: Triangle): void {
    state.freeTriangles.push(triangle.index);
}

export function unlinkTriangle(state: EpaConvexHullBuilderState, triangle: Triangle): void {
    // unlink from neighbours
    for (let i = 0; i < 3; i++) {
        const edge = triangle.edge[i];
        if (edge.neighbourTriangle !== null) {
            const neighbourEdge = edge.neighbourTriangle.edge[edge.neighbourEdge];
            neighbourEdge.neighbourTriangle = null;
            edge.neighbourTriangle = null;
        }
    }

    // if this triangle is not in the priority queue, we can delete it now
    if (!triangle.inQueue) {
        freeTriangle(state, triangle);
    }
}

export function findEdge(state: EpaConvexHullBuilderState, facingTriangle: Triangle, vertex: Vec3, outEdges: Edges): boolean {
    // clear output edges
    outEdges.size = 0;

    // flag as removed
    facingTriangle.removed = true;

    // build our own stack
    const stack = state.stack;
    let curStackPos = 0;

    // start with the triangle/edge provided
    stack[0].triangle = facingTriangle;
    stack[0].edge = 0;
    stack[0].iter = -1;

    // next index that we expect to find
    let nextExpectedStartIdx = -1;

    while (true) {
        const curEntry = stack[curStackPos];

        // next iteration
        if (++curEntry.iter >= 3) {
            // this triangle needs to be removed, unlink it now
            unlinkTriangle(state, curEntry.triangle!);

            // pop from stack
            if (--curStackPos < 0) {
                break;
            }
        } else {
            // visit neighbour
            const e = curEntry.triangle!.edge[(curEntry.edge + curEntry.iter) % 3];
            const n = e.neighbourTriangle;
            if (n !== null && !n.removed) {
                // check if vertex is on the front side of this triangle
                if (triangleIsFacing(n, vertex)) {
                    // vertex on front, this triangle needs to be removed
                    n.removed = true;

                    // add element to the stack
                    curStackPos++;
                    const newEntry = stack[curStackPos];
                    newEntry.triangle = n;
                    newEntry.edge = e.neighbourEdge;
                    newEntry.iter = 0;
                } else {
                    // detect islands - if edge doesn't connect to previous edge
                    if (e.startIndex !== nextExpectedStartIdx && nextExpectedStartIdx !== -1) {
                        return false;
                    }

                    // next expected index is the start index of our neighbour's edge
                    nextExpectedStartIdx = n.edge[e.neighbourEdge].startIndex;

                    // vertex behind, keep edge
                    copyEdge(outEdges.values[outEdges.size++], e);
                }
            }
        }
    }

    // assert that we have a fully connected loop
    const front = outEdges.size === 0 ? null : outEdges.values[0];
    if (front && front.startIndex !== nextExpectedStartIdx) {
        return false;
    }

    // need at least 3 edges to form a valid hull
    return outEdges.size >= 3;
}

export function addPoint(
    state: EpaConvexHullBuilderState,
    facingTriangle: Triangle,
    idx: number,
    closestDistSq: number,
    outTriangles: NewTriangles,
): boolean {
    // get position
    const pos = state.positions[idx];

    // find edge of convex hull of triangles that are not facing the new vertex
    const edges = state.edges;
    edges.size = 0;

    if (!findEdge(state, facingTriangle, pos, edges)) {
        return false;
    }

    // create new triangles and link them in a single pass
    const numEdges = edges.size;
    const edgesValues = edges.values;

    for (let i = 0; i < numEdges; i++) {
        const iNext = (i + 1) % numEdges;
        const edge = edgesValues[i];
        const edgeNext = edgesValues[iNext];

        const nt = createTriangle(state, edge.startIndex, edgeNext.startIndex, idx);
        if (!nt) {
            return false;
        }

        outTriangles.push(nt);

        // check if we need to put this triangle in the priority queue
        if ((nt.closestPointInterior && nt.closestLengthSq < closestDistSq) || nt.closestLengthSq < 0.0) {
            pushTriangleToQueue(state, nt);
        }
    }

    // link edges in second pass (can't do in first pass because we need all triangles created)
    for (let i = 0; i < numEdges; i++) {
        const iNext = (i + 1) % numEdges;
        const t1 = outTriangles[i];
        const t2 = outTriangles[iNext];
        const edge = edgesValues[i];

        linkTriangle(t1, 0, edge.neighbourTriangle!, edge.neighbourEdge);
        linkTriangle(t1, 1, t2, 2);
    }

    return true;
}
