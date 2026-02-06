import { vec3 } from 'mathcat';
import type { BvhSplitStrategy } from './triangle-mesh-bvh';
import * as triangleMeshBvh from './triangle-mesh-bvh';
import type { TriangleMeshData } from './triangle-mesh-data';
import {
    OFFSET_ACTIVE_EDGES,
    OFFSET_INDEX_A,
    OFFSET_INDEX_B,
    OFFSET_INDEX_C,
    OFFSET_MATERIAL_ID,
    OFFSET_NORMAL_X,
    OFFSET_NORMAL_Y,
    OFFSET_NORMAL_Z,
    TRIANGLE_STRIDE,
} from './triangle-mesh-data';

/** settings for building a triangle mesh */
export type TriangleMeshBuilderSettings = {
    positions: number[];
    indices: number[];
    /**
     * Optional per-triangle material indices.
     * Length should match number of triangles (indices.length / 3).
     * Default: -1 for all triangles (no material).
     */
    materialIds?: number[];
    /**
     * Threshold for detecting degenerate triangles.
     * A triangle is considered degenerate if |cross product| < this value.
     */
    degenerateTolerance: number;
    /**
     * BVH split strategy.
     */
    bvhSplitStrategy: BvhSplitStrategy;
    /**
     * Maximum triangles per leaf node in the BVH.
     */
    bvhMaxLeafTris: number;
    /**
     * Cosine threshold for active edge determination.
     * Edges with cos(dihedral_angle) >= this value are considered smooth/inactive.
     */
    activeEdgeCosThresholdAngle: number;
};

/** result of building a triangle mesh */
export type TriangleMeshBuildResult = {
    data: TriangleMeshData;
    bvh: triangleMeshBvh.TriangleMeshBVH;
};

const _a = vec3.create();
const _b = vec3.create();
const _c = vec3.create();
const _edge1 = vec3.create();
const _edge2 = vec3.create();
const _crossProduct = vec3.create();
const _normal = vec3.create();

/** build a triangle mesh from positions and indices */
export function buildTriangleMesh(settings: TriangleMeshBuilderSettings): TriangleMeshBuildResult {
    const tolerance = settings.degenerateTolerance;

    /* deduplicate vertices */
    const { positions, indexMap } = deduplicateVertices(settings.positions);

    /* create initial triangles in interleaved format */
    const rawTriangleCount = Math.floor(settings.indices.length / 3);
    const triangleBuffer = new Array(rawTriangleCount * TRIANGLE_STRIDE).fill(0);

    let validTriangleCount = 0;

    for (let i = 0; i < rawTriangleCount; i++) {
        // get vertex indices from input
        const ia_input = settings.indices[i * 3 + 0];
        const ib_input = settings.indices[i * 3 + 1];
        const ic_input = settings.indices[i * 3 + 2];

        // map to deduplicated indices
        const ia = indexMap[ia_input];
        const ib = indexMap[ib_input];
        const ic = indexMap[ic_input];

        // get vertex positions
        _a[0] = positions[ia * 3 + 0];
        _a[1] = positions[ia * 3 + 1];
        _a[2] = positions[ia * 3 + 2];

        _b[0] = positions[ib * 3 + 0];
        _b[1] = positions[ib * 3 + 1];
        _b[2] = positions[ib * 3 + 2];

        _c[0] = positions[ic * 3 + 0];
        _c[1] = positions[ic * 3 + 1];
        _c[2] = positions[ic * 3 + 2];

        // check for degeneracy
        vec3.subtract(_edge1, _b, _a);
        vec3.subtract(_edge2, _c, _a);
        vec3.cross(_crossProduct, _edge1, _edge2);

        const crossMagnitude = vec3.length(_crossProduct);

        if (crossMagnitude < tolerance) {
            // degenerate triangle, skip
            continue;
        }

        // compute and normalize normal
        vec3.normalize(_normal, _crossProduct);

        // store triangle in buffer at validTriangleCount position
        const offset = validTriangleCount * TRIANGLE_STRIDE;

        // store indices
        triangleBuffer[offset + OFFSET_INDEX_A] = ia;
        triangleBuffer[offset + OFFSET_INDEX_B] = ib;
        triangleBuffer[offset + OFFSET_INDEX_C] = ic;

        // store normal
        triangleBuffer[offset + OFFSET_NORMAL_X] = _normal[0];
        triangleBuffer[offset + OFFSET_NORMAL_Y] = _normal[1];
        triangleBuffer[offset + OFFSET_NORMAL_Z] = _normal[2];

        // store active edges (default: all active, will be updated later)
        triangleBuffer[offset + OFFSET_ACTIVE_EDGES] = 0b111;

        // store material index
        const materialId = settings.materialIds?.[i] ?? -1;
        triangleBuffer[offset + OFFSET_MATERIAL_ID] = materialId;

        // reserved slots already initialized to 0
        validTriangleCount++;
    }

    // trim buffer to actual valid triangle count
    triangleBuffer.length = validTriangleCount * TRIANGLE_STRIDE;

    /* remove duplicate triangles */
    let finalTriangleCount = 0;

    const seen = new Set<bigint>();
    let writeIdx = 0;

    for (let readIdx = 0; readIdx < validTriangleCount; readIdx++) {
        const readOffset = readIdx * TRIANGLE_STRIDE;
        let ia = triangleBuffer[readOffset + OFFSET_INDEX_A];
        let ib = triangleBuffer[readOffset + OFFSET_INDEX_B];
        let ic = triangleBuffer[readOffset + OFFSET_INDEX_C];

        // sort indices to canonical form
        if (ia > ib) {
            const t = ia;
            ia = ib;
            ib = t;
        }
        if (ib > ic) {
            const t = ib;
            ib = ic;
            ic = t;
        }
        if (ia > ib) {
            const t = ia;
            ia = ib;
            ib = t;
        }

        // pack into BigInt: (ia << 64) | (ib << 32) | ic
        const key = (BigInt(ia) << 64n) | (BigInt(ib) << 32n) | BigInt(ic);

        if (!seen.has(key)) {
            seen.add(key);

            // move triangle to write position if needed (in-place compaction)
            if (writeIdx !== readIdx) {
                const writeOffset = writeIdx * TRIANGLE_STRIDE;
                for (let j = 0; j < TRIANGLE_STRIDE; j++) {
                    triangleBuffer[writeOffset + j] = triangleBuffer[readOffset + j];
                }
            }
            writeIdx++;
        }
    }

    finalTriangleCount = writeIdx;

    // trim buffer to actual unique count
    triangleBuffer.length = finalTriangleCount * TRIANGLE_STRIDE;

    const finalTriangleBuffer = triangleBuffer;

    // create data structure
    const data: TriangleMeshData = {
        positions,
        triangleBuffer: finalTriangleBuffer,
        triangleCount: finalTriangleCount,
    };

    /* compute active edges based on dihedral angles */
    computeActiveEdges(data, settings.activeEdgeCosThresholdAngle);

    /* build BVH with cleaned triangles */
    const bvh = triangleMeshBvh.build(data, {
        strategy: settings.bvhSplitStrategy,
        maxLeafTris: settings.bvhMaxLeafTris,
    });

    return {
        data,
        bvh,
    };
}

function deduplicateVertices(inputPositions: number[]): {
    positions: number[];
    indexMap: number[];
} {
    const vertexCount = Math.floor(inputPositions.length / 3);
    const weldDistanceSq = 1e-16; // (1e-8)^2

    // initialize union-find: each vertex points to itself
    const weldedVertices = new Uint32Array(vertexCount);
    for (let i = 0; i < vertexCount; i++) {
        weldedVertices[i] = i;
    }

    // create vertex indices array
    const vertexIndices = new Uint32Array(vertexCount);
    for (let i = 0; i < vertexCount; i++) {
        vertexIndices[i] = i;
    }

    // scratch buffer for vertices on both sides of split plane
    const scratch = new Uint32Array(vertexCount);

    // recursively partition and weld vertices
    indexifyVerticesRecursively(
        inputPositions,
        vertexIndices,
        0,
        vertexCount,
        scratch,
        weldedVertices,
        weldDistanceSq,
        32, // max recursion depth
    );

    // flatten union-find structure and count unique vertices
    let numUniqueVertices = 0;
    for (let i = 0; i < vertexCount; i++) {
        weldedVertices[i] = weldedVertices[weldedVertices[i]];
        if (weldedVertices[i] === i) {
            numUniqueVertices++;
        }
    }

    // collect unique vertices and build index map
    const positions: number[] = [];
    positions.length = numUniqueVertices * 3;
    let writeIdx = 0;

    for (let i = 0; i < vertexCount; i++) {
        if (weldedVertices[i] === i) {
            // this is a unique vertex
            const newIdx = writeIdx++;
            positions[newIdx * 3 + 0] = inputPositions[i * 3 + 0];
            positions[newIdx * 3 + 1] = inputPositions[i * 3 + 1];
            positions[newIdx * 3 + 2] = inputPositions[i * 3 + 2];
            weldedVertices[i] = newIdx;
        } else {
            // duplicate vertex - use the remapped index
            weldedVertices[i] = weldedVertices[weldedVertices[i]];
        }
    }

    return { positions, indexMap: Array.from(weldedVertices) };
}

/**
 * Recursively partition vertices along spatial planes and weld nearby vertices.
 */
function indexifyVerticesRecursively(
    positions: number[],
    indices: Uint32Array,
    start: number,
    count: number,
    scratch: Uint32Array,
    weldedVertices: Uint32Array,
    weldDistanceSq: number,
    maxRecursion: number,
): void {
    // base case: use brute force for small clusters
    if (count <= 8 || maxRecursion === 0) {
        indexifyVerticesBruteForce(positions, indices, start, count, weldedVertices, weldDistanceSq);
        return;
    }

    // calculate bounding box
    let minX = Infinity,
        minY = Infinity,
        minZ = Infinity;
    let maxX = -Infinity,
        maxY = -Infinity,
        maxZ = -Infinity;

    for (let i = start; i < start + count; i++) {
        const idx = indices[i];
        const x = positions[idx * 3 + 0];
        const y = positions[idx * 3 + 1];
        const z = positions[idx * 3 + 2];

        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
    }

    // determine split axis (longest extent)
    const extentX = maxX - minX;
    const extentY = maxY - minY;
    const extentZ = maxZ - minZ;

    let splitAxis: number;
    let splitValue: number;

    if (extentX >= extentY && extentX >= extentZ) {
        splitAxis = 0;
        splitValue = (minX + maxX) * 0.5;
    } else if (extentY >= extentZ) {
        splitAxis = 1;
        splitValue = (minY + maxY) * 0.5;
    } else {
        splitAxis = 2;
        splitValue = (minZ + maxZ) * 0.5;
    }

    const weldDistance = Math.sqrt(weldDistanceSq);

    // partition vertices into left, right, and middle (on both sides)
    let left = start;
    let right = start + count;
    let scratchCount = 0;

    while (left < right) {
        const idx = indices[left];
        const value = positions[idx * 3 + splitAxis];
        const distanceToPlane = value - splitValue;

        if (distanceToPlane < -weldDistance) {
            // left side
            left++;
        } else if (distanceToPlane > weldDistance) {
            // right side - swap with last element
            right--;
            const temp = indices[left];
            indices[left] = indices[right];
            indices[right] = temp;
        } else {
            // on both sides - goes into scratch buffer
            scratch[scratchCount++] = indices[left];
            left++;
        }
    }

    // check if we made progress
    if (scratchCount === count) {
        // all vertices on split plane, fall back to brute force
        indexifyVerticesBruteForce(positions, indices, start, count, weldedVertices, weldDistanceSq);
        return;
    }

    // calculate partition sizes
    const numLeft = left - start;
    const numRight = start + count - right;

    // copy middle vertices to both partitions
    for (let i = 0; i < scratchCount; i++) {
        indices[start + numLeft + i] = scratch[i];
        indices[right + i] = scratch[i];
    }

    // recurse on both partitions
    indexifyVerticesRecursively(
        positions,
        indices,
        start,
        numLeft + scratchCount,
        scratch,
        weldedVertices,
        weldDistanceSq,
        maxRecursion - 1,
    );

    indexifyVerticesRecursively(
        positions,
        indices,
        right,
        numRight + scratchCount,
        scratch,
        weldedVertices,
        weldDistanceSq,
        maxRecursion - 1,
    );
}

/** brute force comparison of vertices in a small cluster. uses union-find to link nearby vertices */
function indexifyVerticesBruteForce(
    positions: number[],
    indices: Uint32Array,
    start: number,
    count: number,
    weldedVertices: Uint32Array,
    weldDistanceSq: number,
): void {
    // compare every vertex with every other vertex
    for (let i = start; i < start + count; i++) {
        const idx1 = indices[i];
        const x1 = positions[idx1 * 3 + 0];
        const y1 = positions[idx1 * 3 + 1];
        const z1 = positions[idx1 * 3 + 2];

        for (let j = i + 1; j < start + count; j++) {
            const idx2 = indices[j];
            const x2 = positions[idx2 * 3 + 0];
            const y2 = positions[idx2 * 3 + 1];
            const z2 = positions[idx2 * 3 + 2];

            // check distance
            const dx = x2 - x1;
            const dy = y2 - y1;
            const dz = z2 - z1;
            const distSq = dx * dx + dy * dy + dz * dz;

            if (distSq <= weldDistanceSq) {
                // find roots of both vertices
                let root1 = idx1;
                while (weldedVertices[root1] < root1) {
                    root1 = weldedVertices[root1];
                }

                let root2 = idx2;
                while (weldedVertices[root2] < root2) {
                    root2 = weldedVertices[root2];
                }

                // link higher to lower
                if (root1 !== root2) {
                    const lowest = root1 < root2 ? root1 : root2;
                    const highest = root1 < root2 ? root2 : root1;
                    weldedVertices[highest] = lowest;

                    // path compression
                    weldedVertices[idx1] = lowest;
                    weldedVertices[idx2] = lowest;
                }

                break; // found a match, move to next vertex
            }
        }
    }
}

const _normal0 = vec3.create();
const _normal1 = vec3.create();

/*
 * Compute which edges are "active" (sharp) based on dihedral angles.
 *
 * An edge is active if:
 * - It's a boundary edge (only one triangle uses it)
 * - It's a non-manifold edge (more than 2 triangles use it)
 * - The dihedral angle threshold is disabled (cosThreshold < 0.0)
 * - The dihedral angle between adjacent triangles exceeds the threshold (cos(angle) < cosThreshold)
 *
 * Active edges are important for collision detection to avoid smoothing at sharp features.
 */

/**
 * Edge reference storage matching Jolt's approach.
 * Stores up to 2 triangle references inline. When a 3rd triangle uses the edge,
 * we mark it as non-manifold immediately without storing additional references.
 */
type EdgeRefs = {
    count: number;
    tri0: number; // packed (triIdx << 3 | edgeBit)
    tri1: number; // packed (triIdx << 3 | edgeBit)
}

function computeActiveEdges(data: TriangleMeshData, cosThreshold: number): void {
    // early exit: if threshold is negative, all edges remain active
    if (cosThreshold < 0.0) {
        return;
    }

    // build edge-to-triangles map
    const edgeMap = new Map<number, EdgeRefs>();

    // phase 1: build - collect edges and mark non-manifold edges immediately
    for (let triIdx = 0; triIdx < data.triangleCount; triIdx++) {
        const offset = triIdx * TRIANGLE_STRIDE;
        const ia = data.triangleBuffer[offset + OFFSET_INDEX_A];
        const ib = data.triangleBuffer[offset + OFFSET_INDEX_B];
        const ic = data.triangleBuffer[offset + OFFSET_INDEX_C];

        // process three edges: AB (bit 0), BC (bit 1), CA (bit 2)
        processEdge(data, edgeMap, ia, ib, triIdx, 0b001);
        processEdge(data, edgeMap, ib, ic, triIdx, 0b010);
        processEdge(data, edgeMap, ic, ia, triIdx, 0b100);
    }

    // phase 2: process - check manifold edges for smoothness
    for (const refs of edgeMap.values()) {
        // only process manifold edges (exactly 2 triangles share this edge)
        if (refs.count !== 2) {
            // boundary edges (count === 1) and non-manifold edges (count > 2) remain active
            continue;
        }

        // unpack triangle indices and edge masks
        const triIdx0 = refs.tri0 >>> 3;
        const edgeMask0 = refs.tri0 & 0b111;
        const triIdx1 = refs.tri1 >>> 3;
        const edgeMask1 = refs.tri1 & 0b111;

        // retrieve triangle normals
        const offset0 = triIdx0 * TRIANGLE_STRIDE;
        _normal0[0] = data.triangleBuffer[offset0 + OFFSET_NORMAL_X];
        _normal0[1] = data.triangleBuffer[offset0 + OFFSET_NORMAL_Y];
        _normal0[2] = data.triangleBuffer[offset0 + OFFSET_NORMAL_Z];

        const offset1 = triIdx1 * TRIANGLE_STRIDE;
        _normal1[0] = data.triangleBuffer[offset1 + OFFSET_NORMAL_X];
        _normal1[1] = data.triangleBuffer[offset1 + OFFSET_NORMAL_Y];
        _normal1[2] = data.triangleBuffer[offset1 + OFFSET_NORMAL_Z];

        // compute dihedral angle via dot product
        const dotProduct = _normal0[0] * _normal1[0] + _normal0[1] * _normal1[1] + _normal0[2] * _normal1[2];

        // if dot product >= threshold, the dihedral angle is small (smooth edge)
        if (dotProduct >= cosThreshold) {
            // mark edge as inactive in both triangles
            const activeEdges0 = data.triangleBuffer[offset0 + OFFSET_ACTIVE_EDGES];
            const activeEdges1 = data.triangleBuffer[offset1 + OFFSET_ACTIVE_EDGES];

            data.triangleBuffer[offset0 + OFFSET_ACTIVE_EDGES] = activeEdges0 & ~edgeMask0;
            data.triangleBuffer[offset1 + OFFSET_ACTIVE_EDGES] = activeEdges1 & ~edgeMask1;
        }
        // else: sharp edge (dot < threshold), edges remain active (initialized to 0b111)
    }
}

/** process an edge during the build phase */
function processEdge(
    data: TriangleMeshData,
    edgeMap: Map<number, EdgeRefs>,
    ia: number,
    ib: number,
    triIdx: number,
    edgeBit: number,
): void {
    const key = edgeKey(ia, ib);
    let refs = edgeMap.get(key);

    if (!refs) {
        // first triangle using this edge
        refs = { count: 1, tri0: (triIdx << 3) | edgeBit, tri1: 0 };
        edgeMap.set(key, refs);
    } else if (refs.count === 1) {
        // second triangle - store it
        refs.tri1 = (triIdx << 3) | edgeBit;
        refs.count = 2;
    } else {
        // third or more triangle - non-manifold edge, mark as active immediately
        // don't store the reference, just mark the edge as active
        const offset = triIdx * TRIANGLE_STRIDE;
        data.triangleBuffer[offset + OFFSET_ACTIVE_EDGES] |= edgeBit;
        refs.count++;
    }
}

/**
 * compute a unique numeric key for an edge using bit-packing.
 * uses 26 bits per vertex index, supporting up to 67,108,864 vertices per mesh.
 */
function edgeKey(ia: number, ib: number): number {
    const lo = ia < ib ? ia : ib;
    const hi = ia < ib ? ib : ia;
    // 2^26 = 67,108,864 - pack two 26-bit indices into safe integer range
    return lo * 67108864 + hi;
}
