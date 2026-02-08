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

    const seen = new Set<string>();
    let writeIdx = 0;

    for (let readIdx = 0; readIdx < validTriangleCount; readIdx++) {
        const readOffset = readIdx * TRIANGLE_STRIDE;
        const ia = triangleBuffer[readOffset + OFFSET_INDEX_A];
        const ib = triangleBuffer[readOffset + OFFSET_INDEX_B];
        const ic = triangleBuffer[readOffset + OFFSET_INDEX_C];
        const materialId = triangleBuffer[readOffset + OFFSET_MATERIAL_ID];

        // rotate indices so lowest comes first (preserves winding order)
        let iaCanon: number, ibCanon: number, icCanon: number;
        if (ia < ib) {
            if (ia < ic) {
                iaCanon = ia;
                ibCanon = ib;
                icCanon = ic;
            } else {
                iaCanon = ic;
                ibCanon = ia;
                icCanon = ib;
            }
        } else {
            if (ib < ic) {
                iaCanon = ib;
                ibCanon = ic;
                icCanon = ia;
            } else {
                iaCanon = ic;
                ibCanon = ia;
                icCanon = ib;
            }
        }

        const key = `${iaCanon},${ibCanon},${icCanon},${materialId}`;

        if (!seen.has(key)) {
            seen.add(key);

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

const WELD_GRID_INV = 1e8; // 1 / 1e-8

function deduplicateVertices(inputPositions: number[]): {
    positions: number[];
    indexMap: number[];
} {
    const vertexCount = Math.floor(inputPositions.length / 3);
    const vertexMap = new Map<string, number>();
    const positions: number[] = [];
    const indexMap = new Array<number>(vertexCount);
    let uniqueCount = 0;

    for (let i = 0; i < vertexCount; i++) {
        const qx = Math.round(inputPositions[i * 3] * WELD_GRID_INV);
        const qy = Math.round(inputPositions[i * 3 + 1] * WELD_GRID_INV);
        const qz = Math.round(inputPositions[i * 3 + 2] * WELD_GRID_INV);
        const key = `${qx},${qy},${qz}`;

        let idx = vertexMap.get(key);
        if (idx === undefined) {
            idx = uniqueCount++;
            vertexMap.set(key, idx);
            positions.push(inputPositions[i * 3], inputPositions[i * 3 + 1], inputPositions[i * 3 + 2]);
        }
        indexMap[i] = idx;
    }

    return { positions, indexMap };
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
};

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
