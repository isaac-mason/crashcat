import { type Box3, box3, type Vec3, vec3 } from 'mathcat';
import type { TriangleMeshData } from './triangle-mesh-data';
import { swapTriangles } from './triangle-mesh-data';

/** cost of traversing a BVH node */
const TRAVERSAL_COST = 1;

/** cost of intersecting a triangle */
const TRIANGLE_INTERSECT_COST = 1.25;

/** number of bins to use for SAH */
const BIN_COUNT = 32;

/** Number of elements per node in the flat buffer */
export const NODE_STRIDE = 8;

// Offsets within a node
export const NODE_MIN_X = 0;
export const NODE_MIN_Y = 1;
export const NODE_MIN_Z = 2;
export const NODE_MAX_X = 3;
export const NODE_MAX_Y = 4;
export const NODE_MAX_Z = 5;
export const NODE_RIGHT_OR_TRI_START = 6; // internal: right child offset, leaf: triangle start
export const NODE_AXIS_OR_TRI_COUNT = 7; // internal: split axis (0-2), leaf: negative encoded count

/**
 * BVH split strategies.
 */
export enum BvhSplitStrategy {
    /**
     * CENTER: Split at midpoint of center bounds.
     * - Fast: O(n) with simple min/max computation
     * - Good balance for most geometries
     */
    CENTER,

    /**
     * AVERAGE: Split at arithmetic mean of centers.
     * - Fast: O(n) iteration to compute mean
     * - Can create unbalanced splits on skewed distributions
     */
    AVERAGE,

    /**
     * SAH: Surface Area Heuristic with binning.
     * - Slow: O(n) with high constant factor (binning + cost evaluation)
     * - Best quality: optimal expected query cost
     * - Use for static meshes queried many times
     */
    SAH,
}

/**
 * Settings for BVH construction.
 */
export type BvhBuildSettings = {
    /** Split strategy to use */
    strategy: BvhSplitStrategy;

    /** Maximum triangles per leaf node */
    maxLeafTris: number;
};

/**
 * Triangle mesh BVH stored as a flat number array.
 * Use accessor functions (nodeIsLeaf, nodeTriStart, etc.) to read node data.
 */
export type TriangleMeshBVH = {
    /** Flat packed node data. Use accessor functions to read. */
    buffer: number[];
};

/** Check if node at offset is a leaf */
export function nodeIsLeaf(buffer: number[], offset: number): boolean {
    return buffer[offset + NODE_AXIS_OR_TRI_COUNT] < 0;
}

/** Get triangle start index (leaf only) */
export function nodeTriStart(buffer: number[], offset: number): number {
    return buffer[offset + NODE_RIGHT_OR_TRI_START];
}

/** Get triangle count (leaf only). Decodes from negative flag. */
export function nodeTriCount(buffer: number[], offset: number): number {
    return -(buffer[offset + NODE_AXIS_OR_TRI_COUNT] + 1);
}

/** Get left child offset. Left child is always contiguous (offset + NODE_STRIDE). */
export function nodeLeft(offset: number): number {
    return offset + NODE_STRIDE;
}

/** Get right child offset (internal only) */
export function nodeRight(buffer: number[], offset: number): number {
    return buffer[offset + NODE_RIGHT_OR_TRI_START];
}

/** Get split axis (internal only): 0=x, 1=y, 2=z */
export function nodeSplitAxis(buffer: number[], offset: number): number {
    return buffer[offset + NODE_AXIS_OR_TRI_COUNT];
}

/** Copy bounds into existing Box3 (avoids allocation) */
export function nodeGetBounds(buffer: number[], offset: number, out: Box3): void {
    out[0][0] = buffer[offset + NODE_MIN_X];
    out[0][1] = buffer[offset + NODE_MIN_Y];
    out[0][2] = buffer[offset + NODE_MIN_Z];
    out[1][0] = buffer[offset + NODE_MAX_X];
    out[1][1] = buffer[offset + NODE_MAX_Y];
    out[1][2] = buffer[offset + NODE_MAX_Z];
}

/** Get center of node bounds (for distance-based sorting) */
export function nodeGetCenter(buffer: number[], offset: number, out: Vec3): void {
    out[0] = (buffer[offset + NODE_MIN_X] + buffer[offset + NODE_MAX_X]) * 0.5;
    out[1] = (buffer[offset + NODE_MIN_Y] + buffer[offset + NODE_MAX_Y]) * 0.5;
    out[2] = (buffer[offset + NODE_MIN_Z] + buffer[offset + NODE_MAX_Z]) * 0.5;
}

/**
 * Test ray-AABB intersection using node bounds directly.
 * Matches three-mesh-bvh's intersectRay() approach.
 */
export function nodeIntersectsRay(
    buffer: number[],
    offset: number,
    originX: number,
    originY: number,
    originZ: number,
    dirX: number,
    dirY: number,
    dirZ: number,
    near: number,
    far: number,
): boolean {
    let tmin: number, tmax: number, tymin: number, tymax: number, tzmin: number, tzmax: number;

    const invdirx = 1 / dirX;
    const invdiry = 1 / dirY;
    const invdirz = 1 / dirZ;

    const minx = buffer[offset + NODE_MIN_X];
    const maxx = buffer[offset + NODE_MAX_X];
    const miny = buffer[offset + NODE_MIN_Y];
    const maxy = buffer[offset + NODE_MAX_Y];
    const minz = buffer[offset + NODE_MIN_Z];
    const maxz = buffer[offset + NODE_MAX_Z];

    if (invdirx >= 0) {
        tmin = (minx - originX) * invdirx;
        tmax = (maxx - originX) * invdirx;
    } else {
        tmin = (maxx - originX) * invdirx;
        tmax = (minx - originX) * invdirx;
    }

    if (invdiry >= 0) {
        tymin = (miny - originY) * invdiry;
        tymax = (maxy - originY) * invdiry;
    } else {
        tymin = (maxy - originY) * invdiry;
        tymax = (miny - originY) * invdiry;
    }

    if (tmin > tymax || tymin > tmax) return false;

    if (tymin > tmin || Number.isNaN(tmin)) tmin = tymin;
    if (tymax < tmax || Number.isNaN(tmax)) tmax = tymax;

    if (invdirz >= 0) {
        tzmin = (minz - originZ) * invdirz;
        tzmax = (maxz - originZ) * invdirz;
    } else {
        tzmin = (maxz - originZ) * invdirz;
        tzmax = (minz - originZ) * invdirz;
    }

    if (tmin > tzmax || tzmin > tmax) return false;

    if (tzmin > tmin || Number.isNaN(tmin)) tmin = tzmin;
    if (tzmax < tmax || Number.isNaN(tmax)) tmax = tzmax;

    return tmin <= far && tmax >= near;
}

/** Test AABB-AABB intersection using node bounds directly */
export function nodeIntersectsBox(
    buffer: number[],
    offset: number,
    boxMinX: number,
    boxMinY: number,
    boxMinZ: number,
    boxMaxX: number,
    boxMaxY: number,
    boxMaxZ: number,
): boolean {
    return (
        buffer[offset + NODE_MIN_X] <= boxMaxX &&
        buffer[offset + NODE_MAX_X] >= boxMinX &&
        buffer[offset + NODE_MIN_Y] <= boxMaxY &&
        buffer[offset + NODE_MAX_Y] >= boxMinY &&
        buffer[offset + NODE_MIN_Z] <= boxMaxZ &&
        buffer[offset + NODE_MAX_Z] >= boxMinZ
    );
}

/**
 * Stride for precomputed triangle build data.
 * Layout per triangle: [centerX, halfExtentX, centerY, halfExtentY, centerZ, halfExtentZ]
 */
const BUILD_DATA_STRIDE = 6;

/** Offset to centerX in build data */
const BUILD_DATA_CENTER_X = 0;
/** Offset to halfExtentX in build data */
const BUILD_DATA_HALF_EXTENT_X = 1;
/** Offset to centerY in build data */
const BUILD_DATA_CENTER_Y = 2;
/** Offset to halfExtentY in build data */
const BUILD_DATA_HALF_EXTENT_Y = 3;
/** Offset to centerZ in build data */
const BUILD_DATA_CENTER_Z = 4;
/** Offset to halfExtentZ in build data */
const BUILD_DATA_HALF_EXTENT_Z = 5;

/**
 * Module-scoped pool for triangle build data.
 * Grows as needed, never shrinks, avoids GC from repeated allocations.
 */
const _buildDataPool: number[] = [];

/**
 * Preallocated SAH bin for reuse.
 */
type PreallocatedSahBin = {
    /** AABB bounds of triangles in this bin */
    bounds: Box3;
    /** Accumulated bounds from left */
    leftCacheBounds: Box3;
    /** Accumulated bounds from right (inclusive of this bin) */
    rightCacheBounds: Box3;
    /** Number of triangles in this bin */
    count: number;
    /** Split candidate position (center value) */
    candidate: number;
};

/**
 * Sort comparator for SAH bins by candidate position.
 */
const _sahBinSort = (a: PreallocatedSahBin, b: PreallocatedSahBin): number => a.candidate - b.candidate;

/**
 * Preallocated SAH bins for large mesh mode.
 * Fixed array of BIN_COUNT bins, reused across all SAH evaluations.
 */
const _sahBins: PreallocatedSahBin[] = /* @__PURE__ */ new Array(BIN_COUNT)
    .fill(null)
    .map(() => ({
        bounds: box3.create(),
        leftCacheBounds: box3.create(),
        rightCacheBounds: box3.create(),
        count: 0,
        candidate: 0,
    }));

/**
 * Reset a preallocated SAH bin to empty state.
 */
function resetSahBin(bin: PreallocatedSahBin): void {
    box3.empty(bin.bounds);
    box3.empty(bin.leftCacheBounds);
    box3.empty(bin.rightCacheBounds);
    bin.count = 0;
    bin.candidate = 0;
}

/**
 * Temporary Box3 for left bounds accumulation in large mesh SAH.
 */
const _leftBounds = /* @__PURE__ */ box3.create();

/**
 * Ensure the build data pool is large enough for the given triangle count.
 * Grows the pool if necessary, never shrinks.
 */
function ensureBuildDataPoolSize(out: number[], triangleCount: number): void {
    const requiredSize = triangleCount * BUILD_DATA_STRIDE;
    if (out.length < requiredSize) {
        const oldLength = out.length;
        out.length = requiredSize;
        out.fill(0, oldLength);
    }
}

/**
 * Precompute bounds (center-halfExtent format) for all triangles into the module-scoped pool.
 * This is computed once at the start of BVH construction and avoids
 * redundant vertex lookups during partitioning and SAH evaluation.
 *
 * Storage format: [centerX, halfExtentX, centerY, halfExtentY, centerZ, halfExtentZ]
 * where center = (min + max) / 2 and halfExtent = (max - min) / 2
 *
 * Note: AABB centers are used for partitioning, which is
 * mathematically valid and matches three-mesh-bvh's proven approach.
 *
 * @param data Triangle mesh data
 */
function precomputeTriangleBuildData(out: number[], data: TriangleMeshData): void {
    const count = data.triangleCount;
    ensureBuildDataPoolSize(out, count);

    const positions = data.positions;
    const buffer = data.triangleBuffer;
    const TRIANGLE_STRIDE = 8; // from triangle-mesh-data.ts

    for (let triIdx = 0; triIdx < count; triIdx++) {
        const bufferOffset = triIdx * TRIANGLE_STRIDE;
        const buildOffset = triIdx * BUILD_DATA_STRIDE;

        // get vertex indices
        const ia = buffer[bufferOffset + 0]; // OFFSET_INDEX_A
        const ib = buffer[bufferOffset + 1]; // OFFSET_INDEX_B
        const ic = buffer[bufferOffset + 2]; // OFFSET_INDEX_C

        // get vertex positions
        const ax = positions[ia * 3 + 0];
        const ay = positions[ia * 3 + 1];
        const az = positions[ia * 3 + 2];
        const bx = positions[ib * 3 + 0];
        const by = positions[ib * 3 + 1];
        const bz = positions[ib * 3 + 2];
        const cx = positions[ic * 3 + 0];
        const cy = positions[ic * 3 + 1];
        const cz = positions[ic * 3 + 2];

        // compute min bounds
        let minX = ax;
        let minY = ay;
        let minZ = az;
        if (bx < minX) minX = bx;
        if (by < minY) minY = by;
        if (bz < minZ) minZ = bz;
        if (cx < minX) minX = cx;
        if (cy < minY) minY = cy;
        if (cz < minZ) minZ = cz;

        // compute max bounds
        let maxX = ax;
        let maxY = ay;
        let maxZ = az;
        if (bx > maxX) maxX = bx;
        if (by > maxY) maxY = by;
        if (bz > maxZ) maxZ = bz;
        if (cx > maxX) maxX = cx;
        if (cy > maxY) maxY = cy;
        if (cz > maxZ) maxZ = cz;

        // compute center and half-extent
        const centerX = (minX + maxX) * 0.5;
        const centerY = (minY + maxY) * 0.5;
        const centerZ = (minZ + maxZ) * 0.5;
        const halfExtentX = (maxX - minX) * 0.5;
        const halfExtentY = (maxY - minY) * 0.5;
        const halfExtentZ = (maxZ - minZ) * 0.5;

        // store: centerX, halfExtentX, centerY, halfExtentY, centerZ, halfExtentZ
        out[buildOffset + BUILD_DATA_CENTER_X] = centerX;
        out[buildOffset + BUILD_DATA_HALF_EXTENT_X] = halfExtentX;
        out[buildOffset + BUILD_DATA_CENTER_Y] = centerY;
        out[buildOffset + BUILD_DATA_HALF_EXTENT_Y] = halfExtentY;
        out[buildOffset + BUILD_DATA_CENTER_Z] = centerZ;
        out[buildOffset + BUILD_DATA_HALF_EXTENT_Z] = halfExtentZ;
    }
}

/**
 * Swap precomputed build data entries when swapping triangles.
 * Must be called alongside swapTriangles to keep buildData in sync.
 */
function swapTriangleBuildData(buildData: number[], indexA: number, indexB: number): void {
    if (indexA === indexB) return;

    const offsetA = indexA * BUILD_DATA_STRIDE;
    const offsetB = indexB * BUILD_DATA_STRIDE;

    // Swap all 6 floats
    for (let i = 0; i < BUILD_DATA_STRIDE; i++) {
        const tmp = buildData[offsetA + i];
        buildData[offsetA + i] = buildData[offsetB + i];
        buildData[offsetB + i] = tmp;
    }
}

/** Internal node type used only during BVH construction */
type TempBvhNode = {
    bounds: Box3;
    left: TempBvhNode | null; // null = leaf
    right: TempBvhNode | null; // null = leaf
    splitAxis: number; // 0/1/2 for internal nodes
    triangleStartIndex: number; // leaf only
    triangleCount: number; // leaf only
};

export function build(data: TriangleMeshData, settings: BvhBuildSettings): TriangleMeshBVH {
    if (data.triangleCount === 0) {
        return { buffer: [] };
    }

    // Precompute triangle bounds (center-halfExtent format)
    precomputeTriangleBuildData(_buildDataPool, data);

    // Build tree structure with temp nodes
    const root = buildRecursive(data, _buildDataPool, 0, data.triangleCount, settings);

    // Count nodes and allocate buffer
    const nodeCount = countNodes(root);
    const buffer: number[] = new Array(nodeCount * NODE_STRIDE);

    // Populate flat buffer in pre-order
    populateBuffer(buffer, root, 0);

    return { buffer };
}

/** Count total nodes in temp tree */
function countNodes(node: TempBvhNode): number {
    if (node.left === null) {
        return 1; // leaf
    }
    return 1 + countNodes(node.left) + countNodes(node.right!);
}

/** Write nodes to buffer in pre-order. Returns next available offset. */
function populateBuffer(buffer: number[], node: TempBvhNode, offset: number): number {
    // Write bounds
    buffer[offset + NODE_MIN_X] = node.bounds[0][0];
    buffer[offset + NODE_MIN_Y] = node.bounds[0][1];
    buffer[offset + NODE_MIN_Z] = node.bounds[0][2];
    buffer[offset + NODE_MAX_X] = node.bounds[1][0];
    buffer[offset + NODE_MAX_Y] = node.bounds[1][1];
    buffer[offset + NODE_MAX_Z] = node.bounds[1][2];

    if (node.left === null) {
        // Leaf node
        buffer[offset + NODE_RIGHT_OR_TRI_START] = node.triangleStartIndex;
        buffer[offset + NODE_AXIS_OR_TRI_COUNT] = -(node.triangleCount + 1); // negative encodes leaf + count
        return offset + NODE_STRIDE;
    } else {
        // Internal node - left child immediately follows parent
        const leftEnd = populateBuffer(buffer, node.left, offset + NODE_STRIDE);

        // Right child follows entire left subtree
        buffer[offset + NODE_RIGHT_OR_TRI_START] = leftEnd; // right child offset
        buffer[offset + NODE_AXIS_OR_TRI_COUNT] = node.splitAxis;

        return populateBuffer(buffer, node.right!, leftEnd);
    }
}

/**
 * Internal recursive function for BVH construction.
 * Creates temp nodes, returns root of subtree.
 */
function buildRecursive(
    data: TriangleMeshData,
    buildData: number[],
    startIndex: number,
    endIndex: number,
    settings: Required<BvhBuildSettings>,
    depth = 0,
): TempBvhNode {
    const count = endIndex - startIndex;

    // Create temp node
    const node: TempBvhNode = {
        bounds: box3.create(),
        left: null,
        right: null,
        splitAxis: 0,
        triangleStartIndex: -1,
        triangleCount: 0,
    };

    // Single-pass computation of both node bounds and center bounds
    // Use module-level scratch variables to avoid allocations
    vec3.set(_centerMin, Infinity, Infinity, Infinity);
    vec3.set(_centerMax, -Infinity, -Infinity, -Infinity);

    if (count > 0) {
        // Initialize with first triangle from precomputed data
        let offset = startIndex * BUILD_DATA_STRIDE;
        const cx = buildData[offset + BUILD_DATA_CENTER_X];
        const hx = buildData[offset + BUILD_DATA_HALF_EXTENT_X];
        const cy = buildData[offset + BUILD_DATA_CENTER_Y];
        const hy = buildData[offset + BUILD_DATA_HALF_EXTENT_Y];
        const cz = buildData[offset + BUILD_DATA_CENTER_Z];
        const hz = buildData[offset + BUILD_DATA_HALF_EXTENT_Z];

        node.bounds[0][0] = cx - hx; // minX
        node.bounds[0][1] = cy - hy; // minY
        node.bounds[0][2] = cz - hz; // minZ
        node.bounds[1][0] = cx + hx; // maxX
        node.bounds[1][1] = cy + hy; // maxY
        node.bounds[1][2] = cz + hz; // maxZ

        // Center bounds initialization
        _centerMin[0] = cx;
        _centerMin[1] = cy;
        _centerMin[2] = cz;
        _centerMax[0] = cx;
        _centerMax[1] = cy;
        _centerMax[2] = cz;

        // Expand by remaining triangles (single pass for both bounds and centers)
        for (let i = startIndex + 1; i < endIndex; i++) {
            offset = i * BUILD_DATA_STRIDE;
            const cx = buildData[offset + BUILD_DATA_CENTER_X];
            const hx = buildData[offset + BUILD_DATA_HALF_EXTENT_X];
            const cy = buildData[offset + BUILD_DATA_CENTER_Y];
            const hy = buildData[offset + BUILD_DATA_HALF_EXTENT_Y];
            const cz = buildData[offset + BUILD_DATA_CENTER_Z];
            const hz = buildData[offset + BUILD_DATA_HALF_EXTENT_Z];

            // Expand node bounds
            const minX = cx - hx;
            const minY = cy - hy;
            const minZ = cz - hz;
            const maxX = cx + hx;
            const maxY = cy + hy;
            const maxZ = cz + hz;

            if (minX < node.bounds[0][0]) node.bounds[0][0] = minX;
            if (minY < node.bounds[0][1]) node.bounds[0][1] = minY;
            if (minZ < node.bounds[0][2]) node.bounds[0][2] = minZ;
            if (maxX > node.bounds[1][0]) node.bounds[1][0] = maxX;
            if (maxY > node.bounds[1][1]) node.bounds[1][1] = maxY;
            if (maxZ > node.bounds[1][2]) node.bounds[1][2] = maxZ;

            // Expand center bounds
            if (cx < _centerMin[0]) _centerMin[0] = cx;
            if (cy < _centerMin[1]) _centerMin[1] = cy;
            if (cz < _centerMin[2]) _centerMin[2] = cz;
            if (cx > _centerMax[0]) _centerMax[0] = cx;
            if (cy > _centerMax[1]) _centerMax[1] = cy;
            if (cz > _centerMax[2]) _centerMax[2] = cz;
        }
    }

    // base case: leaf node (use configurable threshold)
    if (count <= settings.maxLeafTris) {
        node.triangleStartIndex = startIndex;
        node.triangleCount = count;
        return node;
    }

    // get optimal split (strategy picks its own axis and position)
    const split = getOptimalSplit(buildData, startIndex, endIndex, node.bounds, _centerMin, _centerMax, settings.strategy);

    // handle degenerate case (no valid split found)
    if (split === null) {
        // can't split, make this a leaf
        node.triangleStartIndex = startIndex;
        node.triangleCount = count;
        return node;
    }

    // partition triangles using the chosen axis and position
    const mid = partitionTriangles(data, buildData, startIndex, endIndex, split.axis, split.pos);

    // handle degenerate split (all centers on same side)
    if (mid === startIndex || mid === endIndex) {
        // can't split further, make this a leaf
        node.triangleStartIndex = startIndex;
        node.triangleCount = count;
        return node;
    }

    // recursively build children
    node.left = buildRecursive(data, buildData, startIndex, mid, settings, depth + 1);
    node.right = buildRecursive(data, buildData, mid, endIndex, settings, depth + 1);
    node.splitAxis = split.axis;

    return node;
}

const _extent = /* @__PURE__ */ vec3.create();
const _centerSize = /* @__PURE__ */ vec3.create();

/**
 * Scratch variables for center bounds computation.
 * Reused across all buildRecursive calls to avoid allocations.
 */
const _centerMin = /* @__PURE__ */ vec3.create();
const _centerMax = /* @__PURE__ */ vec3.create();

/**
 * Compute optimal split axis and position for given strategy.
 * Follows three-mesh-bvh's getOptimalSplit() pattern.
 *
 * Each strategy computes its own axis:
 * - CENTER: longest center extent
 * - AVERAGE: longest node bounds extent
 * - SAH: evaluates all axes for cost
 *
 * @param buildData Precomputed triangle bounds (center-halfExtent format)
 * @param startIndex First triangle in range
 * @param endIndex One past last triangle
 * @param nodeBounds Node AABB bounds (precomputed)
 * @param centerMin Min of triangle AABB centers (precomputed)
 * @param centerMax Max of triangle AABB centers (precomputed)
 * @param strategy Split strategy constant
 * @returns Split axis and position, or null if no valid split
 */
function getOptimalSplit(
    buildData: number[],
    startIndex: number,
    endIndex: number,
    nodeBounds: Box3,
    centerMin: Vec3,
    centerMax: Vec3,
    strategy: BvhSplitStrategy,
): { axis: number; pos: number } | null {
    let axis = -1;
    let pos = 0;

    if (strategy === BvhSplitStrategy.CENTER) {
        // CENTER: Use longest center extent
        vec3.subtract(_extent, centerMax, centerMin);

        // Prevent degenerate case where all centers are at same point
        const MIN_SIZE = 1e-5;

        if (_extent[0] < MIN_SIZE && _extent[1] < MIN_SIZE && _extent[2] < MIN_SIZE) {
            // All centers at same point, no valid split
            axis = -1;
        } else {
            axis = _extent[0] >= _extent[1] && _extent[0] >= _extent[2] ? 0 : _extent[1] >= _extent[2] ? 1 : 2;

            if (axis !== -1) {
                pos = (centerMin[axis] + centerMax[axis]) / 2;
            }
        }
    } else if (strategy === BvhSplitStrategy.AVERAGE) {
        // AVERAGE: Use longest node bounds extent (NOT center extent)
        // This matches three-mesh-bvh which uses nodeBoundingData for axis selection
        // Use precomputed node bounds rather than recomputing (O(1) vs O(n))
        vec3.subtract(_extent, nodeBounds[1], nodeBounds[0]);
        axis = _extent[0] >= _extent[1] && _extent[0] >= _extent[2] ? 0 : _extent[1] >= _extent[2] ? 1 : 2;

        if (axis !== -1) {
            pos = getAverageCenter(buildData, startIndex, endIndex, axis);
        }
    } else if (strategy === BvhSplitStrategy.SAH) {
        // SAH: Surface Area Heuristic (inline implementation matching three-mesh-bvh)
        const count = endIndex - startIndex;

        // Compute center extent for each axis
        vec3.subtract(_centerSize, centerMax, centerMin);

        // Prevent division by zero
        const MIN_SIZE = 1e-5;
        if (_centerSize[0] < MIN_SIZE && _centerSize[1] < MIN_SIZE && _centerSize[2] < MIN_SIZE) {
            // All centers at same point, axis remains -1
        } else {
            // Compute root surface area for probability calculation
            const rootSurfaceArea = box3.surfaceArea(nodeBounds);
            let bestCost = TRIANGLE_INTERSECT_COST * count; // Cost of making this a leaf
            let bestAxis = -1;
            let bestPos = 0;

            // Try each axis
            for (let a = 0; a < 3; a++) {
                const axisLength = _centerSize[a];
                if (axisLength < MIN_SIZE) {
                    continue; // Skip degenerate axis
                }

                const axisLeft = centerMin[a];
                const binWidth = axisLength / BIN_COUNT;

                // Small mesh mode: use exact triangle positions as candidates
                if (count < BIN_COUNT / 4) {
                    // Use preallocated bins (reuse first 'count' bins)
                    for (let i = 0; i < count; i++) {
                        const triIdx = startIndex + i;
                        const offset = triIdx * BUILD_DATA_STRIDE;

                        const cx = buildData[offset + BUILD_DATA_CENTER_X];
                        const hx = buildData[offset + BUILD_DATA_HALF_EXTENT_X];
                        const cy = buildData[offset + BUILD_DATA_CENTER_Y];
                        const hy = buildData[offset + BUILD_DATA_HALF_EXTENT_Y];
                        const cz = buildData[offset + BUILD_DATA_CENTER_Z];
                        const hz = buildData[offset + BUILD_DATA_HALF_EXTENT_Z];

                        const center = buildData[offset + a * 2]; // centerX/Y/Z based on axis

                        const bin = _sahBins[i];
                        resetSahBin(bin);
                        bin.candidate = center;

                        // initialize bounds with this triangle's AABB
                        bin.bounds[0][0] = cx - hx;
                        bin.bounds[0][1] = cy - hy;
                        bin.bounds[0][2] = cz - hz;
                        bin.bounds[1][0] = cx + hx;
                        bin.bounds[1][1] = cy + hy;
                        bin.bounds[1][2] = cz + hz;
                    }

                    // Sort bins by candidate position (sort in-place, only first 'count' bins)
                    _sahBins.sort(_sahBinSort);

                    // Remove duplicate candidates by tracking split count
                    // Instead of splice, we compact valid bins to the front
                    let splitCount = count;
                    let writeIdx = 1;
                    for (let readIdx = 1; readIdx < count; readIdx++) {
                        if (_sahBins[readIdx].candidate !== _sahBins[writeIdx - 1].candidate) {
                            if (writeIdx !== readIdx) {
                                // Swap bins to compact
                                const temp = _sahBins[writeIdx];
                                _sahBins[writeIdx] = _sahBins[readIdx];
                                _sahBins[readIdx] = temp;
                            }
                            writeIdx++;
                        } else {
                            splitCount--;
                        }
                    }

                    // For each triangle, determine left vs right for each split candidate
                    for (let i = startIndex; i < endIndex; i++) {
                        const offset = i * BUILD_DATA_STRIDE;

                        const cx = buildData[offset + BUILD_DATA_CENTER_X];
                        const hx = buildData[offset + BUILD_DATA_HALF_EXTENT_X];
                        const cy = buildData[offset + BUILD_DATA_CENTER_Y];
                        const hy = buildData[offset + BUILD_DATA_HALF_EXTENT_Y];
                        const cz = buildData[offset + BUILD_DATA_CENTER_Z];
                        const hz = buildData[offset + BUILD_DATA_HALF_EXTENT_Z];

                        const minX = cx - hx;
                        const minY = cy - hy;
                        const minZ = cz - hz;
                        const maxX = cx + hx;
                        const maxY = cy + hy;
                        const maxZ = cz + hz;

                        const center = buildData[offset + a * 2];

                        for (let bi = 0; bi < splitCount; bi++) {
                            const bin = _sahBins[bi];
                            if (center >= bin.candidate) {
                                // Right side - expand right cache bounds
                                if (minX < bin.rightCacheBounds[0][0]) bin.rightCacheBounds[0][0] = minX;
                                if (minY < bin.rightCacheBounds[0][1]) bin.rightCacheBounds[0][1] = minY;
                                if (minZ < bin.rightCacheBounds[0][2]) bin.rightCacheBounds[0][2] = minZ;
                                if (maxX > bin.rightCacheBounds[1][0]) bin.rightCacheBounds[1][0] = maxX;
                                if (maxY > bin.rightCacheBounds[1][1]) bin.rightCacheBounds[1][1] = maxY;
                                if (maxZ > bin.rightCacheBounds[1][2]) bin.rightCacheBounds[1][2] = maxZ;
                            } else {
                                // Left side - expand left cache bounds and increment count
                                if (minX < bin.leftCacheBounds[0][0]) bin.leftCacheBounds[0][0] = minX;
                                if (minY < bin.leftCacheBounds[0][1]) bin.leftCacheBounds[0][1] = minY;
                                if (minZ < bin.leftCacheBounds[0][2]) bin.leftCacheBounds[0][2] = minZ;
                                if (maxX > bin.leftCacheBounds[1][0]) bin.leftCacheBounds[1][0] = maxX;
                                if (maxY > bin.leftCacheBounds[1][1]) bin.leftCacheBounds[1][1] = maxY;
                                if (maxZ > bin.leftCacheBounds[1][2]) bin.leftCacheBounds[1][2] = maxZ;
                                bin.count++;
                            }
                        }
                    }

                    // Evaluate SAH cost for each split candidate
                    for (let bi = 0; bi < splitCount; bi++) {
                        const bin = _sahBins[bi];
                        const leftCount = bin.count;
                        const rightCount = count - leftCount;

                        // Calculate probabilities (only if counts are non-zero)
                        let leftProb = 0;
                        if (leftCount !== 0) {
                            leftProb = box3.surfaceArea(bin.leftCacheBounds) / rootSurfaceArea;
                        }

                        let rightProb = 0;
                        if (rightCount !== 0) {
                            rightProb = box3.surfaceArea(bin.rightCacheBounds) / rootSurfaceArea;
                        }

                        const cost = TRAVERSAL_COST + TRIANGLE_INTERSECT_COST * (leftProb * leftCount + rightProb * rightCount);

                        if (cost < bestCost) {
                            bestCost = cost;
                            bestAxis = a;
                            bestPos = bin.candidate;
                        }
                    }
                } else {
                    // Large mesh mode: use preallocated fixed 32 bins
                    // Reset and initialize bins for this axis
                    for (let b = 0; b < BIN_COUNT; b++) {
                        const bin = _sahBins[b];
                        resetSahBin(bin);
                        bin.candidate = axisLeft + binWidth * (b + 1);
                    }

                    // Bin all triangles
                    for (let i = startIndex; i < endIndex; i++) {
                        const offset = i * BUILD_DATA_STRIDE;

                        const cx = buildData[offset + BUILD_DATA_CENTER_X];
                        const hx = buildData[offset + BUILD_DATA_HALF_EXTENT_X];
                        const cy = buildData[offset + BUILD_DATA_CENTER_Y];
                        const hy = buildData[offset + BUILD_DATA_HALF_EXTENT_Y];
                        const cz = buildData[offset + BUILD_DATA_CENTER_Z];
                        const hz = buildData[offset + BUILD_DATA_HALF_EXTENT_Z];

                        const minX = cx - hx;
                        const minY = cy - hy;
                        const minZ = cz - hz;
                        const maxX = cx + hx;
                        const maxY = cy + hy;
                        const maxZ = cz + hz;

                        const center = buildData[offset + a * 2];

                        // Determine bin index (relativePos >= 0 always since center >= axisLeft)
                        const relativePos = center - axisLeft;
                        let binIndex = Math.floor(relativePos / binWidth);
                        if (binIndex >= BIN_COUNT) binIndex = BIN_COUNT - 1;

                        const bin = _sahBins[binIndex];
                        bin.count++;

                        // Expand bin bounds to include this triangle's AABB
                        if (minX < bin.bounds[0][0]) bin.bounds[0][0] = minX;
                        if (minY < bin.bounds[0][1]) bin.bounds[0][1] = minY;
                        if (minZ < bin.bounds[0][2]) bin.bounds[0][2] = minZ;
                        if (maxX > bin.bounds[1][0]) bin.bounds[1][0] = maxX;
                        if (maxY > bin.bounds[1][1]) bin.bounds[1][1] = maxY;
                        if (maxZ > bin.bounds[1][2]) bin.bounds[1][2] = maxZ;
                    }

                    // Cache right bounds from right-to-left
                    const lastBin = _sahBins[BIN_COUNT - 1];
                    box3.copy(lastBin.rightCacheBounds, lastBin.bounds);

                    for (let b = BIN_COUNT - 2; b >= 0; b--) {
                        const bin = _sahBins[b];
                        const nextBin = _sahBins[b + 1];
                        box3.union(bin.rightCacheBounds, bin.bounds, nextBin.rightCacheBounds);
                    }

                    // Accumulate left bounds and evaluate cost
                    box3.empty(_leftBounds);

                    let leftCount = 0;
                    for (let b = 0; b < BIN_COUNT - 1; b++) {
                        const bin = _sahBins[b];
                        const binCount = bin.count;

                        // Union current bin into left bounds (if not empty)
                        if (binCount !== 0) {
                            if (leftCount === 0) {
                                box3.copy(_leftBounds, bin.bounds);
                            } else {
                                box3.union(_leftBounds, _leftBounds, bin.bounds);
                            }
                        }

                        leftCount += binCount;

                        // Evaluate cost at this split position
                        const nextBin = _sahBins[b + 1];
                        const rightCount = count - leftCount;

                        // Calculate probabilities (only if counts are non-zero)
                        let leftProb = 0;
                        if (leftCount !== 0) {
                            leftProb = box3.surfaceArea(_leftBounds) / rootSurfaceArea;
                        }

                        let rightProb = 0;
                        if (rightCount !== 0) {
                            rightProb = box3.surfaceArea(nextBin.rightCacheBounds) / rootSurfaceArea;
                        }

                        const cost = TRAVERSAL_COST + TRIANGLE_INTERSECT_COST * (leftProb * leftCount + rightProb * rightCount);

                        if (cost < bestCost) {
                            bestCost = cost;
                            bestAxis = a;
                            bestPos = bin.candidate;
                        }
                    }
                }
            }

            if (bestAxis !== -1) {
                axis = bestAxis;
                pos = bestPos;
            }
        }
    } else {
        // Invalid strategy - throw error
        throw new Error(`Invalid BVH split strategy: ${strategy}`);
    }

    // Return null if no valid axis found
    if (axis === -1) {
        return null;
    }

    return { axis, pos };
}

/**
 * Compute average AABB center position along axis.
 * Used by AVERAGE strategy.
 *
 * Note: This is MEAN, not MEDIAN.
 *
 * @param buildData Precomputed triangle bounds (center-halfExtent format)
 * @param startIndex First triangle in range
 * @param endIndex One past last triangle
 * @param axis Axis to average (0=x, 1=y, 2=z)
 * @returns Average center position
 */
function getAverageCenter(buildData: number[], startIndex: number, endIndex: number, axis: number): number {
    let sum = 0;
    const count = endIndex - startIndex;

    // Center is at offset axis * 2 in buildData (0=centerX, 2=centerY, 4=centerZ)
    const centerOffset = axis * 2;
    for (let i = startIndex; i < endIndex; i++) {
        sum += buildData[i * BUILD_DATA_STRIDE + centerOffset];
    }

    return sum / count;
}

/**
 * Partition triangles in-place using Hoare partition scheme.
 * Elements with center < splitPos go left, >= splitPos go right.
 *
 * Used by CENTER and AVERAGE strategies.
 *
 * IMPORTANT: Reorders the triangle buffer in-place. Triangles will be spatially sorted
 * after BVH construction, allowing leaf nodes to store contiguous ranges.
 *
 * @param data triangle mesh data (triangle buffer modified in-place)
 * @param buildData precomputed build data (center-halfExtent format)
 * @param startIndex First triangle index
 * @param endIndex One past last triangle index
 * @param axis Axis to partition on (0=x, 1=y, 2=z)
 * @param splitPos Split position
 * @returns Index of first triangle in right partition
 */
function partitionTriangles(
    data: TriangleMeshData,
    buildData: number[],
    startIndex: number,
    endIndex: number,
    axis: number,
    splitPos: number,
): number {
    let left = startIndex;
    let right = endIndex - 1;

    // Center is at offset axis * 2 in buildData (0=centerX, 2=centerY, 4=centerZ)
    const centerOffset = axis * 2;

    while (left <= right) {
        const leftCenter = buildData[left * BUILD_DATA_STRIDE + centerOffset];
        if (leftCenter < splitPos) {
            left++;
        } else {
            const rightCenter = buildData[right * BUILD_DATA_STRIDE + centerOffset];
            if (rightCenter >= splitPos) {
                right--;
            } else {
                // Swap triangles in buffer and buildData
                swapTriangles(data, left, right);
                swapTriangleBuildData(buildData, left, right);
                left++;
                right--;
            }
        }
    }

    return left;
}

export type TriangleMeshBvhStats = {
    nodeCount: number;
    leafCount: number;
    minDepth: number;
    maxDepth: number;
    avgDepth: number;
};

/** compute depth statistics of the BVH tree, useful for performance analysis */
export function stats(bvh: TriangleMeshBVH): TriangleMeshBvhStats {
    if (bvh.buffer.length === 0) {
        return {
            nodeCount: 0,
            leafCount: 0,
            minDepth: 0,
            maxDepth: 0,
            avgDepth: 0,
        };
    }

    const buffer = bvh.buffer;
    let leafCount = 0;
    let depthSum = 0;
    let minDepth = Infinity;
    let maxDepth = 0;
    let nodeCount = 0;

    const stack: [number, number][] = [[0, 0]]; // [offset, depth]

    while (stack.length > 0) {
        const [offset, depth] = stack.pop()!;
        nodeCount++;

        if (nodeIsLeaf(buffer, offset)) {
            leafCount++;
            minDepth = Math.min(minDepth, depth);
            maxDepth = Math.max(maxDepth, depth);
            depthSum += depth;
        } else {
            stack.push([nodeRight(buffer, offset), depth + 1]);
            stack.push([nodeLeft(offset), depth + 1]);
        }
    }

    return {
        nodeCount,
        leafCount,
        minDepth,
        maxDepth,
        avgDepth: leafCount > 0 ? depthSum / leafCount : 0,
    };
}
