import { type Box3, box3, type Vec3, vec3 } from 'mathcat';
import type { CompoundShapeChild } from '../compound';
import {
    NODE_AXIS_OR_COUNT,
    NODE_MAX_X,
    NODE_MAX_Y,
    NODE_MAX_Z,
    NODE_MIN_X,
    NODE_MIN_Y,
    NODE_MIN_Z,
    NODE_RIGHT_OR_START,
    NODE_STRIDE,
    nodeIsLeaf,
    nodeLeft as nodeLeftCommon,
    nodeRight,
} from './bvh';

/** settings for BVH construction */
export type BvhBuildSettings = {
    /** maximum children per leaf node */
    maxLeafChildren: number;
};

/** static compound BVH stored as a flat number array */
export type StaticCompoundBVH = {
    /** packed node data */
    buffer: number[];
};

/** get child start index (leaf only) */
export function nodeChildStart(buffer: number[], offset: number): number {
    return buffer[offset + NODE_RIGHT_OR_START];
}

/** get child count (leaf only). decodes from negative flag. */
export function nodeChildCount(buffer: number[], offset: number): number {
    return -(buffer[offset + NODE_AXIS_OR_COUNT] + 1);
}

/**
 * stride for precomputed child build data.
 * layout per child: [centerX, halfExtentX, centerY, halfExtentY, centerZ, halfExtentZ]
 */
const BUILD_DATA_STRIDE = 6;

/** offset to centerX in build data */
const BUILD_DATA_CENTER_X = 0;

/** offset to halfExtentX in build data */
const BUILD_DATA_HALF_EXTENT_X = 1;

/** offset to centerY in build data */
const BUILD_DATA_CENTER_Y = 2;

/** offset to halfExtentY in build data */
const BUILD_DATA_HALF_EXTENT_Y = 3;

/** offset to centerZ in build data */
const BUILD_DATA_CENTER_Z = 4;

/** offset to halfExtentZ in build data */
const BUILD_DATA_HALF_EXTENT_Z = 5;

const _extent = /* @__PURE__ */ vec3.create();
const _centerMin = /* @__PURE__ */ vec3.create();
const _centerMax = /* @__PURE__ */ vec3.create();

const _transformedCorner = /* @__PURE__ */ vec3.create();

/**
 * compute child AABB in compound local space (transformed by position/rotation).
 * expands child shape AABB by position and rotation.
 */
function computeChildBounds(out: Box3, child: CompoundShapeChild): void {
    const childAABB = child.shape.aabb;

    // start with empty bounds
    out[0][0] = Infinity;
    out[0][1] = Infinity;
    out[0][2] = Infinity;
    out[1][0] = -Infinity;
    out[1][1] = -Infinity;
    out[1][2] = -Infinity;

    // transform all 8 corners of the child AABB
    for (let x = 0; x < 2; x++) {
        for (let y = 0; y < 2; y++) {
            for (let z = 0; z < 2; z++) {
                _transformedCorner[0] = childAABB[x][0];
                _transformedCorner[1] = childAABB[y][1];
                _transformedCorner[2] = childAABB[z][2];

                // rotate and translate corner
                vec3.transformQuat(_transformedCorner, _transformedCorner, child.quaternion);
                vec3.add(_transformedCorner, _transformedCorner, child.position);

                // expand bounds
                box3.expandByPoint(out, out, _transformedCorner);
            }
        }
    }
}

/**
 * precompute child bounds in center-halfExtent format.
 * also reorders children array to match indices array order.
 */
function precomputeChildBuildData(
    buildData: number[],
    indices: number[],
    children: CompoundShapeChild[],
): void {
    const count = indices.length;
    const childBounds = box3.create();

    for (let i = 0; i < count; i++) {
        const childIndex = indices[i];
        const child = children[childIndex];
        const buildOffset = i * BUILD_DATA_STRIDE;

        computeChildBounds(childBounds, child);

        const minX = childBounds[0][0];
        const minY = childBounds[0][1];
        const minZ = childBounds[0][2];
        const maxX = childBounds[1][0];
        const maxY = childBounds[1][1];
        const maxZ = childBounds[1][2];

        const centerX = (minX + maxX) * 0.5;
        const centerY = (minY + maxY) * 0.5;
        const centerZ = (minZ + maxZ) * 0.5;
        const halfExtentX = (maxX - minX) * 0.5;
        const halfExtentY = (maxY - minY) * 0.5;
        const halfExtentZ = (maxZ - minZ) * 0.5;

        buildData[buildOffset + BUILD_DATA_CENTER_X] = centerX;
        buildData[buildOffset + BUILD_DATA_HALF_EXTENT_X] = halfExtentX;
        buildData[buildOffset + BUILD_DATA_CENTER_Y] = centerY;
        buildData[buildOffset + BUILD_DATA_HALF_EXTENT_Y] = halfExtentY;
        buildData[buildOffset + BUILD_DATA_CENTER_Z] = centerZ;
        buildData[buildOffset + BUILD_DATA_HALF_EXTENT_Z] = halfExtentZ;
    }
}

/** swap two indices in the index array and their corresponding build data */
function swapIndices(indices: number[], buildData: number[], indexA: number, indexB: number): void {
    if (indexA === indexB) return;

    // swap indices
    const tmp = indices[indexA];
    indices[indexA] = indices[indexB];
    indices[indexB] = tmp;

    // swap build data
    const offsetA = indexA * BUILD_DATA_STRIDE;
    const offsetB = indexB * BUILD_DATA_STRIDE;

    for (let i = 0; i < BUILD_DATA_STRIDE; i++) {
        const tmpData = buildData[offsetA + i];
        buildData[offsetA + i] = buildData[offsetB + i];
        buildData[offsetB + i] = tmpData;
    }
}

/** internal node type used only during BVH construction */
type TempBvhNode = {
    bounds: Box3;
    left: TempBvhNode | null;
    right: TempBvhNode | null;
    splitAxis: number;
    childStartIndex: number;
    childCount: number;
};

/**
 * build a BVH over compound shape children.
 * reorders the children array in place for spatial locality.
 */
export function build(children: CompoundShapeChild[], settings: BvhBuildSettings): StaticCompoundBVH {
    const count = children.length;

    if (count === 0) {
        return { buffer: [] };
    }

    // create index array
    const indices: number[] = new Array(count);
    for (let i = 0; i < count; i++) {
        indices[i] = i;
    }

    // precompute bounds
    const buildData: number[] = new Array(count * BUILD_DATA_STRIDE);
    precomputeChildBuildData(buildData, indices, children);

    // build tree structure with temp nodes
    const root = buildRecursive(indices, buildData, 0, count, settings);

    // reorder children array based on final indices order
    const reorderedChildren: CompoundShapeChild[] = new Array(count);
    for (let i = 0; i < count; i++) {
        reorderedChildren[i] = children[indices[i]];
    }
    for (let i = 0; i < count; i++) {
        children[i] = reorderedChildren[i];
    }

    // count nodes and allocate buffer
    const nodeCount = countNodes(root);
    const buffer: number[] = new Array(nodeCount * NODE_STRIDE);

    // populate flat buffer in pre-order
    populateBuffer(buffer, root, 0);

    return { buffer };
}

/** count total nodes in temp tree */
function countNodes(node: TempBvhNode): number {
    if (node.left === null) {
        return 1;
    }
    return 1 + countNodes(node.left) + countNodes(node.right!);
}

/** write nodes to buffer in pre-order. returns next available offset. */
function populateBuffer(buffer: number[], node: TempBvhNode, offset: number): number {
    // write bounds
    buffer[offset + NODE_MIN_X] = node.bounds[0][0];
    buffer[offset + NODE_MIN_Y] = node.bounds[0][1];
    buffer[offset + NODE_MIN_Z] = node.bounds[0][2];
    buffer[offset + NODE_MAX_X] = node.bounds[1][0];
    buffer[offset + NODE_MAX_Y] = node.bounds[1][1];
    buffer[offset + NODE_MAX_Z] = node.bounds[1][2];

    if (node.left === null) {
        // leaf node
        buffer[offset + NODE_RIGHT_OR_START] = node.childStartIndex;
        buffer[offset + NODE_AXIS_OR_COUNT] = -(node.childCount + 1);
        return offset + NODE_STRIDE;
    }

    // internal node - left child immediately follows parent
    const leftEnd = populateBuffer(buffer, node.left, offset + NODE_STRIDE);

    // right child follows entire left subtree
    buffer[offset + NODE_RIGHT_OR_START] = leftEnd;
    buffer[offset + NODE_AXIS_OR_COUNT] = node.splitAxis;

    return populateBuffer(buffer, node.right!, leftEnd);
}

/**
 * internal recursive function for BVH construction.
 */
function buildRecursive(
    indices: number[],
    buildData: number[],
    startIndex: number,
    endIndex: number,
    settings: BvhBuildSettings,
): TempBvhNode {
    const count = endIndex - startIndex;

    const node: TempBvhNode = {
        bounds: box3.create(),
        left: null,
        right: null,
        splitAxis: 0,
        childStartIndex: -1,
        childCount: 0,
    };

    // compute bounds and center bounds
    vec3.set(_centerMin, Infinity, Infinity, Infinity);
    vec3.set(_centerMax, -Infinity, -Infinity, -Infinity);

    if (count > 0) {
        let offset = startIndex * BUILD_DATA_STRIDE;
        const cx = buildData[offset + BUILD_DATA_CENTER_X];
        const hx = buildData[offset + BUILD_DATA_HALF_EXTENT_X];
        const cy = buildData[offset + BUILD_DATA_CENTER_Y];
        const hy = buildData[offset + BUILD_DATA_HALF_EXTENT_Y];
        const cz = buildData[offset + BUILD_DATA_CENTER_Z];
        const hz = buildData[offset + BUILD_DATA_HALF_EXTENT_Z];

        node.bounds[0][0] = cx - hx;
        node.bounds[0][1] = cy - hy;
        node.bounds[0][2] = cz - hz;
        node.bounds[1][0] = cx + hx;
        node.bounds[1][1] = cy + hy;
        node.bounds[1][2] = cz + hz;

        _centerMin[0] = cx;
        _centerMin[1] = cy;
        _centerMin[2] = cz;
        _centerMax[0] = cx;
        _centerMax[1] = cy;
        _centerMax[2] = cz;

        for (let i = startIndex + 1; i < endIndex; i++) {
            offset = i * BUILD_DATA_STRIDE;
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

            if (minX < node.bounds[0][0]) node.bounds[0][0] = minX;
            if (minY < node.bounds[0][1]) node.bounds[0][1] = minY;
            if (minZ < node.bounds[0][2]) node.bounds[0][2] = minZ;
            if (maxX > node.bounds[1][0]) node.bounds[1][0] = maxX;
            if (maxY > node.bounds[1][1]) node.bounds[1][1] = maxY;
            if (maxZ > node.bounds[1][2]) node.bounds[1][2] = maxZ;

            if (cx < _centerMin[0]) _centerMin[0] = cx;
            if (cy < _centerMin[1]) _centerMin[1] = cy;
            if (cz < _centerMin[2]) _centerMin[2] = cz;
            if (cx > _centerMax[0]) _centerMax[0] = cx;
            if (cy > _centerMax[1]) _centerMax[1] = cy;
            if (cz > _centerMax[2]) _centerMax[2] = cz;
        }
    }

    // base case: leaf node
    if (count <= settings.maxLeafChildren) {
        node.childStartIndex = startIndex;
        node.childCount = count;
        return node;
    }

    // get optimal split
    const split = getOptimalSplit(_centerMin, _centerMax);

    // handle degenerate case
    if (split === null) {
        node.childStartIndex = startIndex;
        node.childCount = count;
        return node;
    }

    // partition
    const mid = partitionChildren(indices, buildData, startIndex, endIndex, split.axis, split.pos);

    // handle degenerate split
    if (mid === startIndex || mid === endIndex) {
        node.childStartIndex = startIndex;
        node.childCount = count;
        return node;
    }

    // recursively build children
    node.left = buildRecursive(indices, buildData, startIndex, mid, settings);
    node.right = buildRecursive(indices, buildData, mid, endIndex, settings);
    node.splitAxis = split.axis;

    return node;
}

/**
 * compute optimal split axis and position using spatial median.
 * splits at the midpoint of the centroid bounding box along the longest axis.
 */
function getOptimalSplit(centerMin: Vec3, centerMax: Vec3): { axis: number; pos: number } | null {
    vec3.subtract(_extent, centerMax, centerMin);

    const MIN_SIZE = 1e-5;

    if (_extent[0] < MIN_SIZE && _extent[1] < MIN_SIZE && _extent[2] < MIN_SIZE) {
        return null;
    }

    // choose longest axis
    const axis = _extent[0] >= _extent[1] && _extent[0] >= _extent[2] ? 0 : _extent[1] >= _extent[2] ? 1 : 2;
    
    // split at midpoint
    const pos = (centerMin[axis] + centerMax[axis]) / 2;

    return { axis, pos };
}

/** partition children in-place using Hoare partition scheme */
function partitionChildren(
    indices: number[],
    buildData: number[],
    startIndex: number,
    endIndex: number,
    axis: number,
    splitPos: number,
): number {
    let left = startIndex;
    let right = endIndex - 1;

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
                swapIndices(indices, buildData, left, right);
                left++;
                right--;
            }
        }
    }

    return left;
}

export type StaticCompoundBvhStats = {
    nodeCount: number;
    leafCount: number;
    minDepth: number;
    maxDepth: number;
    avgDepth: number;
    totalChildren: number;
    /** alias for nodeCount */
    totalNodes: number;
    /** alias for leafCount */
    leafNodes: number;
};

/** compute depth statistics of the BVH tree */
export function stats(bvh: StaticCompoundBVH): StaticCompoundBvhStats {
    if (bvh.buffer.length === 0) {
        return {
            nodeCount: 0,
            leafCount: 0,
            minDepth: 0,
            maxDepth: 0,
            avgDepth: 0,
            totalChildren: 0,
            totalNodes: 0,
            leafNodes: 0,
        };
    }

    const buffer = bvh.buffer;
    let leafCount = 0;
    let depthSum = 0;
    let minDepth = Infinity;
    let maxDepth = 0;
    let nodeCount = 0;
    let totalChildren = 0;

    const stack: [number, number][] = [[0, 0]];

    while (stack.length > 0) {
        const [offset, depth] = stack.pop()!;
        nodeCount++;

        if (nodeIsLeaf(buffer, offset)) {
            leafCount++;
            minDepth = Math.min(minDepth, depth);
            maxDepth = Math.max(maxDepth, depth);
            depthSum += depth;
            totalChildren += nodeChildCount(buffer, offset);
        } else {
            stack.push([nodeRight(buffer, offset), depth + 1]);
            stack.push([nodeLeftCommon(offset), depth + 1]);
        }
    }

    return {
        nodeCount,
        leafCount,
        minDepth,
        maxDepth,
        avgDepth: leafCount > 0 ? depthSum / leafCount : 0,
        totalChildren,
        totalNodes: nodeCount,
        leafNodes: leafCount,
    };
}
