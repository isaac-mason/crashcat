import type { Vec3 } from 'mathcat';
import type { Box3 } from 'mathcat/shapes';
/**
 * common BVH node utilities for binary tree structures.
 * shared between triangle mesh BVH and static compound BVH.
 *
 * node layout (8 floats per node):
 * [minX, minY, minZ, maxX, maxY, maxZ, rightOrStart, axisOrCount]
 *
 * internal nodes:
 * - rightOrStart: offset to right child
 * - axisOrCount: split axis (0-2)
 *
 * leaf nodes:
 * - rightOrStart: start index of primitives
 * - axisOrCount: -(count + 1) to encode leaf + count
 */
/** number of elements per node in the flat buffer */
export declare const NODE_STRIDE = 8;
export declare const NODE_MIN_X = 0;
export declare const NODE_MIN_Y = 1;
export declare const NODE_MIN_Z = 2;
export declare const NODE_MAX_X = 3;
export declare const NODE_MAX_Y = 4;
export declare const NODE_MAX_Z = 5;
/** internal: right child offset, leaf: data start index */
export declare const NODE_RIGHT_OR_START = 6;
/** internal: split axis (0-2), leaf: negative encoded count */
export declare const NODE_AXIS_OR_COUNT = 7;
/** check if node at offset is a leaf */
export declare function nodeIsLeaf(buffer: number[], offset: number): boolean;
/** get left child offset. left child is always contiguous (offset + NODE_STRIDE). */
export declare function nodeLeft(offset: number): number;
/** get right child offset (internal only) */
export declare function nodeRight(buffer: number[], offset: number): number;
/** get split axis (internal only): 0=x, 1=y, 2=z */
export declare function nodeSplitAxis(buffer: number[], offset: number): number;
/** copy bounds into existing Box3 */
export declare function nodeGetBounds(out: Box3, buffer: number[], offset: number): void;
/** get center of node bounds */
export declare function nodeGetCenter(out: Vec3, buffer: number[], offset: number): void;
/** test ray-AABB intersection using node bounds directly */
export declare function nodeIntersectsRay(buffer: number[], offset: number, originX: number, originY: number, originZ: number, dirX: number, dirY: number, dirZ: number, near: number, far: number): boolean;
/** test AABB-AABB intersection using node bounds directly */
export declare function nodeIntersectsBox(buffer: number[], offset: number, boxMinX: number, boxMinY: number, boxMinZ: number, boxMaxX: number, boxMaxY: number, boxMaxZ: number): boolean;
