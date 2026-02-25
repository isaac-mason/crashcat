import type { Box3, Vec3 } from 'mathcat';

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
export const NODE_STRIDE = 8;

// offsets within a node
export const NODE_MIN_X = 0;
export const NODE_MIN_Y = 1;
export const NODE_MIN_Z = 2;
export const NODE_MAX_X = 3;
export const NODE_MAX_Y = 4;
export const NODE_MAX_Z = 5;

/** internal: right child offset, leaf: data start index */
export const NODE_RIGHT_OR_START = 6;

/** internal: split axis (0-2), leaf: negative encoded count */
export const NODE_AXIS_OR_COUNT = 7;

/** check if node at offset is a leaf */
export function nodeIsLeaf(buffer: number[], offset: number): boolean {
    return buffer[offset + NODE_AXIS_OR_COUNT] < 0;
}

/** get left child offset. left child is always contiguous (offset + NODE_STRIDE). */
export function nodeLeft(offset: number): number {
    return offset + NODE_STRIDE;
}

/** get right child offset (internal only) */
export function nodeRight(buffer: number[], offset: number): number {
    return buffer[offset + NODE_RIGHT_OR_START];
}

/** get split axis (internal only): 0=x, 1=y, 2=z */
export function nodeSplitAxis(buffer: number[], offset: number): number {
    return buffer[offset + NODE_AXIS_OR_COUNT];
}

/** copy bounds into existing Box3 */
export function nodeGetBounds(out: Box3, buffer: number[], offset: number): void {
    out[0] = buffer[offset + NODE_MIN_X];
    out[1] = buffer[offset + NODE_MIN_Y];
    out[2] = buffer[offset + NODE_MIN_Z];
    out[3] = buffer[offset + NODE_MAX_X];
    out[4] = buffer[offset + NODE_MAX_Y];
    out[5] = buffer[offset + NODE_MAX_Z];
}

/** get center of node bounds */
export function nodeGetCenter(out: Vec3, buffer: number[], offset: number): void {
    out[0] = (buffer[offset + NODE_MIN_X] + buffer[offset + NODE_MAX_X]) * 0.5;
    out[1] = (buffer[offset + NODE_MIN_Y] + buffer[offset + NODE_MAX_Y]) * 0.5;
    out[2] = (buffer[offset + NODE_MIN_Z] + buffer[offset + NODE_MAX_Z]) * 0.5;
}

/** test ray-AABB intersection using node bounds directly */
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

/** test AABB-AABB intersection using node bounds directly */
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
