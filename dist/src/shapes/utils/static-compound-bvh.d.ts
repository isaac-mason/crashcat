import type { CompoundShapeChild } from '../compound.js';
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
export declare function nodeChildStart(buffer: number[], offset: number): number;
/** get child count (leaf only). decodes from negative flag. */
export declare function nodeChildCount(buffer: number[], offset: number): number;
/**
 * build a BVH over compound shape children.
 * reorders the children array in place for spatial locality.
 */
export declare function build(children: CompoundShapeChild[], settings: BvhBuildSettings): StaticCompoundBVH;
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
export declare function stats(bvh: StaticCompoundBVH): StaticCompoundBvhStats;
