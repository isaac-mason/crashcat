import { type Vec3 } from 'mathcat';
import { type Box3 } from 'mathcat/shapes';
import type { CompoundShapeChild } from './compound.js';
import { ShapeType } from './shapes.js';
import type { BvhBuildSettings, StaticCompoundBVH } from './utils/static-compound-bvh.js';
export type StaticCompoundShapeSettings = {
    children: CompoundShapeChild[];
    /** maximum children per leaf node (default: 4) */
    bvhMaxLeafChildren?: number;
};
export declare const DEFAULT_STATIC_COMPOUND_OPTIONS: {
    bvhMaxLeafChildren: number;
};
export type StaticCompoundShape = {
    type: ShapeType.STATIC_COMPOUND;
    children: CompoundShapeChild[];
    bvh: StaticCompoundBVH;
    bvhSettings: BvhBuildSettings;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};
export declare function create(o: StaticCompoundShapeSettings): StaticCompoundShape;
/**
 * updates a static compound shape after properties have changed.
 * recomputes bounds, center of mass, volume, and rebuilds BVH.
 * note: this reorders the children array and invalidates any existing SubShapeIDs.
 */
export declare function update(shape: StaticCompoundShape): void;
export declare const def: import("./shapes").ShapeDef<StaticCompoundShape>;
