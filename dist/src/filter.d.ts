import type { RigidBody } from './body/rigid-body.js';
import type { Layers } from './layers.js';
import type { World } from './world.js';
export type Filter = {
    /** enabled object layers (1 = enabled, 0 = disabled) */
    enabledObjectLayers: number[];
    /** enabled broadphase layers (1 = enabled, 0 = disabled) */
    enabledBroadphaseLayers: number[];
    /** collision mask */
    collisionMask: number;
    /** collision group */
    collisionGroups: number;
    /** body filter callback */
    bodyFilter: ((body: RigidBody) => boolean) | undefined;
};
/** create a layer filter with all layers enabled */
export declare function create(layers: Layers): Filter;
/** create a layer filter with all layers enabled based on the world's layer settings */
export declare function forWorld(world: World): Filter;
/** set filter to all layers enabled with full collision mask/group */
export declare function setAllEnabled(filter: Filter, layers: Layers): void;
/** create an empty layer filter with no layers enabled */
export declare function createEmpty(): Filter;
/** enable all broadphase and object layers in the filter */
export declare function enableAllLayers(filter: Filter, layers: Layers): void;
/** disable all broadphase and object layers in the filter */
export declare function disableAllLayers(filter: Filter, layers: Layers): void;
/** enable an object layer and its corresponding broadphase layer in the filter */
export declare function enableObjectLayer(filter: Filter, layers: Layers, objectLayer: number): void;
/** disable an object layer and its corresponding broadphase layer in the filter */
export declare function disableObjectLayer(filter: Filter, layers: Layers, objectLayer: number): void;
/** enables a broadphase layer and all it's object layers */
export declare function enableBroadphaseLayer(filter: Filter, layers: Layers, broadphaseLayer: number): void;
/** disables a broadphase layer and all it's object layers */
export declare function disableBroadphaseLayer(filter: Filter, layers: Layers, broadphaseLayer: number): void;
/** evaluate if a object layer is enabled in the filter */
export declare function filterObjectLayer(filter: Filter, objectLayer: number): boolean;
/** evaluate if a broadphase layer is enabled in the filter */
export declare function filterBroadphaseLayer(filter: Filter, broadphaseLayer: number): boolean;
/** set filter to match all layers that can collide with the given body's object layer */
export declare function setFromBody(filter: Filter, layers: Layers, body: RigidBody): void;
/** copy filter settings from source to destination */
export declare function copy(out: Filter, source: Filter): void;
/** evaluate if two collision groups and masks should collide */
export declare function shouldPairCollide(groupA: number, maskA: number, groupB: number, maskB: number): boolean;
