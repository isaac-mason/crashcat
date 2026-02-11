import type { RigidBody } from './body/rigid-body';
import type { Layers } from './layers';
import type { World } from './world';

export type Filter = {
    /** enabled object layers (1 = enabled, 0 = disabled) */
    enabledObjectLayers: number[];
    /** enabled broadphase layers (1 = enabled, 0 = disabled) */
    enabledBroadphaseLayers: number[];
    /** collision mask */
    collisionMask: number;
    /** collision group */
    collisionGroup: number;
    /** body filter callback */
    bodyFilter: ((body: RigidBody) => boolean) | undefined;
};

/** create a layer filter with all layers enabled */
export function create(layers: Layers): Filter {
    const filter: Filter = {
        enabledObjectLayers: [],
        enabledBroadphaseLayers: [],
        collisionMask: 0,
        collisionGroup: 0,
        bodyFilter: undefined,
    };
    setAllEnabled(filter, layers);
    return filter;
}

/** create a layer filter with all layers enabled based on the world's layer settings */
export function forWorld(world: World): Filter {
    return create(world.settings.layers);
}

/** set filter to all layers enabled with full collision mask/group */
export function setAllEnabled(filter: Filter, layers: Layers): void {
    // resize and fill object layers
    filter.enabledObjectLayers.length = layers.objectLayers;
    for (let i = 0; i < layers.objectLayers; i++) {
        filter.enabledObjectLayers[i] = 1;
    }

    // resize and fill broadphase layers
    filter.enabledBroadphaseLayers.length = layers.broadphaseLayers;
    for (let i = 0; i < layers.broadphaseLayers; i++) {
        filter.enabledBroadphaseLayers[i] = 1;
    }

    filter.collisionMask = ~0;
    filter.collisionGroup = ~0;
    filter.bodyFilter = undefined;
}

/** create an empty layer filter with no layers enabled */
export function createEmpty(): Filter {
    return {
        enabledObjectLayers: [],
        enabledBroadphaseLayers: [],
        collisionMask: 0,
        collisionGroup: 0,
        bodyFilter: undefined,
    };
}

/** enable all broadphase and object layers in the filter */
export function enableAllLayers(filter: Filter, layers: Layers): void {
    for (let i = 0; i < layers.objectLayers; i++) {
        filter.enabledObjectLayers[i] = 1;
    }
    for (let i = 0; i < layers.broadphaseLayers; i++) {
        filter.enabledBroadphaseLayers[i] = 1;
    }
}

/** disable all broadphase and object layers in the filter */
export function disableAllLayers(filter: Filter, layers: Layers): void {
    for (let i = 0; i < layers.objectLayers; i++) {
        filter.enabledObjectLayers[i] = 0;
    }
    for (let i = 0; i < layers.broadphaseLayers; i++) {
        filter.enabledBroadphaseLayers[i] = 0;
    }
}

/** enable an object layer and its corresponding broadphase layer in the filter */
export function enableObjectLayer(filter: Filter, layers: Layers, objectLayer: number): void {
    filter.enabledObjectLayers[objectLayer] = 1;

    // auto-enable the broadphase layer this object layer belongs to
    const broadphaseLayer = layers.objectLayerToBroadphaseLayer[objectLayer];
    if (broadphaseLayer !== undefined) {
        filter.enabledBroadphaseLayers[broadphaseLayer] = 1;
    }
}

/** disable an object layer and its corresponding broadphase layer in the filter */
export function disableObjectLayer(filter: Filter, layers: Layers, objectLayer: number): void {
    filter.enabledObjectLayers[objectLayer] = 0;

    // check if any other enabled object layers use this broadphase layer
    const broadphaseLayer = layers.objectLayerToBroadphaseLayer[objectLayer];

    if (broadphaseLayer !== undefined) {
        let anyOtherObjectLayerUsing = false;
        for (let i = 0; i < layers.objectLayers; i++) {
            if (
                i !== objectLayer &&
                filter.enabledObjectLayers[i] === 1 &&
                layers.objectLayerToBroadphaseLayer[i] === broadphaseLayer
            ) {
                anyOtherObjectLayerUsing = true;
                break;
            }
        }

        // only disable broadphase layer if no other object layers are using it
        if (!anyOtherObjectLayerUsing) {
            filter.enabledBroadphaseLayers[broadphaseLayer] = 0;
        }
    }
}

/** enables a broadphase layer and all it's object layers */
export function enableBroadphaseLayer(filter: Filter, layers: Layers, broadphaseLayer: number): void {
    filter.enabledBroadphaseLayers[broadphaseLayer] = 1;

    // enable all object layers that use this broadphase layer
    for (let i = 0; i < filter.enabledObjectLayers.length; i++) {
        if (layers.objectLayerToBroadphaseLayer[i] === broadphaseLayer) {
            filter.enabledObjectLayers[i] = 1;
        }
    }
}

/** disables a broadphase layer and all it's object layers */
export function disableBroadphaseLayer(filter: Filter, layers: Layers, broadphaseLayer: number): void {
    filter.enabledBroadphaseLayers[broadphaseLayer] = 0;

    // disable all object layers that use this broadphase layer
    for (let i = 0; i < filter.enabledObjectLayers.length; i++) {
        if (layers.objectLayerToBroadphaseLayer[i] === broadphaseLayer) {
            filter.enabledObjectLayers[i] = 0;
        }
    }
}

/** evaluate if a object layer is enabled in the filter */
export function filterObjectLayer(filter: Filter, objectLayer: number): boolean {
    return filter.enabledObjectLayers[objectLayer] === 1;
}

/** evaluate if a broadphase layer is enabled in the filter */
export function filterBroadphaseLayer(filter: Filter, broadphaseLayer: number): boolean {
    return filter.enabledBroadphaseLayers[broadphaseLayer] === 1;
}

/** set filter to match all layers that can collide with the given body's object layer */
export function setFromBody(filter: Filter, layers: Layers, body: RigidBody): void {
    // disable all layers first
    disableAllLayers(filter, layers);

    // enable all object layers that can collide with this body's layer
    for (let i = 0; i < layers.objectLayers; i++) {
        if (layers.objectLayerPairs[body.objectLayer * layers.objectLayers + i] === 1) {
            filter.enabledObjectLayers[i] = 1;
        }
    }

    // enable all broadphase layers that this body's object layer can query
    for (let i = 0; i < layers.broadphaseLayers; i++) {
        if (layers.objectVsBroadphase[body.objectLayer * layers.broadphaseLayers + i] === 1) {
            filter.enabledBroadphaseLayers[i] = 1;
        }
    }

    // set collision group and mask from body
    filter.collisionGroup = body.collisionGroup;
    filter.collisionMask = body.collisionMask;
}

/** copy filter settings from source to destination */
export function copy(out: Filter, source: Filter): void {
    // copy layer settings
    for (let i = 0; i < source.enabledObjectLayers.length; i++) {
        out.enabledObjectLayers[i] = source.enabledObjectLayers[i];
    }
    for (let i = 0; i < source.enabledBroadphaseLayers.length; i++) {
        out.enabledBroadphaseLayers[i] = source.enabledBroadphaseLayers[i];
    }

    // copy collision properties
    out.collisionGroup = source.collisionGroup;
    out.collisionMask = source.collisionMask;

    // copy body filter
    out.bodyFilter = source.bodyFilter;
}

/** evaluate if two collision groups and masks should collide */
export function shouldPairCollide(groupA: number, maskA: number, groupB: number, maskB: number): boolean {
    return (groupA & maskB) !== 0 && (groupB & maskA) !== 0;
}
