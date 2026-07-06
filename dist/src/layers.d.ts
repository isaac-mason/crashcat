export type Layers = {
    /** number of broadphase layers */
    broadphaseLayers: number;
    /** number of object layers */
    objectLayers: number;
    /** mapping from objectLayer to broadphaseLayer */
    objectLayerToBroadphaseLayer: number[];
    /** collision table: objectLayer pairs. Index i*objectLayers+j indicates if layer i collides with layer j */
    objectLayerPairs: number[];
    /** collision table: objectLayer vs broadphaseLayer. Index i*broadphaseLayers+j indicates if object layer i collides with broadphase layer j */
    objectVsBroadphase: number[];
    /** collision table: broadphaseLayer pairs. Index i*broadphaseLayers+j indicates if broadphase layer i collides with broadphase layer j */
    broadphasePairs: number[];
};
export declare function create(): Layers;
export declare function addBroadphaseLayer(layers: Layers): number;
export declare function addObjectLayer(layers: Layers, broadphaseLayer: number): number;
export declare function enableCollision(layers: Layers, objectLayerA: number, objectLayerB: number): void;
export declare function disableCollision(layers: Layers, objectLayerA: number, objectLayerB: number): void;
export declare function objectLayerCollidesWithBroadphaseLayer(layers: Layers, objectLayer: number, broadphaseLayer: number): boolean;
export declare function broadphaseLayerCollidesWithBroadphaseLayer(layers: Layers, broadphaseLayerA: number, broadphaseLayerB: number): boolean;
export declare function objectLayerCollidesWithObjectLayer(layers: Layers, objectLayerA: number, objectLayerB: number): boolean;
