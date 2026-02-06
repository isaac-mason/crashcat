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
}

export function create(): Layers {
	return {
		broadphaseLayers: 0,
		objectLayers: 0,
		objectLayerToBroadphaseLayer: [],
		objectLayerPairs: [],
		objectVsBroadphase: [],
		broadphasePairs: []
	};
}

export function addBroadphaseLayer(layers: Layers): number {
	const index = layers.broadphaseLayers;
	layers.broadphaseLayers += 1;
	
	// resize broadphase collision table
	const bpCount = layers.broadphaseLayers;
	const bpSize = bpCount * bpCount;
	while (layers.broadphasePairs.length < bpSize) {
		layers.broadphasePairs.push(0);
	}
	
	// resize object vs broadphase table
	const objCount = layers.objectLayers;
	const objBpSize = objCount * bpCount;
	while (layers.objectVsBroadphase.length < objBpSize) {
		layers.objectVsBroadphase.push(0);
	}
	
	return index;
}

export function addObjectLayer(layers: Layers, broadphaseLayer: number): number {
	const index = layers.objectLayers;
	layers.objectLayers += 1;
	layers.objectLayerToBroadphaseLayer[index] = broadphaseLayer;
	
	// resize object layer collision table
	const objCount = layers.objectLayers;
	const objSize = objCount * objCount;
	while (layers.objectLayerPairs.length < objSize) {
		layers.objectLayerPairs.push(0);
	}
	
	// resize object vs broadphase table
	const bpCount = layers.broadphaseLayers;
	const objBpSize = objCount * bpCount;
	while (layers.objectVsBroadphase.length < objBpSize) {
		layers.objectVsBroadphase.push(0);
	}
	
	// auto-enable collision with the broadphase layer it belongs to
	layers.objectVsBroadphase[index * bpCount + broadphaseLayer] = 1;
	
	return index;
}

export function enableCollision(layers: Layers, objectLayerA: number, objectLayerB: number): void {
	const objCount = layers.objectLayers;
	
	// set A → B
	layers.objectLayerPairs[objectLayerA * objCount + objectLayerB] = 1;
	
	// set B → A (symmetric)
	layers.objectLayerPairs[objectLayerB * objCount + objectLayerA] = 1;
	
	// auto-enable broadphase layer collisions
	const bpA = layers.objectLayerToBroadphaseLayer[objectLayerA];
	const bpB = layers.objectLayerToBroadphaseLayer[objectLayerB];
	
	if (bpA !== undefined && bpB !== undefined) {
		const bpCount = layers.broadphaseLayers;
		
		// set bpA → bpB
		layers.broadphasePairs[bpA * bpCount + bpB] = 1;
		
		// set bpB → bpA (symmetric)
		layers.broadphasePairs[bpB * bpCount + bpA] = 1;
		
		// enable cross-broadphase-layer queries
		// objectA can query broadphaseB
		layers.objectVsBroadphase[objectLayerA * bpCount + bpB] = 1;
		
		// objectB can query broadphaseA
		layers.objectVsBroadphase[objectLayerB * bpCount + bpA] = 1;
	}
}

export function disableCollision(layers: Layers, objectLayerA: number, objectLayerB: number): void {
	const objCount = layers.objectLayers;
	
	// clear A → B
	layers.objectLayerPairs[objectLayerA * objCount + objectLayerB] = 0;
	
	// clear B → A (symmetric)
	layers.objectLayerPairs[objectLayerB * objCount + objectLayerA] = 0;
}

export function objectLayerCollidesWithBroadphaseLayer(layers: Layers, objectLayer: number, broadphaseLayer: number): boolean {
	return layers.objectVsBroadphase[objectLayer * layers.broadphaseLayers + broadphaseLayer] === 1;
}

export function broadphaseLayerCollidesWithBroadphaseLayer(layers: Layers, broadphaseLayerA: number, broadphaseLayerB: number): boolean {
	return layers.broadphasePairs[broadphaseLayerA * layers.broadphaseLayers + broadphaseLayerB] === 1;
}

export function objectLayerCollidesWithObjectLayer(layers: Layers, objectLayerA: number, objectLayerB: number): boolean {
	return layers.objectLayerPairs[objectLayerA * layers.objectLayers + objectLayerB] === 1;
}