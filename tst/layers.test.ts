import { describe, expect, test } from 'vitest';
import { layers } from '../src';

describe('Layers', () => {
	test('should create empty layers', () => {
		const layerConfig = layers.create();

		expect(layerConfig.broadphaseLayers).toBe(0);
		expect(layerConfig.objectLayers).toBe(0);
		expect(layerConfig.objectLayerToBroadphaseLayer).toEqual([]);
		expect(layerConfig.objectLayerPairs).toEqual([]);
		expect(layerConfig.objectVsBroadphase).toEqual([]);
		expect(layerConfig.broadphasePairs).toEqual([]);
	});

	test('should add broadphase layers', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const bp1 = layers.addBroadphaseLayer(layerConfig);

		expect(bp0).toBe(0);
		expect(bp1).toBe(1);
		expect(layerConfig.broadphaseLayers).toBe(2);
	});

	test('should add object layers and map to broadphase layers', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const bp1 = layers.addBroadphaseLayer(layerConfig);

		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp1);
		const obj2 = layers.addObjectLayer(layerConfig, bp0);

		expect(obj0).toBe(0);
		expect(obj1).toBe(1);
		expect(obj2).toBe(2);
		expect(layerConfig.objectLayers).toBe(3);

		expect(layerConfig.objectLayerToBroadphaseLayer[obj0]).toBe(bp0);
		expect(layerConfig.objectLayerToBroadphaseLayer[obj1]).toBe(bp1);
		expect(layerConfig.objectLayerToBroadphaseLayer[obj2]).toBe(bp0);
	});

	test('should auto-enable object vs broadphase collision when adding object layer', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const bp1 = layers.addBroadphaseLayer(layerConfig);

		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp1);

		// Object layers should collide with their assigned broadphase layers
		expect(layers.objectLayerCollidesWithBroadphaseLayer(layerConfig, obj0, bp0)).toBe(true);
		expect(layers.objectLayerCollidesWithBroadphaseLayer(layerConfig, obj1, bp1)).toBe(true);

		// But not with other broadphase layers
		expect(layers.objectLayerCollidesWithBroadphaseLayer(layerConfig, obj0, bp1)).toBe(false);
		expect(layers.objectLayerCollidesWithBroadphaseLayer(layerConfig, obj1, bp0)).toBe(false);
	});

	test('should enable collision between object layers', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp0);

		// Initially no collision between object layers
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj0, obj1)).toBe(false);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj1, obj0)).toBe(false);

		// Enable collision
		layers.enableCollision(layerConfig, obj0, obj1);

		// Should be symmetric
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj0, obj1)).toBe(true);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj1, obj0)).toBe(true);
	});

	test('should enable self-collision for object layer', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);

		// Initially no self-collision
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj0, obj0)).toBe(false);

		// Enable self-collision
		layers.enableCollision(layerConfig, obj0, obj0);

		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj0, obj0)).toBe(true);
	});

	test('should disable collision between object layers', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp0);

		// Enable then disable collision
		layers.enableCollision(layerConfig, obj0, obj1);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj0, obj1)).toBe(true);

		layers.disableCollision(layerConfig, obj0, obj1);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj0, obj1)).toBe(false);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, obj1, obj0)).toBe(false);
	});

	test('should auto-enable broadphase layer collision when enabling object layer collision', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const bp1 = layers.addBroadphaseLayer(layerConfig);

		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp1);

		// Initially no collision between different broadphase layers
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bp0, bp1)).toBe(false);
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bp1, bp0)).toBe(false);

		// Enable collision between object layers
		layers.enableCollision(layerConfig, obj0, obj1);

		// Broadphase layers should also be enabled (symmetric)
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bp0, bp1)).toBe(true);
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bp1, bp0)).toBe(true);
	});

	test('should handle broadphase self-collision', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp0);

		// Initially no broadphase self-collision
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bp0, bp0)).toBe(false);

		// Enable collision between two object layers on same broadphase
		layers.enableCollision(layerConfig, obj0, obj1);

		// Broadphase layer should now collide with itself
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bp0, bp0)).toBe(true);
	});

	test('should handle complex multi-layer scenario', () => {
		const layerConfig = layers.create();

		// Create broadphase layers
		const bpMoving = layers.addBroadphaseLayer(layerConfig);
		const bpNotMoving = layers.addBroadphaseLayer(layerConfig);

		// Create object layers
		const objDynamic = layers.addObjectLayer(layerConfig, bpMoving);
		const objStatic = layers.addObjectLayer(layerConfig, bpNotMoving);
		const objSensor = layers.addObjectLayer(layerConfig, bpNotMoving);

		// Enable specific collisions
		layers.enableCollision(layerConfig, objDynamic, objDynamic); // Dynamic vs Dynamic
		layers.enableCollision(layerConfig, objDynamic, objStatic);  // Dynamic vs Static
		layers.enableCollision(layerConfig, objSensor, objDynamic);  // Sensor vs Dynamic

		// Verify object layer collisions
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objDynamic, objDynamic)).toBe(true);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objDynamic, objStatic)).toBe(true);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objStatic, objDynamic)).toBe(true);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objSensor, objDynamic)).toBe(true);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objDynamic, objSensor)).toBe(true);

		// Should NOT collide
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objStatic, objStatic)).toBe(false);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objSensor, objSensor)).toBe(false);
		expect(layers.objectLayerCollidesWithObjectLayer(layerConfig, objStatic, objSensor)).toBe(false);

		// Verify broadphase layer collisions
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bpMoving, bpMoving)).toBe(true);
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bpMoving, bpNotMoving)).toBe(true);
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bpNotMoving, bpMoving)).toBe(true);
		expect(layers.broadphaseLayerCollidesWithBroadphaseLayer(layerConfig, bpNotMoving, bpNotMoving)).toBe(false);
	});

	test('should resize collision tables when adding layers', () => {
		const layerConfig = layers.create();

		const bp0 = layers.addBroadphaseLayer(layerConfig);
		expect(layerConfig.broadphasePairs.length).toBe(1); // 1x1 = 1

		const bp1 = layers.addBroadphaseLayer(layerConfig);
		expect(layerConfig.broadphasePairs.length).toBe(4); // 2x2 = 4

		const _obj0 = layers.addObjectLayer(layerConfig, bp0);
		expect(layerConfig.objectLayerPairs.length).toBe(1); // 1x1 = 1
		expect(layerConfig.objectVsBroadphase.length).toBe(2); // 1x2 = 2

		const _obj1 = layers.addObjectLayer(layerConfig, bp1);
		expect(layerConfig.objectLayerPairs.length).toBe(4); // 2x2 = 4
		expect(layerConfig.objectVsBroadphase.length).toBe(4); // 2x2 = 4
	});
});
