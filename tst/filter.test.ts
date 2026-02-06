import { describe, expect, test } from 'vitest';
import { filter, layers } from '../src';

describe('LayerFilter', () => {
	test('should create filter with all layers enabled by default', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const bp1 = layers.addBroadphaseLayer(layerConfig);
		
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp1);
		
		const testFilter = filter.create(layerConfig);
		
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(true);
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp1)).toBe(true);
	});

	test('should disable all layers', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp0);
		
		const testFilter = filter.create(layerConfig);
		filter.disableAllLayers(testFilter, layerConfig);
		
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(false);
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(false);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(false);
	});

	test('should enable all layers', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		
		const testFilter = filter.create(layerConfig);
		filter.disableAllLayers(testFilter, layerConfig);
		filter.enableAllLayers(testFilter, layerConfig);
		
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
	});

	test('should enable object layer and auto-enable its broadphase layer', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const bp1 = layers.addBroadphaseLayer(layerConfig);
		
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp1);
		
		const testFilter = filter.create(layerConfig);
		filter.disableAllLayers(testFilter, layerConfig);
		
		// Enable obj0, should also enable bp0
		filter.enableObjectLayer(testFilter, layerConfig, obj0);
		
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
		
		// obj1 and bp1 should still be disabled
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(false);
		expect(filter.filterBroadphaseLayer(testFilter, bp1)).toBe(false);
	});

	test('should disable object layer but keep broadphase if another object layer uses it', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp0);
		
		const testFilter = filter.create(layerConfig);
		
		// Both object layers enabled, broadphase enabled
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(true);
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
		
		// Disable obj0
		filter.disableObjectLayer(testFilter, layerConfig, obj0);
		
		// obj0 disabled, but bp0 still enabled because obj1 uses it
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(false);
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
	});

	test('should disable object layer and broadphase when no other object layer uses it', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const bp1 = layers.addBroadphaseLayer(layerConfig);
		
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp1);
		
		const testFilter = filter.create(layerConfig);
		
		// Disable obj0
		filter.disableObjectLayer(testFilter, layerConfig, obj0);
		
		// obj0 and bp0 should be disabled
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(false);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(false);
		
		// obj1 and bp1 should still be enabled
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp1)).toBe(true);
	});

	test('should handle multiple object layers sharing a broadphase layer', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp0);
		const obj2 = layers.addObjectLayer(layerConfig, bp0);
		
		const testFilter = filter.create(layerConfig);
		
		// Disable obj0 and obj1
		filter.disableObjectLayer(testFilter, layerConfig, obj0);
		filter.disableObjectLayer(testFilter, layerConfig, obj1);
		
		// Broadphase should still be enabled because obj2 is using it
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
		
		// Disable obj2
		filter.disableObjectLayer(testFilter, layerConfig, obj2);
		
		// Now broadphase should be disabled
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(false);
	});

	test('should re-enable broadphase when object layer is re-enabled', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		
		const testFilter = filter.create(layerConfig);
		
		// Disable obj0 (also disables bp0)
		filter.disableObjectLayer(testFilter, layerConfig, obj0);
		
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(false);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(false);
		
		// Re-enable obj0 (also re-enables bp0)
		filter.enableObjectLayer(testFilter, layerConfig, obj0);
		
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
	});

	test('should handle complex enable/disable scenarios', () => {
		const layerConfig = layers.create();
		
		const bpMoving = layers.addBroadphaseLayer(layerConfig);
		const bpNotMoving = layers.addBroadphaseLayer(layerConfig);
		
		const objDynamic1 = layers.addObjectLayer(layerConfig, bpMoving);
		const objDynamic2 = layers.addObjectLayer(layerConfig, bpMoving);
		const objStatic = layers.addObjectLayer(layerConfig, bpNotMoving);
		const objSensor = layers.addObjectLayer(layerConfig, bpNotMoving);
		
		const testFilter = filter.create(layerConfig);
		
		// Disable all dynamic objects
		filter.disableObjectLayer(testFilter, layerConfig, objDynamic1);
		filter.disableObjectLayer(testFilter, layerConfig, objDynamic2);
		
		// Moving broadphase should be disabled
		expect(filter.filterBroadphaseLayer(testFilter, bpMoving)).toBe(false);
		
		// NotMoving broadphase should still be enabled (static and sensor use it)
		expect(filter.filterBroadphaseLayer(testFilter, bpNotMoving)).toBe(true);
		
		// Disable static
		filter.disableObjectLayer(testFilter, layerConfig, objStatic);
		
		// NotMoving still enabled (sensor uses it)
		expect(filter.filterBroadphaseLayer(testFilter, bpNotMoving)).toBe(true);
		
		// Disable sensor
		filter.disableObjectLayer(testFilter, layerConfig, objSensor);
		
		// Now NotMoving should be disabled
		expect(filter.filterBroadphaseLayer(testFilter, bpNotMoving)).toBe(false);
		
		// Re-enable one dynamic
		filter.enableObjectLayer(testFilter, layerConfig, objDynamic1);
		
		// Moving broadphase should be re-enabled
		expect(filter.filterBroadphaseLayer(testFilter, bpMoving)).toBe(true);
		expect(filter.filterObjectLayer(testFilter, objDynamic1)).toBe(true);
		expect(filter.filterObjectLayer(testFilter, objDynamic2)).toBe(false);
	});

	test('should work with enableAll and disableAll toggle patterns', () => {
		const layerConfig = layers.create();
		
		const bp0 = layers.addBroadphaseLayer(layerConfig);
		const obj0 = layers.addObjectLayer(layerConfig, bp0);
		const obj1 = layers.addObjectLayer(layerConfig, bp0);
		
		const testFilter = filter.create(layerConfig);
		
		// Start with all enabled
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(true);
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
		
		// Disable all
		filter.disableAllLayers(testFilter, layerConfig);
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(false);
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(false);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(false);
		
		// Enable all
		filter.enableAllLayers(testFilter, layerConfig);
		expect(filter.filterObjectLayer(testFilter, obj0)).toBe(true);
		expect(filter.filterObjectLayer(testFilter, obj1)).toBe(true);
		expect(filter.filterBroadphaseLayer(testFilter, bp0)).toBe(true);
	});
});
