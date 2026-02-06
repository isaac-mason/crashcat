import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import {
	box,
	castShapeVsShape,
	collideShapeVsShape,
	computeMassProperties,
	createAllCastShapeCollector,
	createAllCollideShapeCollector,
	createDefaultCastShapeSettings,
	createDefaultCollideShapeSettings,
	EMPTY_SUB_SHAPE_ID,
	getShapeInnerRadius,
	massProperties,
	MotionType,
	offsetCenterOfMass,
	rigidBody,
	ShapeType,
	sphere,
	updateWorld,
} from '../../src';
import { createTestWorld } from '../helpers';

describe('OffsetCenterOfMassShape - Phase 1', () => {
	test('should create offset center of mass shape', () => {
		const innerSphere = sphere.create({ radius: 1.0 });
		const offset = vec3.fromValues(0, -2, 0);
		const shape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		expect(shape.type).toBe(ShapeType.OFFSET_CENTER_OF_MASS);
		expect(shape.shape).toBe(innerSphere);
		expect(shape.offset).toEqual(offset);
	});

	test('center of mass should be offset from inner shape', () => {
		const innerSphere = sphere.create({ radius: 1.0 });
		const offset = vec3.fromValues(0, -2, 0);
		const shape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		// inner sphere COM is [0, 0, 0]
		// offset shape COM should be [0, -2, 0]
		expect(shape.centerOfMass[0]).toBeCloseTo(0, 5);
		expect(shape.centerOfMass[1]).toBeCloseTo(-2, 5);
		expect(shape.centerOfMass[2]).toBeCloseTo(0, 5);
	});

	test('local bounds should be unchanged from inner shape', () => {
		const innerSphere = sphere.create({ radius: 1.0 });
		const offset = vec3.fromValues(0, 2, 0);
		const shape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		// in crashcat, collision geometry is at shape origin, not COM
		// offsetting COM does not change collision bounds
		// inner sphere bounds: [-1, -1, -1] to [1, 1, 1]
		// expected: same as inner shape
		expect(shape.aabb[0][0]).toBeCloseTo(-1, 5);
		expect(shape.aabb[0][1]).toBeCloseTo(-1, 5);
		expect(shape.aabb[0][2]).toBeCloseTo(-1, 5);
		expect(shape.aabb[1][0]).toBeCloseTo(1, 5);
		expect(shape.aabb[1][1]).toBeCloseTo(1, 5);
		expect(shape.aabb[1][2]).toBeCloseTo(1, 5);
	});

	test('volume should be unchanged from inner shape', () => {
		const innerSphere = sphere.create({ radius: 1.0 });
		const offset = vec3.fromValues(1, 2, 3);
		const shape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		expect(shape.volume).toBeCloseTo(innerSphere.volume, 5);
	});

	test('should handle zero offset', () => {
		const innerSphere = sphere.create({ radius: 1.0 });
		const offset = vec3.fromValues(0, 0, 0);
		const shape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		// with zero offset, should behave like inner shape
		expect(shape.centerOfMass).toEqual(innerSphere.centerOfMass);
		expect(shape.aabb).toEqual(innerSphere.aabb);
		expect(shape.volume).toEqual(innerSphere.volume);
	});
});

describe('OffsetCenterOfMassShape - Phase 2', () => {
	test('should compute mass properties with translated inertia', () => {
		// create a box at origin
		const innerShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
		const offset = vec3.fromValues(0, 2, 0); // offset COM upward
		const shape = offsetCenterOfMass.create({ shape: innerShape, offset });

		const mp = massProperties.create();
		computeMassProperties(mp, shape);

		// mass should be unchanged
		const innerMp = massProperties.create();
		computeMassProperties(innerMp, innerShape);
		expect(mp.mass).toBeCloseTo(innerMp.mass);

		// inertia should be different due to parallel axis theorem
		// inertia increases when mass is farther from COM
		// for offset in Y, Ixx and Izz should increase
		expect(mp.inertia[0]).toBeGreaterThan(innerMp.inertia[0]); // Ixx (mat4 index 0)
		expect(mp.inertia[5]).toBeCloseTo(innerMp.inertia[5]); // Iyy (mat4 index 5, no change for Y offset)
		expect(mp.inertia[10]).toBeGreaterThan(innerMp.inertia[10]); // Izz (mat4 index 10)
	});

	test('should preserve inner radius', () => {
		const innerShape = sphere.create({ radius: 1 });
		const offset = vec3.fromValues(0, -0.5, 0);
		const shape = offsetCenterOfMass.create({ shape: innerShape, offset });

		// inner radius should be unchanged (geometry doesn't move)
		const innerRadius = getShapeInnerRadius(shape);
		const expectedInnerRadius = getShapeInnerRadius(innerShape);
		expect(innerRadius).toBeCloseTo(expectedInnerRadius);
	});

	test('should handle nested offset shapes', () => {
		const innerShape = sphere.create({ radius: 1 });
		const offset1 = vec3.fromValues(0, 0.5, 0);
		const shape1 = offsetCenterOfMass.create({ shape: innerShape, offset: offset1 });

		const offset2 = vec3.fromValues(0, 0.3, 0);
		const shape2 = offsetCenterOfMass.create({ shape: shape1, offset: offset2 });

		// total offset should be 0.5 + 0.3 = 0.8
		expect(shape2.centerOfMass[1]).toBeCloseTo(0.8);

		// bounds stay at shape origin (collision geometry unchanged)
		expect(shape2.aabb[0][1]).toBeCloseTo(-1); // min Y
		expect(shape2.aabb[1][1]).toBeCloseTo(1); // max Y
	});

	test('should compute mass properties for complex offset', () => {
		const innerShape = box.create({ halfExtents: vec3.fromValues(2, 1, 0.5) });
		const offset = vec3.fromValues(1, -1, 0.5); // offset in multiple directions
		const shape = offsetCenterOfMass.create({ shape: innerShape, offset });

		const mp = massProperties.create();
		computeMassProperties(mp, shape);

		// verify mass is preserved
		const innerMp = massProperties.create();
		computeMassProperties(innerMp, innerShape);
		expect(mp.mass).toBeCloseTo(innerMp.mass);

		// all inertia components should increase (offset in all directions)
		expect(mp.inertia[0]).toBeGreaterThan(innerMp.inertia[0]); // Ixx (mat4 index 0)
		expect(mp.inertia[5]).toBeGreaterThan(innerMp.inertia[5]); // Iyy (mat4 index 5)
		expect(mp.inertia[10]).toBeGreaterThan(innerMp.inertia[10]); // Izz (mat4 index 10)
	});
});

describe('OffsetCenterOfMassShape - Phase 3', () => {
	test('should collide offset shape vs sphere', () => {
		// create an offset shape with sphere at origin, COM offset down
		const innerSphere = sphere.create({ radius: 1 });
		const offset = vec3.fromValues(0, -2, 0); // COM is 2 units below geometry
		const offsetShape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		// create a target sphere
		const targetSphere = sphere.create({ radius: 1 });

		const collector = createAllCollideShapeCollector();
		const settings = createDefaultCollideShapeSettings();

		// biome-ignore format: readability
		collideShapeVsShape(
			collector,
			settings,
			offsetShape,
			EMPTY_SUB_SHAPE_ID, 0,
			0, 0, 0, // offsetShape at origin (geometry centered here)
			0, 0, 0, 1, // identity quaternion
			1, 1, 1, // scale
			targetSphere,
			EMPTY_SUB_SHAPE_ID, 0,
			0, 1.9, 0, // target sphere just touching geometry
			0, 0, 0, 1, // identity quaternion
			1, 1, 1, // scale
		);

		// should detect collision
		expect(collector.hits.length).toBeGreaterThan(0);
	});

	test('should collide sphere vs offset shape', () => {
		// reverse order from previous test
		const innerSphere = sphere.create({ radius: 1 });
		const offset = vec3.fromValues(0, 2, 0); // COM is 2 units above geometry
		const offsetShape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		const targetSphere = sphere.create({ radius: 1 });

		const collector = createAllCollideShapeCollector();
		const settings = createDefaultCollideShapeSettings();

		// biome-ignore format: readability
		collideShapeVsShape(
			collector,
			settings,
			targetSphere,
			EMPTY_SUB_SHAPE_ID, 0,
			0, -1.9, 0,
			0, 0, 0, 1,
			1, 1, 1,
			offsetShape,
			EMPTY_SUB_SHAPE_ID, 0,
			0, 0, 0,
			0, 0, 0, 1,
			1, 1, 1,
		);

		expect(collector.hits.length).toBeGreaterThan(0);
	});

	test('should cast offset shape vs sphere', () => {
		const innerSphere = sphere.create({ radius: 1 });
		const offset = vec3.fromValues(0, -1, 0);
		const offsetShape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		const targetSphere = sphere.create({ radius: 1 });

		const collector = createAllCastShapeCollector();
		const settings = createDefaultCastShapeSettings();

		// biome-ignore format: readability
		castShapeVsShape(
			collector,
			settings,
			offsetShape,
			EMPTY_SUB_SHAPE_ID, 0,
			-5, 0, 0, // start position
			0, 0, 0, 1, // quaternion
			1, 1, 1, // scale
			10, 0, 0, // displacement (sweep right)
			targetSphere,
			EMPTY_SUB_SHAPE_ID, 0,
			0, 0, 0, // target at origin
			0, 0, 0, 1,
			1, 1, 1,
		);

		// should hit at some point during the sweep
		expect(collector.hits.length).toBeGreaterThan(0);
		expect(collector.hits[0].fraction).toBeGreaterThan(0);
		expect(collector.hits[0].fraction).toBeLessThan(1);
	});

	test('should cast sphere vs offset shape', () => {
		const innerSphere = sphere.create({ radius: 1 });
		const offset = vec3.fromValues(1, 0, 0);
		const offsetShape = offsetCenterOfMass.create({ shape: innerSphere, offset });

		const movingSphere = sphere.create({ radius: 1 });

		const collector = createAllCastShapeCollector();
		const settings = createDefaultCastShapeSettings();

		// biome-ignore format: readability
		castShapeVsShape(
			collector,
			settings,
			movingSphere,
			EMPTY_SUB_SHAPE_ID, 0,
			-5, 0, 0,
			0, 0, 0, 1,
			1, 1, 1,
			10, 0, 0,
			offsetShape,
			EMPTY_SUB_SHAPE_ID, 0,
			0, 0, 0,
			0, 0, 0, 1,
			1, 1, 1,
		);

		expect(collector.hits.length).toBeGreaterThan(0);
		expect(collector.hits[0].fraction).toBeGreaterThan(0);
		expect(collector.hits[0].fraction).toBeLessThan(1);
	});

	test('should work in physics simulation', () => {
		// create a world with an offset COM shape as a "boat" that's stable
		const { world, layers } = createTestWorld();

		// create a box-shaped boat with COM lowered for stability
		const boatBox = box.create({ halfExtents: vec3.fromValues(2, 0.5, 1) });
		const offset = vec3.fromValues(0, -0.3, 0); // lower COM by 0.3 units
		const boatShape = offsetCenterOfMass.create({ shape: boatBox, offset });

		const boat = rigidBody.create(world, {
			shape: boatShape,
			position: vec3.fromValues(0, 5, 0),
			objectLayer: layers.OBJECT_LAYER_MOVING,
			motionType: MotionType.DYNAMIC,
		});

		// create a ground plane
		const groundBox = box.create({ halfExtents: vec3.fromValues(100, 1, 100) });
		rigidBody.create(world, {
			shape: groundBox,
			position: vec3.fromValues(0, -1, 0),
			objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
			motionType: MotionType.STATIC,
		});

		// step simulation
		for (let i = 0; i < 60; i++) {
			updateWorld(world, undefined, 1 / 60);
		}

		// boat should have settled on ground (y position should be close to ground surface)
		expect(boat.position[1]).toBeLessThan(6);
		expect(boat.position[1]).toBeGreaterThan(0);
	});
});

describe('OffsetCenterOfMassShape - Impulse Tests', () => {
	test('angular impulse with zero offset', () => {
		const { world, layers } = createTestWorld();
		world.settings.gravity = vec3.fromValues(0, 0, 0);

		// create box with zero COM offset
		const halfExtent = vec3.fromValues(0.5, 1.0, 1.5);
		const boxShape = box.create({ halfExtents: halfExtent, density: 1000 });
		const offsetShape = offsetCenterOfMass.create({
			shape: boxShape,
			offset: vec3.fromValues(0, 0, 0),
		});

		const body = rigidBody.create(world, {
			shape: offsetShape,
			position: vec3.fromValues(0, 0, 0),
			objectLayer: layers.OBJECT_LAYER_MOVING,
			motionType: MotionType.DYNAMIC,
		});

		// verify mass and inertia calculated correctly
		const volume = 8.0 * halfExtent[0] * halfExtent[1] * halfExtent[2];
		const mass = volume * 1000; // density
		expect(body.motionProperties.invMass).toBeCloseTo(1.0 / mass, 5);

		// formula: I_y = m/12 * (width² + depth²)
		// see: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
		const inertiaY = (mass / 12.0) * (4 * halfExtent[0] * halfExtent[0] + 4 * halfExtent[2] * halfExtent[2]);
		const invInertiaY = body.motionProperties.invInertiaDiagonal[1];
		expect(invInertiaY).toBeCloseTo(1.0 / inertiaY, 5);

		// add angular impulse around Y axis
		const impulse = vec3.fromValues(0, 10000, 0);
		expect(body.sleeping).toBe(false); // body starts awake
		rigidBody.addAngularImpulse(world, body, impulse);

		// verify resulting velocity change: Δω = I⁻¹ * L
		const deltaV = (1.0 / inertiaY) * impulse[1];
		expect(body.motionProperties.linearVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[1]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[2]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[1]).toBeCloseTo(deltaV, 3);
		expect(body.motionProperties.angularVelocity[2]).toBeCloseTo(0, 5);
	});

	test('angular impulse with COM offset', () => {
		const { world, layers } = createTestWorld();
		world.settings.gravity = vec3.fromValues(0, 0, 0);

		// create box with COM offset
		const halfExtent = vec3.fromValues(0.5, 1.0, 1.5);
		const boxShape = box.create({ halfExtents: halfExtent, density: 1000 });
		const comOffset = vec3.fromValues(5.0, 0, 0);
		const offsetShape = offsetCenterOfMass.create({
			shape: boxShape,
			offset: comOffset,
		});

		const body = rigidBody.create(world, {
			shape: offsetShape,
			position: vec3.fromValues(0, 0, 0),
			objectLayer: layers.OBJECT_LAYER_MOVING,
			motionType: MotionType.DYNAMIC,
		});

		// verify mass and inertia calculated correctly with parallel axis theorem
		const volume = 8.0 * halfExtent[0] * halfExtent[1] * halfExtent[2];
		const mass = volume * 1000; // density
		expect(body.motionProperties.invMass).toBeCloseTo(1.0 / mass, 5);

		// formula: I_y = m/12 * (width² + depth²) + m * offset_x²
		// see: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
		// see: https://en.wikipedia.org/wiki/Parallel_axis_theorem
		const inertiaY =
			(mass / 12.0) * (4 * halfExtent[0] * halfExtent[0] + 4 * halfExtent[2] * halfExtent[2]) +
			mass * comOffset[0] * comOffset[0];
		const invInertiaY = body.motionProperties.invInertiaDiagonal[1];
		expect(invInertiaY).toBeCloseTo(1.0 / inertiaY, 4);

		// add angular impulse around Y axis
		const impulse = vec3.fromValues(0, 10000, 0);
		rigidBody.addAngularImpulse(world, body, impulse);

		// verify resulting velocity change: Δω = I⁻¹ * L
		const deltaV = (1.0 / inertiaY) * impulse[1];
		expect(body.motionProperties.linearVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[1]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[2]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[1]).toBeCloseTo(deltaV, 3);
		expect(body.motionProperties.angularVelocity[2]).toBeCloseTo(0, 5);
	});

	test('torque with zero offset', () => {
		const { world, layers } = createTestWorld();
		world.settings.gravity = vec3.fromValues(0, 0, 0);

		// create box with zero COM offset
		const halfExtent = vec3.fromValues(0.5, 1.0, 1.5);
		const boxShape = box.create({ halfExtents: halfExtent, density: 1000 });
		const offsetShape = offsetCenterOfMass.create({
			shape: boxShape,
			offset: vec3.fromValues(0, 0, 0),
		});

		const body = rigidBody.create(world, {
			shape: offsetShape,
			position: vec3.fromValues(0, 0, 0),
			objectLayer: layers.OBJECT_LAYER_MOVING,
			motionType: MotionType.DYNAMIC,
		});

		// verify mass and inertia
		const volume = 8.0 * halfExtent[0] * halfExtent[1] * halfExtent[2];
		const mass = volume * 1000;
		const inertiaY = (mass / 12.0) * (4 * halfExtent[0] * halfExtent[0] + 4 * halfExtent[2] * halfExtent[2]);
		const invInertiaY = body.motionProperties.invInertiaDiagonal[1];
		expect(invInertiaY).toBeCloseTo(1.0 / inertiaY, 5);

		// add torque around Y axis
		const torque = vec3.fromValues(0, 100000, 0);
		rigidBody.addTorque(world, body, torque, false);

		// angular velocity change should come after the next time step
		expect(body.motionProperties.angularVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[1]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[2]).toBeCloseTo(0, 5);

		const dt = 1 / 60;
		updateWorld(world, undefined, dt);

		// verify resulting velocity change: Δω = I⁻¹ * τ * Δt
		const deltaV = (1.0 / inertiaY) * torque[1] * dt;
		expect(body.motionProperties.linearVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[1]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[2]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[1]).toBeCloseTo(deltaV, 3);
		expect(body.motionProperties.angularVelocity[2]).toBeCloseTo(0, 5);
	});

	test('torque with COM offset', () => {
		const { world, layers } = createTestWorld();
		world.settings.gravity = vec3.fromValues(0, 0, 0);

		// create box with COM offset
		const halfExtent = vec3.fromValues(0.5, 1.0, 1.5);
		const boxShape = box.create({ halfExtents: halfExtent, density: 1000 });
		const comOffset = vec3.fromValues(5.0, 0, 0);
		const offsetShape = offsetCenterOfMass.create({
			shape: boxShape,
			offset: comOffset,
		});

		const body = rigidBody.create(world, {
			shape: offsetShape,
			position: vec3.fromValues(0, 0, 0),
			objectLayer: layers.OBJECT_LAYER_MOVING,
			motionType: MotionType.DYNAMIC,
		});

		// verify mass and inertia with parallel axis theorem
		const volume = 8.0 * halfExtent[0] * halfExtent[1] * halfExtent[2];
		const mass = volume * 1000;
		const inertiaY =
			(mass / 12.0) * (4 * halfExtent[0] * halfExtent[0] + 4 * halfExtent[2] * halfExtent[2]) +
			mass * comOffset[0] * comOffset[0];
		const invInertiaY = body.motionProperties.invInertiaDiagonal[1];
		expect(invInertiaY).toBeCloseTo(1.0 / inertiaY, 4);

		// add torque around Y axis
		const torque = vec3.fromValues(0, 100000, 0);
		rigidBody.addTorque(world, body, torque, false);

		// angular velocity change should come after the next time step
		expect(body.motionProperties.angularVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[1]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[2]).toBeCloseTo(0, 5);

		const dt = 1 / 60;
		updateWorld(world, undefined, dt);

		// verify resulting velocity change: Δω = I⁻¹ * τ * Δt
		const deltaV = (1.0 / inertiaY) * torque[1] * dt;
		expect(body.motionProperties.linearVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[1]).toBeCloseTo(0, 5);
		expect(body.motionProperties.linearVelocity[2]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[0]).toBeCloseTo(0, 5);
		expect(body.motionProperties.angularVelocity[1]).toBeCloseTo(deltaV, 3);
		expect(body.motionProperties.angularVelocity[2]).toBeCloseTo(0, 5);
	});
});
