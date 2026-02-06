import { vec3 } from 'mathcat';
import { describe, expect, test } from 'vitest';
import { emptyShape, ShapeType, sphere, rigidBody, distanceConstraint, ConstraintSpace, MotionType, updateWorld } from '../../src';
import { createTestWorld } from '../helpers';

describe('EmptyShape', () => {
	test('should create empty shape', () => {
		const shape = emptyShape.create();

		expect(shape.type).toBe(ShapeType.EMPTY);
		expect(shape.volume).toBe(0);
		expect(shape.centerOfMass).toEqual([0, 0, 0]);
		expect(shape.aabb[0]).toEqual([0, 0, 0]);
		expect(shape.aabb[1]).toEqual([0, 0, 0]);
	});

	test('should create static body with empty shape', () => {
		const { world, layers } = createTestWorld();
		const body = rigidBody.create(world, {
			shape: emptyShape.create(),
			objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
			motionType: MotionType.STATIC,
			position: vec3.fromValues(0, 5, 0),
		});

		expect(body.shape.type).toBe(ShapeType.EMPTY);
		expect(body.motionType).toBe(MotionType.STATIC);
		expect(body.position).toEqual([0, 5, 0]);
	});

	test('should convert null shape to empty shape', () => {
		const { world, layers } = createTestWorld();
		const body = rigidBody.create(world, {
			shape: null,
			objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
			motionType: MotionType.STATIC,
			position: vec3.fromValues(0, 10, 0),
		});

		expect(body.shape.type).toBe(ShapeType.EMPTY);
		expect(body.motionType).toBe(MotionType.STATIC);
	});

	test('should not participate in collisions', () => {
		const { world, layers } = createTestWorld();

		// Create shapeless anchor at origin
		const anchor = rigidBody.create(world, {
			shape: null,
			objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
			motionType: MotionType.STATIC,
			position: vec3.fromValues(0, 0, 0),
		});

		// Create dynamic sphere at same position
		const sphereBody = rigidBody.create(world, {
			shape: sphere.create({ radius: 1 }),
			objectLayer: layers.OBJECT_LAYER_MOVING,
			motionType: MotionType.DYNAMIC,
			position: vec3.fromValues(0, 0, 0),
		});

		// Run simulation
		updateWorld(world, undefined, 1 / 60);

		// Should have no contacts (empty shape doesn't collide)
		expect(anchor.contactCount).toBe(0);
		expect(sphereBody.contactCount).toBe(0);
	});

	test('should work with constraints', () => {
		const { world, layers } = createTestWorld();

		// Create invisible anchor point
		const anchor = rigidBody.create(world, {
			shape: null,
			objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
			motionType: MotionType.STATIC,
			position: vec3.fromValues(0, 10, 0),
		});

		// Create dynamic pendulum bob
		const bob = rigidBody.create(world, {
			shape: sphere.create({ radius: 0.5 }),
			objectLayer: layers.OBJECT_LAYER_MOVING,
			motionType: MotionType.DYNAMIC,
			position: vec3.fromValues(0, 5, 0),
		});

		// Connect with distance constraint
		const constraint = distanceConstraint.create(world, {
			bodyIdA: anchor.id,
			bodyIdB: bob.id,
			pointA: vec3.fromValues(0, 10, 0),
			pointB: vec3.fromValues(0, 5, 0),
			space: ConstraintSpace.WORLD,
		});

		expect(constraint).toBeDefined();

		// Apply gravity and run simulation
		world.settings.gravityEnabled = true;

		// Run a few steps
		for (let i = 0; i < 30; i++) {
			updateWorld(world, undefined, 1 / 60);
		}

		// Calculate distance between anchor and bob
		const distance = vec3.distance(anchor.position, bob.position);

		// Should maintain the constraint distance (5 units)
		// The constraint should keep the bob at exactly 5 units from anchor
		expect(distance).toBeCloseTo(5, 1);
	});

	test('should be added to broadphase', () => {
		const { world, layers } = createTestWorld();

		const body = rigidBody.create(world, {
			shape: null,
			objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
			motionType: MotionType.STATIC,
			position: vec3.fromValues(0, 0, 0),
		});

		expect(body.dbvtNode).toBeGreaterThanOrEqual(0);
	});

	test('should have zero mass properties', () => {
		const { world, layers } = createTestWorld();

		const body = rigidBody.create(world, {
			shape: emptyShape.create(),
			objectLayer: layers.OBJECT_LAYER_NOT_MOVING,
			motionType: MotionType.STATIC,
		});

		expect(body.massProperties.mass).toBe(0);
	});

	test('should require mass properties for dynamic bodies', () => {
		const { world, layers } = createTestWorld();

		// Dynamic EmptyShape requires explicit mass properties since it has no geometry
		expect(() => {
			rigidBody.create(world, {
				shape: emptyShape.create(),
				objectLayer: layers.OBJECT_LAYER_MOVING,
				motionType: MotionType.DYNAMIC,
				position: vec3.fromValues(0, 10, 0),
			});
		}).toThrow('Invalid mass');
	});
});
