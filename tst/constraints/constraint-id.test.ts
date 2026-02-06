import { describe, expect, test } from 'vitest';
import {
	ConstraintType,
	getConstraintIdIndex,
	getConstraintIdSequence,
	getConstraintIdType,
	serConstraintId,
} from '../../src/constraints/constraint-id';

describe('ConstraintId', () => {
	test('ser/des', () => {
		const index = 123;
		const type = ConstraintType.POINT;
		const sequence = 456;
		const constraintId = serConstraintId(index, type, sequence);
		expect(getConstraintIdIndex(constraintId)).toBe(index);
		expect(getConstraintIdType(constraintId)).toBe(type);
		expect(getConstraintIdSequence(constraintId)).toBe(sequence);
	});

	test('min values', () => {
		const index = 0;
		const type = ConstraintType.POINT;
		const sequence = 0;
		const constraintId = serConstraintId(index, type, sequence);
		expect(getConstraintIdIndex(constraintId)).toBe(index);
		expect(getConstraintIdType(constraintId)).toBe(type);
		expect(getConstraintIdSequence(constraintId)).toBe(sequence);
	});

	test('different types same index', () => {
		const index = 42;
		const sequence = 7;

		const pointId = serConstraintId(index, ConstraintType.POINT, sequence);
		const distanceId = serConstraintId(index, ConstraintType.DISTANCE, sequence);
		const hingeId = serConstraintId(index, ConstraintType.HINGE, sequence);

		// All have same index and sequence
		expect(getConstraintIdIndex(pointId)).toBe(index);
		expect(getConstraintIdIndex(distanceId)).toBe(index);
		expect(getConstraintIdIndex(hingeId)).toBe(index);

		expect(getConstraintIdSequence(pointId)).toBe(sequence);
		expect(getConstraintIdSequence(distanceId)).toBe(sequence);
		expect(getConstraintIdSequence(hingeId)).toBe(sequence);

		// But different types
		expect(getConstraintIdType(pointId)).toBe(ConstraintType.POINT);
		expect(getConstraintIdType(distanceId)).toBe(ConstraintType.DISTANCE);
		expect(getConstraintIdType(hingeId)).toBe(ConstraintType.HINGE);

		// And IDs are different
		expect(pointId).not.toBe(distanceId);
		expect(pointId).not.toBe(hingeId);
		expect(distanceId).not.toBe(hingeId);
	});
});
