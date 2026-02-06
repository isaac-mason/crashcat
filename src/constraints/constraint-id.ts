/**
 * a packed 52 bit number containing a constraint index, type, and sequence number
 * bits 1-24: constraint index (24 bits) - 16M constraints per type
 * bits 25-28: constraint type (4 bits) - 16 constraint types
 * bits 29-52: sequence number (24 bits)
 **/
export type ConstraintId = number;

const CONSTRAINT_INDEX_BITS = 24;
const CONSTRAINT_TYPE_BITS = 4;
const SEQUENCE_BITS = 24;

const CONSTRAINT_INDEX_MASK = (1 << CONSTRAINT_INDEX_BITS) - 1;
const CONSTRAINT_TYPE_MASK = (1 << CONSTRAINT_TYPE_BITS) - 1;
export const SEQUENCE_MASK = (1 << SEQUENCE_BITS) - 1;

const TYPE_SHIFT = CONSTRAINT_INDEX_BITS;
const SEQUENCE_SHIFT = CONSTRAINT_INDEX_BITS + CONSTRAINT_TYPE_BITS;

export const MAX_CONSTRAINT_INDEX = CONSTRAINT_INDEX_MASK;
export const MAX_CONSTRAINT_TYPE = CONSTRAINT_TYPE_MASK;
export const MAX_SEQUENCE = SEQUENCE_MASK;

/** constraint types - encoded in the ConstraintId */
export enum ConstraintType {
	POINT = 0,
	DISTANCE = 1,
	HINGE = 2,
	SLIDER = 3,
	FIXED = 4,
	CONE = 5,
	SWING_TWIST = 6,
	SIX_DOF = 7,
}

/** serializes a constraint index, type, and sequence number into a packed ConstraintId */
export const serConstraintId = (
	index: number,
	type: ConstraintType,
	sequence: number,
): ConstraintId => {
	const i = index & CONSTRAINT_INDEX_MASK;
	const t = type & CONSTRAINT_TYPE_MASK;
	const s = sequence & SEQUENCE_MASK;

	// Use addition and multiplication to avoid 32-bit bitwise limitations
	return i + t * 2 ** TYPE_SHIFT + s * 2 ** SEQUENCE_SHIFT;
};

/** deserializes the constraint index from a packed ConstraintId (index within pool for that type) */
export const getConstraintIdIndex = (id: ConstraintId): number => {
	return id & CONSTRAINT_INDEX_MASK;
};

/** deserializes the constraint type from a packed ConstraintId */
export const getConstraintIdType = (id: ConstraintId): ConstraintType => {
	return (id >>> TYPE_SHIFT) & CONSTRAINT_TYPE_MASK;
};

/** deserializes the sequence number from a packed ConstraintId */
export const getConstraintIdSequence = (id: ConstraintId): number => {
	return Math.floor(id / 2 ** SEQUENCE_SHIFT) & SEQUENCE_MASK;
};

/** an invalid ConstraintId */
export const INVALID_CONSTRAINT_ID: ConstraintId = -1;
