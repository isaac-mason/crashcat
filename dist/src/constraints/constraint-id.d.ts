/**
 * a packed 52 bit number containing a constraint index, type, and sequence number
 * bits 1-24: constraint index (24 bits) - 16M constraints per type
 * bits 25-28: constraint type (4 bits) - 16 constraint types
 * bits 29-52: sequence number (24 bits)
 **/
export type ConstraintId = number;
export declare const SEQUENCE_MASK: number;
export declare const MAX_CONSTRAINT_INDEX: number;
export declare const MAX_CONSTRAINT_TYPE: number;
export declare const MAX_SEQUENCE: number;
/** constraint types - encoded in the ConstraintId */
export declare enum ConstraintType {
    POINT = 0,
    DISTANCE = 1,
    HINGE = 2,
    SLIDER = 3,
    FIXED = 4,
    CONE = 5,
    SWING_TWIST = 6,
    SIX_DOF = 7,
    USER_1 = 8,
    USER_2 = 9,
    USER_3 = 10
}
/** serializes a constraint index, type, and sequence number into a packed ConstraintId */
export declare const serConstraintId: (index: number, type: ConstraintType, sequence: number) => ConstraintId;
/** deserializes the constraint index from a packed ConstraintId (index within pool for that type) */
export declare const getConstraintIdIndex: (id: ConstraintId) => number;
/** deserializes the constraint type from a packed ConstraintId */
export declare const getConstraintIdType: (id: ConstraintId) => ConstraintType;
/** deserializes the sequence number from a packed ConstraintId */
export declare const getConstraintIdSequence: (id: ConstraintId) => number;
/** an invalid ConstraintId */
export declare const INVALID_CONSTRAINT_ID: ConstraintId;
