/**
 * a packed 52 bit number containing a body index and sequence number
 * bits 1-32: body index (32 bits)
 * bits 33-52: sequence number (20 bits)
 **/
export type BodyId = number;

const BODY_INDEX_BITS = 32;
const SEQUENCE_BITS = 20;

const BODY_INDEX_MASK = 0xffffffff;
export const SEQUENCE_MASK = (1 << SEQUENCE_BITS) - 1;

const SEQUENCE_SHIFT = BODY_INDEX_BITS;

export const MAX_BODY_INDEX = BODY_INDEX_MASK;
export const MAX_SEQUENCE = SEQUENCE_MASK;

/** serializes a body index and sequence number into a packed BodyId */
export const serBodyId = (index: number, sequence: number): BodyId => {
    const i = index & BODY_INDEX_MASK;
    const s = sequence & SEQUENCE_MASK;

    return i + s * 2 ** SEQUENCE_SHIFT;
};

/** deserializes the body index from a packed BodyId */
export const getBodyIdIndex = (ref: BodyId): number => {
    return ref >>> 0;
};

/** deserializes the sequence number from a packed BodyId */
export const getBodyIdSequence = (ref: BodyId): number => {
    return Math.floor(ref / 2 ** SEQUENCE_SHIFT) & SEQUENCE_MASK;
};

/** an invalid BodyId */
export const INVALID_BODY_ID: BodyId = -1;
