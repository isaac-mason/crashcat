/**
 * a packed 52 bit number containing a body index and sequence number
 * bits 1-32: body index (32 bits)
 * bits 33-52: sequence number (20 bits)
 **/
export type BodyId = number;
export declare const SEQUENCE_MASK: number;
export declare const MAX_BODY_INDEX = 4294967295;
export declare const MAX_SEQUENCE: number;
/** serializes a body index and sequence number into a packed BodyId */
export declare const serBodyId: (index: number, sequence: number) => BodyId;
/** deserializes the body index from a packed BodyId */
export declare const getBodyIdIndex: (ref: BodyId) => number;
/** deserializes the sequence number from a packed BodyId */
export declare const getBodyIdSequence: (ref: BodyId) => number;
/** an invalid BodyId */
export declare const INVALID_BODY_ID: BodyId;
