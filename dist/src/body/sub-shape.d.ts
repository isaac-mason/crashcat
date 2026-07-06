export type SubShapeId = number;
export declare const MAX_SUB_SHAPE_ID_BITS = 32;
export declare const EMPTY_SUB_SHAPE_ID: SubShapeId;
export type SubShapeIdBuilder = {
    value: number;
    currentBit: number;
};
export type PopResult = {
    value: number;
    remainder: SubShapeId;
};
/** create new sub shape id builder in empty state */
export declare function builder(): SubShapeIdBuilder;
/** check if sub shape id is empty */
export declare function isEmpty(subShapeId: SubShapeId): boolean;
/** pop a level off the sub shape id */
export declare function pop(out: PopResult, subShapeID: SubShapeId, numBits: number): void;
/** push a new level onto the sub shape id */
export declare function push(outBuilder: SubShapeIdBuilder, source: SubShapeIdBuilder, id: number, numBits: number): void;
/** push index onto sub shape id path */
export declare function pushIndex(out: SubShapeIdBuilder, source: SubShapeIdBuilder, index: number, count: number): void;
export declare function popResult(): PopResult;
/** pop index from sub shape id path */
export declare function popIndex(outResult: PopResult, subShapeID: SubShapeId, count: number): void;
