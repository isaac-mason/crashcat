export type SubShapeId = number;

export const MAX_SUB_SHAPE_ID_BITS = 32;
export const EMPTY_SUB_SHAPE_ID: SubShapeId = 0xffffffff;

export type SubShapeIdBuilder = {
    value: number;
    currentBit: number;
};

export type PopResult = {
    value: number;
    remainder: SubShapeId;
};

/** create new sub shape id builder in empty state */
export function builder(): SubShapeIdBuilder {
    return {
        value: EMPTY_SUB_SHAPE_ID,
        currentBit: 0,
    };
}

/** check if sub shape id is empty */
export function isEmpty(subShapeId: SubShapeId): boolean {
    return subShapeId === EMPTY_SUB_SHAPE_ID;
}

/** pop a level off the sub shape id */
export function pop(out: PopResult, subShapeID: SubShapeId, numBits: number): void {
    const mask = (1 << numBits) - 1;
    const fillBits = (EMPTY_SUB_SHAPE_ID << (MAX_SUB_SHAPE_ID_BITS - numBits)) >>> 0;

    out.value = subShapeID & mask;
    out.remainder = ((subShapeID >>> numBits) | fillBits) >>> 0;
}

/** push a new level onto the sub shape id */
export function push(outBuilder: SubShapeIdBuilder, source: SubShapeIdBuilder, id: number, numBits: number): void {
    if (source.currentBit + numBits > MAX_SUB_SHAPE_ID_BITS) {
        throw new Error(`SubShapeID bit overflow: ${source.currentBit + numBits} > ${MAX_SUB_SHAPE_ID_BITS}`);
    }

    // copy source state
    outBuilder.value = source.value;
    outBuilder.currentBit = source.currentBit;

    // clear the bits at the target position
    const mask = ((1 << numBits) - 1) << outBuilder.currentBit;

    outBuilder.value = (outBuilder.value & ~mask) >>> 0;

    // set the new bits at the target position
    outBuilder.value = (outBuilder.value | (id << outBuilder.currentBit)) >>> 0;

    outBuilder.currentBit += numBits;
}

/** push index onto sub shape id path */
export function pushIndex(out: SubShapeIdBuilder, source: SubShapeIdBuilder, index: number, count: number): void {
    const maxIndex = count - 1;
    const numBits = 32 - Math.clz32(maxIndex);
    push(out, source, index, numBits);
}

export function popResult(): PopResult {
    return { value: 0, remainder: EMPTY_SUB_SHAPE_ID };
}

const _popResult = /* @__PURE__ */ popResult();

/** pop index from sub shape id path */
export function popIndex(outResult: PopResult, subShapeID: SubShapeId, count: number): void {
    const maxIndex = count - 1;
    const numBits = 32 - Math.clz32(maxIndex);
    pop(_popResult, subShapeID, numBits);

    outResult.value = _popResult.value;
    outResult.remainder = _popResult.remainder;
}
