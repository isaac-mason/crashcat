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

/**
 * check if sub shape id is empty.
 * a real path can be all ones too (the last child of a power of two count), so shapes never test this
 * while navigating; they pop their level and range check it.
 */
export function isEmpty(subShapeId: SubShapeId): boolean {
    return subShapeId === EMPTY_SUB_SHAPE_ID;
}

/** pop a level off the sub shape id */
export function pop(out: PopResult, subShapeID: SubShapeId, numBits: number): void {
    // js shifts by 32 are shifts by 0, so the zero and full width levels are spelled out
    if (numBits === 0) {
        out.value = 0;
        out.remainder = subShapeID;
        return;
    }
    if (numBits === MAX_SUB_SHAPE_ID_BITS) {
        out.value = subShapeID;
        out.remainder = EMPTY_SUB_SHAPE_ID;
        return;
    }

    // fill the vacated high bits with 1 so an exhausted id reads as empty
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
    const levelMask = numBits === MAX_SUB_SHAPE_ID_BITS ? 0xffffffff : (1 << numBits) - 1;
    const mask = levelMask << outBuilder.currentBit;

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
