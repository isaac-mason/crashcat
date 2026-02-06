import { describe, expect, test } from 'vitest';
import { getBodyIdIndex, getBodyIdSequence, serBodyId } from '../../src/body/body-id';

describe('BodyId', () => {
    test('ser/des', () => {
        const index = 123;
        const sequence = 456;
        const bodyRef = serBodyId(index, sequence);
        expect(getBodyIdIndex(bodyRef)).toBe(index);
        expect(getBodyIdSequence(bodyRef)).toBe(sequence);
    });

    test('min values', () => {
        const index = 0;
        const sequence = 0;
        const bodyRef = serBodyId(index, sequence);
        expect(getBodyIdIndex(bodyRef)).toBe(index);
        expect(getBodyIdSequence(bodyRef)).toBe(sequence);
    });
});
