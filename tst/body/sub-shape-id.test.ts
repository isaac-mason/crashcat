import { quat, vec3 } from 'mathcat';
import { describe, expect, it } from 'vitest';
import { box, compound, sphere, subShape } from '../../src';

describe('SubShapeID', () => {
    it('should detect empty SubShapeIDs', () => {
        expect(subShape.isEmpty(subShape.EMPTY_SUB_SHAPE_ID)).toBe(true);
        expect(subShape.isEmpty(0)).toBe(false);
        expect(subShape.isEmpty(123)).toBe(false);
    });

    it('should push and pop bits correctly', () => {
        const builder1 = subShape.builder();
        const builder2 = subShape.builder();

        // Push a 3-bit value (5 = 0b101)
        subShape.push(builder1, builder2, 5, 3);

        expect(builder1.currentBit).toBe(3);

        // Pop the value back
        const popResult = { value: 0, remainder: subShape.EMPTY_SUB_SHAPE_ID };
        subShape.pop(popResult, builder1.value, 3);

        expect(popResult.value).toBe(5);
        expect(popResult.remainder).toBe(subShape.EMPTY_SUB_SHAPE_ID);
    });

    it('should handle compound shape child IDs', () => {
        const compoundShape = compound.create({
            children: [
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 1 }) },
                { position: vec3.create(), quaternion: quat.create(), shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }) },
                { position: vec3.create(), quaternion: quat.create(), shape: sphere.create({ radius: 2 }) },
            ],
        });

        const builder1 = subShape.builder();
        const builder2 = subShape.builder();

        // Push child index 1
        subShape.pushIndex(builder1, builder2, 1, compoundShape.children.length);

        // Pop it back
        const popResult = subShape.popResult();
        subShape.popIndex(popResult, builder1.value, compoundShape.children.length);
        expect(popResult.value).toBe(1);
        expect(popResult.remainder).toBe(subShape.EMPTY_SUB_SHAPE_ID);
    });

    it('should throw on bit overflow', () => {
        const builder1 = subShape.builder();
        const builder2 = subShape.builder();

        // Try to push more than 32 bits total
        expect(() => {
            subShape.push(builder1, builder2, 0, 33);
        }).toThrow('SubShapeID bit overflow');
    });
});
