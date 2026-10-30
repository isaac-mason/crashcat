import { quat, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    castRayVsShape,
    castShapeVsShape,
    collideShapeVsShape,
    createAllCastRayCollector,
    createAllCastShapeCollector,
    createAllCollideShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    staticCompound,
    subShape,
} from '../../src';

// a static compound holding a static compound, one child per leaf so both walks keep pending nodes:
// outer [box at x = -9, inner at the origin, box at x = 9], inner [box at x = -3, box at x = 3]
function nestedStaticCompound() {
    const unitBox = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0 });
    const inner = staticCompound.create({
        children: [
            { position: vec3.fromValues(-3, 0, 0), quaternion: quat.create(), shape: unitBox },
            { position: vec3.fromValues(3, 0, 0), quaternion: quat.create(), shape: unitBox },
        ],
        bvhMaxLeafChildren: 1,
    });
    return staticCompound.create({
        children: [
            { position: vec3.fromValues(-9, 0, 0), quaternion: quat.create(), shape: unitBox },
            { position: vec3.fromValues(0, 0, 0), quaternion: quat.create(), shape: inner },
            { position: vec3.fromValues(9, 0, 0), quaternion: quat.create(), shape: unitBox },
        ],
        bvhMaxLeafChildren: 1,
    });
}

// the child path a sub shape id names: "outer" or "outer/inner"
function childPath(subShapeId: number): string {
    const outer = subShape.popResult();
    subShape.popIndex(outer, subShapeId, 3);
    if (outer.value !== 1) return `${outer.value}`;
    const inner = subShape.popResult();
    subShape.popIndex(inner, outer.remainder, 2);
    return `1/${inner.value}`;
}

const EVERY_BOX = ['0', '1/0', '1/1', '2'];

describe('static compound nested in a static compound', () => {
    test('a ray through every box reports each one, the inner children included', () => {
        const outer = nestedStaticCompound();
        const collector = createAllCastRayCollector();

        // biome-ignore format: pretty
        castRayVsShape(
            collector, createDefaultCastRaySettings(),
            -20, 0, 0,
            1, 0, 0, 40,
            outer, EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );

        expect(collector.hits.map((hit) => childPath(hit.subShapeId)).sort()).toEqual(EVERY_BOX);
    });

    test('a box overlapping every box collides with each one, the inner children included', () => {
        const outer = nestedStaticCompound();
        const bar = box.create({ halfExtents: [10, 0.5, 0.5], convexRadius: 0 });
        const collector = createAllCollideShapeCollector();

        // biome-ignore format: pretty
        collideShapeVsShape(
            collector, createDefaultCollideShapeSettings(),
            bar, EMPTY_SUB_SHAPE_ID, 0,
            0, 0.25, 0,
            0, 0, 0, 1,
            1, 1, 1,
            outer, EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );

        expect(collector.hits.map((hit) => childPath(hit.subShapeIdB)).sort()).toEqual(EVERY_BOX);
    });

    test('the nested compound as a collides with each box it overlaps', () => {
        const outer = nestedStaticCompound();
        const bar = box.create({ halfExtents: [10, 0.5, 0.5], convexRadius: 0 });
        const collector = createAllCollideShapeCollector();

        // biome-ignore format: pretty
        collideShapeVsShape(
            collector, createDefaultCollideShapeSettings(),
            outer, EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
            bar, EMPTY_SUB_SHAPE_ID, 0,
            0, 0.25, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );

        expect(collector.hits.map((hit) => childPath(hit.subShapeIdA)).sort()).toEqual(EVERY_BOX);
    });

    test('a box swept down through every box hits each one, the inner children included', () => {
        const outer = nestedStaticCompound();
        const bar = box.create({ halfExtents: [10, 0.5, 0.5], convexRadius: 0 });
        const collector = createAllCastShapeCollector();

        // biome-ignore format: pretty
        castShapeVsShape(
            collector, createDefaultCastShapeSettings(),
            bar, EMPTY_SUB_SHAPE_ID, 0,
            0, 5, 0,
            0, 0, 0, 1,
            1, 1, 1,
            0, -10, 0,
            outer, EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );

        expect(collector.hits.map((hit) => childPath(hit.subShapeIdB)).sort()).toEqual(EVERY_BOX);
    });

    test('the nested compound swept up through a bar hits it with each box', () => {
        const outer = nestedStaticCompound();
        const bar = box.create({ halfExtents: [10, 0.5, 0.5], convexRadius: 0 });
        const collector = createAllCastShapeCollector();

        // biome-ignore format: pretty
        castShapeVsShape(
            collector, createDefaultCastShapeSettings(),
            outer, EMPTY_SUB_SHAPE_ID, 0,
            0, -5, 0,
            0, 0, 0, 1,
            1, 1, 1,
            0, 10, 0,
            bar, EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );

        expect(collector.hits.map((hit) => childPath(hit.subShapeIdA)).sort()).toEqual(EVERY_BOX);
    });
});
