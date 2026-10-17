import { describe, expect, test } from 'vitest';
import { box, castShapeVsShape, createAllCastShapeCollector, createDefaultCastShapeSettings, EMPTY_SUB_SHAPE_ID } from '../../src';

describe('convex vs convex cast', () => {
    test('a cast sliding along a face it overlaps hits at the start, the normal perpendicular to the motion', () => {
        const unit = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 });
        const collector = createAllCastShapeCollector();
        // biome-ignore format: pretty
        castShapeVsShape(
            collector, createDefaultCastShapeSettings(),
            unit, EMPTY_SUB_SHAPE_ID, 0,
            0, 0.99, 0,
            0, 0, 0, 1,
            1, 1, 1,
            2, 0, 0,
            unit, EMPTY_SUB_SHAPE_ID, 0,
            0, 0, 0,
            0, 0, 0, 1,
            1, 1, 1,
        );
        expect(collector.hits.length).toBe(1);
        const hit = collector.hits[0];
        expect(hit.fraction).toBe(0);
        expect(hit.normal[0]).toBe(0);
        expect(Math.abs(hit.normal[1])).toBeCloseTo(1, 5);
    });
});
