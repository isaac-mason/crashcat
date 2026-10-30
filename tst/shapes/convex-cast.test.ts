import { describe, expect, test } from 'vitest';
import { box, castShapeVsShape, createAllCastShapeCollector, createDefaultCastShapeSettings, EMPTY_SUB_SHAPE_ID } from '../../src';

describe('convex vs convex cast', () => {
    test('a cast sliding along a face it overlaps hits at the start, the normal perpendicular to the motion', () => {
        const unit = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 });
        const cast = (settings: ReturnType<typeof createDefaultCastShapeSettings>) => {
            const collector = createAllCastShapeCollector();
            // biome-ignore format: pretty
            castShapeVsShape(
                collector, settings,
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
            return collector;
        };

        // the full shapes overlap, so without the deepest point the normal is only the search direction gjk ended on,
        // as jolt's
        const full = cast(createDefaultCastShapeSettings());
        expect(full.hits.length).toBe(1);
        expect(full.hits[0].fraction).toBe(0);

        // shrunk by their convex radius they are apart, so the normal is the face's: perpendicular to the motion, which
        // is not a back face
        const shrunkSettings = createDefaultCastShapeSettings();
        shrunkSettings.useShrunkenShapeAndConvexRadius = true;
        const shrunk = cast(shrunkSettings);
        expect(shrunk.hits.length).toBe(1);
        const hit = shrunk.hits[0];
        expect(hit.fraction).toBe(0);
        expect(hit.normal[0]).toBe(0);
        expect(Math.abs(hit.normal[1])).toBeCloseTo(1, 5);
    });
});
