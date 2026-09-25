import { bench, group } from '@pmndrs/labs';
import { convexPile } from '../scenarios/convex-pile';
import { runWindow, warm } from '../scenarios/scenario';

group('convex-pile @physics', () => {
    bench('convex-pile', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = convexPile.create();
        warm(convexPile, instance);

        yield () => runWindow(convexPile, instance);
    });
});
