import { bench, group } from '@pmndrs/labs';
import { compoundPile } from '../scenarios/compound-pile';
import { runWindow, warm } from '../scenarios/scenario';

group('compound-pile @physics', () => {
    bench('compound-pile', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = compoundPile.create();
        warm(compoundPile, instance);

        yield () => runWindow(compoundPile, instance);
    });
});
