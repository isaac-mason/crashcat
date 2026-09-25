import { bench, group } from '@pmndrs/labs';
import { ragdollPile } from '../scenarios/ragdoll-pile';
import { runWindow, warm } from '../scenarios/scenario';

group('ragdoll-pile @physics', () => {
    bench('ragdoll-pile', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = ragdollPile.create();
        warm(ragdollPile, instance);

        yield () => runWindow(ragdollPile, instance);
    });
});
