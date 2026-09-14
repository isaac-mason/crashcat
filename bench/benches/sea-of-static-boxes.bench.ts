import { bench, group } from '@pmndrs/labs';
import { runWindow, warm } from '../scenarios/scenario';
import { seaOfStaticBoxes } from '../scenarios/sea-of-static-boxes';

group('sea-of-static-boxes @physics', () => {
    bench('sea-of-static-boxes', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = seaOfStaticBoxes.create();
        warm(seaOfStaticBoxes, instance);

        yield () => runWindow(seaOfStaticBoxes, instance);
    });
});
