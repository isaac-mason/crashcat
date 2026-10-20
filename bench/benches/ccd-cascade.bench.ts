import { bench, group } from '@pmndrs/labs';
import { ccdCascade } from '../scenarios/ccd-cascade';
import { runWindow, warm } from '../scenarios/scenario';

group('ccd-cascade @physics', () => {
    bench('ccd-cascade', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = ccdCascade.create();
        warm(ccdCascade, instance);

        yield () => runWindow(ccdCascade, instance);
    });
});
