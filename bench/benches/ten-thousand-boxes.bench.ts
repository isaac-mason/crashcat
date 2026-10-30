import { bench, group } from '@pmndrs/labs';
import { runWindow, warm } from '../scenarios/scenario';
import { tenThousandBoxes } from '../scenarios/ten-thousand-boxes';

group('ten-thousand-boxes @physics', () => {
    bench('ten-thousand-boxes', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = tenThousandBoxes.create();
        warm(tenThousandBoxes, instance);

        yield () => runWindow(tenThousandBoxes, instance);
    });
});
