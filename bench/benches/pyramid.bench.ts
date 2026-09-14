import { bench, group } from '@pmndrs/labs';
import { pyramid } from '../scenarios/pyramid';
import { runWindow, warm } from '../scenarios/scenario';

group('pyramid @physics', () => {
    bench('pyramid', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = pyramid.create();
        warm(pyramid, instance);

        yield () => runWindow(pyramid, instance);
    });
});
