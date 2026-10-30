import { bench, group } from '@pmndrs/labs';
import { boxStacks } from '../scenarios/box-stacks';
import { runWindow, warm } from '../scenarios/scenario';

group('box-stacks @physics', () => {
    bench('box-stacks', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = boxStacks.create();
        warm(boxStacks, instance);

        yield () => runWindow(boxStacks, instance);
    });
});
