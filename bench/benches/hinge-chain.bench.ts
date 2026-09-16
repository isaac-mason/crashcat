import { bench, group } from '@pmndrs/labs';
import { hingeChain } from '../scenarios/hinge-chain';
import { runWindow, warm } from '../scenarios/scenario';

group('hinge-chain @physics', () => {
    bench('hinge-chain', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = hingeChain.create();
        warm(hingeChain, instance);

        yield () => runWindow(hingeChain, instance);
    });
});
