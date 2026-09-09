import { bench, group } from '@pmndrs/labs';
import { hingeChain } from '../scenarios/hinge-chain';
import { runScenario } from '../scenarios/scenario';

group('hinge-chain @physics', () => {
    bench('hinge-chain', function* () {
        yield () => runScenario(hingeChain);
    });
});
