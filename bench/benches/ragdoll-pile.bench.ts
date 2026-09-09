import { bench, group } from '@pmndrs/labs';
import { ragdollPile } from '../scenarios/ragdoll-pile';
import { runScenario } from '../scenarios/scenario';

group('ragdoll-pile @physics', () => {
    bench('ragdoll-pile', function* () {
        yield () => runScenario(ragdollPile);
    });
});
