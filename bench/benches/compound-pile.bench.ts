import { bench, group } from '@pmndrs/labs';
import { compoundPile } from '../scenarios/compound-pile';
import { runScenario } from '../scenarios/scenario';

group('compound-pile @physics', () => {
    bench('compound-pile', function* () {
        yield () => runScenario(compoundPile);
    });
});
