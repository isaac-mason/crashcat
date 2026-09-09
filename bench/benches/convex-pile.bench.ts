import { bench, group } from '@pmndrs/labs';
import { convexPile } from '../scenarios/convex-pile';
import { runScenario } from '../scenarios/scenario';

group('convex-pile @physics', () => {
    bench('convex-pile', function* () {
        yield () => runScenario(convexPile);
    });
});
