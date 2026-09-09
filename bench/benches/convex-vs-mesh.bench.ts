import { bench, group } from '@pmndrs/labs';
import { convexVsMesh } from '../scenarios/convex-vs-mesh';
import { runScenario } from '../scenarios/scenario';

group('convex-vs-mesh @physics', () => {
    bench('convex-vs-mesh', function* () {
        yield () => runScenario(convexVsMesh);
    });
});
