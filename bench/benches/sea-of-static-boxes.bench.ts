import { bench, group } from '@pmndrs/labs';
import { runScenario } from '../scenarios/scenario';
import { seaOfStaticBoxes } from '../scenarios/sea-of-static-boxes';

group('sea-of-static-boxes @physics', () => {
    bench('sea-of-static-boxes', function* () {
        yield () => runScenario(seaOfStaticBoxes);
    });
});
