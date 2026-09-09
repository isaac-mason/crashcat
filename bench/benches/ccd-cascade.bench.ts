import { bench, group } from '@pmndrs/labs';
import { ccdCascade } from '../scenarios/ccd-cascade';
import { runScenario } from '../scenarios/scenario';

group('ccd-cascade @physics', () => {
    bench('ccd-cascade', function* () {
        yield () => runScenario(ccdCascade);
    });
});
