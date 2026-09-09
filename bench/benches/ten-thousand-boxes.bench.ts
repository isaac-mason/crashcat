import { bench, group } from '@pmndrs/labs';
import { runScenario } from '../scenarios/scenario';
import { tenThousandBoxes } from '../scenarios/ten-thousand-boxes';

group('ten-thousand-boxes @physics', () => {
    bench('ten-thousand-boxes', function* () {
        yield () => runScenario(tenThousandBoxes);
    });
});
