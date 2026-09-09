import { bench, group } from '@pmndrs/labs';
import { pyramid } from '../scenarios/pyramid';
import { runScenario } from '../scenarios/scenario';

group('pyramid @physics', () => {
    bench('pyramid', function* () {
        yield () => runScenario(pyramid);
    });
});
