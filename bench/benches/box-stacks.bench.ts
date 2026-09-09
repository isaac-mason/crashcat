import { bench, group } from '@pmndrs/labs';
import { boxStacks } from '../scenarios/box-stacks';
import { runScenario } from '../scenarios/scenario';

group('box-stacks @physics', () => {
    bench('box-stacks', function* () {
        yield () => runScenario(boxStacks);
    });
});
