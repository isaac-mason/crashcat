import { bench, group } from '@pmndrs/labs';
import { characterTerrain } from '../scenarios/character-terrain';
import { runScenario } from '../scenarios/scenario';

group('character-terrain @physics', () => {
    bench('character-terrain', function* () {
        yield () => runScenario(characterTerrain);
    });
});
