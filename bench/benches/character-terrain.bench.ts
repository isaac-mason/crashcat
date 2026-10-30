import { bench, group } from '@pmndrs/labs';
import { characterTerrain } from '../scenarios/character-terrain';
import { runWindow, warm } from '../scenarios/scenario';

group('character-terrain @physics', () => {
    bench('character-terrain', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = characterTerrain.create();
        warm(characterTerrain, instance);

        yield () => runWindow(characterTerrain, instance);
    });
});
