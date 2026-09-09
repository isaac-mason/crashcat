import { bench, group } from '@pmndrs/labs';
import { raycastMesh } from '../scenarios/raycast-mesh';
import { runScenario } from '../scenarios/scenario';

group('raycast-mesh @physics', () => {
    bench('raycast-mesh', function* () {
        yield () => runScenario(raycastMesh);
    });
});
