import { bench, group } from '@pmndrs/labs';
import { raycastMesh } from '../scenarios/raycast-mesh';
import { runWindow, warm } from '../scenarios/scenario';

group('raycast-mesh @physics', () => {
    bench('raycast-mesh', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = raycastMesh.create();
        warm(raycastMesh, instance);

        yield () => runWindow(raycastMesh, instance);
    });
});
