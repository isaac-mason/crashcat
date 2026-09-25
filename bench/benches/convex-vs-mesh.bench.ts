import { bench, group } from '@pmndrs/labs';
import { convexVsMesh } from '../scenarios/convex-vs-mesh';
import { runWindow, warm } from '../scenarios/scenario';

group('convex-vs-mesh @physics', () => {
    bench('convex-vs-mesh', function* () {
        // build and warm-up are untimed — see scenarios/scenario.ts for why the window resets
        const instance = convexVsMesh.create();
        warm(convexVsMesh, instance);

        yield () => runWindow(convexVsMesh, instance);
    });
});
