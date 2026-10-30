// runs each scenario's measured window several times and reports how repeatable it is.
//
// this is the check that the bench rests on: `reset` cannot rewind the contact cache or the shape
// of the broadphase tree, so a window only becomes stationary after a warm-up or two. if the awake
// count still moves between late windows, that scenario's reset is missing state and its numbers
// should not be trusted. this is not a measurement — use `pnpm bench` for that.

import { SCENARIOS } from './scenarios';
import { runWindow } from './scenarios/scenario';

const WINDOWS = 6;

const filter = process.argv[2];
const selected = filter ? SCENARIOS.filter((s) => s.name.includes(filter)) : SCENARIOS;

const header = ['scenario', 'bodies', 'build', 'window', 'ms/step', 'awake per window'];
console.log(
    [header[0].padEnd(22), header[1].padStart(7), header[2].padStart(9), header[3].padStart(9), header[4].padStart(8)].join(' ') +
        '  ' +
        header[5],
);

for (const scenario of selected) {
    const buildStart = performance.now();
    const instance = scenario.create();
    const buildMs = performance.now() - buildStart;
    const bodyCount = instance.world.bodies.pool.length;

    const awake: number[] = [];
    const times: number[] = [];
    for (let w = 0; w < WINDOWS; w++) {
        const start = performance.now();
        awake.push(runWindow(scenario, instance));
        times.push(performance.now() - start);
    }

    // the first couple of windows are the warm-up the bench also runs untimed
    const measured = times.slice(2);
    const median = measured.slice().sort((a, b) => a - b)[Math.floor(measured.length / 2)];
    const stable = awake.slice(2).every((a) => a === awake[2]);

    console.log(
        [
            scenario.name.padEnd(22),
            String(bodyCount).padStart(7),
            `${buildMs.toFixed(1)}ms`.padStart(9),
            `${median.toFixed(1)}ms`.padStart(9),
            (median / scenario.steps).toFixed(3).padStart(8),
        ].join(' ') +
            `  ${awake.join(' ')}` +
            (stable ? '' : '   <- NOT REPEATABLE'),
    );
}
