// one untimed pass over every scenario: body counts and rough ms/step, for sanity-checking a
// scenario and for sizing its `steps`. this is not a measurement — use `pnpm bench` for that.

import { SCENARIOS } from './scenarios';

const filter = process.argv[2];
const selected = filter ? SCENARIOS.filter((s) => s.name.includes(filter)) : SCENARIOS;

console.log(
    [
        'scenario'.padEnd(22),
        'bodies'.padStart(7),
        'awake'.padStart(7),
        'build'.padStart(9),
        'op'.padStart(9),
        'ms/step'.padStart(9),
    ].join(' '),
);

for (const scenario of selected) {
    const buildStart = performance.now();
    const instance = scenario.create();
    const buildMs = performance.now() - buildStart;
    const bodyCount = instance.world.bodies.pool.length;

    const stepStart = performance.now();
    for (let i = 0; i < scenario.steps; i++) instance.step(i);
    const stepMs = performance.now() - stepStart;

    const awake = instance.world.bodies.activeBodyCount;
    console.log(
        [
            scenario.name.padEnd(22),
            String(bodyCount).padStart(7),
            String(awake).padStart(7),
            `${buildMs.toFixed(1)}ms`.padStart(9),
            `${(buildMs + stepMs).toFixed(1)}ms`.padStart(9),
            (stepMs / scenario.steps).toFixed(3).padStart(9),
        ].join(' '),
    );
}
