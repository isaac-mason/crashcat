// Entry-point invoked by `bench/hitch.mjs` under `node --import tsx`.
// Imports a scenario, builds it via `createScenario()`, runs the warmup, then
// times each of N post-warmup steps with performance.now() and writes the raw
// per-step timings (in ms, GC pauses included) to the JSON path in argv[3].
// The parent process (hitch.mjs) owns stats + report rendering.

import { writeFileSync } from 'node:fs';

const scenario = process.argv[2];
const outPath = process.argv[3];
const steps = Number(process.argv[4] ?? 1200);
if (!scenario || !outPath) {
    console.error('usage: hitch-run.ts <scenario-name> <out.json> [steps]');
    process.exit(2);
}

const mod = await import(`./${scenario}.bench.ts`);
if (typeof mod.createScenario !== 'function') {
    console.error(`scenario ${scenario} does not export createScenario()`);
    process.exit(2);
}

const s = mod.createScenario();
const warmupSteps = s.warmupSteps | 0;

// warmup (untimed)
for (let i = 0; i < warmupSteps; i++) s.stepOnce(i);

// timed steps — one performance.now() pair per simulated step. Raw timings are
// kept exactly as measured: GC pauses landing inside a step ARE the hitches we
// want to see, so nothing is filtered or smoothed.
const timings = new Array(steps);
for (let i = 0; i < steps; i++) {
    const t0 = performance.now();
    s.stepOnce(warmupSteps + i);
    timings[i] = performance.now() - t0;
}

writeFileSync(outPath, JSON.stringify({ scenario, warmupSteps, steps, timings }));
