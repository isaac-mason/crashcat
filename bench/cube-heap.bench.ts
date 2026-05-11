// Macro perf benchmark — cube-heap scenario.
//
// Two benches measure different things:
//
//   `cold`         — fresh world + populate + STEPS_PER_OP frames of fall-and-settle.
//                    Captures world construction + the "first second" cost game
//                    code pays when a level loads.
//
//   `steady-state` — pre-settled world; measured op is STEPS_PER_OP frames of
//                    churn against an already-stable heap. This is the cost
//                    that dominates while a game is running and is what we
//                    should optimize against in the inner loop.
//
// Both use a deterministic mulberry32 RNG so labs's adaptive sampling sees a
// stationary signal. Scenario setup lives in ./scenarios/cube-heap so this
// file and ./run-scenario.ts (cpu-prof attribution) share one source of truth.

import { bench, group } from '@pmndrs/labs';

import { createWorld } from '../src';
import {
    makeRng,
    makeWorldSettings,
    populate,
    RNG_SEED,
    runSim,
    STEADY_WARMUP_STEPS,
    STEPS_PER_OP,
} from './scenarios/cube-heap';

group('cube-heap @scenario', () => {
    bench(`cold (${STEPS_PER_OP} frames, fresh world)`, function* () {
        const settings = makeWorldSettings();
        yield () => {
            const world = createWorld(settings);
            const cubes = populate(world, makeRng(RNG_SEED));
            runSim(world, cubes, makeRng(RNG_SEED), STEPS_PER_OP);
        };
    }).gc('inner');

    bench(`steady-state (${STEPS_PER_OP} frames on settled heap)`, function* () {
        const settings = makeWorldSettings();
        const world = createWorld(settings);
        const cubes = populate(world, makeRng(RNG_SEED));
        runSim(world, cubes, makeRng(RNG_SEED ^ 0xdeadbeef), STEADY_WARMUP_STEPS);
        let seedNonce = 0;
        yield () => {
            // New RNG stream per op keeps the churn distribution stationary
            // across labs's repeated invocations.
            runSim(world, cubes, makeRng(RNG_SEED + seedNonce++), STEPS_PER_OP);
        };
    }).gc('inner');
});
