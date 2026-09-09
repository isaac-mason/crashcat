import { defineConfig } from '@pmndrs/labs';

export default defineConfig({
    benchDir: './benches',
    // one op builds a whole scenario and runs its steps, so an op is tens of milliseconds rather
    // than the microseconds labs defaults assume. the block time is raised to reach the sample
    // floor without stretching a full suite run past a couple of minutes.
    blockTime: 2,
    minSamples: 12,
});
