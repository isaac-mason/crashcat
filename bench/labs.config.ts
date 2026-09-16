import { defineConfig } from '@pmndrs/labs';

export default defineConfig({
    benchDir: './benches',
    // a window is a whole scenario, so an op is tens to hundreds of milliseconds rather than the
    // microseconds labs defaults assume. the sample floor is what sets a block's length here, so it
    // is kept low; blockTime is left near the default so the cheap scenarios do not over-sample.
    blockTime: 0.5,
    minSamples: 8,
});
