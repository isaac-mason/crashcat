// Dedicated config for the V8 optimization test suite.
// See tst/v8/v8-opt.test.ts for the rationale; this config exists only so
// the suite isn't filtered out by the main config's `exclude: ['tst/v8/**']`.

import { defineConfig } from 'vitest/config';

export default defineConfig({
    test: {
        include: ['tst/v8/**/*.test.ts'],
        // Subprocess spawns + 50k-iteration hold-under-load can exceed the
        // 5s default per test.
        testTimeout: 30_000,
    },
});
