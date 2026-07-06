import { configDefaults, defineConfig } from 'vitest/config';

export default defineConfig({
    test: {
        setupFiles: ['./tst/setup.ts'],
        // tst/v8 runs only under `pnpm test:v8`. It spawns child node
        // processes with --allow-natives-syntax and parses --trace-turbo-inlining
        // output; vitest workers can't provide those flags themselves, and
        // V8-version-coupled assertions don't belong in the default suite.
        exclude: [...configDefaults.exclude, 'tst/v8/**'],
    },
});
