// Entry-point invoked by `bench/profile.mjs` under `node --cpu-prof`.
// Picks a scenario by argv[2] and runs its profiling-shaped payload. The
// parent process owns the cpu-prof file and post-processes it; this file
// just has to deliver a steady stream of in-scenario CPU samples.

const scenario = process.argv[2];
if (!scenario) {
    console.error('usage: run-scenario.ts <scenario-name>');
    process.exit(2);
}

const mod = await import(`./${scenario}.bench.ts`);
if (typeof mod.runForProfiling !== 'function') {
    console.error(`scenario ${scenario} does not export runForProfiling()`);
    process.exit(2);
}

mod.runForProfiling();

export {};
