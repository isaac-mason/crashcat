// Spawns a bench scenario under `node --cpu-prof` and returns the emitted
// .cpuprofile. Shared by profile.mjs and perf-report.mjs so the run-and-collect
// dance lives in exactly one place.

import { spawnSync } from 'node:child_process';
import { mkdirSync, readdirSync, readFileSync, rmSync, statSync, unlinkSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BENCH_DIR = resolve(__dirname, '..');
export const PROFILE_DIR = resolve(BENCH_DIR, '.profiles');

// run <scenario> under --cpu-prof, parse the emitted profile, and return
//   { profilePath, profile, cleanup }
// cleanup() removes the .profiles dir; callers decide whether to call it
// (skip it for --keep). throws on failure.
export function runScenario(scenario) {
    mkdirSync(PROFILE_DIR, { recursive: true });
    // clean stale profiles so "newest" can't accidentally pick a previous run
    // if cpu-prof writes to a slightly different name.
    for (const f of readdirSync(PROFILE_DIR)) {
        if (f.endsWith('.cpuprofile')) unlinkSync(resolve(PROFILE_DIR, f));
    }

    const runner = resolve(BENCH_DIR, 'run-scenario.ts');
    const r = spawnSync(
        'node',
        [
            '--cpu-prof',
            `--cpu-prof-dir=${PROFILE_DIR}`,
            `--cpu-prof-name=${scenario}.cpuprofile`,
            '--import',
            'tsx',
            runner,
            scenario,
        ],
        { stdio: 'inherit', cwd: BENCH_DIR },
    );
    if (r.status !== 0) {
        throw new Error(`scenario ${scenario} exited ${r.status}`);
    }

    // cpu-prof writes <PROFILE_DIR>/CPU.<timestamp>.<pid>.<seq>.<name>.cpuprofile.
    // Pick the newest one matching our scenario.
    const candidates = readdirSync(PROFILE_DIR)
        .filter((f) => f.endsWith('.cpuprofile') && f.includes(scenario))
        .map((f) => ({ f, t: statSync(resolve(PROFILE_DIR, f)).mtimeMs }))
        .sort((a, b) => b.t - a.t);
    if (candidates.length === 0) {
        throw new Error('no .cpuprofile emitted — did the scenario run?');
    }
    const profilePath = resolve(PROFILE_DIR, candidates[0].f);
    const profile = JSON.parse(readFileSync(profilePath, 'utf8'));

    const cleanup = () => rmSync(PROFILE_DIR, { recursive: true, force: true });
    return { profilePath, profile, cleanup };
}
