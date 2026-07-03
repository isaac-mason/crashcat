// CPU-prof attribution rig for crashcat scenarios (thin stdout wrapper).
//
//   pnpm perf:profile cube-heap          # runs scenario, prints hotspots
//   pnpm perf:profile cube-heap --keep   # also leaves the .cpuprofile on disk
//
// Spawns `node --cpu-prof tsx bench/run-scenario.ts <scenario>`, waits for it
// to finish, parses the emitted .cpuprofile, and prints:
//   - top N hotspots by self time, with file:line
//   - a category roll-up (broadphase / narrowphase / solver / ...)
//
// The heavy lifting (source-map decode, self-time aggregation, categorization)
// lives in ./lib/cpuprofile.mjs and is shared with perf-report.mjs. This file
// just keeps the human-readable stdout dump. For committed markdown reports and
// before/after diffs, use `pnpm perf:report`.

import { analyzeProfile, formatLocation } from './lib/cpuprofile.mjs';
import { runScenario } from './lib/run.mjs';

const TOP_N = 25;

const scenario = process.argv[2];
const keep = process.argv.includes('--keep');
if (!scenario) {
    console.error('usage: pnpm perf:profile <scenario> [--keep]');
    process.exit(2);
}

let result;
try {
    result = runScenario(scenario);
} catch (e) {
    console.error(e.message);
    process.exit(1);
}

const { hotspots, categories, totalMicros, wallMs, sampleCount } = analyzeProfile(result.profile);

const pct = (m) => ((m / Math.max(1, totalMicros)) * 100).toFixed(1).padStart(5);
const ms = (m) => (m / 1000).toFixed(1);

console.log('');
console.log(`Scenario: ${scenario}`);
console.log(
    `Wall: ${wallMs.toFixed(0)}ms  CPU samples: ${sampleCount}  Attributed: ${(totalMicros / 1000).toFixed(0)}ms`,
);
console.log('');
console.log(`Top ${TOP_N} self-time hotspots:`);
console.log('  pct   ms   function                                          location');
console.log('  ----  ----  ------------------------------------------------  ------------------------');
for (const e of hotspots.slice(0, TOP_N)) {
    const fn = (e.name === '(anonymous)' ? '(anonymous)' : e.name).slice(0, 48).padEnd(48);
    console.log(`  ${pct(e.micros)}% ${ms(e.micros).padStart(5)}  ${fn}  ${formatLocation(e)}`);
}

console.log('');
console.log('By category:');
for (const { category, micros } of categories) {
    console.log(`  ${pct(micros)}% ${ms(micros).padStart(6)}ms  ${category}`);
}

if (!keep) {
    result.cleanup();
} else {
    console.log('');
    console.log(`profile kept at: ${result.profilePath}`);
}
