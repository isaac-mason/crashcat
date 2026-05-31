// CPU-prof attribution rig for crashcat scenarios.
//
//   pnpm perf:profile cube-heap          # runs scenario, prints hotspots
//   pnpm perf:profile cube-heap --keep   # also leaves the .cpuprofile on disk
//
// Spawns `node --cpu-prof tsx bench/run-scenario.ts <scenario>`, waits for it
// to finish, parses the emitted .cpuprofile, and prints:
//   - top N hotspots by self time, with file:line
//   - a category roll-up (broadphase / narrowphase / solver / ...)
//
// The category mapping is heuristic — it routes by src/ subfolder so the
// numbers make sense at the lever-picker level ("spend the next attempt on
// narrowphase, not broadphase") rather than per-function.

import { spawnSync } from 'node:child_process';
import { mkdirSync, readdirSync, readFileSync, rmSync, statSync, unlinkSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const PROFILE_DIR = resolve(__dirname, '.profiles');
const TOP_N = 25;

const scenario = process.argv[2];
const keep = process.argv.includes('--keep');
if (!scenario) {
    console.error('usage: pnpm perf:profile <scenario> [--keep]');
    process.exit(2);
}

mkdirSync(PROFILE_DIR, { recursive: true });
// Clean stale profiles so picking "the newest" can't accidentally pick a
// previous run if cpu-prof writes to a slightly different name.
for (const f of readdirSync(PROFILE_DIR)) {
    if (f.endsWith('.cpuprofile')) unlinkSync(resolve(PROFILE_DIR, f));
}

const runner = resolve(__dirname, 'run-scenario.ts');
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
    { stdio: 'inherit' },
);
if (r.status !== 0) {
    console.error(`scenario ${scenario} exited ${r.status}`);
    process.exit(r.status ?? 1);
}

// cpu-prof writes <PROFILE_DIR>/CPU.<timestamp>.<pid>.<seq>.<name>.cpuprofile.
// Pick the newest one matching our scenario.
const candidates = readdirSync(PROFILE_DIR)
    .filter((f) => f.endsWith('.cpuprofile') && f.includes(scenario))
    .map((f) => ({ f, t: statSync(resolve(PROFILE_DIR, f)).mtimeMs }))
    .sort((a, b) => b.t - a.t);
if (candidates.length === 0) {
    console.error('no .cpuprofile emitted — did the scenario run?');
    process.exit(1);
}
const profilePath = resolve(PROFILE_DIR, candidates[0].f);
const profile = JSON.parse(readFileSync(profilePath, 'utf8'));

// .cpuprofile schema (V8):
//   nodes: [{ id, callFrame: {functionName, url, lineNumber, columnNumber}, hitCount?, children: [id] }]
//   samples: [nodeId, ...]
//   timeDeltas: [microseconds, ...]    (parallel to samples — time spent attributed to that sample)
//   startTime, endTime: microseconds
//
// Self-time per node = sum of timeDeltas for samples whose nodeId === this.
// hitCount is an alternative coarser proxy. We use timeDeltas for accuracy.

const byId = new Map();
for (const n of profile.nodes) byId.set(n.id, n);

const selfMicros = new Map(); // nodeId → microseconds
for (let i = 0; i < profile.samples.length; i++) {
    const id = profile.samples[i];
    const dt = profile.timeDeltas[i] ?? 0;
    selfMicros.set(id, (selfMicros.get(id) ?? 0) + Math.max(0, dt));
}

// Roll up by (functionName, url) — different anonymous arrow chunks at the
// same line all attribute together, which is what we want at this resolution.
const aggregate = new Map(); // key → { name, url, line, micros }
for (const [id, micros] of selfMicros) {
    const node = byId.get(id);
    if (!node) continue;
    const cf = node.callFrame;
    const name = cf.functionName || '(anonymous)';
    const url = cf.url || '';
    const line = cf.lineNumber ?? -1;
    const key = `${name}\x00${url}\x00${line}`;
    const entry = aggregate.get(key) ?? { name, url, line, micros: 0 };
    entry.micros += micros;
    aggregate.set(key, entry);
}

const totalMicros = [...selfMicros.values()].reduce((a, b) => a + b, 0);
const wallMs = (profile.endTime - profile.startTime) / 1000;

// Categorize by source path. The url field carries either a file:// path
// (tsx-loaded TS files), a tsx virtual path, or a node:* internal.
function categorize(url) {
    if (!url) return 'runtime';
    if (url.startsWith('node:') || url.includes('node_modules/tsx/')) return 'runtime';
    if (url.includes('node_modules/mathcat/')) return 'math';
    if (url.includes('node_modules/')) return 'runtime';
    if (url.includes('/src/broadphase/')) return 'broadphase';
    if (url.includes('/src/collision/')) return 'narrowphase';
    if (url.includes('/src/manifold/')) return 'manifold';
    if (url.includes('/src/constraints/')) return 'solver';
    if (url.includes('/src/contacts.ts') || url.includes('/src/islands.ts')) return 'solver';
    if (url.includes('/src/ccd.ts')) return 'ccd';
    if (url.includes('/src/body/')) return 'body';
    if (url.includes('/src/shapes/')) return 'shapes';
    if (url.includes('/src/update.ts') || url.includes('/src/world.ts')) return 'step';
    if (url.includes('/src/')) return 'crashcat-util';
    if (url.includes('/bench/')) return 'bench-harness';
    return 'other';
}

const categoryMicros = new Map();
for (const e of aggregate.values()) {
    const cat = categorize(e.url);
    categoryMicros.set(cat, (categoryMicros.get(cat) ?? 0) + e.micros);
}

// --- output -------------------------------------------------------------

const pct = (m) => ((m / Math.max(1, totalMicros)) * 100).toFixed(1).padStart(5);
const ms = (m) => (m / 1000).toFixed(1);

function shortenUrl(url) {
    if (!url) return '';
    const m = url.match(/\/(src|bench)\/.+$/);
    if (m) return m[0].slice(1); // strip leading slash
    const node = url.match(/^node:.+$/);
    if (node) return url;
    const pkg = url.match(/node_modules\/(?:\.pnpm\/)?([^/]+\/)?([^/]+\/[^?]+)/);
    if (pkg) return `…/${pkg[2]}`;
    return url.slice(-60);
}

// Line numbers from cpu-prof are unreliable when source is tsx-transpiled on
// the fly: V8 emits the .ts URL but lines from the generated JS, which often
// collapse to 0/1. We hide them unless the file is a .js (built bundle or
// pure-JS dep) where lineNumber actually maps to source.
function showLine(url, line) {
    if (line < 0) return false;
    return /\.(m?js|cjs)(\?|$)/.test(url);
}

const ranked = [...aggregate.values()]
    .filter((e) => e.micros > 0)
    .sort((a, b) => b.micros - a.micros)
    .slice(0, TOP_N);

console.log('');
console.log(`Scenario: ${scenario}`);
console.log(`Wall: ${wallMs.toFixed(0)}ms  CPU samples: ${profile.samples.length}  Attributed: ${(totalMicros / 1000).toFixed(0)}ms`);
console.log('');
console.log(`Top ${TOP_N} self-time hotspots:`);
console.log('  pct   ms   function                                          location');
console.log('  ----  ----  ------------------------------------------------  ------------------------');
for (const e of ranked) {
    const fn = (e.name === '(anonymous)' ? '(anonymous)' : e.name).slice(0, 48).padEnd(48);
    const loc = showLine(e.url, e.line)
        ? `${shortenUrl(e.url)}:${e.line + 1}`
        : shortenUrl(e.url);
    console.log(`  ${pct(e.micros)}% ${ms(e.micros).padStart(5)}  ${fn}  ${loc}`);
}

console.log('');
console.log('By category:');
const cats = [...categoryMicros.entries()].sort((a, b) => b[1] - a[1]);
for (const [cat, micros] of cats) {
    console.log(`  ${pct(micros)}% ${ms(micros).padStart(6)}ms  ${cat}`);
}

if (!keep) {
    rmSync(PROFILE_DIR, { recursive: true, force: true });
} else {
    console.log('');
    console.log(`profile kept at: ${profilePath}`);
}
