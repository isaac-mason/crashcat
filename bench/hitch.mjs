// crashcat per-step hitch tool — worst-case frame-time distribution per scenario.
//
//   node hitch.mjs <scenario> [--steps N]
//       builds the scenario via createScenario(), runs its warmup, then times
//       each of N (default 1200) post-warmup steps and writes
//       bench/reports/hitch-<scenario>.md + bench/reports/hitch-<scenario>.json
//       (p50/p90/p99/p99.9/max, a text histogram, and the 5 worst steps).
//
// Where perf:report answers "where does the average step spend its time", this
// answers "how bad is the worst step" — the tail that shows up as a dropped
// frame. Raw timings are kept EXACTLY as measured, GC pauses included: a GC
// pause landing inside a step is a real hitch, so it is a metric here, not
// noise to be filtered.
//
// Style-matched to perf-report.mjs (shared table() helper, same metadata block).

import { execFileSync, spawnSync } from 'node:child_process';
import { mkdirSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { cpus } from 'node:os';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BENCH_DIR = __dirname;
const REPORTS_DIR = resolve(BENCH_DIR, 'reports');
const SCRATCH_DIR = resolve(BENCH_DIR, '.profiles');
const DEFAULT_STEPS = 1200;

// histogram bucket upper-edges in ms (log-ish) — the last bucket is the >64ms tail.
const HIST_EDGES = [0.25, 0.5, 1, 2, 4, 8, 16, 32, 64, Infinity];

// --- formatting helpers (mirrors perf-report.mjs) -------------------------

const ms3 = (v) => v.toFixed(3);

function table(headers, aligns, rows) {
    const widths = headers.map((h, i) => Math.max(h.length, ...rows.map((r) => (r[i] ?? '').length), 3));
    const pad = (s, i) => (aligns[i] === 'r' ? s.padStart(widths[i]) : s.padEnd(widths[i]));
    const line = (cells) => `| ${cells.map((c, i) => pad(c, i)).join(' | ')} |`;
    const sep = `| ${widths.map((w, i) => (aligns[i] === 'r' ? `${'-'.repeat(w - 1)}:` : '-'.repeat(w))).join(' | ')} |`;
    return [line(headers), sep, ...rows.map(line)].join('\n');
}

function gitRev() {
    try {
        const rev = execFileSync('git', ['rev-parse', '--short', 'HEAD'], { cwd: __dirname }).toString().trim();
        const dirty = execFileSync('git', ['status', '--porcelain'], { cwd: __dirname }).toString().trim().length > 0;
        return { rev, dirty };
    } catch {
        return { rev: 'unknown', dirty: false };
    }
}

// --- stats ----------------------------------------------------------------

// nearest-rank percentile over an already-sorted ascending array.
function percentile(sorted, q) {
    if (sorted.length === 0) return 0;
    const idx = Math.min(sorted.length - 1, Math.floor(q * sorted.length));
    return sorted[idx];
}

function analyze(timings) {
    const n = timings.length;
    const sorted = timings.slice().sort((a, b) => a - b);
    const mean = timings.reduce((a, b) => a + b, 0) / Math.max(1, n);

    const buckets = HIST_EDGES.map(() => 0);
    for (const v of timings) {
        let b = HIST_EDGES.findIndex((edge) => v < edge);
        if (b < 0) b = HIST_EDGES.length - 1;
        buckets[b]++;
    }

    // 5 worst steps by original index
    const worst = timings
        .map((ms, index) => ({ index, ms }))
        .sort((a, b) => b.ms - a.ms)
        .slice(0, 5);

    return {
        count: n,
        mean,
        p50: percentile(sorted, 0.5),
        p90: percentile(sorted, 0.9),
        p99: percentile(sorted, 0.99),
        p999: percentile(sorted, 0.999),
        max: sorted[n - 1] ?? 0,
        buckets,
        worst,
    };
}

// --- report rendering -----------------------------------------------------

function renderHistogram(buckets, count) {
    const maxCount = Math.max(1, ...buckets);
    const rows = [];
    for (let i = 0; i < buckets.length; i++) {
        const lo = i === 0 ? 0 : HIST_EDGES[i - 1];
        const hi = HIST_EDGES[i];
        const label = hi === Infinity ? `>=${lo}ms` : `${lo}-${hi}ms`;
        const c = buckets[i];
        const bar = '█'.repeat(Math.round((c / maxCount) * 40));
        const pct = ((c / Math.max(1, count)) * 100).toFixed(1);
        rows.push(`${label.padStart(10)} | ${String(c).padStart(5)} (${pct.padStart(5)}%) ${bar}`);
    }
    return rows.join('\n');
}

function renderReport(summary) {
    const { git, stats } = summary;
    const out = [];
    out.push(`# hitch report — ${summary.scenario}`);
    out.push('');
    out.push(
        '_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — ' +
            'a hitch IS the metric here, so nothing is filtered._',
    );
    out.push('');
    out.push('| field | value |');
    out.push('| --- | --- |');
    out.push(`| scenario | \`${summary.scenario}\` |`);
    out.push(`| date | ${summary.date} |`);
    out.push(`| git rev | \`${git.rev}\`${git.dirty ? ' (dirty)' : ''} |`);
    out.push(`| node | ${summary.node} |`);
    out.push(`| cpu | ${summary.cpu} |`);
    out.push(`| warmup steps | ${summary.warmupSteps} |`);
    out.push(`| timed steps | ${stats.count} |`);
    out.push('');

    out.push('## step time (ms)');
    out.push('');
    out.push(
        table(
            ['mean', 'p50', 'p90', 'p99', 'p99.9', 'max'],
            ['r', 'r', 'r', 'r', 'r', 'r'],
            [[ms3(stats.mean), ms3(stats.p50), ms3(stats.p90), ms3(stats.p99), ms3(stats.p999), ms3(stats.max)]],
        ),
    );
    out.push('');

    out.push('## histogram');
    out.push('');
    out.push('```');
    out.push(renderHistogram(stats.buckets, stats.count));
    out.push('```');
    out.push('');

    out.push('## 5 worst steps');
    out.push('');
    out.push(
        table(
            ['#', 'step index', 'ms'],
            ['r', 'r', 'r'],
            stats.worst.map((w, i) => [String(i + 1), String(w.index), ms3(w.ms)]),
        ),
    );
    out.push('');
    return out.join('\n');
}

// --- main -----------------------------------------------------------------

const argv = process.argv.slice(2);
const scenario = argv.find((a) => !a.startsWith('--'));
const stepsArg = argv.indexOf('--steps');
const steps = stepsArg >= 0 ? Number(argv[stepsArg + 1]) : DEFAULT_STEPS;
if (!scenario) {
    console.error('usage: node hitch.mjs <scenario> [--steps N]');
    process.exit(2);
}

mkdirSync(SCRATCH_DIR, { recursive: true });
const timingsPath = resolve(SCRATCH_DIR, `hitch-${scenario}.timings.json`);
const runner = resolve(BENCH_DIR, 'hitch-run.ts');

const r = spawnSync('node', ['--import', 'tsx', runner, scenario, timingsPath, String(steps)], {
    stdio: 'inherit',
    cwd: BENCH_DIR,
});
if (r.status !== 0) {
    console.error(`hitch run for ${scenario} exited ${r.status}`);
    process.exit(1);
}

let raw;
try {
    raw = JSON.parse(readFileSync(timingsPath, 'utf8'));
} catch (e) {
    console.error(`could not read timings: ${e.message}`);
    process.exit(1);
}

const stats = analyze(raw.timings);
const summary = {
    scenario,
    date: new Date().toISOString(),
    git: gitRev(),
    node: process.version,
    cpu: cpus()[0]?.model ?? 'unknown',
    warmupSteps: raw.warmupSteps,
    stats: {
        count: stats.count,
        mean: Number(stats.mean.toFixed(4)),
        p50: Number(stats.p50.toFixed(4)),
        p90: Number(stats.p90.toFixed(4)),
        p99: Number(stats.p99.toFixed(4)),
        p999: Number(stats.p999.toFixed(4)),
        max: Number(stats.max.toFixed(4)),
        histogram: HIST_EDGES.map((hi, i) => ({
            loMs: i === 0 ? 0 : HIST_EDGES[i - 1],
            hiMs: hi === Infinity ? null : hi,
            count: stats.buckets[i],
        })),
        worst: stats.worst.map((w) => ({ index: w.index, ms: Number(w.ms.toFixed(4)) })),
    },
};

const report = renderReport({ ...summary, stats });

mkdirSync(REPORTS_DIR, { recursive: true });
const mdPath = resolve(REPORTS_DIR, `hitch-${scenario}.md`);
const jsonPath = resolve(REPORTS_DIR, `hitch-${scenario}.json`);
writeFileSync(mdPath, `${report}\n`);
writeFileSync(jsonPath, `${JSON.stringify(summary, null, 2)}\n`);

console.log(report);
console.log('');
console.log(`report:  ${mdPath}`);
console.log(`summary: ${jsonPath}`);

rmSync(timingsPath, { force: true });
