// crashcat perf-report CLI — repeatable markdown perf reports from .cpuprofile.
//
//   node perf-report.mjs run <scenario> [--keep]
//       runs the scenario under --cpu-prof, writes
//       bench/reports/<scenario>.md + bench/reports/<scenario>.summary.json,
//       and prints the report. --keep leaves the .cpuprofile on disk.
//
//   node perf-report.mjs diff <baseline.summary.json> <current.summary.json>
//       prints (and writes bench/reports/<base>__vs__<cur>.diff.md) a markdown
//       diff: per-category deltas, per-function hotspot deltas, total delta.
//
// Machine-readable <scenario>.summary.json feeds the diff mode so before/after
// comparisons don't need the raw profiles kept around.

import { execFileSync } from 'node:child_process';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { cpus } from 'node:os';
import { resolve, dirname, basename } from 'node:path';
import { fileURLToPath } from 'node:url';

import { analyzeProfile, formatLocation, shortenUrl, showLine } from './lib/cpuprofile.mjs';
import { runScenario } from './lib/run.mjs';

const __dirname = dirname(fileURLToPath(import.meta.url));
const REPORTS_DIR = resolve(__dirname, 'reports');

// how many hotspots to persist in the summary — deeper than the 30 rendered so
// diff mode has coverage below the fold.
const SUMMARY_HOTSPOTS = 80;
const TOP_N = 30;
// deltas below this many ms are noise — hidden in diff output.
const NOISE_MS = 0.5;

// --- formatting helpers ---------------------------------------------------

const ms1 = (micros) => (micros / 1000).toFixed(1);
const pct1 = (micros, total) => ((micros / Math.max(1, total)) * 100).toFixed(1);
const signed = (n) => (n >= 0 ? `+${n.toFixed(1)}` : n.toFixed(1));

// render a GitHub-flavored markdown table with padded, aligned columns.
// aligns[i] is 'l' or 'r'. rows are string[][].
function table(headers, aligns, rows) {
    const cols = headers.length;
    const widths = headers.map((h, i) =>
        Math.max(h.length, ...rows.map((r) => (r[i] ?? '').length), 3),
    );
    const pad = (s, i) => (aligns[i] === 'r' ? s.padStart(widths[i]) : s.padEnd(widths[i]));
    const line = (cells) => `| ${cells.map((c, i) => pad(c, i)).join(' | ')} |`;
    const sep = `| ${widths
        .map((w, i) => (aligns[i] === 'r' ? '-'.repeat(w - 1) + ':' : '-'.repeat(w)))
        .join(' | ')} |`;
    return [line(headers), sep, ...rows.map(line)].join('\n');
}

// --- metadata -------------------------------------------------------------

function gitRev() {
    try {
        const rev = execFileSync('git', ['rev-parse', '--short', 'HEAD'], { cwd: __dirname })
            .toString()
            .trim();
        const dirty =
            execFileSync('git', ['status', '--porcelain'], { cwd: __dirname }).toString().trim()
                .length > 0;
        return { rev, dirty };
    } catch {
        return { rev: 'unknown', dirty: false };
    }
}

// --- run mode -------------------------------------------------------------

// convert an analysis + metadata into the machine-readable summary object.
function buildSummary(scenario, analysis, meta) {
    const { hotspots, categories, totalMicros, wallMs, startupMs, sampleCount } = analysis;
    return {
        scenario,
        date: meta.date,
        git: meta.git,
        node: meta.node,
        cpu: meta.cpu,
        wallMs: Number(wallMs.toFixed(1)),
        attributedMs: Number((totalMicros / 1000).toFixed(1)),
        startupExcludedMs: Number((startupMs ?? 0).toFixed(1)),
        sampleCount,
        categories: categories.map((c) => ({
            category: c.category,
            ms: Number(ms1(c.micros)),
            pct: Number(pct1(c.micros, totalMicros)),
        })),
        hotspots: hotspots.slice(0, SUMMARY_HOTSPOTS).map((h) => ({
            name: h.name,
            file: shortenUrl(h.url),
            line: showLine(h) ? h.line + 1 : null,
            location: formatLocation(h),
            ms: Number(ms1(h.micros)),
            pct: Number(pct1(h.micros, totalMicros)),
        })),
    };
}

function renderReport(summary) {
    const { git } = summary;
    const out = [];
    out.push(`# perf report — ${summary.scenario}`);
    out.push('');
    out.push('| field | value |');
    out.push('| --- | --- |');
    out.push(`| scenario | \`${summary.scenario}\` |`);
    out.push(`| date | ${summary.date} |`);
    out.push(`| git rev | \`${git.rev}\`${git.dirty ? ' (dirty)' : ''} |`);
    out.push(`| node | ${summary.node} |`);
    out.push(`| cpu | ${summary.cpu} |`);
    out.push(`| wall | ${summary.wallMs} ms |`);
    out.push(`| attributed | ${summary.attributedMs} ms |`);
    out.push(`| startup excluded | ${summary.startupExcludedMs} ms |`);
    out.push(`| samples | ${summary.sampleCount} |`);
    out.push('');

    out.push('## by category');
    out.push('');
    out.push(
        table(
            ['category', 'pct', 'ms'],
            ['l', 'r', 'r'],
            summary.categories.map((c) => [c.category, `${c.pct.toFixed(1)}%`, c.ms.toFixed(1)]),
        ),
    );
    out.push('');

    out.push(`## top ${TOP_N} self-time hotspots`);
    out.push('');
    out.push(
        table(
            ['#', 'pct', 'ms', 'function', 'location'],
            ['r', 'r', 'r', 'l', 'l'],
            summary.hotspots.slice(0, TOP_N).map((h, i) => [
                String(i + 1),
                `${h.pct.toFixed(1)}%`,
                h.ms.toFixed(1),
                h.name === '(anonymous)' ? '(anonymous)' : `\`${h.name}\``,
                `\`${h.location}\``,
            ]),
        ),
    );
    out.push('');
    return out.join('\n');
}

function cmdRun(argv) {
    const scenario = argv.find((a) => !a.startsWith('--'));
    const keep = argv.includes('--keep');
    if (!scenario) {
        console.error('usage: node perf-report.mjs run <scenario> [--keep]');
        process.exit(2);
    }

    let result;
    try {
        result = runScenario(scenario);
    } catch (e) {
        console.error(e.message);
        process.exit(1);
    }

    const analysis = analyzeProfile(result.profile);
    const meta = {
        date: new Date().toISOString(),
        git: gitRev(),
        node: process.version,
        cpu: cpus()[0]?.model ?? 'unknown',
    };
    const summary = buildSummary(scenario, analysis, meta);
    const report = renderReport(summary);

    mkdirSync(REPORTS_DIR, { recursive: true });
    const mdPath = resolve(REPORTS_DIR, `${scenario}.md`);
    const jsonPath = resolve(REPORTS_DIR, `${scenario}.summary.json`);
    writeFileSync(mdPath, `${report}\n`);
    writeFileSync(jsonPath, `${JSON.stringify(summary, null, 2)}\n`);

    console.log(report);
    console.log('');
    console.log(`report:  ${mdPath}`);
    console.log(`summary: ${jsonPath}`);

    if (keep) {
        console.log(`profile kept at: ${result.profilePath}`);
    } else {
        result.cleanup();
    }
}

// --- diff mode ------------------------------------------------------------

function loadSummary(path) {
    try {
        return JSON.parse(readFileSync(path, 'utf8'));
    } catch (e) {
        console.error(`could not read summary ${path}: ${e.message}`);
        process.exit(1);
    }
}

function renderDiff(base, cur) {
    const out = [];
    out.push(`# perf diff — ${base.scenario} → ${cur.scenario}`);
    out.push('');
    out.push('| side | scenario | git rev | attributed ms | date |');
    out.push('| --- | --- | --- | --- | --- |');
    out.push(
        `| baseline | \`${base.scenario}\` | \`${base.git.rev}\`${base.git.dirty ? ' (dirty)' : ''} | ${base.attributedMs} | ${base.date} |`,
    );
    out.push(
        `| current | \`${cur.scenario}\` | \`${cur.git.rev}\`${cur.git.dirty ? ' (dirty)' : ''} | ${cur.attributedMs} | ${cur.date} |`,
    );
    out.push('');

    const totalDelta = cur.attributedMs - base.attributedMs;
    const totalRel = base.attributedMs > 0 ? (totalDelta / base.attributedMs) * 100 : 0;
    out.push(
        `**total attributed time:** ${base.attributedMs} ms → ${cur.attributedMs} ms  (**${signed(totalDelta)} ms**, ${signed(totalRel)}%)`,
    );
    out.push('');

    // --- categories ---
    const catNames = new Set([
        ...base.categories.map((c) => c.category),
        ...cur.categories.map((c) => c.category),
    ]);
    const baseCat = new Map(base.categories.map((c) => [c.category, c]));
    const curCat = new Map(cur.categories.map((c) => [c.category, c]));
    const catRows = [];
    for (const name of catNames) {
        const b = baseCat.get(name) ?? { ms: 0, pct: 0 };
        const c = curCat.get(name) ?? { ms: 0, pct: 0 };
        const dMs = c.ms - b.ms;
        if (Math.abs(dMs) < NOISE_MS) continue;
        const dPct = c.pct - b.pct;
        const rel = b.ms > 0 ? (dMs / b.ms) * 100 : Infinity;
        catRows.push({
            name,
            baseMs: b.ms,
            curMs: c.ms,
            dMs,
            dPct,
            rel,
        });
    }
    catRows.sort((a, b) => Math.abs(b.dMs) - Math.abs(a.dMs));

    out.push('## category deltas');
    out.push('');
    if (catRows.length === 0) {
        out.push(`_no category moved more than ${NOISE_MS} ms._`);
    } else {
        out.push(
            table(
                ['category', 'base ms', 'cur ms', 'Δ ms', 'Δ pts', 'rel %'],
                ['l', 'r', 'r', 'r', 'r', 'r'],
                catRows.map((r) => [
                    r.name,
                    r.baseMs.toFixed(1),
                    r.curMs.toFixed(1),
                    signed(r.dMs),
                    `${signed(r.dPct)}`,
                    Number.isFinite(r.rel) ? `${signed(r.rel)}%` : 'new',
                ]),
            ),
        );
    }
    out.push('');

    // --- functions (matched by name + file) ---
    const key = (h) => `${h.name}\x00${h.file}`;
    const baseFn = new Map(base.hotspots.map((h) => [key(h), h]));
    const curFn = new Map(cur.hotspots.map((h) => [key(h), h]));
    const allKeys = new Set([...baseFn.keys(), ...curFn.keys()]);
    const fnRows = [];
    for (const k of allKeys) {
        const b = baseFn.get(k);
        const c = curFn.get(k);
        const baseMs = b?.ms ?? 0;
        const curMs = c?.ms ?? 0;
        const dMs = curMs - baseMs;
        if (Math.abs(dMs) < NOISE_MS) continue;
        const meta = c ?? b;
        let status = '';
        if (!b) status = 'appeared';
        else if (!c) status = 'gone';
        fnRows.push({
            name: meta.name,
            location: meta.location,
            baseMs,
            curMs,
            dMs,
            status,
        });
    }
    fnRows.sort((a, b) => Math.abs(b.dMs) - Math.abs(a.dMs));

    out.push('## function hotspot deltas');
    out.push('');
    out.push(`_matched by name + file; deltas below ${NOISE_MS} ms hidden._`);
    out.push('');
    if (fnRows.length === 0) {
        out.push(`_no function moved more than ${NOISE_MS} ms._`);
    } else {
        out.push(
            table(
                ['function', 'location', 'base ms', 'cur ms', 'Δ ms', 'note'],
                ['l', 'l', 'r', 'r', 'r', 'l'],
                fnRows.map((r) => [
                    r.name === '(anonymous)' ? '(anonymous)' : `\`${r.name}\``,
                    `\`${r.location}\``,
                    r.baseMs.toFixed(1),
                    r.curMs.toFixed(1),
                    signed(r.dMs),
                    r.status,
                ]),
            ),
        );
    }
    out.push('');
    return out.join('\n');
}

function cmdDiff(argv) {
    const [basePath, curPath] = argv.filter((a) => !a.startsWith('--'));
    if (!basePath || !curPath) {
        console.error(
            'usage: node perf-report.mjs diff <baseline.summary.json> <current.summary.json>',
        );
        process.exit(2);
    }
    const base = loadSummary(resolve(basePath));
    const cur = loadSummary(resolve(curPath));
    const diff = renderDiff(base, cur);

    mkdirSync(REPORTS_DIR, { recursive: true });
    const name = `${basename(basePath, '.summary.json')}__vs__${basename(curPath, '.summary.json')}.diff.md`;
    const outPath = resolve(REPORTS_DIR, name);
    writeFileSync(outPath, `${diff}\n`);

    console.log(diff);
    console.log('');
    console.log(`diff: ${outPath}`);
}

// --- dispatch -------------------------------------------------------------

const mode = process.argv[2];
const rest = process.argv.slice(3);
if (mode === 'run') {
    cmdRun(rest);
} else if (mode === 'diff') {
    cmdDiff(rest);
} else {
    console.error('usage:');
    console.error('  node perf-report.mjs run <scenario> [--keep]');
    console.error('  node perf-report.mjs diff <baseline.summary.json> <current.summary.json>');
    process.exit(2);
}
