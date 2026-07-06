// crashcat focus-map aggregator — where to spend the next optimization attempt.
//
//   node focus.mjs
//       reads every bench/reports/<scenario>.summary.json present and writes
//       bench/reports/focus-map.md (and prints it):
//         (a) a categories × scenarios matrix of self-time percentages (rows =
//             category, columns = scenario, plus a mean column, sorted by mean),
//         (b) a top-15 cross-scenario hotspot table (function+file, summed
//             self-ms across scenarios, and which scenarios it's hot in).
//
// The per-scenario perf reports answer "where does THIS scenario spend time";
// this rolls them up to answer "which subsystem / function dominates across the
// whole suite" — the thing worth optimizing first.
//
// Style-matched to perf-report.mjs (shared table() helper).

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { basename, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const REPORTS_DIR = resolve(__dirname, 'reports');
const TOP_HOTSPOTS = 15;

// --- formatting helper (mirrors perf-report.mjs) --------------------------

function table(headers, aligns, rows) {
    const widths = headers.map((h, i) => Math.max(h.length, ...rows.map((r) => (r[i] ?? '').length), 3));
    const pad = (s, i) => (aligns[i] === 'r' ? s.padStart(widths[i]) : s.padEnd(widths[i]));
    const line = (cells) => `| ${cells.map((c, i) => pad(c, i)).join(' | ')} |`;
    const sep = `| ${widths.map((w, i) => (aligns[i] === 'r' ? `${'-'.repeat(w - 1)}:` : '-'.repeat(w))).join(' | ')} |`;
    return [line(headers), sep, ...rows.map(line)].join('\n');
}

// --- load summaries -------------------------------------------------------

function loadSummaries() {
    let files;
    try {
        files = readdirSync(REPORTS_DIR);
    } catch {
        return [];
    }
    return files
        .filter((f) => f.endsWith('.summary.json'))
        .map((f) => {
            try {
                return {
                    scenario: basename(f, '.summary.json'),
                    data: JSON.parse(readFileSync(resolve(REPORTS_DIR, f), 'utf8')),
                };
            } catch {
                return null;
            }
        })
        .filter((s) => s !== null)
        .sort((a, b) => a.scenario.localeCompare(b.scenario));
}

// --- (a) category × scenario matrix ---------------------------------------

function renderCategoryMatrix(summaries) {
    const scenarios = summaries.map((s) => s.scenario);
    const categories = new Set();
    for (const s of summaries) for (const c of s.data.categories ?? []) categories.add(c.category);

    // pct[category][scenario]
    const pct = new Map();
    for (const cat of categories) pct.set(cat, new Map());
    for (const s of summaries) {
        for (const c of s.data.categories ?? []) pct.get(c.category).set(s.scenario, c.pct);
    }

    const rows = [...categories].map((cat) => {
        const perScenario = scenarios.map((sc) => pct.get(cat).get(sc) ?? 0);
        const mean = perScenario.reduce((a, b) => a + b, 0) / Math.max(1, scenarios.length);
        return { cat, perScenario, mean };
    });
    rows.sort((a, b) => b.mean - a.mean);

    const headers = ['category', ...scenarios, 'mean'];
    const aligns = ['l', ...scenarios.map(() => 'r'), 'r'];
    const body = rows.map((r) => [r.cat, ...r.perScenario.map((v) => (v > 0 ? v.toFixed(1) : '·')), r.mean.toFixed(1)]);
    return table(headers, aligns, body);
}

// --- (b) cross-scenario hotspot roll-up -----------------------------------

function renderHotspotRollup(summaries) {
    // key by function name + file; sum self-ms across scenarios
    const agg = new Map();
    for (const s of summaries) {
        for (const h of s.data.hotspots ?? []) {
            const key = `${h.name}\x00${h.file}`;
            const entry = agg.get(key) ?? { name: h.name, location: h.location ?? h.file, totalMs: 0, scenarios: new Map() };
            entry.totalMs += h.ms;
            entry.scenarios.set(s.scenario, (entry.scenarios.get(s.scenario) ?? 0) + h.ms);
            agg.set(key, entry);
        }
    }

    const rows = [...agg.values()].sort((a, b) => b.totalMs - a.totalMs).slice(0, TOP_HOTSPOTS);

    const body = rows.map((r, i) => {
        // scenarios where this function is hot, heaviest first
        const where = [...r.scenarios.entries()]
            .sort((a, b) => b[1] - a[1])
            .map(([sc, ms]) => `${sc} (${ms.toFixed(0)})`)
            .join(', ');
        return [
            String(i + 1),
            r.name === '(anonymous)' ? '(anonymous)' : `\`${r.name}\``,
            `\`${r.location}\``,
            r.totalMs.toFixed(1),
            where,
        ];
    });
    return table(['#', 'function', 'location', 'Σ self ms', 'hot in (ms)'], ['r', 'l', 'l', 'r', 'l'], body);
}

// --- main -----------------------------------------------------------------

const summaries = loadSummaries();
if (summaries.length === 0) {
    console.error(`no <scenario>.summary.json found in ${REPORTS_DIR} — run perf:report first`);
    process.exit(1);
}

const out = [];
out.push('# focus map');
out.push('');
out.push(`_Cross-scenario roll-up of ${summaries.length} perf summaries: ${summaries.map((s) => s.scenario).join(', ')}._`);
out.push('');
out.push('## category self-time % (rows sorted by mean across scenarios)');
out.push('');
out.push(renderCategoryMatrix(summaries));
out.push('');
out.push(`## top ${TOP_HOTSPOTS} cross-scenario hotspots (summed self-ms)`);
out.push('');
out.push(renderHotspotRollup(summaries));
out.push('');
const report = out.join('\n');

mkdirSync(REPORTS_DIR, { recursive: true });
const outPath = resolve(REPORTS_DIR, 'focus-map.md');
writeFileSync(outPath, `${report}\n`);

console.log(report);
console.log('');
console.log(`focus map: ${outPath}`);
