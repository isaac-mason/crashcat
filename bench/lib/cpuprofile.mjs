// Shared .cpuprofile analysis library for the crashcat perf rig.
//
// Pure functions only — no printing, no process control. Consumers:
//   - bench/profile.mjs       (stdout hotspot dump, back-compat)
//   - bench/perf-report.mjs   (markdown + json reports, diffing)
//
// The bench imports the built `dist/index.js` bundle, so every V8 callFrame url
// points at dist. We decode `dist/index.js.map` (standard source-map v3, VLQ
// mappings) and map each frame's (line, column) back to its original src/ file
// so both the hotspot list and the category roll-up speak in source terms.

import { readFileSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

// --- source-map resolution ------------------------------------------------

// decode a v3 "mappings" string.
// returns: array indexed by generated line → sorted array of [genCol, srcIdx, srcLine]
export function decodeVlqMappings(mappings) {
    const B64 = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
    const charToInt = new Map();
    for (let i = 0; i < B64.length; i++) charToInt.set(B64[i], i);

    const lines = [];
    let genCol = 0;
    let srcIdx = 0;
    let srcLine = 0;
    let srcCol = 0;
    let current = [];
    let i = 0;
    const n = mappings.length;
    const values = [0, 0, 0, 0, 0];

    while (i <= n) {
        const ch = i < n ? mappings[i] : ';';
        if (ch === ';' || ch === ',') {
            i++;
            if (ch === ';') {
                lines.push(current);
                current = [];
                genCol = 0;
            }
            continue;
        }
        // decode one segment (1, 4, or 5 VLQ values)
        let field = 0;
        while (i < n) {
            const c = mappings[i];
            if (c === ';' || c === ',') break;
            let value = 0;
            let shift = 0;
            let digit;
            do {
                digit = charToInt.get(mappings[i]);
                i++;
                value += (digit & 31) << shift;
                shift += 5;
            } while (digit & 32);
            const negate = value & 1;
            value >>>= 1;
            values[field++] = negate ? -value : value;
        }
        genCol += values[0];
        if (field >= 4) {
            srcIdx += values[1];
            srcLine += values[2];
            srcCol += values[3];
            current.push([genCol, srcIdx, srcLine]);
        }
    }
    return lines;
}

// A resolver bundles the per-url source-map cache. resolveOriginal maps a
// 0-based (line, column) in generated code to { url, line } in source, or null.
export function createSourceMapResolver() {
    const cache = new Map(); // url → { sources, lines } | null

    function loadSourceMap(url) {
        if (cache.has(url)) return cache.get(url);
        let result = null;
        if (/\.(m?js|cjs)$/.test(url) && url.startsWith('file://')) {
            const jsPath = fileURLToPath(url);
            const mapPath = `${jsPath}.map`;
            try {
                const map = JSON.parse(readFileSync(mapPath, 'utf8'));
                const mapDir = dirname(mapPath);
                const sources = map.sources.map((s) => resolve(mapDir, map.sourceRoot ?? '', s));
                result = { sources, lines: decodeVlqMappings(map.mappings) };
            } catch {
                result = null;
            }
        }
        cache.set(url, result);
        return result;
    }

    function resolveOriginal(url, line, column) {
        const sm = loadSourceMap(url);
        if (!sm) return null;
        const segments = sm.lines[line];
        if (!segments || segments.length === 0) return null;
        // binary search: greatest genCol <= column
        let lo = 0;
        let hi = segments.length - 1;
        let best = segments[0];
        while (lo <= hi) {
            const mid = (lo + hi) >> 1;
            if (segments[mid][0] <= column) {
                best = segments[mid];
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }
        return { url: sm.sources[best[1]], line: best[2] };
    }

    return { resolveOriginal };
}

// --- categorization -------------------------------------------------------

// route by src/ subfolder so the numbers make sense at the lever-picker level
// ("spend the next attempt on narrowphase, not broadphase") rather than
// per-function. The url field carries either a file:// path (source-resolved TS
// files), a tsx virtual path, or a node:* internal.
export function categorize(url) {
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
    if (url.includes('/src/character/')) return 'character';
    if (url.includes('/src/')) return 'crashcat-util';
    if (url.includes('/bench/')) return 'bench-harness';
    return 'other';
}

// --- profile analysis -----------------------------------------------------

// .cpuprofile schema (V8):
//   nodes: [{ id, callFrame: {functionName, url, lineNumber, columnNumber}, hitCount?, children: [id] }]
//   samples: [nodeId, ...]
//   timeDeltas: [microseconds, ...]    (parallel to samples — time attributed to that sample)
//   startTime, endTime: microseconds
//
// Self-time per node = sum of timeDeltas for samples whose nodeId === this.
// We use timeDeltas (not hitCount) for accuracy.

// Aggregate a parsed .cpuprofile into per-function self-time hotspots plus a
// category roll-up. Returns pure data; formatting is the caller's job.
//
//   {
//     hotspots: [{ name, url, line, resolved, micros }],  // sorted desc by micros
//     categories: [{ category, micros }],                 // sorted desc by micros
//     totalMicros, wallMs, sampleCount,
//   }
export function analyzeProfile(profile, resolver = createSourceMapResolver()) {
    const byId = new Map();
    for (const node of profile.nodes) byId.set(node.id, node);

    // exclude process startup: everything sampled before the first scenario-code
    // frame (built bundle or a .bench.ts frame) is module loading / tsx compile.
    // In short scenarios that overhead is 10-20% of samples and pollutes the
    // category shares; it's reported separately as startupMs, not attributed.
    let startIdx = 0;
    for (let i = 0; i < profile.samples.length; i++) {
        const node = byId.get(profile.samples[i]);
        const url = node?.callFrame?.url ?? '';
        if (url.includes('/dist/index.js') || url.endsWith('.bench.ts')) {
            startIdx = i;
            break;
        }
    }
    let startupMicros = 0;
    for (let i = 0; i < startIdx; i++) {
        startupMicros += Math.max(0, profile.timeDeltas[i] ?? 0);
    }

    const selfMicros = new Map(); // nodeId → microseconds
    for (let i = startIdx; i < profile.samples.length; i++) {
        const id = profile.samples[i];
        const dt = profile.timeDeltas[i] ?? 0;
        selfMicros.set(id, (selfMicros.get(id) ?? 0) + Math.max(0, dt));
    }

    // roll up by (functionName, url, line) — anonymous arrow chunks at the same
    // line all attribute together, which is what we want at this resolution.
    const aggregate = new Map(); // key → { name, url, line, resolved, micros }
    for (const [id, micros] of selfMicros) {
        const node = byId.get(id);
        if (!node) continue;
        const cf = node.callFrame;
        const name = cf.functionName || '(anonymous)';
        let url = cf.url || '';
        let line = cf.lineNumber ?? -1;
        let resolved = false;
        const orig = line >= 0 ? resolver.resolveOriginal(url, line, cf.columnNumber ?? 0) : null;
        if (orig) {
            url = orig.url;
            line = orig.line;
            resolved = true;
        }
        const key = `${name}\x00${url}\x00${line}`;
        const entry = aggregate.get(key) ?? { name, url, line, resolved, micros: 0 };
        entry.micros += micros;
        aggregate.set(key, entry);
    }

    const categoryMicros = new Map();
    for (const e of aggregate.values()) {
        const cat = categorize(e.url);
        categoryMicros.set(cat, (categoryMicros.get(cat) ?? 0) + e.micros);
    }

    const totalMicros = [...selfMicros.values()].reduce((a, b) => a + b, 0);
    const wallMs = (profile.endTime - profile.startTime) / 1000;

    const hotspots = [...aggregate.values()]
        .filter((e) => e.micros > 0)
        .sort((a, b) => b.micros - a.micros);

    const categories = [...categoryMicros.entries()]
        .map(([category, micros]) => ({ category, micros }))
        .sort((a, b) => b.micros - a.micros);

    return {
        hotspots,
        categories,
        totalMicros,
        wallMs,
        startupMs: startupMicros / 1000,
        sampleCount: profile.samples.length,
    };
}

// --- display helpers (shared by profile.mjs + perf-report.mjs) ------------

// strip a long file:// url down to the src/… or bench/… tail, or a package tail.
export function shortenUrl(url) {
    if (!url) return '';
    const m = url.match(/\/(src|bench)\/.+$/);
    if (m) return m[0].slice(1); // strip leading slash
    const node = url.match(/^node:.+$/);
    if (node) return url;
    const pkg = url.match(/node_modules\/(?:\.pnpm\/)?([^/]+\/)?([^/]+\/[^?]+)/);
    if (pkg) return `…/${pkg[2]}`;
    return url.slice(-60);
}

// line numbers only mean something for source-map-resolved frames (line maps to
// the original .ts) and plain .js frames (bundle or pure-JS dep). tsx-transpiled
// .ts frames carry generated-JS lines — hide those.
export function showLine(entry) {
    if (entry.line < 0) return false;
    return entry.resolved || /\.(m?js|cjs)(\?|$)/.test(entry.url);
}

// human "src/foo.ts:123" or just "src/foo.ts" when the line is meaningless.
export function formatLocation(entry) {
    const short = shortenUrl(entry.url);
    return showLine(entry) ? `${short}:${entry.line + 1}` : short;
}
