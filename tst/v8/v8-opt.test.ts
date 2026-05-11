// V8 optimization + inlining regression tests.
//
// Each test spawns a child `node --allow-natives-syntax --trace-turbo-inlining`
// running tst/v8/harness.mjs against a fixture. The child returns a JSON
// payload on stdout (optimization status) and emits TurboFan inlining trace
// lines on stderr. The test parses both.
//
// This suite is **not** part of `pnpm test`. It is excluded by
// vitest.config.ts and runs only under `pnpm test:v8`. Reasons:
//   - --allow-natives-syntax is a node-process flag, not a vitest one.
//   - Inlining heuristics are V8-version-coupled. CI should pin the Node
//     version when running this suite, or treat failures as "look at the
//     trace before accepting it as a regression".
//
// Assertions are intentionally **tight**: we name the specific helpers that
// MUST get inlined into the caller, because losing any of them is a
// meaningful perf regression. If V8 changes its heuristics in a future
// release, the trace tells us exactly what to expect and we update the
// allow-list deliberately.

import { spawnSync } from 'node:child_process';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';

type ProbeResult = {
    name: string;
    fnName: string;
    statusBits: number;
    optimized: boolean;
    turboFanned: boolean;
    maglev: boolean;
    deoptMarked: boolean;
};

function runProbe(fixture: string): { result: ProbeResult; trace: string } {
    const fixturePath = resolve(__dirname, 'fixtures', `${fixture}.mjs`);
    const harnessPath = resolve(__dirname, 'harness.mjs');

    const r = spawnSync(
        'node',
        ['--allow-natives-syntax', '--trace-turbo-inlining', harnessPath, fixturePath],
        { encoding: 'utf8' },
    );
    if (r.status !== 0) {
        throw new Error(
            `harness exited ${r.status}\nstdout: ${r.stdout}\nstderr: ${r.stderr}`,
        );
    }

    // V8's --trace-turbo-inlining writes to **stdout** (not stderr), so the
    // harness JSON and the trace are interleaved on the same stream. The
    // harness guarantees its JSON is the last non-empty line.
    const lines = r.stdout.split('\n').filter(Boolean);
    const lastLine = lines[lines.length - 1];
    if (!lastLine) throw new Error(`harness produced no stdout. stderr: ${r.stderr}`);
    if (!lastLine.startsWith('{')) {
        throw new Error(`expected JSON last line, got: ${lastLine}`);
    }

    const trace = lines.slice(0, -1).join('\n');
    return { result: JSON.parse(lastLine) as ProbeResult, trace };
}

// Parses lines of the form:
//   `Inlining 0xADDR {0xADDR <SharedFunctionInfo NAME>} into 0xADDR {0xADDR <SharedFunctionInfo CALLER>}`
// Returns the set of callee names inlined into `caller`.
function inlinesInto(trace: string, caller: string): Set<string> {
    // Caller names like `castRay$2` contain `$N` which RegExp treats as a
    // backreference — escape regex metacharacters before injection.
    const escaped = caller.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
    const re = new RegExp(
        `Inlining \\S+ \\{\\S+ <SharedFunctionInfo (\\S+)>\\} into \\S+ \\{\\S+ <SharedFunctionInfo ${escaped}>\\}`,
        'g',
    );
    const out = new Set<string>();
    for (const m of trace.matchAll(re)) out.add(m[1]);
    return out;
}

describe('V8 optimization — dbvt.castRay', () => {
    it('TurboFans and stays optimized under hot-load', () => {
        const { result } = runProbe('dbvt-cast-ray');
        expect(result.optimized, `status bits: ${result.statusBits}`).toBe(true);
        expect(result.turboFanned, `status bits: ${result.statusBits}`).toBe(true);
        expect(result.deoptMarked, `status bits: ${result.statusBits}`).toBe(false);
    });

    it('inlines its critical inner helpers', () => {
        const { result, trace } = runProbe('dbvt-cast-ray');
        const inlined = inlinesInto(trace, result.fnName);

        // Tight contract: each of these helpers is small + hot + monomorphic
        // and MUST inline into castRay for the function to hit its expected
        // perf. Losing any one means a measurable regression.
        //
        // Note: `rayHitsBox3` is absent here — compilecat already inlines it
        // at build time, so V8 never sees it as a callable. If that changes
        // (compilecat stops inlining it), it should reappear here.
        //
        // Known gap: `rayDistanceToBox3` is currently *not* inlined by V8
        // (trace says "Cannot consider, reason: 5" = kCannotInlineCandidate).
        // Tracked separately — do not add it to this list until that's fixed.
        const required = [
            'shouldPairCollide',    // filter.shouldPairCollide — bit-mask compare
            'filterObjectLayer',    // filter.filterObjectLayer — bit-mask compare
            'reset',                // bvhStack.reset
        ];

        for (const name of required) {
            expect(
                inlined.has(name),
                `expected ${name} to be inlined into ${result.fnName}. ` +
                `actually inlined: [${[...inlined].sort().join(', ')}]`,
            ).toBe(true);
        }
    });
});
