// V8 optimization probe — child process invoked by tst/v8/v8-opt.test.ts.
//
// Usage:
//   node --allow-natives-syntax --trace-turbo-inlining harness.mjs <fixture-path>
//
// Loads a fixture (must export { name, fn, exercise }), runs the V8 native
// optimization protocol, and prints a JSON result line to stdout. The parent
// test process parses --trace-turbo-inlining output to determine what got
// inlined into `fn`.
//
// Note: V8's --trace-turbo-inlining writes to **stdout**, not stderr, so the
// JSON result is interleaved with trace lines. The parent splits by line
// and takes the last non-empty one (always JSON).
//
// Output (last stdout line):
//   {"name": "...", "fnName": "...", "statusBits": N,
//    "optimized": bool, "turboFanned": bool, "deoptMarked": bool, ...}

import { pathToFileURL } from 'node:url';
import { resolve } from 'node:path';

const fixturePath = process.argv[2];
if (!fixturePath) {
    process.stderr.write('harness: missing fixture path argv[2]\n');
    process.exit(2);
}

const fixture = await import(pathToFileURL(resolve(fixturePath)).href);
const { name, fn, exercise } = fixture;
if (typeof fn !== 'function' || typeof exercise !== 'function') {
    process.stderr.write(`harness: fixture ${fixturePath} must export { name, fn, exercise }\n`);
    process.exit(2);
}

// V8 native intrinsics. With --allow-natives-syntax, `%` is a real operator;
// `eval` keeps this file parseable without the flag.
const v8 = {
    prepare:  eval('(f) => %PrepareFunctionForOptimization(f)'),
    optimize: eval('(f) => %OptimizeFunctionOnNextCall(f)'),
    status:   eval('(f) => %GetOptimizationStatus(f)'),
};

const TURBOFANNED       = 1 << 6;
const MAGLEV            = 1 << 5;
const OPTIMIZED         = 1 << 4;
const MARKED_FOR_DEOPT  = 1 << 14;

// Protocol mirrors the standalone probe — warm up, request opt, hold under
// load. The hold-under-load phase is what surfaces "optimizes then deopts"
// regressions (e.g. a new megamorphic call site).
for (let i = 0; i < 2000; i++) exercise();
v8.prepare(fn);
exercise();
v8.optimize(fn);
exercise();
for (let i = 0; i < 20000; i++) exercise();

const status = v8.status(fn);
const result = {
    name,
    fnName: fn.name,
    statusBits: status,
    optimized:   (status & OPTIMIZED) !== 0,
    turboFanned: (status & TURBOFANNED) !== 0,
    maglev:      (status & MAGLEV) !== 0,
    deoptMarked: (status & MARKED_FOR_DEOPT) !== 0,
};

// One-line JSON to stdout for the parent. stderr already carries the V8 trace.
process.stdout.write(JSON.stringify(result) + '\n');
