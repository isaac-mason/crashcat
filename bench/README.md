# bench — quickstart for future-me

Two distinct tools live here. They consume the same scenarios but answer different questions.

## TL;DR

```bash
pnpm build                              # MUST run first — both rigs resolve `crashcat` to dist/
pnpm --filter bench bench               # macro perf numbers (avg, p75, p99)
pnpm --filter bench perf:profile cube-heap   # cpu-prof hotspot attribution
```

`bench/` is its own workspace package; it depends on `crashcat: workspace:^` and imports straight from the package name. The package entry points still resolve into `dist/`, so a stale `dist/` gives stale answers. Always `pnpm build` at the repo root after touching `src/` or after updating the linked `compilecat`.

If you see a runtime error like `Cannot read properties of undefined (...)`, the build is most likely fine — a bench rig or scenario is calling the API wrong. Fix the rig or the scenario, not the library, until you can reproduce the same error from `pnpm test`.

## What each tool answers

| Question | Tool | Output |
| --- | --- | --- |
| "Is the steady-state hot loop getting faster / slower?" | `pnpm --filter bench bench` | avg / p75 / p99 per bench, written to `.labs/` |
| "Which functions are eating the CPU right now?" | `pnpm --filter bench perf:profile cube-heap` | top-N hotspots + category roll-up by `src/` subfolder |

`labs` answers *how much*. `perf:profile` answers *where*. Use both — neither replaces the other.

## How the bench is structured

- `cube-heap.bench.ts` — 200 dynamic boxes on a plane with deterministic respawn churn. Exercises broadphase under motion.
- `pyramid.bench.ts` — 385 dynamic boxes stacked in a height-10 pyramid, `allowSleeping: false`. Exercises the solver + contact cache under dense persistent stacking.
- Each `.bench.ts` is all-in-one: world setup, populate, step loop, the `bench()` registration, and an exported `runForProfiling()` for the cpu-prof rig. labs handles warm-up/sampling.
- `run-scenario.ts` — thin tsx entry invoked by `profile.mjs` under `node --cpu-prof`. Reads `argv[2]` and dynamically imports `<name>.bench.ts`, calling its `runForProfiling()`.
- `profile.mjs` — spawns the cpu-prof run, parses the `.cpuprofile`, prints hotspots. `--keep` retains the raw profile under `.profiles/`.

RNG is mulberry32 seeded from `RNG_SEED` — labs' adaptive sampling needs a stationary signal, so don't introduce `Math.random()` into a scenario.

## Common gotchas (so I stop tripping on them)

- **Forgot to rebuild.** Bench numbers don't reflect your change because `dist/` is stale. `pnpm build` first.
- **Comparing across machines or thermal states.** `labs` prints CPU stability per run; if the green check goes red, the numbers aren't comparable.
- **Adding work to `cold` thinking it measures hot-loop.** `cold` includes world construction + populate. If you want hot-loop only, look at `steady-state`.
- **`compilecat` linked locally.** `compilecat: "link:/Users/isaacmason/Development/compilecat"` in `package.json` — compilecat changes only take effect after `cd ../compilecat && pnpm build && cd - && pnpm build`.

## Comparing before/after

Worktree the change rather than stashing:

```bash
git worktree add ../crashcat-baseline main
( cd ../crashcat-baseline && pnpm install && pnpm build && pnpm --filter bench bench )
# in the working tree:
pnpm build && pnpm --filter bench bench
```

The labs `.labs/` directory holds the JSON for each run if you want to script a diff later.
