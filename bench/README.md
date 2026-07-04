# crashcat bench + perf rig

CPU-profile attribution for the macro physics scenarios. The bench imports the
built `dist/index.js` bundle, so the rig decodes `dist/index.js.map` to map every
hotspot back to its `src/**/*.ts` origin.

## workflow

```
build → bench → report → diff
```

> Thermal caveat (Apple Silicon): batch runs drift — a scenario late in a hot batch
> can read ±10% vs the same code on a cool machine (measured 2026-07-04: a "+9.8%
> p<.001" single-scenario regression dissolved to parity under interleaved A/B).
> For a suspicious single-scenario delta, verify with interleaved old/new runs
> (alternate processes importing the two bundles) before believing it.

1. **build** — the rig profiles the built bundle, so build first (from repo root):

   ```
   pnpm build
   ```

2. **bench** — throughput numbers via labs (adaptive sampling):

   ```
   pnpm bench
   ```

3. **report** — CPU-prof attribution → committed markdown + machine-readable json:

   ```
   pnpm perf:report <scenario>            # e.g. cube-heap
   pnpm perf:report <scenario> -- --keep  # also leave the .cpuprofile on disk
   ```

   Writes `bench/reports/<scenario>.md` and `bench/reports/<scenario>.summary.json`,
   and prints the report. The `.md` is the human view (metadata, category
   roll-up, top-30 self-time hotspots with `src/…:line`); the `.summary.json`
   is the diff input.

   For a quick stdout-only dump (no files written) use the legacy wrapper:

   ```
   pnpm perf:profile <scenario> [-- --keep]
   ```

4. **diff** — compare two summaries (before/after a change):

   ```
   pnpm perf:diff bench/reports/before.summary.json bench/reports/after.summary.json
   ```

   Prints and writes `bench/reports/<before>__vs__<after>.diff.md`: per-category
   ms/pct-point/relative deltas, per-function hotspot deltas (matched by
   name + file, including appeared/gone functions), and the total attributed-time
   delta. Deltas below 0.5 ms are hidden as noise.

   Typical before/after loop: `perf:report` on the baseline, copy the summary
   aside (or commit it), make your change, `pnpm build`, `perf:report` again,
   then `perf:diff` the two summaries.

## scenarios

Each scenario is `bench/<name>.bench.ts` exporting `runForProfiling()`, run via
`bench/run-scenario.ts`. Every scenario additionally exports

```
createScenario(): { world: World; warmupSteps: number; stepOnce(stepIndex: number): void }
```

which is the single source of truth for construction + per-step work: `stepOnce`
performs exactly one simulated step (including any per-step scenario work — KCC
input, projectile relaunch, churn spawn/remove, the wake-waves sweeper). Both the
labs bench and `runForProfiling` are expressed in terms of it. `stepIndex` is a
global step counter — the first `warmupSteps` calls are the warmup, and per-op
rng re-seeding happens at the op boundaries inside `stepOnce`, so a labs op
(which replays the same window) and the continuous profiling/hitch run both fall
out of the one implementation with identical measured behaviour.

- **cube-heap** — 200 dynamic boxes on a static plane, one box re-spawned per
  frame to keep contacts churning. Stresses broadphase under motion + the
  contact solver.
- **hull-heap** — same harness with a shared 100-vertex convex hull instead of a
  box. Puts real weight on the support machinery (`getSupport` vertex scan,
  per-pair convex-radius shrink) and EPA.
- **pyramid** — 385 dynamic boxes stacked into a height-10 pyramid, sleeping
  disabled so every step does a full solve. Stresses the solver + contact cache
  under dense persistent stacking.
- **kcc-mesh** — 16 kinematic character controllers roaming a procedural sine/cos
  triangle-mesh terrain (64×64 quads, ~50×50m) with 20 dynamic box props.
  Stresses the KCC shape-cast / collide-shape path against a triangle-mesh BVH.
- **mesh-field** — 200 mixed-shape dynamic bodies (box/sphere/capsule/hull)
  scattered over a big triangle-mesh terrain (128×128 quads, ~200×200m) with
  gentle churn tuned so ~40-60% sleep at steady state. Stresses the sleeping
  system + broadphase coverage across a large static mesh.
- **joints** — 24 ragdoll-ish chains of 6-8 capsules linked with a mix of
  swing-twist / hinge / point constraints, dropped on a plane, sleeping disabled
  and one chain teleported back up every ~2s. Stresses the constraint solver.
- **projectiles-terrain** — ~50 LINEAR_CAST (CCD) projectiles ricocheting around a
  blocky voxel-style triangle-mesh arena at ~25 m/s, relaunched on settle. The
  only scenario that exercises continuous collision detection (swept-AABB
  broadphase + shape-cast vs the terrain BVH every step).
- **scaled-hulls** — one shared ~40-vertex convex hull instanced 150 times at 10
  distinct uniform scales (0.4×–2.5×) via `scaled` shapes, heaped like cube-heap.
  Isolates the scaled-convex-hull support path (before/after oracle for a
  uniform-scale fast path).
- **collapse** — a transient (not steady-state): a 10×6×2 wall of 120 dynamic
  boxes on a static ground, hit at t=0 by a heavy dense sphere. Each op builds a
  fresh world and runs the whole 360-step topple with no warmup. Measures contact
  churn + island merging + mass sleep transitions during a transient.
- **body-churn** — spawn/despawn lifecycle: a seeded emitter spawns 2-3 small
  boxes/step and removes bodies older than 75 steps, holding a steady ~188-body
  population bouncing on a static ground. Measures body create/remove, pair-purge
  cascades, freelist reuse, discovery pressure.
- **wake-waves** — periodic sleep/wake: a settle-sleep-style 8×8 grid of box
  stacks (wide spacing so lanes are isolated) with a kinematic sphere sweeping
  through lanes at constant velocity, waking each lane and letting it re-sleep
  behind. Measures the repeated sleep-transition machinery (contact-chain
  destroy/re-add, island rebuild, wake discovery) rather than static rest.
- **compound-heap** — 60 dynamic compound bodies (3-5 child boxes/spheres:
  hammer / L-bracket / small table) heaped into a static box container,
  cube-heap-style churn. Measures the compound sub-shape narrowphase and the
  multi-contact pair chains it produces.

## hitch — per-step worst-case frame times

Where `perf:report` answers "where does the average step spend its time",
`hitch` answers "how bad is the worst step" — the tail that shows up as a dropped
frame.

Read the distribution (p99/p99.9), not the max: measured attribution (2026-07-04,
hull-heap) showed the extreme outlier steps perform the same simulation work as
median steps with zero GC overlap, arriving in consecutive-step clusters — an
OS/runtime artifact (Apple Silicon core migration / JIT deopt), not the engine.
A real algorithmic hitch shows up as elevated work at the outlier, not just
elevated wall time.

```
pnpm hitch <scenario> [--steps N]      # default N = 1200
```

Builds the scenario via `createScenario()`, runs its warmup, then times each of
N post-warmup steps with `performance.now()` and writes
`bench/reports/hitch-<scenario>.md` + `bench/reports/hitch-<scenario>.json`:
p50/p90/p99/p99.9/max step time, a text histogram, and the 5 worst steps. Raw
timings are kept **exactly as measured, GC pauses included** — a GC pause landing
inside a step is a real hitch, so it is the metric here, not noise to filter.

## focus — cross-scenario focus map

```
pnpm focus
```

Reads every `bench/reports/<scenario>.summary.json` present and writes (and
prints) `bench/reports/focus-map.md`: (a) a categories × scenarios matrix of
self-time percentages (rows = category, columns = scenario, plus a mean column,
sorted by mean) and (b) a top-15 cross-scenario hotspot table (function+file,
summed self-ms across scenarios, and which scenarios it is hot in). This rolls
the per-scenario reports up into "which subsystem / function dominates the whole
suite" — the thing worth optimizing first.

## layout

- `perf-report.mjs` — the `run` / `diff` CLI.
- `profile.mjs` — legacy stdout-only wrapper (same output as before, now over the lib).
- `hitch.mjs` / `hitch-run.ts` — the per-step hitch tool (node CLI + its tsx runner).
- `focus.mjs` — the cross-scenario focus-map aggregator.
- `lib/cpuprofile.mjs` — pure analysis: source-map VLQ decode, self-time
  aggregation, categorization, display helpers.
- `lib/run.mjs` — spawns a scenario under `--cpu-prof` and collects the profile.
- `reports/` — committed baseline reports + summaries.
- `.profiles/` — scratch `.cpuprofile` output; auto-cleaned unless `--keep`.
