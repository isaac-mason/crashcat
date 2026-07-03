# crashcat bench + perf rig

CPU-profile attribution for the macro physics scenarios. The bench imports the
built `dist/index.js` bundle, so the rig decodes `dist/index.js.map` to map every
hotspot back to its `src/**/*.ts` origin.

## workflow

```
build → bench → report → diff
```

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
`bench/run-scenario.ts`.

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

## layout

- `perf-report.mjs` — the `run` / `diff` CLI.
- `profile.mjs` — legacy stdout-only wrapper (same output as before, now over the lib).
- `lib/cpuprofile.mjs` — pure analysis: source-map VLQ decode, self-time
  aggregation, categorization, display helpers.
- `lib/run.mjs` — spawns a scenario under `--cpu-prof` and collects the profile.
- `reports/` — committed baseline reports + summaries.
- `.profiles/` — scratch `.cpuprofile` output; auto-cleaned unless `--keep`.
