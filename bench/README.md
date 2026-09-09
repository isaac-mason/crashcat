# crashcat bench

Macro physics scenarios ported from real physics benchmark suites, run headless through
[`@pmndrs/labs`](https://www.npmjs.com/package/@pmndrs/labs) or watched in a browser with the debug
renderer attached. Both read the same scenario files.

## layout

```
scenarios/          scenario definitions. plain crashcat, no labs and no three
benches/            three lines each, wiring one scenario into labs
viewer/             vite page that runs a scenario with the debug renderer
smoke.ts            one untimed pass over every scenario, for sanity and for sizing steps
```

A scenario is a name, a description, a `steps` count and a `create()` that returns
`{ world, step(stepIndex) }`. `step` advances exactly one simulated step **including any per-step
scenario work** — the wrecking ball pyramid drops its weight on a given step, raycast-mesh fires its
ray fan before stepping, character-terrain drives every character controller. Anything PEEL does in
`CommonUpdate` goes there.

## running headless

The engine is consumed from `dist`, so build first from the repo root:

```
pnpm build
```

Then, in `bench/`:

```
pnpm bench                        # every scenario, saved under the current commit
pnpm bench pyramid                # one scenario
pnpm bench "convex-pile pyramid"  # several
pnpm bench "@physics"             # by tag
pnpm bench -n 'before-my-change'  # save under a name
pnpm bench --no-save              # throwaway run
pnpm bench:list                   # what has been saved
pnpm bench:baseline <name>        # pin a saved run as the baseline
pnpm bench:compare <name>         # compare a saved run against the baseline
```

`pnpm smoke` prints body counts and rough ms/step for every scenario in a single cold pass. It is
not a measurement — it exists to sanity-check a scenario and to size its `steps`.

## running with the renderer

```
pnpm viewer
```

Pick a scenario from the dropdown, or link straight to one with `#pyramid`. Pause, single-step and
restart are there for watching a scenario behave rather than for timing it. Nothing in `viewer/` is
imported by `benches/`, so attaching a renderer cannot change what is measured.

## what a measured op is

One op builds a whole world and runs its `steps`. Building every time is deliberate: labs samples an
op many times, and a physics scenario that kept stepping one long-lived world would get cheaper as
it settled, so every sample would be measuring a different simulation. Rebuilding keeps the workload
stationary, at the cost of folding a fixed construction cost into every sample — which is the right
call for scenarios like sea-of-static-boxes, where the broadphase build *is* the thing under test.

`steps` is sized so an op lands around 100-350ms. That is large for labs, so `labs.config.ts` raises
`blockTime` and lowers `minSamples` to reach the sample floor without a run taking all afternoon.

## the scenarios

Every scenario is a port of a named test from PEEL (Pierre Terdiman's Physics Engine Evaluation Lab)
or from Jolt's `PerformanceTest`, with the source named in the file header along with any deviation.

| scenario | source | loads |
| --- | --- | --- |
| `box-stacks` | PEEL `ManySmallBoxStacks10` | stacking stability, many small islands |
| `pyramid` | Jolt `PyramidScene` | one deep contact island, plus an impact |
| `convex-pile` | PEEL `PileOfMediumConvexes` | GJK/EPA, convex hull support |
| `compound-pile` | PEEL `PileOfSmallCompounds` | compound dispatch, sub-shape ids |
| `ten-thousand-boxes` | PEEL `TenThousandsBoxes` | broadphase and sleeping at 10k bodies |
| `sea-of-static-boxes` | PEEL `SeaOfStaticBoxes` | static broadphase build, idle step cost |
| `convex-vs-mesh` | Jolt `ConvexVsMeshScene` | mesh BVH traversal, per-triangle collide |
| `raycast-mesh` | PEEL raycast-vs-static-mesh | broadphase castRay, castRayVsTriangleMesh |
| `ragdoll-pile` | PEEL `PileOfRagdolls_16` | swing-twist constraints, island building |
| `hinge-chain` | PEEL `HingeJointChain` | long serial constraint chains |
| `ccd-cascade` | PEEL `CCDTest_DynamicDynamic_ConvexCascade` | the linear-cast CCD sweep |
| `character-terrain` | Jolt `CharacterVirtualScene` | KCC shape casts against a mesh |

## adding one

1. Write `scenarios/<name>.ts` exporting a `defineScenario({...})`. Name the suite and test it comes
   from in the file header, and spell out any deviation from it.
2. Add it to `SCENARIOS` in `scenarios/index.ts`.
3. Add `benches/<name>.bench.ts` — copy any existing one, it is three lines.
4. Run `pnpm smoke <name>` and set `steps` so the op lands in the 100-350ms band.

## a note on the machine

Apple Silicon drifts under sustained load: a scenario late in a hot batch can read meaningfully
slower than the same code on a cool machine. labs runs each bench in eight fresh interleaved
processes and judges on block medians precisely to absorb this, and it flags a run whose clock
probes vary too much. Believe a flagged verdict only after re-running.
