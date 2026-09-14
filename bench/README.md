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
`{ world, reset, step(stepIndex) }`.

`step` advances exactly one simulated step **including any per-step scenario work** — raycast-mesh
fires its ray fan before stepping, character-terrain drives every character controller. Anything
PEEL does in `CommonUpdate` goes there.

`reset` puts the world back where `create()` left it: every non-static body to its recorded
transform, at rest, awake, plus whatever else the scenario carries (pyramid re-launches its wrecking
ball, character-terrain rewinds its roaming rng and character positions).

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

`pnpm smoke` runs six windows of every scenario and prints body counts, rough ms/step and the awake
count per window. It is not a measurement — it exists to verify that a scenario's window actually
repeats, and to size its `steps`.

## running with the renderer

```
pnpm viewer
```

Pick a scenario from the dropdown, or link straight to one with `#pyramid`. Pause, single-step and
restart are there for watching a scenario behave rather than for timing it. Nothing in `viewer/` is
imported by `benches/`, so attaching a renderer cannot change what is measured.

## what a measured op is

One op is `reset()` followed by `steps` steps. The world is built once, outside the timed region.

The reset is not optional bookkeeping, it is what makes the numbers mean anything. labs sizes its
sample count from the first block's timing to fill `blockTime`, and PEEL scenarios settle — so
without a reset, a candidate that is genuinely faster fits more samples into the budget, runs deeper
into the settled tail where steps are cheap, and reads faster than it really is. The bias amplifies
the very effect the bench is trying to measure. Every op has to do the same work.

`reset` cannot rewind the contact cache's warm-start lambdas or the shape of the broadphase tree,
and crashcat exposes no API that would. So the first window off a fresh world is not quite the same
work as the ones after it; it converges instead. Each bench therefore runs `warm()` — a couple of
untimed windows — before yielding, so every measured op starts from the converged state.
`pnpm smoke` is the check on this: it runs six windows per scenario and flags any whose awake body
count is still moving after the warm-up, which means that scenario's `reset` is missing state.

`steps` is sized to cover the part of the scenario worth measuring, which usually puts a window
somewhere between 30 and 250ms. That is large for labs, so `labs.config.ts` raises `blockTime` and
lowers `minSamples` to reach the sample floor without a run taking all afternoon.

Longer is not better. `ccd-cascade` at 400 steps cost 8x as much per step as at 150, because past
about 385 steps the column has finished falling and the window measures a deep pile of interleaved
plates instead of bodies moving fast enough for CCD to matter. Size `steps` to the phase you care
about, not to fill a time budget.

One consequence worth knowing: world construction is no longer measured anywhere. That matters most
for `sea-of-static-boxes`, whose PEEL purpose is partly the static tree build. Its window now
measures the other half — what a step costs when there is nothing to simulate — and labs' heap
column covers the memory question.

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
| `sea-of-static-boxes` | PEEL `SeaOfStaticBoxes` | the cost of stepping a world at rest |
| `convex-vs-mesh` | Jolt `ConvexVsMeshScene` | mesh BVH traversal, per-triangle collide |
| `raycast-mesh` | PEEL raycast-vs-static-mesh | broadphase castRay, castRayVsTriangleMesh |
| `ragdoll-pile` | PEEL `PileOfRagdolls_16` | swing-twist constraints, island building |
| `hinge-chain` | PEEL `HingeJointChain` | long serial constraint chains |
| `ccd-cascade` | PEEL `CCDTest_DynamicDynamic_ConvexCascade` | the linear-cast CCD sweep |
| `character-terrain` | Jolt `CharacterVirtualScene` | KCC shape casts against a mesh |

## adding one

1. Write `scenarios/<name>.ts` exporting a `defineScenario({...})`. Name the suite and test it comes
   from in the file header, and spell out any deviation from it.
2. Give it a `reset`. `captureBodyState`/`restoreBodyState` cover the bodies; anything else the
   scenario holds — an rng, a cursor, a character's heading — has to be rewound by hand.
3. Add it to `SCENARIOS` in `scenarios/index.ts`.
4. Add `benches/<name>.bench.ts` — copy any existing one.
5. Run `pnpm smoke <name>`. If it prints NOT REPEATABLE the reset is incomplete. Then set `steps` to
   cover the phase of the scenario that is worth measuring, and check the ms/step it reports is not
   drifting upward as you extend the window — if it is, the window has run past the interesting part.
