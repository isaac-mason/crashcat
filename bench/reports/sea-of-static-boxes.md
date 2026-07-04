# perf report — sea-of-static-boxes

| field | value |
| --- | --- |
| scenario | `sea-of-static-boxes` |
| date | 2026-07-04T10:41:56.254Z |
| git rev | `845e848` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 4098.8 ms |
| attributed | 3916.6 ms |
| startup excluded | 181.8 ms |
| samples | 3284 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| narrowphase   | 51.0% | 1997.0 |
| broadphase    | 30.9% | 1210.2 |
| shapes        |  9.9% |  389.2 |
| crashcat-util |  5.1% |  199.6 |
| bench-harness |  1.2% |   47.2 |
| ccd           |  0.5% |   21.4 |
| runtime       |  0.5% |   21.2 |
| solver        |  0.3% |   11.0 |
| math          |  0.3% |   10.4 |
| step          |  0.1% |    4.3 |
| other         |  0.1% |    2.5 |
| body          |  0.1% |    2.5 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                 | location                                                        |
| --: | ----: | -----: | ------------------------ | --------------------------------------------------------------- |
|   1 | 30.1% | 1178.0 | `castRay$3`              | `src/broadphase/dbvt.ts:781`                                    |
|   2 | 24.7% |  967.4 | `rayDistanceToBox3`      | `src/collision/cast-utils.ts:20`                                |
|   3 | 19.6% |  765.9 | `gjkCastRay`             | `src/collision/gjk.ts:729`                                      |
|   4 |  9.9% |  388.0 | `castRayVsConvex`        | `src/shapes/convex.ts:43`                                       |
|   5 |  4.9% |  191.0 | `findCollidingPairs`     | `src/pairs.ts:409`                                              |
|   6 |  4.5% |  177.0 | `getSupport`             | `src/collision/support.ts:139`                                  |
|   7 |  1.8% |   69.0 | `setBoxSupport`          | `src/collision/support.ts:353`                                  |
|   8 |  1.1% |   43.0 | (anonymous)              | `bench/sea-of-static-boxes.bench.ts`                            |
|   9 |  0.5% |   21.4 | `clear`                  | `src/ccd.ts:107`                                                |
|  10 |  0.4% |   14.9 | `(garbage collector)`    | ``                                                              |
|  11 |  0.4% |   13.9 | `addHit`                 | `src/collision/cast-ray-vs-shape.ts:131`                        |
|  12 |  0.3% |   13.5 | `intersectAABBFatLeaves` | `src/broadphase/dbvt.ts:546`                                    |
|  13 |  0.3% |   11.0 | `prepare`                | `src/islands.ts:64`                                             |
|  14 |  0.3% |    9.9 | `insertLeaf`             | `src/broadphase/dbvt.ts:132`                                    |
|  15 |  0.2% |    6.3 | `set`                    | `…/node_modules/mathcat/dist/raycast3.js:36`                    |
|  16 |  0.1% |    4.3 | `updateWorld`            | `src/update.ts:46`                                              |
|  17 |  0.1% |    3.8 | `(idle)`                 | ``                                                              |
|  18 |  0.1% |    3.7 | `add$1`                  | `src/broadphase/dbvt.ts:284`                                    |
|  19 |  0.1% |    2.5 | `rayHitsBox3`            | `src/collision/cast-utils.ts:114`                               |
|  20 |  0.1% |    2.5 | `linkPairEdge`           | `src/pairs.ts:274`                                              |
|  21 |  0.1% |    2.2 | `sPartition`             | `src/broadphase/dbvt.ts:360`                                    |
|  22 |  0.0% |    1.6 | `buildField`             | `bench/sea-of-static-boxes.bench.ts`                            |
|  23 |  0.0% |    1.3 | `Event`                  | `node:internal/event_target`                                    |
|  24 |  0.0% |    1.3 | `__exportAll`            | `file:///Users/isaacmason/Development/crashcat/dist/index.js:3` |
|  25 |  0.0% |    1.3 | `create$32`              | `src/body/rigid-body.ts:390`                                    |
|  26 |  0.0% |    1.3 | `addPairRecord`          | `src/pairs.ts:248`                                              |
|  27 |  0.0% |    1.3 | (anonymous)              | `file:///Users/isaacmason/Development/crashcat/dist/index.js:1` |
|  28 |  0.0% |    1.3 | `create$41`              | `…/node_modules/mathcat/dist/box3.js:8`                         |
|  29 |  0.0% |    1.3 | `makeRigidBody`          | `src/body/rigid-body.ts:255`                                    |
|  30 |  0.0% |    1.3 | `create$13`              | `src/shapes/box.ts:48`                                          |

