# perf report — sea-of-static-boxes

| field | value |
| --- | --- |
| scenario | `sea-of-static-boxes` |
| date | 2026-07-04T10:56:36.913Z |
| git rev | `c7c24fd` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 2848 ms |
| attributed | 2690.2 ms |
| startup excluded | 157 ms |
| samples | 2308 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| broadphase    | 42.4% | 1141.7 |
| narrowphase   | 36.2% |  974.1 |
| shapes        | 11.6% |  312.7 |
| crashcat-util |  6.2% |  168.0 |
| bench-harness |  1.5% |   41.0 |
| runtime       |  0.9% |   25.2 |
| math          |  0.3% |    7.3 |
| solver        |  0.2% |    5.9 |
| step          |  0.2% |    4.9 |
| ccd           |  0.1% |    3.6 |
| body          |  0.1% |    3.3 |
| other         |  0.1% |    2.5 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                 | location                                                        |
| --: | ----: | -----: | ------------------------ | --------------------------------------------------------------- |
|   1 | 41.3% | 1110.4 | `castRay$3`              | `src/broadphase/dbvt.ts:781`                                    |
|   2 | 36.1% |  971.6 | `rayDistanceToBox3`      | `src/collision/cast-utils.ts:20`                                |
|   3 | 11.6% |  312.7 | `castRayVsBox`           | `src/shapes/box.ts:382`                                         |
|   4 |  5.6% |  150.8 | `findCollidingPairs`     | `src/pairs.ts:409`                                              |
|   5 |  1.3% |   33.8 | (anonymous)              | `bench/sea-of-static-boxes.bench.ts`                            |
|   6 |  0.8% |   21.5 | `(garbage collector)`    | ``                                                              |
|   7 |  0.5% |   14.3 | `insertLeaf`             | `src/broadphase/dbvt.ts:132`                                    |
|   8 |  0.3% |    8.1 | `intersectAABBFatLeaves` | `src/broadphase/dbvt.ts:546`                                    |
|   9 |  0.2% |    6.1 | `addPairRecord`          | `src/pairs.ts:248`                                              |
|  10 |  0.2% |    6.0 | `set`                    | `…/node_modules/mathcat/dist/raycast3.js:36`                    |
|  11 |  0.2% |    5.9 | `prepare`                | `src/islands.ts:64`                                             |
|  12 |  0.2% |    4.9 | `updateWorld`            | `src/update.ts:46`                                              |
|  13 |  0.1% |    3.8 | `visit`                  | `src/pairs.ts:224`                                              |
|  14 |  0.1% |    3.7 | `(idle)`                 | ``                                                              |
|  15 |  0.1% |    3.6 | `clear`                  | `src/ccd.ts:107`                                                |
|  16 |  0.1% |    3.3 | `create$32`              | `src/body/rigid-body.ts:390`                                    |
|  17 |  0.1% |    2.9 | `add$1`                  | `src/broadphase/dbvt.ts:284`                                    |
|  18 |  0.1% |    2.5 | `rand01`                 | `bench/sea-of-static-boxes.bench.ts`                            |
|  19 |  0.1% |    2.5 | (anonymous)              | `bench/run-scenario.ts`                                         |
|  20 |  0.1% |    2.5 | `sPartition`             | `src/broadphase/dbvt.ts:360`                                    |
|  21 |  0.1% |    2.5 | (anonymous)              | `file:///Users/isaacmason/Development/crashcat/dist/index.js:1` |
|  22 |  0.1% |    2.3 | `linkPairEdge`           | `src/pairs.ts:274`                                              |
|  23 |  0.1% |    2.1 | `buildField`             | `bench/sea-of-static-boxes.bench.ts`                            |
|  24 |  0.0% |    1.3 | `rebuild`                | `src/broadphase/dbvt.ts:497`                                    |
|  25 |  0.0% |    1.3 | `rayHitsBox3`            | `src/collision/cast-utils.ts:114`                               |
|  26 |  0.0% |    1.3 | `copyCastRayHit`         | `src/collision/cast-ray-vs-shape.ts:33`                         |
|  27 |  0.0% |    1.3 | `castRay`                | `src/query.ts:80`                                               |
|  28 |  0.0% |    1.3 | `isLeaf`                 | `src/broadphase/dbvt.ts:110`                                    |
|  29 |  0.0% |    1.3 | `findPairRecord`         | `src/pairs.ts:366`                                              |
|  30 |  0.0% |    1.3 | `shouldReportPair`       | `src/pairs.ts:152`                                              |

