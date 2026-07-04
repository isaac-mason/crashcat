# perf report — sea-of-static-boxes

| field | value |
| --- | --- |
| scenario | `sea-of-static-boxes` |
| date | 2026-07-04T11:28:19.113Z |
| git rev | `685bdda` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1861.8 ms |
| attributed | 1677.5 ms |
| startup excluded | 183.2 ms |
| samples | 1525 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| broadphase    | 70.6% | 1183.8 |
| crashcat-util | 12.2% |  204.0 |
| shapes        | 11.0% |  184.3 |
| bench-harness |  2.3% |   38.6 |
| runtime       |  1.2% |   20.3 |
| solver        |  0.9% |   14.9 |
| ccd           |  0.5% |    8.2 |
| step          |  0.4% |    6.4 |
| body          |  0.3% |    5.8 |
| math          |  0.3% |    5.8 |
| narrowphase   |  0.2% |    4.2 |
| other         |  0.1% |    1.3 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                       | location                                                        |
| --: | ----: | -----: | ------------------------------ | --------------------------------------------------------------- |
|   1 | 68.4% | 1147.0 | `castRay$3`                    | `src/broadphase/dbvt.ts:782`                                    |
|   2 | 11.3% |  188.8 | `findCollidingPairs`           | `src/pairs.ts:409`                                              |
|   3 | 11.0% |  184.3 | `castRayVsBox`                 | `src/shapes/box.ts:382`                                         |
|   4 |  2.2% |   36.9 | (anonymous)                    | `bench/sea-of-static-boxes.bench.ts`                            |
|   5 |  1.0% |   17.1 | `insertLeaf`                   | `src/broadphase/dbvt.ts:132`                                    |
|   6 |  1.0% |   16.4 | `(garbage collector)`          | ``                                                              |
|   7 |  0.8% |   13.6 | `prepare`                      | `src/islands.ts:64`                                             |
|   8 |  0.8% |   13.0 | `intersectAABBFatLeaves`       | `src/broadphase/dbvt.ts:546`                                    |
|   9 |  0.5% |    8.2 | `clear`                        | `src/ccd.ts:107`                                                |
|  10 |  0.4% |    6.4 | `updateWorld`                  | `src/update.ts:46`                                              |
|  11 |  0.3% |    4.5 | `create$32`                    | `src/body/rigid-body.ts:390`                                    |
|  12 |  0.2% |    3.9 | `castRayVsShape`               | `src/collision/narrowphase.ts:24`                               |
|  13 |  0.2% |    3.8 | `findPairRecord`               | `src/pairs.ts:366`                                              |
|  14 |  0.2% |    3.8 | `addPairRecord`                | `src/pairs.ts:248`                                              |
|  15 |  0.2% |    3.5 | `sPartition`                   | `src/broadphase/dbvt.ts:360`                                    |
|  16 |  0.2% |    2.6 | `(idle)`                       | ``                                                              |
|  17 |  0.1% |    2.5 | `set`                          | `src/query.ts:66`                                               |
|  18 |  0.1% |    2.2 | `expandByMargin`               | `…/node_modules/mathcat/dist/box3.js:214`                       |
|  19 |  0.1% |    1.7 | `buildField`                   | `bench/sea-of-static-boxes.bench.ts`                            |
|  20 |  0.1% |    1.6 | `add$1`                        | `src/broadphase/dbvt.ts:284`                                    |
|  21 |  0.1% |    1.3 | `create$48`                    | `…/node_modules/mathcat/dist/vec3.js:8`                         |
|  22 |  0.1% |    1.3 | `set$8`                        | `…/node_modules/mathcat/dist/vec3.js:73`                        |
|  23 |  0.1% |    1.3 | `visit`                        | `src/pairs.ts:224`                                              |
|  24 |  0.1% |    1.3 | `visit`                        | `src/query.ts:29`                                               |
|  25 |  0.1% |    1.3 | `warmStartVelocityConstraints` | `src/constraints/contact-constraints.ts:964`                    |
|  26 |  0.1% |    1.3 | `emit`                         | `node:events`                                                   |
|  27 |  0.1% |    1.3 | (anonymous)                    | `file:///Users/isaacmason/Development/crashcat/dist/index.js:1` |
|  28 |  0.1% |    1.3 | `registerShapes`               | `src/register.ts:5`                                             |
|  29 |  0.1% |    1.3 | `setMassAndInertiaOfSolidBox`  | `src/body/mass-properties.ts:36`                                |
|  30 |  0.1% |    1.3 | `linkPairEdge`                 | `src/pairs.ts:274`                                              |

