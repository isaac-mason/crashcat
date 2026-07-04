# perf report — sea-of-static-boxes

| field | value |
| --- | --- |
| scenario | `sea-of-static-boxes` |
| date | 2026-07-04T11:03:12.082Z |
| git rev | `3793287` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1926.1 ms |
| attributed | 1766.2 ms |
| startup excluded | 159.3 ms |
| samples | 1578 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| broadphase    | 71.4% | 1261.4 |
| shapes        | 15.4% |  272.0 |
| crashcat-util |  9.4% |  165.5 |
| bench-harness |  1.5% |   25.7 |
| runtime       |  0.9% |   16.5 |
| solver        |  0.4% |    7.5 |
| ccd           |  0.4% |    6.8 |
| body          |  0.3% |    5.8 |
| math          |  0.1% |    2.5 |
| step          |  0.1% |    1.3 |
| other         |  0.1% |    1.2 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                      | location                                                        |
| --: | ----: | -----: | ----------------------------- | --------------------------------------------------------------- |
|   1 | 69.4% | 1225.2 | `castRay$3`                   | `src/broadphase/dbvt.ts:782`                                    |
|   2 | 15.4% |  272.0 | `castRayVsBox`                | `src/shapes/box.ts:382`                                         |
|   3 |  8.6% |  152.7 | `findCollidingPairs`          | `src/pairs.ts:409`                                              |
|   4 |  1.4% |   24.9 | (anonymous)                   | `bench/sea-of-static-boxes.bench.ts`                            |
|   5 |  0.8% |   13.9 | `(garbage collector)`         | ``                                                              |
|   6 |  0.8% |   13.3 | `insertLeaf`                  | `src/broadphase/dbvt.ts:132`                                    |
|   7 |  0.7% |   12.9 | `intersectAABBFatLeaves`      | `src/broadphase/dbvt.ts:546`                                    |
|   8 |  0.4% |    7.5 | `prepare`                     | `src/islands.ts:64`                                             |
|   9 |  0.4% |    6.8 | `clear`                       | `src/ccd.ts:107`                                                |
|  10 |  0.3% |    5.2 | `addPairRecord`               | `src/pairs.ts:248`                                              |
|  11 |  0.2% |    3.6 | `add$1`                       | `src/broadphase/dbvt.ts:284`                                    |
|  12 |  0.1% |    2.5 | `findPairRecord`              | `src/pairs.ts:366`                                              |
|  13 |  0.1% |    2.5 | `(idle)`                      | ``                                                              |
|  14 |  0.1% |    2.5 | `sPartition`                  | `src/broadphase/dbvt.ts:360`                                    |
|  15 |  0.1% |    2.0 | `create$37`                   | `src/body/motion-properties.ts:71`                              |
|  16 |  0.1% |    1.5 | `rebuild`                     | `src/broadphase/dbvt.ts:497`                                    |
|  17 |  0.1% |    1.3 | `castRay$2`                   | `src/broadphase/broadphase.ts:112`                              |
|  18 |  0.1% |    1.3 | `registerShapes`              | `src/register.ts:5`                                             |
|  19 |  0.1% |    1.3 | `expandByMargin`              | `…/node_modules/mathcat/dist/box3.js:214`                       |
|  20 |  0.1% |    1.3 | `updateWorld`                 | `src/update.ts:46`                                              |
|  21 |  0.1% |    1.3 | `visit`                       | `src/query.ts:28`                                               |
|  22 |  0.1% |    1.3 | `setMassAndInertiaOfSolidBox` | `src/body/mass-properties.ts:36`                                |
|  23 |  0.1% |    1.3 | `updateAABB`                  | `src/body/rigid-body.ts:580`                                    |
|  24 |  0.1% |    1.3 | `linkPairEdge`                | `src/pairs.ts:274`                                              |
|  25 |  0.1% |    1.2 | (anonymous)                   | `file:///Users/isaacmason/Development/crashcat/dist/index.js:1` |
|  26 |  0.1% |    1.2 | `create$32`                   | `src/body/rigid-body.ts:390`                                    |
|  27 |  0.1% |    1.1 | `buildTree`                   | `src/broadphase/dbvt.ts:412`                                    |
|  28 |  0.0% |    0.8 | `buildField`                  | `bench/sea-of-static-boxes.bench.ts`                            |
|  29 |  0.0% |    0.8 | `transformQuat`               | `…/node_modules/mathcat/dist/vec3.js:567`                       |
|  30 |  0.0% |    0.8 | `castRay`                     | `src/query.ts:80`                                               |

