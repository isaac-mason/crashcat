# perf report — sea-of-static-boxes

| field | value |
| --- | --- |
| scenario | `sea-of-static-boxes` |
| date | 2026-07-04T12:04:27.229Z |
| git rev | `229ba6a` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1264 ms |
| attributed | 1057.5 ms |
| startup excluded | 206.3 ms |
| samples | 1026 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| broadphase    | 80.9% | 855.3 |
| shapes        |  9.5% | 100.3 |
| runtime       |  4.0% |  42.0 |
| bench-harness |  2.4% |  24.9 |
| ccd           |  0.9% |   9.6 |
| solver        |  0.7% |   7.5 |
| body          |  0.6% |   6.7 |
| step          |  0.5% |   5.1 |
| math          |  0.2% |   1.8 |
| crashcat-util |  0.2% |   1.7 |
| other         |  0.1% |   1.3 |
| narrowphase   |  0.1% |   1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                             | location                                                        |
| --: | ----: | ----: | ------------------------------------ | --------------------------------------------------------------- |
|   1 | 78.6% | 831.7 | `castRay$3`                          | `src/broadphase/dbvt.ts:822`                                    |
|   2 |  9.5% | 100.3 | `castRayVsBox`                       | `src/shapes/box.ts:382`                                         |
|   3 |  3.6% |  38.3 | `(garbage collector)`                | ``                                                              |
|   4 |  2.0% |  21.1 | (anonymous)                          | `bench/sea-of-static-boxes.bench.ts`                            |
|   5 |  1.0% |  10.7 | `insertLeaf`                         | `src/broadphase/dbvt.ts:189`                                    |
|   6 |  0.9% |   9.6 | `clear`                              | `src/ccd.ts:107`                                                |
|   7 |  0.6% |   6.3 | `prepare`                            | `src/islands.ts:64`                                             |
|   8 |  0.5% |   5.1 | `updateWorld`                        | `src/update.ts:46`                                              |
|   9 |  0.4% |   3.8 | `intersectAABBFatLeaves`             | `src/broadphase/dbvt.ts:595`                                    |
|  10 |  0.4% |   3.7 | `(idle)`                             | ``                                                              |
|  11 |  0.3% |   3.3 | `castRay$2`                          | `src/broadphase/broadphase.ts:112`                              |
|  12 |  0.3% |   3.3 | `sPartition`                         | `src/broadphase/dbvt.ts:408`                                    |
|  13 |  0.2% |   2.6 | `setRigidBody`                       | `src/body/rigid-body.ts:300`                                    |
|  14 |  0.2% |   2.5 | `updateCenterOfMassPosition`         | `src/body/rigid-body.ts:537`                                    |
|  15 |  0.1% |   1.3 | `updateShape`                        | `src/body/rigid-body.ts:587`                                    |
|  16 |  0.1% |   1.3 | (anonymous)                          | `file:///Users/isaacmason/Development/crashcat/dist/index.js:1` |
|  17 |  0.1% |   1.3 | `buildField`                         | `bench/sea-of-static-boxes.bench.ts`                            |
|  18 |  0.1% |   1.3 | `add$3`                              | `…/node_modules/mathcat/dist/vec3.js:126`                       |
|  19 |  0.1% |   1.3 | `findCollidingPairs`                 | `src/pairs.ts:424`                                              |
|  20 |  0.1% |   1.3 | `runForProfiling`                    | `bench/sea-of-static-boxes.bench.ts`                            |
|  21 |  0.1% |   1.3 | `createClosestPointOnTriangleResult` | `src/collision/triangle.ts:35`                                  |
|  22 |  0.1% |   1.3 | `finalize`                           | `src/islands.ts:240`                                            |
|  23 |  0.1% |   1.2 | `rebuild`                            | `src/broadphase/dbvt.ts:542`                                    |
|  24 |  0.1% |   1.2 | `rand01`                             | `bench/sea-of-static-boxes.bench.ts`                            |
|  25 |  0.1% |   0.6 | `add$1`                              | `src/broadphase/dbvt.ts:332`                                    |
|  26 |  0.0% |   0.5 | `transformQuat`                      | `…/node_modules/mathcat/dist/vec3.js:567`                       |
|  27 |  0.0% |   0.5 | `bUnionNodes`                        | `src/broadphase/dbvt.ts:113`                                    |
|  28 |  0.0% |   0.4 | `castRay`                            | `src/query.ts:85`                                               |
|  29 |  0.0% |   0.3 | `create$32`                          | `src/body/rigid-body.ts:390`                                    |
|  30 |  0.0% |   0.1 | `optimize`                           | `src/broadphase/broadphase.ts:82`                               |

