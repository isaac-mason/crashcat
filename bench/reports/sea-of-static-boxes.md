# perf report — sea-of-static-boxes

| field | value |
| --- | --- |
| scenario | `sea-of-static-boxes` |
| date | 2026-07-04T11:44:42.511Z |
| git rev | `942ded3` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1390.9 ms |
| attributed | 1228.9 ms |
| startup excluded | 161 ms |
| samples | 1148 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| broadphase    | 68.8% | 845.3 |
| crashcat-util | 14.1% | 173.5 |
| shapes        |  9.9% | 121.6 |
| bench-harness |  2.9% |  36.2 |
| runtime       |  1.3% |  16.4 |
| solver        |  0.9% |  11.2 |
| body          |  0.6% |   7.0 |
| step          |  0.4% |   4.9 |
| math          |  0.3% |   3.9 |
| ccd           |  0.3% |   3.8 |
| narrowphase   |  0.3% |   3.8 |
| character     |  0.1% |   1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                 | location                                  |
| --: | ----: | ----: | ------------------------ | ----------------------------------------- |
|   1 | 66.4% | 816.2 | `castRay$3`              | `src/broadphase/dbvt.ts:818`              |
|   2 | 13.3% | 163.9 | `findCollidingPairs`     | `src/pairs.ts:412`                        |
|   3 |  9.9% | 121.6 | `castRayVsBox`           | `src/shapes/box.ts:382`                   |
|   4 |  2.9% |  36.2 | (anonymous)              | `bench/sea-of-static-boxes.bench.ts`      |
|   5 |  1.0% |  12.7 | `(garbage collector)`    | ``                                        |
|   6 |  0.9% |  10.9 | `intersectAABBFatLeaves` | `src/broadphase/dbvt.ts:593`              |
|   7 |  0.8% |  10.0 | `prepare`                | `src/islands.ts:64`                       |
|   8 |  0.5% |   5.6 | `insertLeaf`             | `src/broadphase/dbvt.ts:189`              |
|   9 |  0.4% |   4.9 | `updateWorld`            | `src/update.ts:46`                        |
|  10 |  0.3% |   3.8 | `clear`                  | `src/ccd.ts:107`                          |
|  11 |  0.3% |   3.8 | `castRayVsShape`         | `src/collision/narrowphase.ts:24`         |
|  12 |  0.3% |   3.2 | `create$32`              | `src/body/rigid-body.ts:390`              |
|  13 |  0.2% |   2.6 | `transformQuat`          | `…/node_modules/mathcat/dist/vec3.js:567` |
|  14 |  0.2% |   2.6 | `readNodeAabb`           | `src/broadphase/dbvt.ts:1141`             |
|  15 |  0.2% |   2.6 | `linkPairEdge`           | `src/pairs.ts:274`                        |
|  16 |  0.2% |   2.5 | `setRigidBody`           | `src/body/rigid-body.ts:300`              |
|  17 |  0.2% |   2.5 | `findPairRecord`         | `src/pairs.ts:366`                        |
|  18 |  0.2% |   2.5 | `add$1`                  | `src/broadphase/dbvt.ts:332`              |
|  19 |  0.2% |   2.5 | `(idle)`                 | ``                                        |
|  20 |  0.2% |   1.9 | `sPartition`             | `src/broadphase/dbvt.ts:408`              |
|  21 |  0.1% |   1.8 | `buildTree`              | `src/broadphase/dbvt.ts:460`              |
|  22 |  0.1% |   1.3 | `rebuild`                | `src/broadphase/dbvt.ts:542`              |
|  23 |  0.1% |   1.3 | `requestNode`            | `src/broadphase/dbvt.ts:72`               |
|  24 |  0.1% |   1.3 | `expandByMargin`         | `…/node_modules/mathcat/dist/box3.js:214` |
|  25 |  0.1% |   1.3 | `visit`                  | `src/query.ts:29`                         |
|  26 |  0.1% |   1.3 | `updateShape`            | `src/body/rigid-body.ts:587`              |
|  27 |  0.1% |   1.3 | (anonymous)              | `src/character/kcc.ts:1083`               |
|  28 |  0.1% |   1.3 | `registerShapes`         | `src/register.ts:5`                       |
|  29 |  0.1% |   1.3 | `visit`                  | `src/pairs.ts:224`                        |
|  30 |  0.1% |   1.3 | `finalize`               | `src/islands.ts:240`                      |

