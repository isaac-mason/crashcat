# perf report — kcc-mesh

| field | value |
| --- | --- |
| scenario | `kcc-mesh` |
| date | 2026-07-03T14:15:21.619Z |
| git rev | `9b65b0d` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1097.3 ms |
| attributed | 907.7 ms |
| startup excluded | 189.1 ms |
| samples | 1027 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 41.0% | 372.2 |
| shapes        | 18.4% | 166.7 |
| math          | 10.1% |  91.9 |
| character     |  9.7% |  87.7 |
| broadphase    |  5.0% |  45.7 |
| solver        |  4.0% |  36.5 |
| runtime       |  3.1% |  28.5 |
| crashcat-util |  2.0% |  18.5 |
| step          |  2.0% |  18.3 |
| body          |  2.0% |  18.0 |
| bench-harness |  1.5% |  13.6 |
| manifold      |  0.8% |   7.7 |
| other         |  0.3% |   2.4 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                             | location                                       |
| --: | ---: | ---: | ------------------------------------ | ---------------------------------------------- |
|   1 | 9.7% | 88.3 | `gjkCastShape`                       | `src/collision/gjk.ts:913`                     |
|   2 | 9.0% | 81.9 | `getSupport`                         | `src/collision/support.ts:125`                 |
|   3 | 8.3% | 75.5 | `collideConvexVsTriangleMesh`        | `src/shapes/triangle-mesh.ts:1209`             |
|   4 | 7.2% | 65.2 | `penetrationCastShape`               | `src/collision/penetration.ts:621`             |
|   5 | 6.6% | 60.0 | `gjkClosestPoints`                   | `src/collision/gjk.ts:1225`                    |
|   6 | 4.5% | 40.5 | `castConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:676`              |
|   7 | 3.5% | 32.0 | `penetrationDepthStepGJK`            | `src/collision/penetration.ts:29`              |
|   8 | 2.6% | 23.4 | `rayDistanceToBox3`                  | `src/collision/cast-utils.ts:20`               |
|   9 | 2.3% | 21.1 | `transformMat4$1`                    | `…/node_modules/mathcat/dist/vec3.js:530`      |
|  10 | 2.2% | 20.0 | `intersectAABB$1`                    | `src/broadphase/dbvt.ts:627`                   |
|  11 | 2.0% | 18.4 | `(garbage collector)`                | ``                                             |
|  12 | 1.6% | 14.2 | `castAABB$1`                         | `src/broadphase/dbvt.ts:936`                   |
|  13 | 1.5% | 13.5 | `getTriangleVertices`                | `src/shapes/utils/triangle-mesh-data.ts:44`    |
|  14 | 1.2% | 10.9 | `addHit`                             | `src/character/kcc.ts:910`                     |
|  15 | 1.0% |  9.1 | `intersectsBox3`                     | `…/node_modules/mathcat/dist/raycast3.js:195`  |
|  16 | 0.9% |  8.5 | `runSim`                             | `bench/kcc-mesh.bench.ts`                      |
|  17 | 0.9% |  8.3 | `updateSupportingContact`            | `src/character/kcc.ts:2290`                    |
|  18 | 0.9% |  8.0 | `updateWorld`                        | `src/update.ts:45`                             |
|  19 | 0.9% |  8.0 | `solveConstraints`                   | `src/character/kcc.ts:2008`                    |
|  20 | 0.9% |  7.9 | `correctFractionForCharacterPadding` | `src/character/kcc.ts:1370`                    |
|  21 | 0.9% |  7.9 | `moveShape`                          | `src/character/kcc.ts:2683`                    |
|  22 | 0.8% |  7.6 | `bounds$2`                           | `…/node_modules/mathcat/dist/triangle3.js:9`   |
|  23 | 0.8% |  7.5 | `cross`                              | `…/node_modules/mathcat/dist/vec3.js:401`      |
|  24 | 0.8% |  7.3 | `buildTriangleMesh`                  | `src/shapes/utils/triangle-mesh-builder.ts:63` |
|  25 | 0.8% |  7.1 | `push`                               | `src/utils/bvh-stack.ts:36`                    |
|  26 | 0.7% |  6.8 | `(idle)`                             | ``                                             |
|  27 | 0.7% |  6.0 | `copy$9`                             | `…/node_modules/mathcat/dist/vec3.js:58`       |
|  28 | 0.6% |  5.9 | `negate`                             | `…/node_modules/mathcat/dist/vec3.js:343`      |
|  29 | 0.6% |  5.6 | `getFirstContactForSweep`            | `src/character/kcc.ts:1441`                    |
|  30 | 0.6% |  5.3 | `update`                             | `src/character/kcc.ts:3784`                    |

