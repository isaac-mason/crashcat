# perf report — kcc-mesh

| field | value |
| --- | --- |
| scenario | `kcc-mesh` |
| date | 2026-07-03T05:08:09.291Z |
| git rev | `ab84e99` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1300.1 ms |
| attributed | 1060.4 ms |
| startup excluded | 239 ms |
| samples | 1118 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 34.7% | 367.7 |
| shapes        | 19.9% | 211.1 |
| character     | 17.1% | 181.2 |
| math          |  9.3% |  98.7 |
| broadphase    |  4.1% |  43.7 |
| step          |  3.3% |  34.5 |
| solver        |  3.1% |  33.0 |
| runtime       |  2.8% |  29.3 |
| crashcat-util |  2.2% |  23.3 |
| body          |  1.6% |  16.8 |
| bench-harness |  0.9% |   9.8 |
| manifold      |  0.6% |   6.2 |
| ccd           |  0.3% |   2.7 |
| other         |  0.2% |   2.5 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                             | location                                      |
| --: | ---: | ---: | ------------------------------------ | --------------------------------------------- |
|   1 | 8.5% | 90.1 | `collideConvexVsTriangleMesh`        | `src/shapes/triangle-mesh.ts:1211`            |
|   2 | 7.9% | 83.9 | `gjkClosestPoints`                   | `src/collision/gjk.ts:1225`                   |
|   3 | 7.2% | 76.0 | `gjkCastShape`                       | `src/collision/gjk.ts:913`                    |
|   4 | 6.1% | 65.2 | `getSupport`                         | `src/collision/support.ts:125`                |
|   5 | 5.8% | 61.2 | `penetrationCastShape`               | `src/collision/penetration.ts:621`            |
|   6 | 4.6% | 49.3 | `finalizeContactTracking`            | `src/character/kcc.ts:2949`                   |
|   7 | 3.8% | 40.0 | `castConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:677`             |
|   8 | 3.4% | 36.1 | `penetrationDepthStepGJK`            | `src/collision/penetration.ts:29`             |
|   9 | 2.8% | 29.2 | `transformMat4$1`                    | `…/node_modules/mathcat/dist/vec3.js:530`     |
|  10 | 2.3% | 24.3 | `rayDistanceToBox3`                  | `src/collision/cast-utils.ts:20`              |
|  11 | 2.0% | 21.5 | `solveConstraints`                   | `src/character/kcc.ts:1991`                   |
|  12 | 2.0% | 21.2 | `calculateTriangleAABB`              | `src/shapes/utils/triangle-mesh-data.ts:93`   |
|  13 | 1.9% | 20.6 | `update`                             | `src/character/kcc.ts:3756`                   |
|  14 | 1.8% | 18.9 | `(garbage collector)`                | ``                                            |
|  15 | 1.7% | 18.2 | `updateWorld`                        | `src/update.ts:45`                            |
|  16 | 1.3% | 13.7 | `castAABB$1`                         | `src/broadphase/dbvt.ts:936`                  |
|  17 | 1.3% | 13.7 | `intersectAABB$1`                    | `src/broadphase/dbvt.ts:627`                  |
|  18 | 1.1% | 12.0 | `addHit`                             | `src/character/kcc.ts:903`                    |
|  19 | 1.1% | 11.7 | `correctFractionForCharacterPadding` | `src/character/kcc.ts:1358`                   |
|  20 | 1.0% | 10.2 | `getTriangleVertices`                | `src/shapes/utils/triangle-mesh-data.ts:37`   |
|  21 | 0.8% |  8.9 | `update$1`                           | `src/broadphase/dbvt.ts:485`                  |
|  22 | 0.8% |  8.8 | `collideTransformedVsShape`          | `src/shapes/transformed.ts:377`               |
|  23 | 0.8% |  8.7 | `intersectsBox3`                     | `…/node_modules/mathcat/dist/raycast3.js:195` |
|  24 | 0.8% |  8.6 | `push`                               | `src/utils/bvh-stack.ts:36`                   |
|  25 | 0.8% |  8.6 | `getFirstContactForSweep`            | `src/character/kcc.ts:1429`                   |
|  26 | 0.8% |  8.4 | `solveVelocityConstraintsForIsland`  | `src/constraints/contact-constraints.ts:1121` |
|  27 | 0.8% |  8.1 | `castTransformedVsShape`             | `src/shapes/transformed.ts:553`               |
|  28 | 0.6% |  6.8 | `(program)`                          | ``                                            |
|  29 | 0.6% |  6.8 | `updateSupportingContact`            | `src/character/kcc.ts:2272`                   |
|  30 | 0.6% |  6.5 | `moveShape`                          | `src/character/kcc.ts:2654`                   |

