# perf report — kcc-mesh

| field | value |
| --- | --- |
| scenario | `kcc-mesh` |
| date | 2026-07-03T06:14:24.034Z |
| git rev | `ab84e99` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1239.5 ms |
| attributed | 1019.8 ms |
| startup excluded | 219.3 ms |
| samples | 978 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 41.6% | 424.2 |
| shapes        | 21.6% | 219.9 |
| character     | 11.8% | 120.5 |
| math          |  9.5% |  97.1 |
| broadphase    |  3.6% |  37.0 |
| solver        |  3.0% |  30.9 |
| runtime       |  2.9% |  29.5 |
| crashcat-util |  2.2% |  22.3 |
| step          |  1.4% |  14.0 |
| bench-harness |  1.2% |  12.0 |
| body          |  1.0% |  10.5 |
| manifold      |  0.2% |   1.9 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                             | location                                      |
| --: | ---: | ---: | ------------------------------------ | --------------------------------------------- |
|   1 | 9.5% | 96.9 | `collideConvexVsTriangleMesh`        | `src/shapes/triangle-mesh.ts:1211`            |
|   2 | 9.1% | 93.0 | `gjkCastShape`                       | `src/collision/gjk.ts:913`                    |
|   3 | 8.1% | 82.9 | `gjkClosestPoints`                   | `src/collision/gjk.ts:1225`                   |
|   4 | 8.0% | 81.2 | `getSupport`                         | `src/collision/support.ts:125`                |
|   5 | 7.4% | 75.7 | `penetrationCastShape`               | `src/collision/penetration.ts:621`            |
|   6 | 4.3% | 43.4 | `castConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:677`             |
|   7 | 4.2% | 42.9 | `penetrationDepthStepGJK`            | `src/collision/penetration.ts:29`             |
|   8 | 2.6% | 26.7 | `rayDistanceToBox3`                  | `src/collision/cast-utils.ts:20`              |
|   9 | 2.5% | 26.0 | `intersectsBox3`                     | `…/node_modules/mathcat/dist/raycast3.js:195` |
|  10 | 2.1% | 21.9 | `(garbage collector)`                | ``                                            |
|  11 | 2.1% | 20.9 | `correctFractionForCharacterPadding` | `src/character/kcc.ts:1374`                   |
|  12 | 2.0% | 20.8 | `calculateTriangleAABB`              | `src/shapes/utils/triangle-mesh-data.ts:93`   |
|  13 | 1.7% | 16.8 | `transformMat4$1`                    | `…/node_modules/mathcat/dist/vec3.js:530`     |
|  14 | 1.6% | 16.1 | `getTriangleVertices`                | `src/shapes/utils/triangle-mesh-data.ts:37`   |
|  15 | 1.3% | 13.1 | `intersectAABB$1`                    | `src/broadphase/dbvt.ts:627`                  |
|  16 | 1.1% | 11.3 | `push`                               | `src/utils/bvh-stack.ts:36`                   |
|  17 | 1.1% | 11.2 | `addHit`                             | `src/character/kcc.ts:912`                    |
|  18 | 1.1% | 11.2 | `solveConstraints`                   | `src/character/kcc.ts:2012`                   |
|  19 | 1.0% | 10.1 | `castAABB$1`                         | `src/broadphase/dbvt.ts:936`                  |
|  20 | 1.0% | 10.0 | `getContactsAtPosition`              | `src/character/kcc.ts:1272`                   |
|  21 | 0.9% |  9.3 | `solveVelocityConstraintsForIsland`  | `src/constraints/contact-constraints.ts:1121` |
|  22 | 0.9% |  9.0 | `bounds$2`                           | `…/node_modules/mathcat/dist/triangle3.js:9`  |
|  23 | 0.8% |  7.9 | `cancelVelocityTowardsSteepSlopes`   | `src/character/kcc.ts:2564`                   |
|  24 | 0.6% |  6.5 | `determineConstraints`               | `src/character/kcc.ts:1664`                   |
|  25 | 0.6% |  6.3 | `(idle)`                             | ``                                            |
|  26 | 0.6% |  6.3 | `finalize`                           | `src/islands.ts:240`                          |
|  27 | 0.6% |  6.2 | `collideTransformedVsShape`          | `src/shapes/transformed.ts:377`               |
|  28 | 0.6% |  6.0 | `findCollidingPairs`                 | `src/broadphase/broadphase.ts:213`            |
|  29 | 0.6% |  5.7 | `runSim`                             | `bench/kcc-mesh.bench.ts`                     |
|  30 | 0.5% |  5.5 | `subtract$1`                         | `…/node_modules/mathcat/dist/vec3.js:154`     |

