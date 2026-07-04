# perf report — kcc-mesh

| field | value |
| --- | --- |
| scenario | `kcc-mesh` |
| date | 2026-07-04T08:04:01.571Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1068.8 ms |
| attributed | 870 ms |
| startup excluded | 198.6 ms |
| samples | 932 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 42.9% | 372.9 |
| shapes        | 18.6% | 162.0 |
| math          |  9.7% |  84.7 |
| character     |  9.3% |  81.0 |
| broadphase    |  4.8% |  41.6 |
| runtime       |  3.5% |  30.4 |
| solver        |  2.7% |  23.5 |
| step          |  2.4% |  20.8 |
| bench-harness |  1.8% |  15.6 |
| body          |  1.7% |  14.8 |
| crashcat-util |  1.5% |  13.3 |
| manifold      |  0.8% |   6.7 |
| other         |  0.3% |   2.5 |

## top 30 self-time hotspots

|   # |   pct |   ms | function                             | location                                       |
| --: | ----: | ---: | ------------------------------------ | ---------------------------------------------- |
|   1 | 11.2% | 97.1 | `gjkCastShape`                       | `src/collision/gjk.ts:924`                     |
|   2 |  7.7% | 67.1 | `penetrationCastShape`               | `src/collision/penetration.ts:618`             |
|   3 |  7.5% | 65.0 | `getSupport`                         | `src/collision/support.ts:125`                 |
|   4 |  7.4% | 64.8 | `gjkClosestPoints`                   | `src/collision/gjk.ts:1246`                    |
|   5 |  6.6% | 57.5 | `collideConvexVsTriangleMesh`        | `src/shapes/triangle-mesh.ts:1239`             |
|   6 |  5.5% | 48.0 | `castConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:693`              |
|   7 |  3.9% | 34.3 | `penetrationDepthStepGJK`            | `src/collision/penetration.ts:29`              |
|   8 |  3.2% | 28.0 | `transformMat4$1`                    | `…/node_modules/mathcat/dist/vec3.js:530`      |
|   9 |  3.2% | 27.9 | `rayDistanceToBox3`                  | `src/collision/cast-utils.ts:20`               |
|  10 |  2.8% | 24.2 | `(garbage collector)`                | ``                                             |
|  11 |  2.0% | 17.4 | `castAABB$1`                         | `src/broadphase/dbvt.ts:973`                   |
|  12 |  1.7% | 15.0 | `getTriangleVertices`                | `src/shapes/utils/triangle-mesh-data.ts:44`    |
|  13 |  1.6% | 13.8 | `correctFractionForCharacterPadding` | `src/character/kcc.ts:1374`                    |
|  14 |  1.4% | 12.1 | `updateWorld`                        | `src/update.ts:46`                             |
|  15 |  1.3% | 11.4 | `addHit`                             | `src/character/kcc.ts:914`                     |
|  16 |  1.2% | 10.7 | `intersectAABB$1`                    | `src/broadphase/dbvt.ts:633`                   |
|  17 |  1.2% | 10.1 | `moveShape`                          | `src/character/kcc.ts:2687`                    |
|  18 |  0.9% |  7.7 | `update$11`                          | `src/broadphase/dbvt.ts:480`                   |
|  19 |  0.8% |  7.3 | (anonymous)                          | `bench/kcc-mesh.bench.ts`                      |
|  20 |  0.7% |  6.4 | `solveConstraints`                   | `src/character/kcc.ts:2012`                    |
|  21 |  0.7% |  6.3 | `addContactConstraint`               | `src/constraints/contact-constraints.ts:583`   |
|  22 |  0.7% |  6.3 | `scale$4`                            | `…/node_modules/mathcat/dist/vec3.js:277`      |
|  23 |  0.7% |  6.2 | `updateSupportingContact`            | `src/character/kcc.ts:2294`                    |
|  24 |  0.7% |  6.1 | `bounds$2`                           | `…/node_modules/mathcat/dist/triangle3.js:9`   |
|  25 |  0.7% |  6.0 | `castTransformedVsShape`             | `src/shapes/transformed.ts:557`                |
|  26 |  0.7% |  5.7 | `findCollidingPairs`                 | `src/pairs.ts:409`                             |
|  27 |  0.6% |  5.0 | `setTriangleSupport`                 | `src/collision/support.ts:376`                 |
|  28 |  0.6% |  5.0 | `sortContacts`                       | `src/character/kcc.ts:1253`                    |
|  29 |  0.6% |  4.9 | `normalize$2`                        | `…/node_modules/mathcat/dist/vec3.js:369`      |
|  30 |  0.6% |  4.8 | `buildTriangleMesh`                  | `src/shapes/utils/triangle-mesh-builder.ts:63` |

