# perf report — projectiles-terrain

| field | value |
| --- | --- |
| scenario | `projectiles-terrain` |
| date | 2026-07-03T05:02:23.025Z |
| git rev | `ab84e99` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1375.4 ms |
| attributed | 1374.3 ms |
| samples | 1017 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 21.9% | 300.7 |
| runtime       | 19.3% | 264.7 |
| shapes        | 17.9% | 246.2 |
| step          | 10.6% | 145.0 |
| broadphase    |  9.9% | 135.9 |
| solver        |  9.0% | 123.8 |
| math          |  6.6% |  90.5 |
| body          |  1.9% |  25.9 |
| crashcat-util |  1.1% |  14.8 |
| manifold      |  1.0% |  14.0 |
| bench-harness |  0.8% |  10.3 |
| ccd           |  0.2% |   2.5 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                            | location                                      |
| --: | ---: | ---: | ----------------------------------- | --------------------------------------------- |
|   1 | 6.4% | 88.4 | `castSphereVsTriangleMesh`          | `src/shapes/triangle-mesh.ts:2177`            |
|   2 | 5.4% | 74.9 | `(idle)`                            | ``                                            |
|   3 | 3.7% | 51.3 | `intersectAABB$1`                   | `src/broadphase/dbvt.ts:627`                  |
|   4 | 3.7% | 50.2 | `rayDistanceToBox3`                 | `src/collision/cast-utils.ts:20`              |
|   5 | 3.6% | 48.9 | `castAABB$1`                        | `src/broadphase/dbvt.ts:936`                  |
|   6 | 3.3% | 46.0 | `collideConvexVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1211`            |
|   7 | 3.2% | 43.3 | `gjkCastShape`                      | `src/collision/gjk.ts:913`                    |
|   8 | 2.8% | 38.7 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1121` |
|   9 | 2.6% | 35.5 | `waitForWorker`                     | `node:internal/modules/esm/hooks`             |
|  10 | 2.6% | 35.4 | `makeSyncRequest`                   | `node:internal/modules/esm/hooks`             |
|  11 | 2.5% | 33.8 | `collideSphereVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1725`            |
|  12 | 2.4% | 33.5 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1225`                   |
|  13 | 2.4% | 33.1 | `findCCDContacts`                   | `src/update.ts:1576`                          |
|  14 | 2.2% | 30.7 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`             |
|  15 | 2.1% | 29.5 | `penetrationCastShape`              | `src/collision/penetration.ts:621`            |
|  16 | 2.1% | 29.4 | `updateWorld`                       | `src/update.ts:45`                            |
|  17 | 2.0% | 27.6 | `castConvexVsTriangleMesh`          | `src/shapes/triangle-mesh.ts:677`             |
|  18 | 2.0% | 27.5 | `update$1`                          | `src/broadphase/dbvt.ts:485`                  |
|  19 | 1.9% | 26.0 | `(garbage collector)`               | ``                                            |
|  20 | 1.7% | 23.5 | `rayHitsBox3`                       | `src/collision/cast-utils.ts:114`             |
|  21 | 1.5% | 20.2 | `compileSourceTextModule`           | `node:internal/modules/esm/utils`             |
|  22 | 1.5% | 20.1 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:582`  |
|  23 | 1.3% | 17.9 | `getSupport`                        | `src/collision/support.ts:125`                |
|  24 | 1.3% | 17.3 | `narrowphase`                       | `src/update.ts:949`                           |
|  25 | 1.2% | 16.7 | `velocityIntegrationUpdate`         | `src/update.ts:1131`                          |
|  26 | 1.2% | 16.4 | `visit`                             | `src/update.ts:1387`                          |
|  27 | 1.1% | 14.5 | `collideConvexVsConvex`             | `src/shapes/convex.ts:247`                    |
|  28 | 1.0% | 14.3 | `penetrationDepthStepEPA`           | `src/collision/penetration.ts:209`            |
|  29 | 0.9% | 12.4 | `subtract$1`                        | `…/node_modules/mathcat/dist/vec3.js:154`     |
|  30 | 0.9% | 11.8 | `collideConvexVsConvexLocal`        | `src/shapes/convex.ts:329`                    |

