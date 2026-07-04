# perf report — projectiles-terrain

| field | value |
| --- | --- |
| scenario | `projectiles-terrain` |
| date | 2026-07-04T08:04:07.235Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1138.4 ms |
| attributed | 891.4 ms |
| startup excluded | 246.9 ms |
| samples | 940 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 29.8% | 265.7 |
| shapes        | 19.6% | 174.8 |
| solver        | 11.1% |  99.0 |
| broadphase    |  9.4% |  83.5 |
| math          |  8.7% |  77.5 |
| step          |  7.9% |  70.8 |
| runtime       |  5.2% |  46.0 |
| body          |  3.5% |  30.8 |
| crashcat-util |  2.3% |  20.8 |
| bench-harness |  1.1% |   9.9 |
| manifold      |  0.7% |   6.5 |
| other         |  0.4% |   3.8 |
| ccd           |  0.3% |   2.3 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                            | location                                       |
| --: | ---: | ---: | ----------------------------------- | ---------------------------------------------- |
|   1 | 6.4% | 57.0 | `rayDistanceToBox3`                 | `src/collision/cast-utils.ts:20`               |
|   2 | 5.5% | 49.3 | `castAABB$1`                        | `src/broadphase/dbvt.ts:973`                   |
|   3 | 4.4% | 39.3 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1246`                    |
|   4 | 4.0% | 35.9 | `penetrationCastShape`              | `src/collision/penetration.ts:618`             |
|   5 | 3.8% | 34.0 | `collideConvexVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1239`             |
|   6 | 3.8% | 33.5 | `(garbage collector)`               | ``                                             |
|   7 | 3.7% | 33.3 | `collideSphereVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1755`             |
|   8 | 3.6% | 32.2 | `castSphereVsTriangleMesh`          | `src/shapes/triangle-mesh.ts:2208`             |
|   9 | 3.4% | 30.7 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1127`  |
|  10 | 2.8% | 25.1 | `updateWorld`                       | `src/update.ts:46`                             |
|  11 | 2.6% | 23.3 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`              |
|  12 | 2.3% | 20.8 | `update$11`                         | `src/broadphase/dbvt.ts:480`                   |
|  13 | 2.3% | 20.3 | `castConvexVsTriangleMesh`          | `src/shapes/triangle-mesh.ts:693`              |
|  14 | 2.2% | 19.7 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:583`   |
|  15 | 2.2% | 19.4 | `gjkCastShape`                      | `src/collision/gjk.ts:924`                     |
|  16 | 1.8% | 15.9 | `subtract$1`                        | `…/node_modules/mathcat/dist/vec3.js:154`      |
|  17 | 1.6% | 14.7 | `multiply$2`                        | `…/node_modules/mathcat/dist/vec3.js:182`      |
|  18 | 1.6% | 14.2 | `getSupport`                        | `src/collision/support.ts:125`                 |
|  19 | 1.5% | 13.3 | `finalize`                          | `src/islands.ts:240`                           |
|  20 | 1.2% | 10.5 | `penetrationDepthStepEPA`           | `src/collision/penetration.ts:209`             |
|  21 | 1.1% |  9.7 | `getClosestPointOnTriangle`         | `src/collision/triangle.ts:62`                 |
|  22 | 1.0% |  8.6 | `rayHitsBox3`                       | `src/collision/cast-utils.ts:114`              |
|  23 | 1.0% |  8.6 | `collideConvexVsConvexLocal`        | `src/shapes/convex.ts:329`                     |
|  24 | 0.9% |  8.2 | `createTriangle$1`                  | `src/collision/epa-convex-hull-builder.ts:251` |
|  25 | 0.8% |  7.2 | `intersectAABBFatLeaves`            | `src/broadphase/dbvt.ts:590`                   |
|  26 | 0.8% |  7.1 | `buildTriangleMesh`                 | `src/shapes/utils/triangle-mesh-builder.ts:63` |
|  27 | 0.8% |  7.1 | `accelerationIntegrationUpdate`     | `src/update.ts:293`                            |
|  28 | 0.8% |  7.0 | `findCCDContacts`                   | `src/update.ts:1638`                           |
|  29 | 0.7% |  6.7 | `narrowphase`                       | `src/update.ts:998`                            |
|  30 | 0.7% |  6.4 | `findEdge$1`                        | `src/collision/epa-convex-hull-builder.ts:548` |

