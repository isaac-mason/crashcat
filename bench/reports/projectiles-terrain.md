# perf report — projectiles-terrain

| field | value |
| --- | --- |
| scenario | `projectiles-terrain` |
| date | 2026-07-03T14:15:22.748Z |
| git rev | `9b65b0d` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 987.2 ms |
| attributed | 819.1 ms |
| startup excluded | 167.5 ms |
| samples | 911 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 30.3% | 248.5 |
| shapes        | 18.7% | 153.3 |
| solver        | 11.2% |  91.5 |
| step          | 10.9% |  89.1 |
| broadphase    | 10.6% |  87.2 |
| math          |  8.4% |  69.2 |
| body          |  3.4% |  27.8 |
| runtime       |  2.6% |  21.0 |
| crashcat-util |  1.8% |  14.5 |
| manifold      |  1.0% |   8.0 |
| bench-harness |  0.6% |   5.3 |
| ccd           |  0.5% |   3.8 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                            | location                                       |
| --: | ---: | ---: | ----------------------------------- | ---------------------------------------------- |
|   1 | 9.1% | 74.8 | `rayDistanceToBox3`                 | `src/collision/cast-utils.ts:20`               |
|   2 | 4.4% | 35.8 | `castAABB$1`                        | `src/broadphase/dbvt.ts:936`                   |
|   3 | 4.3% | 34.9 | `collideSphereVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1723`             |
|   4 | 4.0% | 32.5 | `collideConvexVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1209`             |
|   5 | 3.8% | 31.3 | `castSphereVsTriangleMesh`          | `src/shapes/triangle-mesh.ts:2173`             |
|   6 | 3.4% | 28.0 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`              |
|   7 | 3.2% | 26.2 | `intersectAABB$1`                   | `src/broadphase/dbvt.ts:627`                   |
|   8 | 2.6% | 21.6 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1225`                    |
|   9 | 2.6% | 21.4 | `getSupport`                        | `src/collision/support.ts:125`                 |
|  10 | 2.6% | 21.2 | `updateWorld`                       | `src/update.ts:45`                             |
|  11 | 2.6% | 21.0 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1121`  |
|  12 | 2.5% | 20.1 | `gjkCastShape`                      | `src/collision/gjk.ts:913`                     |
|  13 | 2.4% | 19.6 | `update$1`                          | `src/broadphase/dbvt.ts:485`                   |
|  14 | 2.3% | 19.0 | `castConvexVsTriangleMesh`          | `src/shapes/triangle-mesh.ts:676`              |
|  15 | 2.0% | 16.1 | `penetrationCastShape`              | `src/collision/penetration.ts:621`             |
|  16 | 1.8% | 14.9 | `(garbage collector)`               | ``                                             |
|  17 | 1.8% | 14.9 | `visit`                             | `src/update.ts:1387`                           |
|  18 | 1.7% | 14.1 | `penetrationDepthStepEPA`           | `src/collision/penetration.ts:209`             |
|  19 | 1.5% | 12.3 | `push`                              | `src/utils/bvh-stack.ts:36`                    |
|  20 | 1.5% | 12.2 | `velocityIntegrationUpdate`         | `src/update.ts:1131`                           |
|  21 | 1.4% | 11.8 | `rayHitsBox3`                       | `src/collision/cast-utils.ts:114`              |
|  22 | 1.4% | 11.5 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`            |
|  23 | 1.4% | 11.2 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:582`   |
|  24 | 1.3% | 11.0 | `transformQuat`                     | `…/node_modules/mathcat/dist/vec3.js:567`      |
|  25 | 1.2% |  9.5 | `finalize`                          | `src/islands.ts:240`                           |
|  26 | 1.1% |  9.1 | `intersectsBox3`                    | `…/node_modules/mathcat/dist/raycast3.js:195`  |
|  27 | 1.1% |  9.0 | `createTriangle$1`                  | `src/collision/epa-convex-hull-builder.ts:251` |
|  28 | 1.1% |  8.8 | `getTriangleVertices`               | `src/shapes/utils/triangle-mesh-data.ts:44`    |
|  29 | 1.0% |  8.2 | `getContactsFromCache`              | `src/update.ts:868`                            |
|  30 | 0.9% |  7.0 | `subtract$1`                        | `…/node_modules/mathcat/dist/vec3.js:154`      |

