# perf report — mesh-field

| field | value |
| --- | --- |
| scenario | `mesh-field` |
| date | 2026-07-04T08:04:04.392Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1735.4 ms |
| attributed | 1566.9 ms |
| startup excluded | 167.7 ms |
| samples | 1448 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 25.3% | 396.7 |
| shapes        | 20.8% | 326.6 |
| narrowphase   | 15.2% | 238.9 |
| math          | 13.1% | 204.6 |
| step          |  8.1% | 126.4 |
| broadphase    |  6.2% |  96.4 |
| body          |  5.1% |  79.7 |
| crashcat-util |  2.2% |  34.8 |
| manifold      |  1.9% |  29.6 |
| runtime       |  1.4% |  21.5 |
| bench-harness |  0.6% |   9.3 |
| other         |  0.1% |   1.3 |
| ccd           |  0.1% |   1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                            | location                                       |
| --: | ----: | ----: | ----------------------------------- | ---------------------------------------------- |
|   1 | 11.0% | 172.7 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1127`  |
|   2 |  7.6% | 119.5 | `collideConvexVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1239`             |
|   3 |  6.7% | 105.1 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1246`                    |
|   4 |  6.5% | 102.6 | `collideSphereVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1755`             |
|   5 |  4.1% |  63.7 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:583`   |
|   6 |  3.6% |  57.1 | `getSupport`                        | `src/collision/support.ts:125`                 |
|   7 |  3.0% |  46.5 | `update$11`                         | `src/broadphase/dbvt.ts:480`                   |
|   8 |  2.7% |  41.8 | `transformMat4$1`                   | `…/node_modules/mathcat/dist/vec3.js:530`      |
|   9 |  2.7% |  41.6 | `updateWorld`                       | `src/update.ts:46`                             |
|  10 |  2.6% |  41.3 | `intersectAABBFatLeaves`            | `src/broadphase/dbvt.ts:590`                   |
|  11 |  2.4% |  38.0 | `getTriangleVertices`               | `src/shapes/utils/triangle-mesh-data.ts:44`    |
|  12 |  2.0% |  31.8 | `getClosestPointOnTriangle`         | `src/collision/triangle.ts:62`                 |
|  13 |  1.8% |  28.9 | `solvePositionConstraintsForIsland` | `src/constraints/contact-constraints.ts:1405`  |
|  14 |  1.7% |  26.3 | `multiply$2`                        | `…/node_modules/mathcat/dist/vec3.js:182`      |
|  15 |  1.6% |  25.7 | `subtract$1`                        | `…/node_modules/mathcat/dist/vec3.js:154`      |
|  16 |  1.6% |  25.3 | `narrowphase`                       | `src/update.ts:998`                            |
|  17 |  1.6% |  25.2 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`            |
|  18 |  1.6% |  25.0 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`              |
|  19 |  1.5% |  23.6 | `findCollidingPairs`                | `src/pairs.ts:409`                             |
|  20 |  1.4% |  22.3 | `checkIslandSleep`                  | `src/islands.ts:382`                           |
|  21 |  1.4% |  21.4 | `finalize`                          | `src/islands.ts:240`                           |
|  22 |  1.1% |  17.4 | `warmStartVelocityConstraints`      | `src/constraints/contact-constraints.ts:977`   |
|  23 |  1.0% |  15.3 | `velocityIntegrationUpdate`         | `src/update.ts:1181`                           |
|  24 |  1.0% |  15.3 | `addHit`                            | `src/update.ts:455`                            |
|  25 |  1.0% |  15.2 | `(garbage collector)`               | ``                                             |
|  26 |  0.9% |  13.9 | `getVelocityAtPointCOM`             | `src/body/rigid-body.ts:1035`                  |
|  27 |  0.8% |  12.7 | `getContactsFromCache`              | `src/update.ts:914`                            |
|  28 |  0.8% |  12.0 | `updatePositionFromCenterOfMass`    | `src/body/rigid-body.ts:555`                   |
|  29 |  0.7% |  11.6 | `buildTriangleMesh`                 | `src/shapes/utils/triangle-mesh-builder.ts:63` |
|  30 |  0.7% |  11.2 | `transformQuat`                     | `…/node_modules/mathcat/dist/vec3.js:567`      |

