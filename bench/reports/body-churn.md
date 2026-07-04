# perf report — body-churn

| field | value |
| --- | --- |
| scenario | `body-churn` |
| date | 2026-07-04T08:04:56.663Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 793.7 ms |
| attributed | 626.9 ms |
| startup excluded | 166 ms |
| samples | 660 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 33.0% | 207.2 |
| solver        | 20.7% | 129.8 |
| broadphase    | 17.7% | 110.9 |
| step          |  8.1% |  50.9 |
| shapes        |  4.7% |  29.2 |
| body          |  4.6% |  28.9 |
| math          |  3.8% |  23.5 |
| runtime       |  3.0% |  19.0 |
| manifold      |  2.0% |  12.5 |
| crashcat-util |  1.6% |   9.8 |
| bench-harness |  0.4% |   2.6 |
| other         |  0.4% |   2.5 |

## top 30 self-time hotspots

|   # |   pct |   ms | function                            | location                                       |
| --: | ----: | ---: | ----------------------------------- | ---------------------------------------------- |
|   1 | 10.1% | 63.0 | `intersectAABBFatLeaves`            | `src/broadphase/dbvt.ts:590`                   |
|   2 |  6.6% | 41.6 | `update$11`                         | `src/broadphase/dbvt.ts:480`                   |
|   3 |  6.6% | 41.5 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1127`  |
|   4 |  6.5% | 41.0 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1246`                    |
|   5 |  6.0% | 37.5 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`              |
|   6 |  4.4% | 27.7 | `getSupport`                        | `src/collision/support.ts:125`                 |
|   7 |  3.8% | 23.6 | `updateWorld`                       | `src/update.ts:46`                             |
|   8 |  3.8% | 23.6 | `findEdge$1`                        | `src/collision/epa-convex-hull-builder.ts:548` |
|   9 |  3.6% | 22.3 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:583`   |
|  10 |  3.2% | 20.0 | `createTriangle$1`                  | `src/collision/epa-convex-hull-builder.ts:251` |
|  11 |  3.0% | 18.7 | `addPoint$1`                        | `src/collision/epa-convex-hull-builder.ts:629` |
|  12 |  2.9% | 18.4 | `penetrationDepthStepEPA`           | `src/collision/penetration.ts:209`             |
|  13 |  2.8% | 17.8 | `collideConvexVsConvexLocal`        | `src/shapes/convex.ts:329`                     |
|  14 |  2.8% | 17.4 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`            |
|  15 |  2.4% | 15.0 | `finalize`                          | `src/islands.ts:240`                           |
|  16 |  2.2% | 13.9 | `(garbage collector)`               | ``                                             |
|  17 |  1.4% |  8.8 | `collideShapeVsShape`               | `src/collision/narrowphase.ts:183`             |
|  18 |  1.2% |  7.6 | `calculateNormalVelocityBias`       | `src/constraints/contact-constraints.ts:310`   |
|  19 |  1.2% |  7.6 | `clipPolyVsPlane`                   | `src/manifold/clip.ts:13`                      |
|  20 |  1.2% |  7.4 | `findCollidingPairs`                | `src/pairs.ts:409`                             |
|  21 |  1.1% |  6.6 | `solvePositionConstraintsForIsland` | `src/constraints/contact-constraints.ts:1405`  |
|  22 |  1.0% |  6.5 | `velocityIntegrationUpdate`         | `src/update.ts:1181`                           |
|  23 |  1.0% |  6.3 | `initialize$2`                      | `src/collision/epa-convex-hull-builder.ts:419` |
|  24 |  0.9% |  5.8 | `warmStartVelocityConstraints`      | `src/constraints/contact-constraints.ts:977`   |
|  25 |  0.8% |  5.0 | `addHit`                            | `src/update.ts:455`                            |
|  26 |  0.8% |  5.0 | `getShapeSupportingFace`            | `src/shapes/shapes.ts:489`                     |
|  27 |  0.8% |  5.0 | `narrowphase`                       | `src/update.ts:998`                            |
|  28 |  0.8% |  5.0 | `updatePositionFromCenterOfMass`    | `src/body/rigid-body.ts:555`                   |
|  29 |  0.6% |  3.9 | `getSupportingFace$11`              | `src/shapes/box.ts:157`                        |
|  30 |  0.6% |  3.8 | `getContactsFromCache`              | `src/update.ts:914`                            |

