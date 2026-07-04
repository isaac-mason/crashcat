# perf report — settle-sleep

| field | value |
| --- | --- |
| scenario | `settle-sleep` |
| date | 2026-07-04T08:04:53.561Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 517.9 ms |
| attributed | 358.4 ms |
| startup excluded | 159.1 ms |
| samples | 469 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 40.8% | 146.4 |
| narrowphase   | 14.1% |  50.7 |
| step          | 11.6% |  41.7 |
| body          |  9.8% |  35.1 |
| broadphase    |  6.6% |  23.7 |
| crashcat-util |  5.0% |  18.1 |
| shapes        |  4.3% |  15.3 |
| math          |  3.3% |  11.9 |
| runtime       |  3.3% |  11.8 |
| manifold      |  0.7% |   2.5 |
| bench-harness |  0.4% |   1.4 |

## top 30 self-time hotspots

|   # |   pct |   ms | function                                | location                                                                 |
| --: | ----: | ---: | --------------------------------------- | ------------------------------------------------------------------------ |
|   1 | 16.6% | 59.4 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1127`                            |
|   2 |  5.3% | 19.0 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:583`                             |
|   3 |  4.9% | 17.4 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                                        |
|   4 |  4.5% | 16.0 | `updateWorld`                           | `src/update.ts:46`                                                       |
|   5 |  4.2% | 15.2 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1246`                                              |
|   6 |  4.2% | 14.9 | `findCollidingPairs`                    | `src/pairs.ts:409`                                                       |
|   7 |  3.9% | 14.0 | `prepare`                               | `src/islands.ts:64`                                                      |
|   8 |  3.1% | 11.1 | `getVelocityAtPointCOM`                 | `src/body/rigid-body.ts:1035`                                            |
|   9 |  2.8% | 10.0 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1405`                            |
|  10 |  2.3% |  8.1 | `(garbage collector)`                   | ``                                                                       |
|  11 |  2.2% |  7.8 | `insertLeaf`                            | `src/broadphase/dbvt.ts:111`                                             |
|  12 |  2.1% |  7.7 | `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                               |
|  13 |  2.0% |  7.3 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:977`                             |
|  14 |  1.8% |  6.4 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81`             |
|  15 |  1.8% |  6.3 | `reconstructManifoldFromCache`          | `src/update.ts:845`                                                      |
|  16 |  1.6% |  5.8 | `optimizeIncremental`                   | `src/broadphase/dbvt.ts:544`                                             |
|  17 |  1.6% |  5.8 | `update$11`                             | `src/broadphase/dbvt.ts:480`                                             |
|  18 |  1.6% |  5.7 | `updatePositionFromCenterOfMass`        | `src/body/rigid-body.ts:555`                                             |
|  19 |  1.6% |  5.6 | `velocityIntegrationUpdate`             | `src/update.ts:1181`                                                     |
|  20 |  1.4% |  5.1 | `penetrationDepthStepEPA`               | `src/collision/penetration.ts:209`                                       |
|  21 |  1.4% |  5.0 | `findEdge$1`                            | `src/collision/epa-convex-hull-builder.ts:548`                           |
|  22 |  1.4% |  5.0 | `storeAppliedImpulses`                  | `src/constraints/contact-constraints.ts:1365`                            |
|  23 |  1.3% |  4.8 | `linkContactConstraints`                | `src/islands.ts:184`                                                     |
|  24 |  1.1% |  4.0 | `accelerationIntegrationUpdate`         | `src/update.ts:293`                                                      |
|  25 |  1.1% |  3.9 | `calculateNormalVelocityBias`           | `src/constraints/contact-constraints.ts:310`                             |
|  26 |  1.1% |  3.9 | `calculateConstraintProperties`         | `src/constraints/constraint-part/angular-friction-constraint-part.ts:77` |
|  27 |  1.1% |  3.8 | `createTriangle$1`                      | `src/collision/epa-convex-hull-builder.ts:251`                           |
|  28 |  1.1% |  3.8 | `getContactsFromCache`                  | `src/update.ts:914`                                                      |
|  29 |  1.1% |  3.8 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:476`                             |
|  30 |  1.0% |  3.8 | `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                               |

