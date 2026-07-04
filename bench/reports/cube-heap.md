# perf report — cube-heap

| field | value |
| --- | --- |
| scenario | `cube-heap` |
| date | 2026-07-04T08:03:33.721Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 7609.2 ms |
| attributed | 7398.9 ms |
| startup excluded | 210 ms |
| samples | 5508 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| solver        | 47.2% | 3493.5 |
| narrowphase   | 17.8% | 1313.5 |
| step          |  6.7% |  497.4 |
| body          |  6.0% |  443.8 |
| math          |  5.2% |  386.5 |
| shapes        |  4.9% |  364.7 |
| manifold      |  3.6% |  262.7 |
| broadphase    |  3.0% |  218.9 |
| crashcat-util |  2.7% |  200.0 |
| runtime       |  2.6% |  195.9 |
| bench-harness |  0.2% |   18.0 |
| ccd           |  0.0% |    2.9 |
| other         |  0.0% |    1.3 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                                  | location                                                      |
| --: | ----: | -----: | ----------------------------------------- | ------------------------------------------------------------- |
|   1 | 20.5% | 1517.8 | `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1127`                 |
|   2 |  6.7% |  496.3 | `addContactConstraint`                    | `src/constraints/contact-constraints.ts:583`                  |
|   3 |  6.7% |  493.5 | `gjkClosestPoints`                        | `src/collision/gjk.ts:1246`                                   |
|   4 |  4.5% |  336.6 | `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1405`                 |
|   5 |  3.3% |  247.3 | `getSupport`                              | `src/collision/support.ts:125`                                |
|   6 |  2.9% |  214.3 | `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:977`                  |
|   7 |  2.6% |  194.0 | `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                           |
|   8 |  2.6% |  191.8 | `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                             |
|   9 |  2.4% |  179.1 | `(garbage collector)`                     | ``                                                            |
|  10 |  2.4% |  178.5 | `findCollidingPairs`                      | `src/pairs.ts:409`                                            |
|  11 |  2.2% |  166.3 | `updateWorld`                             | `src/update.ts:46`                                            |
|  12 |  2.1% |  157.5 | `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                     |
|  13 |  2.0% |  151.4 | `intersectAABBFatLeaves`                  | `src/broadphase/dbvt.ts:590`                                  |
|  14 |  2.0% |  149.5 | `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1035`                                 |
|  15 |  2.0% |  147.9 | `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                    |
|  16 |  2.0% |  147.3 | `narrowphase`                             | `src/update.ts:998`                                           |
|  17 |  1.8% |  134.0 | `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`  |
|  18 |  1.7% |  125.3 | `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                            |
|  19 |  1.5% |  108.7 | `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1365`                 |
|  20 |  1.4% |  105.0 | `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1547`                 |
|  21 |  1.3% |   95.3 | `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:476`                  |
|  22 |  1.2% |   87.9 | `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                     |
|  23 |  1.2% |   85.3 | `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810` |
|  24 |  1.0% |   70.9 | `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                    |
|  25 |  0.9% |   69.6 | `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                |
|  26 |  0.9% |   68.1 | `combineMaterial`                         | `src/constraints/combine-material.ts:40`                      |
|  27 |  0.8% |   62.1 | `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:629`                |
|  28 |  0.8% |   57.4 | `getContactsFromCache`                    | `src/update.ts:914`                                           |
|  29 |  0.8% |   56.7 | `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:548`                |
|  30 |  0.7% |   55.4 | `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:310`                  |

