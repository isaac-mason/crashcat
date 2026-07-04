# perf report — cube-heap

| field | value |
| --- | --- |
| scenario | `cube-heap` |
| date | 2026-07-03T15:46:30.571Z |
| git rev | `0252042` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 6377.3 ms |
| attributed | 6189 ms |
| startup excluded | 187.9 ms |
| samples | 5525 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| solver        | 46.5% | 2878.5 |
| narrowphase   | 17.8% | 1102.4 |
| step          | 10.3% |  638.4 |
| broadphase    |  7.1% |  441.6 |
| body          |  5.6% |  346.4 |
| shapes        |  4.3% |  267.6 |
| math          |  3.9% |  241.5 |
| manifold      |  3.5% |  218.0 |
| runtime       |  0.8% |   47.0 |
| bench-harness |  0.1% |    3.8 |
| other         |  0.0% |    1.3 |
| ccd           |  0.0% |    1.3 |
| crashcat-util |  0.0% |    1.3 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                                  | location                                                      |
| --: | ----: | -----: | ----------------------------------------- | ------------------------------------------------------------- |
|   1 | 20.4% | 1261.1 | `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1121`                 |
|   2 |  6.4% |  394.3 | `gjkClosestPoints`                        | `src/collision/gjk.ts:1225`                                   |
|   3 |  6.2% |  386.4 | `intersectAABB$1`                         | `src/broadphase/dbvt.ts:577`                                  |
|   4 |  5.4% |  337.0 | `addContactConstraint`                    | `src/constraints/contact-constraints.ts:582`                  |
|   5 |  5.0% |  306.9 | `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1399`                 |
|   6 |  4.9% |  300.2 | `getSupport`                              | `src/collision/support.ts:125`                                |
|   7 |  4.1% |  256.4 | `updateWorld`                             | `src/update.ts:45`                                            |
|   8 |  2.5% |  156.4 | `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                           |
|   9 |  2.4% |  149.7 | `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:971`                  |
|  10 |  2.2% |  138.1 | `getContactsFromCache`                    | `src/update.ts:871`                                           |
|  11 |  1.9% |  118.4 | `narrowphase`                             | `src/update.ts:952`                                           |
|  12 |  1.8% |  112.3 | `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                     |
|  13 |  1.8% |  110.2 | `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1011`                                 |
|  14 |  1.6% |   96.3 | `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                    |
|  15 |  1.5% |   89.8 | `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`  |
|  16 |  1.3% |   79.9 | `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:629`                |
|  17 |  1.3% |   78.2 | `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1359`                 |
|  18 |  1.1% |   70.9 | `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                             |
|  19 |  1.1% |   69.9 | `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810` |
|  20 |  1.1% |   68.3 | `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1541`                 |
|  21 |  1.1% |   65.7 | `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                            |
|  22 |  1.1% |   65.2 | `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                     |
|  23 |  1.0% |   64.9 | `findContact`                             | `src/contacts.ts:680`                                         |
|  24 |  1.0% |   64.7 | `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                |
|  25 |  1.0% |   62.1 | `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                    |
|  26 |  0.9% |   55.6 | `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:548`                |
|  27 |  0.8% |   51.2 | `manifoldBetweenTwoFaces`                 | `src/manifold/manifold.ts:394`                                |
|  28 |  0.7% |   46.1 | `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:475`                  |
|  29 |  0.7% |   43.5 | `linkContactConstraints`                  | `src/islands.ts:184`                                          |
|  30 |  0.7% |   43.0 | `setBoxSupport`                           | `src/collision/support.ts:296`                                |

