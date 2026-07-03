# perf report — cube-heap

| field | value |
| --- | --- |
| scenario | `cube-heap` |
| date | 2026-07-03T04:39:50.928Z |
| git rev | `58a218d` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 6983.8 ms |
| attributed | 6983.2 ms |
| samples | 5601 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| solver        | 45.2% | 3156.8 |
| narrowphase   | 15.8% | 1104.5 |
| step          |  9.6% |  669.6 |
| broadphase    |  9.1% |  638.6 |
| body          |  5.2% |  362.0 |
| shapes        |  4.3% |  302.0 |
| math          |  4.0% |  276.8 |
| runtime       |  3.3% |  231.3 |
| manifold      |  3.2% |  225.5 |
| ccd           |  0.1% |    6.1 |
| bench-harness |  0.1% |    5.1 |
| crashcat-util |  0.1% |    3.8 |
| other         |  0.0% |    1.3 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                                  | location                                                      |
| --: | ----: | -----: | ----------------------------------------- | ------------------------------------------------------------- |
|   1 | 18.6% | 1297.7 | `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1121`                 |
|   2 |  7.7% |  541.0 | `intersectAABB$1`                         | `src/broadphase/dbvt.ts:627`                                  |
|   3 |  6.6% |  459.4 | `addContactConstraint`                    | `src/constraints/contact-constraints.ts:582`                  |
|   4 |  5.5% |  382.7 | `gjkClosestPoints`                        | `src/collision/gjk.ts:1225`                                   |
|   5 |  5.0% |  348.5 | `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1399`                 |
|   6 |  4.5% |  316.8 | `getSupport`                              | `src/collision/support.ts:125`                                |
|   7 |  3.5% |  247.7 | `updateWorld`                             | `src/update.ts:45`                                            |
|   8 |  2.5% |  171.8 | `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                           |
|   9 |  2.3% |  159.4 | `getContactsFromCache`                    | `src/update.ts:868`                                           |
|  10 |  2.1% |  148.2 | `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:971`                  |
|  11 |  1.8% |  129.0 | `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                     |
|  12 |  1.8% |  125.0 | `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1007`                                 |
|  13 |  1.8% |  123.7 | `narrowphase`                             | `src/update.ts:949`                                           |
|  14 |  1.5% |  106.9 | `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`  |
|  15 |  1.3% |   88.1 | `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                    |
|  16 |  1.2% |   86.0 | `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1359`                 |
|  17 |  1.2% |   80.7 | `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                    |
|  18 |  1.1% |   78.6 | `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:548`                |
|  19 |  1.1% |   77.3 | `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                |
|  20 |  1.1% |   76.8 | `findContact`                             | `src/contacts.ts:647`                                         |
|  21 |  1.1% |   76.2 | `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                            |
|  22 |  1.0% |   71.9 | `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810` |
|  23 |  1.0% |   70.8 | `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1541`                 |
|  24 |  1.0% |   69.4 | `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                     |
|  25 |  0.9% |   65.8 | `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:309`                  |
|  26 |  0.9% |   64.4 | `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                             |
|  27 |  0.9% |   60.8 | `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:475`                  |
|  28 |  0.9% |   60.0 | `update$1`                                | `src/broadphase/dbvt.ts:485`                                  |
|  29 |  0.8% |   57.5 | `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:629`                |
|  30 |  0.8% |   54.9 | `getSupportingFace$11`                    | `src/shapes/box.ts:157`                                       |

