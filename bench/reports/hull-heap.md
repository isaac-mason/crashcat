# perf report — hull-heap

| field | value |
| --- | --- |
| scenario | `hull-heap` |
| date | 2026-07-03T04:39:37.631Z |
| git rev | `58a218d` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 11132.3 ms |
| attributed | 11131.4 ms |
| samples | 8886 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| narrowphase   | 41.8% | 4652.1 |
| solver        | 23.7% | 2639.1 |
| shapes        | 12.2% | 1352.9 |
| step          |  7.4% |  821.0 |
| broadphase    |  5.8% |  647.3 |
| body          |  2.8% |  315.3 |
| math          |  2.6% |  286.3 |
| runtime       |  2.1% |  239.1 |
| manifold      |  1.5% |  169.8 |
| bench-harness |  0.1% |    7.1 |
| other         |  0.0% |    1.3 |
| crashcat-util |  0.0% |    0.1 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                            | location                                                     |
| --: | ----: | -----: | ----------------------------------- | ------------------------------------------------------------ |
|   1 | 28.7% | 3194.3 | `getSupport`                        | `src/collision/support.ts:125`                               |
|   2 | 10.9% | 1209.6 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1121`                |
|   3 |  9.4% | 1049.0 | `getSupportingFace$8`               | `src/shapes/convex-hull.ts:529`                              |
|   4 |  5.1% |  562.5 | `intersectAABB$1`                   | `src/broadphase/dbvt.ts:627`                                 |
|   5 |  4.4% |  490.8 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1225`                                  |
|   6 |  3.6% |  400.3 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:582`                 |
|   7 |  3.1% |  340.4 | `updateWorld`                       | `src/update.ts:45`                                           |
|   8 |  2.2% |  247.0 | `createTriangle$1`                  | `src/collision/epa-convex-hull-builder.ts:251`               |
|   9 |  2.0% |  225.8 | `findEdge$1`                        | `src/collision/epa-convex-hull-builder.ts:548`               |
|  10 |  1.9% |  211.7 | `solvePositionConstraintsForIsland` | `src/constraints/contact-constraints.ts:1399`                |
|  11 |  1.9% |  210.2 | `getContactsFromCache`              | `src/update.ts:868`                                          |
|  12 |  1.7% |  188.0 | `penetrationDepthStepEPA`           | `src/collision/penetration.ts:209`                           |
|  13 |  1.6% |  174.4 | `addPoint$1`                        | `src/collision/epa-convex-hull-builder.ts:629`               |
|  14 |  1.4% |  154.2 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`                          |
|  15 |  1.4% |  153.1 | `narrowphase`                       | `src/update.ts:949`                                          |
|  16 |  1.1% |  121.4 | `warmStartVelocityConstraints`      | `src/constraints/contact-constraints.ts:971`                 |
|  17 |  1.1% |  119.4 | `collideConvexVsConvexLocal`        | `src/shapes/convex.ts:329`                                   |
|  18 |  1.1% |  117.4 | `getVelocityAtPointCOM`             | `src/body/rigid-body.ts:1007`                                |
|  19 |  0.8% |   84.3 | `clipPolyVsPlane`                   | `src/manifold/clip.ts:13`                                    |
|  20 |  0.8% |   84.2 | `multiply3x3TransposedVec`          | `…/node_modules/mathcat/dist/mat4.js:603`                    |
|  21 |  0.7% |   81.0 | `collideConvexVsConvex`             | `src/shapes/convex.ts:247`                                   |
|  22 |  0.7% |   76.8 | `storeAppliedImpulses`              | `src/constraints/contact-constraints.ts:1359`                |
|  23 |  0.7% |   76.0 | `calculateInverseEffectiveMass`     | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  24 |  0.7% |   75.2 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`                            |
|  25 |  0.6% |   71.7 | `calculateNormalVelocityBias`       | `src/constraints/contact-constraints.ts:309`                 |
|  26 |  0.5% |   55.9 | `update$1`                          | `src/broadphase/dbvt.ts:485`                                 |
|  27 |  0.5% |   53.6 | `findContact`                       | `src/contacts.ts:647`                                        |
|  28 |  0.5% |   51.3 | `sortContactIndices`                | `src/constraints/contact-constraints.ts:1541`                |
|  29 |  0.4% |   45.2 | `fromRotationTranslation`           | `…/node_modules/mathcat/dist/mat4.js:1150`                   |
|  30 |  0.4% |   44.8 | `setCachedBodyPair`                 | `src/contacts.ts:174`                                        |

