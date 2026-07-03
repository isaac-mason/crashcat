# perf report — scaled-hulls

| field | value |
| --- | --- |
| scenario | `scaled-hulls` |
| date | 2026-07-03T04:39:43.803Z |
| git rev | `58a218d` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 6018.3 ms |
| attributed | 6018.1 ms |
| samples | 4736 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| solver        | 28.6% | 1719.0 |
| narrowphase   | 26.8% | 1612.6 |
| step          | 10.9% |  654.9 |
| shapes        | 10.1% |  610.8 |
| broadphase    |  9.6% |  575.1 |
| runtime       |  4.3% |  258.3 |
| body          |  4.0% |  243.3 |
| math          |  3.7% |  223.8 |
| manifold      |  1.8% |  107.7 |
| bench-harness |  0.1% |    6.1 |
| crashcat-util |  0.0% |    2.6 |
| ccd           |  0.0% |    2.5 |
| other         |  0.0% |    1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                                | location                                                     |
| --: | ----: | ----: | --------------------------------------- | ------------------------------------------------------------ |
|   1 | 16.2% | 975.0 | `getSupport`                            | `src/collision/support.ts:125`                               |
|   2 | 12.6% | 757.2 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1121`                |
|   3 |  7.9% | 474.1 | `intersectAABB$1`                       | `src/broadphase/dbvt.ts:627`                                 |
|   4 |  5.5% | 330.0 | `getSupportingFace$8`                   | `src/shapes/convex-hull.ts:529`                              |
|   5 |  4.9% | 295.4 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:582`                 |
|   6 |  4.5% | 272.5 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1225`                                  |
|   7 |  3.9% | 237.0 | `updateWorld`                           | `src/update.ts:45`                                           |
|   8 |  3.2% | 190.8 | `getContactsFromCache`                  | `src/update.ts:868`                                          |
|   9 |  2.6% | 154.9 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1399`                |
|  10 |  2.2% | 129.5 | `narrowphase`                           | `src/update.ts:949`                                          |
|  11 |  1.9% | 111.4 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                          |
|  12 |  1.4% |  83.3 | `getVelocityAtPointCOM`                 | `src/body/rigid-body.ts:1007`                                |
|  13 |  1.4% |  82.5 | `createTriangle$1`                      | `src/collision/epa-convex-hull-builder.ts:251`               |
|  14 |  1.2% |  74.3 | `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                   |
|  15 |  1.2% |  71.3 | `update$1`                              | `src/broadphase/dbvt.ts:485`                                 |
|  16 |  1.1% |  67.5 | `collideShapeVsScaled`                  | `src/shapes/scaled.ts:360`                                   |
|  17 |  1.1% |  66.4 | `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                   |
|  18 |  1.1% |  66.2 | `penetrationDepthStepEPA`               | `src/collision/penetration.ts:209`                           |
|  19 |  1.1% |  65.2 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:971`                 |
|  20 |  1.1% |  64.7 | `findEdge$1`                            | `src/collision/epa-convex-hull-builder.ts:548`               |
|  21 |  1.0% |  62.2 | `addPoint$1`                            | `src/collision/epa-convex-hull-builder.ts:629`               |
|  22 |  0.9% |  54.5 | `clipPolyVsPlane`                       | `src/manifold/clip.ts:13`                                    |
|  23 |  0.9% |  53.8 | `(garbage collector)`                   | ``                                                           |
|  24 |  0.9% |  51.3 | `storeAppliedImpulses`                  | `src/constraints/contact-constraints.ts:1359`                |
|  25 |  0.8% |  50.8 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  26 |  0.8% |  50.6 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                            |
|  27 |  0.7% |  39.9 | `multiply3x3TransposedVec`              | `…/node_modules/mathcat/dist/mat4.js:603`                    |
|  28 |  0.6% |  35.3 | `makeSyncRequest`                       | `node:internal/modules/esm/hooks`                            |
|  29 |  0.6% |  34.7 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:475`                 |
|  30 |  0.6% |  34.7 | `(idle)`                                | ``                                                           |

