# perf report — scaled-hulls

| field | value |
| --- | --- |
| scenario | `scaled-hulls` |
| date | 2026-07-04T09:33:11.789Z |
| git rev | `7ee6b81` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 5553.6 ms |
| attributed | 5343.3 ms |
| startup excluded | 209.7 ms |
| samples | 4409 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| solver        | 31.2% | 1666.3 |
| narrowphase   | 29.7% | 1584.5 |
| shapes        | 10.6% |  566.0 |
| step          |  7.4% |  395.4 |
| broadphase    |  6.1% |  324.2 |
| body          |  4.6% |  247.5 |
| math          |  4.4% |  234.5 |
| manifold      |  2.2% |  119.0 |
| crashcat-util |  2.1% |  110.6 |
| runtime       |  1.3% |   71.9 |
| bench-harness |  0.3% |   18.4 |
| ccd           |  0.1% |    3.7 |
| other         |  0.0% |    1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                                | location                                                     |
| --: | ----: | ----: | --------------------------------------- | ------------------------------------------------------------ |
|   1 | 17.4% | 930.8 | `getSupport`                            | `src/collision/support.ts:125`                               |
|   2 | 14.3% | 765.0 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1127`                |
|   3 |  5.4% | 290.2 | `getSupportingFace$8`                   | `src/shapes/convex-hull.ts:531`                              |
|   4 |  4.9% | 264.4 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:583`                 |
|   5 |  4.8% | 258.5 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1246`                                  |
|   6 |  4.7% | 249.9 | `intersectAABBFatLeaves`                | `src/broadphase/dbvt.ts:590`                                 |
|   7 |  3.1% | 163.9 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1405`                |
|   8 |  2.6% | 139.0 | `narrowphase`                           | `src/update.ts:998`                                          |
|   9 |  2.6% | 136.8 | `updateWorld`                           | `src/update.ts:46`                                           |
|  10 |  2.1% | 112.5 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                          |
|  11 |  1.8% |  98.1 | `createTriangle$1`                      | `src/collision/epa-convex-hull-builder.ts:173`               |
|  12 |  1.8% |  97.4 | `findCollidingPairs`                    | `src/pairs.ts:409`                                           |
|  13 |  1.6% |  87.0 | `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                   |
|  14 |  1.6% |  85.8 | `findEdge$1`                            | `src/collision/epa-convex-hull-builder.ts:506`               |
|  15 |  1.6% |  83.0 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:977`                 |
|  16 |  1.4% |  74.1 | `getVelocityAtPointCOM`                 | `src/body/rigid-body.ts:1035`                                |
|  17 |  1.3% |  66.9 | `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                   |
|  18 |  1.2% |  66.3 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                            |
|  19 |  1.2% |  62.3 | `collideShapeVsScaled`                  | `src/shapes/scaled.ts:360`                                   |
|  20 |  1.1% |  60.1 | `update$11`                             | `src/broadphase/dbvt.ts:480`                                 |
|  21 |  1.0% |  51.5 | `sortContactIndices`                    | `src/constraints/contact-constraints.ts:1547`                |
|  22 |  1.0% |  50.9 | `multiply3x3TransposedVec`              | `…/node_modules/mathcat/dist/mat4.js:603`                    |
|  23 |  0.9% |  50.5 | `clipPolyVsPlane`                       | `src/manifold/clip.ts:13`                                    |
|  24 |  0.9% |  50.3 | `addPoint$1`                            | `src/collision/epa-convex-hull-builder.ts:596`               |
|  25 |  0.9% |  49.8 | `penetrationDepthStepEPA`               | `src/collision/penetration.ts:210`                           |
|  26 |  0.9% |  49.5 | `(garbage collector)`                   | ``                                                           |
|  27 |  0.9% |  46.1 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  28 |  0.9% |  46.0 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:476`                 |
|  29 |  0.7% |  40.0 | `calculateNormalVelocityBias`           | `src/constraints/contact-constraints.ts:310`                 |
|  30 |  0.7% |  39.7 | `storeAppliedImpulses`                  | `src/constraints/contact-constraints.ts:1365`                |

