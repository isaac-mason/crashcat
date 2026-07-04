# perf report — collapse

| field | value |
| --- | --- |
| scenario | `collapse` |
| date | 2026-07-04T08:04:55.249Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1016.2 ms |
| attributed | 854.8 ms |
| startup excluded | 161.3 ms |
| samples | 862 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 35.8% | 305.6 |
| narrowphase   | 24.0% | 205.3 |
| step          | 10.2% |  87.0 |
| broadphase    |  7.6% |  64.6 |
| body          |  5.2% |  44.1 |
| shapes        |  4.3% |  36.4 |
| runtime       |  3.8% |  32.5 |
| math          |  3.3% |  28.3 |
| manifold      |  3.0% |  26.0 |
| crashcat-util |  2.5% |  21.3 |
| bench-harness |  0.3% |   2.6 |
| other         |  0.1% |   1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                                | location                                                     |
| --: | ----: | ----: | --------------------------------------- | ------------------------------------------------------------ |
|   1 | 17.0% | 145.1 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1127`                |
|   2 |  7.3% |  62.5 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1246`                                  |
|   3 |  5.3% |  45.4 | `updateWorld`                           | `src/update.ts:46`                                           |
|   4 |  4.6% |  39.6 | `intersectAABBFatLeaves`                | `src/broadphase/dbvt.ts:590`                                 |
|   5 |  4.3% |  37.2 | `getSupport`                            | `src/collision/support.ts:125`                               |
|   6 |  4.0% |  34.2 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:583`                 |
|   7 |  3.3% |  28.5 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1405`                |
|   8 |  3.2% |  27.6 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                            |
|   9 |  3.2% |  27.5 | `(garbage collector)`                   | ``                                                           |
|  10 |  2.3% |  19.5 | `penetrationDepthStepEPA`               | `src/collision/penetration.ts:209`                           |
|  11 |  2.2% |  18.8 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                          |
|  12 |  1.9% |  16.6 | `update$11`                             | `src/broadphase/dbvt.ts:480`                                 |
|  13 |  1.9% |  15.8 | `findCollidingPairs`                    | `src/pairs.ts:409`                                           |
|  14 |  1.8% |  15.4 | `createTriangle$1`                      | `src/collision/epa-convex-hull-builder.ts:251`               |
|  15 |  1.8% |  15.2 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:977`                 |
|  16 |  1.6% |  13.6 | `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                   |
|  17 |  1.5% |  13.2 | `addPoint$1`                            | `src/collision/epa-convex-hull-builder.ts:629`               |
|  18 |  1.5% |  13.2 | `findEdge$1`                            | `src/collision/epa-convex-hull-builder.ts:548`               |
|  19 |  1.5% |  12.8 | `clipPolyVsPlane`                       | `src/manifold/clip.ts:13`                                    |
|  20 |  1.4% |  11.5 | `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                   |
|  21 |  1.2% |  10.5 | `narrowphase`                           | `src/update.ts:998`                                          |
|  22 |  1.2% |  10.3 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  23 |  1.2% |  10.0 | `calculateNormalVelocityBias`           | `src/constraints/contact-constraints.ts:310`                 |
|  24 |  1.0% |   8.9 | `sortContactIndices`                    | `src/constraints/contact-constraints.ts:1547`                |
|  25 |  0.9% |   7.5 | `addEpaSupportPoint`                    | `src/collision/penetration.ts:150`                           |
|  26 |  0.9% |   7.3 | `reconstructManifoldFromCache`          | `src/update.ts:845`                                          |
|  27 |  0.8% |   7.1 | `finalizeAndCreateConstraints`          | `src/update.ts:600`                                          |
|  28 |  0.8% |   6.8 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:476`                 |
|  29 |  0.7% |   6.3 | `getVelocityAtPointCOM`                 | `src/body/rigid-body.ts:1035`                                |
|  30 |  0.7% |   6.2 | `getShapeSupportingFace`                | `src/shapes/shapes.ts:489`                                   |

