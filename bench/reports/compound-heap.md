# perf report — compound-heap

| field | value |
| --- | --- |
| scenario | `compound-heap` |
| date | 2026-07-04T08:05:00.825Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1177.7 ms |
| attributed | 987.8 ms |
| startup excluded | 189.3 ms |
| samples | 965 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| narrowphase   | 34.3% | 338.6 |
| solver        | 23.2% | 228.9 |
| shapes        | 16.8% | 166.2 |
| step          |  6.1% |  60.5 |
| body          |  4.5% |  44.6 |
| broadphase    |  4.2% |  41.6 |
| math          |  4.1% |  40.1 |
| runtime       |  2.5% |  24.7 |
| manifold      |  2.4% |  24.0 |
| crashcat-util |  1.4% |  13.6 |
| other         |  0.3% |   2.6 |
| bench-harness |  0.2% |   2.5 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                            | location                                                     |
| --: | ----: | ----: | ----------------------------------- | ------------------------------------------------------------ |
|   1 | 12.1% | 119.8 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1246`                                  |
|   2 |  9.3% |  92.1 | `getSupport`                        | `src/collision/support.ts:125`                               |
|   3 |  9.0% |  88.4 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1127`                |
|   4 |  6.3% |  62.1 | `collideCompoundVsShape`            | `src/shapes/compound.ts:518`                                 |
|   5 |  5.2% |  50.9 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:583`                 |
|   6 |  4.8% |  47.1 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`                            |
|   7 |  4.6% |  45.0 | `collideConvexVsConvex`             | `src/shapes/convex.ts:247`                                   |
|   8 |  3.2% |  31.3 | `collideConvexVsConvexLocal`        | `src/shapes/convex.ts:329`                                   |
|   9 |  2.8% |  27.5 | `updateWorld`                       | `src/update.ts:46`                                           |
|  10 |  2.7% |  26.5 | `solvePositionConstraintsForIsland` | `src/constraints/contact-constraints.ts:1405`                |
|  11 |  2.0% |  20.2 | `collideShapeVsCompound`            | `src/shapes/compound.ts:604`                                 |
|  12 |  1.9% |  19.0 | `update$11`                         | `src/broadphase/dbvt.ts:480`                                 |
|  13 |  1.8% |  17.7 | `setBoxSupport`                     | `src/collision/support.ts:296`                               |
|  14 |  1.7% |  16.3 | `intersectAABBFatLeaves`            | `src/broadphase/dbvt.ts:590`                                 |
|  15 |  1.6% |  16.0 | `(garbage collector)`               | ``                                                           |
|  16 |  1.5% |  14.7 | `penetrationDepthStepEPA`           | `src/collision/penetration.ts:209`                           |
|  17 |  1.5% |  14.7 | `fromRotationTranslation`           | `…/node_modules/mathcat/dist/mat4.js:1150`                   |
|  18 |  1.4% |  13.8 | `calculateInverseEffectiveMass`     | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  19 |  1.4% |  13.7 | `addPoint$1`                        | `src/collision/epa-convex-hull-builder.ts:629`               |
|  20 |  1.3% |  12.6 | `clipPolyVsPlane`                   | `src/manifold/clip.ts:13`                                    |
|  21 |  1.1% |  11.1 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`                          |
|  22 |  1.1% |  11.0 | `findCollidingPairs`                | `src/pairs.ts:409`                                           |
|  23 |  1.0% |  10.0 | `findEdge$1`                        | `src/collision/epa-convex-hull-builder.ts:548`               |
|  24 |  0.9% |   8.9 | `updateAABB`                        | `src/body/rigid-body.ts:580`                                 |
|  25 |  0.7% |   7.4 | `addHit`                            | `src/update.ts:455`                                          |
|  26 |  0.7% |   7.3 | `getSleepTestPoints`                | `src/body/sleep.ts:22`                                       |
|  27 |  0.7% |   6.8 | `velocityIntegrationUpdate`         | `src/update.ts:1181`                                         |
|  28 |  0.6% |   6.4 | `createTriangle$1`                  | `src/collision/epa-convex-hull-builder.ts:251`               |
|  29 |  0.6% |   6.3 | `getVelocityAtPointCOM`             | `src/body/rigid-body.ts:1035`                                |
|  30 |  0.6% |   6.3 | `manifoldBetweenTwoFaces`           | `src/manifold/manifold.ts:394`                               |

