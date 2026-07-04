# perf report — hull-heap

| field | value |
| --- | --- |
| scenario | `hull-heap` |
| date | 2026-07-04T09:53:13.556Z |
| git rev | `e8316a4` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 10165.8 ms |
| attributed | 9962.8 ms |
| startup excluded | 202.9 ms |
| samples | 8046 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| narrowphase   | 39.5% | 3934.5 |
| solver        | 29.1% | 2900.9 |
| shapes        | 10.2% | 1011.9 |
| step          |  6.4% |  641.8 |
| body          |  4.2% |  419.1 |
| broadphase    |  3.4% |  343.3 |
| math          |  2.8% |  282.7 |
| manifold      |  1.7% |  167.1 |
| crashcat-util |  1.6% |  160.8 |
| runtime       |  0.8% |   81.2 |
| bench-harness |  0.1% |   11.0 |
| ccd           |  0.1% |    6.1 |
| other         |  0.0% |    2.3 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                                | location                                                     |
| --: | ----: | -----: | --------------------------------------- | ------------------------------------------------------------ |
|   1 | 22.8% | 2269.4 | `getSupport`                            | `src/collision/support.ts:139`                               |
|   2 | 13.7% | 1367.7 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1127`                |
|   3 |  6.4% |  641.4 | `getSupportingFace$8`                   | `src/shapes/convex-hull.ts:576`                              |
|   4 |  4.7% |  470.0 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1246`                                  |
|   5 |  4.4% |  438.5 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:583`                 |
|   6 |  3.2% |  317.5 | `createTriangle$1`                      | `src/collision/epa-convex-hull-builder.ts:173`               |
|   7 |  2.8% |  275.6 | `intersectAABBFatLeaves`                | `src/broadphase/dbvt.ts:590`                                 |
|   8 |  2.7% |  272.3 | `findEdge$1`                            | `src/collision/epa-convex-hull-builder.ts:506`               |
|   9 |  2.6% |  255.1 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1405`                |
|  10 |  2.3% |  228.6 | `narrowphase`                           | `src/update.ts:998`                                          |
|  11 |  2.2% |  215.7 | `penetrationDepthStepEPA`               | `src/collision/penetration.ts:210`                           |
|  12 |  1.8% |  179.5 | `addPoint$1`                            | `src/collision/epa-convex-hull-builder.ts:596`               |
|  13 |  1.8% |  176.5 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:977`                 |
|  14 |  1.7% |  165.9 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                          |
|  15 |  1.6% |  160.9 | `updateWorld`                           | `src/update.ts:46`                                           |
|  16 |  1.5% |  154.4 | `findCollidingPairs`                    | `src/pairs.ts:409`                                           |
|  17 |  1.4% |  134.6 | `getVelocityAtPointCOM`                 | `src/body/rigid-body.ts:1035`                                |
|  18 |  1.3% |  125.8 | `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                   |
|  19 |  1.2% |  115.4 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                            |
|  20 |  1.1% |  109.3 | `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                   |
|  21 |  1.0% |   95.9 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  22 |  0.9% |   90.2 | `storeAppliedImpulses`                  | `src/constraints/contact-constraints.ts:1365`                |
|  23 |  0.9% |   84.7 | `clipPolyVsPlane`                       | `src/manifold/clip.ts:13`                                    |
|  24 |  0.8% |   79.7 | `getContactsFromCache`                  | `src/update.ts:914`                                          |
|  25 |  0.8% |   78.0 | `sortContactIndices`                    | `src/constraints/contact-constraints.ts:1547`                |
|  26 |  0.6% |   62.7 | `calculateNormalVelocityBias`           | `src/constraints/contact-constraints.ts:310`                 |
|  27 |  0.6% |   61.7 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:476`                 |
|  28 |  0.6% |   59.7 | `normalize$2`                           | `…/node_modules/mathcat/dist/vec3.js:369`                    |
|  29 |  0.6% |   56.8 | `(garbage collector)`                   | ``                                                           |
|  30 |  0.5% |   53.7 | `getShapeSupportingFace`                | `src/shapes/shapes.ts:489`                                   |

