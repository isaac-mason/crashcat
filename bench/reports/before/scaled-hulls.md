# perf report — kitbash

| field | value |
| --- | --- |
| scenario | `kitbash` |
| date | 2026-07-03T04:33:45.708Z |
| git rev | `58a218d` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 7805.4 ms |
| attributed | 7805.1 ms |
| samples | 6246 |

## by category

| category      |   pct |     ms |
| ------------- | ----: | -----: |
| narrowphase   | 51.5% | 4018.6 |
| solver        | 19.2% | 1500.7 |
| shapes        |  7.4% |  576.0 |
| step          |  6.6% |  512.1 |
| broadphase    |  6.0% |  469.7 |
| runtime       |  3.0% |  237.0 |
| math          |  2.5% |  191.8 |
| body          |  2.2% |  169.3 |
| manifold      |  1.5% |  115.1 |
| bench-harness |  0.1% |    7.3 |
| other         |  0.0% |    3.8 |
| crashcat-util |  0.0% |    2.5 |
| ccd           |  0.0% |    1.3 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                                | location                                                     |
| --: | ----: | -----: | --------------------------------------- | ------------------------------------------------------------ |
|   1 | 32.2% | 2510.9 | `computeScaledShrunkHullPoints`         | `src/collision/support.ts:532`                               |
|   2 | 11.2% |  874.9 | `getSupport`                            | `src/collision/support.ts:123`                               |
|   3 |  8.2% |  638.3 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1121`                |
|   4 |  5.1% |  400.9 | `intersectAABB$1`                       | `src/broadphase/dbvt.ts:627`                                 |
|   5 |  4.4% |  344.0 | `getSupportingFace$8`                   | `src/shapes/convex-hull.ts:512`                              |
|   6 |  3.3% |  254.8 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1225`                                  |
|   7 |  3.1% |  240.8 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:582`                 |
|   8 |  2.5% |  192.4 | `updateWorld`                           | `src/update.ts:45`                                           |
|   9 |  1.8% |  144.0 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1399`                |
|  10 |  1.8% |  138.0 | `getContactsFromCache`                  | `src/update.ts:868`                                          |
|  11 |  1.5% |  114.1 | `narrowphase`                           | `src/update.ts:949`                                          |
|  12 |  0.9% |   69.2 | `findEdge$1`                            | `src/collision/epa-convex-hull-builder.ts:548`               |
|  13 |  0.9% |   69.0 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                          |
|  14 |  0.9% |   68.0 | `penetrationDepthStepEPA`               | `src/collision/penetration.ts:209`                           |
|  15 |  0.8% |   64.9 | `createTriangle$1`                      | `src/collision/epa-convex-hull-builder.ts:251`               |
|  16 |  0.8% |   64.3 | `getVelocityAtPointCOM`                 | `src/body/rigid-body.ts:1007`                                |
|  17 |  0.8% |   61.8 | `addPoint$1`                            | `src/collision/epa-convex-hull-builder.ts:629`               |
|  18 |  0.8% |   61.6 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:971`                 |
|  19 |  0.8% |   61.0 | `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                   |
|  20 |  0.8% |   59.6 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                            |
|  21 |  0.7% |   55.8 | `update$1`                              | `src/broadphase/dbvt.ts:485`                                 |
|  22 |  0.7% |   54.8 | `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                   |
|  23 |  0.7% |   52.0 | `clipPolyVsPlane`                       | `src/manifold/clip.ts:13`                                    |
|  24 |  0.6% |   50.2 | `multiply3x3TransposedVec`              | `…/node_modules/mathcat/dist/mat4.js:603`                    |
|  25 |  0.6% |   47.0 | `sortContactIndices`                    | `src/constraints/contact-constraints.ts:1541`                |
|  26 |  0.6% |   43.0 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  27 |  0.5% |   41.3 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:475`                 |
|  28 |  0.5% |   40.0 | `collideShapeVsScaled`                  | `src/shapes/scaled.ts:356`                                   |
|  29 |  0.5% |   36.4 | `findContact`                           | `src/contacts.ts:647`                                        |
|  30 |  0.5% |   36.2 | `setHullSupport`                        | `src/collision/support.ts:661`                               |

