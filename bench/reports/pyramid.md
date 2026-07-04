# perf report — pyramid

| field | value |
| --- | --- |
| scenario | `pyramid` |
| date | 2026-07-04T08:04:22.916Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 14705.5 ms |
| attributed | 14516.2 ms |
| startup excluded | 188.4 ms |
| samples | 11609 |

## by category

| category      |   pct |      ms |
| ------------- | ----: | ------: |
| solver        | 84.6% | 12281.1 |
| body          |  6.3% |   913.1 |
| step          |  3.4% |   498.9 |
| narrowphase   |  2.1% |   307.1 |
| math          |  1.9% |   281.7 |
| crashcat-util |  0.6% |    90.3 |
| runtime       |  0.3% |    44.6 |
| broadphase    |  0.3% |    44.2 |
| shapes        |  0.2% |    27.9 |
| manifold      |  0.1% |    10.0 |
| bench-harness |  0.1% |     8.9 |
| ccd           |  0.0% |     6.0 |
| other         |  0.0% |     2.5 |

## top 30 self-time hotspots

|   # |   pct |     ms | function                                  | location                                                      |
| --: | ----: | -----: | ----------------------------------------- | ------------------------------------------------------------- |
|   1 | 56.3% | 8178.0 | `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1127`                 |
|   2 | 10.3% | 1490.8 | `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1405`                 |
|   3 |  6.1% |  881.2 | `addContactConstraint`                    | `src/constraints/contact-constraints.ts:583`                  |
|   4 |  3.6% |  520.5 | `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                           |
|   5 |  3.2% |  461.3 | `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:977`                  |
|   6 |  2.3% |  334.2 | `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1035`                                 |
|   7 |  1.9% |  277.1 | `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`  |
|   8 |  1.9% |  275.3 | `reconstructManifoldFromCache`            | `src/update.ts:845`                                           |
|   9 |  1.2% |  176.3 | `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1365`                 |
|  10 |  1.0% |  145.8 | `linkContactConstraints`                  | `src/islands.ts:184`                                          |
|  11 |  0.9% |  131.1 | `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1547`                 |
|  12 |  0.9% |  123.5 | `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:310`                  |
|  13 |  0.9% |  123.4 | `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                     |
|  14 |  0.8% |  111.8 | `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:476`                  |
|  15 |  0.8% |  110.7 | (anonymous)                               | `src/constraints/contact-constraints.ts:1548`                 |
|  16 |  0.6% |   92.4 | `updateWorld`                             | `src/update.ts:46`                                            |
|  17 |  0.6% |   82.5 | `findCollidingPairs`                      | `src/pairs.ts:409`                                            |
|  18 |  0.6% |   81.6 | `getContactsFromCache`                    | `src/update.ts:914`                                           |
|  19 |  0.4% |   63.8 | `fromQuat$1`                              | `…/node_modules/mathcat/dist/mat4.js:1518`                    |
|  20 |  0.4% |   58.6 | `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:629`                |
|  21 |  0.4% |   51.8 | `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                            |
|  22 |  0.4% |   51.2 | `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810` |
|  23 |  0.3% |   50.5 | `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                |
|  24 |  0.2% |   34.8 | `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                             |
|  25 |  0.2% |   33.8 | `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:548`                |
|  26 |  0.2% |   32.8 | `(garbage collector)`                     | ``                                                            |
|  27 |  0.2% |   32.3 | `combineMaterial`                         | `src/constraints/combine-material.ts:40`                      |
|  28 |  0.2% |   29.8 | `gjkClosestPoints`                        | `src/collision/gjk.ts:1246`                                   |
|  29 |  0.2% |   25.5 | `update$11`                               | `src/broadphase/dbvt.ts:480`                                  |
|  30 |  0.2% |   24.2 | `getSupport`                              | `src/collision/support.ts:125`                                |

