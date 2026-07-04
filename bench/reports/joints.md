# perf report — joints

| field | value |
| --- | --- |
| scenario | `joints` |
| date | 2026-07-03T15:46:31.646Z |
| git rev | `0252042` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 923 ms |
| attributed | 734.9 ms |
| startup excluded | 187.7 ms |
| samples | 809 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 47.5% | 349.4 |
| math          | 15.2% | 111.4 |
| step          | 13.0% |  95.6 |
| broadphase    | 12.1% |  89.3 |
| body          |  9.6% |  70.7 |
| runtime       |  1.2% |   8.9 |
| bench-harness |  0.8% |   6.0 |
| narrowphase   |  0.3% |   2.4 |
| ccd           |  0.2% |   1.3 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                            | location                                                                |
| --: | ---: | ---: | ----------------------------------- | ----------------------------------------------------------------------- |
|   1 | 7.7% | 56.8 | `intersectAABB$1`                   | `src/broadphase/dbvt.ts:577`                                            |
|   2 | 7.7% | 56.6 | `solveVelocityConstraint$5`         | `src/constraints/constraint-part/point-constraint-part.ts:204`          |
|   3 | 7.3% | 53.6 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`                                     |
|   4 | 7.3% | 53.3 | `updateWorld`                       | `src/update.ts:45`                                                      |
|   5 | 7.1% | 52.0 | `applyVelocityStep$2`               | `src/constraints/constraint-part/point-constraint-part.ts:255`          |
|   6 | 4.8% | 35.3 | `multiply3x3`                       | `…/node_modules/mathcat/dist/mat4.js:510`                               |
|   7 | 4.2% | 31.2 | `calculateConstraintProperties$6`   | `src/constraints/constraint-part/point-constraint-part.ts:77`           |
|   8 | 2.5% | 18.5 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1121`                           |
|   9 | 2.5% | 18.3 | `update$1`                          | `src/broadphase/dbvt.ts:475`                                            |
|  10 | 2.5% | 18.2 | `solveVelocity`                     | `src/constraints/swing-twist-constraint.ts:677`                         |
|  11 | 2.4% | 17.5 | `fromQuat$1`                        | `…/node_modules/mathcat/dist/mat4.js:1518`                              |
|  12 | 2.3% | 17.0 | `accelerationIntegrationUpdate`     | `src/update.ts:290`                                                     |
|  13 | 2.3% | 16.8 | `solveVelocity$4`                   | `src/constraints/hinge-constraint.ts:665`                               |
|  14 | 2.3% | 16.7 | `add$2`                             | `…/node_modules/mathcat/dist/mat4.js:2039`                              |
|  15 | 2.1% | 15.4 | `solvePositionConstraint$5`         | `src/constraints/constraint-part/point-constraint-part.ts:307`          |
|  16 | 1.8% | 13.6 | `multiply3x3RightTransposed`        | `…/node_modules/mathcat/dist/mat4.js:558`                               |
|  17 | 1.7% | 12.8 | `updateAABB`                        | `src/body/rigid-body.ts:571`                                            |
|  18 | 1.4% | 10.0 | `calculateConstraintProperties$3`   | `src/constraints/constraint-part/hinge-rotation-constraint-part.ts:105` |
|  19 | 1.3% |  9.7 | `solvePosition`                     | `src/constraints/swing-twist-constraint.ts:747`                         |
|  20 | 1.3% |  9.2 | `multiply3x3Vec`                    | `…/node_modules/mathcat/dist/mat4.js:620`                               |
|  21 | 1.2% |  9.0 | `solvePosition$4`                   | `src/constraints/hinge-constraint.ts:741`                               |
|  22 | 1.2% |  8.6 | `findCollidingPairs`                | `src/broadphase/broadphase.ts:218`                                      |
|  23 | 1.2% |  8.5 | `velocityIntegrationUpdate`         | `src/update.ts:1134`                                                    |
|  24 | 1.1% |  8.0 | `invert3x3`                         | `…/node_modules/mathcat/dist/mat4.js:313`                               |
|  25 | 1.0% |  7.2 | `solvePositionConstraintsForIsland` | `src/constraints/contact-constraints.ts:1399`                           |
|  26 | 1.0% |  7.0 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:582`                            |
|  27 | 0.9% |  6.7 | `solvePosition$3`                   | `src/constraints/point-constraint.ts:219`                               |
|  28 | 0.8% |  5.8 | `(garbage collector)`               | ``                                                                      |
|  29 | 0.8% |  5.8 | `prepare`                           | `src/islands.ts:64`                                                     |
|  30 | 0.8% |  5.5 | `reconstructManifoldFromCache`      | `src/update.ts:802`                                                     |

