# perf report — joints

| field | value |
| --- | --- |
| scenario | `joints` |
| date | 2026-07-04T08:03:59.600Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 839.2 ms |
| attributed | 641.8 ms |
| startup excluded | 197 ms |
| samples | 689 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 48.1% | 308.4 |
| body          | 14.7% |  94.1 |
| math          | 14.6% |  93.5 |
| step          | 13.1% |  84.3 |
| broadphase    |  3.4% |  22.0 |
| crashcat-util |  2.9% |  18.8 |
| runtime       |  2.1% |  13.4 |
| bench-harness |  0.7% |   4.6 |
| narrowphase   |  0.2% |   1.3 |
| other         |  0.2% |   1.2 |
| shapes        |  0.0% |   0.3 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                              | location                                                                |
| --: | ---: | ---: | ------------------------------------- | ----------------------------------------------------------------------- |
|   1 | 9.2% | 59.0 | `getInverseInertiaForRotation`        | `src/body/motion-properties.ts:500`                                     |
|   2 | 9.0% | 57.5 | `applyVelocityStep$2`                 | `src/constraints/constraint-part/point-constraint-part.ts:255`          |
|   3 | 8.9% | 57.1 | `solveVelocityConstraint$5`           | `src/constraints/constraint-part/point-constraint-part.ts:204`          |
|   4 | 7.8% | 50.4 | `updateWorld`                         | `src/update.ts:46`                                                      |
|   5 | 3.9% | 25.3 | `multiply3x3`                         | `…/node_modules/mathcat/dist/mat4.js:510`                               |
|   6 | 3.7% | 23.8 | `calculateConstraintProperties$6`     | `src/constraints/constraint-part/point-constraint-part.ts:77`           |
|   7 | 3.0% | 19.0 | `solveVelocity$4`                     | `src/constraints/hinge-constraint.ts:672`                               |
|   8 | 2.8% | 17.8 | `fromQuat$1`                          | `…/node_modules/mathcat/dist/mat4.js:1518`                              |
|   9 | 2.6% | 16.4 | `multiply3x3RightTransposed`          | `…/node_modules/mathcat/dist/mat4.js:558`                               |
|  10 | 2.5% | 16.0 | `solvePositionConstraint$5`           | `src/constraints/constraint-part/point-constraint-part.ts:307`          |
|  11 | 2.4% | 15.5 | `update$11`                           | `src/broadphase/dbvt.ts:480`                                            |
|  12 | 2.3% | 15.0 | `findCollidingPairs`                  | `src/pairs.ts:409`                                                      |
|  13 | 2.3% | 14.9 | `solveVelocityConstraintsForIsland`   | `src/constraints/contact-constraints.ts:1127`                           |
|  14 | 2.3% | 14.5 | `updatePositionFromCenterOfMass`      | `src/body/rigid-body.ts:555`                                            |
|  15 | 2.1% | 13.8 | `solveVelocity`                       | `src/constraints/swing-twist-constraint.ts:684`                         |
|  16 | 2.1% | 13.7 | `updateAABB`                          | `src/body/rigid-body.ts:580`                                            |
|  17 | 1.8% | 11.3 | `calculateConstraintProperties$2`     | `src/constraints/constraint-part/hinge-rotation-constraint-part.ts:105` |
|  18 | 1.6% | 10.1 | `velocityIntegrationUpdate`           | `src/update.ts:1181`                                                    |
|  19 | 1.4% |  8.7 | `add$2`                               | `…/node_modules/mathcat/dist/mat4.js:2039`                              |
|  20 | 1.2% |  7.8 | `multiply3x3Vec`                      | `…/node_modules/mathcat/dist/mat4.js:620`                               |
|  21 | 1.2% |  7.6 | `accelerationIntegrationUpdate`       | `src/update.ts:293`                                                     |
|  22 | 1.2% |  7.4 | `(garbage collector)`                 | ``                                                                      |
|  23 | 1.0% |  6.4 | `finalize`                            | `src/islands.ts:240`                                                    |
|  24 | 1.0% |  6.3 | `solveVelocityConstraintsForIsland$1` | `src/constraints/constraints.ts:303`                                    |
|  25 | 1.0% |  6.2 | `addContactConstraint`                | `src/constraints/contact-constraints.ts:583`                            |
|  26 | 1.0% |  6.1 | `solvePosition`                       | `src/constraints/swing-twist-constraint.ts:754`                         |
|  27 | 0.8% |  5.1 | `resetForces`                         | `src/update.ts:1976`                                                    |
|  28 | 0.8% |  5.0 | `reconstructManifoldFromCache`        | `src/update.ts:845`                                                     |
|  29 | 0.8% |  5.0 | `solveVelocity$3`                     | `src/constraints/point-constraint.ts:216`                               |
|  30 | 0.8% |  5.0 | `solvePosition$4`                     | `src/constraints/hinge-constraint.ts:748`                               |

