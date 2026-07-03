# perf report — joints

| field | value |
| --- | --- |
| scenario | `joints` |
| date | 2026-07-03T05:02:21.500Z |
| git rev | `ab84e99` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1029.4 ms |
| attributed | 1029.1 ms |
| samples | 837 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 37.8% | 389.2 |
| runtime       | 20.4% | 210.1 |
| math          | 11.8% | 121.4 |
| broadphase    | 11.8% | 121.2 |
| body          |  9.2% |  94.5 |
| step          |  7.6% |  78.1 |
| bench-harness |  0.9% |   9.5 |
| other         |  0.2% |   2.5 |
| ccd           |  0.1% |   1.3 |
| crashcat-util |  0.1% |   1.2 |
| shapes        |  0.0% |   0.2 |

## top 30 self-time hotspots

|   # |  pct |   ms | function                            | location                                                       |
| --: | ---: | ---: | ----------------------------------- | -------------------------------------------------------------- |
|   1 | 7.5% | 77.3 | `intersectAABB$1`                   | `src/broadphase/dbvt.ts:627`                                   |
|   2 | 7.3% | 75.1 | `solveVelocityConstraint$5`         | `src/constraints/constraint-part/point-constraint-part.ts:204` |
|   3 | 6.8% | 69.8 | `applyVelocityStep$2`               | `src/constraints/constraint-part/point-constraint-part.ts:255` |
|   4 | 6.6% | 67.5 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`                            |
|   5 | 4.0% | 41.4 | `calculateConstraintProperties$6`   | `src/constraints/constraint-part/point-constraint-part.ts:77`  |
|   6 | 3.9% | 40.3 | `updateWorld`                       | `src/update.ts:45`                                             |
|   7 | 3.7% | 38.0 | `makeSyncRequest`                   | `node:internal/modules/esm/hooks`                              |
|   8 | 3.0% | 31.3 | `waitForWorker`                     | `node:internal/modules/esm/hooks`                              |
|   9 | 2.7% | 28.3 | `(idle)`                            | ``                                                             |
|  10 | 2.6% | 27.2 | `multiply3x3`                       | `…/node_modules/mathcat/dist/mat4.js:510`                      |
|  11 | 2.4% | 24.4 | `add$2`                             | `…/node_modules/mathcat/dist/mat4.js:2039`                     |
|  12 | 2.1% | 21.9 | `findCollidingPairs`                | `src/broadphase/broadphase.ts:213`                             |
|  13 | 2.1% | 21.2 | `solveVelocity$4`                   | `src/constraints/hinge-constraint.ts:665`                      |
|  14 | 1.9% | 19.9 | `compileSourceTextModule`           | `node:internal/modules/esm/utils`                              |
|  15 | 1.8% | 18.5 | `multiply3x3RightTransposed`        | `…/node_modules/mathcat/dist/mat4.js:558`                      |
|  16 | 1.7% | 17.1 | `updateAABB`                        | `src/body/rigid-body.ts:567`                                   |
|  17 | 1.6% | 16.2 | `multiply3x3Vec`                    | `…/node_modules/mathcat/dist/mat4.js:620`                      |
|  18 | 1.5% | 15.3 | `finalize`                          | `src/islands.ts:240`                                           |
|  19 | 1.5% | 15.2 | `velocityIntegrationUpdate`         | `src/update.ts:1131`                                           |
|  20 | 1.4% | 14.4 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1121`                  |
|  21 | 1.3% | 13.6 | `(garbage collector)`               | ``                                                             |
|  22 | 1.3% | 13.1 | `update$1`                          | `src/broadphase/dbvt.ts:485`                                   |
|  23 | 1.2% | 12.5 | `solveVelocity`                     | `src/constraints/swing-twist-constraint.ts:677`                |
|  24 | 1.2% | 12.4 | `fromQuat$1`                        | `…/node_modules/mathcat/dist/mat4.js:1518`                     |
|  25 | 1.2% | 12.2 | `invert3x3`                         | `…/node_modules/mathcat/dist/mat4.js:313`                      |
|  26 | 1.1% | 11.8 | `solvePositionConstraint$5`         | `src/constraints/constraint-part/point-constraint-part.ts:307` |
|  27 | 1.1% | 11.3 | `solvePosition$4`                   | `src/constraints/hinge-constraint.ts:741`                      |
|  28 | 1.0% | 10.5 | `(program)`                         | ``                                                             |
|  29 | 0.8% |  8.0 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:582`                   |
|  30 | 0.7% |  7.5 | `setupVelocity`                     | `src/constraints/swing-twist-constraint.ts:484`                |

