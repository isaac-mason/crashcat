# perf report — settle-sleep

| field | value |
| --- | --- |
| scenario | `settle-sleep` |
| date | 2026-07-04T06:45:24.178Z |
| git rev | `5f27936` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 601.5 ms |
| attributed | 400.5 ms |
| startup excluded | 200.1 ms |
| samples | 529 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 37.1% | 148.8 |
| narrowphase   | 15.4% |  61.9 |
| crashcat-util | 10.8% |  43.1 |
| step          |  8.2% |  33.0 |
| body          |  7.8% |  31.2 |
| broadphase    |  7.1% |  28.5 |
| math          |  3.9% |  15.8 |
| runtime       |  3.8% |  15.4 |
| shapes        |  2.4% |   9.6 |
| ccd           |  1.3% |   5.0 |
| manifold      |  0.9% |   3.8 |
| bench-harness |  0.8% |   3.4 |
| other         |  0.3% |   1.3 |

## top 30 self-time hotspots

|   # |   pct |   ms | function                                | location                                                     |
| --: | ----: | ---: | --------------------------------------- | ------------------------------------------------------------ |
|   1 | 17.2% | 68.7 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1127`                |
|   2 | 10.1% | 40.6 | `findCollidingPairs`                    | `src/pairs.ts:408`                                           |
|   3 |  6.7% | 26.8 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:583`                 |
|   4 |  5.6% | 22.5 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                            |
|   5 |  4.8% | 19.2 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1246`                                  |
|   6 |  3.4% | 13.6 | `updateWorld`                           | `src/update.ts:46`                                           |
|   7 |  2.5% | 10.2 | `(garbage collector)`                   | ``                                                           |
|   8 |  2.0% |  7.9 | `prepare`                               | `src/islands.ts:64`                                          |
|   9 |  1.9% |  7.5 | `updateSleepState`                      | `src/body/sleep.ts:89`                                       |
|  10 |  1.6% |  6.3 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                          |
|  11 |  1.5% |  6.1 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  12 |  1.5% |  5.9 | `fromQuat$1`                            | `…/node_modules/mathcat/dist/mat4.js:1518`                   |
|  13 |  1.4% |  5.7 | `velocityIntegrationUpdate`             | `src/update.ts:1181`                                         |
|  14 |  1.3% |  5.1 | `optimizeIncremental`                   | `src/broadphase/dbvt.ts:544`                                 |
|  15 |  1.3% |  5.0 | `clear`                                 | `src/ccd.ts:107`                                             |
|  16 |  1.2% |  5.0 | `intersectAABBFatLeaves`                | `src/broadphase/dbvt.ts:590`                                 |
|  17 |  1.2% |  4.8 | `storeAppliedImpulses`                  | `src/constraints/contact-constraints.ts:1365`                |
|  18 |  1.2% |  4.7 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:476`                 |
|  19 |  1.1% |  4.3 | `update$11`                             | `src/broadphase/dbvt.ts:480`                                 |
|  20 |  1.0% |  4.1 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:977`                 |
|  21 |  1.0% |  4.0 | `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                   |
|  22 |  1.0% |  3.9 | `getSleepTestPoints`                    | `src/body/sleep.ts:22`                                       |
|  23 |  1.0% |  3.9 | `getSupport`                            | `src/collision/support.ts:125`                               |
|  24 |  1.0% |  3.8 | `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                   |
|  25 |  0.9% |  3.8 | `getContactsFromCache`                  | `src/update.ts:914`                                          |
|  26 |  0.9% |  3.8 | `(idle)`                                | ``                                                           |
|  27 |  0.9% |  3.8 | `createTriangle$1`                      | `src/collision/epa-convex-hull-builder.ts:251`               |
|  28 |  0.9% |  3.8 | `removeLeaf`                            | `src/broadphase/dbvt.ts:395`                                 |
|  29 |  0.9% |  3.7 | `insertLeaf`                            | `src/broadphase/dbvt.ts:111`                                 |
|  30 |  0.9% |  3.6 | `addEpaSupportPoint`                    | `src/collision/penetration.ts:150`                           |

