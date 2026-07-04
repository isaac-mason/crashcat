# perf report — wake-waves

| field | value |
| --- | --- |
| scenario | `wake-waves` |
| date | 2026-07-04T08:04:58.785Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 738.6 ms |
| attributed | 569.4 ms |
| startup excluded | 168.2 ms |
| samples | 643 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 40.4% | 229.8 |
| narrowphase   | 23.2% | 132.2 |
| step          |  8.5% |  48.6 |
| broadphase    |  6.5% |  37.1 |
| math          |  5.1% |  29.0 |
| body          |  5.0% |  28.5 |
| crashcat-util |  3.6% |  20.7 |
| shapes        |  3.4% |  19.4 |
| runtime       |  1.8% |  10.3 |
| manifold      |  1.3% |   7.7 |
| bench-harness |  0.6% |   3.2 |
| ccd           |  0.3% |   1.5 |
| other         |  0.2% |   1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                            | location                                                     |
| --: | ----: | ----: | ----------------------------------- | ------------------------------------------------------------ |
|   1 | 20.7% | 118.1 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1127`                |
|   2 |  9.1% |  51.7 | `penetrationDepthStepGJK`           | `src/collision/penetration.ts:29`                            |
|   3 |  7.7% |  43.9 | `gjkClosestPoints`                  | `src/collision/gjk.ts:1246`                                  |
|   4 |  6.0% |  34.0 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:583`                 |
|   5 |  5.1% |  29.2 | `solvePositionConstraintsForIsland` | `src/constraints/contact-constraints.ts:1405`                |
|   6 |  3.4% |  19.2 | `updateWorld`                       | `src/update.ts:46`                                           |
|   7 |  2.5% |  14.4 | `warmStartVelocityConstraints`      | `src/constraints/contact-constraints.ts:977`                 |
|   8 |  2.5% |  14.0 | `intersectAABBFatLeaves`            | `src/broadphase/dbvt.ts:590`                                 |
|   9 |  2.4% |  13.8 | `findCollidingPairs`                | `src/pairs.ts:409`                                           |
|  10 |  2.2% |  12.7 | `getSupport`                        | `src/collision/support.ts:125`                               |
|  11 |  2.2% |  12.4 | `update$11`                         | `src/broadphase/dbvt.ts:480`                                 |
|  12 |  1.8% |  10.1 | `getVelocityAtPointCOM`             | `src/body/rigid-body.ts:1035`                                |
|  13 |  1.6% |   8.8 | `storeAppliedImpulses`              | `src/constraints/contact-constraints.ts:1365`                |
|  14 |  1.5% |   8.8 | `reconstructManifoldFromCache`      | `src/update.ts:845`                                          |
|  15 |  1.4% |   8.3 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`                          |
|  16 |  1.4% |   7.7 | `penetrationDepthStepEPA`           | `src/collision/penetration.ts:209`                           |
|  17 |  1.3% |   7.5 | `multiply3x3TransposedVec`          | `…/node_modules/mathcat/dist/mat4.js:603`                    |
|  18 |  1.3% |   7.5 | `createTriangle$1`                  | `src/collision/epa-convex-hull-builder.ts:251`               |
|  19 |  1.2% |   6.8 | `(garbage collector)`               | ``                                                           |
|  20 |  1.1% |   6.5 | `finalizeAndCreateConstraints`      | `src/update.ts:600`                                          |
|  21 |  1.1% |   6.4 | `collideConvexVsConvexLocal`        | `src/shapes/convex.ts:329`                                   |
|  22 |  1.1% |   6.2 | `narrowphase`                       | `src/update.ts:998`                                          |
|  23 |  1.0% |   5.7 | `getSupportingFace$11`              | `src/shapes/box.ts:157`                                      |
|  24 |  0.8% |   4.8 | `collideConvexVsConvex`             | `src/shapes/convex.ts:247`                                   |
|  25 |  0.7% |   3.9 | `transformMat4$1`                   | `…/node_modules/mathcat/dist/vec3.js:530`                    |
|  26 |  0.7% |   3.8 | `calculateInverseEffectiveMass`     | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  27 |  0.6% |   3.7 | `updateSleepState`                  | `src/body/sleep.ts:89`                                       |
|  28 |  0.6% |   3.6 | `getSleepTestPoints`                | `src/body/sleep.ts:22`                                       |
|  29 |  0.5% |   2.6 | `velocityIntegrationUpdate`         | `src/update.ts:1181`                                         |
|  30 |  0.4% |   2.5 | `combineMaterial`                   | `src/constraints/combine-material.ts:40`                     |

