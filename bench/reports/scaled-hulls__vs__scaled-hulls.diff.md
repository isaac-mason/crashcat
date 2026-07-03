# perf diff — kitbash → scaled-hulls

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `kitbash` | `58a218d` (dirty) | 7805.1 | 2026-07-03T04:33:45.708Z |
| current | `scaled-hulls` | `58a218d` (dirty) | 6018.1 | 2026-07-03T04:39:43.803Z |

**total attributed time:** 7805.1 ms → 6018.1 ms  (**-1787.0 ms**, -22.9%)

## category deltas

| category      | base ms | cur ms |    Δ ms | Δ pts |  rel % |
| ------------- | ------: | -----: | ------: | ----: | -----: |
| narrowphase   |  4018.6 | 1612.6 | -2406.0 | -24.7 | -59.9% |
| solver        |  1500.7 | 1719.0 |  +218.3 |  +9.4 | +14.5% |
| step          |   512.1 |  654.9 |  +142.8 |  +4.3 | +27.9% |
| broadphase    |   469.7 |  575.1 |  +105.4 |  +3.6 | +22.4% |
| body          |   169.3 |  243.3 |   +74.0 |  +1.8 | +43.7% |
| shapes        |   576.0 |  610.8 |   +34.8 |  +2.7 |  +6.0% |
| math          |   191.8 |  223.8 |   +32.0 |  +1.2 | +16.7% |
| runtime       |   237.0 |  258.3 |   +21.3 |  +1.3 |  +9.0% |
| manifold      |   115.1 |  107.7 |    -7.4 |  +0.3 |  -6.4% |
| other         |     3.8 |    1.3 |    -2.5 |  +0.0 | -65.8% |
| bench-harness |     7.3 |    6.1 |    -1.2 |  +0.0 | -16.4% |
| ccd           |     1.3 |    2.5 |    +1.2 |  +0.0 | +92.3% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                  | location                                                                 | base ms | cur ms |    Δ ms | note     |
| ----------------------------------------- | ------------------------------------------------------------------------ | ------: | -----: | ------: | -------- |
| `computeScaledShrunkHullPoints`           | `src/collision/support.ts:532`                                           |  2510.9 |    0.0 | -2510.9 | gone     |
| `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1121`                            |   638.3 |  757.2 |  +118.9 |          |
| `getSupport`                              | `src/collision/support.ts:125`                                           |   874.9 |  975.0 |  +100.1 |          |
| `intersectAABB$1`                         | `src/broadphase/dbvt.ts:627`                                             |   400.9 |  474.1 |   +73.2 |          |
| `addContactConstraint`                    | `src/constraints/contact-constraints.ts:582`                             |   240.8 |  295.4 |   +54.6 |          |
| `getContactsFromCache`                    | `src/update.ts:868`                                                      |   138.0 |  190.8 |   +52.8 |          |
| `updateWorld`                             | `src/update.ts:45`                                                       |   192.4 |  237.0 |   +44.6 |          |
| `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                                      |    69.0 |  111.4 |   +42.4 |          |
| `setHullSupport`                          | `src/collision/support.ts:677`                                           |    36.2 |    8.5 |   -27.7 |          |
| `collideShapeVsScaled`                    | `src/shapes/scaled.ts:360`                                               |    40.0 |   67.5 |   +27.5 |          |
| `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1359`                            |    28.8 |   51.3 |   +22.5 |          |
| `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1541`                            |    47.0 |   24.6 |   -22.4 |          |
| `(garbage collector)`                     | ``                                                                       |    31.5 |   53.8 |   +22.3 |          |
| `transformMat4$1`                         | `…/node_modules/mathcat/dist/vec3.js:530`                                |     0.0 |   21.3 |   +21.3 | appeared |
| `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1007`                                            |    64.3 |   83.3 |   +19.0 |          |
| `reconstructManifoldFromCache`            | `src/update.ts:799`                                                      |     0.0 |   18.3 |   +18.3 | appeared |
| `gjkClosestPoints`                        | `src/collision/gjk.ts:1225`                                              |   254.8 |  272.5 |   +17.7 |          |
| `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                           |    64.9 |   82.5 |   +17.6 |          |
| `findContact`                             | `src/contacts.ts:647`                                                    |    36.4 |   20.1 |   -16.3 |          |
| `update$1`                                | `src/broadphase/dbvt.ts:485`                                             |    55.8 |   71.3 |   +15.5 |          |
| `narrowphase`                             | `src/update.ts:949`                                                      |   114.1 |  129.5 |   +15.4 |          |
| `getSupportingFace$8`                     | `src/shapes/convex-hull.ts:529`                                          |   344.0 |  330.0 |   -14.0 |          |
| `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                               |    61.0 |   74.3 |   +13.3 |          |
| `fromRotationTranslation`                 | `…/node_modules/mathcat/dist/mat4.js:1150`                               |    19.0 |   32.2 |   +13.2 |          |
| `subRotationStep`                         | `src/body/rigid-body-step.ts:83`                                         |     0.0 |   12.3 |   +12.3 | appeared |
| `findCollidingPairs`                      | `src/broadphase/broadphase.ts:213`                                       |     8.9 |   20.8 |   +11.9 |          |
| `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                               |    54.8 |   66.4 |   +11.6 |          |
| `setCachedBodyPair`                       | `src/contacts.ts:174`                                                    |    11.5 |   22.5 |   +11.0 |          |
| `collideScaledVsShape`                    | `src/shapes/scaled.ts:287`                                               |     0.0 |   11.0 |   +11.0 | appeared |
| `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1399`                            |   144.0 |  154.9 |   +10.9 |          |
| `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                                |    50.2 |   39.9 |   -10.3 |          |
| `calculateConstraintProperties`           | `src/constraints/constraint-part/angular-friction-constraint-part.ts:77` |     0.0 |   10.2 |   +10.2 | appeared |
| `cross`                                   | `…/node_modules/mathcat/dist/vec3.js:401`                                |    12.2 |   22.3 |   +10.1 |          |
| `combineMaterial`                         | `src/constraints/combine-material.ts:40`                                 |     5.0 |   15.0 |   +10.0 |          |
| `velocityIntegrationUpdate`               | `src/update.ts:1131`                                                     |    15.3 |   24.9 |    +9.6 |          |
| `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                                        |    59.6 |   50.6 |    -9.0 |          |
| `getOrthogonalBasis`                      | `src/shapes/plane.ts:211`                                                |     8.0 |    0.0 |    -8.0 | gone     |
| `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`             |    43.0 |   50.8 |    +7.8 |          |
| `normalize$2`                             | `…/node_modules/mathcat/dist/vec3.js:369`                                |     0.0 |    7.5 |    +7.5 | appeared |
| `getAdaptivePlaneSupportingFace`          | `src/shapes/plane.ts:636`                                                |     7.4 |    0.0 |    -7.4 | gone     |
| `transformPlane`                          | `src/shapes/plane.ts:197`                                                |     6.6 |    0.0 |    -6.6 | gone     |
| `manifoldBetweenTwoFaces`                 | `src/manifold/manifold.ts:394`                                           |    31.4 |   24.8 |    -6.6 |          |
| `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:475`                             |    41.3 |   34.7 |    -6.6 |          |
| `resetForces`                             | `src/update.ts:1914`                                                     |     0.0 |    6.5 |    +6.5 | appeared |
| `invert$2`                                | `…/node_modules/mathcat/dist/mat4.js:251`                                |     6.4 |    0.0 |    -6.4 | gone     |
| `prepare`                                 | `src/islands.ts:64`                                                      |     6.3 |    0.0 |    -6.3 | gone     |
| `insertLeaf`                              | `src/broadphase/dbvt.ts:118`                                             |     0.0 |    6.3 |    +6.3 | appeared |
| `createContact`                           | `src/contacts.ts:373`                                                    |     0.0 |    6.3 |    +6.3 | appeared |
| `scalePlane`                              | `src/shapes/plane.ts:185`                                                |     0.0 |    6.2 |    +6.2 | appeared |
| `(program)`                               | ``                                                                       |    15.5 |    9.4 |    -6.1 |          |
| `lineLengths`                             | `node:internal/source_map/source_map_cache`                              |     6.1 |    0.0 |    -6.1 | gone     |
| `finalize`                                | `src/islands.ts:240`                                                     |    18.5 |   12.6 |    -5.9 |          |
| `scaleAndAdd`                             | `…/node_modules/mathcat/dist/vec3.js:292`                                |     5.2 |    0.0 |    -5.2 | gone     |
| `swapShapes`                              | `src/manifold/manifold.ts:86`                                            |     5.1 |    0.0 |    -5.1 | gone     |
| `computeBarycentricCoordinates3d`         | `src/collision/closest-points.ts:57`                                     |     5.0 |    0.0 |    -5.0 | gone     |
| `transformQuat`                           | `…/node_modules/mathcat/dist/vec3.js:567`                                |    18.9 |   23.8 |    +4.9 |          |
| `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:548`                           |    69.2 |   64.7 |    -4.5 |          |
| `getSleepTestPoints`                      | `src/body/sleep.ts:22`                                                   |     7.6 |   12.0 |    +4.4 |          |
| `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:971`                             |    61.6 |   65.2 |    +3.6 |          |
| `(idle)`                                  | ``                                                                       |    31.3 |   34.7 |    +3.4 |          |
| `collideConvexVsPlane`                    | `src/shapes/plane.ts:483`                                                |    20.2 |   23.6 |    +3.4 |          |
| `destroyContact`                          | `src/contacts.ts:435`                                                    |    11.3 |   14.6 |    +3.3 |          |
| `checkIslandSleep`                        | `src/islands.ts:378`                                                     |    19.3 |   22.5 |    +3.2 |          |
| `accelerationIntegrationUpdate`           | `src/update.ts:287`                                                      |    11.3 |   14.5 |    +3.2 |          |
| `initialize$2`                            | `src/collision/epa-convex-hull-builder.ts:419`                           |     6.0 |    9.1 |    +3.1 |          |
| `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:309`                             |    30.4 |   27.3 |    -3.1 |          |
| `clipPolyVsPoly`                          | `src/manifold/clip.ts:108`                                               |    15.3 |   12.3 |    -3.0 |          |
| `makeSyncRequest`                         | `node:internal/modules/esm/hooks`                                        |    32.7 |   35.3 |    +2.6 |          |
| `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                                |    52.0 |   54.5 |    +2.5 |          |
| `pruneContactPoints`                      | `src/manifold/manifold.ts:168`                                           |    11.3 |   13.8 |    +2.5 |          |
| `clamp`                                   | `…/node_modules/mathcat/dist/common.js:69`                               |     8.5 |    6.2 |    -2.3 |          |
| `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810`            |    21.1 |   23.4 |    +2.3 |          |
| `destroyUnprocessedContacts`              | `src/contacts.ts:700`                                                    |    10.2 |   12.3 |    +2.1 |          |
| `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                                       |    68.0 |   66.2 |    -1.8 |          |
| `destroyStaleContactsBetweenBodies`       | `src/contacts.ts:570`                                                    |     8.3 |   10.0 |    +1.7 |          |
| `normalize$1`                             | `…/node_modules/mathcat/dist/vec4.js:325`                                |     7.7 |    6.3 |    -1.4 |          |
| `compileSourceTextModule`                 | `node:internal/modules/esm/utils`                                        |    18.7 |   20.0 |    +1.3 |          |
| `setAxisAngle`                            | `…/node_modules/mathcat/dist/quat.js:36`                                 |    13.8 |   15.1 |    +1.3 |          |
| `updateBodyPositions`                     | `src/update.ts:371`                                                      |     8.6 |    7.5 |    -1.1 |          |
| `multiply$1`                              | `…/node_modules/mathcat/dist/mat4.js:448`                                |    11.0 |   12.0 |    +1.0 |          |
| `setContactSettings`                      | `src/constraints/contact-constraints.ts:243`                             |     7.6 |    8.5 |    +0.9 |          |
| `addHit`                                  | `src/update.ts:432`                                                      |    18.7 |   19.5 |    +0.8 |          |
| `waitForWorker`                           | `node:internal/modules/esm/hooks`                                        |    31.9 |   32.4 |    +0.5 |          |

