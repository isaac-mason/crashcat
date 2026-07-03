# perf diff — cube-heap → cube-heap

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `cube-heap` | `58a218d` (dirty) | 6538.5 | 2026-07-03T03:20:46.860Z |
| current | `cube-heap` | `58a218d` (dirty) | 6983.2 | 2026-07-03T04:39:50.928Z |

**total attributed time:** 6538.5 ms → 6983.2 ms  (**+444.7 ms**, +6.8%)

## category deltas

| category      | base ms | cur ms |   Δ ms | Δ pts |   rel % |
| ------------- | ------: | -----: | -----: | ----: | ------: |
| solver        |  2848.1 | 3156.8 | +308.7 |  +1.6 |  +10.8% |
| broadphase    |   555.1 |  638.6 |  +83.5 |  +0.6 |  +15.0% |
| narrowphase   |  1067.0 | 1104.5 |  +37.5 |  -0.5 |   +3.5% |
| shapes        |   273.0 |  302.0 |  +29.0 |  +0.1 |  +10.6% |
| runtime       |   248.7 |  231.3 |  -17.4 |  -0.5 |   -7.0% |
| math          |   262.8 |  276.8 |  +14.0 |  +0.0 |   +5.3% |
| step          |   680.9 |  669.6 |  -11.3 |  -0.8 |   -1.7% |
| manifold      |   232.2 |  225.5 |   -6.7 |  -0.4 |   -2.9% |
| ccd           |     2.6 |    6.1 |   +3.5 |  +0.1 | +134.6% |
| crashcat-util |     0.9 |    3.8 |   +2.9 |  +0.1 | +322.2% |
| bench-harness |     2.5 |    5.1 |   +2.6 |  +0.1 | +104.0% |
| other         |     3.8 |    1.3 |   -2.5 |  -0.1 |  -65.8% |
| body          |   360.8 |  362.0 |   +1.2 |  -0.3 |   +0.3% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                  | location                                                                 | base ms | cur ms |  Δ ms | note     |
| ----------------------------------------- | ------------------------------------------------------------------------ | ------: | -----: | ----: | -------- |
| `intersectAABB$1`                         | `src/broadphase/dbvt.ts:627`                                             |   452.2 |  541.0 | +88.8 |          |
| `addContactConstraint`                    | `src/constraints/contact-constraints.ts:582`                             |   383.8 |  459.4 | +75.6 |          |
| `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1399`                            |   300.3 |  348.5 | +48.2 |          |
| `getSupport`                              | `src/collision/support.ts:125`                                           |   275.5 |  316.8 | +41.3 |          |
| `narrowphase`                             | `src/update.ts:949`                                                      |   161.5 |  123.7 | -37.8 |          |
| `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                               |    52.2 |   80.7 | +28.5 |          |
| `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1121`                            |  1269.6 | 1297.7 | +28.1 |          |
| `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:309`                             |    38.5 |   65.8 | +27.3 |          |
| `findContact`                             | `src/contacts.ts:647`                                                    |    53.1 |   76.8 | +23.7 |          |
| `gjkClosestPoints`                        | `src/collision/gjk.ts:1225`                                              |   404.6 |  382.7 | -21.9 |          |
| `destroyContact`                          | `src/contacts.ts:435`                                                    |    19.4 |   40.7 | +21.3 |          |
| `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1007`                                            |   103.9 |  125.0 | +21.1 |          |
| `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:971`                             |   129.9 |  148.2 | +18.3 |          |
| `updateWorld`                             | `src/update.ts:45`                                                       |   230.8 |  247.7 | +16.9 |          |
| `setCachedBodyPair`                       | `src/contacts.ts:174`                                                    |    33.8 |   48.8 | +15.0 |          |
| `reconstructManifoldFromCache`            | `src/update.ts:799`                                                      |    21.1 |   36.0 | +14.9 |          |
| `normalize$2`                             | `…/node_modules/mathcat/dist/vec3.js:369`                                |     0.0 |   14.0 | +14.0 | appeared |
| `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1359`                            |    72.1 |   86.0 | +13.9 |          |
| `checkIslandSleep`                        | `src/islands.ts:378`                                                     |    21.5 |   34.5 | +13.0 |          |
| `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                               |   100.7 |   88.1 | -12.6 |          |
| `getShapeSupportingFace`                  | `src/shapes/shapes.ts:494`                                               |    29.7 |   41.6 | +11.9 |          |
| `cross`                                   | `…/node_modules/mathcat/dist/vec3.js:401`                                |     8.7 |   20.6 | +11.9 |          |
| `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810`            |    83.2 |   71.9 | -11.3 |          |
| `manifoldBetweenTwoFaces`                 | `src/manifold/manifold.ts:394`                                           |    55.4 |   44.4 | -11.0 |          |
| `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                                      |   182.4 |  171.8 | -10.6 |          |
| `clamp`                                   | `…/node_modules/mathcat/dist/common.js:69`                               |     0.0 |   10.0 | +10.0 | appeared |
| `addRotationStep`                         | `src/body/rigid-body-step.ts:55`                                         |    20.2 |   30.0 |  +9.8 |          |
| `destroyStaleContactsBetweenBodies`       | `src/contacts.ts:570`                                                    |     0.0 |    9.8 |  +9.8 | appeared |
| `updateAABB`                              | `src/body/rigid-body.ts:567`                                             |    21.2 |   11.4 |  -9.8 |          |
| `fromRotationTranslation`                 | `…/node_modules/mathcat/dist/mat4.js:1150`                               |    22.5 |   32.2 |  +9.7 |          |
| `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:548`                           |    68.9 |   78.6 |  +9.7 |          |
| `resetForces`                             | `src/update.ts:1914`                                                     |     8.8 |    0.0 |  -8.8 | gone     |
| `addHit`                                  | `src/update.ts:432`                                                      |    47.6 |   39.0 |  -8.6 |          |
| `waitForWorker`                           | `node:internal/modules/esm/hooks`                                        |    38.4 |   30.2 |  -8.2 |          |
| `combineMaterial`                         | `src/constraints/combine-material.ts:40`                                 |     7.1 |   15.0 |  +7.9 |          |
| `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                           |    69.6 |   77.3 |  +7.7 |          |
| `computeContactSortKey`                   | `src/constraints/contact-constraints.ts:1523`                            |     7.7 |    0.0 |  -7.7 | gone     |
| `multiply$1`                              | `…/node_modules/mathcat/dist/mat4.js:448`                                |    17.4 |    9.8 |  -7.6 |          |
| `pruneContactPoints`                      | `src/manifold/manifold.ts:168`                                           |    10.1 |   17.6 |  +7.5 |          |
| `prepare`                                 | `src/islands.ts:64`                                                      |     7.5 |    0.0 |  -7.5 | gone     |
| `getSleepTestPoints`                      | `src/body/sleep.ts:22`                                                   |    23.6 |   16.3 |  -7.3 |          |
| `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:475`                             |    53.9 |   60.8 |  +6.9 |          |
| `setContactSettings`                      | `src/constraints/contact-constraints.ts:243`                             |    22.8 |   29.6 |  +6.8 |          |
| `copy$9`                                  | `…/node_modules/mathcat/dist/vec3.js:58`                                 |     6.7 |    0.0 |  -6.7 | gone     |
| `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`             |   100.3 |  106.9 |  +6.6 |          |
| `calculateConstraintProperties`           | `src/constraints/constraint-part/angular-friction-constraint-part.ts:77` |     8.7 |   15.3 |  +6.6 |          |
| `computeBarycentricCoordinates3d`         | `src/collision/closest-points.ts:57`                                     |     0.0 |    6.5 |  +6.5 | appeared |
| `addEpaSupportPoint`                      | `src/collision/penetration.ts:150`                                       |     6.4 |    0.0 |  -6.4 | gone     |
| `initialize$2`                            | `src/collision/epa-convex-hull-builder.ts:419`                           |     0.0 |    6.3 |  +6.3 | appeared |
| `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1541`                            |    77.1 |   70.8 |  -6.3 |          |
| `getContactsFromCache`                    | `src/update.ts:868`                                                      |   153.7 |  159.4 |  +5.7 |          |
| `setAxisAngle`                            | `…/node_modules/mathcat/dist/quat.js:36`                                 |    30.7 |   25.0 |  -5.7 |          |
| `velocityIntegrationUpdate`               | `src/update.ts:1131`                                                     |    28.4 |   34.0 |  +5.6 |          |
| `update$1`                                | `src/broadphase/dbvt.ts:485`                                             |    65.6 |   60.0 |  -5.6 |          |
| `finalize`                                | `src/islands.ts:240`                                                     |    13.5 |   19.0 |  +5.5 |          |
| `transformMat4$1`                         | `…/node_modules/mathcat/dist/vec3.js:530`                                |    17.6 |   12.4 |  -5.2 |          |
| `getSupportingFace$11`                    | `src/shapes/box.ts:157`                                                  |    50.0 |   54.9 |  +4.9 |          |
| (anonymous)                               | `src/constraints/contact-constraints.ts:1542`                            |    37.5 |   42.4 |  +4.9 |          |
| `clipPolyVsPoly`                          | `src/manifold/clip.ts:108`                                               |    38.8 |   34.5 |  -4.3 |          |
| `hasContactsBetweenBodyIds`               | `src/contacts.ts:479`                                                    |    22.8 |   18.8 |  -4.0 |          |
| `(program)`                               | ``                                                                       |    15.0 |   11.0 |  -4.0 |          |
| `makeSyncRequest`                         | `node:internal/modules/esm/hooks`                                        |    34.7 |   30.8 |  -3.9 |          |
| `fromQuat$1`                              | `…/node_modules/mathcat/dist/mat4.js:1518`                               |    13.6 |   17.5 |  +3.9 |          |
| `findCollidingPairs`                      | `src/broadphase/broadphase.ts:213`                                       |    23.5 |   27.4 |  +3.9 |          |
| `normalize$1`                             | `…/node_modules/mathcat/dist/vec4.js:325`                                |    17.5 |   13.7 |  -3.8 |          |
| `multiply`                                | `…/node_modules/mathcat/dist/quat.js:93`                                 |    16.3 |   12.5 |  -3.8 |          |
| `getOrthogonalBasis`                      | `src/shapes/plane.ts:211`                                                |    11.2 |    7.6 |  -3.6 |          |
| `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                                        |    61.0 |   64.4 |  +3.4 |          |
| `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                                       |    79.6 |   76.2 |  -3.4 |          |
| `createContact`                           | `src/contacts.ts:373`                                                    |    15.3 |   12.7 |  -2.6 |          |
| `transformQuat`                           | `…/node_modules/mathcat/dist/vec3.js:567`                                |    11.2 |   13.8 |  +2.6 |          |
| `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                                |    72.0 |   69.4 |  -2.6 |          |
| `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:629`                           |    60.0 |   57.5 |  -2.5 |          |
| `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                                |   126.6 |  129.0 |  +2.4 |          |
| `accelerationIntegrationUpdate`           | `src/update.ts:287`                                                      |    16.2 |   13.8 |  -2.4 |          |
| `destroyUnprocessedContacts`              | `src/contacts.ts:700`                                                    |    16.4 |   18.7 |  +2.3 |          |
| `collideConvexVsPlane`                    | `src/shapes/plane.ts:483`                                                |    15.2 |   12.9 |  -2.3 |          |
| `compileForInternalLoader`                | `node:internal/bootstrap/realm`                                          |    11.0 |    8.8 |  -2.2 |          |
| `setBoxSupport`                           | `src/collision/support.ts:296`                                           |    27.6 |   25.8 |  -1.8 |          |
| `linkContactConstraints`                  | `src/islands.ts:184`                                                     |    32.7 |   33.9 |  +1.2 |          |
| `compileSourceTextModule`                 | `node:internal/modules/esm/utils`                                        |    19.6 |   18.5 |  -1.1 |          |
| `(garbage collector)`                     | ``                                                                       |    45.4 |   46.4 |  +1.0 |          |
| `updateBodyPositions`                     | `src/update.ts:371`                                                      |     8.2 |    7.5 |  -0.7 |          |
| `(idle)`                                  | ``                                                                       |    29.9 |   30.4 |  +0.5 |          |

