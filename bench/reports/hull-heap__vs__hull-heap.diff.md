# perf diff — hull-heap → hull-heap

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `hull-heap` | `58a218d` (dirty) | 17488.8 | 2026-07-03T03:21:10.218Z |
| current | `hull-heap` | `58a218d` (dirty) | 11131.4 | 2026-07-03T04:39:37.631Z |

**total attributed time:** 17488.8 ms → 11131.4 ms  (**-6357.4 ms**, -36.4%)

## category deltas

| category      | base ms | cur ms |    Δ ms | Δ pts |   rel % |
| ------------- | ------: | -----: | ------: | ----: | ------: |
| narrowphase   | 10215.2 | 4652.1 | -5563.1 | -16.6 |  -54.5% |
| solver        |  3147.5 | 2639.1 |  -508.4 |  +5.7 |  -16.2% |
| step          |   976.0 |  821.0 |  -155.0 |  +1.8 |  -15.9% |
| math          |   321.4 |  286.3 |   -35.1 |  +0.8 |  -10.9% |
| body          |   347.0 |  315.3 |   -31.7 |  +0.8 |   -9.1% |
| broadphase    |   678.9 |  647.3 |   -31.6 |  +1.9 |   -4.7% |
| manifold      |   200.1 |  169.8 |   -30.3 |  +0.4 |  -15.1% |
| crashcat-util |     3.8 |    0.1 |    -3.7 |  +0.0 |  -97.4% |
| runtime       |   235.8 |  239.1 |    +3.3 |  +0.8 |   +1.4% |
| bench-harness |     8.5 |    7.1 |    -1.4 |  +0.1 |  -16.5% |
| ccd           |     1.3 |    0.0 |    -1.3 |  +0.0 | -100.0% |
| shapes        |  1352.1 | 1352.9 |    +0.8 |  +4.5 |   +0.1% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                  | location                                                                 | base ms | cur ms |    Δ ms | note     |
| ----------------------------------------- | ------------------------------------------------------------------------ | ------: | -----: | ------: | -------- |
| `computeShrunkHullPoints`                 | `src/collision/support.ts:421`                                           |  5417.5 |    0.0 | -5417.5 | gone     |
| `addContactConstraint`                    | `src/constraints/contact-constraints.ts:582`                             |   514.7 |  400.3 |  -114.4 |          |
| `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1121`                            |  1312.9 | 1209.6 |  -103.3 |          |
| `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:971`                             |   196.5 |  121.4 |   -75.1 |          |
| `narrowphase`                             | `src/update.ts:949`                                                      |   228.1 |  153.1 |   -75.0 |          |
| `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:475`                             |   106.6 |   40.6 |   -66.0 |          |
| `updateWorld`                             | `src/update.ts:45`                                                       |   394.4 |  340.4 |   -54.0 |          |
| `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:548`                           |   276.0 |  225.8 |   -50.2 |          |
| `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                                       |   231.7 |  188.0 |   -43.7 |          |
| `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1359`                            |   118.2 |   76.8 |   -41.4 |          |
| `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                           |   288.2 |  247.0 |   -41.2 |          |
| `setContactSettings`                      | `src/constraints/contact-constraints.ts:243`                             |    56.5 |   17.6 |   -38.9 |          |
| `getContactsFromCache`                    | `src/update.ts:868`                                                      |   239.6 |  210.2 |   -29.4 |          |
| `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:629`                           |   198.7 |  174.4 |   -24.3 |          |
| `getSupportingFace$8`                     | `src/shapes/convex-hull.ts:529`                                          |  1071.8 | 1049.0 |   -22.8 |          |
| `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                                |   104.0 |   84.2 |   -19.8 |          |
| `linkContactConstraints`                  | `src/islands.ts:184`                                                     |    44.5 |   25.0 |   -19.5 |          |
| `calculateConstraintProperties`           | `src/constraints/constraint-part/angular-friction-constraint-part.ts:77` |    18.6 |    0.0 |   -18.6 | gone     |
| `gjkClosestPoints`                        | `src/collision/gjk.ts:1225`                                              |   472.9 |  490.8 |   +17.9 |          |
| `intersectAABB$1`                         | `src/broadphase/dbvt.ts:627`                                             |   580.4 |  562.5 |   -17.9 |          |
| `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                               |   101.9 |  119.4 |   +17.5 |          |
| `multiply`                                | `…/node_modules/mathcat/dist/quat.js:93`                                 |    17.2 |    0.0 |   -17.2 | gone     |
| `findContact`                             | `src/contacts.ts:647`                                                    |    70.3 |   53.6 |   -16.7 |          |
| `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1007`                                            |   132.7 |  117.4 |   -15.3 |          |
| `clipPolyVsPoly`                          | `src/manifold/clip.ts:108`                                               |    39.6 |   24.6 |   -15.0 |          |
| `getSleepTestPoints`                      | `src/body/sleep.ts:22`                                                   |    24.9 |   11.2 |   -13.7 |          |
| `fromRotationTranslation`                 | `…/node_modules/mathcat/dist/mat4.js:1150`                               |    34.0 |   45.2 |   +11.2 |          |
| `transformQuat`                           | `…/node_modules/mathcat/dist/vec3.js:567`                                |    22.5 |   11.3 |   -11.2 |          |
| `hasContactsBetweenBodyIds`               | `src/contacts.ts:479`                                                    |    12.2 |   22.6 |   +10.4 |          |
| `normalize$2`                             | `…/node_modules/mathcat/dist/vec3.js:369`                                |    24.8 |   14.6 |   -10.2 |          |
| `manifoldBetweenTwoFaces`                 | `src/manifold/manifold.ts:394`                                           |    47.2 |   37.1 |   -10.1 |          |
| `multiply3x3Vec`                          | `…/node_modules/mathcat/dist/mat4.js:620`                                |     0.0 |    9.8 |    +9.8 | appeared |
| `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                                |    93.6 |   84.3 |    -9.3 |          |
| `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1399`                            |   202.4 |  211.7 |    +9.3 |          |
| `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`             |    85.2 |   76.0 |    -9.2 |          |
| `getShapeSupportingFace`                  | `src/shapes/shapes.ts:494`                                               |    32.3 |   41.2 |    +8.9 |          |
| `setAxisAngle`                            | `…/node_modules/mathcat/dist/quat.js:36`                                 |     0.0 |    8.9 |    +8.9 | appeared |
| `scaleAndAdd`                             | `…/node_modules/mathcat/dist/vec3.js:292`                                |     0.0 |    8.8 |    +8.8 | appeared |
| `readFileUtf8`                            | ``                                                                       |     0.0 |    8.8 |    +8.8 | appeared |
| `destroyContact`                          | `src/contacts.ts:435`                                                    |    28.9 |   20.1 |    -8.8 |          |
| `update$1`                                | `src/broadphase/dbvt.ts:485`                                             |    64.2 |   55.9 |    -8.3 |          |
| `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1541`                            |    59.3 |   51.3 |    -8.0 |          |
| `invert$2`                                | `…/node_modules/mathcat/dist/mat4.js:251`                                |     7.6 |    0.0 |    -7.6 | gone     |
| `insertLeaf`                              | `src/broadphase/dbvt.ts:118`                                             |     0.0 |    7.6 |    +7.6 | appeared |
| `destroyUnprocessedContacts`              | `src/contacts.ts:700`                                                    |    17.6 |   10.2 |    -7.4 |          |
| `resetForces`                             | `src/update.ts:1914`                                                     |     7.4 |    0.0 |    -7.4 | gone     |
| `setHullSupport`                          | `src/collision/support.ts:677`                                           |    27.3 |   20.3 |    -7.0 |          |
| `multiply$1`                              | `…/node_modules/mathcat/dist/mat4.js:448`                                |    23.7 |   17.6 |    -6.1 |          |
| `velocityIntegrationUpdate`               | `src/update.ts:1131`                                                     |    28.8 |   34.7 |    +5.9 |          |
| `checkIslandSleep`                        | `src/islands.ts:378`                                                     |    26.5 |   21.0 |    -5.5 |          |
| `destroyStaleContactsBetweenBodies`       | `src/contacts.ts:570`                                                    |    24.8 |   19.7 |    -5.1 |          |
| `createContact`                           | `src/contacts.ts:373`                                                    |    18.8 |   13.9 |    -4.9 |          |
| `updateAABB`                              | `src/body/rigid-body.ts:567`                                             |    21.0 |   16.1 |    -4.9 |          |
| `setCachedBodyPair`                       | `src/contacts.ts:174`                                                    |    40.0 |   44.8 |    +4.8 |          |
| `getAdaptivePlaneSupportingFace`          | `src/shapes/plane.ts:636`                                                |    18.2 |   13.5 |    -4.7 |          |
| `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                                      |   149.7 |  154.2 |    +4.5 |          |
| `combineMaterial`                         | `src/constraints/combine-material.ts:40`                                 |    23.4 |   27.9 |    +4.5 |          |
| `addHit`                                  | `src/update.ts:432`                                                      |    27.6 |   31.6 |    +4.0 |          |
| `finalize`                                | `src/islands.ts:240`                                                     |    15.1 |   19.0 |    +3.9 |          |
| `(garbage collector)`                     | ``                                                                       |    44.9 |   41.1 |    -3.8 |          |
| `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810`            |    13.6 |   17.2 |    +3.6 |          |
| `transformMat4$1`                         | `…/node_modules/mathcat/dist/vec3.js:530`                                |    11.0 |    7.6 |    -3.4 |          |
| `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:309`                             |    74.9 |   71.7 |    -3.2 |          |
| `prepare`                                 | `src/islands.ts:64`                                                      |    13.8 |   10.7 |    -3.1 |          |
| `cross`                                   | `…/node_modules/mathcat/dist/vec3.js:401`                                |    21.3 |   24.3 |    +3.0 |          |
| `(program)`                               | ``                                                                       |    17.0 |   14.5 |    -2.5 |          |
| `reconstructManifoldFromCache`            | `src/update.ts:799`                                                      |    16.1 |   13.7 |    -2.4 |          |
| `updateBodyPositions`                     | `src/update.ts:371`                                                      |    11.0 |    8.7 |    -2.3 |          |
| `transformPlane`                          | `src/shapes/plane.ts:197`                                                |     7.4 |    9.5 |    +2.1 |          |
| `fromQuat$1`                              | `…/node_modules/mathcat/dist/mat4.js:1518`                               |     9.2 |   11.2 |    +2.0 |          |
| `waitForWorker`                           | `node:internal/modules/esm/hooks`                                        |    30.1 |   32.0 |    +1.9 |          |
| `accelerationIntegrationUpdate`           | `src/update.ts:287`                                                      |    20.5 |   18.6 |    -1.9 |          |
| `getSupport`                              | `src/collision/support.ts:125`                                           |  3192.7 | 3194.3 |    +1.6 |          |
| `findCollidingPairs`                      | `src/broadphase/broadphase.ts:213`                                       |    20.1 |   18.7 |    -1.4 |          |
| `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                               |    79.7 |   81.0 |    +1.3 |          |
| `compileSourceTextModule`                 | `node:internal/modules/esm/utils`                                        |    21.1 |   19.9 |    -1.2 |          |
| `(idle)`                                  | ``                                                                       |    33.5 |   32.4 |    -1.1 |          |
| `initialize$2`                            | `src/collision/epa-convex-hull-builder.ts:419`                           |    16.4 |   17.5 |    +1.1 |          |
| `pruneContactPoints`                      | `src/manifold/manifold.ts:168`                                           |    19.7 |   18.8 |    -0.9 |          |
| (anonymous)                               | `src/constraints/contact-constraints.ts:1542`                            |    32.3 |   31.7 |    -0.6 |          |
| `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                                        |    74.7 |   75.2 |    +0.5 |          |
| `collideConvexVsPlane`                    | `src/shapes/plane.ts:483`                                                |    23.4 |   23.9 |    +0.5 |          |

