# perf diff — projectiles-terrain → projectiles-terrain

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `projectiles-terrain` | `9b65b0d` (dirty) | 824 | 2026-07-03T13:48:59.378Z |
| current | `projectiles-terrain` | `9b65b0d` (dirty) | 912.7 | 2026-07-03T13:50:27.406Z |

**total attributed time:** 824 ms → 912.7 ms  (**+88.7 ms**, +10.8%)

## category deltas

| category      | base ms | cur ms |  Δ ms | Δ pts |   rel % |
| ------------- | ------: | -----: | ----: | ----: | ------: |
| shapes        |   173.6 |  265.0 | +91.4 |  +7.9 |  +52.6% |
| step          |    92.0 |   56.3 | -35.7 |  -5.0 |  -38.8% |
| narrowphase   |   209.5 |  236.4 | +26.9 |  +0.5 |  +12.8% |
| math          |    64.3 |   84.0 | +19.7 |  +1.4 |  +30.6% |
| runtime       |    29.7 |   11.8 | -17.9 |  -2.3 |  -60.3% |
| solver        |   102.1 |   88.5 | -13.6 |  -2.7 |  -13.3% |
| broadphase    |    97.0 |  104.4 |  +7.4 |  -0.4 |   +7.6% |
| bench-harness |     4.3 |   11.0 |  +6.7 |  +0.7 | +155.8% |
| manifold      |     6.3 |   12.5 |  +6.2 |  +0.6 |  +98.4% |
| crashcat-util |    18.7 |   13.4 |  -5.3 |  -0.8 |  -28.3% |
| ccd           |     1.3 |    4.0 |  +2.7 |  +0.2 | +207.7% |
| body          |    22.5 |   23.1 |  +0.6 |  -0.2 |   +2.7% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                  | location                                                        | base ms | cur ms |  Δ ms | note     |
| ----------------------------------------- | --------------------------------------------------------------- | ------: | -----: | ----: | -------- |
| `updateWorld`                             | `src/update.ts:45`                                              |    29.2 |    8.4 | -20.8 |          |
| `collideConvexVsTriangleMesh`             | `src/shapes/triangle-mesh.ts:1213`                              |    28.9 |   49.3 | +20.4 |          |
| `getSupport`                              | `src/collision/support.ts:125`                                  |     9.1 |   24.2 | +15.1 |          |
| `collideSphereVsTriangleMesh`             | `src/shapes/triangle-mesh.ts:1727`                              |    38.7 |   53.1 | +14.4 |          |
| `gjkClosestPoints`                        | `src/collision/gjk.ts:1225`                                     |    20.1 |   33.5 | +13.4 |          |
| `squaredDistance`                         | `…/node_modules/mathcat/dist/vec3.js:318`                       |     0.0 |   13.2 | +13.2 | appeared |
| `(garbage collector)`                     | ``                                                              |    20.1 |    7.1 | -13.0 |          |
| `rayDistanceToBox3`                       | `src/collision/cast-utils.ts:20`                                |    48.8 |   36.3 | -12.5 |          |
| `getOptimalSplit`                         | `src/shapes/utils/triangle-mesh-bvh.ts:451`                     |     0.0 |   12.1 | +12.1 | appeared |
| `addContactConstraint`                    | `src/constraints/contact-constraints.ts:582`                    |    21.0 |    9.0 | -12.0 |          |
| `nodeGetCenter`                           | `src/shapes/utils/bvh.ts:67`                                    |     0.0 |   10.6 | +10.6 | appeared |
| `castAABB$1`                              | `src/broadphase/dbvt.ts:936`                                    |    29.0 |   39.5 | +10.5 |          |
| `transformMat4$1`                         | `…/node_modules/mathcat/dist/vec3.js:530`                       |     0.0 |   10.1 | +10.1 | appeared |
| `castSphereVsTriangleMesh`                | `src/shapes/triangle-mesh.ts:2179`                              |    47.6 |   57.3 |  +9.7 |          |
| `castConvexVsTriangleMesh`                | `src/shapes/triangle-mesh.ts:679`                               |    14.3 |   23.6 |  +9.3 |          |
| `rayHitsBox3`                             | `src/collision/cast-utils.ts:114`                               |    13.5 |    5.2 |  -8.3 |          |
| `nodeIntersectsBox`                       | `src/shapes/utils/bvh.ts:137`                                   |     0.0 |    8.0 |  +8.0 | appeared |
| `velocityIntegrationUpdate`               | `src/update.ts:1131`                                            |    11.3 |    3.7 |  -7.6 |          |
| `calculateTriangleAABB`                   | `src/shapes/utils/triangle-mesh-data.ts:93`                     |     3.8 |   11.4 |  +7.6 |          |
| `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1121`                   |    18.7 |   26.2 |  +7.5 |          |
| `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                       |     2.5 |    9.6 |  +7.1 |          |
| `runSim`                                  | `bench/projectiles-terrain.bench.ts`                            |     3.1 |    9.4 |  +6.3 |          |
| `(program)`                               | ``                                                              |     5.9 |    0.0 |  -5.9 | gone     |
| `gjkCastShape`                            | `src/collision/gjk.ts:913`                                      |    23.0 |   28.9 |  +5.9 |          |
| `update$1`                                | `src/broadphase/dbvt.ts:485`                                    |    22.3 |   27.6 |  +5.3 |          |
| `transformQuat`                           | `…/node_modules/mathcat/dist/vec3.js:567`                       |     5.2 |    0.0 |  -5.2 | gone     |
| `raySphereFromOrigin`                     | `src/collision/sphere-triangle.ts:156`                          |     0.0 |    5.2 |  +5.2 | appeared |
| `push`                                    | `src/utils/bvh-stack.ts:36`                                     |    13.8 |    8.7 |  -5.1 |          |
| `checkIslandSleep`                        | `src/islands.ts:378`                                            |     5.1 |    0.0 |  -5.1 | gone     |
| `copy$9`                                  | `…/node_modules/mathcat/dist/vec3.js:58`                        |     0.0 |    5.1 |  +5.1 | appeared |
| `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                       |     0.0 |    5.1 |  +5.1 | appeared |
| `multiply$2`                              | `…/node_modules/mathcat/dist/vec3.js:182`                       |     5.0 |    0.0 |  -5.0 | gone     |
| `findContact`                             | `src/contacts.ts:647`                                           |     0.0 |    5.0 |  +5.0 | appeared |
| `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                               |    24.1 |   29.0 |  +4.9 |          |
| `subtract$1`                              | `…/node_modules/mathcat/dist/vec3.js:154`                       |     9.7 |    4.9 |  -4.8 |          |
| `penetrationCastShape`                    | `src/collision/penetration.ts:621`                              |    24.1 |   19.4 |  -4.7 |          |
| `getTriangleVertices`                     | `src/shapes/utils/triangle-mesh-data.ts:37`                     |    10.0 |   14.5 |  +4.5 |          |
| `intersectAABB$1`                         | `src/broadphase/dbvt.ts:627`                                    |    34.5 |   30.7 |  -3.8 |          |
| `fromQuat$1`                              | `…/node_modules/mathcat/dist/mat4.js:1518`                      |     3.8 |    0.0 |  -3.8 | gone     |
| `setCachedBodyPair`                       | `src/contacts.ts:174`                                           |     3.8 |    0.0 |  -3.8 | gone     |
| `pruneContactPoints`                      | `src/manifold/manifold.ts:168`                                  |     3.8 |    0.0 |  -3.8 | gone     |
| `addHit`                                  | `src/update.ts:432`                                             |     3.8 |    0.0 |  -3.8 | gone     |
| `manifoldBetweenTwoFaces`                 | `src/manifold/manifold.ts:394`                                  |     0.0 |    3.8 |  +3.8 | appeared |
| `updateBodyPositions`                     | `src/update.ts:371`                                             |     0.0 |    3.8 |  +3.8 | appeared |
| `visit`                                   | `src/update.ts:1387`                                            |    11.2 |    7.4 |  -3.8 |          |
| `findCollidingPairs`                      | `src/broadphase/broadphase.ts:213`                              |     6.2 |    2.5 |  -3.7 |          |
| `setBoxSupport`                           | `src/collision/support.ts:296`                                  |     6.2 |    2.5 |  -3.7 |          |
| `intersectsBox3`                          | `…/node_modules/mathcat/dist/raycast3.js:195`                   |     3.7 |    0.0 |  -3.7 | gone     |
| `sortCCDBodies`                           | `src/update.ts:1842`                                            |     3.7 |    0.0 |  -3.7 | gone     |
| `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                      |     3.7 |    0.0 |  -3.7 | gone     |
| `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`    |     3.7 |    0.0 |  -3.7 | gone     |
| `getSupportingFace$11`                    | `src/shapes/box.ts:157`                                         |     3.7 |    0.0 |  -3.7 | gone     |
| `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1007`                                   |     0.0 |    3.6 |  +3.6 | appeared |
| `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:251`                  |     2.5 |    5.8 |  +3.3 |          |
| `collideShapeVsShape`                     | `src/collision/narrowphase.ts:183`                              |     0.0 |    3.2 |  +3.2 | appeared |
| `set$8`                                   | `…/node_modules/mathcat/dist/vec3.js:73`                        |     0.0 |    3.1 |  +3.1 | appeared |
| `hasContactsBetweenBodyIds`               | `src/contacts.ts:479`                                           |     0.0 |    2.9 |  +2.9 | appeared |
| `rayCylinder`                             | `src/collision/sphere-triangle.ts:17`                           |     3.7 |    6.6 |  +2.9 |          |
| `castShapeVsShape`                        | `src/collision/narrowphase.ts:91`                               |     2.8 |    0.0 |  -2.8 | gone     |
| `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:475`                    |     5.2 |    2.5 |  -2.7 |          |
| `conjugate`                               | `…/node_modules/mathcat/dist/quat.js:333`                       |     2.6 |    0.0 |  -2.6 | gone     |
| `getActiveEdges`                          | `src/shapes/utils/triangle-mesh-data.ts:77`                     |     2.6 |    0.0 |  -2.6 | gone     |
| `destroyUnprocessedContacts`              | `src/contacts.ts:700`                                           |     0.0 |    2.6 |  +2.6 | appeared |
| `updateSleepState`                        | `src/body/sleep.ts:83`                                          |     0.0 |    2.6 |  +2.6 | appeared |
| `build`                                   | `src/shapes/utils/triangle-mesh-bvh.ts:248`                     |     0.0 |    2.6 |  +2.6 | appeared |
| `solveVelocityConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:745`   |     0.0 |    2.6 |  +2.6 | appeared |
| `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:209`                              |     6.3 |    8.8 |  +2.5 |          |
| `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1399`                   |     7.4 |    4.9 |  -2.5 |          |
| `updateAABB`                              | `src/body/rigid-body.ts:567`                                    |     5.0 |    2.5 |  -2.5 |          |
| `isScaleInsideOut$1`                      | `…/node_modules/mathcat/dist/vec3.js:756`                       |     2.5 |    0.0 |  -2.5 | gone     |
| `setQuaternion$1`                         | `src/body/rigid-body.ts:634`                                    |     2.5 |    0.0 |  -2.5 | gone     |
| `castConvexVsConvex`                      | `src/shapes/convex.ts:531`                                      |     2.5 |    0.0 |  -2.5 | gone     |
| `fromRotationTranslation`                 | `…/node_modules/mathcat/dist/mat4.js:1150`                      |     2.5 |    0.0 |  -2.5 | gone     |
| `applyCCD`                                | `src/update.ts:1693`                                            |     2.5 |    0.0 |  -2.5 | gone     |
| `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1359`                   |     2.5 |    0.0 |  -2.5 | gone     |
| `getShapeSupportingFace`                  | `src/shapes/shapes.ts:494`                                      |     2.5 |    0.0 |  -2.5 | gone     |
| `updateBody`                              | `src/broadphase/broadphase.ts:196`                              |     2.5 |    0.0 |  -2.5 | gone     |
| `resetCCDBody`                            | `src/ccd.ts:75`                                                 |     0.0 |    2.5 |  +2.5 | appeared |
| `length`                                  | `…/node_modules/mathcat/dist/vec3.js:30`                        |     0.0 |    2.5 |  +2.5 | appeared |
| `set$7`                                   | `…/node_modules/mathcat/dist/vec4.js:66`                        |     0.0 |    2.5 |  +2.5 | appeared |
| `setSphereSupport`                        | `src/collision/support.ts:321`                                  |     2.4 |    0.0 |  -2.4 | gone     |
| `buildRecursive`                          | `src/shapes/utils/triangle-mesh-bvh.ts:308`                     |     0.0 |    2.4 |  +2.4 | appeared |
| `accelerationIntegrationUpdate`           | `src/update.ts:287`                                             |     6.0 |    3.7 |  -2.3 |          |
| `multiply$1`                              | `…/node_modules/mathcat/dist/mat4.js:448`                       |     2.3 |    0.0 |  -2.3 | gone     |
| `markAllUnprocessed`                      | `src/contacts.ts:682`                                           |     0.0 |    2.3 |  +2.3 | appeared |
| (anonymous)                               | `file:///Users/isaacmason/Development/crashcat/dist/index.js:1` |     0.0 |    2.3 |  +2.3 | appeared |
| `clipPolyVsPoly`                          | `src/manifold/clip.ts:108`                                      |     0.0 |    2.3 |  +2.3 | appeared |
| `getClosestPointOnTriangle`               | `src/collision/triangle.ts:62`                                  |     7.6 |    9.8 |  +2.2 |          |
| `initialize$2`                            | `src/collision/epa-convex-hull-builder.ts:419`                  |     3.8 |    6.0 |  +2.2 |          |
| `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:971`                    |     2.6 |    4.7 |  +2.1 |          |
| `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                      |     2.5 |    4.2 |  +1.7 |          |
| `cross`                                   | `…/node_modules/mathcat/dist/vec3.js:401`                       |     3.1 |    4.6 |  +1.5 |          |
| `finalize`                                | `src/islands.ts:240`                                            |    12.6 |   11.3 |  -1.3 |          |
| `narrowphase`                             | `src/update.ts:949`                                             |     7.4 |    6.2 |  -1.2 |          |
| `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:629`                  |     2.5 |    3.7 |  +1.2 |          |
| `scale$4`                                 | `…/node_modules/mathcat/dist/vec3.js:277`                       |     3.8 |    4.9 |  +1.1 |          |
| `release`                                 | `src/utils/pool.ts:17`                                          |     2.5 |    3.5 |  +1.0 |          |
| `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:309`                    |     3.8 |    4.7 |  +0.9 |          |

