# perf diff — kcc-mesh → kcc-mesh

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `kcc-mesh` | `9b65b0d` (dirty) | 1068.9 | 2026-07-03T13:48:58.232Z |
| current | `kcc-mesh` | `9b65b0d` (dirty) | 907.7 | 2026-07-03T14:15:21.619Z |

**total attributed time:** 1068.9 ms → 907.7 ms  (**-161.2 ms**, -15.1%)

## category deltas

| category      | base ms | cur ms |  Δ ms | Δ pts |   rel % |
| ------------- | ------: | -----: | ----: | ----: | ------: |
| shapes        |   234.1 |  166.7 | -67.4 |  -3.5 |  -28.8% |
| character     |   137.6 |   87.7 | -49.9 |  -3.2 |  -36.3% |
| broadphase    |    65.7 |   45.7 | -20.0 |  -1.1 |  -30.4% |
| narrowphase   |   392.1 |  372.2 | -19.9 |  +4.3 |   -5.1% |
| bench-harness |     4.8 |   13.6 |  +8.8 |  +1.1 | +183.3% |
| math          |    84.8 |   91.9 |  +7.1 |  +2.2 |   +8.4% |
| step          |    25.3 |   18.3 |  -7.0 |  -0.4 |  -27.7% |
| solver        |    41.5 |   36.5 |  -5.0 |  +0.1 |  -12.0% |
| runtime       |    31.6 |   28.5 |  -3.1 |  +0.1 |   -9.8% |
| manifold      |     9.8 |    7.7 |  -2.1 |  -0.1 |  -21.4% |
| crashcat-util |    20.0 |   18.5 |  -1.5 |  +0.1 |   -7.5% |
| ccd           |     1.3 |    0.0 |  -1.3 |  -0.1 | -100.0% |
| other         |     1.3 |    2.4 |  +1.1 |  +0.2 |  +84.6% |
| body          |    19.0 |   18.0 |  -1.0 |  +0.2 |   -5.3% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                | location                                                      | base ms | cur ms |  Δ ms | note     |
| --------------------------------------- | ------------------------------------------------------------- | ------: | -----: | ----: | -------- |
| `collideConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1209`                            |   100.1 |   75.5 | -24.6 |          |
| `calculateTriangleAABB`                 | `src/shapes/utils/triangle-mesh-data.ts:93`                   |    24.2 |    0.0 | -24.2 | gone     |
| `gjkCastShape`                          | `src/collision/gjk.ts:913`                                    |   108.1 |   88.3 | -19.8 |          |
| `getSupport`                            | `src/collision/support.ts:125`                                |    63.0 |   81.9 | +18.9 |          |
| `gjkClosestPoints`                      | `src/collision/gjk.ts:1225`                                   |    78.5 |   60.0 | -18.5 |          |
| `correctFractionForCharacterPadding`    | `src/character/kcc.ts:1370`                                   |    20.7 |    7.9 | -12.8 |          |
| `penetrationCastShape`                  | `src/collision/penetration.ts:621`                            |    53.7 |   65.2 | +11.5 |          |
| `solveConstraints`                      | `src/character/kcc.ts:2008`                                   |    18.1 |    8.0 | -10.1 |          |
| `setCapsuleSupport`                     | `src/collision/support.ts:336`                                |     8.4 |    0.0 |  -8.4 | gone     |
| `getSurfaceNormal`                      | `src/body/rigid-body.ts:924`                                  |    11.1 |    3.6 |  -7.5 |          |
| `(garbage collector)`                   | ``                                                            |    25.3 |   18.4 |  -6.9 |          |
| `finalizeContactTracking`               | `src/character/kcc.ts:2968`                                   |     6.8 |    0.0 |  -6.8 | gone     |
| `castConvexVsTriangleMesh`              | `src/shapes/triangle-mesh.ts:676`                             |    47.1 |   40.5 |  -6.6 |          |
| `getContactsAtPosition`                 | `src/character/kcc.ts:1268`                                   |     9.9 |    3.9 |  -6.0 |          |
| `runSim`                                | `bench/kcc-mesh.bench.ts`                                     |     2.6 |    8.5 |  +5.9 |          |
| `negate`                                | `…/node_modules/mathcat/dist/vec3.js:343`                     |     0.0 |    5.9 |  +5.9 | appeared |
| `update`                                | `src/character/kcc.ts:3784`                                   |    11.0 |    5.3 |  -5.7 |          |
| `addHit`                                | `src/character/kcc.ts:910`                                    |     5.2 |   10.9 |  +5.7 |          |
| `intersectAABB$1`                       | `src/broadphase/dbvt.ts:627`                                  |    25.6 |   20.0 |  -5.6 |          |
| `move`                                  | `src/character/kcc.ts:3327`                                   |     7.4 |    2.0 |  -5.4 |          |
| `compareConstraints`                    | `src/character/kcc.ts:1912`                                   |     0.0 |    5.1 |  +5.1 | appeared |
| `updateGroundVelocity`                  | `src/character/kcc.ts:3087`                                   |     4.8 |    0.0 |  -4.8 | gone     |
| `updateWorld`                           | `src/update.ts:45`                                            |    12.6 |    8.0 |  -4.6 |          |
| `castAABB$1`                            | `src/broadphase/dbvt.ts:936`                                  |    18.7 |   14.2 |  -4.5 |          |
| `dot$2`                                 | `…/node_modules/mathcat/dist/vec3.js:390`                     |     0.0 |    4.5 |  +4.5 | appeared |
| `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1121`                 |     8.1 |    3.8 |  -4.3 |          |
| `buildTriangleMesh`                     | `src/shapes/utils/triangle-mesh-builder.ts:63`                |     3.0 |    7.3 |  +4.3 |          |
| `steerAgent`                            | `bench/kcc-mesh.bench.ts`                                     |     0.0 |    4.1 |  +4.1 | appeared |
| `collideTransformedVsShape`             | `src/shapes/transformed.ts:377`                               |     7.8 |    3.8 |  -4.0 |          |
| `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:475`                  |     4.0 |    0.0 |  -4.0 | gone     |
| `checkIslandSleep`                      | `src/islands.ts:378`                                          |     3.9 |    0.0 |  -3.9 | gone     |
| `getSurfaceNormal$1`                    | `src/shapes/triangle-mesh.ts:184`                             |     3.8 |    0.0 |  -3.8 | gone     |
| `optimizeIncremental`                   | `src/broadphase/dbvt.ts:587`                                  |     3.8 |    0.0 |  -3.8 | gone     |
| `clipPolyVsPoly`                        | `src/manifold/clip.ts:108`                                    |     3.8 |    0.0 |  -3.8 | gone     |
| `updateAABB`                            | `src/body/rigid-body.ts:567`                                  |     3.7 |    0.0 |  -3.7 | gone     |
| `clamp`                                 | `…/node_modules/mathcat/dist/common.js:69`                    |     0.0 |    3.7 |  +3.7 | appeared |
| `set`                                   | `src/query.ts:258`                                            |     0.0 |    3.7 |  +3.7 | appeared |
| `distance`                              | `…/node_modules/mathcat/dist/vec3.js:305`                     |     3.5 |    0.0 |  -3.5 | gone     |
| `multiply3x3Vec`                        | `…/node_modules/mathcat/dist/mat4.js:620`                     |     3.5 |    0.0 |  -3.5 | gone     |
| `fromRotationTranslationScale`          | `…/node_modules/mathcat/dist/mat4.js:1400`                    |     0.0 |    3.5 |  +3.5 | appeared |
| `intersectsBox3`                        | `…/node_modules/mathcat/dist/raycast3.js:195`                 |    12.5 |    9.1 |  -3.4 |          |
| `cross`                                 | `…/node_modules/mathcat/dist/vec3.js:401`                     |     4.1 |    7.5 |  +3.4 |          |
| `pushIndex`                             | `src/body/sub-shape.ts:60`                                    |     0.0 |    3.4 |  +3.4 | appeared |
| `rayDistanceToBox3`                     | `src/collision/cast-utils.ts:20`                              |    20.0 |   23.4 |  +3.4 |          |
| `(program)`                             | ``                                                            |     0.0 |    3.3 |  +3.3 | appeared |
| `sortContacts`                          | `src/character/kcc.ts:1249`                                   |     0.0 |    3.3 |  +3.3 | appeared |
| `addHit`                                | `src/update.ts:432`                                           |     0.0 |    3.2 |  +3.2 | appeared |
| `bounds$2`                              | `…/node_modules/mathcat/dist/triangle3.js:9`                  |     4.5 |    7.6 |  +3.1 |          |
| `castShape`                             | `src/query.ts:185`                                            |     5.2 |    2.2 |  -3.0 |          |
| `create$6`                              | `src/shapes/triangle-mesh.ts:113`                             |     3.0 |    0.0 |  -3.0 | gone     |
| `updateSupportingContact`               | `src/character/kcc.ts:2290`                                   |    11.2 |    8.3 |  -2.9 |          |
| `update$1`                              | `src/broadphase/dbvt.ts:485`                                  |     7.6 |    4.8 |  -2.8 |          |
| `determineConstraints`                  | `src/character/kcc.ts:1660`                                   |     6.3 |    3.6 |  -2.7 |          |
| `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                             |    34.6 |   32.0 |  -2.6 |          |
| `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                    |     2.6 |    0.0 |  -2.6 | gone     |
| `accelerationIntegrationUpdate`         | `src/update.ts:287`                                           |     2.6 |    0.0 |  -2.6 | gone     |
| `castShapeVsShape`                      | `src/collision/narrowphase.ts:91`                             |     2.6 |    0.0 |  -2.6 | gone     |
| `calculateConstraintProperties$5`       | `src/constraints/constraint-part/axis-constraint-part.ts:164` |     0.0 |    2.6 |  +2.6 | appeared |
| `clipPolyVsPlane`                       | `src/manifold/clip.ts:13`                                     |     0.0 |    2.6 |  +2.6 | appeared |
| `processEdge`                           | `src/shapes/utils/triangle-mesh-builder.ts:340`               |     2.5 |    0.0 |  -2.5 | gone     |
| `resetCachedManifold`                   | `src/contacts.ts:264`                                         |     2.5 |    0.0 |  -2.5 | gone     |
| `findCollidingPairs`                    | `src/broadphase/broadphase.ts:213`                            |     2.5 |    0.0 |  -2.5 | gone     |
| `reconstructManifoldFromCache`          | `src/update.ts:799`                                           |     2.5 |    0.0 |  -2.5 | gone     |
| `castAABB`                              | `src/broadphase/broadphase.ts:328`                            |     2.5 |    0.0 |  -2.5 | gone     |
| `transformFaceWithMat4Scale`            | `src/utils/face.ts:68`                                        |     2.5 |    0.0 |  -2.5 | gone     |
| `compareContactsForDeterminism`         | `src/character/kcc.ts:1241`                                   |     2.5 |    0.0 |  -2.5 | gone     |
| `insertLeaf`                            | `src/broadphase/dbvt.ts:118`                                  |     2.5 |    0.0 |  -2.5 | gone     |
| `multiply`                              | `…/node_modules/mathcat/dist/quat.js:93`                      |     2.5 |    0.0 |  -2.5 | gone     |
| `copySimplex`                           | `src/collision/simplex.ts:32`                                 |     0.0 |    2.5 |  +2.5 | appeared |
| `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:971`                  |     0.0 |    2.5 |  +2.5 | appeared |
| `combineMaterial`                       | `src/constraints/combine-material.ts:40`                      |     0.0 |    2.5 |  +2.5 | appeared |
| `transformQuat`                         | `…/node_modules/mathcat/dist/vec3.js:567`                     |     0.0 |    2.5 |  +2.5 | appeared |
| `getTriangleVertices`                   | `src/shapes/utils/triangle-mesh-data.ts:44`                   |    15.8 |   13.5 |  -2.3 |          |
| `conjugate`                             | `…/node_modules/mathcat/dist/quat.js:333`                     |     0.0 |    2.3 |  +2.3 | appeared |
| `moveToContact`                         | `src/character/kcc.ts:3398`                                   |     0.0 |    2.3 |  +2.3 | appeared |
| `releaseAllConstraints`                 | `src/character/kcc.ts:665`                                    |     0.0 |    2.3 |  +2.3 | appeared |
| `velocityIntegrationUpdate`             | `src/update.ts:1131`                                          |     0.0 |    2.3 |  +2.3 | appeared |
| `getShapeSurfaceNormal`                 | `src/shapes/shapes.ts:464`                                    |     0.0 |    2.2 |  +2.2 | appeared |
| `finalizeAndCreateConstraints`          | `src/update.ts:577`                                           |     0.0 |    2.2 |  +2.2 | appeared |
| `updateBody`                            | `src/broadphase/broadphase.ts:196`                            |     0.0 |    2.1 |  +2.1 | appeared |
| `removeConflictingContacts`             | `src/character/kcc.ts:1597`                                   |     0.0 |    2.1 |  +2.1 | appeared |
| `subtract$1`                            | `…/node_modules/mathcat/dist/vec3.js:154`                     |     5.8 |    3.8 |  -2.0 |          |
| `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1399`                 |     2.0 |    0.0 |  -2.0 | gone     |
| `resetContactTracking`                  | `src/character/kcc.ts:2780`                                   |     1.8 |    0.0 |  -1.8 | gone     |
| `(idle)`                                | ``                                                            |     5.1 |    6.8 |  +1.7 |          |
| `getShapeSupportingFace`                | `src/shapes/shapes.ts:494`                                    |     0.0 |    1.7 |  +1.7 | appeared |
| `transformMat4$1`                       | `…/node_modules/mathcat/dist/vec3.js:530`                     |    22.6 |   21.1 |  -1.5 |          |
| `moveShape`                             | `src/character/kcc.ts:2683`                                   |     6.4 |    7.9 |  +1.5 |          |
| `visit`                                 | `src/query.ts:108`                                            |     4.0 |    2.5 |  -1.5 |          |
| `updateShape`                           | `src/body/rigid-body.ts:574`                                  |     0.0 |    1.5 |  +1.5 | appeared |
| `prepare`                               | `src/islands.ts:64`                                           |     2.2 |    3.6 |  +1.4 |          |
| `computeBarycentricCoordinates3d`       | `src/collision/closest-points.ts:57`                          |     4.2 |    2.9 |  -1.3 |          |
| `buildRecursive`                        | `src/shapes/utils/triangle-mesh-bvh.ts:308`                   |     3.6 |    2.3 |  -1.3 |          |
| `addContactConstraint`                  | `src/constraints/contact-constraints.ts:582`                  |     6.1 |    4.8 |  -1.3 |          |
| `setBoxSupport`                         | `src/collision/support.ts:296`                                |     3.8 |    2.5 |  -1.3 |          |
| `copy$9`                                | `…/node_modules/mathcat/dist/vec3.js:58`                      |     7.2 |    6.0 |  -1.2 |          |
| `pruneContactPoints`                    | `src/manifold/manifold.ts:168`                                |     2.5 |    3.6 |  +1.1 |          |
| `finalize`                              | `src/islands.ts:240`                                          |     4.1 |    3.2 |  -0.9 |          |
| `manifoldBetweenTwoFaces`               | `src/manifold/manifold.ts:394`                                |     2.2 |    1.5 |  -0.7 |          |
| `push`                                  | `src/utils/bvh-stack.ts:36`                                   |     6.4 |    7.1 |  +0.7 |          |
| `setTriangleSupport`                    | `src/collision/support.ts:376`                                |     4.0 |    3.5 |  -0.5 |          |

