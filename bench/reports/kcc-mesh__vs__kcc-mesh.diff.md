# perf diff — kcc-mesh → kcc-mesh

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `kcc-mesh` | `ab84e99` (dirty) | 1060.4 | 2026-07-03T05:08:09.291Z |
| current | `kcc-mesh` | `ab84e99` (dirty) | 1019.8 | 2026-07-03T06:14:24.034Z |

**total attributed time:** 1060.4 ms → 1019.8 ms  (**-40.6 ms**, -3.8%)

## category deltas

| category      | base ms | cur ms |  Δ ms | Δ pts |   rel % |
| ------------- | ------: | -----: | ----: | ----: | ------: |
| character     |   181.2 |  120.5 | -60.7 |  -5.3 |  -33.5% |
| narrowphase   |   367.7 |  424.2 | +56.5 |  +6.9 |  +15.4% |
| step          |    34.5 |   14.0 | -20.5 |  -1.9 |  -59.4% |
| shapes        |   211.1 |  219.9 |  +8.8 |  +1.7 |   +4.2% |
| broadphase    |    43.7 |   37.0 |  -6.7 |  -0.5 |  -15.3% |
| body          |    16.8 |   10.5 |  -6.3 |  -0.6 |  -37.5% |
| manifold      |     6.2 |    1.9 |  -4.3 |  -0.4 |  -69.4% |
| ccd           |     2.7 |    0.0 |  -2.7 |  -0.3 | -100.0% |
| other         |     2.5 |    0.0 |  -2.5 |  -0.2 | -100.0% |
| bench-harness |     9.8 |   12.0 |  +2.2 |  +0.3 |  +22.4% |
| solver        |    33.0 |   30.9 |  -2.1 |  -0.1 |   -6.4% |
| math          |    98.7 |   97.1 |  -1.6 |  +0.2 |   -1.6% |
| crashcat-util |    23.3 |   22.3 |  -1.0 |  +0.0 |   -4.3% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                | location                                                        | base ms | cur ms |  Δ ms | note     |
| --------------------------------------- | --------------------------------------------------------------- | ------: | -----: | ----: | -------- |
| `finalizeContactTracking`               | `src/character/kcc.ts:2976`                                     |    49.3 |    3.5 | -45.8 |          |
| `intersectsBox3`                        | `…/node_modules/mathcat/dist/raycast3.js:195`                   |     8.7 |   26.0 | +17.3 |          |
| `gjkCastShape`                          | `src/collision/gjk.ts:913`                                      |    76.0 |   93.0 | +17.0 |          |
| `update`                                | `src/character/kcc.ts:3793`                                     |    20.6 |    3.7 | -16.9 |          |
| `getSupport`                            | `src/collision/support.ts:125`                                  |    65.2 |   81.2 | +16.0 |          |
| `penetrationCastShape`                  | `src/collision/penetration.ts:621`                              |    61.2 |   75.7 | +14.5 |          |
| `updateWorld`                           | `src/update.ts:45`                                              |    18.2 |    3.8 | -14.4 |          |
| `transformMat4$1`                       | `…/node_modules/mathcat/dist/vec3.js:530`                       |    29.2 |   16.8 | -12.4 |          |
| `solveConstraints`                      | `src/character/kcc.ts:2012`                                     |    21.5 |   11.2 | -10.3 |          |
| `correctFractionForCharacterPadding`    | `src/character/kcc.ts:1374`                                     |    11.7 |   20.9 |  +9.2 |          |
| `addHit`                                | `src/character/kcc.ts:1093`                                     |    12.0 |    3.0 |  -9.0 |          |
| `cancelVelocityTowardsSteepSlopes`      | `src/character/kcc.ts:2564`                                     |     0.0 |    7.9 |  +7.9 | appeared |
| `collideConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1211`                              |    90.1 |   96.9 |  +6.8 |          |
| `(program)`                             | ``                                                              |     6.8 |    0.0 |  -6.8 | gone     |
| `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                               |    36.1 |   42.9 |  +6.8 |          |
| `findCollidingPairs`                    | `src/broadphase/broadphase.ts:213`                              |     0.0 |    6.0 |  +6.0 | appeared |
| `getTriangleVertices`                   | `src/shapes/utils/triangle-mesh-data.ts:37`                     |    10.2 |   16.1 |  +5.9 |          |
| `getFirstContactForSweep`               | `src/character/kcc.ts:1445`                                     |     8.6 |    3.0 |  -5.6 |          |
| `getContactsAtPosition`                 | `src/character/kcc.ts:1272`                                     |     4.7 |   10.0 |  +5.3 |          |
| `bounds$2`                              | `…/node_modules/mathcat/dist/triangle3.js:9`                    |     3.7 |    9.0 |  +5.3 |          |
| `negate`                                | `…/node_modules/mathcat/dist/vec3.js:343`                       |     0.0 |    5.2 |  +5.2 | appeared |
| `update$1`                              | `src/broadphase/dbvt.ts:485`                                    |     8.9 |    3.9 |  -5.0 |          |
| `findListenerContact`                   | `src/character/kcc.ts:600`                                      |     5.0 |    0.0 |  -5.0 | gone     |
| `setTriangleSupport`                    | `src/collision/support.ts:376`                                  |     0.0 |    4.8 |  +4.8 | appeared |
| `addContactConstraint`                  | `src/constraints/contact-constraints.ts:582`                    |     0.0 |    4.4 |  +4.4 | appeared |
| `castTransformedVsShape`                | `src/shapes/transformed.ts:553`                                 |     8.1 |    3.8 |  -4.3 |          |
| `computeActiveEdges`                    | `src/shapes/utils/triangle-mesh-builder.ts:276`                 |     0.0 |    4.2 |  +4.2 | appeared |
| `set$8`                                 | `…/node_modules/mathcat/dist/vec3.js:73`                        |     4.0 |    0.0 |  -4.0 | gone     |
| `determineConstraints`                  | `src/character/kcc.ts:1664`                                     |     2.5 |    6.5 |  +4.0 |          |
| `steerAgent`                            | `bench/kcc-mesh.bench.ts`                                       |     0.0 |    3.9 |  +3.9 | appeared |
| `releaseAllContacts`                    | `src/character/kcc.ts:638`                                      |     0.0 |    3.9 |  +3.9 | appeared |
| `multiply$2`                            | `…/node_modules/mathcat/dist/vec3.js:182`                       |     0.0 |    3.9 |  +3.9 | appeared |
| `updateSupportingContact`               | `src/character/kcc.ts:2295`                                     |     6.8 |    3.0 |  -3.8 |          |
| `collideConvexVsConvex`                 | `src/shapes/convex.ts:247`                                      |     3.8 |    0.0 |  -3.8 | gone     |
| `castConvexVsConvexLocal`               | `src/shapes/convex.ts:605`                                      |     3.8 |    0.0 |  -3.8 | gone     |
| `dot$2`                                 | `…/node_modules/mathcat/dist/vec3.js:390`                       |     3.8 |    0.0 |  -3.8 | gone     |
| `releaseAllListenerContacts`            | `src/character/kcc.ts:591`                                      |     3.8 |    0.0 |  -3.8 | gone     |
| `resetContact`                          | `src/character/kcc.ts:715`                                      |     3.8 |    0.0 |  -3.8 | gone     |
| `sortContacts`                          | `src/character/kcc.ts:1253`                                     |     0.0 |    3.8 |  +3.8 | appeared |
| `handleContact`                         | `src/character/kcc.ts:2924`                                     |     0.0 |    3.8 |  +3.8 | appeared |
| `computeBarycentricCoordinates3d`       | `src/collision/closest-points.ts:57`                            |     5.7 |    2.0 |  -3.7 |          |
| `fromRotationTranslationScale`          | `…/node_modules/mathcat/dist/mat4.js:1400`                      |     3.7 |    0.0 |  -3.7 | gone     |
| `isScaleInsideOut$1`                    | `…/node_modules/mathcat/dist/vec3.js:756`                       |     3.7 |    0.0 |  -3.7 | gone     |
| `fixNormal`                             | `src/collision/active-edges.ts:74`                              |     0.0 |    3.7 |  +3.7 | appeared |
| (anonymous)                             | `src/character/kcc.ts:612`                                      |     3.6 |    0.0 |  -3.6 | gone     |
| `castAABB$1`                            | `src/broadphase/dbvt.ts:936`                                    |    13.7 |   10.1 |  -3.6 |          |
| `castConvexVsTriangleMesh`              | `src/shapes/triangle-mesh.ts:677`                               |    40.0 |   43.4 |  +3.4 |          |
| `register`                              | `src/shapes/static-compound.ts:168`                             |     0.0 |    3.3 |  +3.3 | appeared |
| `create$6`                              | `src/shapes/triangle-mesh.ts:113`                               |     0.0 |    3.2 |  +3.2 | appeared |
| `getContactsFromCache`                  | `src/update.ts:868`                                             |     3.1 |    0.0 |  -3.1 | gone     |
| `(garbage collector)`                   | ``                                                              |    18.9 |   21.9 |  +3.0 |          |
| `push`                                  | `src/utils/bvh-stack.ts:36`                                     |     8.6 |   11.3 |  +2.7 |          |
| `compareConstraints`                    | `src/character/kcc.ts:1895`                                     |     2.7 |    0.0 |  -2.7 | gone     |
| `clear`                                 | `src/ccd.ts:107`                                                |     2.7 |    0.0 |  -2.7 | gone     |
| `invert$2`                              | `…/node_modules/mathcat/dist/mat4.js:251`                       |     0.0 |    2.7 |  +2.7 | appeared |
| `(idle)`                                | ``                                                              |     3.6 |    6.3 |  +2.7 |          |
| `collideTransformedVsShape`             | `src/shapes/transformed.ts:377`                                 |     8.8 |    6.2 |  -2.6 |          |
| `moveShape`                             | `src/character/kcc.ts:2691`                                     |     6.5 |    3.9 |  -2.6 |          |
| `updateGroundVelocity`                  | `src/character/kcc.ts:3068`                                     |     2.6 |    0.0 |  -2.6 | gone     |
| `destroyUnprocessedContacts`            | `src/contacts.ts:700`                                           |     2.6 |    0.0 |  -2.6 | gone     |
| `insertLeaf`                            | `src/broadphase/dbvt.ts:118`                                    |     2.6 |    0.0 |  -2.6 | gone     |
| `addImpulseAtPosition`                  | `src/body/rigid-body.ts:800`                                    |     0.0 |    2.6 |  +2.6 | appeared |
| `visit`                                 | `src/query.ts:218`                                              |     5.1 |    2.5 |  -2.6 |          |
| `getShapeSupportingFace`                | `src/shapes/shapes.ts:494`                                      |     4.8 |    2.3 |  -2.5 |          |
| `processEdge`                           | `src/shapes/utils/triangle-mesh-builder.ts:340`                 |     2.5 |    0.0 |  -2.5 | gone     |
| `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:475`                    |     2.5 |    0.0 |  -2.5 | gone     |
| `fromRotationTranslation`               | `…/node_modules/mathcat/dist/mat4.js:1150`                      |     2.5 |    0.0 |  -2.5 | gone     |
| `set`                                   | `src/query.ts:258`                                              |     2.5 |    0.0 |  -2.5 | gone     |
| `scale$4`                               | `…/node_modules/mathcat/dist/vec3.js:277`                       |     2.5 |    0.0 |  -2.5 | gone     |
| `velocityIntegrationUpdate`             | `src/update.ts:1131`                                            |     2.5 |    0.0 |  -2.5 | gone     |
| `scale$1`                               | `…/node_modules/mathcat/dist/box3.js:294`                       |     2.5 |    0.0 |  -2.5 | gone     |
| (anonymous)                             | `file:///Users/isaacmason/Development/crashcat/dist/index.js:1` |     2.5 |    0.0 |  -2.5 | gone     |
| `accelerationIntegrationUpdate`         | `src/update.ts:287`                                             |     0.0 |    2.5 |  +2.5 | appeared |
| `transformMat4`                         | `…/node_modules/mathcat/dist/box3.js:326`                       |     0.0 |    2.5 |  +2.5 | appeared |
| `collideConvexVsConvexLocal`            | `src/shapes/convex.ts:329`                                      |     0.0 |    2.5 |  +2.5 | appeared |
| `castConvexVsConvex`                    | `src/shapes/convex.ts:531`                                      |     0.0 |    2.5 |  +2.5 | appeared |
| `copy$9`                                | `…/node_modules/mathcat/dist/vec3.js:58`                        |     0.0 |    2.5 |  +2.5 | appeared |
| `compareContactsForDeterminism`         | `src/character/kcc.ts:1245`                                     |     0.0 |    2.5 |  +2.5 | appeared |
| `multiply3x3Vec`                        | `…/node_modules/mathcat/dist/mat4.js:620`                       |     2.4 |    0.0 |  -2.4 | gone     |
| `updateInnerBodyTransform`              | `src/character/kcc.ts:3014`                                     |     2.4 |    0.0 |  -2.4 | gone     |
| `getSurfaceNormal`                      | `src/body/rigid-body.ts:924`                                    |     5.1 |    2.7 |  -2.4 |          |
| `rayDistanceToBox3`                     | `src/collision/cast-utils.ts:20`                                |    24.3 |   26.7 |  +2.4 |          |
| `castShape`                             | `src/query.ts:185`                                              |     2.5 |    4.8 |  +2.3 |          |
| `collideShapeVsShape`                   | `src/collision/narrowphase.ts:183`                              |     2.3 |    0.0 |  -2.3 | gone     |
| `hasContactsBetweenBodyIds`             | `src/contacts.ts:479`                                           |     2.2 |    0.0 |  -2.2 | gone     |
| `scaleAndAdd`                           | `…/node_modules/mathcat/dist/vec3.js:292`                       |     0.0 |    2.2 |  +2.2 | appeared |
| `rayHitsBox3`                           | `src/collision/cast-utils.ts:114`                               |     4.3 |    2.2 |  -2.1 |          |
| `finalize`                              | `src/islands.ts:240`                                            |     4.3 |    6.3 |  +2.0 |          |
| `manifoldBetweenTwoFaces`               | `src/manifold/manifold.ts:394`                                  |     0.0 |    1.9 |  +1.9 | appeared |
| `move`                                  | `src/character/kcc.ts:3335`                                     |     0.0 |    1.8 |  +1.8 | appeared |
| `moveToContact`                         | `src/character/kcc.ts:3406`                                     |     0.0 |    1.8 |  +1.8 | appeared |
| `build`                                 | `src/shapes/utils/triangle-mesh-bvh.ts:248`                     |     0.0 |    1.8 |  +1.8 | appeared |
| `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1399`                   |     4.1 |    2.5 |  -1.6 |          |
| `runSim`                                | `bench/kcc-mesh.bench.ts`                                       |     4.4 |    5.7 |  +1.3 |          |
| `buildTriangleMesh`                     | `src/shapes/utils/triangle-mesh-builder.ts:62`                  |     3.3 |    2.1 |  -1.2 |          |
| `clamp`                                 | `…/node_modules/mathcat/dist/common.js:69`                      |     3.8 |    2.7 |  -1.1 |          |
| `gjkClosestPoints`                      | `src/collision/gjk.ts:1225`                                     |    83.9 |   82.9 |  -1.0 |          |
| `updateAABB`                            | `src/body/rigid-body.ts:567`                                    |     3.5 |    2.5 |  -1.0 |          |
| `buildRecursive`                        | `src/shapes/utils/triangle-mesh-bvh.ts:308`                     |     3.3 |    4.3 |  +1.0 |          |
| `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1121`                   |     8.4 |    9.3 |  +0.9 |          |
| `normalize$2`                           | `…/node_modules/mathcat/dist/vec3.js:369`                       |     3.0 |    2.4 |  -0.6 |          |
| `intersectAABB$1`                       | `src/broadphase/dbvt.ts:627`                                    |    13.7 |   13.1 |  -0.6 |          |
| `subtract$1`                            | `…/node_modules/mathcat/dist/vec3.js:154`                       |     5.0 |    5.5 |  +0.5 |          |

