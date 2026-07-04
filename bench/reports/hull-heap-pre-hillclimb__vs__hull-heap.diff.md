# perf diff — hull-heap → hull-heap

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `hull-heap` | `7ee6b81` (dirty) | 11724.8 | 2026-07-04T09:33:06.036Z |
| current | `hull-heap` | `e8316a4` (dirty) | 9962.8 | 2026-07-04T09:53:13.556Z |

**total attributed time:** 11724.8 ms → 9962.8 ms  (**-1762.0 ms**, -15.0%)

## category deltas

| category      | base ms | cur ms |   Δ ms | Δ pts |  rel % |
| ------------- | ------: | -----: | -----: | ----: | -----: |
| narrowphase   |  4896.6 | 3934.5 | -962.1 |  -2.3 | -19.6% |
| shapes        |  1324.2 | 1011.9 | -312.3 |  -1.1 | -23.6% |
| solver        |  3201.8 | 2900.9 | -300.9 |  +1.8 |  -9.4% |
| math          |   393.8 |  282.7 | -111.1 |  -0.6 | -28.2% |
| runtime       |   111.5 |   81.2 |  -30.3 |  -0.2 | -27.2% |
| broadphase    |   372.5 |  343.3 |  -29.2 |  +0.2 |  -7.8% |
| step          |   617.4 |  641.8 |  +24.4 |  +1.1 |  +4.0% |
| body          |   437.9 |  419.1 |  -18.8 |  +0.5 |  -4.3% |
| manifold      |   182.8 |  167.1 |  -15.7 |  +0.1 |  -8.6% |
| bench-harness |    21.6 |   11.0 |  -10.6 |  -0.1 | -49.1% |
| crashcat-util |   156.0 |  160.8 |   +4.8 |  +0.3 |  +3.1% |
| other         |     3.5 |    2.3 |   -1.2 |  +0.0 | -34.3% |
| ccd           |     5.0 |    6.1 |   +1.1 |  +0.1 | +22.0% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                  | location                                                      | base ms | cur ms |   Δ ms | note     |
| ----------------------------------------- | ------------------------------------------------------------- | ------: | -----: | -----: | -------- |
| `getSupport`                              | `src/collision/support.ts:139`                                |  3255.9 | 2269.4 | -986.5 |          |
| `getSupportingFace$8`                     | `src/shapes/convex-hull.ts:576`                               |  1025.5 |  641.4 | -384.1 |          |
| `addContactConstraint`                    | `src/constraints/contact-constraints.ts:583`                  |   528.1 |  438.5 |  -89.6 |          |
| `solveVelocityConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1127`                 |  1438.0 | 1367.7 |  -70.3 |          |
| `multiply3x3TransposedVec`                | `…/node_modules/mathcat/dist/mat4.js:603`                     |   107.9 |   51.1 |  -56.8 |          |
| `(program)`                               | ``                                                            |    57.9 |    5.1 |  -52.8 |          |
| `penetrationDepthStepGJK`                 | `src/collision/penetration.ts:29`                             |    71.9 |  115.4 |  +43.5 |          |
| `collideConvexVsConvex`                   | `src/shapes/convex.ts:247`                                    |    68.2 |  109.3 |  +41.1 |          |
| `calculateFrictionConstraintProperties`   | `src/constraints/contact-constraints.ts:476`                  |   101.6 |   61.7 |  -39.9 |          |
| `calculateNormalVelocityBias`             | `src/constraints/contact-constraints.ts:310`                  |    93.1 |   62.7 |  -30.4 |          |
| `update$11`                               | `src/broadphase/dbvt.ts:480`                                  |    76.1 |   45.8 |  -30.3 |          |
| `warmStartVelocityConstraints`            | `src/constraints/contact-constraints.ts:977`                  |   205.5 |  176.5 |  -29.0 |          |
| `createTriangle$1`                        | `src/collision/epa-convex-hull-builder.ts:173`                |   344.7 |  317.5 |  -27.2 |          |
| `narrowphase`                             | `src/update.ts:998`                                           |   204.5 |  228.6 |  +24.1 |          |
| `setHullSupport`                          | `src/collision/support.ts:736`                                |    16.2 |   40.1 |  +23.9 |          |
| `normalize$2`                             | `…/node_modules/mathcat/dist/vec3.js:369`                     |    81.9 |   59.7 |  -22.2 |          |
| `gjkClosestPoints`                        | `src/collision/gjk.ts:1246`                                   |   491.3 |  470.0 |  -21.3 |          |
| `(idle)`                                  | ``                                                            |     0.0 |   19.3 |  +19.3 | appeared |
| `penetrationDepthStepEPA`                 | `src/collision/penetration.ts:210`                            |   197.9 |  215.7 |  +17.8 |          |
| `getInverseInertiaForRotation`            | `src/body/motion-properties.ts:500`                           |   183.6 |  165.9 |  -17.7 |          |
| `clipPolyVsPlane`                         | `src/manifold/clip.ts:13`                                     |   101.5 |   84.7 |  -16.8 |          |
| `storeAppliedImpulses`                    | `src/constraints/contact-constraints.ts:1365`                 |   106.6 |   90.2 |  -16.4 |          |
| `solvePositionConstraintsForIsland`       | `src/constraints/contact-constraints.ts:1405`                 |   269.2 |  255.1 |  -14.1 |          |
| `pruneContactPoints`                      | `src/manifold/manifold.ts:168`                                |    12.5 |   26.3 |  +13.8 |          |
| `getContactsFromCache`                    | `src/update.ts:914`                                           |    66.3 |   79.7 |  +13.4 |          |
| `fromRotationTranslation`                 | `…/node_modules/mathcat/dist/mat4.js:1150`                    |    54.4 |   41.1 |  -13.3 |          |
| `updateSleepState`                        | `src/body/sleep.ts:89`                                        |    28.8 |   42.0 |  +13.2 |          |
| `addEpaSupportPoint`                      | `src/collision/penetration.ts:150`                            |    26.6 |   13.6 |  -13.0 |          |
| `getShapeSupportingFace`                  | `src/shapes/shapes.ts:489`                                    |    40.7 |   53.7 |  +13.0 |          |
| `addPoint$1`                              | `src/collision/epa-convex-hull-builder.ts:596`                |   166.7 |  179.5 |  +12.8 |          |
| `getVelocityAtPointCOM`                   | `src/body/rigid-body.ts:1035`                                 |   147.0 |  134.6 |  -12.4 |          |
| `getOrthogonalBasis`                      | `src/shapes/plane.ts:211`                                     |     9.8 |   21.9 |  +12.1 |          |
| (anonymous)                               | `bench/hull-heap.bench.ts`                                    |    18.2 |    6.7 |  -11.5 |          |
| `createContact`                           | `src/contacts.ts:313`                                         |    19.0 |    7.7 |  -11.3 |          |
| `combineMaterial`                         | `src/constraints/combine-material.ts:40`                      |    39.7 |   28.6 |  -11.1 |          |
| `resetForces`                             | `src/update.ts:1976`                                          |    11.1 |    0.0 |  -11.1 | gone     |
| `(garbage collector)`                     | ``                                                            |    46.0 |   56.8 |  +10.8 |          |
| `intersectAABBFatLeaves`                  | `src/broadphase/dbvt.ts:590`                                  |   265.1 |  275.6 |  +10.5 |          |
| `fromQuat$1`                              | `…/node_modules/mathcat/dist/mat4.js:1518`                    |    23.7 |   13.2 |  -10.5 |          |
| `updateAABB`                              | `src/body/rigid-body.ts:580`                                  |    29.0 |   18.7 |  -10.3 |          |
| `setContactSettings`                      | `src/constraints/contact-constraints.ts:244`                  |    46.5 |   36.4 |  -10.1 |          |
| `transformMat4$1`                         | `…/node_modules/mathcat/dist/vec3.js:530`                     |    20.3 |   10.2 |  -10.1 |          |
| `finalize`                                | `src/islands.ts:240`                                          |    32.0 |   42.0 |  +10.0 |          |
| `clipPolyVsPoly`                          | `src/manifold/clip.ts:108`                                    |    33.0 |   23.9 |   -9.1 |          |
| `calculateInverseEffectiveMass`           | `src/constraints/constraint-part/axis-constraint-part.ts:81`  |   104.7 |   95.9 |   -8.8 |          |
| `clamp`                                   | `…/node_modules/mathcat/dist/common.js:69`                    |     0.0 |    8.5 |   +8.5 | appeared |
| `optimizeIncremental`                     | `src/broadphase/dbvt.ts:544`                                  |     8.3 |    0.0 |   -8.3 | gone     |
| `accelerationIntegrationUpdate`           | `src/update.ts:293`                                           |    19.8 |   11.6 |   -8.2 |          |
| `transformPlane`                          | `src/shapes/plane.ts:197`                                     |     7.6 |    0.0 |   -7.6 | gone     |
| `scaleAndAdd`                             | `…/node_modules/mathcat/dist/vec3.js:292`                     |     0.0 |    7.6 |   +7.6 | appeared |
| `updateWorld`                             | `src/update.ts:46`                                            |   168.5 |  160.9 |   -7.6 |          |
| `multiply$1`                              | `…/node_modules/mathcat/dist/mat4.js:448`                     |     7.7 |   15.2 |   +7.5 |          |
| `normalize$1`                             | `…/node_modules/mathcat/dist/vec4.js:325`                     |     7.4 |    0.0 |   -7.4 | gone     |
| `findCollidingPairs`                      | `src/pairs.ts:409`                                            |   147.2 |  154.4 |   +7.2 |          |
| `insertLeaf`                              | `src/broadphase/dbvt.ts:111`                                  |     6.8 |    0.0 |   -6.8 | gone     |
| `clear`                                   | `src/ccd.ts:107`                                              |     0.0 |    6.1 |   +6.1 | appeared |
| `findContactInPair`                       | `src/contacts.ts:457`                                         |     7.9 |   13.6 |   +5.7 |          |
| `isLeaf`                                  | `src/broadphase/dbvt.ts:89`                                   |     0.0 |    5.3 |   +5.3 | appeared |
| `removeLeaf`                              | `src/broadphase/dbvt.ts:395`                                  |    10.2 |    5.1 |   -5.1 |          |
| `multiply`                                | `…/node_modules/mathcat/dist/quat.js:93`                      |    17.1 |   12.5 |   -4.6 |          |
| `computeBarycentricCoordinates3d`         | `src/collision/closest-points.ts:52`                          |    12.8 |    8.6 |   -4.2 |          |
| `updatePositionFromCenterOfMass`          | `src/body/rigid-body.ts:555`                                  |     9.8 |   14.0 |   +4.2 |          |
| `computeBodyPairDelta`                    | `src/update.ts:826`                                           |    39.0 |   43.1 |   +4.1 |          |
| `collideConvexVsPlane`                    | `src/shapes/plane.ts:483`                                     |    28.1 |   32.2 |   +4.1 |          |
| `linkContactConstraints`                  | `src/islands.ts:184`                                          |    48.0 |   52.0 |   +4.0 |          |
| (anonymous)                               | `src/constraints/contact-constraints.ts:1548`                 |    29.2 |   32.9 |   +3.7 |          |
| `manifoldBetweenTwoFaces`                 | `src/manifold/manifold.ts:394`                                |    31.8 |   28.2 |   -3.6 |          |
| `collideConvexVsConvexLocal`              | `src/shapes/convex.ts:329`                                    |   122.5 |  125.8 |   +3.3 |          |
| `getAdaptivePlaneSupportingFace`          | `src/shapes/plane.ts:636`                                     |    10.0 |   12.5 |   +2.5 |          |
| `solvePositionConstraintWithMassOverride` | `src/constraints/constraint-part/axis-constraint-part.ts:810` |    20.2 |   22.6 |   +2.4 |          |
| `sortContactIndices`                      | `src/constraints/contact-constraints.ts:1547`                 |    80.0 |   78.0 |   -2.0 |          |
| `cross`                                   | `…/node_modules/mathcat/dist/vec3.js:401`                     |    20.0 |   22.0 |   +2.0 |          |
| `findEdge$1`                              | `src/collision/epa-convex-hull-builder.ts:506`                |   274.2 |  272.3 |   -1.9 |          |
| `initialize$2`                            | `src/collision/epa-convex-hull-builder.ts:353`                |    20.3 |   22.1 |   +1.8 |          |
| `popClosestTriangleFromQueue`             | `src/collision/epa-convex-hull-builder.ts:385`                |     6.5 |    5.1 |   -1.4 |          |
| `updateBodyPositions`                     | `src/update.ts:377`                                           |    10.1 |   11.3 |   +1.2 |          |
| `addHit`                                  | `src/update.ts:455`                                           |    41.5 |   42.5 |   +1.0 |          |
| `setAxisAngle`                            | `…/node_modules/mathcat/dist/quat.js:36`                      |    12.4 |   11.4 |   -1.0 |          |
| `getSleepTestPoints`                      | `src/body/sleep.ts:22`                                        |    19.2 |   20.1 |   +0.9 |          |
| `reconstructManifoldFromCache`            | `src/update.ts:845`                                           |    20.5 |   21.3 |   +0.8 |          |
| `velocityIntegrationUpdate`               | `src/update.ts:1181`                                          |    32.4 |   33.1 |   +0.7 |          |
| `resetCachedManifold`                     | `src/contacts.ts:226`                                         |     7.3 |    6.6 |   -0.7 |          |

