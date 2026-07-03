# perf diff — mesh-field → mesh-field

| side | scenario | git rev | attributed ms | date |
| --- | --- | --- | --- | --- |
| baseline | `mesh-field` | `9b65b0d` (dirty) | 1909.7 | 2026-07-03T13:48:56.836Z |
| current | `mesh-field` | `9b65b0d` (dirty) | 1835.4 | 2026-07-03T13:50:24.954Z |

**total attributed time:** 1909.7 ms → 1835.4 ms  (**-74.3 ms**, -3.9%)

## category deltas

| category      | base ms | cur ms |  Δ ms | Δ pts |   rel % |
| ------------- | ------: | -----: | ----: | ----: | ------: |
| shapes        |   381.5 |  477.0 | +95.5 |  +6.0 |  +25.0% |
| solver        |   489.3 |  411.4 | -77.9 |  -3.2 |  -15.9% |
| math          |   262.5 |  210.2 | -52.3 |  -2.2 |  -19.9% |
| narrowphase   |   284.5 |  260.0 | -24.5 |  -0.7 |   -8.6% |
| body          |    72.8 |   53.4 | -19.4 |  -0.9 |  -26.6% |
| broadphase    |   188.0 |  174.5 | -13.5 |  -0.3 |   -7.2% |
| step          |   135.8 |  147.7 | +11.9 |  +0.9 |   +8.8% |
| runtime       |    32.2 |   21.4 | -10.8 |  -0.5 |  -33.5% |
| manifold      |    34.4 |   44.9 | +10.5 |  +0.6 |  +30.5% |
| crashcat-util |    15.7 |   25.8 | +10.1 |  +0.6 |  +64.3% |
| bench-harness |     9.3 |    7.9 |  -1.4 |  -0.1 |  -15.1% |
| ccd           |     1.3 |    0.0 |  -1.3 |  -0.1 | -100.0% |
| other         |     2.5 |    1.3 |  -1.2 |  +0.0 |  -48.0% |

## function hotspot deltas

_matched by name + file; deltas below 0.5 ms hidden._

| function                                | location                                                      | base ms | cur ms |  Δ ms | note     |
| --------------------------------------- | ------------------------------------------------------------- | ------: | -----: | ----: | -------- |
| `getOptimalSplit`                       | `src/shapes/utils/triangle-mesh-bvh.ts:451`                   |     0.0 |   61.1 | +61.1 | appeared |
| `collideConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1213`                            |   139.0 |  168.4 | +29.4 |          |
| `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1121`                 |   169.4 |  147.3 | -22.1 |          |
| `getClosestPointOnTriangle`             | `src/collision/triangle.ts:62`                                |    45.3 |   25.0 | -20.3 |          |
| `intersectAABB$1`                       | `src/broadphase/dbvt.ts:627`                                  |   119.1 |  102.4 | -16.7 |          |
| `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:971`                  |    29.9 |   14.0 | -15.9 |          |
| `push`                                  | `src/utils/bvh-stack.ts:36`                                   |    10.5 |   24.5 | +14.0 |          |
| `gjkClosestPoints`                      | `src/collision/gjk.ts:1225`                                   |   124.6 |  111.5 | -13.1 |          |
| `subtract$1`                            | `…/node_modules/mathcat/dist/vec3.js:154`                     |    29.5 |   16.6 | -12.9 |          |
| `multiply$2`                            | `…/node_modules/mathcat/dist/vec3.js:182`                     |    48.7 |   36.5 | -12.2 |          |
| `setTriangleSupport`                    | `src/collision/support.ts:376`                                |     0.0 |   10.8 | +10.8 | appeared |
| `addContactConstraint`                  | `src/constraints/contact-constraints.ts:582`                  |    90.4 |   79.7 | -10.7 |          |
| `getSleepTestPoints`                    | `src/body/sleep.ts:22`                                        |    17.8 |    8.3 |  -9.5 |          |
| `cross`                                 | `…/node_modules/mathcat/dist/vec3.js:401`                     |    18.9 |    9.5 |  -9.4 |          |
| `updateWorld`                           | `src/update.ts:45`                                            |    42.4 |   50.8 |  +8.4 |          |
| `checkIslandSleep`                      | `src/islands.ts:378`                                          |    18.1 |   10.6 |  -7.5 |          |
| `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1399`                 |    28.6 |   21.1 |  -7.5 |          |
| `dot$2`                                 | `…/node_modules/mathcat/dist/vec3.js:390`                     |     7.5 |    0.0 |  -7.5 | gone     |
| `updateBodyPositions`                   | `src/update.ts:371`                                           |     3.7 |   11.2 |  +7.5 |          |
| `manifoldBetweenTwoFaces`               | `src/manifold/manifold.ts:394`                                |     5.3 |   12.1 |  +6.8 |          |
| `set$8`                                 | `…/node_modules/mathcat/dist/vec3.js:73`                      |     6.3 |    0.0 |  -6.3 | gone     |
| `getSupport`                            | `src/collision/support.ts:125`                                |    70.5 |   64.2 |  -6.3 |          |
| `reconstructManifoldFromCache`          | `src/update.ts:799`                                           |     0.0 |    6.0 |  +6.0 | appeared |
| `transformMat4$1`                       | `…/node_modules/mathcat/dist/vec3.js:530`                     |    45.3 |   39.5 |  -5.8 |          |
| `copy$9`                                | `…/node_modules/mathcat/dist/vec3.js:58`                      |     0.0 |    5.7 |  +5.7 | appeared |
| `getTriangleVertices`                   | `src/shapes/utils/triangle-mesh-data.ts:37`                   |    33.6 |   39.3 |  +5.7 |          |
| `multiply3x3TransposedVec`              | `…/node_modules/mathcat/dist/mat4.js:603`                     |    10.1 |    4.6 |  -5.5 |          |
| `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:475`                  |    14.6 |    9.2 |  -5.4 |          |
| `getContactsFromCache`                  | `src/update.ts:868`                                           |    20.8 |   15.5 |  -5.3 |          |
| `getShapeSupportingFace`                | `src/shapes/shapes.ts:494`                                    |     0.0 |    5.3 |  +5.3 | appeared |
| `(program)`                             | ``                                                            |     8.7 |    3.4 |  -5.3 |          |
| `finalizeAndCreateConstraints`          | `src/update.ts:577`                                           |     0.0 |    5.2 |  +5.2 | appeared |
| `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81`  |    13.7 |    8.6 |  -5.1 |          |
| `scale$4`                               | `…/node_modules/mathcat/dist/vec3.js:277`                     |     5.1 |    0.0 |  -5.1 | gone     |
| `fixNormal`                             | `src/collision/active-edges.ts:74`                            |     0.0 |    5.1 |  +5.1 | appeared |
| `collideShapeVsShape`                   | `src/collision/narrowphase.ts:183`                            |     0.0 |    5.1 |  +5.1 | appeared |
| `narrowphase`                           | `src/update.ts:949`                                           |    24.7 |   19.7 |  -5.0 |          |
| `(garbage collector)`                   | ``                                                            |    18.5 |   13.5 |  -5.0 |          |
| `getSupportingFace$8`                   | `src/shapes/convex-hull.ts:529`                               |     5.0 |    0.0 |  -5.0 | gone     |
| `linkContactConstraints`                | `src/islands.ts:184`                                          |     5.0 |    0.0 |  -5.0 | gone     |
| `setContactSettings`                    | `src/constraints/contact-constraints.ts:243`                  |     4.9 |    0.0 |  -4.9 | gone     |
| `getVelocityAtPointCOM`                 | `src/body/rigid-body.ts:1007`                                 |    14.7 |    9.8 |  -4.9 |          |
| `velocityIntegrationUpdate`             | `src/update.ts:1131`                                          |    11.0 |   15.5 |  +4.5 |          |
| `addHit`                                | `src/update.ts:432`                                           |    18.8 |   14.4 |  -4.4 |          |
| `create$6`                              | `src/shapes/triangle-mesh.ts:113`                             |     4.4 |    0.0 |  -4.4 | gone     |
| `update$1`                              | `src/broadphase/dbvt.ts:485`                                  |    49.1 |   44.7 |  -4.4 |          |
| `collideSphereVsSphere`                 | `src/shapes/sphere.ts:222`                                    |     4.3 |    0.0 |  -4.3 | gone     |
| `setShapeSupport`                       | `src/shapes/shapes.ts:341`                                    |     0.0 |    4.3 |  +4.3 | appeared |
| `transformQuat`                         | `…/node_modules/mathcat/dist/vec3.js:567`                     |    13.7 |    9.4 |  -4.3 |          |
| `accelerationIntegrationUpdate`         | `src/update.ts:287`                                           |    12.6 |    8.3 |  -4.3 |          |
| `calculateNormalVelocityBias`           | `src/constraints/contact-constraints.ts:309`                  |    16.1 |   20.3 |  +4.2 |          |
| `buildRecursive`                        | `src/shapes/utils/triangle-mesh-bvh.ts:308`                   |    10.5 |   14.5 |  +4.0 |          |
| `expandByExtents`                       | `…/node_modules/mathcat/dist/box3.js:197`                     |     3.8 |    0.0 |  -3.8 | gone     |
| `getDeduplicationIndex`                 | `src/broadphase/broadphase.ts:54`                             |     3.8 |    0.0 |  -3.8 | gone     |
| `findEdge$1`                            | `src/collision/epa-convex-hull-builder.ts:548`                |     3.7 |    0.0 |  -3.7 | gone     |
| `setAxisAngle`                          | `…/node_modules/mathcat/dist/quat.js:36`                      |     0.0 |    3.7 |  +3.7 | appeared |
| `fromRotationTranslationScale`          | `…/node_modules/mathcat/dist/mat4.js:1400`                    |     0.0 |    3.7 |  +3.7 | appeared |
| `clipPolyVsPlane`                       | `src/manifold/clip.ts:13`                                     |     6.3 |    9.9 |  +3.6 |          |
| `createContact`                         | `src/contacts.ts:373`                                         |     3.6 |    0.0 |  -3.6 | gone     |
| `addPoint$1`                            | `src/collision/epa-convex-hull-builder.ts:629`                |     3.5 |    0.0 |  -3.5 | gone     |
| `setCachedBodyPair`                     | `src/contacts.ts:174`                                         |     7.7 |    4.3 |  -3.4 |          |
| `set$7`                                 | `…/node_modules/mathcat/dist/vec4.js:66`                      |     0.0 |    3.3 |  +3.3 | appeared |
| `runSim`                                | `bench/mesh-field.bench.ts`                                   |     0.0 |    3.2 |  +3.2 | appeared |
| `add$3`                                 | `…/node_modules/mathcat/dist/vec3.js:126`                     |     8.5 |    5.6 |  -2.9 |          |
| `updateBody`                            | `src/broadphase/broadphase.ts:196`                            |     0.0 |    2.8 |  +2.8 | appeared |
| `conjugate`                             | `…/node_modules/mathcat/dist/quat.js:333`                     |     2.7 |    0.0 |  -2.7 | gone     |
| `setCapsuleSupport`                     | `src/collision/support.ts:336`                                |     0.0 |    2.7 |  +2.7 | appeared |
| `hasContactsBetweenBodyIds`             | `src/contacts.ts:479`                                         |     8.8 |    6.2 |  -2.6 |          |
| `getSupportingFace$11`                  | `src/shapes/box.ts:157`                                       |     2.6 |    0.0 |  -2.6 | gone     |
| `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                           |    18.9 |   21.3 |  +2.4 |          |
| `storeAppliedImpulses`                  | `src/constraints/contact-constraints.ts:1359`                 |    10.1 |   12.5 |  +2.4 |          |
| `findCollidingPairs`                    | `src/broadphase/broadphase.ts:213`                            |    12.4 |   14.7 |  +2.3 |          |
| `clipPolyVsEdge`                        | `src/manifold/clip.ts:190`                                    |    15.4 |   13.5 |  -1.9 |          |
| `bounds$2`                              | `…/node_modules/mathcat/dist/triangle3.js:9`                  |     7.6 |    9.3 |  +1.7 |          |
| `finalize`                              | `src/islands.ts:240`                                          |    22.2 |   23.9 |  +1.7 |          |
| `calculateConstraintProperties$5`       | `src/constraints/constraint-part/axis-constraint-part.ts:164` |     5.0 |    3.5 |  -1.5 |          |
| `destroyContact`                        | `src/contacts.ts:435`                                         |     5.0 |    6.3 |  +1.3 |          |
| `updateAABB`                            | `src/body/rigid-body.ts:567`                                  |     7.6 |    6.5 |  -1.1 |          |
| `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                             |    24.9 |   23.8 |  -1.1 |          |
| `intersectsBox3$1`                      | `…/node_modules/mathcat/dist/box3.js:392`                     |    16.1 |   17.2 |  +1.1 |          |
| `collideSphereVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1727`                            |   147.7 |  146.7 |  -1.0 |          |
| `sortContactIndices`                    | `src/constraints/contact-constraints.ts:1541`                 |     6.1 |    5.5 |  -0.6 |          |
| `normalize$2`                           | `…/node_modules/mathcat/dist/vec3.js:369`                     |     7.5 |    8.0 |  +0.5 |          |
| `(idle)`                                | ``                                                            |     5.0 |    4.5 |  -0.5 |          |
| `findContact`                           | `src/contacts.ts:647`                                         |     3.8 |    4.3 |  +0.5 |          |

