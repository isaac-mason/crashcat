# perf report — mesh-field

| field | value |
| --- | --- |
| scenario | `mesh-field` |
| date | 2026-07-03T14:15:20.380Z |
| git rev | `9b65b0d` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 2005.1 ms |
| attributed | 1816.8 ms |
| startup excluded | 187.7 ms |
| samples | 1807 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 24.0% | 435.8 |
| shapes        | 20.8% | 377.0 |
| math          | 14.7% | 266.6 |
| narrowphase   | 13.9% | 251.8 |
| broadphase    |  9.6% | 174.8 |
| step          |  7.1% | 128.3 |
| body          |  4.1% |  75.2 |
| manifold      |  2.1% |  37.5 |
| crashcat-util |  1.7% |  30.9 |
| runtime       |  1.3% |  23.0 |
| bench-harness |  0.7% |  13.5 |
| other         |  0.1% |   2.4 |

## top 30 self-time hotspots

|   # |  pct |    ms | function                                | location                                                     |
| --: | ---: | ----: | --------------------------------------- | ------------------------------------------------------------ |
|   1 | 9.1% | 164.5 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1121`                |
|   2 | 8.8% | 159.0 | `collideConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1209`                           |
|   3 | 7.0% | 127.1 | `collideSphereVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1723`                           |
|   4 | 5.6% | 102.0 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1225`                                  |
|   5 | 5.3% |  97.0 | `intersectAABB$1`                       | `src/broadphase/dbvt.ts:627`                                 |
|   6 | 4.4% |  80.7 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:582`                 |
|   7 | 3.1% |  56.8 | `getSupport`                            | `src/collision/support.ts:125`                               |
|   8 | 3.0% |  54.1 | `multiply$2`                            | `…/node_modules/mathcat/dist/vec3.js:182`                    |
|   9 | 2.8% |  50.5 | `update$1`                              | `src/broadphase/dbvt.ts:485`                                 |
|  10 | 2.4% |  44.2 | `transformMat4$1`                       | `…/node_modules/mathcat/dist/vec3.js:530`                    |
|  11 | 2.1% |  38.7 | `updateWorld`                           | `src/update.ts:45`                                           |
|  12 | 2.1% |  37.5 | `getClosestPointOnTriangle`             | `src/collision/triangle.ts:62`                               |
|  13 | 1.6% |  29.2 | `subtract$1`                            | `…/node_modules/mathcat/dist/vec3.js:154`                    |
|  14 | 1.5% |  27.3 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1399`                |
|  15 | 1.5% |  26.7 | `getTriangleVertices`                   | `src/shapes/utils/triangle-mesh-data.ts:44`                  |
|  16 | 1.4% |  25.2 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                          |
|  17 | 1.3% |  24.1 | `finalize`                              | `src/islands.ts:240`                                         |
|  18 | 1.3% |  24.1 | `intersectsBox3$1`                      | `…/node_modules/mathcat/dist/box3.js:392`                    |
|  19 | 1.2% |  22.1 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                            |
|  20 | 1.1% |  20.8 | `push`                                  | `src/utils/bvh-stack.ts:36`                                  |
|  21 | 1.1% |  19.6 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:475`                 |
|  22 | 1.1% |  19.4 | `transformQuat`                         | `…/node_modules/mathcat/dist/vec3.js:567`                    |
|  23 | 1.0% |  19.0 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:971`                 |
|  24 | 1.0% |  17.6 | `getContactsFromCache`                  | `src/update.ts:868`                                          |
|  25 | 0.9% |  17.2 | `velocityIntegrationUpdate`             | `src/update.ts:1131`                                         |
|  26 | 0.9% |  17.0 | `calculateInverseEffectiveMass`         | `src/constraints/constraint-part/axis-constraint-part.ts:81` |
|  27 | 0.9% |  16.7 | `addHit`                                | `src/update.ts:432`                                          |
|  28 | 0.9% |  15.8 | `multiply3x3TransposedVec`              | `…/node_modules/mathcat/dist/mat4.js:603`                    |
|  29 | 0.9% |  15.7 | `(garbage collector)`                   | ``                                                           |
|  30 | 0.9% |  15.5 | `narrowphase`                           | `src/update.ts:949`                                          |

