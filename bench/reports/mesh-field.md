# perf report — mesh-field

| field | value |
| --- | --- |
| scenario | `mesh-field` |
| date | 2026-07-03T05:02:20.337Z |
| git rev | `ab84e99` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 2934.9 ms |
| attributed | 2934 ms |
| samples | 2111 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| solver        | 23.1% | 679.0 |
| shapes        | 20.3% | 595.9 |
| runtime       | 12.7% | 371.3 |
| narrowphase   | 11.1% | 325.2 |
| math          | 10.9% | 320.4 |
| step          |  8.3% | 242.6 |
| broadphase    |  6.2% | 182.5 |
| body          |  3.9% | 113.8 |
| manifold      |  1.8% |  53.7 |
| crashcat-util |  0.8% |  23.5 |
| bench-harness |  0.8% |  23.3 |
| other         |  0.1% |   2.9 |

## top 30 self-time hotspots

|   # |  pct |    ms | function                                | location                                                                 |
| --: | ---: | ----: | --------------------------------------- | ------------------------------------------------------------------------ |
|   1 | 7.3% | 213.8 | `collideConvexVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1211`                                       |
|   2 | 6.8% | 199.6 | `solveVelocityConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1121`                            |
|   3 | 5.3% | 154.4 | `collideSphereVsTriangleMesh`           | `src/shapes/triangle-mesh.ts:1725`                                       |
|   4 | 4.8% | 139.9 | `gjkClosestPoints`                      | `src/collision/gjk.ts:1225`                                              |
|   5 | 3.9% | 113.8 | `addContactConstraint`                  | `src/constraints/contact-constraints.ts:582`                             |
|   6 | 3.7% | 108.8 | `intersectAABB$1`                       | `src/broadphase/dbvt.ts:627`                                             |
|   7 | 3.5% | 101.5 | `(garbage collector)`                   | ``                                                                       |
|   8 | 2.8% |  82.6 | `updateWorld`                           | `src/update.ts:45`                                                       |
|   9 | 2.6% |  75.0 | `getSupport`                            | `src/collision/support.ts:125`                                           |
|  10 | 2.4% |  71.4 | `(idle)`                                | ``                                                                       |
|  11 | 2.4% |  69.7 | `multiply$2`                            | `…/node_modules/mathcat/dist/vec3.js:182`                                |
|  12 | 2.2% |  63.5 | `getTriangleVertices`                   | `src/shapes/utils/triangle-mesh-data.ts:37`                              |
|  13 | 1.9% |  55.6 | `transformMat4$1`                       | `…/node_modules/mathcat/dist/vec3.js:530`                                |
|  14 | 1.9% |  54.8 | `solvePositionConstraintsForIsland`     | `src/constraints/contact-constraints.ts:1399`                            |
|  15 | 1.8% |  53.3 | `narrowphase`                           | `src/update.ts:949`                                                      |
|  16 | 1.8% |  53.2 | `update$1`                              | `src/broadphase/dbvt.ts:485`                                             |
|  17 | 1.5% |  43.5 | `calculateFrictionConstraintProperties` | `src/constraints/contact-constraints.ts:475`                             |
|  18 | 1.5% |  43.4 | `buildTriangleMesh`                     | `src/shapes/utils/triangle-mesh-builder.ts:62`                           |
|  19 | 1.4% |  40.4 | `finalize`                              | `src/islands.ts:240`                                                     |
|  20 | 1.3% |  36.8 | `warmStartVelocityConstraints`          | `src/constraints/contact-constraints.ts:971`                             |
|  21 | 1.2% |  35.0 | `waitForWorker`                         | `node:internal/modules/esm/hooks`                                        |
|  22 | 1.2% |  35.0 | `makeSyncRequest`                       | `node:internal/modules/esm/hooks`                                        |
|  23 | 1.1% |  32.9 | `getInverseInertiaForRotation`          | `src/body/motion-properties.ts:500`                                      |
|  24 | 1.1% |  31.1 | `getContactsFromCache`                  | `src/update.ts:868`                                                      |
|  25 | 1.1% |  31.1 | `getClosestPointOnTriangle`             | `src/collision/triangle.ts:62`                                           |
|  26 | 1.0% |  29.9 | `isActive`                              | `src/constraints/constraint-part/angular-friction-constraint-part.ts:62` |
|  27 | 1.0% |  28.0 | `penetrationDepthStepGJK`               | `src/collision/penetration.ts:29`                                        |
|  28 | 0.8% |  24.4 | `getSleepTestPoints`                    | `src/body/sleep.ts:22`                                                   |
|  29 | 0.8% |  23.2 | `velocityIntegrationUpdate`             | `src/update.ts:1131`                                                     |
|  30 | 0.8% |  22.2 | `storeAppliedImpulses`                  | `src/constraints/contact-constraints.ts:1359`                            |

