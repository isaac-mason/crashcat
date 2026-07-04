# perf report — raycasts

| field | value |
| --- | --- |
| scenario | `raycasts` |
| date | 2026-07-04T08:04:43.408Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| wall | 1184.3 ms |
| attributed | 1020.8 ms |
| startup excluded | 163.3 ms |
| samples | 995 |

## by category

| category      |   pct |    ms |
| ------------- | ----: | ----: |
| shapes        | 31.0% | 316.5 |
| narrowphase   | 23.1% | 235.7 |
| math          | 11.2% | 114.4 |
| broadphase    | 11.0% | 112.0 |
| solver        | 10.1% | 103.3 |
| bench-harness |  4.5% |  46.2 |
| step          |  3.0% |  30.3 |
| runtime       |  2.3% |  23.9 |
| body          |  2.0% |  20.3 |
| crashcat-util |  1.7% |  16.9 |
| manifold      |  0.1% |   1.3 |

## top 30 self-time hotspots

|   # |   pct |    ms | function                            | location                                       |
| --: | ----: | ----: | ----------------------------------- | ---------------------------------------------- |
|   1 | 22.5% | 229.3 | `castRayVsTriangleMesh`             | `src/shapes/triangle-mesh.ts:267`              |
|   2 | 19.6% | 199.9 | `rayDistanceToBox3`                 | `src/collision/cast-utils.ts:20`               |
|   3 |  9.4% |  95.5 | `castRay$3`                         | `src/broadphase/dbvt.ts:825`                   |
|   4 |  6.8% |  69.3 | `intersectsTriangle`                | `…/node_modules/mathcat/dist/raycast3.js:90`   |
|   5 |  4.3% |  43.8 | `collideSphereVsTriangleMesh`       | `src/shapes/triangle-mesh.ts:1755`             |
|   6 |  4.1% |  41.5 | (anonymous)                         | `bench/raycasts.bench.ts`                      |
|   7 |  3.8% |  38.9 | `solveVelocityConstraintsForIsland` | `src/constraints/contact-constraints.ts:1127`  |
|   8 |  2.0% |  20.3 | `addContactConstraint`              | `src/constraints/contact-constraints.ts:583`   |
|   9 |  1.9% |  18.9 | `(garbage collector)`               | ``                                             |
|  10 |  1.8% |  18.0 | `getTriangleVertices`               | `src/shapes/utils/triangle-mesh-data.ts:44`    |
|  11 |  1.7% |  17.7 | `getClosestPointOnTriangle`         | `src/collision/triangle.ts:62`                 |
|  12 |  1.5% |  14.9 | `updateWorld`                       | `src/update.ts:46`                             |
|  13 |  1.2% |  11.8 | `update$11`                         | `src/broadphase/dbvt.ts:480`                   |
|  14 |  1.1% |  11.1 | `findCollidingPairs`                | `src/pairs.ts:409`                             |
|  15 |  0.9% |   8.8 | `multiply$2`                        | `…/node_modules/mathcat/dist/vec3.js:182`      |
|  16 |  0.7% |   7.5 | `warmStartVelocityConstraints`      | `src/constraints/contact-constraints.ts:977`   |
|  17 |  0.7% |   7.5 | `transformQuat`                     | `…/node_modules/mathcat/dist/vec3.js:567`      |
|  18 |  0.6% |   6.4 | `getInverseInertiaForRotation`      | `src/body/motion-properties.ts:500`            |
|  19 |  0.6% |   5.9 | `cross`                             | `…/node_modules/mathcat/dist/vec3.js:401`      |
|  20 |  0.5% |   5.5 | `castRayVsConvex`                   | `src/shapes/convex.ts:43`                      |
|  21 |  0.5% |   5.5 | `castRayVsShape`                    | `src/collision/narrowphase.ts:24`              |
|  22 |  0.5% |   5.1 | `rayHitsBox3`                       | `src/collision/cast-utils.ts:114`              |
|  23 |  0.5% |   5.0 | `narrowphase`                       | `src/update.ts:998`                            |
|  24 |  0.4% |   4.5 | `buildTriangleMesh`                 | `src/shapes/utils/triangle-mesh-builder.ts:63` |
|  25 |  0.4% |   4.2 | `accelerationIntegrationUpdate`     | `src/update.ts:293`                            |
|  26 |  0.4% |   4.1 | `checkIslandSleep`                  | `src/islands.ts:382`                           |
|  27 |  0.4% |   3.8 | `resetCachedManifold`               | `src/contacts.ts:226`                          |
|  28 |  0.4% |   3.8 | `multiply3x3TransposedVec`          | `…/node_modules/mathcat/dist/mat4.js:603`      |
|  29 |  0.4% |   3.8 | `solvePositionConstraintsForIsland` | `src/constraints/contact-constraints.ts:1405`  |
|  30 |  0.4% |   3.8 | `push`                              | `src/body/sub-shape.ts:39`                     |

