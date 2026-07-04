# CHANGELOG

## v0.0.5 (Unreleased)

- perf: pair discovery skips pairs that can never collide by motion type / sensor role (`bodiesCanCollide`, shared with emission) — overlapping static geometry no longer accrues dead persistent pair records (a dense static field previously created tens of thousands, swept every frame)
- fix: broadphase `castAABB` spuriously pruned in-range nodes for casts shorter than 1 unit (fraction compared against world length); it now does best-t pruning against the collector's positive early-out fraction like `castRay`, and drops the redundant per-node re-test
- fix: the CCD ray-vs-expanded-AABB early-out had the same fraction/length mismatch and could cull valid CCD candidates for sub-unit sweeps
- refactor: **breaking** — `dbvt.rebuild(dbvt)` no longer takes a max-depth argument; the always-re-partition depth is a per-tree `DBVT.maxDepthMarkChanged` field
- refactor: cast hot paths no longer use `raycast3` structs — scalar ray args throughout, with `rayIntersectsTriangle` joining `rayDistanceToBox3`/`rayHitsBox3` in cast-utils; bit-identical results
- perf: dirty-gated balanced DBVT rebuild replaces Bullet's incremental-only rotation — a settled/static tree is rebuilt once into a balanced structure (leaf-preserving partial rebuild à la Jolt's `QuadTree::UpdatePrepare`, gated by a dirty flag, one tree per step) then left alone, instead of forever rotating a degenerate tree; a large static field goes from tree height ~127 to ~16 (~5× fewer node visits per ray)
- perf: analytic ray-vs-box narrowphase (`castRayVsBox`) replaces the generic GJK convex cast for box shapes — a local-space slab test matching Jolt's `BoxShape::CastRay` and meep's `ray_box_local`; box-ray narrowphase ~4× cheaper, bit-identical results
- perf: broadphase `castRay` prunes nodes whose fat-AABB entry is beyond the closest hit found so far (best-t), threading the collector's early-out fraction into the walk like Jolt/meep
- perf: DBVT node storage is now two flat parallel arrays — `topo: number[]` (PACKED_SMI: parent/left/right/body/changed) and `bounds: number[]` (PACKED_DOUBLE: 6 floats) — instead of a node object with a separate `Box3`, removing the pointer-chase (broadphase walk ~22% faster); external callers use `readNodeAabb`/`nodeLeft`/`nodeRight`/`nodeParent`/`nodeChanged`/`nodeBodyIndex`/`nodeCount` accessors
- perf: hot dbvt traversals (`castRay`, overlap/point queries, shape cast) inline their ray-slab / aabb tests via compilecat `@optimize`, removing per-node call + array-indexing overhead
- perf: drop the redundant per-node ray-aabb re-test in the dbvt raycast — the node's stored entry distance already proves the hit
- feat: persistent body pairs — moved-only broadphase discovery (box2d-v3 style): bodies that escape their fat leaf AABB queue for pair discovery, overlapping pairs persist between frames as records with intrusive per-body edge lists, and the per-pair pose cache lives on the record; broadphase-heavy scenes are 10-15% faster
- feat: **breaking** — contacts are now nested under their persistent pair record (chained via `prevInPair`/`nextInPair`, staleness via frame stamps) instead of per-body edge lists; the per-frame global contact sweeps are gone, so step cost scales with active contacts rather than total contacts (a fully-settled world steps ~60% faster; sleeping pairs cost two field compares). The `Contact` type loses `edges`/`processedThisFrame` and gains `pairRecord`/`prevInPair`/`nextInPair`/`lastProcessedFrame`; `RigidBody` loses `headContactKey`/`contactCount` — enumerate a body's contacts via its pair list and each pair's contact chain. `onContactRemoved` for transition removals (sleep, teleport, filter changes) now fires during pair finding, before the step's added/persisted events, rather than after the narrowphase loop
- perf: contact solver setup and position-solve trims — position-solve world inertia is now computed lazily on the first penetrating point (separated speculative contacts skip it), the two full inverse-inertia matrices are no longer stored per constraint (setup scratch only), and the position-solve inertia scale copy is skipped when the scale is 1; bit-identical results, dense stacking ~7% faster
- perf: convex hull support queries accelerated by hill climbing over a vertex adjacency graph baked at shape create (auto for hulls above 32 vertices; `bakeSupportAdjacency` overrides) — warm-started within each narrowphase pair, ~30% less support time in hull-heavy scenes (~16% faster overall)
- perf: `getSupportingFace` no longer normalizes every plane normal for uniformly-scaled convex hulls (one sqrt per face per call removed)
- perf: EPA hull builder restructured to flat `number[]` arrays (triangles/edges/silhouette stack as stride-indexed parallel arrays instead of pooled objects) — bit-identical results, hull-heavy scenes ~10% faster
- fix: issues with nested usage of reversedCollideShapeVsShape
- feat: change build setup so d.ts files are compatible with typescript NodeNext module resolution
- feat: update mathcat from v0.0.11 to v0.0.13
- feat: double-buffered cached contact manifolds — warm-start λ matching now reads the previous step's data instead of aliasing this step's writes, matching Jolt's mReadCache/mWriteCache split
- fix: stack stability — corrected inverted motion-type swap in narrowphase so collision detection runs in the dynamic body's local space, matching Jolt; tall box stacks now settle instead of rocking indefinitely
- feat: body-pair contact cache — skip GJK/EPA when bodies stay within 1mm/1° of last fresh narrowphase pose, reconstructing the manifold from cached body-local data; eliminates penetration-axis jitter and significantly improves stacking stability (matches Jolt's `mUseBodyPairContactCache`)
- fix: debug.* drawing for scaled and plane shapes
- fix: CCD (LINEAR_CAST) contacts now fire the contact listener on the step of impact; previously `onContactAdded`/`onContactPersisted` were never called for continuous collisions, so contacts only surfaced a step later via the discrete narrowphase (and not at all when a body rebounded clear). Gate now matches Jolt's `mFractionPlusSlop < 1`
- feat: **breaking** — convex hull vertices are now stored as flat `number[]` SoA arrays on `ConvexHullShape` (`pointPositions`/`pointNumFaces`/`pointFaces`/`numPoints`) instead of a `ConvexHullPoint[]` object array; read `shape.pointPositions[i * 3 + k]` in place of `shape.points[i].position[k]`, and the `ConvexHullPoint` type is removed.
- perf: convex-radius shrink rewritten as an allocation-free scalar routine (scaled + unscaled, no `plane3` objects).
- feat: **breaking** — convex support is now a single monomorphic `getSupport(out, support, direction)` over one tagged `Support` struct, filled per-pair via `setShapeSupport` (or the per-shape `set*Support` setters). The `ShapeSupportPool`, `getShapeSupportFunction`, `createShapeSupportPool`, `TransformedSupport`, and `AddConvexRadiusSupport` machinery is removed; transform and convex-radius inflation are now folded onto the struct (`hasTransform`/`transform`, `addRadius`).
- feat: start using [compilecat](htps://github.com/isaacmason/compilecat) to opt-in optimize some hot-path functions

## v0.0.4

- fix: convex hull shapes no longer recenter geometry around center of mass. points and planes are now stored in the original shape space, fixing common misalignment problems between physics and visual meshes. COM is still computed and stored for dynamics (inertia, integration).
- fix: KCC characters getting stuck when surrounded by steep slopes due to penetration recovery velocity being included in vertical wall constraints
- fix: incorrect closest-point-on-edge in convex hull builder, increase coplanar tolerance to fix degenerate hulls
- feat: defer DOF masking in contact solver to single write-back per constraint
- feat: reduce convex radius if too big for shape
- feat: added `debugRenderer.dispose` method to `crashcat/three` debug renderer utility
- feat: add `debug.body`, `debug.shape` debug utilities
- feat: remove debug assertions from production builds

## v0.0.3

- feat: minor refactors in dbvt
- fix: `debug.bodies` output for capsule, sphere

## v0.0.2

- feat: add `StaticCompoundShape`, alternative to `CompoundShape` with a BVH for faster queries
- feat: early exit velocity solving for island when converged
- feat: rename `kcc.setRotation()` to `kcc.setQuaternion()`
- feat: add `estimateCollisionResponse()` utility for estimating collision impulses from contact manifolds
- feat: change `castRayVsShape` signature to flatten `ray: Raycast3` to `originX, originY, originZ, directionX, directionY, directionZ, length`
- feat: export `kcc.DEFAULT_KCC_SETTINGS`
- feat: add renderer agnostic `debug` utils for drawing debug lines for bodies, contacts, joints
- feat: update mathcat to v0.0.11, update `Box3` usage, changed from `[min: Vec3, max: Vec3]` to `[minX, minY, minZ, maxX, maxY, maxZ]`

## v0.0.1

- Initial release!
