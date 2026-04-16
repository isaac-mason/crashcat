# CHANGELOG

## v0.0.4 (Unreleased)

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
