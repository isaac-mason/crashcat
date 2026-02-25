# CHANGELOG

## v0.0.2 (Unreleased)

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
