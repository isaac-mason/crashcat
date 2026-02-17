# CHANGELOG

## v0.0.2 (Unreleased)

- feat: add `StaticCompoundShape`, alternative to `CompoundShape` with a BVH for faster queries
- feat: early exit velocity solving for island when converged
- feat: rename `kcc.setRotation()` to `kcc.setQuaternion()`
- feat: add `estimateCollisionResponse()` utility for estimating collision impulses from contact manifolds
- feat: change castRayVsShape signature to flatten `ray: Raycast3` to `originX, originY, originZ, directionX, directionY, directionZ, length`

## v0.0.1

- Initial release!
