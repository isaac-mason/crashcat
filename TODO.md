# TODO

## DONE

- kinematic character controller v0
    - [x] IgnoreSingleBodyFilterChained simplification - can use query.ts
    - [x] max seperation fixes
    - [x] stair stepping, stick to floor validation
    - [x] dynamic body impulses
    - [x] conveyor belt velocity
    - [x] ground velocity - moving platform

- "custom shapes" refactoring
    - [x] getLeafShape etc. become part of ShapeDef
    - [x] castRay, collidePoint -> these become part of the ShapeDef
    - [x] decoupled shapedef support functions
    - [x] then: collideShapeVsShapeFns, castShapeVsShapeFns -> shapes register themselves into these dispatch tables
    - [x] challenge: how to handle "iterate all convex shapes", "iterate all shapes" when shapes are registering themselves? e.g. offset center of mass shape registers against all other shapes with a re-dispatching function.
    - [x] collideConvexVsConvexLocal
    - [x] simple voxels example

- cylinder shape

- offset center of mass shape

- plane shape

- shape material id
    - for user-defined handling, e.g. contact sounds, effects, whatever.
    - just add materialId, user can decide what that means.
    - (already partially done) triangle mesh per-triangle material ids

- util like BodyInterface::ActivateBodiesInAABox

- InternalEdgeRemovingCollector

- charactervirtual fix https://github.com/jrouwe/JoltPhysics/commit/83eea75749660afb8c654541274079ff2bbb11b3

- addforce related fix? https://github.com/jrouwe/JoltPhysics/pull/1890/changes

- debug renderer improvements
    - better wireframe rendering
    - better api for enabling/disabling default options in specific examples

## WIP

- kcc standing on small mass sphere

- user constraints verification & cleanup
    - several implemented, needs more verification, needs examples testing motors
    - generally needs a cleanup pass. particularly constraint parts code.
    - [x] ragdoll
    - [x] sleeping - wasn't skipping user constraints with both bodies sleeping
    - [ ] swing twist stability
    - [ ] constraints motors example a bit wonky?
    - [ ] slider constraint behaving differently to JoltPhysics.js constraints example?

- sphere vs triangle mesh verification

## TODO

- kcc unit tests

- broadphase
    - optimization pass & validation

- EPA trianglesQueue & general optimization pass 

- vehicle example - not in core lib to start

- add staticCompoundShape that creates a bvh

- check shape tree shaking is working properly

- dcc / DynamicCharacterController API?

- kcc character vs character (without inner rigid body) collision

- estimate collision response api
