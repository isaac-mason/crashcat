/** @module crashcat */

export * as bitmask from './bitmask';
export type { BodyId } from './body/body-id';
export * from './body/dof';
export type { MassProperties } from './body/mass-properties';
export * as massProperties from './body/mass-properties';
export type { MotionProperties } from './body/motion-properties';
export * as motionProperties from './body/motion-properties';
export { MotionQuality } from './body/motion-properties';
export * from './body/motion-type';
export type { RigidBody, RigidBodySettings } from './body/rigid-body';
export * as rigidBody from './body/rigid-body';
export { INACTIVE_BODY_INDEX } from './body/sleep';
export type { SubShapeId } from './body/sub-shape';
export * as subShape from './body/sub-shape';
export { EMPTY_SUB_SHAPE_ID } from './body/sub-shape';
export type { BodyVisitor } from './broadphase/body-visitor';
export * as broadphase from './broadphase/broadphase';
export * as dbvt from './broadphase/dbvt';
export type { KCC, KCCSettings } from './character/kcc';
export * as kcc from './character/kcc';
export * as activeEdges from './collision/active-edges';
export * from './collision/cast-ray-vs-shape';
export * from './collision/cast-shape-vs-shape';
export * from './collision/collide-point-vs-shape';
export * from './collision/collide-shape-vs-shape';
export type { CollisionEstimationResult } from './collision/estimate-collision-response';
export { createCollisionEstimationResult, estimateCollisionResponse } from './collision/estimate-collision-response';
export * from './collision/gjk';
export * from './collision/narrowphase';
export * from './collision/penetration';
export * from './collision/simplex';
export * from './collision/support';
export * from './constraints/combine-material';
export type { ConeConstraint, ConeConstraintSettings } from './constraints/cone-constraint';
export * as coneConstraint from './constraints/cone-constraint';
export { ConstraintType } from './constraints/constraint-id';
export * as motorSettings from './constraints/constraint-part/motor-settings';
export { type MotorSettings, MotorState } from './constraints/constraint-part/motor-settings';
export * as springSettings from './constraints/constraint-part/spring-settings';
export { SpringMode, type SpringSettings } from './constraints/constraint-part/spring-settings';
export { SwingType } from './constraints/constraint-part/swing-twist-constraint-part';
export type {
    Constraint,
    ConstraintIterationOverrides,
    ConstraintSortFields,
    ConstraintTypeRegistry,
} from './constraints/constraints';
export * as constraints from './constraints/constraints';
export { ConstraintSpace } from './constraints/constraints';
export type {
    ContactConstraint,
    ContactConstraints,
    ContactSettings,
    WorldContactPoint,
} from './constraints/contact-constraints';
export type { DistanceConstraint, DistanceConstraintSettings } from './constraints/distance-constraint';
export * as distanceConstraint from './constraints/distance-constraint';
export type { FixedConstraint, FixedConstraintSettings } from './constraints/fixed-constraint';
export * as fixedConstraint from './constraints/fixed-constraint';
export type { HingeConstraint, HingeConstraintSettings } from './constraints/hinge-constraint';
export * as hingeConstraint from './constraints/hinge-constraint';
export type { PointConstraint, PointConstraintSettings } from './constraints/point-constraint';
export * as pointConstraint from './constraints/point-constraint';
export type { SixDOFConstraint, SixDOFConstraintSettings } from './constraints/six-dof-constraint';
export * as sixDOFConstraint from './constraints/six-dof-constraint';
export { SixDOFAxis } from './constraints/six-dof-constraint';
export type { SliderConstraint, SliderConstraintSettings } from './constraints/slider-constraint';
export * as sliderConstraint from './constraints/slider-constraint';
export type { SwingTwistConstraint, SwingTwistConstraintSettings } from './constraints/swing-twist-constraint';
export * as swingTwistConstraint from './constraints/swing-twist-constraint';
export type { Contact } from './contacts';
export * as contacts from './contacts';
export * as debug from './debug';
export type { Filter } from './filter';
export * as filter from './filter';
export * as layers from './layers';
export * from './listener';

export type { ContactManifold } from './manifold';
export { getWorldSpaceContactPointOnA, getWorldSpaceContactPointOnB } from './manifold/manifold';
export type { Pairs } from './pairs';
export * as pairs from './pairs';
export * from './query';
export * from './register';
export * from './register-all';
export type { BoxShape, BoxShapeSettings } from './shapes/box';
export * as box from './shapes/box';
export type { CapsuleShape, CapsuleShapeSettings } from './shapes/capsule';
export * as capsule from './shapes/capsule';
export type { CompoundShape, CompoundShapeSettings } from './shapes/compound';
export * as compound from './shapes/compound';
export * from './shapes/convex';
export type { ConvexHullShape, ConvexHullShapeSettings } from './shapes/convex-hull';
export * as convexHull from './shapes/convex-hull';
export type { CylinderShape, CylinderShapeSettings } from './shapes/cylinder';
export * as cylinder from './shapes/cylinder';
export type { EmptyShape } from './shapes/empty-shape';
export * as emptyShape from './shapes/empty-shape';
export type { OffsetCenterOfMassShape, OffsetCenterOfMassShapeSettings } from './shapes/offset-center-of-mass';
export * as offsetCenterOfMass from './shapes/offset-center-of-mass';
export type { PlaneShape, PlaneShapeSettings } from './shapes/plane';
export * as plane from './shapes/plane';
export type { ScaledShape, ScaledShapeSettings } from './shapes/scaled';
export * as scaled from './shapes/scaled';
export * from './shapes/shapes';
export type { SphereShape, SphereShapeSettings } from './shapes/sphere';
export * as sphere from './shapes/sphere';
export type { StaticCompoundShape, StaticCompoundShapeSettings } from './shapes/static-compound';
export * as staticCompound from './shapes/static-compound';
export type { TransformedShape, TransformedShapeSettings } from './shapes/transformed';
export * as transformed from './shapes/transformed';
export type { TriangleMeshShape, TriangleMeshShapeSettings } from './shapes/triangle-mesh';
export * as triangleMesh from './shapes/triangle-mesh';
export * as bvh from './shapes/utils/bvh';
export * as convexHullBuilder from './shapes/utils/convex-hull-builder';
export * as staticCompoundBvh from './shapes/utils/static-compound-bvh';
export * as triangleMeshBuilder from './shapes/utils/triangle-mesh-builder';
export * as triangleMeshBvh from './shapes/utils/triangle-mesh-bvh';
export * from './update';
export type { Face } from './utils/face';
export * from './utils/face';
export * from './world';
export * from './world-settings';
