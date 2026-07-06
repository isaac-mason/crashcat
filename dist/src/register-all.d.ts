import * as coneConstraint from './constraints/cone-constraint.js';
import * as distanceConstraint from './constraints/distance-constraint.js';
import * as fixedConstraint from './constraints/fixed-constraint.js';
import * as hingeConstraint from './constraints/hinge-constraint.js';
import * as sixDOFConstraint from './constraints/six-dof-constraint.js';
import * as sliderConstraint from './constraints/slider-constraint.js';
import * as swingTwistConstraint from './constraints/swing-twist-constraint.js';
import * as box from './shapes/box.js';
import * as capsule from './shapes/capsule.js';
import * as compound from './shapes/compound.js';
import * as convexHull from './shapes/convex-hull.js';
import * as cylinder from './shapes/cylinder.js';
import * as emptyShape from './shapes/empty-shape.js';
import * as offsetCenterOfMass from './shapes/offset-center-of-mass.js';
import * as plane from './shapes/plane.js';
import * as scaled from './shapes/scaled.js';
import * as sphere from './shapes/sphere.js';
import * as staticCompound from './shapes/static-compound.js';
import * as transformed from './shapes/transformed.js';
import * as triangleMesh from './shapes/triangle-mesh.js';
export declare const ALL_SHAPE_DEFS: (import(".").ShapeDef<box.BoxShape> | import(".").ShapeDef<capsule.CapsuleShape> | import(".").ShapeDef<convexHull.ConvexHullShape> | import(".").ShapeDef<cylinder.CylinderShape> | import(".").ShapeDef<sphere.SphereShape> | import(".").ShapeDef<compound.CompoundShape> | import(".").ShapeDef<emptyShape.EmptyShape> | import(".").ShapeDef<offsetCenterOfMass.OffsetCenterOfMassShape> | import(".").ShapeDef<plane.PlaneShape> | import(".").ShapeDef<scaled.ScaledShape> | import(".").ShapeDef<staticCompound.StaticCompoundShape> | import(".").ShapeDef<transformed.TransformedShape> | import(".").ShapeDef<triangleMesh.TriangleMeshShape>)[];
export declare const ALL_CONSTRAINT_DEFS: (import("./constraints/constraints").ConstraintDef<coneConstraint.ConeConstraint> | import("./constraints/constraints").ConstraintDef<distanceConstraint.DistanceConstraint> | import("./constraints/constraints").ConstraintDef<fixedConstraint.FixedConstraint> | import("./constraints/constraints").ConstraintDef<hingeConstraint.HingeConstraint> | import("./constraints/constraints").ConstraintDef<sixDOFConstraint.SixDOFConstraint> | import("./constraints/constraints").ConstraintDef<sliderConstraint.SliderConstraint> | import("./constraints/constraints").ConstraintDef<swingTwistConstraint.SwingTwistConstraint>)[];
/** register all built-in shapes */
export declare function registerAllShapes(): void;
/** register all built-in constraints */
export declare function registerAllConstraints(): void;
/** register all built-in shapes and constraints */
export declare function registerAll(): void;
