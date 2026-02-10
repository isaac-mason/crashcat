import * as coneConstraint from './constraints/cone-constraint';
import * as distanceConstraint from './constraints/distance-constraint';
import * as fixedConstraint from './constraints/fixed-constraint';
import * as hingeConstraint from './constraints/hinge-constraint';
import * as pointConstraint from './constraints/point-constraint';
import * as sixDOFConstraint from './constraints/six-dof-constraint';
import * as sliderConstraint from './constraints/slider-constraint';
import * as swingTwistConstraint from './constraints/swing-twist-constraint';
import { registerShapes, registerConstraints } from './register';
import * as box from './shapes/box';
import * as capsule from './shapes/capsule';
import * as compound from './shapes/compound';
import * as convexHull from './shapes/convex-hull';
import * as cylinder from './shapes/cylinder';
import * as emptyShape from './shapes/empty-shape';
import * as offsetCenterOfMass from './shapes/offset-center-of-mass';
import * as plane from './shapes/plane';
import * as scaled from './shapes/scaled';
import * as sphere from './shapes/sphere';
import * as transformed from './shapes/transformed';
import * as triangleMesh from './shapes/triangle-mesh';

export const ALL_SHAPE_DEFS = [
    sphere.def,
    box.def,
    capsule.def,
    convexHull.def,
    cylinder.def,
    plane.def,
    triangleMesh.def,
    compound.def,
    transformed.def,
    scaled.def,
    emptyShape.def,
    offsetCenterOfMass.def,
];

export const ALL_CONSTRAINT_DEFS = [
    pointConstraint.def,
    distanceConstraint.def,
    hingeConstraint.def,
    sliderConstraint.def,
    fixedConstraint.def,
    coneConstraint.def,
    swingTwistConstraint.def,
    sixDOFConstraint.def,
];

/** register all built-in shapes */
export function registerAllShapes(): void {
    registerShapes(ALL_SHAPE_DEFS);
}

/** register all built-in constraints */
export function registerAllConstraints(): void {
    registerConstraints(ALL_CONSTRAINT_DEFS);
}

/** register all built-in shapes and constraints */
export function registerAll(): void {
    registerAllShapes();
    registerAllConstraints();
}
