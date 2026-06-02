import { type ConstraintDef } from './constraints/constraints.js';
import { type ShapeDef } from './shapes/shapes.js';
/** register shape definitions */
export declare function registerShapes(defs: Array<ShapeDef<any>>): void;
/** register two-body constraint definitions */
export declare function registerConstraints(defs: Array<ConstraintDef<any>>): void;
