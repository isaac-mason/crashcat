import { registerConstraintDef, type ConstraintDef } from './constraints/constraints';
import { type ShapeDef, shapeDefs } from './shapes/shapes';

/** register shape definitions */
export function registerShapes(defs: Array<ShapeDef<any>>): void {
    for (const def of defs) {
        shapeDefs[def.type] = def;
    }

    for (const shapeDef of Object.values(shapeDefs)) {
        shapeDef.register();
    }
}

/** register two-body constraint definitions */
export function registerConstraints(defs: Array<ConstraintDef<any>>): void {
    for (const def of defs) {
        registerConstraintDef(def);
    }
}
