import { registerUserConstraintDef, type UserConstraintDef } from './constraints/constraints';
import * as support from './collision/support';
import { type ShapeDef, shapeDefs } from './shapes/shapes';

/** register shape definitions */
export function registerShapes(defs: Array<ShapeDef<any>>): void {
    for (const def of defs) {
        // add to shape defs
        shapeDefs[def.type] = def;
        // populate support pools
        support.allocateShapeSupportPools(def);
    }

    for (const shapeDef of Object.values(shapeDefs)) {
        shapeDef.register();
    }
}

/** register two-body constraint definitions */
export function registerConstraints(defs: Array<UserConstraintDef<any>>): void {
    for (const def of defs) {
        registerUserConstraintDef(def);
    }
}
