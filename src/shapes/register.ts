import { type ShapeDef, shapeDefs } from './shapes';
import * as support from '../collision/support';

export function registerShapes(defs: Array<ShapeDef<any>>) {
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
