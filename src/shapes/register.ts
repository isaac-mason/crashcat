import * as box from './box';
import * as capsule from './capsule';
import * as compound from './compound';
import * as convexHull from './convex-hull';
import * as cylinder from './cylinder';
import * as emptyShape from './empty-shape';
import * as offsetCenterOfMass from './offset-center-of-mass';
import * as plane from './plane';
import * as scaled from './scaled';
import { type ShapeDef, shapeDefs } from './shapes';
import * as sphere from './sphere';
import * as transformed from './transformed';
import * as triangleMesh from './triangle-mesh';
import * as support from '../collision/support';

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

export function registerAllShapes() {
    registerShapes(ALL_SHAPE_DEFS);
}

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
