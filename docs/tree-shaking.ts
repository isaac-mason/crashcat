/* eslint-disable @typescript-eslint/no-unused-vars */
import { createWorld, createWorldSettings } from 'crashcat';

/* SNIPPET_START: register-all */
import { registerAll } from 'crashcat';

// register all built-in shapes and constraints
// this is simple but includes everything in the bundle
registerAll();
/* SNIPPET_END: register-all */

/* SNIPPET_START: register-selective */
import { registerShapes, registerConstraints } from 'crashcat';
import { sphere, box, capsule } from 'crashcat';
import { hingeConstraint, distanceConstraint } from 'crashcat';

// only register the shapes you need
registerShapes([sphere.def, box.def, capsule.def]);

// only register the constraints you need
registerConstraints([hingeConstraint.def, distanceConstraint.def]);
/* SNIPPET_END: register-selective */
