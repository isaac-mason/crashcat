import { registerShapes, sphere, box } from '../../dist/index.js';

registerShapes([sphere.def, box.def]);
const s = sphere.create({ radius: 1 });
const b = box.create({ halfExtents: [1, 1, 1] });
console.log(s, b);
