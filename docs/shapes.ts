import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    compound,
    convexHull,
    createWorld,
    createWorldSettings,
    cylinder,
    enableCollision,
    MotionType,
    massProperties,
    offsetCenterOfMass,
    rigidBody,
    scaled,
    sphere,
    triangleMesh,
} from 'crashcat';
import { quat } from 'mathcat';

const worldSettings = createWorldSettings();
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);
const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
const world = createWorld(worldSettings);

/* SNIPPET_START: sphere */
// simplest and fastest convex shape
sphere.create({ radius: 1 });

// with density for mass calculation
sphere.create({ radius: 1, density: 1000 }); // kg/m³
/* SNIPPET_END: sphere */

/* SNIPPET_START: box */
// defined by half extents from center
box.create({ halfExtents: [1, 2, 0.5] });

// with density
box.create({ halfExtents: [1, 1, 1], density: 500 });
/* SNIPPET_END: box */

/* SNIPPET_START: capsule */
// cylinder with hemispherical caps
capsule.create({
    halfHeightOfCylinder: 1, // half height of cylindrical section
    radius: 0.5,
});

// with density
capsule.create({
    halfHeightOfCylinder: 1,
    radius: 0.5,
    density: 800,
});
/* SNIPPET_END: capsule */

/* SNIPPET_START: cylinder */
// defined by half height and radius
cylinder.create({
    halfHeight: 1,
    radius: 0.5,
});

// with density
cylinder.create({
    halfHeight: 1,
    radius: 0.5,
    density: 1200,
});
/* SNIPPET_END: cylinder */

/* SNIPPET_START: convex-hull */
// convex envelope of a set of points
convexHull.create({
    positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], // flat array [x,y,z, x,y,z, ...]
});

// with density
convexHull.create({
    positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
    density: 900,
});
/* SNIPPET_END: convex-hull */

/* SNIPPET_START: triangle-mesh */
// triangle mesh: for complex static geometry like terrain
const meshShape = triangleMesh.create({
    positions: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10],
    indices: [0, 1, 2, 0, 2, 3],
});

// triangle meshes typically used with static bodies
rigidBody.create(world, {
    shape: meshShape,
    motionType: MotionType.STATIC,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
});

// for dynamic triangle meshes, provide mass properties explicitly
const dynamicMeshProps = massProperties.create();
massProperties.setMassAndInertiaOfSolidBox(dynamicMeshProps, [2, 2, 2], 1000);

rigidBody.create(world, {
    shape: meshShape,
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    massPropertiesOverride: dynamicMeshProps,
});
/* SNIPPET_END: triangle-mesh */

/* SNIPPET_START: compound */
// compound: combine multiple shapes into one
compound.create({
    children: [
        {
            position: [0, 0, 0],
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [2, 0.5, 1] }), // main body
        },
        {
            position: [0, 1, 0],
            quaternion: quat.create(),
            shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }), // turret
        },
    ],
});

// useful for complex objects like vehicles, characters, furniture
rigidBody.create(world, {
    shape: compound.create({
        children: [
            {
                position: [0, 0, 0],
                quaternion: quat.create(),
                shape: box.create({ halfExtents: [1, 1, 1] }),
            },
        ],
    }),
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});
/* SNIPPET_END: compound */

/* SNIPPET_START: scaled */
// non-uniform scaling of any shape
scaled.create({
    shape: box.create({ halfExtents: [1, 1, 1] }),
    scale: [2, 0.5, 1], // stretch in x, squash in y
});

// works with any shape
scaled.create({
    shape: sphere.create({ radius: 1 }),
    scale: [1, 2, 1], // creates an ellipsoid
});
/* SNIPPET_END: scaled */

/* SNIPPET_START: offset-center-of-mass */
// shift center of mass without changing collision geometry
// useful for stability (e.g., lowering COM on tall objects)
const stableShape = offsetCenterOfMass.create({
    shape: box.create({ halfExtents: [0.5, 2, 0.5] }), // tall box
    offset: [0, -1, 0], // lower center of mass
});

rigidBody.create(world, {
    shape: stableShape,
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
});
/* SNIPPET_END: offset-center-of-mass */

/* SNIPPET_START: reuse */
// create a shape once
const sharedBoxShape = box.create({ halfExtents: [1, 1, 1] });

// reuse it for multiple bodies
rigidBody.create(world, {
    shape: sharedBoxShape,
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    position: [0, 5, 0],
});

rigidBody.create(world, {
    shape: sharedBoxShape, // same shape instance
    motionType: MotionType.DYNAMIC,
    objectLayer: OBJECT_LAYER_MOVING,
    position: [5, 5, 0],
});
/* SNIPPET_END: reuse */

/* SNIPPET_START: offline */
// offline (node.js, build step, bundler macro, etc)
const pregeneratedTriangleMeshShape = triangleMesh.create({
    positions: [/* large terrain data */],
    indices: [/* large index data */],
});

// serialize to JSON
const triangleMeshShapeJson = JSON.stringify(pregeneratedTriangleMeshShape);
// save to file or bundle with app

// runtime (browser):
const triangleMeshShape = JSON.parse(triangleMeshShapeJson);

// use the pre-processed shape directly
rigidBody.create(world, {
    shape: triangleMeshShape,
    motionType: MotionType.STATIC,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
});
/* SNIPPET_END: offline */
