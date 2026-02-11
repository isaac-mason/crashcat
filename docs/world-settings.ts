import { addBroadphaseLayer, addObjectLayer, createWorld, createWorldSettings, enableCollision, registerAll } from 'crashcat';

// register all built-in shapes and constraints
registerAll();

// create world settings container
const worldSettings = createWorldSettings();

// add broadphase layers
// each broadphase layer has its own spatial acceleration structure
// common approach: separate "moving" and "not moving" layers
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

// add object layers
// object layers belong to a broadphase layer and let you define collision rules
const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

// enable collisions between object layers
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

// create the world from settings
const world = createWorld(worldSettings);
