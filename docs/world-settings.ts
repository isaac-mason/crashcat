import { addBroadphaseLayer, addObjectLayer, createWorld, createWorldSettings, enableCollision, registerAll } from 'crashcat';

// we can use registerShapes and registerConstraints to granularly declare
// which shapes and constraints we want to use for the best tree shaking.
// but early in development, it's easier to just register everything.
registerAll();

// this is a container for all settings related to world simulation.
// in a real project, you'd put this in a e.g. "physics-world-settings.ts" file seperate from the physics world
// creation, and import it and below constants where needed.
const worldSettings = createWorldSettings();

// earth gravity
worldSettings.gravity = [0, -9.81, 0];

// we're first up going to define "broadphase layers".
// for simple projects, a "moving" and "not moving" broadphase layer is a good start.
export const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
export const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

// next, we'll define some "object layers".
export const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
export const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

// here we declare that "moving" objects should collide with "not moving" objects, and with other "moving" objects.
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);

// time to create the physics world from the settings we've defined
const world = createWorld(worldSettings);
