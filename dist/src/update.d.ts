import { type Listener } from './listener.js';
import type { World } from './world.js';
/**
 * Updates the physics world with a given time step
 * @param world the physics world to update
 * @param listener optional contact listener for collision events
 * @param timeStep the time step to advance the world by, in seconds
 */
export declare function updateWorld(world: World, listener: Listener | undefined, timeStep: number): void;
