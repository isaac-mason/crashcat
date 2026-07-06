import { type Vec3 } from 'mathcat';
import type { World } from '../world.js';
import type { MotionProperties } from './motion-properties.js';
import type { RigidBody } from './rigid-body.js';
/** sentinel value indicating a body is not in the active bodies list (sleeping or static) */
export declare const INACTIVE_BODY_INDEX: number;
/**
 * get the 3 test points for sleep detection:
 * - center of mass
 * - center of mass + largest bounding box axis
 * - center of mass + second largest bounding box axis
 *
 * @optimize
 */
export declare function getSleepTestPoints(body: RigidBody, outPoints: [Vec3, Vec3, Vec3]): void;
/** reset the sleep test spheres to center around the given points with radius 0 */
export declare function resetSleepTestSpheres(mp: MotionProperties, points: [Vec3, Vec3, Vec3]): void;
/** update the sleep state of a body, returns true if the body can sleep, false if it cannot */
export declare function updateSleepState(body: RigidBody, deltaTime: number, maxMovement: number, timeBeforeSleep: number): boolean;
/** reset the sleep timer for a body (called when body is activated or velocity is set) */
export declare function resetSleepTimer(body: RigidBody): void;
/** adds a body to the active bodies list, alled when a body wakes up or is created as non-sleeping */
export declare function addBodyToActiveBodies(world: World, body: RigidBody): void;
/** removes a body from the active bodies list using swap-remove, called when a body goes to sleep or is destroyed */
export declare function removeBodyFromActiveBodies(world: World, body: RigidBody): void;
/** puts a body to sleep, sleeping bodies are excluded from physics simulation until woken */
export declare function sleep(world: World, body: RigidBody): void;
/** wakes a sleeping body and all connected bodies (via contacts and constraints) */
export declare function wake(world: World, body: RigidBody): void;
