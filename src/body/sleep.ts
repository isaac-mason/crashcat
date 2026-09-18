import { type Vec3, vec3 } from 'math';
import type { World } from '../world';
import type { MotionProperties } from './motion-properties';
import { MotionType } from './motion-type';
import type { RigidBody } from './rigid-body';

/** sentinel value indicating a body is not in the active bodies list (sleeping or static) */
export const INACTIVE_BODY_INDEX = Number.MAX_SAFE_INTEGER;


/**
 * get the 3 test points for sleep detection:
 * - center of mass
 * - center of mass + largest bounding box axis
 * - center of mass + second largest bounding box axis
 */
export function getSleepTestPoints(body: RigidBody, outPoints: [Vec3, Vec3, Vec3]): void {
    const com = body.centerOfMassPosition;

    // center of mass is the first position
    vec3.copy(outPoints[0], com);

    // half-sizes of shape AABB
    const aabb = body.shape.aabb;
    const ex = (aabb[3] - aabb[0]) * 0.5;
    const ey = (aabb[4] - aabb[1]) * 0.5;
    const ez = (aabb[5] - aabb[2]) * 0.5;

    // the two largest extents pick the two rotated axes to test; a rotation matrix column comes
    // straight from the quaternion, so only those two are built
    const qx = body.quaternion[0];
    const qy = body.quaternion[1];
    const qz = body.quaternion[2];
    const qw = body.quaternion[3];
    const p1 = outPoints[1];
    const p2 = outPoints[2];
    if (ex <= ey && ex <= ez) {
        // x is smallest: y and z axes
        p1[0] = com[0] + 2 * (qx * qy - qw * qz) * ey;
        p1[1] = com[1] + (1 - 2 * (qx * qx + qz * qz)) * ey;
        p1[2] = com[2] + 2 * (qy * qz + qw * qx) * ey;
        p2[0] = com[0] + 2 * (qx * qz + qw * qy) * ez;
        p2[1] = com[1] + 2 * (qy * qz - qw * qx) * ez;
        p2[2] = com[2] + (1 - 2 * (qx * qx + qy * qy)) * ez;
    } else if (ey <= ez) {
        // y is smallest: x and z axes
        p1[0] = com[0] + (1 - 2 * (qy * qy + qz * qz)) * ex;
        p1[1] = com[1] + 2 * (qx * qy + qw * qz) * ex;
        p1[2] = com[2] + 2 * (qx * qz - qw * qy) * ex;
        p2[0] = com[0] + 2 * (qx * qz + qw * qy) * ez;
        p2[1] = com[1] + 2 * (qy * qz - qw * qx) * ez;
        p2[2] = com[2] + (1 - 2 * (qx * qx + qy * qy)) * ez;
    } else {
        // z is smallest: x and y axes
        p1[0] = com[0] + (1 - 2 * (qy * qy + qz * qz)) * ex;
        p1[1] = com[1] + 2 * (qx * qy + qw * qz) * ex;
        p1[2] = com[2] + 2 * (qx * qz - qw * qy) * ex;
        p2[0] = com[0] + 2 * (qx * qy - qw * qz) * ey;
        p2[1] = com[1] + (1 - 2 * (qx * qx + qz * qz)) * ey;
        p2[2] = com[2] + 2 * (qy * qz + qw * qx) * ey;
    }
}

/** reset the sleep test spheres to center around the given points with radius 0 */
export function resetSleepTestSpheres(mp: MotionProperties, points: [Vec3, Vec3, Vec3]): void {
    for (let i = 0; i < 3; i++) {
        vec3.copy(mp.sleepTestSpheres[i].center, points[i]);
        mp.sleepTestSpheres[i].radius = 0;
    }
    mp.sleepTestTimer = 0;
}

const _updateSleepState_points: [Vec3, Vec3, Vec3] = [vec3.create(), vec3.create(), vec3.create()];

/** update the sleep state of a body, returns true if the body can sleep, false if it cannot */
export function updateSleepState(body: RigidBody, deltaTime: number, maxMovement: number, timeBeforeSleep: number): boolean {
    const mp = body.motionProperties;

    // sensors and bodies with allowSleeping=false never sleep
    if (!mp.allowSleeping || body.sensor) {
        return false; // cannot sleep
    }

    // get current test points
    getSleepTestPoints(body, _updateSleepState_points);

    // check if any sphere exceeds max movement
    for (let i = 0; i < 3; i++) {
        const sphere = mp.sleepTestSpheres[i];

        // grow sphere to encapsulate the current point
        const distanceToPoint = vec3.distance(sphere.center, _updateSleepState_points[i]);
        sphere.radius = Math.max(sphere.radius, distanceToPoint);

        // if exceeded threshold, reset and return cannot sleep
        if (sphere.radius > maxMovement) {
            resetSleepTestSpheres(mp, _updateSleepState_points);
            return false;
        }
    }

    // accumulate sleep time
    mp.sleepTestTimer += deltaTime;
    return mp.sleepTestTimer >= timeBeforeSleep; // can sleep
}

const _resetSleepTimer_points: [Vec3, Vec3, Vec3] = [vec3.create(), vec3.create(), vec3.create()];

/** reset the sleep timer for a body (called when body is activated or velocity is set) */
export function resetSleepTimer(body: RigidBody): void {
    if (body.motionType !== MotionType.DYNAMIC) {
        return;
    }

    getSleepTestPoints(body, _resetSleepTimer_points);
    resetSleepTestSpheres(body.motionProperties, _resetSleepTimer_points);
}

/** adds a body to the active bodies list, alled when a body wakes up or is created as non-sleeping */
export function addBodyToActiveBodies(world: World, body: RigidBody): void {
    const bodies = world.bodies;

    // body already active
    if (body.activeIndex !== INACTIVE_BODY_INDEX) return;

    // assign new index at end of array
    body.activeIndex = bodies.activeBodyCount;
    bodies.activeBodyIndices[bodies.activeBodyCount] = body.index;
    bodies.activeBodyCount++;
}

/** removes a body from the active bodies list using swap-remove, called when a body goes to sleep or is destroyed */
export function removeBodyFromActiveBodies(world: World, body: RigidBody): void {
    const bodies = world.bodies;

    // body already inactive
    if (body.activeIndex === INACTIVE_BODY_INDEX) return;

    const lastIndex = bodies.activeBodyCount - 1;

    if (body.activeIndex !== lastIndex) {
        // swap with last body to fill the hole
        const lastBodyIndex = bodies.activeBodyIndices[lastIndex];
        bodies.activeBodyIndices[body.activeIndex] = lastBodyIndex;

        // update swapped body's activeIndex
        const lastBody = bodies.pool[lastBodyIndex];
        lastBody.activeIndex = body.activeIndex;
    }

    // mark as inactive. islands are only built over active bodies, so the island index goes stale
    // here; clearing it on exit keeps islands.prepare from resetting the whole body pool every step
    body.activeIndex = INACTIVE_BODY_INDEX;
    body.islandIndex = -1;
    bodies.activeBodyCount--;
}

/** puts a body to sleep, sleeping bodies are excluded from physics simulation until woken */
export function sleep(world: World, body: RigidBody): void {
    // exit if body is static
    if (body.motionType === MotionType.STATIC) return;

    // exit if already sleeping
    if (body.sleeping) return;

    // remove from active list
    removeBodyFromActiveBodies(world, body);

    // mark as sleeping
    body.sleeping = true;

    // reset velocities
    vec3.zero(body.motionProperties.linearVelocity);
    vec3.zero(body.motionProperties.angularVelocity);
}

/** wakes a sleeping body and all connected bodies (via contacts and constraints) */
export function wake(world: World, body: RigidBody): void {
    // exit if body is static
    if (body.motionType === MotionType.STATIC) return;

    // always reset sleep timer
    resetSleepTimer(body);

    // exit if already awake
    if (!body.sleeping) return;

    // mark as awake
    body.sleeping = false;

    // add to active list
    addBodyToActiveBodies(world, body);
}
