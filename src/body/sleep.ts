import { box3, type Vec3, vec3 } from 'mathcat';
import type { World } from '../world';
import type { MotionProperties } from './motion-properties';
import { MotionType } from './motion-type';
import type { RigidBody } from './rigid-body';

/** sentinel value indicating a body is not in the active bodies list (sleeping or static) */
export const INACTIVE_BODY_INDEX = Number.MAX_SAFE_INTEGER;

const _getSleepTestPoints_extent = vec3.create();
const _getSleepTestPoints_tempVec = vec3.create();
const _getSleepTestPoints_xAxis = vec3.fromValues(1, 0, 0);
const _getSleepTestPoints_yAxis = vec3.fromValues(0, 1, 0);
const _getSleepTestPoints_zAxis = vec3.fromValues(0, 0, 1);

/**
 * get the 3 test points for sleep detection:
 * - center of mass
 * - center of mass + largest bounding box axis
 * - center of mass + second largest bounding box axis
 */
export function getSleepTestPoints(body: RigidBody, outPoints: [Vec3, Vec3, Vec3]): void {
    // center of mass is the first position
    vec3.copy(outPoints[0], body.centerOfMassPosition);

    // get bounding box extent (half-sizes)
    box3.extents(_getSleepTestPoints_extent, body.shape.aabb);

    // find smallest axis (we'll use the other two)
    const x = _getSleepTestPoints_extent[0];
    const y = _getSleepTestPoints_extent[1];
    const z = _getSleepTestPoints_extent[2];

    let lowestComponent: number;
    if (x <= y && x <= z) {
        lowestComponent = 0; // X is smallest
    } else if (y <= z) {
        lowestComponent = 1; // Y is smallest
    } else {
        lowestComponent = 2; // Z is smallest
    }

    switch (lowestComponent) {
        case 0: // X is smallest, use Y and Z axes
            // outPoints[1] = COM + rotated(Y-axis) * extent.y
            vec3.transformQuat(_getSleepTestPoints_tempVec, _getSleepTestPoints_yAxis, body.quaternion);
            vec3.scaleAndAdd(outPoints[1], body.centerOfMassPosition, _getSleepTestPoints_tempVec, y);

            // outPoints[2] = COM + rotated(Z-axis) * extent.z
            vec3.transformQuat(_getSleepTestPoints_tempVec, _getSleepTestPoints_zAxis, body.quaternion);
            vec3.scaleAndAdd(outPoints[2], body.centerOfMassPosition, _getSleepTestPoints_tempVec, z);
            break;

        case 1: // Y is smallest, use X and Z axes
            // outPoints[1] = COM + rotated(X-axis) * extent.x
            vec3.transformQuat(_getSleepTestPoints_tempVec, _getSleepTestPoints_xAxis, body.quaternion);
            vec3.scaleAndAdd(outPoints[1], body.centerOfMassPosition, _getSleepTestPoints_tempVec, x);

            // outPoints[2] = COM + rotated(Z-axis) * extent.z
            vec3.transformQuat(_getSleepTestPoints_tempVec, _getSleepTestPoints_zAxis, body.quaternion);
            vec3.scaleAndAdd(outPoints[2], body.centerOfMassPosition, _getSleepTestPoints_tempVec, z);
            break;

        case 2: // Z is smallest, use X and Y axes
            // outPoints[1] = COM + rotated(X-axis) * extent.x
            vec3.transformQuat(_getSleepTestPoints_tempVec, _getSleepTestPoints_xAxis, body.quaternion);
            vec3.scaleAndAdd(outPoints[1], body.centerOfMassPosition, _getSleepTestPoints_tempVec, x);

            // outPoints[2] = COM + rotated(Y-axis) * extent.y
            vec3.transformQuat(_getSleepTestPoints_tempVec, _getSleepTestPoints_yAxis, body.quaternion);
            vec3.scaleAndAdd(outPoints[2], body.centerOfMassPosition, _getSleepTestPoints_tempVec, y);
            break;
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

    // mark as inactive
    body.activeIndex = INACTIVE_BODY_INDEX;
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
