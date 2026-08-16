import { type Mat3, mat3, type Vec3, vec3 } from 'math';
import { box3 } from 'math/shapes';
import type { World } from '../world';
import type { MotionProperties } from './motion-properties';
import { MotionType } from './motion-type';
import type { RigidBody } from './rigid-body';

/** sentinel value indicating a body is not in the active bodies list (sleeping or static) */
export const INACTIVE_BODY_INDEX = Number.MAX_SAFE_INTEGER;

const _extents: Vec3 = /* @__PURE__ */ vec3.create();
const _rot: Mat3 = /* @__PURE__ */ mat3.create();
const _axis: Vec3 = /* @__PURE__ */ vec3.create();

/**
 * get the 3 test points for sleep detection:
 * - center of mass
 * - center of mass + largest bounding box axis
 * - center of mass + second largest bounding box axis
 *
 * @optimize
 */
export function getSleepTestPoints(body: RigidBody, outPoints: [Vec3, Vec3, Vec3]): void {
    const com = body.centerOfMassPosition;

    // center of mass is the first position
    vec3.copy(outPoints[0], com);

    // half-sizes of shape AABB
    box3.extents(_extents, body.shape.aabb);
    const ex = _extents[0];
    const ey = _extents[1];
    const ez = _extents[2];

    // rotation matrix from body quaternion (column-major)
    // col0 = rotated X axis [0,1,2], col1 = rotated Y axis [3,4,5], col2 = rotated Z axis [6,7,8]
    mat3.fromQuat(_rot, body.quaternion);

    // find smallest extent axis — use the two largest for test points
    // pick (col, scale) pairs for point 1 and point 2
    let c1: number; // column offset for axis 1
    let s1: number; // scale for axis 1
    let c2: number; // column offset for axis 2
    let s2: number; // scale for axis 2
    if (ex <= ey && ex <= ez) {
        // X is smallest: use Y and Z axes
        c1 = 3;
        s1 = ey;
        c2 = 6;
        s2 = ez;
    } else if (ey <= ez) {
        // Y is smallest: use X and Z axes
        c1 = 0;
        s1 = ex;
        c2 = 6;
        s2 = ez;
    } else {
        // Z is smallest: use X and Y axes
        c1 = 0;
        s1 = ex;
        c2 = 3;
        s2 = ey;
    }

    // point 1 = com + axis1 * scale1
    _axis[0] = _rot[c1];
    _axis[1] = _rot[c1 + 1];
    _axis[2] = _rot[c1 + 2];
    vec3.scaleAndAdd(outPoints[1], com, _axis, s1);

    // point 2 = com + axis2 * scale2
    _axis[0] = _rot[c2];
    _axis[1] = _rot[c2 + 1];
    _axis[2] = _rot[c2 + 2];
    vec3.scaleAndAdd(outPoints[2], com, _axis, s2);
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
