import type { Bodies } from './body/bodies';
import { MotionType } from './body/motion-type';
import type { RigidBody } from './body/rigid-body';
import { INACTIVE_BODY_INDEX, sleep, updateSleepState } from './body/sleep';
import type { ConstraintId, ConstraintType } from './constraints/constraint-id';
import type { Constraints } from './constraints/constraints';
import * as constraints from './constraints/constraints';
import type { ContactConstraints } from './constraints/contact-constraints';
import type { Contacts } from './contacts';
import { assert } from './utils/assert';
import type { World } from './world';
import type { WorldSettings } from './world-settings';

const NO_ISLAND = -1;

/** temporary object for reading constraint iteration overrides */
const _finalize_constraintOverrides = /* @__PURE__ */ constraints.createConstraintIterationOverrides();

/** islands state, groups connected bodies for independent constraint solving */
export type Islands = {
    /** union-find parent links for each active body (active index -> parent active index) */
    bodyLinks: number[];
    /** island index for each active body after finalization (active index -> island index) */
    bodyIslands: number[];
    /** contact links: contact index -> minimum active body index */
    contactLinks: number[];
    /** constraint links: active constraint index -> minimum active body index */
    constraintLinks: number[];
    /** active constraint IDs in order (dense array of active constraints) */
    constraintIds: ConstraintId[];
    /** finalized islands data (available after finalize() is called) */
    islands: Island[];
};

/** island data after finalization */
export type Island = {
    /** island index */
    index: number;
    /** indices of bodies in this island */
    bodyIndices: number[];
    /** indices of contacts in this island */
    contactIndices: number[];
    /** constraint IDs in this island (packed ConstraintId with type, index, seq) */
    constraintIds: ConstraintId[];
    /** number of velocity solver iterations for this island */
    numVelocitySteps: number;
    /** number of position solver iterations for this island */
    numPositionSteps: number;
};

/** init the island builder state */
export function init(): Islands {
    return {
        bodyLinks: [],
        bodyIslands: [],
        contactLinks: [],
        constraintLinks: [],
        constraintIds: [],
        islands: [],
    };
}

/** initialize island builder with active bodies (dynamic + kinematic), each active body starts as its own island */
export function prepare(state: Islands, bodies: Bodies, maxContacts: number): void {
    // reset all bodies' island to -1 (not in any island)
    for (const body of bodies.pool) {
        if (body && !body._pooled) {
            body.islandIndex = NO_ISLAND;
        }
    }

    const numActiveBodies = bodies.activeBodyCount;
    state.bodyLinks.length = numActiveBodies;
    state.bodyIslands.length = numActiveBodies;

    // allocate contactLinks to maxContacts size and initialize to INACTIVE_INDEX
    state.contactLinks.length = maxContacts;
    for (let i = 0; i < maxContacts; i++) {
        state.contactLinks[i] = INACTIVE_BODY_INDEX;
    }

    // each body starts pointing to itself (separate island)
    for (let i = 0; i < numActiveBodies; i++) {
        state.bodyLinks[i] = i;
        state.bodyIslands[i] = 0;
    }

    // reset constraint tracking arrays (will be populated by linkConstraint calls)
    state.constraintLinks.length = 0;
    state.constraintIds.length = 0;
}

/** get the lowest body index in an island chain, this follows the chain without path compression - compression happens in LinkBodies */
function getLowestBodyIndex(state: Islands, bodyIndex: number): number {
    let index = bodyIndex;
    while (state.bodyLinks[index] !== index) {
        index = state.bodyLinks[index];
    }
    return index;
}

/** link two bodies into the same island (union operation) */
export function linkBodies(state: Islands, bodies: Bodies, bodyIndexA: number, bodyIndexB: number): void {
    // both need to be active, we don't want to create an island with static objects
    const maxActiveBodies = bodies.activeBodyCount;
    if (bodyIndexA >= maxActiveBodies || bodyIndexB >= maxActiveBodies) {
        return;
    }

    // start the algorithm with the two bodies
    let firstLinkTo = bodyIndexA;
    let secondLinkTo = bodyIndexB;

    // follow the chain until we get to the body with lowest index
    firstLinkTo = getLowestBodyIndex(state, firstLinkTo);
    secondLinkTo = getLowestBodyIndex(state, secondLinkTo);

    // if the targets are the same, the bodies are already connected
    if (firstLinkTo === secondLinkTo) {
        return;
    }

    // we always link the highest to the lowest
    if (firstLinkTo < secondLinkTo) {
        state.bodyLinks[secondLinkTo] = firstLinkTo;
    } else {
        state.bodyLinks[firstLinkTo] = secondLinkTo;
    }

    // path compression: chains of bodies can become really long, resulting in an O(N) loop
    // to prevent this we update the link of the bodies that were passed in to directly point
    // to the lowest index that we found (only if lower than current value)
    const lowestLinkTo = Math.min(firstLinkTo, secondLinkTo);
    if (state.bodyLinks[bodyIndexA] > lowestLinkTo) {
        state.bodyLinks[bodyIndexA] = lowestLinkTo;
    }
    if (state.bodyLinks[bodyIndexB] > lowestLinkTo) {
        state.bodyLinks[bodyIndexB] = lowestLinkTo;
    }
}

/**
 * Link bodies connected by a contact. Stores the minimum active body index - does NOT call LinkBodies.
 * The union-find happens only through explicit body links (constraints, etc).
 *
 * INACTIVE_BODY_INDEX is 0xffffffff, so min() will pick the active body when one is inactive, or INACTIVE_BODY_INDEX when both are inactive.
 */
export function linkContact(state: Islands, contactIndex: number, bodyA: RigidBody, bodyB: RigidBody): void {
    // store minimum of the two activeIndex values
    // - both active: stores the smaller active index
    // - one active, one inactive: stores the active index (INACTIVE_BODY_INDEX is max value)
    // - both inactive: stores INACTIVE_BODY_INDEX
    state.contactLinks[contactIndex] = Math.min(bodyA.activeIndex, bodyB.activeIndex);
}

/**
 * Link a constraint to islands. Links the bodies together and stores the constraint ID.
 * @param state island builder state
 * @param constraintId the packed constraint ID (contains type, index, sequence)
 * @param bodyA first body
 * @param bodyB second body
 */
export function linkConstraint(
    state: Islands,
    bodies: Bodies,
    constraintId: ConstraintId,
    bodyA: RigidBody,
    bodyB: RigidBody,
): void {
    // link bodies together in union-find (if both active)
    if (bodyA.activeIndex !== INACTIVE_BODY_INDEX && bodyB.activeIndex !== INACTIVE_BODY_INDEX) {
        linkBodies(state, bodies, bodyA.activeIndex, bodyB.activeIndex);
    }

    // store the constraint ID and minimum active body index (both arrays grow together)
    state.constraintIds.push(constraintId);
    state.constraintLinks.push(Math.min(bodyA.activeIndex, bodyB.activeIndex));
}

/**
 * Link all contact constraints to islands.
 * For each constraint, links the two bodies and records the contact index.
 */
export function linkContactConstraints(
    state: Islands,
    contactConstraints: ContactConstraints,
    contacts: Contacts,
    bodies: Bodies,
): void {
    for (let constraintIdx = 0; constraintIdx < contactConstraints.constraints.length; constraintIdx++) {
        const constraint = contactConstraints.constraints[constraintIdx];
        const contact = contacts.contacts[constraint.contactIndex];
        const bodyA = bodies.pool[contact.bodyIndexA];
        const bodyB = bodies.pool[contact.bodyIndexB];

        // link bodies in union-find if both are active (dynamic/kinematic)
        if (bodyA.activeIndex !== INACTIVE_BODY_INDEX && bodyB.activeIndex !== INACTIVE_BODY_INDEX) {
            linkBodies(state, bodies, bodyA.activeIndex, bodyB.activeIndex);
        }

        // link the contact with the constraint index
        linkContact(state, constraintIdx, bodyA, bodyB);
    }
}

/**
 * Link all user constraints to islands.
 * Iterates all constraint types and links active, enabled constraints.
 */
export function linkUserConstraints(state: Islands, constraintsState: Constraints, bodies: Bodies): void {
    for (const type in constraintsState.pools) {
        const pool = constraintsState.pools[type as unknown as ConstraintType]!;
        for (const constraint of pool.constraints) {
            if (!constraint._pooled && constraint.enabled) {
                const bodyA = bodies.pool[constraint.bodyIndexA];
                const bodyB = bodies.pool[constraint.bodyIndexB];

                // only link if at least one body is active
                if (bodyA.activeIndex !== INACTIVE_BODY_INDEX || bodyB.activeIndex !== INACTIVE_BODY_INDEX) {
                    linkConstraint(state, bodies, constraint.id, bodyA, bodyB);
                }
            }
        }
    }
}

/** compare islands by constraint count for sorting */
function compareIslandSize(a: Island, b: Island): number {
    const numConstraintsA = a.contactIndices.length + a.constraintIds.length;
    const numConstraintsB = b.contactIndices.length + b.constraintIds.length;
    return numConstraintsB - numConstraintsA;
}

/**
 * Finalize islands and store grouped island data on builder.
 * 1. Build body islands (assign island indices based on union-find)
 * 2. Map contacts to islands (based on the body index they link to)
 * 3. Sort islands by size (largest first)
 */
export function finalize(state: Islands, bodies: Bodies, constraintsState: Constraints, worldSettings: WorldSettings): void {
    const numActiveBodies = bodies.activeBodyCount;

    // build body islands, calculate island index for all active bodies
    let numIslands = 0;

    // first island always starts at 0
    const islandStarts: number[] = [0];

    for (let i = 0; i < numActiveBodies; i++) {
        const linkTo = state.bodyLinks[i];

        if (linkTo !== i) {
            // links to another body, take island index from other body
            // (must have been filled in already since we loop from low to high)
            assert(linkTo < i, 'Body links should point to lower indices');
            const islandIndex = state.bodyIslands[linkTo];
            state.bodyIslands[i] = islandIndex;

            // increment the start of the next island
            islandStarts[islandIndex + 1] = (islandStarts[islandIndex + 1] || 0) + 1;
        } else {
            // does not link to other body, this is the start of a new island
            state.bodyIslands[i] = numIslands;
            numIslands++;

            // set the start of the next island to 1
            islandStarts[numIslands] = 1;
        }
    }

    // make the start array absolute (so far we only counted)
    for (let island = 1; island < numIslands; island++) {
        islandStarts[island] += islandStarts[island - 1];
    }

    // build islands array
    const islands: Island[] = [];
    for (let i = 0; i < numIslands; i++) {
        islands.push({
            index: i,
            bodyIndices: [],
            contactIndices: [],
            constraintIds: [],
            numVelocitySteps: 0,
            numPositionSteps: 0,
        });
    }

    // group bodies by island using the active body list
    for (let activeIndex = 0; activeIndex < numActiveBodies; activeIndex++) {
        const bodyIndex = bodies.activeBodyIndices[activeIndex];
        const body = bodies.pool[bodyIndex];
        if (!body || body._pooled) continue;

        const islandIndex = state.bodyIslands[activeIndex];
        assert(islandIndex >= 0 && islandIndex < numIslands, 'Invalid island index');

        islands[islandIndex].bodyIndices.push(activeIndex);

        // set island property on body
        body.islandIndex = islandIndex;
    }

    // map contacts to islands, roup contacts by the island of the active body they link to
    for (let contactIndex = 0; contactIndex < state.contactLinks.length; contactIndex++) {
        const activeBodyIndex = state.contactLinks[contactIndex];

        // skip if marked as INACTIVE_INDEX (both bodies static or contact not linked)
        if (activeBodyIndex >= INACTIVE_BODY_INDEX) continue;

        assert(activeBodyIndex >= 0 && activeBodyIndex < numActiveBodies, 'Invalid active body index');

        const islandIndex = state.bodyIslands[activeBodyIndex];
        assert(islandIndex >= 0 && islandIndex < numIslands, 'Invalid island index');

        islands[islandIndex].contactIndices.push(contactIndex);
    }

    // map constraints to islands using the unified constraint arrays
    for (let activeConstraintIndex = 0; activeConstraintIndex < state.constraintIds.length; activeConstraintIndex++) {
        const activeBodyIndex = state.constraintLinks[activeConstraintIndex];

        // skip if marked as INACTIVE_INDEX (both bodies static or constraint not linked)
        if (activeBodyIndex >= INACTIVE_BODY_INDEX) continue;

        assert(activeBodyIndex >= 0 && activeBodyIndex < numActiveBodies, 'Invalid active body index');

        const islandIndex = state.bodyIslands[activeBodyIndex];
        assert(islandIndex >= 0 && islandIndex < numIslands, 'Invalid island index');

        // push the constraint ID (not the active index) - the ID contains type, index, seq
        islands[islandIndex].constraintIds.push(state.constraintIds[activeConstraintIndex]);
    }

    // sort islands by constraint count so the biggest islands go first for better load balancing
    // (matching JoltPhysics: jobs that take longest run first to improve parallel efficiency)
    islands.sort(compareIslandSize);

    // update island indices after sorting
    for (let i = 0; i < islands.length; i++) {
        islands[i].index = i;
    }

    // calculate solver iterations for each island
    for (const island of islands) {
        let numVelocitySteps = 0;
        let numPositionSteps = 0;
        let applyDefaultVelocity = false;
        let applyDefaultPosition = false;

        // check all bodies in island
        for (const activeIndex of island.bodyIndices) {
            const bodyIndex = bodies.activeBodyIndices[activeIndex];
            const body = bodies.pool[bodyIndex];
            if (!body || body.motionType !== MotionType.DYNAMIC) continue;

            const mp = body.motionProperties;
            numVelocitySteps = Math.max(numVelocitySteps, mp.numVelocityStepsOverride);
            applyDefaultVelocity ||= mp.numVelocityStepsOverride === 0;

            numPositionSteps = Math.max(numPositionSteps, mp.numPositionStepsOverride);
            applyDefaultPosition ||= mp.numPositionStepsOverride === 0;
        }

        // check all constraints in island
        for (const constraintId of island.constraintIds) {
            constraints.getConstraintIterationOverrides(_finalize_constraintOverrides, constraintsState, constraintId);

            numVelocitySteps = Math.max(numVelocitySteps, _finalize_constraintOverrides.velocity);
            applyDefaultVelocity ||= _finalize_constraintOverrides.velocity === 0;

            numPositionSteps = Math.max(numPositionSteps, _finalize_constraintOverrides.position);
            applyDefaultPosition ||= _finalize_constraintOverrides.position === 0;
        }

        // apply defaults if any body or constraint uses 0
        if (applyDefaultVelocity) {
            numVelocitySteps = Math.max(numVelocitySteps, worldSettings.solver.velocityIterations);
        }
        if (applyDefaultPosition) {
            numPositionSteps = Math.max(numPositionSteps, worldSettings.solver.positionIterations);
        }

        island.numVelocitySteps = numVelocitySteps;
        island.numPositionSteps = numPositionSteps;
    }

    state.islands = islands;
}

/** check if an island can sleep and deactivate all bodies in it if so, called after solving constraints for an island */
export function checkIslandSleep(island: Island, world: World, deltaTime: number): void {
    if (!world.settings.sleeping.allowSleeping) {
        return;
    }

    const bodies = world.bodies;
    const timeBeforeSleep = world.settings.sleeping.timeBeforeSleep;
    const maxMovement = world.settings.sleeping.pointVelocitySleepThreshold * timeBeforeSleep;

    let allCanSleep = true;

    // check each body in island (sleeping bodies are excluded from islands during init)
    for (const activeIndex of island.bodyIndices) {
        const bodyIndex = bodies.activeBodyIndices[activeIndex];
        const body = bodies.pool[bodyIndex];
        if (!body || body.motionType !== MotionType.DYNAMIC) continue;

        const canSleep = updateSleepState(body, deltaTime, maxMovement, timeBeforeSleep);
        allCanSleep = allCanSleep && canSleep;
    }

    // if all bodies can sleep, deactivate the island
    if (allCanSleep) {
        for (const activeIndex of island.bodyIndices) {
            const bodyIndex = bodies.activeBodyIndices[activeIndex];
            const body = bodies.pool[bodyIndex];
            if (body && body.motionType === MotionType.DYNAMIC) {
                sleep(world, body);
            }
        }
    }
}
