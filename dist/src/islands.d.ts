import type { Bodies } from './body/bodies.js';
import type { RigidBody } from './body/rigid-body.js';
import type { ConstraintId } from './constraints/constraint-id.js';
import type { Constraints } from './constraints/constraints.js';
import type { ContactConstraints } from './constraints/contact-constraints.js';
import type { Contacts } from './contacts.js';
import type { World } from './world.js';
import type { WorldSettings } from './world-settings.js';
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
export declare function init(): Islands;
/** initialize island builder with active bodies (dynamic + kinematic), each active body starts as its own island */
export declare function prepare(state: Islands, bodies: Bodies, maxContacts: number): void;
/** link two bodies into the same island (union operation) */
export declare function linkBodies(state: Islands, bodies: Bodies, bodyIndexA: number, bodyIndexB: number): void;
/**
 * Link bodies connected by a contact. Stores the minimum active body index - does NOT call LinkBodies.
 * The union-find happens only through explicit body links (constraints, etc).
 *
 * INACTIVE_BODY_INDEX is 0xffffffff, so min() will pick the active body when one is inactive, or INACTIVE_BODY_INDEX when both are inactive.
 */
export declare function linkContact(state: Islands, contactIndex: number, bodyA: RigidBody, bodyB: RigidBody): void;
/**
 * Link a constraint to islands. Links the bodies together and stores the constraint ID.
 * @param state island builder state
 * @param constraintId the packed constraint ID (contains type, index, sequence)
 * @param bodyA first body
 * @param bodyB second body
 */
export declare function linkConstraint(state: Islands, bodies: Bodies, constraintId: ConstraintId, bodyA: RigidBody, bodyB: RigidBody): void;
/**
 * Link all contact constraints to islands.
 * For each constraint, links the two bodies and records the contact index.
 */
export declare function linkContactConstraints(state: Islands, contactConstraints: ContactConstraints, contacts: Contacts, bodies: Bodies): void;
/**
 * Link all user constraints to islands.
 * Iterates all constraint types and links active, enabled constraints.
 */
export declare function linkUserConstraints(state: Islands, constraintsState: Constraints, bodies: Bodies): void;
/**
 * Finalize islands and store grouped island data on builder.
 * 1. Build body islands (assign island indices based on union-find)
 * 2. Map contacts to islands (based on the body index they link to)
 * 3. Sort islands by size (largest first)
 */
export declare function finalize(state: Islands, bodies: Bodies, constraintsState: Constraints, worldSettings: WorldSettings): void;
/** check if an island can sleep and deactivate all bodies in it if so, called after solving constraints for an island */
export declare function checkIslandSleep(island: Island, world: World, deltaTime: number): void;
