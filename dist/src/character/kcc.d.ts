import { type Quat, type Vec3, type Vec4 } from 'mathcat';
import type { BodyId } from '../body/body-id.js';
import { MotionType } from '../body/motion-type.js';
import type { RigidBody } from '../body/rigid-body.js';
import type { Filter } from '../filter.js';
import type { Shape } from '../shapes/shapes.js';
import type { World } from '../world.js';
/** KCC (kinematic character controller) state */
export type KCC = {
    /** world-space position */
    position: Vec3;
    /** orientation */
    quaternion: Quat;
    /** linear velocity */
    linearVelocity: Vec3;
    /** up direction (normalized) */
    up: Vec3;
    /** collision shape */
    shape: Shape;
    /** local space offset of shape relative to character position */
    shapeOffset: Vec3;
    /** character mass (kg) - for pushing objects */
    mass: number;
    /** max push force (N) */
    maxStrength: number;
    /** look-ahead distance for predictive contacts */
    predictiveContactDistance: number;
    /** max collision detection iterations per update */
    maxCollisionIterations: number;
    /** max constraint solving iterations */
    maxConstraintIterations: number;
    /** early-out threshold for time remaining */
    minTimeRemaining: number;
    /** penetration tolerance */
    collisionTolerance: number;
    /** spacing from surfaces */
    characterPadding: number;
    /** max contacts per query */
    maxNumHits: number;
    /** hit merging angle (cosine) */
    hitReductionCosMaxAngle: number;
    /** penetration recovery speed (0-1) */
    penetrationRecoverySpeed: number;
    /** max slope angle (radians) - slopes steeper are not walkable */
    maxSlopeAngle: number;
    /** cosine of max slope angle (precomputed) */
    cosMaxSlopeAngle: number;
    /** whether to collide with triangle mesh back faces */
    backFaceMode: BackFaceMode;
    /** enable enhanced internal edge removal (expensive, off by default) */
    enhancedInternalEdgeRemoval: boolean;
    /**
     * Plane defining what can support the character (normal.xyz, constant.w).
     * Every contact behind this plane can support the character.
     * Every contact in front of this plane only collides without providing support.
     * Default: [up.x, up.y, up.z, -1e10] - accepts any contact.
     * Plane equation: dot(point, normal) + constant = 0
     * SignedDistance(point) = dot(point, normal) + constant
     */
    supportingVolumePlane: Vec4;
    /** current ground info */
    ground: GroundInfo;
    /** inner rigid body ID for physics visibility (or INVALID_BODY_ID) */
    innerRigidBodyId: BodyId;
    /** configuration for the inner rigid body (for visibility to physics world) */
    innerRigidBody: {
        shape: Shape;
        objectLayer: number;
    } | undefined;
    /** active contacts for current frame */
    contacts: CharacterContact[];
    /** pool of listener contact tracking values */
    listenerContacts: ListenerContactsPool;
    /** last delta time from update() - used for supporting contact velocity check */
    lastDeltaTime: number;
};
/** ground state of the character */
export declare enum GroundState {
    /** on walkable surface, can move freely */
    ON_GROUND = 0,
    /** on slope too steep to climb */
    ON_STEEP_GROUND = 1,
    /** touching something but not supported */
    NOT_SUPPORTED = 2,
    /** not touching anything */
    IN_AIR = 3
}
/** how to handle back faces of triangle meshes during collision detection */
export declare enum BackFaceMode {
    /** ignore back faces of triangle meshes */
    IGNORE = 0,
    /** collide with back faces of triangle meshes */
    COLLIDE = 1
}
/** contact information for a character collision */
export type CharacterContact = {
    /** world position of contact */
    position: Vec3;
    /** velocity at contact point (from contacted body) */
    linearVelocity: Vec3;
    /** normal pointing toward character */
    contactNormal: Vec3;
    /** true surface normal (may differ from contactNormal for edges) */
    surfaceNormal: Vec3;
    /** distance to surface (negative = penetration) */
    distance: number;
    /** fraction along sweep (0-1) */
    fraction: number;
    /** body ID of contacted body (use BodyId for persistence safety) */
    bodyId: BodyId;
    /** sub-shape ID on contacted body */
    subShapeId: number;
    /** material ID of contacted sub-shape */
    materialId: number;
    /** motion type of contacted body (for constraint prioritization) */
    motionType: MotionType;
    /** is the contacted body a sensor? (sensors don't push character) */
    isSensor: boolean;
    /** did actual collision occur? (vs just predictive) */
    hadCollision: boolean;
    /** was contact discarded by validation? */
    wasDiscarded: boolean;
    /** can this contact push the character? */
    canPushCharacter: boolean;
    /** can character push body? (apply impulses) */
    canReceiveImpulses: boolean;
};
/** movement constraint derived from a contact, used during constraint solving */
export type CharacterConstraint = {
    /** contact that generated this constraint */
    contact: CharacterContact;
    /** time of impact (can be negative if penetrating) */
    toi: number;
    /** velocity projected on contact normal */
    projectedVelocity: number;
    /** velocity of contact (with penetration correction) */
    linearVelocity: Vec3;
    /** plane constraint normal */
    planeNormal: Vec3;
    /** plane constraint distance from character center */
    planeDistance: number;
    /** is this a steep slope constraint? */
    isSteepSlope: boolean;
};
/** information about the character's current ground contact */
export type GroundInfo = {
    /** current ground state */
    state: GroundState;
    /** ground contact normal (average of supporting normals) */
    normal: Vec3;
    /** ground velocity */
    velocity: Vec3;
    /** body ID of ground (or INVALID_BODY_ID if in air) */
    bodyId: BodyId;
    /** sub-shape ID of ground contact */
    subShapeId: number;
    /** position of ground contact */
    position: Vec3;
};
/** per-contact behavior settings, can be modified in listener callbacks to customize contact behavior */
export type CharacterContactSettings = {
    /** can body push character? */
    canPushCharacter: boolean;
    /** can character push body? (apply impulses) */
    canReceiveImpulses: boolean;
};
/**
 * Internal tracking value for contact change detection.
 * Used to track which contacts are new, persisted, or removed.
 */
type ListenerContactValue = {
    /** pool index when active, -1 when pooled */
    poolIndex: number;
    /** packed key for lookup (bodyId << 16 | subShapeId) */
    packedKey: number;
    /** body ID of the contact */
    bodyId: BodyId;
    /** sub-shape ID of the contact */
    subShapeId: number;
    /** count: 0 = not seen this frame, 1+ = seen */
    count: number;
    /** cached settings from callback */
    settings: CharacterContactSettings;
};
/** settings for creating a kinematic character */
export type KCCSettings = {
    /** collision shape for the character */
    shape: Shape;
    /** local space offset of shape relative to character position @default [0,0,0] */
    shapeOffset?: Vec3;
    /** character mass in kg @default 70 */
    mass?: number;
    /** maximum push force in N @default 100 */
    maxStrength?: number;
    /** up direction @default [0,1,0] */
    up?: Vec3;
    /** max slope angle in radians @default ~0.873 = 50° */
    maxSlopeAngle?: number;
    /** look-ahead distance for predictive contacts @default 0.1 */
    predictiveContactDistance?: number;
    /** spacing from surfaces @default 0.02 */
    characterPadding?: number;
    /** max collision detection iterations per update @default 5 */
    maxCollisionIterations?: number;
    /** max constraint solving iterations @default 15 */
    maxConstraintIterations?: number;
    /** penetration tolerance @default 1e-3 */
    collisionTolerance?: number;
    /** penetration recovery speed 0-1 @default 1.0 */
    penetrationRecoverySpeed?: number;
    /** early-out threshold for time remaining @default 1e-4 */
    minTimeRemaining?: number;
    /** max contacts per query @default 256 */
    maxNumHits?: number;
    /** hit merging angle as cosine @default cos(2.5°)≈0.999 */
    hitReductionCosMaxAngle?: number;
    /** how to handle triangle mesh back faces @default BackFaceMode.COLLIDE */
    backFaceMode?: BackFaceMode;
    /** enable enhanced internal edge removal @default false */
    enhancedInternalEdgeRemoval?: boolean;
    /**
     * Plane defining what can support the character (normal.xyz, constant.w).
     * For a capsule shape you might use [0, 1, 0, -characterRadius] for stricter filtering.
     * @default [up.x,up.y,up.z,-1e10] - accepts any contact.
     */
    supportingVolumePlane?: Vec4;
    /** configuration for the optional inner rigid body (for character presence in the physics world) */
    innerRigidBody?: {
        shape: Shape;
        objectLayer: number;
    };
};
/** listener for character contact events, implement to customize contact handling and receive contact notifications */
export type CharacterListener = {
    /** called to adjust a body's velocity before collision detection (e.g., for conveyor belts) */
    onAdjustBodyVelocity?: (character: KCC, body: RigidBody, ioLinearVelocity: Vec3, // can be modified
    ioAngularVelocity: Vec3) => void;
    /** called to validate contact, returns true to accept the contact, false to reject it */
    onContactValidate?: (character: KCC, body: RigidBody, subShapeId: number, contactPosition: Vec3, contactNormal: Vec3) => boolean;
    /** called when a new contact is added */
    onContactAdded?: (character: KCC, body: RigidBody, subShapeId: number, contactPosition: Vec3, contactNormal: Vec3, settings: CharacterContactSettings) => void;
    /** called when an existing contact persists (still active this frame) */
    onContactPersisted?: (character: KCC, body: RigidBody, subShapeId: number, contactPosition: Vec3, contactNormal: Vec3, settings: CharacterContactSettings) => void;
    /** called during constraint solving to allow velocity modification */
    onContactSolve?: (character: KCC, body: RigidBody, subShapeId: number, contactPosition: Vec3, contactNormal: Vec3, contactVelocity: Vec3, characterVelocity: Vec3, ioCharacterVelocity: Vec3) => void;
    /** called when a contact is removed */
    onContactRemoved?: (character: KCC, body: RigidBody, subShapeId: number) => void;
};
/** default settings for kinematic character controller when settings are not specified */
export declare const DEFAULT_KCC_SETTINGS: {
    readonly up: Vec3;
    readonly mass: 70;
    readonly maxStrength: 100;
    readonly predictiveContactDistance: 0.1;
    readonly maxCollisionIterations: 5;
    readonly maxConstraintIterations: 15;
    readonly minTimeRemaining: 0.0001;
    readonly collisionTolerance: 0.001;
    readonly characterPadding: 0.02;
    readonly maxNumHits: 256;
    readonly hitReductionCosMaxAngle: number;
    readonly penetrationRecoverySpeed: 1;
    readonly maxSlopeAngle: number;
    readonly backFaceMode: BackFaceMode.COLLIDE;
    readonly enhancedInternalEdgeRemoval: false;
};
/**
 * Creates a new kinematic character controller.
 *
 * If you want to add an inner rigid body to the physics world, you must also call add() and remove() as needed.
 * Otherwise, if the character is purely "virtual", you can skip that step.
 *
 * @param settings character settings
 * @param position initial world position
 * @param quaternion initial quaternion
 * @returns the new kinematic character controller
 */
export declare function create(settings: KCCSettings, position: Vec3, quaternion: Quat): KCC;
/**
 * Adds the character to the physics world by creating an inner rigid body.
 *
 * The inner body serves multiple purposes:
 * 1. **Visibility**: Regular raycasts can hit the character
 * 2. **Contact Callbacks**: Physics Listener receives callbacks
 * 3. **CCD**: Fast objects won't pass through character
 * 4. **Sensors**: Character can trigger sensor volumes
 *
 * Note: Inner body collision does NOT affect character movement.
 * The character controller's own collision detection is separate.
 *
 * Requires innerRigidBody to be set on the character.
 *
 * @param world the physics world
 * @param character the character controller
 */
export declare function add(world: World, character: KCC): void;
/**
 * Removes the character from the physics world by destroying the inner body.
 * @param world the physics world
 * @param character the character controller
 */
export declare function remove(world: World, character: KCC): void;
/** pool for managing ListenerContactValue objects with minimal allocations */
type ListenerContactsPool = {
    /** all contacts (pooled and active) */
    pool: ListenerContactValue[];
    /** indices of free (pooled) contacts */
    freeIndices: number[];
};
/**
 * Settings for update() with stair walking and floor sticking.
 */
export type UpdateSettings = {
    /** step down distance for floor sticking (default [0, -0.5, 0]) */
    stickToFloorStepDown: Vec3;
    /** step up distance for stair walking (default [0, 0.4, 0]) */
    walkStairsStepUp: Vec3;
    /** minimum forward movement for stair detection (default 0.02) */
    walkStairsMinStepForward: number;
    /** test distance for floor validation (default 0.15) */
    walkStairsStepForwardTest: number;
    /** cos angle for forward contact detection (default cos(75°) ≈ 0.259) */
    walkStairsCosAngleForwardContact: number;
    /** extra step down after stair walk (default [0, 0, 0]) */
    walkStairsStepDownExtra: Vec3;
};
export declare function createDefaultUpdateSettings(): UpdateSettings;
/**
 * Checks if a slope is too steep to walk on.
 * @param character the character controller
 * @param normal the surface normal to test
 * @returns true if the slope is steeper than maxSlopeAngle
 */
export declare function isSlopeTooSteep(character: KCC, normal: Vec3): boolean;
/**
 * Gets the world-space center of mass transform for the character.
 * This accounts for the shape offset.
 *
 * @param character the character controller
 * @param outPosition output for the center of mass position
 * @param outQuaternion output for the orientation
 */
export declare function getCenterOfMassTransform(character: KCC, outPosition: Vec3, outQuaternion: Quat): void;
/**
 * Checks if the character is supported (on ground or on steep ground that provides support).
 * @param character the character controller
 * @returns true if the character is on ground or on steep ground
 */
export declare function isSupported(character: KCC): boolean;
/**
 * Sets the character's position and synchronizes the inner body if present.
 * @param world the physics world
 * @param character the character controller
 * @param position the new world-space position
 */
export declare function setPosition(world: World, character: KCC, position: Vec3): void;
/**
 * Sets the character's rotation and synchronizes the inner body if present.
 * @param world the physics world
 * @param character the character controller
 * @param quaternion the new rotation quaternion
 */
export declare function setQuaternion(world: World, character: KCC, quaternion: Quat): void;
/**
 * Updates the ground velocity to account for rotation of the ground body.
 * This is needed for rotating platforms - the character should move with the platform's
 * rotation, not just its linear velocity.
 *
 * Note: This function recalculates the ground velocity from the ground body,
 * applying any listener adjustments (e.g., conveyor belts). It should only be called
 * when you want to refresh the ground velocity based on the current ground body state.
 *
 * @param character the character controller
 * @param world the physics world
 * @param listener optional listener for adjusting body velocities
 */
export declare function updateGroundVelocity(world: World, character: KCC, listener?: CharacterListener): void;
/**
 * Refreshes the contacts for a character at its current position.
 *
 * Call this after teleporting the character or making other changes that may affect
 * which contacts are active. This performs full collision detection and updates
 * the ground state and active contacts.
 *
 * @param world the physics world
 * @param character the character controller
 * @param filter collision filter
 * @param listener optional listener for callbacks (will receive onContactRemoved for contacts that are no longer active)
 */
export declare function refreshContacts(world: World, character: KCC, filter: Filter, listener?: CharacterListener): void;
/**
 * Checks if the character has collided with a specific body during the current frame.
 *
 * @param character the character controller
 * @param bodyId the body ID to check for collision with
 * @returns true if the character has collided with the specified body
 */
export declare function hasCollidedWith(character: KCC, bodyId: BodyId): boolean;
/**
 * Checks if the character has steep slopes it might need to walk up.
 *
 * This is useful to determine if the character should attempt stair-walking logic.
 * Returns true if the character is supported and has steep slope contacts that
 * the character is pushing into (based on velocity direction).
 *
 * @param character the character controller
 * @param linearVelocity the desired linear velocity (used to check movement direction)
 * @returns true if there are steep slopes the character might need to walk up
 */
export declare function canWalkStairs(character: KCC, linearVelocity: Vec3): boolean;
/**
 * Changes the shape of the character, checking for penetration first.
 *
 * Algorithm:
 * 1. Early exit if shape is the same
 * 2. If penetration testing enabled (maxPenDepth < MAX_VALUE):
 *    a. Test collisions with new shape in temp pool
 *    b. Reject if any contact penetrates deeper than threshold (and not sensor)
 *    c. Track contact changes: resetContactTracking, replace contacts, updateSupportingContact, finalizeContactTracking
 * 3. Set new shape
 * 4. Update inner body shape if it exists
 *
 * @param world the physics world
 * @param character the character controller
 * @param newShape the new shape to use
 * @param filter collision filter for penetration testing
 * @param listener optional listener for contact callbacks
 * @param maxPenetrationDepth maximum allowed penetration, use `Number.MAX_VALUE` / `Infinity` to skip testing
 * @returns true if shape was changed successfully, false if blocked by penetration
 */
export declare function setShape(world: World, character: KCC, newShape: Shape, filter: Filter, listener: CharacterListener | undefined, maxPenetrationDepth: number): boolean;
/**
 * Low-level movement function - moves character with collision resolution.
 *
 * This is the core movement routine that handles physics simulation:
 * 1. early exit if deltaTime <= 0
 * 2. store deltaTime for supporting contact velocity check
 * 3. moveShape() to slide through the world
 * 4. updateSupportingContact() to determine ground state
 * 5. updateInnerBodyTransform() if inner body exists
 * 6. apply gravity impulse to ground body if character is on ground
 *
 * Note: This function does NOT handle contact tracking (added/persisted/removed callbacks).
 * For full functionality including contact tracking, stair walking, and floor sticking,
 * use update() instead. If you use move() directly, you must call resetContactTracking()
 * before and finalizeContactTracking() after if you want removal callbacks.
 *
 * @param world the physics world
 * @param character the character controller
 * @param deltaTime time step in seconds
 * @param gravity gravity vector, used in impulses on dynamic ground bodies
 * @param listener optional listener for contact callbacks
 * @param filter collision filter
 */
export declare function move(world: World, character: KCC, deltaTime: number, gravity: Vec3, listener: CharacterListener | undefined, filter: Filter): void;
/**
 * Sticks character to floor when leaving slopes.
 * Prevents character from "bouncing" when walking down slopes or steps.
 *
 * 1. Sweep down to find floor
 * 2. If floor found, move to contact position
 * 3. Update supporting contact state
 *
 * @param world the physics world
 * @param character the character controller
 * @param stepDown down direction and distance (e.g., [0, -0.5, 0])
 * @param filter collision filter
 * @param listener optional listener for callbacks
 * @returns true if floor found and character was moved, false otherwise
 */
export declare function stickToFloor(world: World, character: KCC, stepDown: Vec3, filter: Filter, listener: CharacterListener | undefined): boolean;
/**
 * Attempts to walk up stairs.
 * 1. Sweep up to find headroom
 * 2. Collect steep slope normals we're pushing against
 * 3. Move horizontally at elevated position
 * 4. Check that we made progress toward steep slopes
 * 5. Sweep down to find floor
 * 6. Validate floor is not too steep (or use forward test)
 * 7. Move to final position
 * 8. Override ground state to ON_GROUND
 * @param world the physics world
 * @param character the character controller
 * @param deltaTime time step
 * @param stepUp up step vector (e.g., [0, 0.4, 0])
 * @param stepForward forward step vector (based on velocity * deltaTime)
 * @param stepForwardTest test distance for floor validation
 * @param stepDownExtra extra down step after stair walk
 * @param filter collision filter
 * @param listener optional listener for callbacks
 * @returns true if stair walk succeeded, false otherwise
 */
export declare function walkStairs(world: World, character: KCC, deltaTime: number, stepUp: Vec3, stepForward: Vec3, stepForwardTest: Vec3, stepDownExtra: Vec3, filter: Filter, listener: CharacterListener | undefined): boolean;
/**
 * Main update function - moves character with full collision handling.
 *
 * This is the recommended function for updating character movement. It includes:
 * 1. Contact tracking (added/persisted/removed callbacks)
 * 2. Velocity cancellation toward steep slopes
 * 3. Core movement with collision resolution (via move())
 * 4. Floor sticking when leaving slopes
 * 5. Stair walking when blocked by steep surfaces
 *
 * @param world the physics world
 * @param character the character controller
 * @param deltaTime time step in seconds
 * @param gravity gravity vector
 * @param settings update settings (stair walking, floor sticking)
 * @param listener optional listener for callbacks
 * @param filter collision filter
 */
export declare function update(world: World, character: KCC, deltaTime: number, gravity: Vec3, settings: UpdateSettings, listener: CharacterListener | undefined, filter: Filter): void;
export {};
