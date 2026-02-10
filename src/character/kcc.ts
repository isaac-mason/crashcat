import { type Quat, quat, type Vec3, vec3, type Vec4, vec4 } from 'mathcat';
import type { BodyId } from '../body/body-id';
import { INVALID_BODY_ID } from '../body/body-id';
import * as motionProperties from '../body/motion-properties';
import { MotionType } from '../body/motion-type';
import type { RigidBody } from '../body/rigid-body';
import * as rigidBody from '../body/rigid-body';
import {
    type CastShapeHit,
    type CastShapeSettings,
    CastShapeStatus,
    createDefaultCastShapeSettings,
} from '../collision/cast-shape-vs-shape';
import {
    type CollideShapeHit,
    type CollideShapeSettings,
    createDefaultCollideShapeSettings,
} from '../collision/collide-shape-vs-shape';
import { createGjkCastShapeResult, gjkCastShape } from '../collision/gjk';
import {
    type AddConvexRadiusSupport,
    createAddConvexRadiusSupport,
    createPolygonSupport,
    createShapeSupportPool,
    getShapeSupportFunction,
    setAddConvexRadiusSupport,
    setPolygonSupport,
    type ShapeSupportPool,
    SupportFunctionMode,
} from '../collision/support';
import type { Filter } from '../filter';
import * as query from '../query';
import type { Shape } from '../shapes/shapes';
import { getShapeSupportingFace, ShapeType } from '../shapes/shapes';
import { createFace } from '../utils/face';
import type { World } from '../world';

const ignoreSingleBodyChainedBodyFilterState = {
    bodyId: INVALID_BODY_ID,
    innerBodyFilter: undefined as ((body: RigidBody) => boolean) | undefined,
};

/** chained body filter function that first checks the ignored body ID, then calls the original bodyFilter if one was provided */
function ignoreSingleBodyChainedBodyFilter(body: RigidBody): boolean {
    // first check if this is the body we want to ignore
    if (body.id === ignoreSingleBodyChainedBodyFilterState.bodyId) {
        return false;
    }
    // then call the original filter if it exists
    if (ignoreSingleBodyChainedBodyFilterState.innerBodyFilter) {
        return ignoreSingleBodyChainedBodyFilterState.innerBodyFilter(body);
    }
    // no original filter, accept the body
    return true;
}

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
    innerRigidBody:
        | {
              shape: Shape;
              objectLayer: number;
          }
        | undefined;

    /** active contacts for current frame */
    contacts: CharacterContact[];

    /** pool of listener contact tracking values */
    listenerContacts: ListenerContactsPool;
    /** last delta time from update() - used for supporting contact velocity check */
    lastDeltaTime: number;
};

/** ground state of the character */
export enum GroundState {
    /** on walkable surface, can move freely */
    ON_GROUND = 0,
    /** on slope too steep to climb */
    ON_STEEP_GROUND = 1,
    /** touching something but not supported */
    NOT_SUPPORTED = 2,
    /** not touching anything */
    IN_AIR = 3,
}

/** how to handle back faces of triangle meshes during collision detection */
export enum BackFaceMode {
    /** ignore back faces of triangle meshes */
    IGNORE = 0,
    /** collide with back faces of triangle meshes */
    COLLIDE = 1,
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
    // TODO: shape material id
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
    onAdjustBodyVelocity?: (
        character: KCC,
        body: RigidBody,
        ioLinearVelocity: Vec3, // can be modified
        ioAngularVelocity: Vec3, // can be modified
    ) => void;

    /** called to validate contact, returns true to accept the contact, false to reject it */
    onContactValidate?: (
        character: KCC,
        body: RigidBody,
        subShapeId: number,
        contactPosition: Vec3,
        contactNormal: Vec3,
    ) => boolean;

    /** called when a new contact is added */
    onContactAdded?: (
        character: KCC,
        body: RigidBody,
        subShapeId: number,
        contactPosition: Vec3,
        contactNormal: Vec3,
        settings: CharacterContactSettings,
    ) => void;

    /** called when an existing contact persists (still active this frame) */
    onContactPersisted?: (
        character: KCC,
        body: RigidBody,
        subShapeId: number,
        contactPosition: Vec3,
        contactNormal: Vec3,
        settings: CharacterContactSettings,
    ) => void;

    /** called during constraint solving to allow velocity modification */
    onContactSolve?: (
        character: KCC,
        body: RigidBody,
        subShapeId: number,
        contactPosition: Vec3,
        contactNormal: Vec3,
        contactVelocity: Vec3,
        characterVelocity: Vec3,
        ioCharacterVelocity: Vec3, // can be modified
    ) => void;

    /** called when a contact is removed */
    onContactRemoved?: (character: KCC, body: RigidBody, subShapeId: number) => void;
};

const _create_supportingVolume = /* @__PURE__ */ vec4.create();

/** default settings for kinematic character controller when settings are not specified */
const DEFAULT_KCC_SETTINGS = {
    up: [0, 1, 0] as Vec3,
    mass: 70,
    maxStrength: 100,
    predictiveContactDistance: 0.1,
    maxCollisionIterations: 5,
    maxConstraintIterations: 15,
    minTimeRemaining: 1e-4,
    collisionTolerance: 1e-3,
    characterPadding: 0.02,
    maxNumHits: 256,
    hitReductionCosMaxAngle: Math.cos((2.5 * Math.PI) / 180),
    penetrationRecoverySpeed: 1.0,
    maxSlopeAngle: (50 * Math.PI) / 180,
    backFaceMode: BackFaceMode.COLLIDE,
    enhancedInternalEdgeRemoval: false,
} as const;

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
export function create(settings: KCCSettings, position: Vec3, quaternion: Quat): KCC {
    // get up direction from settings or default to Y-up, normalized
    const up = vec3.normalize(vec3.create(), settings.up ?? DEFAULT_KCC_SETTINGS.up);

    // get max slope angle (default 50 degrees)
    const maxSlopeAngle = settings.maxSlopeAngle ?? DEFAULT_KCC_SETTINGS.maxSlopeAngle;

    // compute supporting volume plane if not provided
    if (settings.supportingVolumePlane) {
        vec4.copy(_create_supportingVolume, settings.supportingVolumePlane);
    } else {
        // default: infinite plane below character
        // plane { Vec3::sAxisY(), -1.0e10f }
        // this accepts any contact as potentially supporting
        vec4.set(_create_supportingVolume, up[0], up[1], up[2], -1.0e10);
    }

    // create the character
    const character: KCC = {
        // transform
        position: vec3.clone(position),
        quaternion: quat.clone(quaternion),

        // movement
        linearVelocity: vec3.create(),
        up: up,

        // shape
        shape: settings.shape,
        shapeOffset: settings.shapeOffset ? vec3.clone(settings.shapeOffset) : vec3.create(),

        // configuration
        mass: settings.mass ?? DEFAULT_KCC_SETTINGS.mass,
        maxStrength: settings.maxStrength ?? DEFAULT_KCC_SETTINGS.maxStrength,
        predictiveContactDistance: settings.predictiveContactDistance ?? DEFAULT_KCC_SETTINGS.predictiveContactDistance,
        maxCollisionIterations: settings.maxCollisionIterations ?? DEFAULT_KCC_SETTINGS.maxCollisionIterations,
        maxConstraintIterations: settings.maxConstraintIterations ?? DEFAULT_KCC_SETTINGS.maxConstraintIterations,
        minTimeRemaining: settings.minTimeRemaining ?? DEFAULT_KCC_SETTINGS.minTimeRemaining,
        collisionTolerance: settings.collisionTolerance ?? DEFAULT_KCC_SETTINGS.collisionTolerance,
        characterPadding: settings.characterPadding ?? DEFAULT_KCC_SETTINGS.characterPadding,
        maxNumHits: settings.maxNumHits ?? DEFAULT_KCC_SETTINGS.maxNumHits,
        hitReductionCosMaxAngle: settings.hitReductionCosMaxAngle ?? DEFAULT_KCC_SETTINGS.hitReductionCosMaxAngle,
        penetrationRecoverySpeed: settings.penetrationRecoverySpeed ?? DEFAULT_KCC_SETTINGS.penetrationRecoverySpeed,
        maxSlopeAngle: maxSlopeAngle,
        cosMaxSlopeAngle: Math.cos(maxSlopeAngle),
        backFaceMode: settings.backFaceMode ?? DEFAULT_KCC_SETTINGS.backFaceMode,
        enhancedInternalEdgeRemoval: settings.enhancedInternalEdgeRemoval ?? DEFAULT_KCC_SETTINGS.enhancedInternalEdgeRemoval,

        // supporting Volume
        supportingVolumePlane: vec4.clone(_create_supportingVolume),

        // ground state (initialized to GroundState.IN_AIR)
        ground: {
            state: GroundState.IN_AIR,
            normal: vec3.fromValues(0, 1, 0),
            velocity: vec3.create(),
            bodyId: INVALID_BODY_ID,
            subShapeId: 0,
            position: vec3.create(),
        },

        // inner body (none initially, use add() to create)
        innerRigidBodyId: INVALID_BODY_ID,
        innerRigidBody: settings.innerRigidBody,

        // contacts (active contacts for current frame, borrowed from module-scope free list)
        contacts: [],

        // contact tracking
        listenerContacts: createListenerContactsPool(),
        lastDeltaTime: 0,
    };

    return character;
}

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
export function add(world: World, character: KCC): void {
    // don't create if already exists
    if (character.innerRigidBodyId !== INVALID_BODY_ID) {
        return;
    }

    // can't add without inner body configuration
    if (!character.innerRigidBody) {
        return;
    }

    // calculate inner body position
    getInnerBodyPosition(character, _innerBody_position);

    // create kinematic body
    const innerRigidBody = rigidBody.create(world, {
        shape: character.innerRigidBody.shape,
        objectLayer: character.innerRigidBody.objectLayer,
        motionType: MotionType.KINEMATIC,
        position: _innerBody_position,
        quaternion: character.quaternion,
        // disable sleeping so sensor callbacks work
        allowSleeping: false,
    });

    character.innerRigidBodyId = innerRigidBody.id;
}

/**
 * Removes the character from the physics world by destroying the inner body.
 * @param world the physics world
 * @param character the character controller
 */
export function remove(world: World, character: KCC): void {
    // Early exit if no inner body
    if (character.innerRigidBodyId === INVALID_BODY_ID) {
        return;
    }

    const innerRigidBody = rigidBody.get(world, character.innerRigidBodyId);

    if (innerRigidBody) {
        rigidBody.remove(world, innerRigidBody);
    }

    character.innerRigidBodyId = INVALID_BODY_ID;
}

/** default capacity for listener contacts pool */
const DEFAULT_LISTENER_CONTACTS_POOL_SIZE = 256;

/** pool for managing ListenerContactValue objects with minimal allocations */
type ListenerContactsPool = {
    /** all contacts (pooled and active) */
    pool: ListenerContactValue[];
    /** indices of free (pooled) contacts */
    freeIndices: number[];
};

/** creates a listener contact value (used for initialization and pooling) */
function createListenerContactValue(): ListenerContactValue {
    return {
        poolIndex: -1,
        packedKey: 0,
        bodyId: 0,
        subShapeId: 0,
        count: 0,
        settings: {
            canPushCharacter: true,
            canReceiveImpulses: true,
        },
    };
}

/** creates a new listener contacts pool with the given capacity */
function createListenerContactsPool(capacity: number = DEFAULT_LISTENER_CONTACTS_POOL_SIZE): ListenerContactsPool {
    const pool: ListenerContactsPool = {
        pool: [],
        freeIndices: [],
    };
    for (let i = 0; i < capacity; i++) {
        const value = createListenerContactValue();
        pool.pool.push(value);
        pool.freeIndices.push(i);
    }
    return pool;
}

/** pack bodyId and subShapeId into a single 32-bit number */
function packListenerContactKey(bodyId: BodyId, subShapeId: number): number {
    // use lower 16 bits for subShapeId, upper 16 bits for bodyId
    // this works because body IDs are typically small integers and subShapeIds are also limited
    return ((bodyId & 0xffff) << 16) | (subShapeId & 0xffff);
}

/** acquires a listener contact from the pool */
function acquireListenerContact(pool: ListenerContactsPool): ListenerContactValue {
    let value: ListenerContactValue;
    if (pool.freeIndices.length > 0) {
        const index = pool.freeIndices.pop()!;
        value = pool.pool[index];
        value.poolIndex = index;
    } else {
        // grow pool if needed
        value = createListenerContactValue();
        value.poolIndex = pool.pool.length;
        pool.pool.push(value);
    }
    return value;
}

/** releases all listener contacts back to the pool */
function releaseAllListenerContacts(pool: ListenerContactsPool): void {
    pool.freeIndices.length = 0;
    for (let i = 0; i < pool.pool.length; i++) {
        pool.pool[i].poolIndex = -1;
        pool.freeIndices.push(i);
    }
}

/** finds a listener contact by packed key, returns null if not found */
function findListenerContact(pool: ListenerContactsPool, packedKey: number): ListenerContactValue | null {
    // linear scan through active contacts
    for (const value of pool.pool) {
        if (value.poolIndex !== -1 && value.packedKey === packedKey) {
            return value;
        }
    }
    return null;
}

/** iterates all active listener contacts */
function getActiveListenerContacts(listenerContacts: ListenerContactsPool): ListenerContactValue[] {
    return listenerContacts.pool.filter((v) => v.poolIndex !== -1);
}

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

export function createDefaultUpdateSettings(): UpdateSettings {
    return {
        stickToFloorStepDown: [0, -0.5, 0],
        walkStairsStepUp: [0, 0.4, 0],
        walkStairsMinStepForward: 0.02,
        walkStairsStepForwardTest: 0.15,
        walkStairsCosAngleForwardContact: Math.cos((75 * Math.PI) / 180),
        walkStairsStepDownExtra: [0, 0, 0],
    };
}

/** contacts pool */
const _contactsPool: CharacterContact[] = [];

/** acquires a contact from the free list or creates a new one */
function acquireContact(contacts: CharacterContact[]): CharacterContact {
    const contact = _contactsPool.pop() ?? createCharacterContact();
    contacts.push(contact);
    return contact;
}

/** releases all contacts back to the free list */
function releaseAllContacts(contacts: CharacterContact[]): void {
    for (let i = 0; i < contacts.length; i++) {
        _contactsPool.push(contacts[i]);
    }
    contacts.length = 0;
}

/** releases a single contact at index back to free list (swap-remove for O(1)) */
function releaseContact(contacts: CharacterContact[], index: number): void {
    _contactsPool.push(contacts[index]);
    contacts[index] = contacts[contacts.length - 1];
    contacts.pop();
}

/** constraints free list (module-scope) */
const _constraintsPool: CharacterConstraint[] = [];

/** active constraints (module-scope, reused across calls) */
const _activeConstraints: CharacterConstraint[] = [];

/** acquires a constraint from the free list and adds to constraints array */
function acquireConstraint(constraints: CharacterConstraint[]): CharacterConstraint {
    const constraint = _constraintsPool.pop() ?? createEmptyCharacterConstraint();
    constraints.push(constraint);
    return constraint;
}

/** releases all constraints back to the free list */
function releaseAllConstraints(constraints: CharacterConstraint[]): void {
    for (let i = 0; i < constraints.length; i++) {
        _constraintsPool.push(constraints[i]);
    }
    constraints.length = 0;
}

/**
 * Creates a CharacterContact with default values.
 * Use this to create scratch contacts or initialize contact storage.
 */
function createCharacterContact(): CharacterContact {
    return {
        position: vec3.create(),
        linearVelocity: vec3.create(),
        contactNormal: vec3.create(),
        surfaceNormal: vec3.create(),
        distance: 0,
        fraction: 0,
        bodyId: INVALID_BODY_ID,
        subShapeId: 0,
        materialId: -1,
        motionType: MotionType.STATIC,
        isSensor: false,
        hadCollision: false,
        wasDiscarded: false,
        canPushCharacter: false,
        canReceiveImpulses: true,
    };
}

/** resets a contact to default values (for reuse) */
function resetContact(contact: CharacterContact): void {
    vec3.set(contact.position, 0, 0, 0);
    vec3.set(contact.linearVelocity, 0, 0, 0);
    vec3.set(contact.contactNormal, 0, 0, 0);
    vec3.set(contact.surfaceNormal, 0, 0, 0);
    contact.distance = 0;
    contact.fraction = 0;
    contact.bodyId = INVALID_BODY_ID;
    contact.subShapeId = 0;
    contact.materialId = -1;
    contact.motionType = MotionType.STATIC;
    contact.isSensor = false;
    contact.hadCollision = false;
    contact.wasDiscarded = false;
    contact.canPushCharacter = false;
    contact.canReceiveImpulses = true;
}

/** creates an empty CharacterConstraint with default values */
function createEmptyCharacterConstraint(): CharacterConstraint {
    return {
        contact: null!, // will be set when acquired
        toi: 0,
        projectedVelocity: 0,
        linearVelocity: vec3.create(),
        planeNormal: vec3.create(),
        planeDistance: 0,
        isSteepSlope: false,
    };
}

/**
 * When cosMaxSlopeAngle is greater than this value, slope checking is disabled.
 * All contacts are considered supported regardless of angle.
 */
const NO_MAX_SLOPE_ANGLE = 0.9999;

/**
 * Checks if a slope is too steep to walk on.
 * @param character the character controller
 * @param normal the surface normal to test
 * @returns true if the slope is steeper than maxSlopeAngle
 */
export function isSlopeTooSteep(character: KCC, normal: Vec3): boolean {
    // If slope checking is disabled, no slope is too steep
    if (character.cosMaxSlopeAngle > NO_MAX_SLOPE_ANGLE) {
        return false;
    }
    // Slope is too steep if the dot product of normal and up is less than cos(maxSlopeAngle)
    // A flat surface has dot(normal, up) = 1, a vertical wall has dot = 0
    const dot = vec3.dot(normal, character.up);
    return dot < character.cosMaxSlopeAngle;
}

/**
 * Gets the world-space center of mass transform for the character.
 * This accounts for the shape offset.
 *
 * @param character the character controller
 * @param outPosition output for the center of mass position
 * @param outQuaternion output for the orientation
 */
export function getCenterOfMassTransform(character: KCC, outPosition: Vec3, outQuaternion: Quat): void {
    // rotate shapeOffset by character quaternion and add to position
    vec3.transformQuat(outPosition, character.shapeOffset, character.quaternion);
    vec3.add(outPosition, outPosition, character.position);

    // quaternion is unchanged
    quat.copy(outQuaternion, character.quaternion);
}

/**
 * Checks if the character is supported (on ground or on steep ground that provides support).
 * @param character the character controller
 * @returns true if the character is on ground or on steep ground
 */
export function isSupported(character: KCC): boolean {
    return character.ground.state === GroundState.ON_GROUND || character.ground.state === GroundState.ON_STEEP_GROUND;
}

const _groundVel_axis = /* @__PURE__ */ vec3.create();
const _groundVel_rotQuat = /* @__PURE__ */ quat.create();
const _groundVel_offset = /* @__PURE__ */ vec3.create();
const _groundVel_newPosition = /* @__PURE__ */ vec3.create();
const _groundVel_result = /* @__PURE__ */ vec3.create();

/**
 * Calculates the ground velocity at the character's position for a kinematic body.
 * This accounts for both linear velocity and rotational velocity around the body's center of mass.
 *
 * 1. If angular velocity is negligible, just return linear velocity
 * 2. Calculate rotation quaternion from angular velocity over deltaTime
 * 3. Rotate character position around body center of mass
 * 4. Calculate effective velocity from position change
 *
 * @param character the character controller
 * @param bodyCenterOfMass the body's center of mass position
 * @param linearVelocity the body's linear velocity
 * @param angularVelocity the body's angular velocity
 * @param deltaTime the time step
 * @returns the effective ground velocity at the character's position
 */
function calculateCharacterGroundVelocity(
    character: KCC,
    bodyCenterOfMass: Vec3,
    linearVelocity: Vec3,
    angularVelocity: Vec3,
    deltaTime: number,
): Vec3 {
    // get angular velocity magnitude
    const angularVelLenSq = vec3.squaredLength(angularVelocity);
    if (angularVelLenSq < 1e-12) {
        // no significant rotation - just return linear velocity
        vec3.copy(_groundVel_result, linearVelocity);
        return _groundVel_result;
    }

    const angularVelLen = Math.sqrt(angularVelLenSq);

    // calculate rotation axis (normalized angular velocity)
    vec3.scale(_groundVel_axis, angularVelocity, 1 / angularVelLen);

    // calculate rotation angle for this time step
    const angle = angularVelLen * deltaTime;

    // create rotation quaternion
    quat.setAxisAngle(_groundVel_rotQuat, _groundVel_axis, angle);

    // calculate offset from body center of mass to character position
    vec3.sub(_groundVel_offset, character.position, bodyCenterOfMass);

    // rotate the offset by the rotation quaternion
    vec3.transformQuat(_groundVel_newPosition, _groundVel_offset, _groundVel_rotQuat);

    // add back the center of mass to get new position
    vec3.add(_groundVel_newPosition, _groundVel_newPosition, bodyCenterOfMass);

    // calculate velocity: linearVelocity + (newPosition - characterPosition) / deltaTime
    if (deltaTime > 1e-8) {
        vec3.sub(_groundVel_result, _groundVel_newPosition, character.position);
        vec3.scale(_groundVel_result, _groundVel_result, 1 / deltaTime);
        vec3.add(_groundVel_result, _groundVel_result, linearVelocity);
    } else {
        vec3.copy(_groundVel_result, linearVelocity);
    }

    return _groundVel_result;
}

const _characterCollideSettings: CollideShapeSettings = /* @__PURE__ */ createDefaultCollideShapeSettings();
const _characterCastSettings: CastShapeSettings = /* @__PURE__ */ createDefaultCastShapeSettings();

const _getContacts_shapePos = /* @__PURE__ */ vec3.create();
const _getContacts_paddingOffset = /* @__PURE__ */ vec3.create();

const _paddingCorrection_characterSupportPool = /* @__PURE__ */ createShapeSupportPool();
const _paddingCorrection_polygonSupport = /* @__PURE__ */ createPolygonSupport();
const _paddingCorrection_addConvexRadius = /* @__PURE__ */ createAddConvexRadiusSupport();
const _paddingCorrection_face = /* @__PURE__ */ createFace();
const _paddingCorrection_gjkResult = /* @__PURE__ */ createGjkCastShapeResult();
const _paddingCorrection_negativeNormal = /* @__PURE__ */ vec3.create();
const _paddingCorrection_scale = /* @__PURE__ */ vec3.fromValues(1, 1, 1);
const _paddingCorrection_fractionWrapper = { value: 0 };
const _correctFraction_tempPos = /* @__PURE__ */ vec3.create();
const _correctFraction_tempQuat = /* @__PURE__ */ quat.create();

const _contactVelocity_r = /* @__PURE__ */ vec3.create();
const _contactVelocity_angularComponent = /* @__PURE__ */ vec3.create();
const _contactVelocity_adjustedLinear = /* @__PURE__ */ vec3.create();
const _contactVelocity_adjustedAngular = /* @__PURE__ */ vec3.create();
const _contactVelocity_staticLinear = /* @__PURE__ */ vec3.create();
const _contactVelocity_staticAngular = /* @__PURE__ */ vec3.create();

const _surfaceNormal_temp = /* @__PURE__ */ vec3.create();

/** character collide shape collector - collects hits into contacts array */
const characterCollideCollector = {
    bodyIdB: -1,
    earlyOutFraction: Number.MAX_VALUE,
    contacts: null! as CharacterContact[],
    maxHits: 256,
    hitCount: 0,
    world: null! as World,
    listener: undefined as CharacterListener | undefined,
    character: null! as KCC,
    maxHitsExceeded: false,
    hitReductionCosMaxAngle: -1,

    addHit(hit: CollideShapeHit): void {
        // if we've reached max hits, try to merge similar contacts before giving up
        if (this.hitCount >= this.maxHits) {
            this.maxHitsExceeded = true;

            // try to merge similar contacts if enabled
            if (this.hitReductionCosMaxAngle > -1) {
                const contacts = this.contacts;

                // loop backwards through contacts to find similar ones
                for (let i = contacts.length - 1; i >= 0; i--) {
                    const contactI = contacts[i];

                    for (let j = i - 1; j >= 0; j--) {
                        const contactJ = contacts[j];
                        // check if same body (including subShapeId) and normals are similar
                        if (
                            contactI.bodyId === contactJ.bodyId &&
                            contactI.subShapeId === contactJ.subShapeId &&
                            vec3.dot(contactI.contactNormal, contactJ.contactNormal) > this.hitReductionCosMaxAngle
                        ) {
                            // remove the contact with bigger distance (less penetrating)
                            if (contactI.distance > contactJ.distance) {
                                releaseContact(contacts, i);
                                this.hitCount--;

                                // break - can't continue with i since we just removed it
                                break;
                            } else {
                                releaseContact(contacts, j);
                                this.hitCount--;

                                // continue checking other contacts against i
                            }
                        }
                    }

                    // if we freed up space, we can add the new contact
                    if (this.hitCount < this.maxHits) {
                        break;
                    }
                }
            }

            // if still at max hits after reduction, give up
            if (this.hitCount >= this.maxHits) {
                return;
            }
        }

        const contact = acquireContact(this.contacts);

        // position is the contact point on shape B (the world body)
        vec3.copy(contact.position, hit.pointB);

        // normal points toward character (from B to A)
        // penetrationAxis points from A to B, so negate it
        vec3.normalize(contact.contactNormal, hit.penetrationAxis);
        vec3.negate(contact.contactNormal, contact.contactNormal);

        // get actual geometric surface normal
        // this is more stable than contact normal from gjk / epa
        const body = rigidBody.get(this.world, hit.bodyIdB);

        if (!body) {
            // body not found - shouldn't happen, skip this contact
            return;
        }

        rigidBody.getSurfaceNormal(_surfaceNormal_temp, body, contact.position, hit.subShapeIdB);
        vec3.copy(contact.surfaceNormal, _surfaceNormal_temp);

        // flip surface normal if hitting back face
        if (vec3.dot(contact.contactNormal, contact.surfaceNormal) < 0) {
            vec3.negate(contact.surfaceNormal, contact.surfaceNormal);
        }

        // replace surface normal with contact normal if contact normal points more upward,
        // this handles edges/corners better
        const contactDotUp = vec3.dot(contact.contactNormal, this.character.up);
        const surfaceDotUp = vec3.dot(contact.surfaceNormal, this.character.up);
        if (contactDotUp > surfaceDotUp) {
            vec3.copy(contact.surfaceNormal, contact.contactNormal);
        }

        // distance is negative penetration (negative = penetrating)
        contact.distance = -hit.penetration;
        contact.fraction = 0; // static collision, no sweep fraction

        contact.bodyId = hit.bodyIdB;
        contact.subShapeId = hit.subShapeIdB;
        contact.materialId = hit.materialIdB;

        // store motion type, velocity, and sensor flag
        if (body) {
            contact.motionType = body.motionType;
            contact.isSensor = body.sensor;

            // get body velocity (zero for static, real velocity for dynamic/kinematic)
            let linearVelocity: Vec3;
            let angularVelocity: Vec3;
            if (body.motionType === MotionType.DYNAMIC || body.motionType === MotionType.KINEMATIC) {
                linearVelocity = body.motionProperties.linearVelocity;
                angularVelocity = body.motionProperties.angularVelocity;
            } else {
                // static body - start with zero velocity
                vec3.zero(_contactVelocity_staticLinear);
                vec3.zero(_contactVelocity_staticAngular);
                linearVelocity = _contactVelocity_staticLinear;
                angularVelocity = _contactVelocity_staticAngular;
            }

            // allow listener to adjust the body's velocity (e.g., for conveyor belts on static bodies)
            if (this.listener?.onAdjustBodyVelocity) {
                vec3.copy(_contactVelocity_adjustedLinear, linearVelocity);
                vec3.copy(_contactVelocity_adjustedAngular, angularVelocity);
                this.listener.onAdjustBodyVelocity(
                    this.character,
                    body,
                    _contactVelocity_adjustedLinear,
                    _contactVelocity_adjustedAngular,
                );
                linearVelocity = _contactVelocity_adjustedLinear;
                angularVelocity = _contactVelocity_adjustedAngular;
            }

            // calculate point velocity: v_point = v_linear + ω × (position - centerOfMass)
            vec3.sub(_contactVelocity_r, contact.position, body.centerOfMassPosition);
            vec3.cross(_contactVelocity_angularComponent, angularVelocity, _contactVelocity_r);
            vec3.add(contact.linearVelocity, linearVelocity, _contactVelocity_angularComponent);
        } else {
            contact.motionType = MotionType.STATIC;
            contact.isSensor = false;
            vec3.zero(contact.linearVelocity);
        }

        contact.hadCollision = false;
        contact.wasDiscarded = false;
        contact.canPushCharacter = true;

        this.hitCount++;
    },

    addMiss(): void {
        // no-op
    },

    shouldEarlyOut(): boolean {
        return this.hitCount >= this.maxHits;
    },

    reset(): void {
        this.bodyIdB = -1;
        this.contacts = null!;
        this.maxHitsExceeded = false;
        this.maxHits = 256;
        this.hitCount = 0;
        this.world = null!;
        this.listener = undefined;
        this.character = null!;
    },
};

/** character cast shape collector - finds earliest hit */
const characterCastCollector = /* @__PURE__ */ (() => ({
    bodyIdB: -1,
    earlyOutFraction: 1.0,
    outContact: null! as CharacterContact,
    hasHit: false,
    world: null! as World,
    ignoredContacts: undefined as CharacterContact[] | undefined,
    listener: undefined as CharacterListener | undefined,
    character: null! as KCC,
    displacement: vec3.create() as Vec3, // Store displacement for validation checks

    addHit(hit: CastShapeHit): void {
        if (hit.status !== CastShapeStatus.COLLIDING) return;

        // only accept hits earlier than current best
        if (hit.fraction >= this.earlyOutFraction) return;

        // ignore collisions at fraction = 0
        // these are already-penetrating contacts handled by collide collector
        if (hit.fraction <= 0) return;

        // ignore penetrations we're moving away from
        if (vec3.dot(hit.normal, this.displacement) >= 0) return;

        // test if this contact should be ignored
        if (this.ignoredContacts) {
            for (const ignored of this.ignoredContacts) {
                if (ignored.bodyId === hit.bodyIdB && ignored.subShapeId === hit.subShapeIdB) {
                    return;
                }
            }
        }

        const contact = this.outContact;

        // position is the contact point on shape B
        vec3.copy(contact.position, hit.pointB);

        // normal points toward character
        vec3.copy(contact.contactNormal, hit.normal);

        // get actual geometric surface normal
        // this is more stable than contact normal from GJK/EPA
        const body = rigidBody.get(this.world, hit.bodyIdB);

        if (!body) {
            // body not found - shouldn't happen, skip this contact
            return;
        }

        // sensors don't affect character movement, only detection
        if (body.sensor) {
            return;
        }

        rigidBody.getSurfaceNormal(_surfaceNormal_temp, body, contact.position, hit.subShapeIdB);
        vec3.copy(contact.surfaceNormal, _surfaceNormal_temp);

        // flip surface normal if hitting back face
        if (vec3.dot(contact.contactNormal, contact.surfaceNormal) < 0) {
            vec3.negate(contact.surfaceNormal, contact.surfaceNormal);
        }

        // replace surface normal with contact normal if contact normal points more upward,
        // this handles edges/corners better
        const contactDotUp = vec3.dot(contact.contactNormal, this.character.up);
        const surfaceDotUp = vec3.dot(contact.surfaceNormal, this.character.up);
        if (contactDotUp > surfaceDotUp) {
            vec3.copy(contact.surfaceNormal, contact.contactNormal);
        }

        // distance for sweep hit
        contact.distance = -hit.penetrationDepth;
        contact.fraction = hit.fraction;

        contact.bodyId = hit.bodyIdB;
        contact.subShapeId = hit.subShapeIdB;
        contact.materialId = hit.materialIdB;

        // store motion type, velocity, and sensor flag
        contact.motionType = body.motionType;
        contact.isSensor = body.sensor;

        // get body velocity (zero for static, real velocity for dynamic/kinematic)
        let linearVelocity: Vec3;
        let angularVelocity: Vec3;
        if (body.motionType === MotionType.DYNAMIC || body.motionType === MotionType.KINEMATIC) {
            linearVelocity = body.motionProperties.linearVelocity;
            angularVelocity = body.motionProperties.angularVelocity;
        } else {
            // static body - start with zero velocity
            vec3.zero(_contactVelocity_staticLinear);
            vec3.zero(_contactVelocity_staticAngular);
            linearVelocity = _contactVelocity_staticLinear;
            angularVelocity = _contactVelocity_staticAngular;
        }

        // allow listener to adjust the body's velocity (e.g., for conveyor belts on static bodies)
        if (this.listener?.onAdjustBodyVelocity) {
            vec3.copy(_contactVelocity_adjustedLinear, linearVelocity);
            vec3.copy(_contactVelocity_adjustedAngular, angularVelocity);
            this.listener.onAdjustBodyVelocity(
                this.character,
                body,
                _contactVelocity_adjustedLinear,
                _contactVelocity_adjustedAngular,
            );
            linearVelocity = _contactVelocity_adjustedLinear;
            angularVelocity = _contactVelocity_adjustedAngular;
        }

        // calculate point velocity: v_point = v_linear + ω × (position - centerOfMass)
        vec3.sub(_contactVelocity_r, contact.position, body.centerOfMassPosition);
        vec3.cross(_contactVelocity_angularComponent, angularVelocity, _contactVelocity_r);
        vec3.add(contact.linearVelocity, linearVelocity, _contactVelocity_angularComponent);

        contact.hadCollision = true;
        contact.wasDiscarded = false;
        contact.canPushCharacter = true;

        // only accept contacts that will cause significant penetration AND are validated
        // predictedPenetration = current distance + how much we'll move into the surface
        // if predicted penetration doesn't exceed threshold, contact is ignored (grazing contact)
        const predictedPenetration = contact.distance + vec3.dot(contact.contactNormal, this.displacement);
        if (
            predictedPenetration < -this.character.collisionTolerance &&
            validateContact(this.world, this.character, contact, this.listener)
        ) {
            // Contact will cause significant penetration and listener approved it
            this.hasHit = true;
            this.earlyOutFraction = hit.fraction;
        }
        // otherwise: contact is NOT accepted
    },

    addMiss(): void {
        // no-op
    },

    shouldEarlyOut(): boolean {
        // early out if we found a hit at fraction 0 (already penetrating)
        return this.hasHit && this.earlyOutFraction <= 0;
    },

    reset(): void {
        this.bodyIdB = -1;
        this.earlyOutFraction = 1.0;
        this.outContact = null!;
        this.hasHit = false;
        this.world = null!;
        this.ignoredContacts = undefined;
        this.listener = undefined!;
        this.character = null!;
        vec3.zero(this.displacement);
    },
}))();

/**
 * Comparison function for deterministic contact sorting.
 * Primary: sort by bodyId (lower first)
 * Secondary: sort by subShapeId (lower first)
 */
function compareContactsForDeterminism(a: CharacterContact, b: CharacterContact): number {
    if (a.bodyId !== b.bodyId) {
        return a.bodyId - b.bodyId;
    }
    return a.subShapeId - b.subShapeId;
}

/** sorts contacts for determinism, ensures identical ordering across frames/runs */
function sortContacts(contacts: CharacterContact[]): void {
    contacts.sort(compareContactsForDeterminism);
}

const SCALE_V1 = /* @__PURE__ */ vec3.fromValues(1, 1, 1);

/**
 * Gets contacts at the given position using broadphase + narrowphase collision detection.
 * Includes deterministic sorting and hit reduction.
 *
 * Contacts are acquired from the module-scope free list.
 *
 * @param world the physics world
 * @param character the character controller
 * @param position position to test
 * @param movementDirection normalized movement direction (for active edge handling)
 * @param filter collision filter
 * @param contacts array to write contacts to (will be cleared first)
 */
function getContactsAtPosition(
    world: World,
    character: KCC,
    position: Vec3,
    movementDirection: Vec3,
    filter: Filter,
    listener: CharacterListener | undefined,
    contacts: CharacterContact[],
): void {
    // release any previously active contacts back to free list
    releaseAllContacts(contacts);

    // calculate shape world position (position + rotated shapeOffset + characterPadding * up)
    vec3.transformQuat(_getContacts_shapePos, character.shapeOffset, character.quaternion);
    vec3.add(_getContacts_shapePos, _getContacts_shapePos, position);

    // Add character padding in the up direction
    vec3.scale(_getContacts_paddingOffset, character.up, character.characterPadding);
    vec3.add(_getContacts_shapePos, _getContacts_shapePos, _getContacts_paddingOffset);

    // set max separation distance for predictive contacts
    _characterCollideSettings.maxSeparationDistance = character.predictiveContactDistance + character.characterPadding;

    // set active edge movement direction for triangle mesh edge handling
    vec3.copy(_characterCollideSettings.activeEdgeMovementDirection, movementDirection);

    // set active edge mode (will be overridden to false if enhanced edge removal is used)
    _characterCollideSettings.collideOnlyWithActiveEdges = true;

    // setup collector
    characterCollideCollector.contacts = contacts;
    characterCollideCollector.world = world;
    characterCollideCollector.maxHits = character.maxNumHits;
    characterCollideCollector.hitCount = 0;
    characterCollideCollector.listener = listener;
    characterCollideCollector.character = character;
    characterCollideCollector.hitReductionCosMaxAngle = character.hitReductionCosMaxAngle;
    characterCollideCollector.maxHitsExceeded = false;

    // setup chained filter to ignore character's inner body
    const originalBodyFilter = filter.bodyFilter;
    ignoreSingleBodyChainedBodyFilterState.bodyId = character.innerRigidBodyId;
    ignoreSingleBodyChainedBodyFilterState.innerBodyFilter = originalBodyFilter;
    filter.bodyFilter = ignoreSingleBodyChainedBodyFilter;

    // query broadphase and perform narrowphase collision
    if (character.enhancedInternalEdgeRemoval) {
        query.collideShapeWithInternalEdgeRemoval(
            world,
            characterCollideCollector,
            _characterCollideSettings,
            character.shape,
            _getContacts_shapePos,
            character.quaternion,
            SCALE_V1,
            filter,
        );
    } else {
        query.collideShape(
            world,
            characterCollideCollector,
            _characterCollideSettings,
            character.shape,
            _getContacts_shapePos,
            character.quaternion,
            SCALE_V1,
            filter,
        );
    }

    // restore original filter
    filter.bodyFilter = originalBodyFilter;
    ignoreSingleBodyChainedBodyFilterState.bodyId = INVALID_BODY_ID;
    ignoreSingleBodyChainedBodyFilterState.innerBodyFilter = undefined;

    // reset collector
    characterCollideCollector.reset();

    // sort active contacts for determinism
    sortContacts(contacts);

    // reduce distance to contact by padding to ensure we stay away from objects
    // this makes collision detection cheaper and ensures proper spacing
    for (let i = 0; i < contacts.length; i++) {
        contacts[i].distance -= character.characterPadding;
    }
}

/**
 * Recursively corrects the fraction for character padding by unwrapping decorator shapes.
 * Accumulates transformations through decorator shape hierarchy before performing GJK cast.
 * @param supportPool shape support pool for support function creation
 * @param shape the character shape to cast (may be decorator)
 * @param position start position (accumulated through recursion)
 * @param quaternion start rotation (accumulated through recursion)
 * @param displacement movement vector
 * @param scale accumulated scale
 * @param polygon the inflated polygon to cast against
 * @param inOutFraction object containing the fraction to correct (modified in place)
 * @param collisionTolerance tolerance for GJK
 * @returns true if correction was successful
 */
function correctFractionForCharacterPadding(
    supportPool: ShapeSupportPool,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    displacement: Vec3,
    scale: Vec3,
    polygon: AddConvexRadiusSupport,
    inOutFraction: { value: number },
    collisionTolerance: number,
): boolean {
    // handle transformed decorator shape
    if (shape.type === ShapeType.TRANSFORMED) {
        // accumulate the transformation
        vec3.transformQuat(_correctFraction_tempPos, shape.position, quaternion);
        vec3.add(_correctFraction_tempPos, position, _correctFraction_tempPos);

        quat.multiply(_correctFraction_tempQuat, quaternion, shape.quaternion);

        // recurse with accumulated transform
        return correctFractionForCharacterPadding(
            supportPool,
            shape.shape,
            _correctFraction_tempPos,
            _correctFraction_tempQuat,
            displacement,
            scale,
            polygon,
            inOutFraction,
            collisionTolerance,
        );
    }

    // at the convex level, perform the GJK cast
    const characterSupport = getShapeSupportFunction(supportPool, shape, SupportFunctionMode.INCLUDE_CONVEX_RADIUS, scale);

    // cast the shape against the polygon
    gjkCastShape(
        _paddingCorrection_gjkResult,
        position,
        quaternion,
        characterSupport,
        polygon,
        displacement,
        collisionTolerance,
        characterSupport.convexRadius,
        polygon.convexRadius,
        inOutFraction.value,
    );

    if (_paddingCorrection_gjkResult.hit) {
        inOutFraction.value = _paddingCorrection_gjkResult.lambda;
        return true;
    }

    return false;
}

/**
 * Sweep test to verify movement path. Returns earliest hit along displacement.
 * @param out output contact (only valid if function returns true)
 * @param world the physics world
 * @param character the character controller
 * @param position start position
 * @param displacement movement displacement vector
 * @param filter collision filter
 * @param ignoredContacts optional array of contacts to ignore
 * @returns true if contact found, false if path is clear
 */
function getFirstContactForSweep(
    out: CharacterContact,
    world: World,
    character: KCC,
    position: Vec3,
    displacement: Vec3,
    filter: Filter,
    listener: CharacterListener | undefined,
    ignoredContacts: CharacterContact[] | undefined,
): boolean {
    // too small displacement, skip checking
    const displacementLenSq = vec3.squaredLength(displacement);
    if (displacementLenSq < 1.0e-8) return false;

    // reset contact
    resetContact(out);

    // calculate shape world position (position + rotated shapeOffset + characterPadding * up)
    vec3.transformQuat(_getContacts_shapePos, character.shapeOffset, character.quaternion);
    vec3.add(_getContacts_shapePos, _getContacts_shapePos, position);

    // Add character padding in the up direction
    vec3.scale(_getContacts_paddingOffset, character.up, character.characterPadding);
    vec3.add(_getContacts_shapePos, _getContacts_shapePos, _getContacts_paddingOffset);

    // calculate how much extra fraction we need to add to the cast to account for the character padding
    const displacementLen = Math.sqrt(displacementLenSq);
    const characterPaddingFraction = character.characterPadding / displacementLen;

    // setup cast settings
    _characterCastSettings.collisionTolerance = character.collisionTolerance;
    _characterCastSettings.collideWithBackfaces = character.backFaceMode === BackFaceMode.COLLIDE;
    _characterCastSettings.collideOnlyWithActiveEdges = true;
    _characterCastSettings.useShrunkenShapeAndConvexRadius = true;
    _characterCastSettings.returnDeepestPoint = false;

    // setup collector
    characterCastCollector.outContact = out;
    characterCastCollector.hasHit = false;
    characterCastCollector.earlyOutFraction = 1.0 + characterPaddingFraction; // allow cast beyond 1.0 for padding correction
    characterCastCollector.world = world;
    characterCastCollector.ignoredContacts = ignoredContacts;
    characterCastCollector.listener = listener;
    characterCastCollector.character = character;
    vec3.copy(characterCastCollector.displacement, displacement); // Store displacement for validation checks

    // setup chained filter to ignore character's inner body
    const originalBodyFilter = filter.bodyFilter;
    ignoreSingleBodyChainedBodyFilterState.bodyId = character.innerRigidBodyId;
    ignoreSingleBodyChainedBodyFilterState.innerBodyFilter = originalBodyFilter;
    filter.bodyFilter = ignoreSingleBodyChainedBodyFilter;

    // cast shape through broadphase
    const scale = vec3.fromValues(1, 1, 1);
    query.castShape(
        world,
        characterCastCollector,
        _characterCastSettings,
        character.shape,
        _getContacts_shapePos,
        character.quaternion,
        scale,
        displacement,
        filter,
    );

    // restore original filter
    filter.bodyFilter = originalBodyFilter;
    ignoreSingleBodyChainedBodyFilterState.bodyId = INVALID_BODY_ID;
    ignoreSingleBodyChainedBodyFilterState.innerBodyFilter = undefined;

    const hasHit = characterCastCollector.hasHit;

    // reset collector
    characterCastCollector.reset();

    if (!hasHit) {
        return false;
    }

    // apply advanced character padding correction
    // get the body we hit to fetch its transformed shape
    const hitBody = rigidBody.get(world, out.bodyId);

    let corrected = false;
    if (hitBody) {
        // compute supporting face on the hit body
        // direction: negative contact normal (pointing into the body)
        vec3.negate(_paddingCorrection_negativeNormal, out.contactNormal);

        getShapeSupportingFace(
            _paddingCorrection_face,
            hitBody.shape,
            out.subShapeId,
            _paddingCorrection_negativeNormal,
            hitBody.position,
            hitBody.quaternion,
            _paddingCorrection_scale,
        );

        // only use advanced correction if we have a face with 2+ vertices
        if (_paddingCorrection_face.numVertices >= 2) {
            // create polygon support from face
            setPolygonSupport(_paddingCorrection_polygonSupport, _paddingCorrection_face);

            // inflate polygon by character padding
            setAddConvexRadiusSupport(
                _paddingCorrection_addConvexRadius,
                character.characterPadding,
                _paddingCorrection_polygonSupport,
            );

            // use recursive function to handle decorator shapes and perform GJK cast
            _paddingCorrection_fractionWrapper.value = out.fraction + characterPaddingFraction;
            corrected = correctFractionForCharacterPadding(
                _paddingCorrection_characterSupportPool,
                character.shape,
                _getContacts_shapePos,
                character.quaternion,
                displacement,
                _paddingCorrection_scale,
                _paddingCorrection_addConvexRadius,
                _paddingCorrection_fractionWrapper,
                character.collisionTolerance,
            );

            if (corrected) {
                out.fraction = _paddingCorrection_fractionWrapper.value;
            }
        }
    }

    // fallback: simple fraction adjustment
    if (!corrected) {
        out.fraction = Math.max(0, out.fraction - characterPaddingFraction);
    }

    // ensure fraction never exceeds 1
    out.fraction = Math.min(out.fraction, 1);

    return true;
}

/**
 * Removes contacts with opposing penetration normals.
 * Prevents character from being stuck between two penetrating surfaces.
 *
 * Only uses this algorithm if penetrating further than minRequiredPenetration
 * (1.25 * characterPadding) to account for numerical precision issues.
 * For each pair of penetrating contacts, if their normals point in opposite
 * directions (dot product < 0), discard the one with smaller penetration depth.
 *
 * @param contacts array of contacts to process (modified in place)
 * @param characterPadding character padding value for threshold calculation
 * @param outIgnored array to receive ignored contacts
 */
function removeConflictingContacts(contacts: CharacterContact[], characterPadding: number, outIgnored: CharacterContact[]): void {
    outIgnored.length = 0;

    // only use this algorithm if penetrating further than this threshold
    // (due to numerical precision we can always penetrate a little)
    // we need to account for padding that was removed from contact distances
    const minRequiredPenetration = 1.25 * characterPadding;

    const n = contacts.length;

    for (let i = 0; i < n; i++) {
        const contactA = contacts[i];
        if (contactA.wasDiscarded) continue;
        // only check penetrating contacts beyond threshold
        if (contactA.distance > -minRequiredPenetration) continue;

        for (let j = i + 1; j < n; j++) {
            const contactB = contacts[j];
            if (contactB.wasDiscarded) continue;
            if (contactB.distance > -minRequiredPenetration) continue;

            // check if normals oppose each other
            const dot = vec3.dot(contactA.contactNormal, contactB.contactNormal);
            if (dot < 0) {
                // normals oppose - discard the one with smaller penetration
                // (less negative distance = smaller penetration)
                if (contactA.distance > contactB.distance) {
                    // a has smaller penetration, discard it
                    contactA.wasDiscarded = true;
                    outIgnored.push(contactA);
                    break; // move to next i
                } else {
                    // b has smaller penetration, discard it
                    contactB.wasDiscarded = true;
                    outIgnored.push(contactB);
                }
            }
        }
    }
}

const _determineConstraints_contactVelocity = /* @__PURE__ */ vec3.create();
const _determineConstraints_penetrationRecovery = /* @__PURE__ */ vec3.create();
const _determineConstraints_horizontalNormal = /* @__PURE__ */ vec3.create();

/**
 * Converts contacts to movement constraints.
 *
 * For each contact:
 * 1. Create a plane constraint from contact normal
 * 2. Add penetration recovery velocity if penetrating
 * 3. For steep slopes, create additional vertical wall constraint
 *
 * @param character the character controller
 * @param contacts array of contacts to convert
 * @param deltaTime time step
 * @param constraints array to write constraints to (will be cleared first)
 */
function determineConstraints(
    character: KCC,
    contacts: CharacterContact[],
    deltaTime: number,
    constraints: CharacterConstraint[],
): void {
    // release any previously active constraints back to free list
    releaseAllConstraints(constraints);

    const invDeltaTime = deltaTime > 0 ? 1 / deltaTime : 0;

    for (const contact of contacts) {
        if (contact.wasDiscarded) continue;

        // calculate contact velocity with penetration recovery
        const contactVelocity = vec3.copy(_determineConstraints_contactVelocity, contact.linearVelocity);

        // penetrating contact: add a contact velocity that pushes the character out at the desired speed
        if (contact.distance < 0) {
            const penetrationRecovery = vec3.scale(
                _determineConstraints_penetrationRecovery,
                contact.contactNormal,
                contact.distance * character.penetrationRecoverySpeed * invDeltaTime,
            );
            vec3.sub(contactVelocity, contactVelocity, penetrationRecovery);
        }

        // create main constraint
        const constraint = acquireConstraint(constraints);
        constraint.contact = contact;
        constraint.toi = 0;
        constraint.projectedVelocity = 0;
        vec3.copy(constraint.linearVelocity, contactVelocity);
        vec3.copy(constraint.planeNormal, contact.contactNormal);
        constraint.planeDistance = contact.distance;
        constraint.isSteepSlope = false;

        // check for steep slope - create additional vertical wall constraint
        if (isSlopeTooSteep(character, contact.surfaceNormal)) {
            const dotUp = vec3.dot(contact.contactNormal, character.up);

            // only create vertical wall if normal has upward component
            if (dotUp > 1e-3) {
                constraint.isSteepSlope = true;

                // calculate horizontal component of normal (remove up component)
                // horizontalNormal = normalize(contactNormal - up * dotUp)
                vec3.scaleAndAdd(_determineConstraints_horizontalNormal, contact.contactNormal, character.up, -dotUp);
                const lenSq = vec3.squaredLength(_determineConstraints_horizontalNormal);

                if (lenSq > 1e-6) {
                    vec3.normalize(_determineConstraints_horizontalNormal, _determineConstraints_horizontalNormal);

                    // create vertical wall constraint
                    const verticalConstraint = acquireConstraint(constraints);
                    verticalConstraint.contact = contact;
                    verticalConstraint.toi = 0;
                    verticalConstraint.projectedVelocity = 0;

                    // project contact velocity onto horizontal normal so both planes push at equal rate
                    const contactVelDotHoriz = vec3.dot(contactVelocity, _determineConstraints_horizontalNormal);
                    vec3.scale(verticalConstraint.linearVelocity, _determineConstraints_horizontalNormal, contactVelDotHoriz);

                    vec3.copy(verticalConstraint.planeNormal, _determineConstraints_horizontalNormal);

                    // calculate distance to travel horizontally to hit the contact plane
                    const normalDotContact = vec3.dot(_determineConstraints_horizontalNormal, contact.contactNormal);
                    if (Math.abs(normalDotContact) > 1e-6) {
                        verticalConstraint.planeDistance = contact.distance / normalDotContact;
                    } else {
                        verticalConstraint.planeDistance = contact.distance;
                    }
                    verticalConstraint.isSteepSlope = true;
                }
            }
        }
    }
}

const _applyImpulse_worldImpulse = /* @__PURE__ */ vec3.create();
const _applyImpulse_downComponent = /* @__PURE__ */ vec3.create();
const _applyImpulse_r = /* @__PURE__ */ vec3.create(); // vector from center of mass to contact point
const _applyImpulse_jacobian = /* @__PURE__ */ vec3.create(); // r × normal
const _applyImpulse_invIJ = /* @__PURE__ */ vec3.create(); // I⁻¹ * jacobian (world space)

const IMPULSE_DAMPING_FACTOR = 0.9;
const IMPULSE_PENETRATION_RESOLUTION_FACTOR = 0.4;

/**
 * Applies impulse to a dynamic body from character contact.
 * Uses proper rigid body mechanics with moment of inertia at contact point.
 *
 * 1. calculate relative velocity and delta velocity needed
 * 2. calculate effective mass at contact point (includes rotational effects)
 * 3. calculate impulse P = M_eff * deltaV
 * 4. clamp impulse by character max strength
 * 5. cancel downward component to prevent double-counting gravity
 * 6. apply impulse to body at contact point (linear + angular)
 *
 * @param world the physics world
 * @param character the character controller
 * @param contact the contact to apply impulse for
 * @param characterVelocity character's current velocity
 * @param deltaTime time step
 */
function applyImpulseToBody(
    world: World,
    character: KCC,
    contact: CharacterContact,
    characterVelocity: Vec3,
    deltaTime: number,
): void {
    // only apply impulses to dynamic bodies
    if (contact.motionType !== MotionType.DYNAMIC) return;

    // don't apply impulses to sensors
    if (contact.isSensor) return;

    // don't apply impulses if contact settings disable it
    if (!contact.canReceiveImpulses) return;

    const body = rigidBody.get(world, contact.bodyId);
    if (!body) return;

    const mp = body.motionProperties;
    const invMass = mp.invMass;

    // exit if static or infinite mass
    if (invMass <= 0) return;

    // calculate relative velocity (character - contact point velocity)
    // relativeVelocity = characterVelocity - contactLinearVelocity
    const relVelX = characterVelocity[0] - contact.linearVelocity[0];
    const relVelY = characterVelocity[1] - contact.linearVelocity[1];
    const relVelZ = characterVelocity[2] - contact.linearVelocity[2];

    // project relative velocity onto contact normal
    const projectedVelocity =
        relVelX * contact.contactNormal[0] + relVelY * contact.contactNormal[1] + relVelZ * contact.contactNormal[2];

    // calculate delta velocity needed
    // deltaV = -projectedVelocity * damping - penetrationCorrection
    let deltaVelocity = -projectedVelocity * IMPULSE_DAMPING_FACTOR;

    // add penetration correction if penetrating
    if (contact.distance < 0 && deltaTime > 0) {
        deltaVelocity -= (Math.min(contact.distance, 0) * IMPULSE_PENETRATION_RESOLUTION_FACTOR) / deltaTime;
    }

    // don't apply impulses if we're separating
    if (deltaVelocity < 0) return;

    // calculate vector from body center of mass to contact point
    // r = contactPosition - bodyCenterOfMass
    vec3.sub(_applyImpulse_r, contact.position, body.centerOfMassPosition);

    // calculate jacobian: r × normal (for angular contribution)
    vec3.cross(_applyImpulse_jacobian, _applyImpulse_r, contact.contactNormal);

    // calculate I⁻¹ * jacobian using proper world-space inverse inertia
    motionProperties.multiplyWorldSpaceInverseInertiaByVector(_applyImpulse_invIJ, mp, body.quaternion, _applyImpulse_jacobian);

    // calculate inverse effective mass at contact point:
    // 1/M_eff = 1/m + jacobianᵀ * (I⁻¹ * jacobian)
    const angularContribution = vec3.dot(_applyImpulse_jacobian, _applyImpulse_invIJ);

    // total inverse effective mass
    const invEffectiveMass = invMass + angularContribution;

    // calculate impulse: P = deltaV / invEffectiveMass
    let impulse = deltaVelocity / invEffectiveMass;

    // clamp impulse by max strength (P = F * dt)
    const maxImpulse = character.maxStrength * deltaTime;
    impulse = Math.min(impulse, maxImpulse);

    // calculate world space impulse vector
    vec3.scale(_applyImpulse_worldImpulse, contact.contactNormal, -impulse);

    // cancel impulse in down direction (we apply gravity later)
    const impulseDotUp = vec3.dot(_applyImpulse_worldImpulse, character.up);
    if (impulseDotUp < 0) {
        vec3.scale(_applyImpulse_downComponent, character.up, impulseDotUp);
        vec3.sub(_applyImpulse_worldImpulse, _applyImpulse_worldImpulse, _applyImpulse_downComponent);
    }

    // apply impulse at contact point
    rigidBody.addImpulseAtPosition(world, body, _applyImpulse_worldImpulse, contact.position);
}

const _solveConstraints_newVelocity = /* @__PURE__ */ vec3.create();
const _solveConstraints_edgeDirection = /* @__PURE__ */ vec3.create();
const _solveConstraints_lastVelocity = /* @__PURE__ */ vec3.create();
const _solveConstraints_verticalPlaneNormal = /* @__PURE__ */ vec3.create();
const _solveConstraints_relativeVelocity = /* @__PURE__ */ vec3.create();
const _solveConstraints_perpVelocity1 = /* @__PURE__ */ vec3.create();
const _solveConstraints_perpVelocity2 = /* @__PURE__ */ vec3.create();

/**
 * Calculates the time of impact (TOI) for a constraint.
 *
 * Returns:
 * - FLT_MAX if constraint is not pushing (projectedVelocity < threshold)
 * - FLT_MAX if penetration would be acceptable (within tolerance)
 * - 0 or positive time when constraint will be hit
 *
 * Positive projectedVelocity means the constraint is pushing the character (constraint velocity
 * exceeds character velocity in the normal direction). This happens when:
 * - Character is moving into the plane (e.g., gravity pushing down into floor)
 * - Constraint has penetration recovery velocity pushing character out
 */
function calculateConstraintTOI(
    constraint: CharacterConstraint,
    velocity: Vec3,
    displacement: Vec3,
    timeRemaining: number,
): number {
    // distance to plane (signed distance from current accumulated displacement)
    const distToPlane = vec3.dot(constraint.planeNormal, displacement) + constraint.planeDistance;

    // projected velocity: how fast the constraint is pushing relative to character motion
    const projectedVelocity =
        vec3.dot(constraint.linearVelocity, constraint.planeNormal) - vec3.dot(velocity, constraint.planeNormal);

    constraint.projectedVelocity = projectedVelocity;

    // if projectedVelocity < threshold, constraint is not pushing us
    if (projectedVelocity < 1e-6) {
        return Number.MAX_VALUE;
    }

    // check if penetration would be acceptable
    // if predicted penetration at end of time step is within tolerance, accept the movement
    if (distToPlane - projectedVelocity * timeRemaining > -1e-4) {
        // too little penetration, accept the movement
        return Number.MAX_VALUE;
    }

    // time to reach plane = distance / velocity
    return Math.max(0, distToPlane / projectedVelocity);
}

/**
 * Comparison function for constraint sorting.
 *
 * Priority:
 * 1. If both at TOI <= 0: Sort by projectedVelocity (highest first) - prioritizes hardest pushing constraints
 * 2. Then by TOI (earliest first)
 * 3. Then static bodies before dynamic bodies (tiebreaker)
 */
function compareConstraints(a: CharacterConstraint, b: CharacterConstraint): number {
    // if both constraints hit at t <= 0, order by the one that will push the character furthest first
    // note: because we add velocity to penetrating contacts, this also resolves contacts that penetrate the most
    if (a.toi <= 0 && b.toi <= 0) {
        if (a.projectedVelocity !== b.projectedVelocity) {
            return b.projectedVelocity - a.projectedVelocity; // Descending (highest first)
        }
    }

    // then sort on time of impact
    if (a.toi !== b.toi) {
        return a.toi - b.toi;
    }

    // as a tie breaker, sort by motion type (dynamic first, static last)
    // higher motion type values come first (descending order)
    return b.contact.motionType - a.contact.motionType;
}

/**
 * Projects velocity onto a constraint plane (slides along surface).
 * This removes the component of velocity moving into the plane, preserving tangent velocity.
 */
function slideVelocityAlongPlane(outVelocity: Vec3, velocity: Vec3, constraint: CharacterConstraint): void {
    // relVelDotNormal = how much character is moving into plane relative to constraint
    const relVelDotNormal =
        vec3.dot(velocity, constraint.planeNormal) - vec3.dot(constraint.linearVelocity, constraint.planeNormal);

    // new_velocity = velocity - relVelDotNormal * plane_normal
    // this cancels the component of velocity going into the plane
    vec3.scaleAndAdd(outVelocity, velocity, constraint.planeNormal, -relVelDotNormal);
}

/** slides velocity along the intersection of two constraint planes */
function slideAlongEdge(
    outVelocity: Vec3,
    velocity: Vec3,
    constraint1: CharacterConstraint,
    constraint2: CharacterConstraint,
): void {
    // edge direction is cross product of the two normals
    vec3.cross(_solveConstraints_edgeDirection, constraint1.planeNormal, constraint2.planeNormal);

    const edgeLenSq = vec3.squaredLength(_solveConstraints_edgeDirection);
    if (edgeLenSq < 1e-12) {
        // planes are parallel, just use first constraint
        slideVelocityAlongPlane(outVelocity, velocity, constraint1);
        return;
    }

    vec3.normalize(_solveConstraints_edgeDirection, _solveConstraints_edgeDirection);

    // project velocity onto edge direction
    const velocityAlongEdge = vec3.dot(velocity, _solveConstraints_edgeDirection);
    vec3.scale(outVelocity, _solveConstraints_edgeDirection, velocityAlongEdge);

    // calculate perpendicular velocity components (perpendicular to slide direction)
    // perpVelocity = constraintVelocity - dot(constraintVelocity, slideDir) * slideDir
    const vel1AlongEdge = vec3.dot(constraint1.linearVelocity, _solveConstraints_edgeDirection);
    vec3.scaleAndAdd(
        _solveConstraints_perpVelocity1,
        constraint1.linearVelocity,
        _solveConstraints_edgeDirection,
        -vel1AlongEdge,
    );

    const vel2AlongEdge = vec3.dot(constraint2.linearVelocity, _solveConstraints_edgeDirection);
    vec3.scaleAndAdd(
        _solveConstraints_perpVelocity2,
        constraint2.linearVelocity,
        _solveConstraints_edgeDirection,
        -vel2AlongEdge,
    );

    // add perpendicular components to output velocity
    vec3.add(outVelocity, outVelocity, _solveConstraints_perpVelocity1);
    vec3.add(outVelocity, outVelocity, _solveConstraints_perpVelocity2);
}

/**
 * Solves character movement within constraints.
 *
 * Iteratively finds the earliest constraint contact, moves to it,
 * and slides velocity along the constraint plane.
 *
 * @param world the physics world
 * @param character the character controller
 * @param velocity input/output velocity (modified during solving)
 * @param deltaTime total time step
 * @param timeRemaining remaining time to simulate
 * @param listener optional listener for contact callbacks
 * @param outDisplacement output displacement
 * @param ioIgnoredContacts array to track ignored contacts (mutated)
 * @param pool constraints pool to use
 * @returns time actually simulated
 */
function solveConstraints(
    world: World,
    character: KCC,
    velocity: Vec3,
    deltaTime: number,
    timeRemaining: number,
    listener: CharacterListener | undefined,
    outDisplacement: Vec3,
    ioIgnoredContacts: CharacterContact[],
    constraints: CharacterConstraint[],
): number {
    vec3.zero(outDisplacement);

    if (constraints.length === 0) {
        // no constraints - move freely
        vec3.scaleAndAdd(outDisplacement, outDisplacement, velocity, timeRemaining);
        return timeRemaining;
    }

    // track last velocity for reversal detection
    // if velocity reverses direction, we should stop to prevent corner sticking
    vec3.copy(_solveConstraints_lastVelocity, velocity);

    // track previous constraints for two-plane intersection
    const previousConstraints: CharacterConstraint[] = [];

    let timeSimulated = 0;

    for (let iteration = 0; iteration < character.maxConstraintIterations; iteration++) {
        /* calculate TOI for all constraints */
        for (const constraint of constraints) {
            constraint.toi = calculateConstraintTOI(constraint, velocity, outDisplacement, timeRemaining);
        }

        /* sort constraints by priority */
        constraints.sort(compareConstraints);

        /* find first valid constraint (one we're moving toward within time remaining) */
        let activeConstraint: CharacterConstraint | null = null;
        let reachedGoal = false;

        for (const constraint of constraints) {
            // can we reach our goal without hitting this constraint?
            if (constraint.toi >= timeRemaining) {
                // we can reach our goal!
                vec3.scaleAndAdd(outDisplacement, outDisplacement, velocity, timeRemaining);
                timeSimulated += timeRemaining;
                reachedGoal = true;
                break;
            }

            // skip contacts that were previously discarded
            if (constraint.contact.wasDiscarded) {
                continue;
            }

            // skip if we're not moving toward this constraint
            if (constraint.projectedVelocity <= 1e-10) {
                continue;
            }

            // handle contact - validation, hadCollision, onContactAdded/Persisted callbacks, impulse application
            if (
                !constraint.contact.hadCollision &&
                !handleContact(world, character, constraint.contact, listener, velocity, deltaTime)
            ) {
                // Constraint should be ignored, mark it as discarded
                constraint.contact.wasDiscarded = true;

                // Add to ignored contacts for GetFirstContactForSweep
                ioIgnoredContacts.push(constraint.contact);
                continue;
            }

            // cancel velocity of constraint if it cannot push the character
            if (!constraint.contact.canPushCharacter) {
                vec3.zero(constraint.linearVelocity);
            }

            // we found a valid constraint
            activeConstraint = constraint;
            break;
        }

        // if we reached our goal or all constraints were discarded
        if (reachedGoal) {
            break;
        }

        if (!activeConstraint) {
            // all constraints were discarded, we can reach our goal!
            vec3.scaleAndAdd(outDisplacement, outDisplacement, velocity, timeRemaining);
            timeSimulated += timeRemaining;
            break;
        }

        /* move to constraint */
        const moveTime = Math.max(0, activeConstraint.toi);
        vec3.scaleAndAdd(outDisplacement, outDisplacement, velocity, moveTime);
        timeRemaining -= moveTime;
        timeSimulated += moveTime;

        // if there's not enough time left to be simulated, bail
        if (timeRemaining < character.minTimeRemaining) {
            break;
        }

        // if we've moved significantly, clear all previous contacts
        if (moveTime > 1e-4) {
            previousConstraints.length = 0;
        }

        /* handle steep slope - cancel velocity towards slope before computing sliding velocity */
        // prevents sliding up the slope due to hitting it before the vertical wall constraint)
        if (activeConstraint.isSteepSlope) {
            // create a vertical plane that blocks any further movement up the slope
            // verticalPlaneNormal = planeNormal - up * dot(planeNormal, up) (not normalized)
            const dotUp = vec3.dot(activeConstraint.planeNormal, character.up);
            vec3.scaleAndAdd(_solveConstraints_verticalPlaneNormal, activeConstraint.planeNormal, character.up, -dotUp);

            // get relative velocity between character and constraint
            vec3.subtract(_solveConstraints_relativeVelocity, velocity, activeConstraint.linearVelocity);

            // remove velocity towards the slope (only if moving toward it)
            const relVelDotVertical = vec3.dot(_solveConstraints_relativeVelocity, _solveConstraints_verticalPlaneNormal);
            if (relVelDotVertical < 0) {
                const verticalLenSq = vec3.squaredLength(_solveConstraints_verticalPlaneNormal);
                if (verticalLenSq > 1e-8) {
                    // velocity -= relVelDotVertical * verticalPlaneNormal / verticalLenSq
                    vec3.scaleAndAdd(
                        velocity,
                        velocity,
                        _solveConstraints_verticalPlaneNormal,
                        -relVelDotVertical / verticalLenSq,
                    );
                }
            }
        }

        /* calculate new velocity (slide along plane) */
        slideVelocityAlongPlane(_solveConstraints_newVelocity, velocity, activeConstraint);

        /* find the previous constraint that we will violate the most (two-plane sliding) */
        // search for highest penetration, excluding parallel normals
        let highestPenetration = 0;
        let otherConstraint: CharacterConstraint | null = null;

        for (const prevConstraint of previousConstraints) {
            if (prevConstraint === activeConstraint) continue;

            // calculate how much we will penetrate if we move in this direction
            // penetration = (constraintVelocity - newVelocity) · normal
            const penetration =
                vec3.dot(prevConstraint.linearVelocity, prevConstraint.planeNormal) -
                vec3.dot(_solveConstraints_newVelocity, prevConstraint.planeNormal);

            if (penetration > highestPenetration) {
                // we don't want parallel or anti-parallel normals as that will cause our cross product to become zero
                // slack is approx 10 degrees (cos(10°) ≈ 0.984)
                const dot = vec3.dot(prevConstraint.planeNormal, activeConstraint.planeNormal);
                if (dot < 0.984 && dot > -0.984) {
                    highestPenetration = penetration;
                    otherConstraint = prevConstraint;
                }
            }

            // cancel the constraint velocity in the active constraint plane's direction so that we won't try to apply it again and keep ping ponging between planes
            const velDotActivePlane = vec3.dot(prevConstraint.linearVelocity, activeConstraint.planeNormal);
            const cancelAmount = Math.min(0, velDotActivePlane);
            vec3.scaleAndAdd(
                prevConstraint.linearVelocity,
                prevConstraint.linearVelocity,
                activeConstraint.planeNormal,
                -cancelAmount,
            );

            // cancel the active constraint's velocity in this constraint plane's direction so that we won't try to apply it again and keep ping ponging between planes
            const activeVelDotPlane = vec3.dot(activeConstraint.linearVelocity, prevConstraint.planeNormal);
            const cancelActive = Math.min(0, activeVelDotPlane);
            vec3.scaleAndAdd(
                activeConstraint.linearVelocity,
                activeConstraint.linearVelocity,
                prevConstraint.planeNormal,
                -cancelActive,
            );
        }

        // if we found a 2nd constraint, slide along the intersection of both planes
        if (otherConstraint) {
            slideAlongEdge(_solveConstraints_newVelocity, velocity, activeConstraint, otherConstraint);
        }

        /* fire contact solve callback (allows velocity modification) */
        if (listener?.onContactSolve) {
            const body = rigidBody.get(world, activeConstraint.contact.bodyId);

            if (body) {
                listener.onContactSolve(
                    character,
                    body,
                    activeConstraint.contact.subShapeId,
                    activeConstraint.contact.position,
                    activeConstraint.contact.contactNormal,
                    activeConstraint.contact.linearVelocity,
                    velocity,
                    _solveConstraints_newVelocity,
                );
            }
        }

        /* update velocity */
        vec3.copy(velocity, _solveConstraints_newVelocity);
        previousConstraints.push(activeConstraint);

        // early out conditions
        // check if constraint has no pushing velocity and character velocity is near zero
        if (activeConstraint.projectedVelocity < 1e-8 && vec3.squaredLength(velocity) < 1e-8) {
            break;
        }

        // check for velocity reversal
        // if constraint has velocity, accept the new velocity and update last_velocity
        // otherwise check if velocity reversed relative to last_velocity
        const constraintVelLenSq = vec3.squaredLength(activeConstraint.linearVelocity);
        if (constraintVelLenSq > 1e-16) {
            // constraint has velocity - update last_velocity reference
            vec3.copy(_solveConstraints_lastVelocity, activeConstraint.linearVelocity);
        } else {
            // check if velocity reversed
            if (vec3.dot(velocity, _solveConstraints_lastVelocity) < 0) {
                break;
            }
        }
    }

    return timeSimulated;
}

/**
 * Calculates signed distance from a point to the supporting volume plane.
 * Points in front of the plane (positive distance) are "side hits", not supporting.
 * Points behind the plane (negative distance) can support the character.
 *
 * @param point point in character local space
 * @param plane supporting volume plane as Vec4 [normal.x, normal.y, normal.z, distance]
 * @returns signed distance from point to plane
 */
function supportingVolumeSignedDistance(point: Vec3, plane: Vec4): number {
    // plane equation: n·p + d = 0
    // signed distance = n·p + d (where d is the plane constant stored in w)
    return point[0] * plane[0] + point[1] * plane[1] + point[2] * plane[2] + plane[3];
}

const _updateSupporting_avgNormal = /* @__PURE__ */ vec3.create();
const _updateSupporting_avgVelocity = /* @__PURE__ */ vec3.create();
const _updateSupporting_contactLocalPos = /* @__PURE__ */ vec3.create();
const _updateSupporting_invQ = /* @__PURE__ */ quat.create();

const _updateSupporting_downVelocity = /* @__PURE__ */ vec3.create();
const _updateSupporting_displacement = /* @__PURE__ */ vec3.create();
const _updateSupporting_ignoredContacts: CharacterContact[] = [];

/**
 * Updates ground state from current contacts.
 * Determines if character is on ground, on steep ground, not supported, or in air.
 *
 * Algorithm:
 * 1. Flag contacts as having collision if close enough (distance < collisionTolerance)
 *    and not moving away from the surface
 * 2. For each colliding contact, determine if it's supporting or sliding based on
 *    supporting volume plane and slope angle
 * 3. Calculate average ground normal and velocity from supporting contacts
 * 4. For steep-slope-only scenarios, check if multiple slopes create corner support
 * 5. Set ground state accordingly
 *
 * @param world the physics world
 * @param character the character controller
 * @param skipContactVelocityCheck if true, skip the velocity check for collision marking
 * @param lastDeltaTime the delta time from the last update (for kinematic velocity and corner support)
 * @param listener optional listener for contact callbacks
 */
function updateSupportingContact(
    world: World,
    character: KCC,
    skipContactVelocityCheck: boolean,
    lastDeltaTime: number,
    listener: CharacterListener | undefined,
): void {
    const contacts = character.contacts;

    /* step 1: flag contacts as having collision if they're close enough */
    // skip contacts we're moving away from (unless skipContactVelocityCheck is true)
    for (let i = 0; i < contacts.length; i++) {
        const contact = contacts[i];
        if (contact.wasDiscarded) continue;
        if (contact.hadCollision) continue; // already marked by moveShape

        // check distance threshold
        if (contact.distance >= character.collisionTolerance) continue;

        // check velocity - skip if moving away from contact (unless skipping check)
        if (!skipContactVelocityCheck) {
            // relativeVelocity = characterVelocity - contactVelocity
            // if dot(surfaceNormal, relativeVelocity) > 0, we're moving away
            const relVelDotNormal =
                contact.surfaceNormal[0] * (character.linearVelocity[0] - contact.linearVelocity[0]) +
                contact.surfaceNormal[1] * (character.linearVelocity[1] - contact.linearVelocity[1]) +
                contact.surfaceNormal[2] * (character.linearVelocity[2] - contact.linearVelocity[2]);

            if (relVelDotNormal > 1e-4) continue; // moving away from contact
        }

        // validate and add contact
        // note: settings are discarded, canPushCharacter keeps its default value
        if (validateContact(world, character, contact, listener)) {
            const dummySettings: CharacterContactSettings = {
                canPushCharacter: true,
                canReceiveImpulses: true,
            };
            contactAdded(world, character, contact, listener, dummySettings);
            contact.hadCollision = true;
        } else {
            contact.wasDiscarded = true;
        }
    }

    /* step 2: calculate inverse transform to convert contact positions to local space */
    // we need to check if contacts are within the supporting volume
    // invTransform = inverse(rotation) * -position
    const invQ = quat.conjugate(_updateSupporting_invQ, character.quaternion);

    /* step 3: determine supporting contacts and calculate averages */
    let numSupported = 0;
    let numSliding = 0;
    let numAvgNormal = 0;
    vec3.zero(_updateSupporting_avgNormal);
    vec3.zero(_updateSupporting_avgVelocity);

    let supportingContact: CharacterContact | null = null;
    let maxCosAngle = -Infinity;
    let deepestContact: CharacterContact | null = null;
    let smallestDistance = Infinity;

    for (const contact of contacts) {
        if (!contact.hadCollision || contact.wasDiscarded) continue;

        // calculate angle between surface normal and up
        const cosAngle = vec3.dot(contact.surfaceNormal, character.up);

        // find deepest contact (smallest distance)
        if (contact.distance < smallestDistance) {
            deepestContact = contact;
            smallestDistance = contact.distance;
        }

        // check if contact is within supporting volume
        // transform contact position to character local space
        vec3.sub(_updateSupporting_contactLocalPos, contact.position, character.position);
        vec3.transformQuat(_updateSupporting_contactLocalPos, _updateSupporting_contactLocalPos, invQ);

        // check signed distance to supporting volume plane
        // (i.e., it does NOT subtract mShapeOffset)
        if (supportingVolumeSignedDistance(_updateSupporting_contactLocalPos, character.supportingVolumePlane) > 0) {
            // contact is in front of supporting plane - not a supporting contact
            continue;
        }

        // find contact with normal pointing most upward
        if (cosAngle > maxCosAngle) {
            supportingContact = contact;
            maxCosAngle = cosAngle;
        }

        // check if this is a supporting or sliding contact
        // if slope checking is disabled (cosMaxSlopeAngle > NO_MAX_SLOPE_ANGLE), all contacts are supported
        const isSupported = character.cosMaxSlopeAngle > NO_MAX_SLOPE_ANGLE || cosAngle >= character.cosMaxSlopeAngle;
        if (isSupported) {
            numSupported++;
        } else {
            numSliding++;
        }

        // use contacts with normal angle < 85° from up for averaging
        // cos(85°) ≈ 0.0872, we use 0.08
        if (cosAngle >= 0.08) {
            vec3.add(_updateSupporting_avgNormal, _updateSupporting_avgNormal, contact.surfaceNormal);
            numAvgNormal++;

            // for static/dynamic or non-supporting contacts, use contact velocity.
            // for kinematic supporting contacts, calculate velocity at character position to properly track rotating platforms
            if (contact.motionType !== MotionType.KINEMATIC || !isSupported) {
                vec3.add(_updateSupporting_avgVelocity, _updateSupporting_avgVelocity, contact.linearVelocity);
            } else {
                // for kinematic supporting contacts, calculate ground velocity at character position
                // this accounts for the rotational velocity of the platform
                const body = rigidBody.get(world, contact.bodyId);
                if (body?.motionProperties) {
                    // get adjusted body velocity (for conveyor belts on kinematic bodies)
                    let linearVelocity = body.motionProperties.linearVelocity;
                    let angularVelocity = body.motionProperties.angularVelocity;

                    if (listener?.onAdjustBodyVelocity) {
                        vec3.copy(_contactVelocity_adjustedLinear, linearVelocity);
                        vec3.copy(_contactVelocity_adjustedAngular, angularVelocity);
                        listener.onAdjustBodyVelocity(
                            character,
                            body,
                            _contactVelocity_adjustedLinear,
                            _contactVelocity_adjustedAngular,
                        );
                        linearVelocity = _contactVelocity_adjustedLinear;
                        angularVelocity = _contactVelocity_adjustedAngular;
                    }

                    const groundVel = calculateCharacterGroundVelocity(
                        character,
                        body.centerOfMassPosition,
                        linearVelocity,
                        angularVelocity,
                        lastDeltaTime,
                    );
                    vec3.add(_updateSupporting_avgVelocity, _updateSupporting_avgVelocity, groundVel);
                } else {
                    // fallback to contact velocity
                    vec3.add(_updateSupporting_avgVelocity, _updateSupporting_avgVelocity, contact.linearVelocity);
                }
            }
        }
    }

    // choose best contact: prefer supporting contact, fall back to deepest
    const bestContact = supportingContact ?? deepestContact;

    /* step 4: set ground normal and velocity */
    const ground = character.ground;

    if (numAvgNormal >= 1) {
        vec3.normalize(ground.normal, _updateSupporting_avgNormal);
        vec3.scale(ground.velocity, _updateSupporting_avgVelocity, 1 / numAvgNormal);
    } else if (bestContact) {
        vec3.copy(ground.normal, bestContact.surfaceNormal);
        vec3.copy(ground.velocity, bestContact.linearVelocity);
    } else {
        vec3.zero(ground.normal);
        vec3.zero(ground.velocity);
    }

    // copy contact properties to ground info
    if (bestContact) {
        ground.bodyId = bestContact.bodyId;
        ground.subShapeId = bestContact.subShapeId;
        vec3.copy(ground.position, bestContact.position);
    } else {
        ground.bodyId = INVALID_BODY_ID;
        ground.subShapeId = 0;
        vec3.zero(ground.position);
    }

    /* step 5: determine ground state */
    if (numSupported > 0) {
        // standing on walkable surface
        ground.state = GroundState.ON_GROUND;
    } else if (numSliding > 0) {
        // only steep slopes - check if we're moving upward relative to ground
        if (deepestContact) {
            const relVelDotUp =
                (character.linearVelocity[0] - deepestContact.linearVelocity[0]) * character.up[0] +
                (character.linearVelocity[1] - deepestContact.linearVelocity[1]) * character.up[1] +
                (character.linearVelocity[2] - deepestContact.linearVelocity[2]) * character.up[2];

            if (relVelDotUp > 1e-4) {
                // moving upward relative to ground - can't be on ground
                ground.state = GroundState.ON_STEEP_GROUND;
            } else {
                // if we're sliding down, we may actually be standing on multiple sliding contacts
                // in such a way that we can't slide off - in this case we're also supported

                // convert the contacts into constraints
                determineConstraints(character, contacts, lastDeltaTime, _activeConstraints);

                // solve displacement using these constraints with -up velocity
                // this checks if we would move at all when "falling"
                vec3.negate(_updateSupporting_downVelocity, character.up);
                _updateSupporting_ignoredContacts.length = 0;

                const timeSimulated = solveConstraints(
                    world,
                    character,
                    _updateSupporting_downVelocity,
                    1.0, // deltaTime = 1.0 (unit time)
                    1.0, // timeRemaining = 1.0
                    undefined, // no listener during corner check
                    _updateSupporting_displacement,
                    _updateSupporting_ignoredContacts,
                    _activeConstraints,
                );

                // release constraints after use
                releaseAllConstraints(_activeConstraints);

                // if we're blocked then we're supported, otherwise we're sliding
                // threshold: displacement² < (0.6 * lastDeltaTime)²
                const minRequiredDisplacementSq = 0.36 * lastDeltaTime * lastDeltaTime; // 0.6² = 0.36
                const displacementLenSq = vec3.squaredLength(_updateSupporting_displacement);

                if (timeSimulated < 0.001 || displacementLenSq < minRequiredDisplacementSq) {
                    // blocked by constraints - considered supported
                    ground.state = GroundState.ON_GROUND;
                } else {
                    ground.state = GroundState.ON_STEEP_GROUND;
                }
            }
        } else {
            ground.state = GroundState.ON_STEEP_GROUND;
        }
    } else {
        // not supported by anything
        ground.state = bestContact ? GroundState.NOT_SUPPORTED : GroundState.IN_AIR;
    }
}

const _cancelVelocity_normal = /* @__PURE__ */ vec3.create();

/**
 * Cancels velocity component toward steep slopes.
 * This prevents the character from accelerating into steep slopes.
 *
 * Call this BEFORE update() to modify the desired velocity.
 *
 * Algorithm:
 * 1. If on ground or in air, return velocity unchanged
 * 2. For each steep slope contact:
 *    a. Get horizontal component of contact normal
 *    b. If velocity is toward the slope, cancel that component
 *
 * @param character the character controller
 * @param desiredVelocity input desired velocity
 * @param outVelocity output velocity with steep slope components canceled
 */
function cancelVelocityTowardsSteepSlopes(character: KCC, desiredVelocity: Vec3, outVelocity: Vec3): void {
    // start with desired velocity
    vec3.copy(outVelocity, desiredVelocity);

    // if we're on ground or in air, no cancellation needed
    // note: this is important as WalkStairs may override ground state
    const groundState = character.ground.state;

    if (groundState === GroundState.ON_GROUND || groundState === GroundState.IN_AIR) {
        return;
    }

    const contacts = character.contacts;

    for (let i = 0; i < contacts.length; i++) {
        const contact = contacts[i];
        if (!contact.hadCollision || contact.wasDiscarded) continue;

        // only process steep slopes
        if (!isSlopeTooSteep(character, contact.surfaceNormal)) continue;

        // use contact normal for better sliding (surface normal may point away from movement)
        vec3.copy(_cancelVelocity_normal, contact.contactNormal);

        // remove vertical component to get horizontal normal
        const dotUp = vec3.dot(_cancelVelocity_normal, character.up);
        vec3.scaleAndAdd(_cancelVelocity_normal, _cancelVelocity_normal, character.up, -dotUp);

        // cancel horizontal movement toward the slope
        const dot = vec3.dot(_cancelVelocity_normal, outVelocity);
        if (dot < 0) {
            // moving toward slope - cancel that component
            const lenSq = vec3.squaredLength(_cancelVelocity_normal);
            if (lenSq > 1e-12) {
                vec3.scaleAndAdd(outVelocity, outVelocity, _cancelVelocity_normal, -dot / lenSq);
            }
        }
    }
}

const _hasSteepSlopes_horizontal = /* @__PURE__ */ vec3.create();

/**
 * Checks if there are steep slopes facing the movement direction.
 * This is used to determine if the character should attempt stair walking.
 *
 * Algorithm:
 * 1. Return false if not supported (must be on ground or steep ground)
 * 2. Return false if no horizontal velocity
 * 3. For each steep slope contact:
 *    a. Check if we're pushing into the contact (velocity toward it)
 *    b. If yes, return true - we have steep slopes to potentially walk
 *
 * @param character the character controller
 * @param linearVelocity the desired linear velocity (used to check movement direction)
 * @returns true if there are steep slopes the character might need to walk up
 */
function hasSteepSlopesToWalk(character: KCC, linearVelocity: Vec3): boolean {
    // Must be supported to walk stairs
    if (!isSupported(character)) {
        return false;
    }

    // Get horizontal velocity (remove vertical component)
    const dotUp = vec3.dot(linearVelocity, character.up);
    vec3.scaleAndAdd(_hasSteepSlopes_horizontal, linearVelocity, character.up, -dotUp);

    // Need enough horizontal velocity
    if (vec3.squaredLength(_hasSteepSlopes_horizontal) < 1e-12) {
        return false;
    }

    const contacts = character.contacts;

    for (let i = 0; i < contacts.length; i++) {
        const contact = contacts[i];
        if (!contact.hadCollision || contact.wasDiscarded) continue;

        // Only check steep slopes
        if (!isSlopeTooSteep(character, contact.surfaceNormal)) continue;

        // Check if we're pushing into the contact
        // relativeVelocity = horizontalVelocity - contactVelocity
        // If dot(surfaceNormal, relativeVelocity) < 0, we're pushing into it
        const relVelDotNormal =
            contact.surfaceNormal[0] * (_hasSteepSlopes_horizontal[0] - contact.linearVelocity[0]) +
            contact.surfaceNormal[1] * (_hasSteepSlopes_horizontal[1] - contact.linearVelocity[1]) +
            contact.surfaceNormal[2] * (_hasSteepSlopes_horizontal[2] - contact.linearVelocity[2]);

        if (relVelDotNormal < 0) {
            // Pushing into a steep slope - we may need to walk stairs
            return true;
        }
    }

    return false;
}

const _moveShape_movementDirection = /* @__PURE__ */ vec3.create();
const _moveShape_displacement = /* @__PURE__ */ vec3.create();
const _moveShape_sweepContact = /* @__PURE__ */ createCharacterContact();
const _moveShape_ignoredContacts: CharacterContact[] = [];
const _moveShape_velocity = /* @__PURE__ */ vec3.create();

/**
 * core movement with collision resolution.
 *
 * for each collision iteration (up to maxCollisionIterations):
 *    - get contacts at current position
 *    - remove conflicting contacts (opposing penetration normals)
 *    - convert contacts to constraints
 *    - solve movement within constraints
 *    - verify path with sweep test
 *    - update position
 *    - early out if displacement is too small
 *
 * @param world the physics world
 * @param character the character controller
 * @param position position to move from (modified in place)
 * @param velocity movement velocity
 * @param deltaTime time step
 * @param filter collision filter
 * @param listener optional listener for contact callbacks
 */
function moveShape(
    world: World,
    character: KCC,
    position: Vec3,
    velocity: Vec3,
    deltaTime: number,
    filter: Filter,
    listener: CharacterListener | undefined,
): void {
    // calculate movement direction for contact detection
    const velocityLenSq = vec3.squaredLength(velocity);
    if (velocityLenSq > 1e-12) {
        const velocityLen = Math.sqrt(velocityLenSq);
        vec3.scale(_moveShape_movementDirection, velocity, 1 / velocityLen);
    } else {
        vec3.zero(_moveShape_movementDirection);
    }

    // copy velocity to scratch (solveConstraints modifies it)
    vec3.copy(_moveShape_velocity, velocity);

    let timeRemaining = deltaTime;

    for (let iteration = 0; iteration < character.maxCollisionIterations; iteration++) {
        // early out if not enough time remaining
        if (timeRemaining < character.minTimeRemaining) {
            break;
        }

        /* get contacts at current position */
        getContactsAtPosition(world, character, position, _moveShape_movementDirection, filter, listener, character.contacts);

        const contacts = character.contacts;

        /* remove conflicting contacts (opposing penetration normals) */
        _moveShape_ignoredContacts.length = 0;
        removeConflictingContacts(contacts, character.characterPadding, _moveShape_ignoredContacts);

        /* convert contacts to constraints */
        determineConstraints(character, contacts, deltaTime, _activeConstraints);

        /* solve movement within constraints */
        // pass ignoredContacts so rejected contacts are tracked
        let timeSimulated = solveConstraints(
            world,
            character,
            _moveShape_velocity,
            deltaTime,
            timeRemaining,
            listener,
            _moveShape_displacement,
            _moveShape_ignoredContacts,
            _activeConstraints,
        );

        /* verify path with sweep test */
        // do a sweep to test if the path is really unobstructed
        // note: getFirstContactForSweep already checks for small displacement internally
        if (
            getFirstContactForSweep(
                _moveShape_sweepContact,
                world,
                character,
                position,
                _moveShape_displacement,
                filter,
                listener,
                _moveShape_ignoredContacts,
            )
        ) {
            // hit something during sweep - limit displacement and time
            const fraction = _moveShape_sweepContact.fraction;
            vec3.scale(_moveShape_displacement, _moveShape_displacement, fraction);
            timeSimulated *= fraction;

            // note: the sweep contact will be detected in the next iteration via getContactsAtPosition
        }

        /* update position */
        vec3.add(position, position, _moveShape_displacement);
        timeRemaining -= timeSimulated;

        /* early out if displacement is too small (we're stuck) */
        if (vec3.squaredLength(_moveShape_displacement) < 1e-8) {
            break;
        }
    }

    // release constraints after all iterations
    releaseAllConstraints(_activeConstraints);
}

/**
 * Resets contact tracking for a new frame.
 * @param character the character controller
 */
function resetContactTracking(character: KCC): void {
    // mark all current contacts with hadCollision as "not seen this frame"
    const contacts = character.contacts;
    for (let i = 0; i < contacts.length; i++) {
        const contact = contacts[i];
        if (contact.hadCollision) {
            const packedKey = packListenerContactKey(contact.bodyId, contact.subShapeId);
            const value = acquireListenerContact(character.listenerContacts);
            value.packedKey = packedKey;
            value.bodyId = contact.bodyId;
            value.subShapeId = contact.subShapeId;
            value.count = 0;
            value.settings.canPushCharacter = true;
            value.settings.canReceiveImpulses = true;
        }
    }
}

const _validateContact_negatedNormal = /* @__PURE__ */ vec3.create();

/**
 * validates a contact with the listener.
 * @returns true if contact should be processed, false to discard
 */
function validateContact(
    world: World,
    character: KCC,
    contact: CharacterContact,
    listener: CharacterListener | undefined,
): boolean {
    if (!listener?.onContactValidate) {
        return true;
    }

    const body = rigidBody.get(world, contact.bodyId);

    if (!body) {
        return true;
    }

    // negate contactNormal: internal storage points toward character,
    // but callbacks expect normal pointing away (into surface)
    return listener.onContactValidate(
        character,
        body,
        contact.subShapeId,
        contact.position,
        vec3.negate(_validateContact_negatedNormal, contact.contactNormal),
    );
}

const _contactAdded_negatedNormal = /* @__PURE__ */ vec3.create();

/**
 * Fires contact added or persisted callback based on tracking state.
 * @param world the physics world
 * @param character the character controller
 * @param contact the contact being processed
 * @param listener optional listener for callbacks
 * @param ioSettings contact settings (modified by callback)
 */
function contactAdded(
    world: World,
    character: KCC,
    contact: CharacterContact,
    listener: CharacterListener | undefined,
    ioSettings: CharacterContactSettings,
): void {
    if (!listener) {
        return;
    }

    const packedKey = packListenerContactKey(contact.bodyId, contact.subShapeId);
    const tracked = findListenerContact(character.listenerContacts, packedKey);

    if (tracked) {
        // contact was known from before - fire onContactPersisted (max 1 per contact)
        if (++tracked.count === 1) {
            if (listener.onContactPersisted) {
                const body = rigidBody.get(world, contact.bodyId);
                if (body) {
                    // negate contactNormal: storage points toward character,
                    // callbacks expect normal pointing away (into surface)
                    listener.onContactPersisted(
                        character,
                        body,
                        contact.subShapeId,
                        contact.position,
                        vec3.negate(_contactAdded_negatedNormal, contact.contactNormal),
                        ioSettings,
                    );
                }
            }
            tracked.settings.canPushCharacter = ioSettings.canPushCharacter;
            tracked.settings.canReceiveImpulses = ioSettings.canReceiveImpulses;
        } else {
            // reuse settings from last call (avoid multiple callbacks for same contact)
            ioSettings.canPushCharacter = tracked.settings.canPushCharacter;
            ioSettings.canReceiveImpulses = tracked.settings.canReceiveImpulses;
        }
    } else {
        // new contact - fire onContactAdded
        if (listener.onContactAdded) {
            const body = rigidBody.get(world, contact.bodyId);
            if (body) {
                // negate contactNormal: storage points toward character,
                // callbacks expect normal pointing away (into surface)
                listener.onContactAdded(
                    character,
                    body,
                    contact.subShapeId,
                    contact.position,
                    vec3.negate(_contactAdded_negatedNormal, contact.contactNormal),
                    ioSettings,
                );
            }
        }
        // insert into tracking pool
        const value = acquireListenerContact(character.listenerContacts);
        value.packedKey = packedKey;
        value.bodyId = contact.bodyId;
        value.subShapeId = contact.subShapeId;
        value.count = 1;
        value.settings.canPushCharacter = ioSettings.canPushCharacter;
        value.settings.canReceiveImpulses = ioSettings.canReceiveImpulses;
    }
}

/**
 * Handles a contact during constraint solving.
 *
 * 1. Validate contact with listener (ValidateContact)
 * 2. Set hadCollision = true
 * 3. Call contactAdded (fires onContactAdded or onContactPersisted, modifies settings)
 * 4. IMMEDIATELY store contact.canPushCharacter from settings.canPushCharacter
 * 5. Check if sensor - if yes, return false (no further interaction)
 * 6. Store contact.canReceiveImpulses from settings.canReceiveImpulses
 * 7. Apply impulse to dynamic bodies (BEFORE velocity solving)
 *
 * @param world the physics world
 * @param character the character controller
 * @param contact the contact to handle
 * @param listener optional listener for callbacks
 * @param characterVelocity character's current velocity (for impulse calculation)
 * @param deltaTime time step (for impulse calculation)
 * @returns true if contact should be processed, false to discard
 */
function handleContact(
    world: World,
    character: KCC,
    contact: CharacterContact,
    listener: CharacterListener | undefined,
    characterVelocity: Vec3,
    deltaTime: number,
): boolean {
    /* 1. validate the contact point */
    if (!validateContact(world, character, contact, listener)) {
        return false;
    }

    /* 2. we collided */
    contact.hadCollision = true;

    /* 3. send contact added event - settings will be modified by listener callback */
    const settings: CharacterContactSettings = {
        canPushCharacter: true,
        canReceiveImpulses: true,
    };
    contactAdded(world, character, contact, listener, settings);

    /* 4. store canPushCharacter from settings */
    contact.canPushCharacter = settings.canPushCharacter;

    /* 5. sensors don't have further interaction beyond callbacks */
    if (contact.isSensor) {
        return false;
    }

    /* 6. store canReceiveImpulses from settings */
    contact.canReceiveImpulses = settings.canReceiveImpulses;

    /* 7. apply impulse to dynamic bodies */
    // this must happen BEFORE velocity solving, not after!
    // impulse calculation uses the character's input velocity, not the solved velocity.
    if (settings.canReceiveImpulses && contact.motionType === MotionType.DYNAMIC) {
        applyImpulseToBody(world, character, contact, characterVelocity, deltaTime);
    }

    return true;
}

/**
 * Finalizes contact tracking and fires removal callbacks.
 * Call this at the end of a frame after all contact processing.
 *
 * @param world the physics world
 * @param character the character controller
 * @param listener optional listener for callbacks
 */
function finalizeContactTracking(world: World, character: KCC, listener: CharacterListener | undefined): void {
    // re-mark all contacts - reset all counts to 0, then mark active ones as 1
    const activeListenerContacts = getActiveListenerContacts(character.listenerContacts);
    for (const value of activeListenerContacts) {
        value.count = 0;
    }

    const contacts = character.contacts;
    for (let i = 0; i < contacts.length; i++) {
        const contact = contacts[i];
        if (contact.hadCollision) {
            const packedKey = packListenerContactKey(contact.bodyId, contact.subShapeId);
            const tracked = findListenerContact(character.listenerContacts, packedKey);
            if (tracked) {
                tracked.count = 1;
            }
        }
    }

    // fire removal callbacks for contacts not seen this frame
    if (listener?.onContactRemoved) {
        for (const value of activeListenerContacts) {
            if (value.count === 0) {
                const body = rigidBody.get(world, value.bodyId);
                if (body) {
                    listener.onContactRemoved(character, body, value.subShapeId);
                }
            }
        }
    }

    // release all listener contacts back to pool
    releaseAllListenerContacts(character.listenerContacts);
}

const _innerBody_shapeOffsetRotated = /* @__PURE__ */ vec3.create();
const _innerBody_paddingOffset = /* @__PURE__ */ vec3.create();
const _innerBody_position = /* @__PURE__ */ vec3.create();

/**
 * Calculates the inner body position.
 * @param character The character controller
 * @param out output vector for the position
 */
function getInnerBodyPosition(character: KCC, out: Vec3): void {
    // rotate shapeOffset by character quaternion
    vec3.transformQuat(_innerBody_shapeOffsetRotated, character.shapeOffset, character.quaternion);

    // calculate padding offset along up direction
    vec3.scale(_innerBody_paddingOffset, character.up, character.characterPadding);

    // position + rotatedShapeOffset + paddingOffset
    vec3.add(out, character.position, _innerBody_shapeOffsetRotated);
    vec3.add(out, out, _innerBody_paddingOffset);
}

/**
 * Updates the inner body transform to match the character.
 * Call this after moving the character to keep the inner body in sync.
 *
 * TODO: DontActivate?
 *
 * @param world the physics world
 * @param character the character controller
 */
function updateInnerBodyTransform(world: World, character: KCC): void {
    // early exit if no inner body
    if (character.innerRigidBodyId === INVALID_BODY_ID) {
        return;
    }

    const innerRigidBody = rigidBody.get(world, character.innerRigidBodyId);

    if (!innerRigidBody) {
        return;
    }

    // calculate inner body position
    getInnerBodyPosition(character, _innerBody_position);

    // update transform
    rigidBody.setTransform(world, innerRigidBody, _innerBody_position, character.quaternion, false);
}

/**
 * Sets the character's position and synchronizes the inner body if present.
 * @param world the physics world
 * @param character the character controller
 * @param position the new world-space position
 */
export function setPosition(world: World, character: KCC, position: Vec3): void {
    vec3.copy(character.position, position);
    updateInnerBodyTransform(world, character);
}

/**
 * Sets the character's rotation and synchronizes the inner body if present.
 * @param world the physics world
 * @param character the character controller
 * @param quaternion the new rotation quaternion
 */
export function setRotation(world: World, character: KCC, quaternion: Quat): void {
    quat.copy(character.quaternion, quaternion);
    updateInnerBodyTransform(world, character);
}

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
export function updateGroundVelocity(world: World, character: KCC, listener?: CharacterListener): void {
    // early exit if no ground body
    if (character.ground.bodyId === INVALID_BODY_ID) {
        return;
    }

    const groundBody = rigidBody.get(world, character.ground.bodyId);
    if (!groundBody) {
        // body was removed - clear ground info
        character.ground.bodyId = INVALID_BODY_ID;
        vec3.zero(character.ground.velocity);
        return;
    }

    // get body velocity (zero for static, real velocity for dynamic/kinematic)
    let linearVelocity: Vec3;
    let angularVelocity: Vec3;
    if (groundBody.motionType === MotionType.DYNAMIC || groundBody.motionType === MotionType.KINEMATIC) {
        linearVelocity = groundBody.motionProperties.linearVelocity;
        angularVelocity = groundBody.motionProperties.angularVelocity;
    } else {
        // static body - start with zero velocity
        vec3.zero(_contactVelocity_staticLinear);
        vec3.zero(_contactVelocity_staticAngular);
        linearVelocity = _contactVelocity_staticLinear;
        angularVelocity = _contactVelocity_staticAngular;
    }

    // allow listener to adjust the body's velocity (e.g., for conveyor belts on static bodies)
    if (listener?.onAdjustBodyVelocity) {
        vec3.copy(_contactVelocity_adjustedLinear, linearVelocity);
        vec3.copy(_contactVelocity_adjustedAngular, angularVelocity);
        listener.onAdjustBodyVelocity(character, groundBody, _contactVelocity_adjustedLinear, _contactVelocity_adjustedAngular);
        linearVelocity = _contactVelocity_adjustedLinear;
        angularVelocity = _contactVelocity_adjustedAngular;
    }

    // calculate ground velocity using adjusted velocities
    const groundVel = calculateCharacterGroundVelocity(
        character,
        groundBody.centerOfMassPosition,
        linearVelocity,
        angularVelocity,
        character.lastDeltaTime,
    );
    vec3.copy(character.ground.velocity, groundVel);
}

const _refreshContacts_movementDirection = /* @__PURE__ */ vec3.create();

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
export function refreshContacts(world: World, character: KCC, filter: Filter, listener?: CharacterListener): void {
    // start tracking contact changes (mark existing contacts as "not seen")
    resetContactTracking(character);

    // calculate movement direction from velocity (or zero if stationary)
    const velocityLenSq = vec3.squaredLength(character.linearVelocity);
    if (velocityLenSq > 1e-12) {
        vec3.normalize(_refreshContacts_movementDirection, character.linearVelocity);
    } else {
        vec3.zero(_refreshContacts_movementDirection);
    }

    // get contacts at current position
    getContactsAtPosition(
        world,
        character,
        character.position,
        _refreshContacts_movementDirection,
        filter,
        listener,
        character.contacts,
    );

    // update supporting contact (determines ground state, marks contacts as hadCollision)
    updateSupportingContact(world, character, true /* skipContactVelocityCheck */, character.lastDeltaTime, listener);

    // finalize contact tracking (fires onContactRemoved callbacks for contacts no longer active)
    finalizeContactTracking(world, character, listener);
}

/**
 * Checks if the character has collided with a specific body during the current frame.
 *
 * @param character the character controller
 * @param bodyId the body ID to check for collision with
 * @returns true if the character has collided with the specified body
 */
export function hasCollidedWith(character: KCC, bodyId: BodyId): boolean {
    const contacts = character.contacts;
    for (let i = 0; i < contacts.length; i++) {
        const contact = contacts[i];
        if (contact.hadCollision && contact.bodyId === bodyId) {
            return true;
        }
    }
    return false;
}

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
export function canWalkStairs(character: KCC, linearVelocity: Vec3): boolean {
    return hasSteepSlopesToWalk(character, linearVelocity);
}

const _setShape_movementDirection = /* @__PURE__ */ vec3.create();

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
export function setShape(
    world: World,
    character: KCC,
    newShape: Shape,
    filter: Filter,
    listener: CharacterListener | undefined,
    maxPenetrationDepth: number,
): boolean {
    // if shape is the same, nothing to do
    if (character.shape === newShape) {
        return true;
    }

    // check if we need to test for penetration
    if (maxPenetrationDepth < Number.MAX_VALUE) {
        // get movement direction from velocity
        const velocityLenSq = vec3.squaredLength(character.linearVelocity);
        if (velocityLenSq > 1e-12) {
            vec3.scale(_setShape_movementDirection, character.linearVelocity, 1 / Math.sqrt(velocityLenSq));
        } else {
            vec3.zero(_setShape_movementDirection);
        }

        // create temporary contacts array for testing new shape
        const tempContacts: CharacterContact[] = [];

        // test collisions with new shape using temporary array
        const oldShape = character.shape;
        character.shape = newShape;
        getContactsAtPosition(world, character, character.position, _setShape_movementDirection, filter, listener, tempContacts);
        character.shape = oldShape;

        // validate contacts
        for (let i = 0; i < tempContacts.length; i++) {
            const contact = tempContacts[i];
            // reject if penetrating deeper than threshold and not a sensor
            if (contact.distance < -maxPenetrationDepth && !contact.isSensor) {
                // shape change rejected - release temp contacts
                releaseAllContacts(tempContacts);
                return false;
            }
        }

        // shape change is valid - track contact changes
        resetContactTracking(character);
        character.shape = newShape;
        // release old contacts and use temp contacts
        releaseAllContacts(character.contacts);
        character.contacts = tempContacts;
        updateSupportingContact(world, character, true /* skipContactVelocityCheck */, character.lastDeltaTime, listener);
        finalizeContactTracking(world, character, listener);
    } else {
        // no penetration test needed - just change the shape
        character.shape = newShape;
    }

    // update inner body shape if it exists
    if (character.innerRigidBodyId !== INVALID_BODY_ID) {
        const innerBody = rigidBody.get(world, character.innerRigidBodyId);
        if (innerBody) {
            innerBody.shape = newShape;
            rigidBody.updateShape(world, innerBody);
        }
    }

    return true;
}

const _move_gravityImpulse = /* @__PURE__ */ vec3.create();

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
export function move(
    world: World,
    character: KCC,
    deltaTime: number,
    gravity: Vec3,
    listener: CharacterListener | undefined,
    filter: Filter,
): void {
    // early exit if no time to simulate
    if (deltaTime <= 0) {
        return;
    }

    // store delta time for supporting contact velocity check
    character.lastDeltaTime = deltaTime;

    // slide the shape through the world with collision resolution
    moveShape(
        world,
        character,
        character.position, // in/out - position is updated
        character.linearVelocity,
        deltaTime,
        filter,
        listener,
    );

    // determine ground state from active contacts
    updateSupportingContact(world, character, false /* skipContactVelocityCheck */, deltaTime, listener);

    // sync inner body transform to character position/rotation
    updateInnerBodyTransform(world, character);

    // apply gravity impulse to ground body
    if (character.ground.bodyId !== INVALID_BODY_ID && character.mass > 0) {
        const groundBody = rigidBody.get(world, character.ground.bodyId);

        if (groundBody && groundBody.motionType === MotionType.DYNAMIC) {
            // calculate impulse magnitude from gravity component toward ground
            const normalDotGravity = vec3.dot(character.ground.normal, gravity);

            // only apply if gravity is pushing into the ground (negative dot)
            if (normalDotGravity < 0) {
                const impulseScale = -(((character.mass * normalDotGravity) / vec3.len(gravity)) * deltaTime);
                vec3.scale(_move_gravityImpulse, gravity, impulseScale);
                rigidBody.addImpulseAtPosition(world, groundBody, _move_gravityImpulse, character.ground.position);
            }
        }
    }
}

const _moveToContact_movementDirection = /* @__PURE__ */ vec3.create();

/**
 * Moves character to a contact position and updates state.
 *
 * 1. Set position
 * 2. Trigger contact added callback
 * 3. Get contacts at new position
 * 4. Ensure the contact is marked as colliding
 * 5. Store active contacts (with nested tracking)
 * 6. Update inner body transform
 *
 * @param world the physics world
 * @param character the character controller
 * @param position new position to move to
 * @param contact the contact we're moving to
 * @param filter collision filter
 * @param listener optional listener for callbacks
 */
function moveToContact(
    world: World,
    character: KCC,
    position: Vec3,
    contact: CharacterContact,
    filter: Filter,
    listener: CharacterListener | undefined,
): void {
    // set the new position
    setPosition(world, character, position);

    // trigger contact added callback
    const settings: CharacterContactSettings = {
        canPushCharacter: true,
        canReceiveImpulses: true,
    };
    contactAdded(world, character, contact, listener, settings);

    // get contacts at new position
    // use normalized velocity or zero vector
    if (vec3.squaredLength(character.linearVelocity) > 1e-12) {
        vec3.normalize(_moveToContact_movementDirection, character.linearVelocity);
    } else {
        vec3.zero(_moveToContact_movementDirection);
    }
    getContactsAtPosition(
        world,
        character,
        character.position,
        _moveToContact_movementDirection,
        filter,
        listener,
        character.contacts,
    );

    // ensure the contact we moved to is marked as colliding
    const contacts = character.contacts;
    let foundContact = false;
    for (let i = 0; i < contacts.length; i++) {
        const c = contacts[i];
        if (c.bodyId === contact.bodyId && c.subShapeId === contact.subShapeId) {
            c.hadCollision = true;
            foundContact = true;
        }
    }

    // if contact not found in new position, add it manually
    if (!foundContact) {
        const newContact = acquireContact(character.contacts);
        vec3.copy(newContact.position, contact.position);
        vec3.copy(newContact.linearVelocity, contact.linearVelocity);
        vec3.copy(newContact.contactNormal, contact.contactNormal);
        vec3.copy(newContact.surfaceNormal, contact.surfaceNormal);
        newContact.distance = contact.distance;
        newContact.fraction = contact.fraction;
        newContact.bodyId = contact.bodyId;
        newContact.subShapeId = contact.subShapeId;
        newContact.motionType = contact.motionType;
        newContact.hadCollision = true;
        newContact.wasDiscarded = false;
        newContact.canPushCharacter = contact.canPushCharacter;
    }

    // store active contacts
    updateSupportingContact(world, character, true /* skipContactVelocityCheck */, character.lastDeltaTime, listener);

    // ensure that the rigid body ends up at the new position
    updateInnerBodyTransform(world, character);
}

const _stickToFloor_contact = /* @__PURE__ */ createCharacterContact();
const _stickToFloor_newPosition = /* @__PURE__ */ vec3.create();

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
export function stickToFloor(
    world: World,
    character: KCC,
    stepDown: Vec3,
    filter: Filter,
    listener: CharacterListener | undefined,
): boolean {
    // try to find the floor
    if (
        !getFirstContactForSweep(
            _stickToFloor_contact,
            world,
            character,
            character.position,
            stepDown,
            filter,
            listener,
            undefined,
        )
    ) {
        // no floor found
        return false;
    }

    // calculate new position
    vec3.scaleAndAdd(_stickToFloor_newPosition, character.position, stepDown, _stickToFloor_contact.fraction);

    // move to the contact
    moveToContact(world, character, _stickToFloor_newPosition, _stickToFloor_contact, filter, listener);

    return true;
}

const _walkStairs_contact = /* @__PURE__ */ createCharacterContact();
const _walkStairs_testContact = /* @__PURE__ */ createCharacterContact();
const _walkStairs_up = /* @__PURE__ */ vec3.create();
const _walkStairs_down = /* @__PURE__ */ vec3.create();
const _walkStairs_upPosition = /* @__PURE__ */ vec3.create();
const _walkStairs_newPosition = /* @__PURE__ */ vec3.create();
const _walkStairs_testPosition = /* @__PURE__ */ vec3.create();
const _walkStairs_horizontalMovement = /* @__PURE__ */ vec3.create();
const _walkStairs_characterVelocity = /* @__PURE__ */ vec3.create();
const _walkStairs_horizontalVelocity = /* @__PURE__ */ vec3.create();
const _walkStairs_steepSlopeNormalsPool: Vec3[] = [];
for (let i = 0; i < 16; i++) {
    _walkStairs_steepSlopeNormalsPool.push(vec3.create());
}
let _walkStairs_steepSlopeNormalsCount = 0;

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
export function walkStairs(
    world: World,
    character: KCC,
    deltaTime: number,
    stepUp: Vec3,
    stepForward: Vec3,
    stepForwardTest: Vec3,
    stepDownExtra: Vec3,
    filter: Filter,
    listener: CharacterListener | undefined,
): boolean {
    /* step 1: move up */
    vec3.copy(_walkStairs_up, stepUp);
    if (
        getFirstContactForSweep(
            _walkStairs_contact,
            world,
            character,
            character.position,
            _walkStairs_up,
            filter,
            listener,
            undefined,
        )
    ) {
        if (_walkStairs_contact.fraction < 1e-6) {
            // no movement possible, cancel
            return false;
        }
        // limit up movement to first contact
        vec3.scale(_walkStairs_up, _walkStairs_up, _walkStairs_contact.fraction);
    }
    vec3.add(_walkStairs_upPosition, character.position, _walkStairs_up);

    /* step 2: collect steep slope normals before moveShape updates contacts */
    // calculate character velocity from step forward
    vec3.scale(_walkStairs_characterVelocity, stepForward, 1 / deltaTime);

    // get horizontal velocity
    const dotUp = vec3.dot(_walkStairs_characterVelocity, character.up);
    vec3.scaleAndAdd(_walkStairs_horizontalVelocity, _walkStairs_characterVelocity, character.up, -dotUp);

    // collect steep slopes we're pushing against
    _walkStairs_steepSlopeNormalsCount = 0;
    const contacts = character.contacts;
    for (let i = 0; i < contacts.length; i++) {
        const c = contacts[i];
        if (!c.hadCollision || c.wasDiscarded) continue;
        if (!isSlopeTooSteep(character, c.surfaceNormal)) continue;

        // check if pushing into contact
        const relVelDotNormal =
            c.surfaceNormal[0] * (_walkStairs_horizontalVelocity[0] - c.linearVelocity[0]) +
            c.surfaceNormal[1] * (_walkStairs_horizontalVelocity[1] - c.linearVelocity[1]) +
            c.surfaceNormal[2] * (_walkStairs_horizontalVelocity[2] - c.linearVelocity[2]);

        if (relVelDotNormal < 0) {
            // copy normal to pre-allocated pool (contacts will be modified by moveShape)
            if (_walkStairs_steepSlopeNormalsCount < _walkStairs_steepSlopeNormalsPool.length) {
                vec3.copy(_walkStairs_steepSlopeNormalsPool[_walkStairs_steepSlopeNormalsCount], c.surfaceNormal);
                _walkStairs_steepSlopeNormalsCount++;
            }
        }
    }

    if (_walkStairs_steepSlopeNormalsCount === 0) {
        // no steep slopes to walk, cancel
        return false;
    }

    /* step 3: horizontal movement at elevated position */
    vec3.copy(_walkStairs_newPosition, _walkStairs_upPosition);
    moveShape(
        world,
        character,
        _walkStairs_newPosition, // in/out
        _walkStairs_characterVelocity,
        deltaTime,
        filter,
        listener,
    );

    // calculate horizontal movement
    vec3.sub(_walkStairs_horizontalMovement, _walkStairs_newPosition, _walkStairs_upPosition);
    const horizontalMovementSq = vec3.squaredLength(_walkStairs_horizontalMovement);
    if (horizontalMovementSq < 1e-8) {
        // no horizontal movement, cancel
        return false;
    }

    /* step 4: check if we made progress toward any steep slope */
    // we need to have moved at least 5% of step forward against a steep slope
    const stepForwardLen = vec3.len(stepForward);
    const maxDot = -0.05 * stepForwardLen;
    let madeProgress = false;
    for (let i = 0; i < _walkStairs_steepSlopeNormalsCount; i++) {
        const dot = vec3.dot(_walkStairs_steepSlopeNormalsPool[i], _walkStairs_horizontalMovement);
        if (dot < maxDot) {
            madeProgress = true;
            break;
        }
    }
    if (!madeProgress) {
        // just sliding along slope, cancel
        return false;
    }

    /* step 5: move down toward floor */
    // travel same distance down as we went up, plus extra
    vec3.negate(_walkStairs_down, _walkStairs_up);
    vec3.add(_walkStairs_down, _walkStairs_down, stepDownExtra);

    if (
        !getFirstContactForSweep(
            _walkStairs_contact,
            world,
            character,
            _walkStairs_newPosition,
            _walkStairs_down,
            filter,
            listener,
            undefined,
        )
    ) {
        // no floor found, we're in mid air, cancel
        return false;
    }

    /* step 6: validate floor is walkable */
    if (isSlopeTooSteep(character, _walkStairs_contact.surfaceNormal)) {
        // floor is too steep - try forward test
        if (vec3.squaredLength(stepForwardTest) < 1e-12) {
            // no test position provided, cancel
            return false;
        }

        // move further forward to test floor
        vec3.copy(_walkStairs_testPosition, _walkStairs_upPosition);
        vec3.scale(_walkStairs_characterVelocity, stepForwardTest, 1 / deltaTime);
        moveShape(
            world,
            character,
            _walkStairs_testPosition, // in/out
            _walkStairs_characterVelocity,
            deltaTime,
            filter,
            listener,
        );

        // check if we moved further than before
        vec3.sub(_walkStairs_horizontalMovement, _walkStairs_testPosition, _walkStairs_upPosition);
        const testHorizontalMovementSq = vec3.squaredLength(_walkStairs_horizontalMovement);
        if (testHorizontalMovementSq <= horizontalMovementSq + 1e-8) {
            // didn't move further, cancel
            return false;
        }

        // sweep down at test position
        if (
            !getFirstContactForSweep(
                _walkStairs_testContact,
                world,
                character,
                _walkStairs_testPosition,
                _walkStairs_down,
                filter,
                listener,
                undefined,
            )
        ) {
            // no floor at test position, cancel
            return false;
        }

        // check if test floor is walkable
        if (isSlopeTooSteep(character, _walkStairs_testContact.surfaceNormal)) {
            // test floor also too steep, cancel
            return false;
        }
    }

    /* step 7: calculate final position and move there */
    vec3.scaleAndAdd(_walkStairs_newPosition, _walkStairs_newPosition, _walkStairs_down, _walkStairs_contact.fraction);
    moveToContact(world, character, _walkStairs_newPosition, _walkStairs_contact, filter, listener);

    /* step 8: override ground state to ON_GROUND */
    // it's possible the contact normal was too steep, but the forward test found a valid floor
    character.ground.state = GroundState.ON_GROUND;

    return true;
}

const _update_oldPosition = /* @__PURE__ */ vec3.create();
const _update_desiredVelocity = /* @__PURE__ */ vec3.create();
const _update_canceledVelocity = /* @__PURE__ */ vec3.create();
const _update_desiredHorizontalStep = /* @__PURE__ */ vec3.create();
const _update_achievedHorizontalStep = /* @__PURE__ */ vec3.create();
const _update_stepForward = /* @__PURE__ */ vec3.create();
const _update_stepForwardNormalized = /* @__PURE__ */ vec3.create();
const _update_stepForwardTest = /* @__PURE__ */ vec3.create();
const _update_groundHorizontal = /* @__PURE__ */ vec3.create();

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
export function update(
    world: World,
    character: KCC,
    deltaTime: number,
    gravity: Vec3,
    settings: UpdateSettings,
    listener: CharacterListener | undefined,
    filter: Filter,
): void {
    // reset contact tracking at start of frame
    resetContactTracking(character);

    // save desired velocity and cancel velocity toward steep slopes
    vec3.copy(_update_desiredVelocity, character.linearVelocity);
    cancelVelocityTowardsSteepSlopes(character, _update_desiredVelocity, _update_canceledVelocity);
    vec3.copy(character.linearVelocity, _update_canceledVelocity);

    // remember old position
    vec3.copy(_update_oldPosition, character.position);

    // track if on ground before update
    let groundToAir = isSupported(character);

    // main update
    move(world, character, deltaTime, gravity, listener, filter);

    // check if we went from supported to not supported
    if (isSupported(character)) {
        groundToAir = false;
    }

    // stick to floor if we went from ground to air
    if (groundToAir && vec3.squaredLength(settings.stickToFloorStepDown) > 1e-12) {
        // check we're not moving up
        vec3.sub(_update_achievedHorizontalStep, character.position, _update_oldPosition);
        const velocityUp = vec3.dot(_update_achievedHorizontalStep, character.up) / deltaTime;
        if (velocityUp <= 1e-6) {
            stickToFloor(world, character, settings.stickToFloorStepDown, filter, listener);
        }
    }

    // walk stairs if enabled
    if (vec3.squaredLength(settings.walkStairsStepUp) > 1e-12) {
        // calculate desired horizontal movement
        vec3.scale(_update_desiredHorizontalStep, _update_desiredVelocity, deltaTime);
        const desiredDotUp = vec3.dot(_update_desiredHorizontalStep, character.up);
        vec3.scaleAndAdd(_update_desiredHorizontalStep, _update_desiredHorizontalStep, character.up, -desiredDotUp);
        const desiredHorizontalStepLen = vec3.len(_update_desiredHorizontalStep);

        if (desiredHorizontalStepLen > 0) {
            // calculate achieved horizontal movement
            vec3.sub(_update_achievedHorizontalStep, character.position, _update_oldPosition);
            const achievedDotUp = vec3.dot(_update_achievedHorizontalStep, character.up);
            vec3.scaleAndAdd(_update_achievedHorizontalStep, _update_achievedHorizontalStep, character.up, -achievedDotUp);

            // only count movement in direction of desired movement
            vec3.scale(_update_stepForwardNormalized, _update_desiredHorizontalStep, 1 / desiredHorizontalStepLen);
            const achievedInDesiredDir = Math.max(0, vec3.dot(_update_achievedHorizontalStep, _update_stepForwardNormalized));
            vec3.scale(_update_achievedHorizontalStep, _update_stepForwardNormalized, achievedInDesiredDir);
            const achievedHorizontalStepLen = vec3.len(_update_achievedHorizontalStep);

            // check if we didn't achieve desired movement and we're blocked by steep slopes
            if (
                achievedHorizontalStepLen + 1e-4 < desiredHorizontalStepLen &&
                hasSteepSlopesToWalk(character, _update_desiredVelocity)
            ) {
                // calculate step forward (clamped to minimum)
                const stepForwardLen = Math.max(
                    settings.walkStairsMinStepForward,
                    desiredHorizontalStepLen - achievedHorizontalStepLen,
                );
                vec3.scale(_update_stepForward, _update_stepForwardNormalized, stepForwardLen);

                // calculate step forward test direction
                // start with ground normal projected to horizontal
                vec3.copy(_update_groundHorizontal, character.ground.normal);
                vec3.negate(_update_groundHorizontal, _update_groundHorizontal);
                const groundDotUp = vec3.dot(_update_groundHorizontal, character.up);
                vec3.scaleAndAdd(_update_groundHorizontal, _update_groundHorizontal, character.up, -groundDotUp);

                const groundHorizontalLenSq = vec3.squaredLength(_update_groundHorizontal);
                if (groundHorizontalLenSq > 1e-12) {
                    vec3.scale(_update_stepForwardTest, _update_groundHorizontal, 1 / Math.sqrt(groundHorizontalLenSq));
                } else {
                    vec3.copy(_update_stepForwardTest, _update_stepForwardNormalized);
                }

                // if ground normal direction differs too much from movement, use movement direction
                if (
                    vec3.dot(_update_stepForwardTest, _update_stepForwardNormalized) < settings.walkStairsCosAngleForwardContact
                ) {
                    vec3.copy(_update_stepForwardTest, _update_stepForwardNormalized);
                }

                // scale to test distance
                vec3.scale(_update_stepForwardTest, _update_stepForwardTest, settings.walkStairsStepForwardTest);

                // try walking stairs
                walkStairs(
                    world,
                    character,
                    deltaTime,
                    settings.walkStairsStepUp,
                    _update_stepForward,
                    _update_stepForwardTest,
                    settings.walkStairsStepDownExtra,
                    filter,
                    listener,
                );
            }
        }
    }

    // finalize contact tracking and fire removal callbacks at end of frame
    finalizeContactTracking(world, character, listener);
}
