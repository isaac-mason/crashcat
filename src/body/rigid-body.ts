import { type Box3, box3, type Mat4, mat4, type Quat, quat, type Vec3, vec3 } from 'mathcat';
import * as broadphase from '../broadphase/broadphase';
import { MaterialCombineMode } from '../constraints/combine-material';
import type { ConstraintId } from '../constraints/constraint-id';
import * as constraints from '../constraints/constraints';
import * as contacts from '../contacts';
import * as filter from '../filter';
import {
    computeMassProperties,
    type GetLeafShapeResult,
    type GetSubShapeTransformedShapeResult,
    shapeDefs,
    type Shape,
} from '../shapes/shapes';
import * as emptyShape from '../shapes/empty-shape';
import { getShapeSurfaceNormal } from '../shapes/shapes';
import type { World } from '../world';
import { type BodyId, getBodyIdIndex, getBodyIdSequence, INVALID_BODY_ID, SEQUENCE_MASK, serBodyId } from './body-id';
import * as massProperties from './mass-properties';
import * as motionProperties from './motion-properties';
import { MotionType } from './motion-type';
import * as sleepModule from './sleep';
import { addBodyToActiveBodies, INACTIVE_BODY_INDEX, removeBodyFromActiveBodies, resetSleepTimer } from './sleep';
import * as subShape from './sub-shape';

/** describes how mass properties are calculated for a rigid body */
export enum MassPropertiesOverride {
    /** calculate both mass and inertia from shape density */
    CALCULATE_MASS_AND_INERTIA = 0,
    /** override mass, calculate and scale inertia from shape */
    CALCULATE_INERTIA = 1,
    /** use provided mass and inertia as-is */
    MASS_AND_INERTIA_PROVIDED = 2,
}

/** settings for creating a new rigid body */
export type RigidBodySettings = {
    /** shape for the body. Use null to create a body with an EmptyShape */
    shape: Shape | null;

    /** object layer the body belongs to */
    objectLayer: number;

    /** motion type (static, kinematic, dynamic) */
    motionType: MotionType;

    /** an optional user-defined field, defaults to null */
    userData?: unknown;

    /** starting position @default [0,0,0] */
    position?: Vec3;

    /** starting quaternion @default [0,0,0,1] */
    quaternion?: Quat;

    /** motion quality @default MotionQuality.DISCRETE @see motionProperties.MotionQuality.DISCRETE */
    motionQuality?: motionProperties.MotionQuality;

    /** allowed degrees of freedom @see DEFAULT_RIGID_BODY_SETTINGS */
    allowedDegreesOfFreedom?: number;

    /** factor for gravity applied to this body @see DEFAULT_RIGID_BODY_SETTINGS */
    gravityFactor?: number;
    /** linear damping @see DEFAULT_RIGID_BODY_SETTINGS */
    linearDamping?: number;
    /** angular damping @see DEFAULT_RIGID_BODY_SETTINGS */
    angularDamping?: number;
    /** max linear velocity @see DEFAULT_RIGID_BODY_SETTINGS */
    maxLinearVelocity?: number;
    /** max angular velocity @see DEFAULT_RIGID_BODY_SETTINGS */
    maxAngularVelocity?: number;

    /** body friction @see DEFAULT_RIGID_BODY_SETTINGS */
    friction?: number;
    /** body restitution @see DEFAULT_RIGID_BODY_SETTINGS */
    restitution?: number;

    /** the material combine mode for friction @see DEFAULT_RIGID_BODY_SETTINGS */
    frictionCombineMode?: MaterialCombineMode;
    /** the material combine mode for restitution @see DEFAULT_RIGID_BODY_SETTINGS */
    restitutionCombineMode?: MaterialCombineMode;

    /** groups this body is part of @see DEFAULT_RIGID_BODY_SETTINGS */
    collisionGroup?: number;
    /** groups this body collides with @see DEFAULT_RIGID_BODY_SETTINGS */
    collisionMask?: number;

    /** whether to use manifold reduction for this body @see DEFAULT_RIGID_BODY_SETTINGS @see RigidBody.useManifoldReduction */
    useManifoldReduction?: boolean;

    /** whether this body is allowed to sleep @see DEFAULT_RIGID_BODY_SETTINGS */
    allowSleeping?: boolean;

    /** @default false */
    sensor?: boolean;

    /** enable enhanced internal edge removal to eliminate ghost collisions on internal edges @default false */
    enhancedInternalEdgeRemoval?: boolean;

    /** if true, and the body MotionType is KINEMATIC, this body will collide with non-dynamic bodies @see DEFAULT_RIGID_BODY_SETTINGS */
    collideKinematicVsNonDynamic?: boolean;

    /**
     * Override mass only. Inertia will be calculated from the shape and scaled to match this mass.
     * This is ignored if massPropertiesOverride is provided.
     */
    mass?: number;

    /**
     * Override mass properties for this body (both mass and inertia).
     * Useful for triangle meshes and other complex shapes where mass properties
     * cannot be derived from the shape alone, or to use custom values.
     * If not provided, mass properties are computed from the shape.
     * If provided, it will be copied into the body mass properties (not referenced).
     * Takes precedence over the mass field.
     */
    massPropertiesOverride?: massProperties.MassProperties;
};

export const DEFAULT_RIGID_BODY_SETTINGS = {
    motionQuality: motionProperties.MotionQuality.DISCRETE,
    gravityFactor: 1.0,
    linearDamping: 0.05,
    angularDamping: 0.05,
    maxLinearVelocity: 500.0,
    maxAngularVelocity: 0.25 * Math.PI * 60.0,
    allowSleeping: true,
    friction: 0.2,
    restitution: 0.0,
    frictionCombineMode: MaterialCombineMode.GEOMETRIC_MEAN,
    restitutionCombineMode: MaterialCombineMode.MAX,
    allowedDegreesOfFreedom: 0b111111,
    useManifoldReduction: true,
    collisionGroup: 0xffffffff,
    collisionMask: 0xffffffff,
    sensor: false,
    enhancedInternalEdgeRemoval: false,
    collideKinematicVsNonDynamic: false,
};

/** physics body in a physics world */
export type RigidBody = {
    /** @internal whether this body is currently pooled, in which case it should be ignored */
    _pooled: boolean;

    /** unique body identifier */
    id: BodyId;

    /** index from body ID (index into World.bodies array) */
    index: number;

    /** sequence number from body ID (for fast stale reference detection) */
    sequence: number;

    /** an optional user-defined field, defaults to null */
    userData: unknown | null;

    /** the body's shape */
    shape: Shape;

    /** the body's world-space position at the shape origin, @see centerOfMassPosition for the center of mass position */
    position: Vec3;

    /** the body's orientation */
    quaternion: Quat;

    /** world-space center of mass position */
    centerOfMassPosition: Vec3;

    /** the body's motion type */
    motionType: MotionType;

    /** mass properties (mass, inertia tensor, etc.) */
    massProperties: massProperties.MassProperties;

    /** how mass properties are calculated/overridden */
    massPropertiesOverride: MassPropertiesOverride;

    /** motion properties (velocity, forces, damping, etc.) */
    motionProperties: motionProperties.MotionProperties;

    /** coefficient of friction */
    friction: number;

    /** coefficient of restitution (bounciness) */
    restitution: number;

    /** how to combine friction when this body collides with another */
    frictionCombineMode: MaterialCombineMode;

    /** how to combine restitution when this body collides with another */
    restitutionCombineMode: MaterialCombineMode;

    /** collision group bitfield - "I am in these groups" */
    collisionGroup: number;

    /** collision mask bitfield - "I collide with these groups" */
    collisionMask: number;

    /**
     * whether to use manifold reduction for this body.
     * if you requires tracking exactly which sub shape ids are in contact, you can turn off manifold reduction. note that this comes at a performance cost.
     * manifold reduction by default will combine contacts with similar normals that come from different sub shape ids (e.g. different triangles in a mesh shape or different compound shapes).
     */
    useManifoldReduction: boolean;

    /** whether this body is a sensor (detects collisions but does not respond physically) */
    sensor: boolean;

    /** enable enhanced internal edge removal to eliminate ghost collisions on internal edges */
    enhancedInternalEdgeRemoval: boolean;

    /** whether kinematic bodies can collide with static/other kinematic bodies */
    collideKinematicVsNonDynamic: boolean;

    /** @readonly whether the body is currently sleeping */
    sleeping: boolean;

    /** world-space AABB */
    aabb: Box3;

    /** object layer this body is in */
    objectLayer: number;

    /** broadphase layer this body is in, -1 if none */
    broadphaseLayer: number;

    /** which broadphase dbvt node contains this body */
    dbvtNode: number;

    /**
     * Head of the intrusive doubly-linked list of contacts involving this body.
     * Packed key: (contactId << 1) | edgeIndex.
     * Use INVALID_CONTACT_KEY (-1) for empty list.
     */
    headContactKey: number;

    /** number of contacts involving this body */
    contactCount: number;

    /** island index this body belongs to (set during island building, -1 if not in an island) */
    islandIndex: number;

    /** active body index for island building (sequential index 0,1,2... for dynamic+kinematic bodies, -1 for static) */
    activeIndex: number;

    /** CCD body index (index into world.ccd.ccdBodies array, -1 if not using CCD this frame) */
    ccdBodyIndex: number;

    /** constraint IDs referencing this body (for cleanup on body removal) */
    constraintIds: ConstraintId[];
};

/** creates a new body with default properties */
function makeRigidBody(): RigidBody {
    return {
        _pooled: true,
        id: INVALID_BODY_ID,
        index: -1,
        sequence: -1,
        userData: null,
        position: vec3.create(),
        quaternion: quat.create(),
        centerOfMassPosition: vec3.create(),
        massProperties: massProperties.create(),
        massPropertiesOverride: MassPropertiesOverride.CALCULATE_MASS_AND_INERTIA,
        motionProperties: motionProperties.create(),
        friction: 0,
        restitution: 0,
        frictionCombineMode: MaterialCombineMode.AVERAGE,
        restitutionCombineMode: MaterialCombineMode.AVERAGE,
        collisionGroup: 0,
        collisionMask: 0,
        useManifoldReduction: true,
        motionType: MotionType.STATIC,
        sleeping: false,
        sensor: false,
        enhancedInternalEdgeRemoval: false,
        collideKinematicVsNonDynamic: false,
        shape: null! as Shape,
        aabb: box3.create(),
        objectLayer: 0,
        broadphaseLayer: -1,
        dbvtNode: -1,
        headContactKey: contacts.INVALID_CONTACT_KEY,
        contactCount: 0,
        islandIndex: -1,
        activeIndex: INACTIVE_BODY_INDEX,
        ccdBodyIndex: -1,
        constraintIds: [],
    };
}

const VEC3_ZERO = vec3.create();
const QUAT_IDENTITY = quat.create();

const _setRigidBody_tmpMassProperties = massProperties.create();

/** initializes a body object with the given settings */
function setRigidBody(body: RigidBody, o: RigidBodySettings): void {
    body._pooled = false;

    body.userData = o.userData ?? null;

    vec3.copy(body.position, o.position ?? VEC3_ZERO);
    quat.copy(body.quaternion, o.quaternion ?? QUAT_IDENTITY);
    vec3.zero(body.centerOfMassPosition);

    // handle mass properties with three-tier precedence:
    // 1. massPropertiesOverride (full control)
    // 2. mass (calculate and scale inertia)
    // 3. auto-calculate from shape (default)
    if (o.massPropertiesOverride) {
        // full override - use provided mass and inertia as-is
        massProperties.copy(body.massProperties, o.massPropertiesOverride);
        body.massPropertiesOverride = MassPropertiesOverride.MASS_AND_INERTIA_PROVIDED;
    } else if (o.mass !== undefined) {
        // mass-only override - will calculate inertia from shape and scale it
        // this is deferred to updateShape() where we have access to the shape
        body.massProperties.mass = o.mass;
        body.massPropertiesOverride = MassPropertiesOverride.CALCULATE_INERTIA;
    } else {
        // auto-calculate from shape
        massProperties.reset(body.massProperties);
        body.massPropertiesOverride = MassPropertiesOverride.CALCULATE_MASS_AND_INERTIA;
    }

    const mp = body.motionProperties;

    // reset velocities and accumulators (important for pooled bodies)
    vec3.zero(mp.linearVelocity);
    vec3.zero(mp.angularVelocity);
    vec3.zero(mp.force);
    vec3.zero(mp.torque);

    mp.gravityFactor = o.gravityFactor ?? DEFAULT_RIGID_BODY_SETTINGS.gravityFactor;
    mp.linearDamping = o.linearDamping ?? DEFAULT_RIGID_BODY_SETTINGS.linearDamping;
    mp.angularDamping = o.angularDamping ?? DEFAULT_RIGID_BODY_SETTINGS.angularDamping;
    mp.maxLinearVelocity = o.maxLinearVelocity ?? DEFAULT_RIGID_BODY_SETTINGS.maxLinearVelocity;
    mp.maxAngularVelocity = o.maxAngularVelocity ?? DEFAULT_RIGID_BODY_SETTINGS.maxAngularVelocity;
    mp.allowedDegreesOfFreedom = o.allowedDegreesOfFreedom ?? DEFAULT_RIGID_BODY_SETTINGS.allowedDegreesOfFreedom;
    mp.allowSleeping = o.allowSleeping ?? DEFAULT_RIGID_BODY_SETTINGS.allowSleeping;
    mp.motionQuality = o.motionQuality ?? DEFAULT_RIGID_BODY_SETTINGS.motionQuality;

    body.friction = o.friction ?? DEFAULT_RIGID_BODY_SETTINGS.friction;
    body.restitution = o.restitution ?? DEFAULT_RIGID_BODY_SETTINGS.restitution;

    body.frictionCombineMode = o.frictionCombineMode ?? DEFAULT_RIGID_BODY_SETTINGS.frictionCombineMode;
    body.restitutionCombineMode = o.restitutionCombineMode ?? DEFAULT_RIGID_BODY_SETTINGS.restitutionCombineMode;

    body.collisionGroup = o.collisionGroup ?? DEFAULT_RIGID_BODY_SETTINGS.collisionGroup;
    body.collisionMask = o.collisionMask ?? DEFAULT_RIGID_BODY_SETTINGS.collisionMask;

    body.useManifoldReduction = o.useManifoldReduction ?? DEFAULT_RIGID_BODY_SETTINGS.useManifoldReduction;

    body.motionType = o.motionType;

    body.sleeping = false;

    body.sensor = o.sensor ?? DEFAULT_RIGID_BODY_SETTINGS.sensor;
    body.enhancedInternalEdgeRemoval =
        o.enhancedInternalEdgeRemoval ?? DEFAULT_RIGID_BODY_SETTINGS.enhancedInternalEdgeRemoval;
    body.collideKinematicVsNonDynamic =
        o.collideKinematicVsNonDynamic ?? DEFAULT_RIGID_BODY_SETTINGS.collideKinematicVsNonDynamic;

    // convert null shape to EmptyShape for convenience
    body.shape = o.shape ?? emptyShape.create();

    box3.empty(body.aabb);

    body.objectLayer = o.objectLayer;
    body.broadphaseLayer = -1;
    body.dbvtNode = -1;

    body.activeIndex = INACTIVE_BODY_INDEX;

    body.headContactKey = contacts.INVALID_CONTACT_KEY;
    body.contactCount = 0;
    body.islandIndex = -1;
    body.ccdBodyIndex = -1;

    body.constraintIds.length = 0;
}

/**
 * Creates a new body in the physics world
 * @param world the physics world
 * @param settings settings for the new body
 * @returns the newly created rigid body
 */
export function create(world: World, settings: RigidBodySettings): RigidBody {
    // get next sequence
    const sequence = world.bodies.nextSequence;
    world.bodies.nextSequence = (world.bodies.nextSequence + 1) & SEQUENCE_MASK;

    // get body from pool
    let index: number;
    let body: RigidBody;
    if (world.bodies.freeIndices.length > 0) {
        // reuse existing pooled body
        index = world.bodies.freeIndices.pop()!;
        body = world.bodies.pool[index];
    } else {
        // expand pool
        index = world.bodies.pool.length;
        body = makeRigidBody();
        world.bodies.pool.push(body);
    }

    // set body id, index, sequence
    body.id = serBodyId(index, sequence);
    body.index = index;
    body.sequence = sequence;

    // initialize rigid body with settings
    setRigidBody(body, settings);

    // update shape-related properties
    updateShape(world, body);

    // initialize sleep test spheres
    if (body.motionType === MotionType.DYNAMIC) {
        resetSleepTimer(body);
    }

    // add to broadphase
    broadphase.addBody(world.broadphase, body, world.settings.layers);

    // add to active bodies list if dynamic/kinematic and not sleeping
    if (body.motionType !== MotionType.STATIC && !body.sleeping) {
        addBodyToActiveBodies(world, body);
    }

    return body;
}

/**
 * Removes a body from the world
 * @returns true if the body was successfully removed, false if the body was already pooled (invalid)
 */
export function remove(world: World, body: RigidBody): boolean {
    // return false if already pooled
    if (body._pooled) {
        return false;
    }

    // remove from active bodies list
    removeBodyFromActiveBodies(world, body);

    // destroy all contacts involving this body
    contacts.destroyBodyContacts(world.contacts, world.bodies, body);

    // destroy all constraints involving this body
    constraints.destroyBodyConstraints(world, body);

    // remove from broadphase
    broadphase.removeBody(world.broadphase, body);

    // mark as pooled
    body._pooled = true;

    // reset user data eagerly
    body.userData = null;

    // return index to pool
    world.bodies.freeIndices.push(body.index);

    return true;
}

/**
 * Gets a body by ID with validation.
 * Returns undefined if the body doesn't exist, is pooled, or has a mismatched sequence number (stale reference).
 */
export function get(world: World, bodyId: BodyId): RigidBody | undefined {
    const index = getBodyIdIndex(bodyId);
    if (index < 0 || index >= world.bodies.pool.length) {
        return undefined;
    }

    const body = world.bodies.pool[index];
    if (!body || body._pooled) {
        return undefined;
    }

    // check sequence matches to catch stale references
    const sequence = getBodyIdSequence(bodyId);
    if (body.sequence !== sequence) {
        return undefined;
    }

    return body;
}

/**
 * Generator that yields all physics bodies in the world.
 * Skips inactive body objects in the world state that are pooled.
 */
export function* iterate(world: World): Generator<RigidBody> {
    for (const body of world.bodies.pool) {
        if (body && !body._pooled) {
            yield body;
        }
    }
}

const _getInverseInertia_rot = mat4.create();

/**
 * get the world-space inverse inertia matrix for a body.
 * for non-dynamic bodies, returns zero matrix.
 * @param out output Mat4 to store the result
 * @param body the rigid body
 * @returns out parameter
 */
export function getInverseInertia(out: Mat4, body: RigidBody): Mat4 {
    if (body.motionType !== MotionType.DYNAMIC) {
        return mat4.zero(out);
    }
    mat4.fromQuat(_getInverseInertia_rot, body.quaternion);
    return motionProperties.getInverseInertiaForRotation(out, body.motionProperties, _getInverseInertia_rot);
}

/**
 * Updates the world-space center of mass position based on the body's transform and shape.
 * Must be called whenever position, quaternion, or shape changes.
 */
export function updateCenterOfMassPosition(body: RigidBody): void {
    // copy shape-local center of mass
    vec3.copy(body.centerOfMassPosition, body.shape.centerOfMass);

    // rotate to world space
    vec3.transformQuat(body.centerOfMassPosition, body.centerOfMassPosition, body.quaternion);

    // translate to world position
    vec3.add(body.centerOfMassPosition, body.centerOfMassPosition, body.position);
}

const _updatePositionFromCenterOfMass_shapeCenterOfMassInWorldSpace = vec3.create();

/**
 * Updates the body's position (shape origin) based on centerOfMassPosition.
 * This derives position from centerOfMassPosition, which is the primary property modified by physics.
 * Formula: position = centerOfMassPosition - rotation × shape.centerOfMass
 */
export function updatePositionFromCenterOfMass(body: RigidBody): void {
    // get shape center of mass in world space
    const shapeCenterOfMassInWorldSpace = _updatePositionFromCenterOfMass_shapeCenterOfMassInWorldSpace;
    vec3.copy(shapeCenterOfMassInWorldSpace, body.shape.centerOfMass);
    vec3.transformQuat(shapeCenterOfMassInWorldSpace, shapeCenterOfMassInWorldSpace, body.quaternion);

    // position = centerOfMassPosition - shapeCenterOfMassInWorldSpace
    vec3.sub(body.position, body.centerOfMassPosition, shapeCenterOfMassInWorldSpace);
}

const _updateBodyAABB_corner = vec3.create();

/**
 * Updates the world-space AABB based on the body's transform and shape AABB.
 * Must be called whenever position, quaternion, or shape changes.
 */
function updateAABB(body: RigidBody): void {
    // transform shape AABB to world space by transforming all 8 corners
    const shapeAABB = body.shape.aabb;
    const min = shapeAABB[0];
    const max = shapeAABB[1];

    // transform first corner (min, min, min)
    vec3.set(_updateBodyAABB_corner, min[0], min[1], min[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);

    // initialize AABB with first corner
    vec3.copy(body.aabb[0], _updateBodyAABB_corner);
    vec3.copy(body.aabb[1], _updateBodyAABB_corner);

    // transform and expand by remaining 7 corners
    // (max, min, min)
    vec3.set(_updateBodyAABB_corner, max[0], min[1], min[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);
    box3.expandByPoint(body.aabb, body.aabb, _updateBodyAABB_corner);

    // (min, max, min)
    vec3.set(_updateBodyAABB_corner, min[0], max[1], min[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);
    box3.expandByPoint(body.aabb, body.aabb, _updateBodyAABB_corner);

    // (max, max, min)
    vec3.set(_updateBodyAABB_corner, max[0], max[1], min[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);
    box3.expandByPoint(body.aabb, body.aabb, _updateBodyAABB_corner);

    // (min, min, max)
    vec3.set(_updateBodyAABB_corner, min[0], min[1], max[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);
    box3.expandByPoint(body.aabb, body.aabb, _updateBodyAABB_corner);

    // (max, min, max)
    vec3.set(_updateBodyAABB_corner, max[0], min[1], max[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);
    box3.expandByPoint(body.aabb, body.aabb, _updateBodyAABB_corner);

    // (min, max, max)
    vec3.set(_updateBodyAABB_corner, min[0], max[1], max[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);
    box3.expandByPoint(body.aabb, body.aabb, _updateBodyAABB_corner);

    // (max, max, max)
    vec3.set(_updateBodyAABB_corner, max[0], max[1], max[2]);
    vec3.transformQuat(_updateBodyAABB_corner, _updateBodyAABB_corner, body.quaternion);
    vec3.add(_updateBodyAABB_corner, _updateBodyAABB_corner, body.position);
    box3.expandByPoint(body.aabb, body.aabb, _updateBodyAABB_corner);
}

/** updates body properties related to its shape, call this whenever the body's shape changes */
export function updateShape(world: World, body: RigidBody) {
    // handle mass properties based on mode
    if (body.massPropertiesOverride === MassPropertiesOverride.CALCULATE_MASS_AND_INERTIA) {
        // auto-calculate from shape
        computeMassProperties(body.massProperties, body.shape);
    } else if (body.massPropertiesOverride === MassPropertiesOverride.CALCULATE_INERTIA) {
        // mass-only override: calculate inertia from shape and scale to match mass
        const overrideMass = body.massProperties.mass;

        // calculate mass properties from shape
        computeMassProperties(_setRigidBody_tmpMassProperties, body.shape);

        // copy to body and scale to the override mass
        massProperties.copy(body.massProperties, _setRigidBody_tmpMassProperties);
        massProperties.scaleToMass(body.massProperties, overrideMass);
    }
    // else: MASS_AND_INERTIA_PROVIDED - use as-is, already set in setBody

    // only initialize motion properties for dynamic/kinematic bodies
    if (body.motionType !== MotionType.STATIC) {
        motionProperties.setMassProperties(
            body.motionProperties,
            body.motionProperties.allowedDegreesOfFreedom,
            body.massProperties,
        );
    }

    // recompute world-space center of mass after shape changes
    updateCenterOfMassPosition(body);

    // recompute world-space AABB after shape changes
    updateAABB(body);

    // notify broadphase of AABB change
    broadphase.updateBody(world.broadphase, body);
}

/**
 * Sets the body's position and recomputes the world-space center of mass and AABB.
 */
export function setPosition(world: World, body: RigidBody, position: Vec3, wake: boolean): void {
    // set position
    vec3.copy(body.position, position);

    // update body properties
    updateCenterOfMassPosition(body);
    updateAABB(body);

    // update broadphase
    broadphase.updateBody(world.broadphase, body);

    // optionally wake body
    if (wake) {
        sleepModule.wake(world, body);
    }
}

/**
 * Sets the body's orientation and recomputes the world-space center of mass and AABB.
 */
export function setQuaternion(world: World, body: RigidBody, quaternion: Quat, wake: boolean): void {
    // set quaternion
    quat.copy(body.quaternion, quaternion);

    // update body properties
    updateCenterOfMassPosition(body);
    updateAABB(body);

    // update broadphase
    broadphase.updateBody(world.broadphase, body);

    // optionally wake body
    if (wake) {
        sleepModule.wake(world, body);
    }
}

/**
 * Sets both the body's position and orientation, then recomputes the world-space center of mass and AABB.
 * This is the most efficient way to update both transform components in one operation.
 */
export function setTransform(world: World, body: RigidBody, position: Vec3, quaternion: Quat, wake: boolean): void {
    // set transform
    vec3.copy(body.position, position);
    quat.copy(body.quaternion, quaternion);

    // update body properties
    updateCenterOfMassPosition(body);
    updateAABB(body);

    // update broadphase
    broadphase.updateBody(world.broadphase, body);

    // optionally wake body
    if (wake) {
        sleepModule.wake(world, body);
    }
}

/**
 * Sets the body's object layer and updates broadphase accordingly.
 */
export function setObjectLayer(world: World, body: RigidBody, layer: number): void {
    body.objectLayer = layer;

    broadphase.reinsertBody(world.broadphase, body, world.settings.layers);
}

/**
 * Sets the body's motion type and updates active body tracking.
 * @param world The physics world
 * @param body The body to modify
 * @param motionType the new motion type
 * @param wake if true and changing to dynamic/kinematic, wakes the body
 */
export function setMotionType(world: World, body: RigidBody, motionType: MotionType, wake: boolean): void {
    const oldMotionType = body.motionType;
    if (oldMotionType === motionType) return;

    // update motion type
    body.motionType = motionType;

    // handle active bodies list transitions
    if (motionType === MotionType.STATIC) {
        // becoming static: remove from active list, zero velocities
        removeBodyFromActiveBodies(world, body);
        vec3.zero(body.motionProperties.linearVelocity);
        vec3.zero(body.motionProperties.angularVelocity);
    } else if (oldMotionType === MotionType.STATIC) {
        // was static, now dynamic/kinematic: add to active list if not sleeping
        if (!body.sleeping) {
            addBodyToActiveBodies(world, body);
        }
        // reset sleep timer for dynamic bodies
        if (motionType === MotionType.DYNAMIC) {
            resetSleepTimer(body);
        }
    }

    // optionally wake (only relevant for dynamic/kinematic)
    if (wake && motionType !== MotionType.STATIC) {
        sleepModule.wake(world, body);
    }

    // reinitialize motion properties if becoming dynamic/kinematic
    if (motionType !== MotionType.STATIC && oldMotionType === MotionType.STATIC) {
        motionProperties.setMassProperties(
            body.motionProperties,
            body.motionProperties.allowedDegreesOfFreedom,
            body.massProperties,
        );
    }
}

/**
 * Adds a force at the body's center of mass.
 * Force is accumulated and integrated over time during physics step.
 * Only affects dynamic bodies.
 * @param world The physics world
 * @param body The body to apply force to
 * @param force Force vector in world space
 * @param wake If true, wakes the body if sleeping
 */
export function addForce(world: World, body: RigidBody, force: Vec3, wake: boolean): void {
    if (body.motionType !== MotionType.DYNAMIC) return;
    motionProperties.addForce(body.motionProperties, force);
    if (wake) sleepModule.wake(world, body);
}

/**
 * Adds a torque (angular force) directly.
 * Torque is accumulated and integrated over time during physics step.
 * Only affects dynamic bodies.
 * @param world the physics world
 * @param body the body to apply torque to
 * @param torque torque vector in world space
 * @param wake if true, wakes the body if sleeping
 */
export function addTorque(world: World, body: RigidBody, torque: Vec3, wake: boolean): void {
    if (body.motionType !== MotionType.DYNAMIC) return;
    motionProperties.addTorque(body.motionProperties, torque);
    if (wake) sleepModule.wake(world, body);
}

/**
 * Adds a force at a specific world-space position.
 * Generates both linear force and torque: τ = r × F
 * where r is the moment arm (position - center of mass)
 * Only affects dynamic bodies.
 * @param world the physics world
 * @param body the body to apply force to
 * @param force force vector in world space
 * @param worldPosition position in world space where force is applied
 * @param wake if true, wakes the body if sleeping
 */
export function addForceAtPosition(world: World, body: RigidBody, force: Vec3, worldPosition: Vec3, wake: boolean): void {
    if (body.motionType !== MotionType.DYNAMIC) return;
    motionProperties.addForceAtPosition(body.motionProperties, force, worldPosition, body.centerOfMassPosition);
    if (wake) sleepModule.wake(world, body);
}

/**
 * Applies an impulse at the body's center of mass.
 * Impulse instantly changes velocity: Δv = impulse / mass
 * Only affects dynamic bodies.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to apply impulse to
 * @param impulse impulse vector in world space (kg·m/s)
 */
export function addImpulse(world: World, body: RigidBody, impulse: Vec3): void {
    if (body.motionType !== MotionType.DYNAMIC) return;
    if (body.motionProperties.invMass <= 0) return;
    motionProperties.addImpulse(body.motionProperties, impulse);
    sleepModule.wake(world, body);
}

/**
 * Applies an impulse at a specific world-space position.
 * Changes both linear and angular velocity.
 * Only affects dynamic bodies. Always wakes the body.
 * @param world the physics world
 * @param body the body to apply impulse to
 * @param impulse impulse vector in world space (kg·m/s)
 * @param worldPosition position in world space where impulse is applied
 */
export function addImpulseAtPosition(world: World, body: RigidBody, impulse: Vec3, worldPosition: Vec3): void {
    if (body.motionType !== MotionType.DYNAMIC) return;
    motionProperties.addImpulseAtPosition(
        body.motionProperties,
        impulse,
        worldPosition,
        body.centerOfMassPosition,
        body.quaternion,
    );
    sleepModule.wake(world, body);
}

/**
 * Applies an angular impulse (instant change to angular velocity).
 * Δω = I⁻¹ × impulse
 * Only affects dynamic bodies.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to apply angular impulse to
 * @param impulse angular impulse vector in world space (kg·m²/s)
 */
export function addAngularImpulse(world: World, body: RigidBody, impulse: Vec3): void {
    if (body.motionType !== MotionType.DYNAMIC) return;
    motionProperties.addAngularImpulse(body.motionProperties, impulse, body.quaternion);
    sleepModule.wake(world, body);
}

/**
 * Sets the linear velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to set velocity on
 * @param velocity new linear velocity vector in world space (m/s)
 */
export function setLinearVelocity(world: World, body: RigidBody, velocity: Vec3): void {
    if (body.motionType === MotionType.STATIC) return;
    motionProperties.setLinearVelocity(body.motionProperties, velocity);
    sleepModule.wake(world, body);
}

/**
 * Adds to the linear velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to modify
 * @param velocityDelta velocity change to add in world space (m/s)
 */
export function addLinearVelocity(world: World, body: RigidBody, velocityDelta: Vec3): void {
    if (body.motionType === MotionType.STATIC) return;
    motionProperties.addLinearVelocity(body.motionProperties, velocityDelta);
    sleepModule.wake(world, body);
}

/**
 * Sets the angular velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to set angular velocity on
 * @param velocity new angular velocity vector in world space (rad/s)
 */
export function setAngularVelocity(world: World, body: RigidBody, velocity: Vec3): void {
    if (body.motionType === MotionType.STATIC) return;
    motionProperties.setAngularVelocity(body.motionProperties, velocity);
    sleepModule.wake(world, body);
}

/**
 * Adds to the angular velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to modify
 * @param velocityDelta angular velocity change to add in world space (rad/s)
 */
export function addAngularVelocity(world: World, body: RigidBody, velocityDelta: Vec3): void {
    if (body.motionType === MotionType.STATIC) return;
    motionProperties.addAngularVelocity(body.motionProperties, velocityDelta);
    sleepModule.wake(world, body);
}

/**
 * Clears all accumulated forces and torques.
 * Should be called after each physics step.
 */
export function clearForces(body: RigidBody): void {
    vec3.zero(body.motionProperties.force);
    vec3.zero(body.motionProperties.torque);
}

const _moveKinematic_newCom = vec3.create();
const _moveKinematic_deltaPos = vec3.create();
const _moveKinematic_deltaRot = quat.create();
const _moveKinematic_quatConj = quat.create();

/**
 * Moves a kinematic body towards a target position and rotation over the given delta time.
 * @param body the kinematic body to move
 * @param targetPosition target world-space position
 * @param targetQuaternion target world-space quaternion
 * @param deltaTime time step over which to move the body
 */
export function moveKinematic(body: RigidBody, targetPosition: Vec3, targetQuaternion: Quat, deltaTime: number): void {
    // calculate center of mass at target situation
    const shapeCom = body.shape.centerOfMass;
    vec3.transformQuat(_moveKinematic_newCom, shapeCom, targetQuaternion);
    vec3.add(_moveKinematic_newCom, targetPosition, _moveKinematic_newCom);

    // calculate delta position (from current COM to new COM)
    vec3.sub(_moveKinematic_deltaPos, _moveKinematic_newCom, body.centerOfMassPosition);

    // calculate delta rotation: deltaRotation = targetRotation * currentRotation^-1
    quat.conjugate(_moveKinematic_quatConj, body.quaternion);
    quat.multiply(_moveKinematic_deltaRot, targetQuaternion, _moveKinematic_quatConj);

    motionProperties.moveKinematic(body.motionProperties, _moveKinematic_deltaPos, _moveKinematic_deltaRot, deltaTime);
}

const _getWorldSurfaceNormal_localPos = vec3.create();
const _getWorldSurfaceNormal_normal = vec3.create();
const _getWorldSurfaceNormal_invQuat = quat.create();

/**
 * Gets the surface normal at a world-space position on the body's shape.
 * The query is performed relative to the shape's local coordinate system (shape origin, not center of mass).
 */
export function getSurfaceNormal(out: Vec3, body: RigidBody, worldPosition: Vec3, subShapeId: number): Vec3 {
    // transform world position to local space (relative to shape origin)
    vec3.subtract(_getWorldSurfaceNormal_localPos, worldPosition, body.position);
    quat.conjugate(_getWorldSurfaceNormal_invQuat, body.quaternion);
    vec3.transformQuat(_getWorldSurfaceNormal_localPos, _getWorldSurfaceNormal_localPos, _getWorldSurfaceNormal_invQuat);

    // get normal in local space
    getShapeSurfaceNormal(_getWorldSurfaceNormal_normal, body.shape, _getWorldSurfaceNormal_localPos, subShapeId);

    // transform normal to world space (rotation only, no translation)
    vec3.transformQuat(out, _getWorldSurfaceNormal_normal, body.quaternion);

    return out;
}

/** create a GetLeafShapeResult object */
export function createGetLeafShapeResult(): GetLeafShapeResult {
    return {
        shape: null,
        remainder: subShape.EMPTY_SUB_SHAPE_ID,
    };
}

/**
 * Navigate to a leaf shape following SubShapeID hierarchy.
 * @param out result object to write to @see createGetLeafShapeResult
 * @param rootShape Initial shape to start navigation from
 * @param subShapeId sub shape id path
 */
export function getLeafShape(out: GetLeafShapeResult, rootShape: Shape, subShapeId: subShape.SubShapeId): void {
    const shapeDef = shapeDefs[rootShape.type];
    shapeDef.getLeafShape(out, rootShape, subShapeId);
}

/** create a GetSubShapeTransformedShapeResult object */
export function createGetSubShapeTransformedShapeResult(): GetSubShapeTransformedShapeResult {
    return {
        shape: null,
        position: vec3.create(),
        rotation: quat.create(),
        scale: vec3.create(),
        remainder: subShape.EMPTY_SUB_SHAPE_ID,
    };
}

/**
 * Get transformed shape for a specific SubShapeID path.
 * @param out Single output parameter: { shape, position, rotation, scale, remainder }
 * @param rootShape Initial shape to start navigation from
 * @param subShapeId SubShapeID path
 * @param position Initial position
 * @param rotation Initial rotation
 * @param scale Initial scale
 */
export function getSubShapeTransformedShape(
    out: GetSubShapeTransformedShapeResult,
    rootShape: Shape,
    subShapeId: subShape.SubShapeId,
    position: Vec3,
    rotation: Quat,
    scale: Vec3,
): void {
    // initialize output with input transforms
    vec3.copy(out.position, position);
    quat.copy(out.rotation, rotation);
    vec3.copy(out.scale, scale);

    // delegate to shape implementation
    const shapeDef = shapeDefs[rootShape.type];
    shapeDef.getSubShapeTransformedShape(out, rootShape, subShapeId);
}

const _getPointVelocity_pointRelativeToCOM = vec3.create();

/**
 * Get velocity of a point on the body (point relative to center of mass).
 * Returns zero for static bodies.
 *
 * @param out output vector for velocity
 * @param body body to get velocity from
 * @param pointRelativeToCOM point position relative to center of mass
 * @returns out parameter
 */
export function getVelocityAtPointCOM(out: Vec3, body: RigidBody, pointRelativeToCOM: Vec3): Vec3 {
    if (body.motionType !== MotionType.STATIC) {
        return motionProperties.getPointVelocityCOM(out, body.motionProperties, pointRelativeToCOM);
    }
    return vec3.zero(out);
}

/**
 * Get velocity of a point on the body (point in world space).
 * Returns zero for static bodies.
 *
 * @param out output vector for velocity
 * @param body body to get velocity from
 * @param worldPoint point position in world space
 * @returns out parameter
 */
export function getVelocityAtPoint(out: Vec3, body: RigidBody, worldPoint: Vec3): Vec3 {
    vec3.subtract(_getPointVelocity_pointRelativeToCOM, worldPoint, body.centerOfMassPosition);
    return getVelocityAtPointCOM(out, body, _getPointVelocity_pointRelativeToCOM);
}

/** Returns true if two bodies share a constraint (are connected) */
export function bodiesShareConstraint(bodyA: RigidBody, bodyB: RigidBody): boolean {
    // fast path: if either has no constraints, they can't be connected
    if (bodyA.constraintIds.length === 0 || bodyB.constraintIds.length === 0) {
        return false;
    }

    // iterate the shorter list and check if any id exists in the longer list
    const shorter = bodyA.constraintIds.length <= bodyB.constraintIds.length ? bodyA.constraintIds : bodyB.constraintIds;
    const longer = bodyA.constraintIds.length <= bodyB.constraintIds.length ? bodyB.constraintIds : bodyA.constraintIds;

    for (const id of shorter) {
        if (longer.includes(id)) {
            return true;
        }
    }
    return false;
}

const WakeInAABBVisitor = {
    shouldExit: false,
    world: null! as World,
    bodiesToWake: [] as RigidBody[],
    visit(body: RigidBody) {
        // only wake sleeping dynamic/kinematic bodies
        if (body.sleeping && body.motionType !== MotionType.STATIC) {
            this.bodiesToWake.push(body);
        }
    },
    setup(world: World) {
        this.world = world;
        this.bodiesToWake.length = 0;
        this.shouldExit = false;
    },
    reset() {
        this.world = null!;
        this.bodiesToWake.length = 0;
    },
};

const _wakeInAABB_filter = filter.createEmpty();

/**
 * Wake all sleeping bodies within an AABB.
 * This is useful for performance optimization when you know a specific region of the world
 * needs to become active (e.g., after a large explosion or when loading a new section of a level).
 * @param world the physics world
 * @param aabb the axis-aligned bounding box to query
 */
export function wakeInAABB(world: World, aabb: Box3): void {
    // setup filter with all layers enabled
    filter.enableAllLayers(_wakeInAABB_filter, world.settings.layers);
    _wakeInAABB_filter.collisionMask = ~0;
    _wakeInAABB_filter.collisionGroup = ~0;

    // setup visitor to collect sleeping bodies
    WakeInAABBVisitor.setup(world);

    // query broadphase for all bodies in AABB
    broadphase.intersectAABB(world, aabb, _wakeInAABB_filter, WakeInAABBVisitor);

    // wake all collected bodies
    for (const body of WakeInAABBVisitor.bodiesToWake) {
        sleepModule.wake(world, body);
    }

    // cleanup
    WakeInAABBVisitor.reset();
}

export { sleep, wake } from './sleep';
