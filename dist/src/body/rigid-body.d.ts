import { type Box3, type Mat4, type Quat, type Vec3 } from 'mathcat';
import { MaterialCombineMode } from '../constraints/combine-material.js';
import type { ConstraintId } from '../constraints/constraint-id.js';
import { type GetLeafShapeResult, type GetSubShapeTransformedShapeResult, type Shape } from '../shapes/shapes.js';
import type { World } from '../world.js';
import { type BodyId } from './body-id.js';
import * as massProperties from './mass-properties.js';
import * as motionProperties from './motion-properties.js';
import { MotionType } from './motion-type.js';
import * as subShape from './sub-shape.js';
/** describes how mass properties are calculated for a rigid body */
export declare enum MassPropertiesOverride {
    /** calculate both mass and inertia from shape density */
    CALCULATE_MASS_AND_INERTIA = 0,
    /** override mass, calculate and scale inertia from shape */
    CALCULATE_INERTIA = 1,
    /** use provided mass and inertia as-is */
    MASS_AND_INERTIA_PROVIDED = 2
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
    collisionGroups?: number;
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
export declare const DEFAULT_RIGID_BODY_SETTINGS: {
    motionQuality: motionProperties.MotionQuality;
    gravityFactor: number;
    linearDamping: number;
    angularDamping: number;
    maxLinearVelocity: number;
    maxAngularVelocity: number;
    allowSleeping: boolean;
    friction: number;
    restitution: number;
    frictionCombineMode: MaterialCombineMode;
    restitutionCombineMode: MaterialCombineMode;
    allowedDegreesOfFreedom: number;
    useManifoldReduction: boolean;
    collisionGroups: number;
    collisionMask: number;
    sensor: boolean;
    enhancedInternalEdgeRemoval: boolean;
    collideKinematicVsNonDynamic: boolean;
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
    collisionGroups: number;
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
/**
 * Creates a new body in the physics world
 * @param world the physics world
 * @param settings settings for the new body
 * @returns the newly created rigid body
 */
export declare function create(world: World, settings: RigidBodySettings): RigidBody;
/**
 * Removes a body from the world.
 *
 * onContactRemoved events for the body's live contacts are deferred and fired during the
 * next updateWorld (matching jolt semantics — the callback carries body IDs, so it is safe
 * even though the body is destroyed by then).
 * @returns true if the body was successfully removed, false if the body was already pooled (invalid)
 */
export declare function remove(world: World, body: RigidBody): boolean;
/**
 * Gets a body by ID with validation.
 * Returns undefined if the body doesn't exist, is pooled, or has a mismatched sequence number (stale reference).
 */
export declare function get(world: World, bodyId: BodyId): RigidBody | undefined;
/**
 * Generator that yields all physics bodies in the world.
 * Skips inactive body objects in the world state that are pooled.
 */
export declare function iterate(world: World): Generator<RigidBody>;
/**
 * get the world-space inverse inertia matrix for a body.
 * for non-dynamic bodies, returns zero matrix.
 * @param out output Mat4 to store the result
 * @param body the rigid body
 * @returns out parameter
 */
export declare function getInverseInertia(out: Mat4, body: RigidBody): Mat4;
/**
 * Updates the world-space center of mass position based on the body's transform and shape.
 * Must be called whenever position, quaternion, or shape changes.
 */
export declare function updateCenterOfMassPosition(body: RigidBody): void;
/**
 * Updates the body's position (shape origin) based on centerOfMassPosition.
 * This derives position from centerOfMassPosition, which is the primary property modified by physics.
 * Formula: position = centerOfMassPosition - rotation × shape.centerOfMass
 */
export declare function updatePositionFromCenterOfMass(world: World, body: RigidBody): void;
/**
 * Updates the world-space AABB based on the body's transform and shape AABB.
 * Must be called whenever position, quaternion, or shape changes.
 *
 * @optimize
 */
export declare function updateAABB(body: RigidBody): void;
/** updates body properties related to its shape, call this whenever the body's shape changes */
export declare function updateShape(world: World, body: RigidBody): void;
/**
 * Sets the body's position and recomputes the world-space center of mass and AABB.
 */
export declare function setPosition(world: World, body: RigidBody, position: Vec3, wake: boolean): void;
/**
 * Sets the body's orientation and recomputes the world-space center of mass and AABB.
 */
export declare function setQuaternion(world: World, body: RigidBody, quaternion: Quat, wake: boolean): void;
/**
 * Sets both the body's position and orientation, then recomputes the world-space center of mass and AABB.
 * This is the most efficient way to update both transform components in one operation.
 */
export declare function setTransform(world: World, body: RigidBody, position: Vec3, quaternion: Quat, wake: boolean): void;
/**
 * Sets the body's object layer and updates broadphase accordingly.
 */
export declare function setObjectLayer(world: World, body: RigidBody, layer: number): void;
/**
 * Sets the body's motion type and updates active body tracking.
 * @param world The physics world
 * @param body The body to modify
 * @param motionType the new motion type
 * @param wake if true and changing to dynamic/kinematic, wakes the body
 */
export declare function setMotionType(world: World, body: RigidBody, motionType: MotionType, wake: boolean): void;
/**
 * Adds a force at the body's center of mass.
 * Force is accumulated and integrated over time during physics step.
 * Only affects dynamic bodies.
 * @param world The physics world
 * @param body The body to apply force to
 * @param force Force vector in world space
 * @param wake If true, wakes the body if sleeping
 */
export declare function addForce(world: World, body: RigidBody, force: Vec3, wake: boolean): void;
/**
 * Adds a torque (angular force) directly.
 * Torque is accumulated and integrated over time during physics step.
 * Only affects dynamic bodies.
 * @param world the physics world
 * @param body the body to apply torque to
 * @param torque torque vector in world space
 * @param wake if true, wakes the body if sleeping
 */
export declare function addTorque(world: World, body: RigidBody, torque: Vec3, wake: boolean): void;
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
export declare function addForceAtPosition(world: World, body: RigidBody, force: Vec3, worldPosition: Vec3, wake: boolean): void;
/**
 * Applies an impulse at the body's center of mass.
 * Impulse instantly changes velocity: Δv = impulse / mass
 * Only affects dynamic bodies.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to apply impulse to
 * @param impulse impulse vector in world space (kg·m/s)
 */
export declare function addImpulse(world: World, body: RigidBody, impulse: Vec3): void;
/**
 * Applies an impulse at a specific world-space position.
 * Changes both linear and angular velocity.
 * Only affects dynamic bodies. Always wakes the body.
 * @param world the physics world
 * @param body the body to apply impulse to
 * @param impulse impulse vector in world space (kg·m/s)
 * @param worldPosition position in world space where impulse is applied
 */
export declare function addImpulseAtPosition(world: World, body: RigidBody, impulse: Vec3, worldPosition: Vec3): void;
/**
 * Applies an angular impulse (instant change to angular velocity).
 * Δω = I⁻¹ × impulse
 * Only affects dynamic bodies.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to apply angular impulse to
 * @param impulse angular impulse vector in world space (kg·m²/s)
 */
export declare function addAngularImpulse(world: World, body: RigidBody, impulse: Vec3): void;
/**
 * Sets the linear velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to set velocity on
 * @param velocity new linear velocity vector in world space (m/s)
 */
export declare function setLinearVelocity(world: World, body: RigidBody, velocity: Vec3): void;
/**
 * Adds to the linear velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to modify
 * @param velocityDelta velocity change to add in world space (m/s)
 */
export declare function addLinearVelocity(world: World, body: RigidBody, velocityDelta: Vec3): void;
/**
 * Sets the angular velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to set angular velocity on
 * @param velocity new angular velocity vector in world space (rad/s)
 */
export declare function setAngularVelocity(world: World, body: RigidBody, velocity: Vec3): void;
/**
 * Adds to the angular velocity of the body.
 * Always wakes the body.
 * @param world the physics world
 * @param body the body to modify
 * @param velocityDelta angular velocity change to add in world space (rad/s)
 */
export declare function addAngularVelocity(world: World, body: RigidBody, velocityDelta: Vec3): void;
/**
 * Clears all accumulated forces and torques.
 * Should be called after each physics step.
 */
export declare function clearForces(body: RigidBody): void;
/**
 * Moves a kinematic body towards a target position and rotation over the given delta time.
 * @param body the kinematic body to move
 * @param targetPosition target world-space position
 * @param targetQuaternion target world-space quaternion
 * @param deltaTime time step over which to move the body
 */
export declare function moveKinematic(body: RigidBody, targetPosition: Vec3, targetQuaternion: Quat, deltaTime: number): void;
/**
 * Gets the surface normal at a world-space position on the body's shape.
 * The query is performed relative to the shape's local coordinate system (shape origin, not center of mass).
 */
export declare function getSurfaceNormal(out: Vec3, body: RigidBody, worldPosition: Vec3, subShapeId: number): Vec3;
/** create a GetLeafShapeResult object */
export declare function createGetLeafShapeResult(): GetLeafShapeResult;
/**
 * Navigate to a leaf shape following SubShapeID hierarchy.
 * @param out result object to write to @see createGetLeafShapeResult
 * @param rootShape Initial shape to start navigation from
 * @param subShapeId sub shape id path
 */
export declare function getLeafShape(out: GetLeafShapeResult, rootShape: Shape, subShapeId: subShape.SubShapeId): void;
/** create a GetSubShapeTransformedShapeResult object */
export declare function createGetSubShapeTransformedShapeResult(): GetSubShapeTransformedShapeResult;
/**
 * Get transformed shape for a specific SubShapeID path.
 * @param out Single output parameter: { shape, position, rotation, scale, remainder }
 * @param rootShape Initial shape to start navigation from
 * @param subShapeId SubShapeID path
 * @param position Initial position
 * @param rotation Initial rotation
 * @param scale Initial scale
 */
export declare function getSubShapeTransformedShape(out: GetSubShapeTransformedShapeResult, rootShape: Shape, subShapeId: subShape.SubShapeId, position: Vec3, rotation: Quat, scale: Vec3): void;
/**
 * Get velocity of a point on the body (point relative to center of mass).
 * Returns zero for static bodies.
 *
 * @param out output vector for velocity
 * @param body body to get velocity from
 * @param pointRelativeToCOM point position relative to center of mass
 * @returns out parameter
 */
export declare function getVelocityAtPointCOM(out: Vec3, body: RigidBody, pointRelativeToCOM: Vec3): Vec3;
/**
 * Get velocity of a point on the body (point in world space).
 * Returns zero for static bodies.
 *
 * @param out output vector for velocity
 * @param body body to get velocity from
 * @param worldPoint point position in world space
 * @returns out parameter
 */
export declare function getVelocityAtPoint(out: Vec3, body: RigidBody, worldPoint: Vec3): Vec3;
/** Returns true if two bodies share a constraint (are connected) */
export declare function bodiesShareConstraint(bodyA: RigidBody, bodyB: RigidBody): boolean;
/**
 * Wake all sleeping bodies within an AABB.
 * This is useful for performance optimization when you know a specific region of the world
 * needs to become active (e.g., after a large explosion or when loading a new section of a level).
 * @param world the physics world
 * @param aabb the axis-aligned bounding box to query
 */
export declare function wakeInAABB(world: World, aabb: Box3): void;
export { sleep, wake } from './sleep.js';
