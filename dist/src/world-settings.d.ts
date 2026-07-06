import type { Vec3 } from 'mathcat';
import * as layers from './layers.js';
export type WorldSettings = {
    /** Collision layers */
    layers: layers.Layers;
    /**
     * Gravity vector.
     * Unit: m/s²
     * @default [0,-9.81,0]
     */
    gravity: Vec3;
    /**
     * Enable/disable gravity globally.
     * @default true
     */
    gravityEnabled: boolean;
    /** Contact cache settings */
    contacts: {
        /**
         * Maximum allowed distance between old and new contact point to preserve contact
         * forces for warm start.
         * Unit: meters²
         * @default 0.0001 (0.01² or 1cm²)
         */
        contactPointPreserveLambdaMaxDistSq: number;
    };
    /** Narrowphase collision detection settings */
    narrowphase: {
        /**
         * If objects are closer than this distance, they are considered to be colliding (used for GJK).
         * Unit: meters
         * @default 1e-4
         */
        collisionTolerance: number;
        /**
         * A factor that determines the accuracy of the penetration depth calculation.
         * If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate.
         * Unit: dimensionless
         * @default 1e-4
         */
        penetrationTolerance: number;
        /**
         * Whether to return the deepest point during collision detection.
         * @default false
         */
        returnDeepestPoint: boolean;
        /**
         * When false, we prevent collision against non-active (shared) edges.
         * Mainly useful for triangle meshes to avoid ghost collisions at shared edges.
         * @default true
         */
        collideOnlyWithActiveEdges: boolean;
        /**
         * Radius around objects inside which speculative contact points will be detected.
         * Note that if this is too big you will get ghost collisions as speculative contacts
         * are based on the closest points during the collision detection step which may not
         * be the actual closest points by the time the two objects hit.
         * Unit: meters
         * @default 0.02 (2cm)
         */
        speculativeContactDistance: number;
        /**
         * Max distance to use to determine if two points are on the same plane for
         * determining the contact manifold between two shape faces.
         * Unit: meters
         * @default 1e-3 (1mm)
         */
        manifoldTolerance: number;
        /**
         * Collide with backfaces during raycasts/shapecasts.
         * @default false
         */
        collideWithBackfaces: boolean;
        /**
         * Whether or not to reduce manifolds with similar contact normals into one contact manifold.
         * This groups contacts with similar normals that come from different SubShapeIDs
         * (e.g. different triangles in a mesh shape or different compound shapes).
         * If you need to track exactly which SubShapeIDs are in contact, turn this off per-body
         * using body.useManifoldReduction = false.
         * @default true
         */
        useManifoldReduction: boolean;
        /**
         * Maximum angle between normals that allows manifolds between different sub shapes
         * of the same body pair to be combined.
         * Stored as cos(max angle).
         * @default cos(5°) ≈ 0.996
         */
        normalCosMaxDeltaRotation: number;
        /**
         * If true, the narrowphase can be skipped entirely for a body pair when
         * neither body has moved enough relative to the other since last frame.
         * Reuses last frame's contact manifolds verbatim — eliminates GJK/EPA
         * jitter on the penetration axis, which significantly improves stacking
         * stability. Mirrors Jolt's PhysicsSettings::mUseBodyPairContactCache.
         * @default true
         */
        useBodyPairContactCache: boolean;
        /**
         * Max change in the relative translation between two bodies (body B's
         * COM in body A's local frame) before the cached manifold is considered
         * stale and narrowphase runs again. Stored as the squared distance.
         * Unit: meters²
         * @default 1e-6 (1mm)
         */
        bodyPairCacheMaxDeltaPositionSq: number;
        /**
         * Max change in the relative orientation between two bodies before the
         * cached manifold is considered stale. Stored as cos(maxAngle/2) so it
         * compares directly against the absolute quaternion dot of last frame's
         * and this frame's relative rotations.
         * @default cos(1°) ≈ 0.99985
         */
        bodyPairCacheCosMaxDeltaRotationDiv2: number;
    };
    /** Solver options */
    solver: {
        /**
         * Minimal velocity needed before a collision can be elastic.
         * If the relative velocity between colliding objects in the direction of the contact
         * normal is lower than this, the restitution will be zero regardless of the configured
         * value. This lets an object settle sooner.
         * Unit: m/s
         * @default 1.0
         */
        minVelocityForRestitution: number;
        /**
         * Number of solver velocity iterations to run.
         * Note that this needs to be >= 2 in order for friction to work (friction is applied
         * using the non-penetration impulse from the previous iteration).
         * @default 10
         */
        velocityIterations: number;
        /**
         * Number of solver position iterations to run.
         * @default 2
         */
        positionIterations: number;
        /**
         * Whether or not to use warm starting for constraints (initially applying previous frames impulses).
         * This helps the solver converge faster.
         * @default true
         */
        warmStarting: boolean;
        /**
         * How much bodies are allowed to sink into each other.
         * Used to avoid jitter when objects are stacked.
         * Unit: meters
         * @default 0.02 (2cm)
         */
        penetrationSlop: number;
        /**
         * Baumgarte stabilization factor (how much of the position error to 'fix' in 1 update).
         * Unit: dimensionless
         * Range: 0 (nothing) to 1 (100%)
         * @default 0.2 (20%)
         */
        baumgarteFactor: number;
        /**
         * Scale factor for warm start impulses.
         * @default 1.0
         */
        warmStartImpulseRatio: number;
        /**
         * Maximum distance to correct in a single iteration when solving position constraints.
         * Unit: meters
         * @default 0.2 (20cm)
         */
        maxPenetrationDistance: number;
    };
    /** Sleeping options */
    sleeping: {
        /**
         * If objects can go to sleep or not.
         * Sleeping bodies are excluded from physics simulation until woken.
         * @default true
         */
        allowSleeping: boolean;
        /**
         * Time before object is allowed to go to sleep.
         * Unit: seconds
         * @default 0.5
         */
        timeBeforeSleep: number;
        /**
         * To detect if an object is sleeping, we use 3 points:
         * - The center of mass.
         * - The centers of the faces of the bounding box that are furthest away from the center.
         *
         * The movement of these points is tracked and if the velocity of all 3 points is lower
         * than this value, the object is allowed to go to sleep.
         * Unit: m/s
         * @default 0.03 (3cm/s)
         */
        pointVelocitySleepThreshold: number;
    };
    /** Continuous Collision Detection (CCD) options */
    ccd: {
        /**
         * Fraction of inner radius to trigger CCD.
         * If movement < (threshold * innerRadius), use discrete update.
         * Lower = more CCD (expensive), Higher = more tunneling risk.
         * @default 0.05 (5%)
         */
        linearCastThreshold: number;
        /**
         * Fraction of inner radius allowed to penetrate during CCD.
         * Provides penetration slop for stability.
         * @default 0.25 (25%)
         */
        linearCastMaxPenetration: number;
        /**
         * Minimum relative velocity for restitution (bounce) in CCD.
         * Below this, collisions are perfectly inelastic.
         * Unit: m/s
         * @default 1.0
         */
        minVelocityForRestitution: number;
    };
};
export declare const createWorldSettings: () => WorldSettings;
/** add a new broadphase layer, returns the index of the new layer */
export declare function addBroadphaseLayer(settings: WorldSettings): number;
/** add a new object layer mapped to the given broadphase layer, returns the index of the new object layer */
export declare function addObjectLayer(settings: WorldSettings, broadphaseLayer: number): number;
/** enable collision between two object layers */
export declare function enableCollision(settings: WorldSettings, objectLayerA: number, objectLayerB: number): void;
/** disable collision between two object layers */
export declare function disableCollision(settings: WorldSettings, objectLayerA: number, objectLayerB: number): void;
