import { box3, mat4, quat, raycast3, type Vec3, vec3 } from 'mathcat';
import * as motionProperties from './body/motion-properties';
import { MotionQuality } from './body/motion-properties';
import { MotionType } from './body/motion-type';
import type { RigidBody } from './body/rigid-body';
import * as rigidBody from './body/rigid-body';
import { EMPTY_SUB_SHAPE_ID } from './body/sub-shape';
import type { BodyVisitor } from './broadphase/body-visitor';
import * as broadphase from './broadphase/broadphase';
import * as ccd from './ccd';
import {
    type CastShapeCollector,
    type CastShapeHit,
    type CollideShapeCollector,
    type CollideShapeHit,
    castShapeVsShape,
    collideShapeVsShape,
    collideShapeVsShapeWithInternalEdgeRemoval,
    copyCastShapeHit,
    createCastShapeHit,
    createDefaultCastShapeSettings,
    createDefaultCollideShapeSettings,
} from './collision/narrowphase';
import { combineMaterial } from './constraints/combine-material';
import * as axisConstraintPart from './constraints/constraint-part/axis-constraint-part';
import * as constraints from './constraints/constraints';
import * as contactConstraints from './constraints/contact-constraints';
import * as contacts from './contacts';
import * as filter from './filter';
import * as islands from './islands';
import { ContactValidateResult, type Listener } from './listener';
import * as manifold from './manifold/manifold';
import { MAX_CONTACT_POINTS } from './manifold/manifold';
import { getShapeInnerRadius } from './shapes/shapes';
import type { World } from './world';

/**
 * Updates the physics world with a given time step
 * @param world the physics world to update
 * @param listener optional contact listener for collision events
 * @param timeStep the time step to advance the world by, in seconds
 */
export function updateWorld(world: World, listener: Listener | undefined, timeStep: number): void {
    /* reset CCD state for this frame */
    ccd.clear(world.ccd, world.bodies);

    /* clear previous frame's contact constraints */
    contactConstraints.clear(world.contactConstraints);

    /* mark all body pairs and contacts as unprocessed (we will check 'processed' later for a "contact removed" condition) */
    contacts.markAllUnprocessed(world.contacts);

    /* integrate forces into velocities */
    accelerationIntegrationUpdate(world, timeStep);

    /* broadphase: find potentially colliding body pairs */
    broadphase.findCollidingPairs(world, world.settings.narrowphase.speculativeContactDistance, listener);

    /* narrowphase: check collision for each potentially colliding pair */
    const pairs = world.broadphase.pairs;

    for (let i = 0; i < pairs.n; i++) {
        const bodyIndexA = pairs.pool[i * 2];
        const bodyIndexB = pairs.pool[i * 2 + 1];

        let bodyA = world.bodies.pool[bodyIndexA];
        let bodyB = world.bodies.pool[bodyIndexB];

        // ensure that bodyA has the higher motion type (i.e. dynamic trumps kinematic), this ensures that we do the collision detection in the space of a moving body,
        // which avoids accuracy problems when testing a very large static object against a small dynamic object
        // ensure that bodyA id < bodyB id when motion types are the same.
        if (bodyA.motionType > bodyB.motionType || (bodyA.motionType === bodyB.motionType && bodyB.id < bodyA.id)) {
            const temp = bodyA;
            bodyA = bodyB;
            bodyB = temp;
        }

        // perform narrowphase collision detection, creates contact constraints
        const anyConstraintsCreated = narrowphase(world, bodyA, bodyB, listener, timeStep);

        // if a contact constraint was created, wake up sleeping dynamic bodies
        if (anyConstraintsCreated) {
            if (bodyA.motionType === MotionType.DYNAMIC && bodyA.sleeping) {
                rigidBody.wake(world, bodyA);
            }
            if (bodyB.motionType === MotionType.DYNAMIC && bodyB.sleeping) {
                rigidBody.wake(world, bodyB);
            }
        }

        // destroy stale contacts (unprocessed = sub-shapes no longer colliding)
        contacts.destroyStaleContactsBetweenBodies(world.contacts, bodyA, bodyB, listener);
    }

    /* clean up stale contacts */
    contacts.destroyUnprocessedContacts(world.contacts, world.bodies, listener);

    /* only solve if time step is positive */
    if (timeStep > 0) {
        /* wake sleeping bodies connected to active user constraints */
        wakeBodiesInUserConstraints(world);

        /* build islands */
        islands.prepare(world.islands, world.bodies, world.contacts.contacts.length);
        islands.linkContactConstraints(world.islands, world.contactConstraints, world.contacts, world.bodies);
        islands.linkUserConstraints(world.islands, world.constraints, world.bodies);
        islands.finalize(world.islands, world.bodies, world.constraints, world.settings);

        /* mark constraints as sleeping if all bodies are sleeping */
        constraints.updateSleeping(world.constraints, world.bodies);

        /* setup velocity constraints for user constraints */
        constraints.setupVelocityConstraints(world.constraints, world.bodies, timeStep);

        /* warm start velocity constraints with dynamic ratio */
        if (world.settings.solver.warmStarting) {
            /* calculate warm start impulse ratio to account for variable frame rates */
            // scale cached impulses based on the ratio of current to previous delta time.
            // this prevents impulse overshoot/undershoot when frame rate varies.
            // if previous delta time is 0 (first frame), ratio is 0 (no warm start).
            const warmStartImpulseRatio =
                world.settings.solver.warmStarting && world.previousTimeStep > 0 ? timeStep / world.previousTimeStep : 0.0;

            // warm start contact constraints
            contactConstraints.warmStartVelocityConstraints(world.contactConstraints, world.bodies, warmStartImpulseRatio);

            // warm start user constraints
            constraints.warmStartVelocityConstraints(world.constraints, world.bodies, warmStartImpulseRatio);
        }

        /* store current delta time for next frame's warm start calculation */
        world.previousTimeStep = timeStep;

        /* solve velocity constraints, per island */
        for (const island of world.islands.islands) {
            // skip empty islands (no contacts AND no constraints)
            if (island.contactIndices.length === 0 && island.constraintIds.length === 0) {
                continue;
            }

            /* sort for deterministic simulation, order will remain stable for position solving later */
            constraints.sortConstraintIds(world.constraints, island.constraintIds);
            contactConstraints.sortContactIndices(world.contactConstraints, world.bodies, island.contactIndices);

            /* solve velocity constraints for this island */
            for (let i = 0; i < island.numVelocitySteps; i++) {
                contactConstraints.solveVelocityConstraintsForIsland(
                    world.contactConstraints,
                    world.bodies,
                    island.contactIndices,
                );

                constraints.solveVelocityConstraintsForIsland(world.constraints, world.bodies, island.constraintIds, timeStep);
            }
        }

        /* store applied impulses for warm starting next frame */
        contactConstraints.storeAppliedImpulses(world.contactConstraints, world.contacts);

        /* integrate velocities into positions */
        // for MotionQuality.DISCRETE, we integrate angular velocity and linear velocity
        // for MotionQuality.LINEAR_CAST meeting conditions for CCD, we only integrate angular velocity into the quaternion,
        // and the CCD pipeline handles position integration.
        velocityIntegrationUpdate(world, timeStep);

        /* find CCD contacts for bodies with LINEAR_CAST motion quality */
        findCCDContacts(world, timeStep, listener);

        /* sort CCD bodies by time of impact */
        sortCCDBodies(world.ccd);

        /* resolve CCD contacts (move bodies to collision points) */
        resolveCCDContacts(world);

        /* solve position constraints, per island */
        for (const island of world.islands.islands) {
            // skip empty islands (no contacts AND no constraints)
            if (island.contactIndices.length === 0 && island.constraintIds.length === 0) {
                continue;
            }

            for (let i = 0; i < island.numPositionSteps; i++) {
                let appliedImpulse = contactConstraints.solvePositionConstraintsForIsland(
                    world.contactConstraints,
                    world.bodies,
                    island.contactIndices,
                    world.settings.solver.penetrationSlop,
                    world.settings.solver.baumgarteFactor,
                    world.settings.solver.maxPenetrationDistance,
                );

                appliedImpulse =
                    constraints.solvePositionConstraintsForIsland(
                        world.constraints,
                        world.bodies,
                        island.constraintIds,
                        world.settings.solver.baumgarteFactor,
                        timeStep,
                    ) || appliedImpulse;

                // early termination: island converged if no impulses applied
                if (!appliedImpulse) {
                    break;
                }
            }
        }

        /* update body positions after position solver (derive position from centerOfMassPosition) */
        updateBodyPositions(world);

        /* update body sleeping for each island */
        for (const island of world.islands.islands) {
            islands.checkIslandSleep(island, world, timeStep);
        }
    }

    /* clear all forces */
    for (const body of world.bodies.pool) {
        if (body._pooled || body.motionType === MotionType.STATIC || body.sleeping) continue;
        rigidBody.clearForces(body);
    }
}

const _acceleration_forcesAccel = /* @__PURE__ */ vec3.create();
const _acceleration_gravityAccel = /* @__PURE__ */ vec3.create();
const _acceleration_linearAccel = /* @__PURE__ */ vec3.create();
const _acceleration_angularAccel = /* @__PURE__ */ vec3.create();
const _acceleration_rotation = /* @__PURE__ */ mat4.create();
const _acceleration_worldInverseInertia = /* @__PURE__ */ mat4.create();

/** integrates forces into velocities (F = ma -> a = F/m -> v += a*dt), applies gravity, damping, and velocity clamping */
function accelerationIntegrationUpdate(world: World, timeStep: number): void {
    for (const body of world.bodies.pool) {
        if (body._pooled) continue;
        if (body.motionType !== MotionType.DYNAMIC) continue;
        if (body.sleeping) continue;

        const mp = body.motionProperties;

        // linear acceleration: a = F/m + g
        vec3.scale(_acceleration_forcesAccel, mp.force, mp.invMass);
        vec3.scale(_acceleration_gravityAccel, world.settings.gravity, world.settings.gravityEnabled ? mp.gravityFactor : 0);
        vec3.add(_acceleration_linearAccel, _acceleration_forcesAccel, _acceleration_gravityAccel);

        // integrate linear velocity: v += a * dt
        vec3.scale(_acceleration_linearAccel, _acceleration_linearAccel, timeStep);
        vec3.add(mp.linearVelocity, mp.linearVelocity, _acceleration_linearAccel);

        // apply linear damping: v *= max(0, 1 - damping * dt)
        const linearDampingFactor = Math.max(0, 1 - mp.linearDamping * timeStep);
        vec3.scale(mp.linearVelocity, mp.linearVelocity, linearDampingFactor);

        // clamp linear velocity to max
        const linearSpeedSq = vec3.squaredLength(mp.linearVelocity);
        const maxLinearSq = mp.maxLinearVelocity * mp.maxLinearVelocity;
        if (linearSpeedSq > maxLinearSq) {
            const scale = mp.maxLinearVelocity / Math.sqrt(linearSpeedSq);
            vec3.scale(mp.linearVelocity, mp.linearVelocity, scale);
        }

        // enforce translation DOF constraints (zero out locked axes)
        const allowedTranslation = mp.allowedDegreesOfFreedom & 0b111;
        if (!(allowedTranslation & 0b001)) mp.linearVelocity[0] = 0; // X locked
        if (!(allowedTranslation & 0b010)) mp.linearVelocity[1] = 0; // Y locked
        if (!(allowedTranslation & 0b100)) mp.linearVelocity[2] = 0; // Z locked

        // angular acceleration: α = I^-1 * τ
        mat4.fromQuat(_acceleration_rotation, body.quaternion);
        const worldInverseInertia = _acceleration_worldInverseInertia;
        motionProperties.getInverseInertiaForRotation(worldInverseInertia, mp, _acceleration_rotation);
        mat4.multiply3x3Vec(_acceleration_angularAccel, worldInverseInertia, mp.torque);

        // integrate angular velocity: ω += α * dt
        vec3.scale(_acceleration_angularAccel, _acceleration_angularAccel, timeStep);
        vec3.add(mp.angularVelocity, mp.angularVelocity, _acceleration_angularAccel);

        // apply angular damping: ω *= max(0, 1 - damping * dt)
        const angularDampingFactor = Math.max(0, 1 - mp.angularDamping * timeStep);
        vec3.scale(mp.angularVelocity, mp.angularVelocity, angularDampingFactor);

        // clamp angular velocity to max
        const angularSpeedSq = vec3.squaredLength(mp.angularVelocity);
        const maxAngularSq = mp.maxAngularVelocity * mp.maxAngularVelocity;
        if (angularSpeedSq > maxAngularSq) {
            const scale = mp.maxAngularVelocity / Math.sqrt(angularSpeedSq);
            vec3.scale(mp.angularVelocity, mp.angularVelocity, scale);
        }

        // enforce rotation DOF constraints (zero out locked axes)
        const allowedRotation = (mp.allowedDegreesOfFreedom >> 3) & 0b111;
        if (!(allowedRotation & 0b001)) mp.angularVelocity[0] = 0; // x rotation locked
        if (!(allowedRotation & 0b010)) mp.angularVelocity[1] = 0; // y rotation locked
        if (!(allowedRotation & 0b100)) mp.angularVelocity[2] = 0; // z rotation locked
    }
}

/** updates body positions after physics solvers, derives position (shape origin) from centerOfMassPosition (the primary property modified by physics) */
function updateBodyPositions(world: World): void {
    for (const body of world.bodies.pool) {
        if (body._pooled || body.motionType === MotionType.STATIC || body.sleeping) {
            continue;
        }
        rigidBody.updatePositionFromCenterOfMass(body);
        rigidBody.setTransform(world, body, body.position, body.quaternion, false);
    }
}

/** collide shape collector for narrowphase with manifold reduction */
class NarrowphaseWithReductionCollector implements CollideShapeCollector {
    bodyIdB = -1;

    earlyOutFraction = Number.MAX_VALUE;

    // state configured via setup()
    world: World = null!;
    bodyA: RigidBody = null!;
    bodyB: RigidBody = null!;
    listener: Listener | undefined = undefined;
    deltaTime: number = null!;

    // accumulated manifolds (max 32)
    manifolds: manifold.ContactManifold[] = [];
    maxManifolds = 32;

    // validation state
    validateBodyPair = true;

    reset(): void {
        this.world = null!;
        this.bodyA = null!;
        this.bodyB = null!;
    }

    setup(world: World, bodyA: RigidBody, bodyB: RigidBody, listener: Listener | undefined, deltaTime: number): void {
        // release all accumulated manifolds back to pool
        for (const m of this.manifolds) {
            manifold.contactManifoldPool.release(m);
        }
        this.manifolds.length = 0;
        this.earlyOutFraction = Number.MAX_VALUE;
        this.validateBodyPair = true;

        this.world = world;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.listener = listener;
        this.deltaTime = deltaTime;
    }

    addHit(hit: CollideShapeHit): void {
        // validate contact first (before any processing)
        if (this.validateBodyPair && this.listener?.onContactValidate) {
            const result = this.listener.onContactValidate(this.bodyA!, this.bodyB!, this.bodyA!.position, hit);

            switch (result) {
                case ContactValidateResult.ACCEPT_CONTACT:
                    // continue processing this hit
                    break;
                case ContactValidateResult.ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
                    // accept and stop calling validation for future hits
                    this.validateBodyPair = false;
                    break;
                case ContactValidateResult.REJECT_CONTACT:
                    // skip this hit, continue with next
                    return;
                case ContactValidateResult.REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
                    // early-out from collision detection
                    this.earlyOutFraction = 0;
                    return;
            }
        }

        const baseOffset = this.bodyA!.position;
        const normalThreshold = this.world.settings.narrowphase.normalCosMaxDeltaRotation;
        const maxContactDistance =
            this.world.settings.narrowphase.speculativeContactDistance + this.world.settings.narrowphase.manifoldTolerance;

        // normalize hit normal
        vec3.normalize(_narrowphase_worldSpaceNormal, hit.penetrationAxis);

        // try to find existing manifold with similar normal
        let foundManifold: manifold.ContactManifold | null = null;
        for (const m of this.manifolds) {
            const dot = vec3.dot(_narrowphase_worldSpaceNormal, m.worldSpaceNormal);
            if (dot >= normalThreshold) {
                foundManifold = m;
                break;
            }
        }

        // create new manifold if needed, or accumulate into existing
        if (!foundManifold) {
            if (this.manifolds.length >= this.maxManifolds) {
                // array full - replace shallowest manifold if this hit is deeper
                let shallowest = this.manifolds[0];
                for (let i = 1; i < this.manifolds.length; i++) {
                    if (this.manifolds[i].penetrationDepth < shallowest.penetrationDepth) {
                        shallowest = this.manifolds[i];
                    }
                }

                if (hit.penetration < shallowest.penetrationDepth) {
                    // this hit is shallower than shallowest manifold - skip it
                    return;
                }

                // replace shallowest manifold with this hit
                foundManifold = shallowest;
                manifold.resetContactManifold(foundManifold);
                vec3.copy(foundManifold.baseOffset, baseOffset);
                vec3.copy(foundManifold.worldSpaceNormal, _narrowphase_worldSpaceNormal);
                foundManifold.penetrationDepth = hit.penetration;
                foundManifold.subShapeIdA = hit.subShapeIdA;
                foundManifold.subShapeIdB = hit.subShapeIdB;
                foundManifold.materialIdA = hit.materialIdA;
                foundManifold.materialIdB = hit.materialIdB;
            } else {
                // create new manifold
                foundManifold = manifold.contactManifoldPool.request();
                manifold.resetContactManifold(foundManifold);
                vec3.copy(foundManifold.baseOffset, baseOffset);
                vec3.copy(foundManifold.worldSpaceNormal, _narrowphase_worldSpaceNormal);
                foundManifold.penetrationDepth = hit.penetration;
                foundManifold.subShapeIdA = hit.subShapeIdA;
                foundManifold.subShapeIdB = hit.subShapeIdB;
                foundManifold.materialIdA = hit.materialIdA;
                foundManifold.materialIdB = hit.materialIdB;
                this.manifolds.push(foundManifold);
            }
        } else {
            // accumulate normal (will be normalized later before creating constraints)
            vec3.add(foundManifold.worldSpaceNormal, foundManifold.worldSpaceNormal, _narrowphase_worldSpaceNormal);
        }

        // generate contact points for this hit using temp manifold
        manifold.resetContactManifold(_narrowphase_tempManifold);
        vec3.copy(_narrowphase_tempManifold.baseOffset, baseOffset);

        if (hit.faceA.numVertices >= 2 && hit.faceB.numVertices >= 3) {
            // clip polygons to generate contact region
            manifold.manifoldBetweenTwoFaces(
                _narrowphase_tempManifold,
                hit.pointA,
                hit.pointB,
                hit.penetrationAxis,
                maxContactDistance,
                hit.faceA,
                hit.faceB,
            );
        } else {
            // single contact point
            manifold.setContactPoint(_narrowphase_tempManifold, 0, hit.pointA, hit.pointB);
            _narrowphase_tempManifold.numContactPoints = 1;
        }

        // copy contact points from temp manifold into found manifold
        for (let i = 0; i < _narrowphase_tempManifold.numContactPoints; i++) {
            if (foundManifold.numContactPoints >= 64) break; // safety limit

            const srcIdx = i * 3;
            const dstIdx = foundManifold.numContactPoints * 3;

            foundManifold.relativeContactPointsOnA[dstIdx] = _narrowphase_tempManifold.relativeContactPointsOnA[srcIdx];
            foundManifold.relativeContactPointsOnA[dstIdx + 1] = _narrowphase_tempManifold.relativeContactPointsOnA[srcIdx + 1];
            foundManifold.relativeContactPointsOnA[dstIdx + 2] = _narrowphase_tempManifold.relativeContactPointsOnA[srcIdx + 2];

            foundManifold.relativeContactPointsOnB[dstIdx] = _narrowphase_tempManifold.relativeContactPointsOnB[srcIdx];
            foundManifold.relativeContactPointsOnB[dstIdx + 1] = _narrowphase_tempManifold.relativeContactPointsOnB[srcIdx + 1];
            foundManifold.relativeContactPointsOnB[dstIdx + 2] = _narrowphase_tempManifold.relativeContactPointsOnB[srcIdx + 2];

            foundManifold.numContactPoints++;
        }

        // update penetration depth to deepest
        if (hit.penetration > foundManifold.penetrationDepth) {
            foundManifold.penetrationDepth = hit.penetration;
        }

        // prune if manifold has > 32 points
        if (foundManifold.numContactPoints > 32) {
            manifold.pruneContactPoints(foundManifold, foundManifold.worldSpaceNormal);
        }
    }

    addMiss(): void {
        // no-op
    }

    finalizeAndCreateConstraints(): boolean {
        // finalizes accumulated manifolds and creates contact constraints.
        // called after collision detection completes.
        // returns true if any constraints were created.

        let constraintsCreated = false;

        for (const currentManifold of this.manifolds) {
            // normalize accumulated normal (sum of all merged hit normals)
            vec3.normalize(currentManifold.worldSpaceNormal, currentManifold.worldSpaceNormal);

            // final pruning to 4 points (if we have more than 4)
            if (currentManifold.numContactPoints > MAX_CONTACT_POINTS) {
                manifold.pruneContactPoints(currentManifold, currentManifold.worldSpaceNormal);
            }

            if (currentManifold.numContactPoints > 0) {
                const created = contactConstraints.addContactConstraint(
                    this.world.contactConstraints,
                    this.world.contacts,
                    this.bodyA,
                    this.bodyB,
                    currentManifold,
                    this.world.settings,
                    this.listener,
                    this.deltaTime,
                );
                constraintsCreated = constraintsCreated || created;
            }
        }

        return constraintsCreated;
    }

    shouldEarlyOut(): boolean {
        return this.earlyOutFraction <= 0;
    }
}

/** collide shape collector for narrowphase without manifold reduction */
class NarrowphaseWithoutReductionCollector implements CollideShapeCollector {
    bodyIdB = -1;

    earlyOutFraction = Number.MAX_VALUE;

    // state configured via setup()
    world: World = null!;
    bodyA: RigidBody = null!;
    bodyB: RigidBody = null!;
    listener: Listener | undefined = undefined;
    deltaTime: number = null!;

    // track if any constraints were created
    constraintsCreated = false;

    // validation state
    validateBodyPair = true;

    // scratch objects
    _tempManifold = manifold.createContactManifold();

    reset(): void {
        this.world = null!;
        this.bodyA = null!;
        this.bodyB = null!;
    }

    setup(world: World, bodyA: RigidBody, bodyB: RigidBody, listener: Listener | undefined, deltaTime: number): void {
        this.constraintsCreated = false;
        this.earlyOutFraction = Number.MAX_VALUE;
        this.validateBodyPair = true;

        this.world = world;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.listener = listener;
        this.deltaTime = deltaTime;
    }

    addHit(hit: CollideShapeHit): void {
        // validate contact first (before any processing)
        if (this.validateBodyPair && this.listener?.onContactValidate) {
            const result = this.listener.onContactValidate(this.bodyA!, this.bodyB!, this.bodyA!.position, hit);

            switch (result) {
                case ContactValidateResult.ACCEPT_CONTACT:
                    // continue processing this hit
                    break;
                case ContactValidateResult.ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
                    // accept and stop calling validation for future hits
                    this.validateBodyPair = false;
                    break;
                case ContactValidateResult.REJECT_CONTACT:
                    // skip this hit, continue with next
                    return;
                case ContactValidateResult.REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
                    // early-out from collision detection
                    this.earlyOutFraction = 0;
                    return;
            }
        }

        const maxContactDistance =
            this.world.settings.narrowphase.speculativeContactDistance + this.world.settings.narrowphase.manifoldTolerance;

        // create manifold from this single hit
        const tempManifold = this._tempManifold;
        manifold.resetContactManifold(tempManifold);
        vec3.copy(tempManifold.baseOffset, this.bodyA.position);
        vec3.normalize(tempManifold.worldSpaceNormal, hit.penetrationAxis);
        tempManifold.penetrationDepth = hit.penetration;
        tempManifold.subShapeIdA = hit.subShapeIdA;
        tempManifold.subShapeIdB = hit.subShapeIdB;
        tempManifold.materialIdA = hit.materialIdA;
        tempManifold.materialIdB = hit.materialIdB;

        // generate contact points for this hit
        if (hit.faceA.numVertices >= 2 && hit.faceB.numVertices >= 3) {
            // clip polygons to generate contact region
            manifold.manifoldBetweenTwoFaces(
                tempManifold,
                hit.pointA,
                hit.pointB,
                hit.penetrationAxis,
                maxContactDistance,
                hit.faceA,
                hit.faceB,
            );
        } else {
            // single contact point
            manifold.setContactPoint(tempManifold, 0, hit.pointA, hit.pointB);
            tempManifold.numContactPoints = 1;
        }

        // prune to 4 contact points
        if (tempManifold.numContactPoints > MAX_CONTACT_POINTS) {
            manifold.pruneContactPoints(tempManifold, tempManifold.worldSpaceNormal);
        }

        // immediately create constraint
        if (tempManifold.numContactPoints > 0) {
            const created = contactConstraints.addContactConstraint(
                this.world.contactConstraints,
                this.world.contacts,
                this.bodyA,
                this.bodyB,
                tempManifold,
                this.world.settings,
                this.listener,
                this.deltaTime,
            );
            this.constraintsCreated = this.constraintsCreated || created;
        }
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return this.earlyOutFraction <= 0;
    }
}

const NARROWPHASE_REDUCTION_COLLECTOR = /* @__PURE__ */ new NarrowphaseWithReductionCollector();
const NARROWPHASE_WITHOUT_REDUCTION_COLLECTOR = /* @__PURE__ */ new NarrowphaseWithoutReductionCollector();

const _narrowphase_collideSettings = /* @__PURE__ */ createDefaultCollideShapeSettings();
const _narrowphase_worldSpaceNormal = /* @__PURE__ */ vec3.create();
const _narrowphase_tempManifold = /* @__PURE__ */ manifold.createContactManifold();

/**
 * performs narrowphase collision detection for a body pair, returns whether any constraints were created
 *
 * notes on shape-local space vs center-of-mass-local space:
 * - collide shape vs shape is dispatched in world space, and later transformed into shape origin local space in shape-vs-shape functions.
 * - when we create contact constraints, we use center of mass local positions, which are used for constraint solving.
 * - in future it may make sense to do upfront local space transforms here for numerical stability.
 * - the specific shape vs shape functions do these transforms later though, so algorithms such as gjk/epa will be ok for numerical stability.
 */
function narrowphase(
    world: World,
    bodyA: RigidBody,
    bodyB: RigidBody,
    listener: Listener | undefined,
    deltaTime: number,
): boolean {
    // set collide settings from world options
    const collideSettings = _narrowphase_collideSettings;
    collideSettings.collectFaces = true;
    collideSettings.collideWithBackfaces = world.settings.narrowphase.collideWithBackfaces;
    collideSettings.collisionTolerance = world.settings.narrowphase.collisionTolerance;
    collideSettings.penetrationTolerance = world.settings.narrowphase.penetrationTolerance;
    collideSettings.returnDeepestPoint = world.settings.narrowphase.returnDeepestPoint;
    collideSettings.collideOnlyWithActiveEdges = world.settings.narrowphase.collideOnlyWithActiveEdges;

    // set active edges direction using linear velocity difference
    vec3.sub(
        collideSettings.activeEdgeMovementDirection,
        bodyA.motionProperties.linearVelocity,
        bodyB.motionProperties.linearVelocity,
    );

    // set max separation distance
    collideSettings.maxSeparationDistance =
        bodyA.sensor || bodyB.sensor ? 0 : world.settings.narrowphase.speculativeContactDistance;

    // choose collector based on manifold reduction setting
    const useManifoldReduction =
        world.settings.narrowphase.useManifoldReduction && bodyA.useManifoldReduction && bodyB.useManifoldReduction;

    if (useManifoldReduction) {
        // use reduction collector - accumulates manifolds, creates constraints after
        const collector = NARROWPHASE_REDUCTION_COLLECTOR;

        // setup collector for this body pair
        collector.setup(world, bodyA, bodyB, listener, deltaTime);

        // check if enhanced internal edge removal is enabled for either body
        const useEnhancedEdgeRemoval = bodyA.enhancedInternalEdgeRemoval || bodyB.enhancedInternalEdgeRemoval;

        // perform collision detection
        if (useEnhancedEdgeRemoval) {
            // biome-ignore format: pretty
            collideShapeVsShapeWithInternalEdgeRemoval(
                collector,
                collideSettings,
                // body a
                bodyA.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyA.position[0],   bodyA.position[1],   bodyA.position[2],
                bodyA.quaternion[0], bodyA.quaternion[1], bodyA.quaternion[2], bodyA.quaternion[3],
                1, 1, 1,
                // body b
                bodyB.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyB.position[0],   bodyB.position[1],   bodyB.position[2],
                bodyB.quaternion[0], bodyB.quaternion[1], bodyB.quaternion[2], bodyB.quaternion[3],
                1, 1, 1,
            );
        } else {
            // biome-ignore format: pretty
            collideShapeVsShape(
                collector,
                collideSettings,
                // body a
                bodyA.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyA.position[0],   bodyA.position[1],   bodyA.position[2],
                bodyA.quaternion[0], bodyA.quaternion[1], bodyA.quaternion[2], bodyA.quaternion[3],
                1, 1, 1,
                // body b
                bodyB.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyB.position[0],   bodyB.position[1],   bodyB.position[2],
                bodyB.quaternion[0], bodyB.quaternion[1], bodyB.quaternion[2], bodyB.quaternion[3],
                1, 1, 1,
            );
        }

        // finalize and create constraints
        const constraintsCreated = collector.finalizeAndCreateConstraints();

        collector.reset();

        // return whether any constraints were created
        return constraintsCreated;
    } else {
        // use non-reduction collector - creates constraints immediately in addHit
        const collector = NARROWPHASE_WITHOUT_REDUCTION_COLLECTOR;

        // setup collector for this body pair
        collector.setup(world, bodyA, bodyB, listener, deltaTime);

        // check if enhanced internal edge removal is enabled for either body
        const useEnhancedEdgeRemoval = bodyA.enhancedInternalEdgeRemoval || bodyB.enhancedInternalEdgeRemoval;

        // perform collision detection
        if (useEnhancedEdgeRemoval) {
            // biome-ignore format: pretty
            collideShapeVsShapeWithInternalEdgeRemoval(
                collector,
                collideSettings,
                // body a
                bodyA.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyA.position[0],   bodyA.position[1],   bodyA.position[2],
                bodyA.quaternion[0], bodyA.quaternion[1], bodyA.quaternion[2], bodyA.quaternion[3],
                1, 1, 1,
                // body b
                bodyB.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyB.position[0],   bodyB.position[1],   bodyB.position[2],
                bodyB.quaternion[0], bodyB.quaternion[1], bodyB.quaternion[2], bodyB.quaternion[3],
                1, 1, 1,
            );
        } else {
            // biome-ignore format: pretty
            collideShapeVsShape(
                collector,
                collideSettings,
                // body a
                bodyA.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyA.position[0],   bodyA.position[1],   bodyA.position[2],
                bodyA.quaternion[0], bodyA.quaternion[1], bodyA.quaternion[2], bodyA.quaternion[3],
                1, 1, 1,
                // body b
                bodyB.shape,
                EMPTY_SUB_SHAPE_ID, 0,
                bodyB.position[0],   bodyB.position[1],   bodyB.position[2],
                bodyB.quaternion[0], bodyB.quaternion[1], bodyB.quaternion[2], bodyB.quaternion[3],
                1, 1, 1,
            );
        }

        const constraintsCreated = collector.constraintsCreated;

        collector.reset();

        // return whether any constraints were created
        return constraintsCreated;
    }
}

/**
 * wake sleeping bodies connected to active constraints
 *
 * a constraint is "active" if:
 * - enabled AND
 * - at least one body is awake AND
 * - at least one body is dynamic
 *
 * this allows bodies connected by constraints to sleep together (e.g., ragdolls at rest).
 * when one body wakes, the wake propagates through the constraint chain frame-by-frame.
 */
function wakeBodiesInUserConstraints(world: World): void {
    const { constraints, bodies } = world;

    // note: we repeat ourselves below to avoid polymorphic/megamorphic IC reads.
    // this is a bit annoying but important for performance.

    // wake point constraint bodies
    for (const constraint of constraints.pointConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }

    // wake distance constraint bodies
    for (const constraint of constraints.distanceConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }

    // wake hinge constraint bodies
    for (const constraint of constraints.hingeConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }

    // wake fixed constraint bodies
    for (const constraint of constraints.fixedConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }

    // wake swing-twist constraint bodies
    for (const constraint of constraints.swingTwistConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }

    // wake slider constraint bodies
    for (const constraint of constraints.sliderConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }

    // wake cone constraint bodies
    for (const constraint of constraints.coneConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }

    // wake sixDOF constraint bodies
    for (const constraint of constraints.sixDOFConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];

            const hasActiveBody = !bodyA.sleeping || !bodyB.sleeping;
            const hasDynamicBody = bodyA.motionType === MotionType.DYNAMIC || bodyB.motionType === MotionType.DYNAMIC;

            if (hasActiveBody && hasDynamicBody) {
                if (bodyA.sleeping && bodyA.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyA);
                }
                if (bodyB.sleeping && bodyB.motionType === MotionType.DYNAMIC) {
                    rigidBody.wake(world, bodyB);
                }
            }
        }
    }
}

const _velocity_rotationVector = /* @__PURE__ */ vec3.create();
const _velocity_axis = /* @__PURE__ */ vec3.create();
const _velocity_rotationQuat = /* @__PURE__ */ quat.create();
const _velocity_displacement = /* @__PURE__ */ vec3.create();

/** integrates velocities into positions (p += v*dt) and angular velocities into orientations */
function velocityIntegrationUpdate(world: World, timeStep: number): void {
    for (const body of world.bodies.pool) {
        if (body._pooled || body.motionType === MotionType.STATIC || body.sleeping) continue;

        const mp = body.motionProperties;

        // clamp velocities for dynamic bodies
        if (body.motionType === MotionType.DYNAMIC) {
            motionProperties.clampLinearVelocity(mp);
            motionProperties.clampAngularVelocity(mp);
        }

        // rotate first, helps avoid artifacts with long thin bodies
        // angular integration: orientation = rotation_quat * orientation
        // convert angular velocity to axis-angle rotation
        const rotationVector = vec3.scale(_velocity_rotationVector, mp.angularVelocity, timeStep);
        const angle = vec3.length(rotationVector);

        if (angle > 1e-6) {
            // create rotation quaternion from axis-angle
            const axis = vec3.scale(_velocity_axis, rotationVector, 1 / angle);
            const rotationQuat = quat.setAxisAngle(_velocity_rotationQuat, axis, angle);

            // apply rotation: q' = rotation * q
            quat.multiply(body.quaternion, rotationQuat, body.quaternion);

            // normalize to prevent drift
            quat.normalize(body.quaternion, body.quaternion);
        }

        // calculate linear displacement
        const displacement = vec3.scale(_velocity_displacement, mp.linearVelocity, timeStep);

        // if the position should be updated (or if it is delayed because of ccd)
        let updatePosition = true;

        // ccd only for dynamic bodies with LinearCast motion quality (not sensors)
        // kinematic bodies cannot be stopped by collisions (infinite mass), so ccd is meaningless for them
        const useCCD = mp.motionQuality === MotionQuality.LINEAR_CAST && body.motionType === MotionType.DYNAMIC && !body.sensor;

        if (useCCD) {
            const innerRadius = getShapeInnerRadius(body.shape);
            const threshold = world.settings.ccd.linearCastThreshold * innerRadius;
            const displacementLenSq = vec3.squaredLength(displacement);

            if (displacementLenSq > threshold * threshold) {
                // acquire CCD body from pool
                const ccdBody = ccd.ccdBodyPool.request();
                ccd.resetCCDBody(ccdBody);
                const ccdIndex = world.ccd.ccdBodies.length;
                world.ccd.ccdBodies.push(ccdBody);
                body.ccdBodyIndex = ccdIndex;

                // initialize CCD body
                ccdBody.bodyIndex = body.index;
                ccdBody.hitBodyIndex = -1;
                vec3.copy(ccdBody.deltaPosition, displacement);
                ccdBody.fraction = 1.0;
                ccdBody.fractionPlusSlop = 1.0;
                ccdBody.linearCastThresholdSq = threshold * threshold;
                ccdBody.maxPenetration = Math.min(
                    world.settings.solver.penetrationSlop,
                    world.settings.ccd.linearCastMaxPenetration * innerRadius,
                );

                // update quaternion
                rigidBody.setQuaternion(world, body, body.quaternion, false);

                // delay position update until CCD resolves collisions
                updatePosition = false;
            }
        }

        if (updatePosition) {
            // move the body now (using center of mass)
            vec3.add(body.centerOfMassPosition, body.centerOfMassPosition, displacement);
            rigidBody.updatePositionFromCenterOfMass(body);
            rigidBody.setTransform(world, body, body.position, body.quaternion, false);
        }
    }
}

/**
 * calculate the motion of a body for CCD correction.
 * if the body is using LinearCast (hasn't moved yet), returns velocity * deltaTime.
 * works for both dynamic and kinematic bodies.
 * otherwise returns zero (body already moved in discrete update).
 */
function calculateBodyMotion(out: Vec3, body: RigidBody, deltaTime: number): void {
    const isLinearCast = body.motionProperties?.motionQuality === MotionQuality.LINEAR_CAST;
    const isMoving = body.motionType === MotionType.DYNAMIC || body.motionType === MotionType.KINEMATIC;

    if (isMoving && isLinearCast) {
        vec3.scale(out, body.motionProperties.linearVelocity, deltaTime);
        return;
    }

    vec3.set(out, 0, 0, 0);
}

const _ccd_relativeDisplacement = /* @__PURE__ */ vec3.create();
const _ccd_expandedAABB = /* @__PURE__ */ box3.create();
const _ccd_shapeExtent = /* @__PURE__ */ vec3.create();
const _ccd_normal = /* @__PURE__ */ vec3.create();
const _ccd_ray = /* @__PURE__ */ raycast3.create();
const _ccd_bodyBMotion = /* @__PURE__ */ vec3.create();

/** collector that processes shape cast hits for CCD, updates CCDBody with earliest collision found */
class CCDCastShapeCollector implements CastShapeCollector {
    // setup state
    ccdBody: ccd.CCDBody = null!;
    bodyA: RigidBody = null!;
    bodyB: RigidBody = null!;
    timeStep = 0;

    // result state
    bodyIdB = -1;
    earlyOutFraction = Number.MAX_VALUE;
    hasHit = false;
    hit = createCastShapeHit();

    setup(ccdBody: ccd.CCDBody, bodyA: RigidBody, bodyB: RigidBody, timeStep: number): void {
        this.hasHit = false;
        this.ccdBody = ccdBody;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.bodyIdB = bodyB.id;
        this.timeStep = timeStep;
        this.earlyOutFraction = ccdBody.fractionPlusSlop;
    }

    reset(): void {
        this.hasHit = false;
        this.earlyOutFraction = Number.MAX_VALUE;
        this.ccdBody = null!;
        this.bodyA = null!;
        this.bodyB = null!;
    }

    addHit(hit: CastShapeHit): void {
        const fraction = hit.fraction;

        // normalize hit normal
        vec3.normalize(_ccd_normal, hit.penetrationAxis);

        // calculate penetration slop allowance
        const denominator = vec3.dot(_ccd_normal, this.ccdBody.deltaPosition);

        // avoid division by zero or near-zero denominator
        if (denominator <= this.ccdBody.maxPenetration) return;

        // add penetration allowance to fraction
        const fractionPlusSlop = fraction + this.ccdBody.maxPenetration / denominator;

        // only update if this is earliest hit
        if (fractionPlusSlop >= this.ccdBody.fractionPlusSlop) return;

        // store FULL hit data (need faceA, faceB for manifold creation)
        copyCastShapeHit(this.hit, hit);

        // store earliest collision
        this.ccdBody.hitBodyIndex = this.bodyB.index;
        this.ccdBody.fraction = fraction;
        this.ccdBody.fractionPlusSlop = fractionPlusSlop;
        vec3.copy(this.ccdBody.contactNormal, _ccd_normal);
        this.ccdBody.subShapeId1 = hit.subShapeIdA;
        this.ccdBody.subShapeId2 = hit.subShapeIdB;

        // CRITICAL: Correct contact point for bodyB's movement
        // If bodyB also has CCD, it hasn't moved yet, so contact points need to be
        // adjusted for where bodyB will be at the time of collision
        calculateBodyMotion(_ccd_bodyBMotion, this.bodyB, this.timeStep);
        if (vec3.squaredLength(_ccd_bodyBMotion) > 1e-12) {
            // Scale by fraction (bodyB moves fraction * deltaTime before collision)
            vec3.scale(_ccd_bodyBMotion, _ccd_bodyBMotion, fraction);
            // Offset contact point by bodyB's movement
            vec3.add(this.ccdBody.contactPoint, hit.pointA, _ccd_bodyBMotion);
        } else {
            // BodyB not using CCD or already moved - use contact point as-is
            vec3.copy(this.ccdBody.contactPoint, hit.pointA);
        }

        // populate contact settings
        const settings = this.ccdBody.contactSettings;
        settings.combinedFriction = combineMaterial(
            this.bodyA.friction,
            this.bodyB.friction,
            this.bodyA.frictionCombineMode,
            this.bodyB.frictionCombineMode,
        );
        settings.combinedRestitution = combineMaterial(
            this.bodyA.restitution,
            this.bodyB.restitution,
            this.bodyA.restitutionCombineMode,
            this.bodyB.restitutionCombineMode,
        );
        settings.isSensor = false; // CCD doesn't support sensors
        // mass/inertia scales default to 1.0 (already initialized)

        this.hasHit = true;
        this.earlyOutFraction = fractionPlusSlop;
    }

    addMiss(): void {
        // no-op
    }

    shouldEarlyOut(): boolean {
        return false;
    }
}

/** body visitor that processes potential CCD hits for a given CCDBody */
class CCDBodyVisitor implements BodyVisitor {
    shouldExit = false;

    // state configured via setup()
    world: World = null!;
    ccdBody: ccd.CCDBody = null!;
    bodyA: RigidBody = null!;
    timeStep = 0;
    earlyOutFraction = Number.MAX_VALUE;

    collector = new CCDCastShapeCollector();
    castSettings = createDefaultCastShapeSettings();

    setup(world: World, ccdBody: ccd.CCDBody, bodyA: RigidBody, timeStep: number): void {
        this.shouldExit = false;
        this.world = world;
        this.ccdBody = ccdBody;
        this.bodyA = bodyA;
        this.timeStep = timeStep;
        this.earlyOutFraction = ccdBody.fractionPlusSlop;
    }

    reset(): void {
        this.shouldExit = false;
        this.world = null!;
        this.ccdBody = null!;
        this.bodyA = null!;
        this.earlyOutFraction = Number.MAX_VALUE;
    }

    visit(bodyB: RigidBody): void {
        // self-collision filter
        if (this.bodyA.index === bodyB.index) return;

        // duplicate CCD pair filter
        const ccdBodyBIndex = bodyB.ccdBodyIndex;
        if (ccdBodyBIndex >= 0) {
            // both bodies have CCD - only process once (lower bodyIndex processes pair)
            const ccdBodyB = this.world.ccd.ccdBodies[ccdBodyBIndex];
            if (this.ccdBody.bodyIndex > ccdBodyB.bodyIndex) return;
        }

        // collision group filter
        if (
            !filter.shouldPairCollide(
                this.bodyA.collisionGroup,
                this.bodyA.collisionMask,
                bodyB.collisionGroup,
                bodyB.collisionMask,
            )
        ) {
            return;
        }

        // sensor filter - CCD doesn't support sensors
        if (bodyB.sensor) return;

        // calculate relative displacement
        vec3.copy(_ccd_relativeDisplacement, this.ccdBody.deltaPosition);
        if (ccdBodyBIndex >= 0) {
            const ccdBodyB = this.world.ccd.ccdBodies[ccdBodyBIndex];
            vec3.subtract(_ccd_relativeDisplacement, _ccd_relativeDisplacement, ccdBodyB.deltaPosition);
        }

        // threshold check
        if (vec3.squaredLength(_ccd_relativeDisplacement) < this.ccdBody.linearCastThresholdSq) return;

        // AABB early-out test: cast ray from bodyA's position along relative displacement
        // expand bodyB's AABB by bodyA's extent to account for bodyA's size
        box3.size(_ccd_shapeExtent, this.bodyA.aabb);
        vec3.scale(_ccd_shapeExtent, _ccd_shapeExtent, 0.5);
        box3.copy(_ccd_expandedAABB, bodyB.aabb);
        box3.expandByExtents(_ccd_expandedAABB, _ccd_expandedAABB, _ccd_shapeExtent);

        // ray vs AABB test: early-out if ray doesn't intersect expanded AABB
        const rayLength = vec3.length(_ccd_relativeDisplacement);
        raycast3.set(_ccd_ray, this.bodyA.position, _ccd_relativeDisplacement, rayLength);
        if (!raycast3.intersectsBox3(_ccd_ray, _ccd_expandedAABB)) {
            return;
        }

        // narrowphase: shape cast
        const collector = this.collector;
        collector.setup(this.ccdBody, this.bodyA, bodyB, this.timeStep);

        // reset collector's early-out to visitor's current best (optimization)
        collector.earlyOutFraction = this.earlyOutFraction;

        const settings = this.castSettings;
        settings.returnDeepestPoint = false;
        settings.collideWithBackfaces = this.world.settings.narrowphase.collideWithBackfaces;

        // biome-ignore format: pretty
        castShapeVsShape(
            collector,
            settings,
            this.bodyA.shape,
            EMPTY_SUB_SHAPE_ID, 0,
            this.bodyA.position[0], this.bodyA.position[1], this.bodyA.position[2],
            this.bodyA.quaternion[0], this.bodyA.quaternion[1], this.bodyA.quaternion[2], this.bodyA.quaternion[3],
            1, 1, 1,
            _ccd_relativeDisplacement[0], _ccd_relativeDisplacement[1], _ccd_relativeDisplacement[2],
            bodyB.shape,
            EMPTY_SUB_SHAPE_ID, 0,
            bodyB.position[0], bodyB.position[1], bodyB.position[2],
            bodyB.quaternion[0], bodyB.quaternion[1], bodyB.quaternion[2], bodyB.quaternion[3],
            1, 1, 1,
        );

        // update visitor's early-out based on narrowphase result
        this.earlyOutFraction = Math.min(this.earlyOutFraction, collector.earlyOutFraction);

        collector.reset();
    }
}

const CCD_BODY_VISITOR = /* @__PURE__ */ new CCDBodyVisitor();

const _ccd_filter = /* @__PURE__ */ filter.createEmpty();

/** handle CCD contact added event - calculates material properties and fires contact listener callbacks */
function onCCDContactAdded(
    world: World,
    listener: Listener | undefined,
    bodyA: RigidBody,
    bodyB: RigidBody,
    contactManifold: manifold.ContactManifold,
    contactSettings: contactConstraints.ContactSettings,
): void {
    // calculate combined material properties (matches regular contacts)
    contactSettings.combinedFriction = combineMaterial(
        bodyA.friction,
        bodyB.friction,
        bodyA.frictionCombineMode,
        bodyB.frictionCombineMode,
    );
    contactSettings.combinedRestitution = combineMaterial(
        bodyA.restitution,
        bodyB.restitution,
        bodyA.restitutionCombineMode,
        bodyB.restitutionCombineMode,
    );
    contactSettings.isSensor = bodyA.sensor || bodyB.sensor; // sensor if either is sensor

    // if no listener, we're done (material properties set)
    if (!listener) return;

    // CCD contacts are always added to cache (only dynamic bodies use CCD)
    // swap bodies so orderedBodyA.id < orderedBodyB.id (matches addContactConstraint behavior)
    let orderedBodyA = bodyA;
    let orderedBodyB = bodyB;
    let swappedManifold = contactManifold;

    if (bodyA.id > bodyB.id) {
        orderedBodyA = bodyB;
        orderedBodyB = bodyA;

        // swap manifold sub-shape IDs to match body swap
        swappedManifold = manifold.contactManifoldPool.request();
        manifold.resetContactManifold(swappedManifold);
        swappedManifold.subShapeIdA = contactManifold.subShapeIdB;
        swappedManifold.subShapeIdB = contactManifold.subShapeIdA;
        vec3.copy(swappedManifold.baseOffset, contactManifold.baseOffset);
        vec3.copy(swappedManifold.worldSpaceNormal, contactManifold.worldSpaceNormal);
        swappedManifold.penetrationDepth = contactManifold.penetrationDepth;
        swappedManifold.numContactPoints = contactManifold.numContactPoints;

        // copy contact points
        for (let i = 0; i < contactManifold.numContactPoints * 3; i++) {
            swappedManifold.relativeContactPointsOnA[i] = contactManifold.relativeContactPointsOnB[i];
            swappedManifold.relativeContactPointsOnB[i] = contactManifold.relativeContactPointsOnA[i];
        }
    }

    // find existing contact
    const existingContact = contacts.findContact(
        world.contacts,
        orderedBodyA,
        orderedBodyB,
        swappedManifold.subShapeIdA,
        swappedManifold.subShapeIdB,
    );

    if (existingContact) {
        // contact persisted from previous frame
        // mark it as CCD contact to prevent warm-starting with stale impulses
        existingContact.flags |= contacts.CachedManifoldFlags.CCDContact;
        existingContact.processedThisFrame = true;

        listener.onContactPersisted?.(orderedBodyA, orderedBodyB, swappedManifold, contactSettings);
    } else {
        // new contact - create it with CCD flag
        const contact = contacts.createContact(
            world.contacts,
            orderedBodyA,
            orderedBodyB,
            swappedManifold.subShapeIdA,
            swappedManifold.subShapeIdB,
        );
        contact.flags = contacts.CachedManifoldFlags.CCDContact;
        contact.processedThisFrame = true;

        listener.onContactAdded?.(orderedBodyA, orderedBodyB, swappedManifold, contactSettings);
    }

    // swap mass/inertia scales back if we swapped bodies
    if (orderedBodyA !== bodyA) {
        [contactSettings.invMassScale1, contactSettings.invMassScale2] = [
            contactSettings.invMassScale2,
            contactSettings.invMassScale1,
        ];
        [contactSettings.invInertiaScale1, contactSettings.invInertiaScale2] = [
            contactSettings.invInertiaScale2,
            contactSettings.invInertiaScale1,
        ];
    }

    // release swapped manifold if we created one
    if (swappedManifold !== contactManifold) {
        manifold.contactManifoldPool.release(swappedManifold);
    }
}

const _findCCDContacts_sweptAABB = /* @__PURE__ */ box3.create();
const _findCCDContacts_contactManifold = /* @__PURE__ */ manifold.createContactManifold();

/** finds earliest collision for each CCD body, done after velocity integration */
function findCCDContacts(world: World, timeStep: number, listener: Listener | undefined): void {
    const visitor = CCD_BODY_VISITOR;
    const bodyFilter = _ccd_filter;

    // process each CCD body
    for (let i = 0; i < world.ccd.ccdBodies.length; i++) {
        const ccdBody = world.ccd.ccdBodies[i];
        const bodyA = world.bodies.pool[ccdBody.bodyIndex];

        // compute swept AABB (encompasses start + end positions)
        ccd.computeSweptAABB(_findCCDContacts_sweptAABB, bodyA, ccdBody.deltaPosition);

        // setup visitor for this CCD body
        visitor.setup(world, ccdBody, bodyA, timeStep);

        // setup filter for this body's layers
        filter.setFromBody(bodyFilter, world.settings.layers, bodyA);

        // broadphase: cast swept AABB through tree
        broadphase.castAABB(world, _findCCDContacts_sweptAABB, ccdBody.deltaPosition, bodyFilter, visitor);

        // create contact manifold if there was a hit
        if (ccdBody.hitBodyIndex !== -1 && visitor.collector.hit) {
            const bodyB = world.bodies.pool[ccdBody.hitBodyIndex];
            const hit = visitor.collector.hit;

            // reset scratch manifold
            const contactManifold = _findCCDContacts_contactManifold;
            manifold.resetContactManifold(contactManifold);

            // set base offset (bodyA's COM at start of sweep)
            vec3.copy(contactManifold.baseOffset, bodyA.centerOfMassPosition);

            // generate contact points via clipping
            const maxContactDistance =
                world.settings.narrowphase.speculativeContactDistance + world.settings.narrowphase.manifoldTolerance;

            if (hit.faceA.numVertices >= 2 && hit.faceB.numVertices >= 3) {
                // full face-vs-face clipping
                manifold.manifoldBetweenTwoFaces(
                    contactManifold,
                    hit.pointA,
                    hit.pointB,
                    hit.penetrationAxis,
                    maxContactDistance,
                    hit.faceA,
                    hit.faceB,
                );
            } else {
                // single point contact
                manifold.setContactPoint(contactManifold, 0, hit.pointA, hit.pointB);
                contactManifold.numContactPoints = 1;
            }

            // set metadata
            contactManifold.subShapeIdA = hit.subShapeIdA;
            contactManifold.subShapeIdB = hit.subShapeIdB;
            contactManifold.materialIdA = hit.materialIdA;
            contactManifold.materialIdB = hit.materialIdB;
            contactManifold.penetrationDepth = hit.penetrationDepth;
            vec3.copy(contactManifold.worldSpaceNormal, ccdBody.contactNormal);

            // prune to 4 points max
            if (contactManifold.numContactPoints > MAX_CONTACT_POINTS) {
                manifold.pruneContactPoints(contactManifold, contactManifold.worldSpaceNormal);
            }

            // call listener callbacks (calculates material properties, fires events)
            onCCDContactAdded(world, listener, bodyA, bodyB, contactManifold, ccdBody.contactSettings);

            // handle sensor conversion
            if (ccdBody.contactSettings.isSensor) {
                ccdBody.fractionPlusSlop = 1.0;
                ccdBody.hitBodyIndex = -1;
            } else {
                // calculate average contact point for impulse resolution
                if (contactManifold.numContactPoints > 1) {
                    const avgPoint = vec3.create();
                    for (let j = 0; j < contactManifold.numContactPoints; j++) {
                        const idx = j * 3;
                        avgPoint[0] += contactManifold.relativeContactPointsOnB[idx];
                        avgPoint[1] += contactManifold.relativeContactPointsOnB[idx + 1];
                        avgPoint[2] += contactManifold.relativeContactPointsOnB[idx + 2];
                    }
                    vec3.scale(avgPoint, avgPoint, 1.0 / contactManifold.numContactPoints);
                    vec3.add(ccdBody.contactPoint, contactManifold.baseOffset, avgPoint);
                }
                // else: single point already set in addHit() with bodyB correction
            }
        }

        // clean up after use
        visitor.reset();
    }
}

const _applyCCD_bodyAPosAtHit = /* @__PURE__ */ vec3.create();
const _applyCCD_r1PlusU = /* @__PURE__ */ vec3.create();
const _applyCCD_r2 = /* @__PURE__ */ vec3.create();
const _applyCCD_v1 = /* @__PURE__ */ vec3.create();
const _applyCCD_v2 = /* @__PURE__ */ vec3.create();
const _applyCCD_relVel = /* @__PURE__ */ vec3.create();
const _applyCCD_rotationA = /* @__PURE__ */ mat4.create();
const _applyCCD_rotationB = /* @__PURE__ */ mat4.create();
const _applyCCD_invInertiaA = /* @__PURE__ */ mat4.create();
const _applyCCD_invInertiaB = /* @__PURE__ */ mat4.create();
const _applyCCD_normalScaled = /* @__PURE__ */ vec3.create();
const _applyCCD_tangentVel = /* @__PURE__ */ vec3.create();
const _applyCCD_frictionDir = /* @__PURE__ */ vec3.create();

/** apply collision impulse for CCD contact using constraint solver infrastructure, does a single solve iteration using @see AxisConstraintPart */
function applyCCD(world: World, ccdBody: ccd.CCDBody, bodyA: RigidBody, bodyB: RigidBody): void {
    // skip if bodyA is not dynamic
    if (bodyA.motionType !== MotionType.DYNAMIC) return;

    // calculate r1_plus_u (contact point relative to current COM after partial movement)
    // r1_plus_u = contactPoint - (COM + fraction * delta)
    // this is the point relative to COM at collision time
    vec3.scaleAndAdd(_applyCCD_bodyAPosAtHit, bodyA.centerOfMassPosition, ccdBody.deltaPosition, ccdBody.fraction);
    vec3.subtract(_applyCCD_r1PlusU, ccdBody.contactPoint, _applyCCD_bodyAPosAtHit);

    // calculate r2 (contact point relative to bodyB's current COM)
    vec3.subtract(_applyCCD_r2, ccdBody.contactPoint, bodyB.centerOfMassPosition);

    // calculate relative velocity at contact point using r1_plus_u
    rigidBody.getVelocityAtPointCOM(_applyCCD_v1, bodyA, _applyCCD_r1PlusU);
    if (bodyB.motionType !== MotionType.STATIC) {
        rigidBody.getVelocityAtPointCOM(_applyCCD_v2, bodyB, _applyCCD_r2);
    } else {
        vec3.zero(_applyCCD_v2);
    }
    vec3.subtract(_applyCCD_relVel, _applyCCD_v2, _applyCCD_v1);
    const normalVel = vec3.dot(_applyCCD_relVel, ccdBody.contactNormal);

    // only resolve if approaching (negative normal velocity)
    if (normalVel >= 0) return;

    // get contact settings
    const settings = ccdBody.contactSettings;

    // calculate restitution bias
    let normalVelocityBias = 0;
    if (settings.combinedRestitution > 0 && normalVel < -world.settings.ccd.minVelocityForRestitution) {
        normalVelocityBias = settings.combinedRestitution * normalVel;
    }

    // setup normal constraint
    const normalConstraint = axisConstraintPart.create();

    // get inverse masses with scaling from contact settings
    const invMassA = bodyA.motionType === MotionType.DYNAMIC ? settings.invMassScale1 * bodyA.motionProperties.invMass : 0;
    const invMassB = bodyB.motionType === MotionType.DYNAMIC ? settings.invMassScale2 * bodyB.motionProperties.invMass : 0;

    if (bodyA.motionType === MotionType.DYNAMIC) {
        mat4.fromQuat(_applyCCD_rotationA, bodyA.quaternion);
        motionProperties.getInverseInertiaForRotation(_applyCCD_invInertiaA, bodyA.motionProperties, _applyCCD_rotationA);
    } else {
        mat4.zero(_applyCCD_invInertiaA);
    }

    if (bodyB.motionType === MotionType.DYNAMIC) {
        mat4.fromQuat(_applyCCD_rotationB, bodyB.quaternion);
        motionProperties.getInverseInertiaForRotation(_applyCCD_invInertiaB, bodyB.motionProperties, _applyCCD_rotationB);
    } else {
        mat4.zero(_applyCCD_invInertiaB);
    }

    // setup constraint properties with r1_plus_u
    axisConstraintPart.calculateConstraintPropertiesWithMassOverride(
        normalConstraint,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        settings.invInertiaScale1,
        settings.invInertiaScale2,
        _applyCCD_invInertiaA,
        _applyCCD_invInertiaB,
        _applyCCD_r1PlusU, // r1 + u (movement from hit to current)
        _applyCCD_r2,
        ccdBody.contactNormal,
        normalVelocityBias,
    );

    // one iteration!
    axisConstraintPart.solveVelocityConstraintWithMassOverride(
        normalConstraint,
        bodyA,
        bodyB,
        invMassA,
        invMassB,
        ccdBody.contactNormal,
        -Infinity,
        Infinity, // unbounded for normal constraint
    );

    // apply friction constraint (clamped to friction cone)
    if (settings.combinedFriction > 0) {
        // calculate tangent velocity (remove normal component)
        vec3.scale(_applyCCD_normalScaled, ccdBody.contactNormal, normalVel);
        vec3.subtract(_applyCCD_tangentVel, _applyCCD_relVel, _applyCCD_normalScaled);
        const tangentLenSq = vec3.squaredLength(_applyCCD_tangentVel);

        // JoltPhysics uses 1.0e-12f for squared length check
        if (tangentLenSq > 1e-12) {
            const tangentLen = Math.sqrt(tangentLenSq);
            vec3.scale(_applyCCD_frictionDir, _applyCCD_tangentVel, 1.0 / tangentLen);

            // get max friction from normal lambda (Coulomb friction cone)
            const normalLambda = axisConstraintPart.getTotalLambdaValue(normalConstraint);
            const maxFriction = settings.combinedFriction * normalLambda;

            const frictionConstraint = axisConstraintPart.create();
            axisConstraintPart.calculateConstraintPropertiesWithMassOverride(
                frictionConstraint,
                bodyA,
                bodyB,
                invMassA,
                invMassB,
                settings.invInertiaScale1,
                settings.invInertiaScale2,
                _applyCCD_invInertiaA,
                _applyCCD_invInertiaB,
                _applyCCD_r1PlusU,
                _applyCCD_r2,
                _applyCCD_frictionDir,
                0, // no bias for friction
            );

            axisConstraintPart.solveVelocityConstraintWithMassOverride(
                frictionConstraint,
                bodyA,
                bodyB,
                invMassA,
                invMassB,
                _applyCCD_frictionDir,
                -maxFriction,
                maxFriction, // clamp to friction cone
            );
        }
    }

    // clamp velocities to prevent instability
    motionProperties.clampLinearVelocity(bodyA.motionProperties);
    motionProperties.clampAngularVelocity(bodyA.motionProperties);

    if (bodyB.motionType === MotionType.DYNAMIC) {
        motionProperties.clampLinearVelocity(bodyB.motionProperties);
        motionProperties.clampAngularVelocity(bodyB.motionProperties);
    }
}

/** comparator for sorting CCD bodies by time of impact */
function compareCCDBodies(a: ccd.CCDBody, b: ccd.CCDBody): number {
    // primary sort: time of impact (earliest first)
    if (a.fractionPlusSlop !== b.fractionPlusSlop) {
        return a.fractionPlusSlop - b.fractionPlusSlop;
    }

    // secondary sort: deterministic tie-breaker using bodyIndex
    return a.bodyIndex - b.bodyIndex;
}

/** sort CCD bodies by time of impact (earliest first), uses fractionPlusSlop (includes penetration allowance) for stability */
function sortCCDBodies(state: ccd.CCD): void {
    const count = state.ccdBodies.length;
    if (count <= 1) return; // nothing to sort

    // sort only the active portion of the array
    const ccdBodies = state.ccdBodies;

    // create sorted slice
    const sorted = ccdBodies.slice(0, count).sort(compareCCDBodies);

    // copy sorted array back
    for (let i = 0; i < count; i++) {
        ccdBodies[i] = sorted[i];
    }
}

const _resolveCCDContacts_movement = /* @__PURE__ */ vec3.create();

/** resolve CCD contacts in chronological order, moves bodies to their collision points and applies impulses */
function resolveCCDContacts(world: World): void {
    // process in sorted order (earliest to latest)
    for (let i = 0; i < world.ccd.ccdBodies.length; i++) {
        const ccdBody = world.ccd.ccdBodies[i];
        const bodyA = world.bodies.pool[ccdBody.bodyIndex];

        // handle no collision case
        if (ccdBody.hitBodyIndex === -1) {
            // no collision, move full distance
            // (kinematic bodies already moved in velocity integration, only dynamic needs update)
            if (bodyA.motionType === MotionType.DYNAMIC) {
                vec3.add(bodyA.position, bodyA.position, ccdBody.deltaPosition);
                rigidBody.setTransform(world, bodyA, bodyA.position, bodyA.quaternion, false);
            }
            continue;
        }

        const bodyB = world.bodies.pool[ccdBody.hitBodyIndex];

        // check for cascading collision invalidation
        const ccdBodyBIndex = bodyB.ccdBodyIndex;
        if (ccdBodyBIndex >= 0) {
            // bodyB also has CCD
            const ccdBodyB = world.ccd.ccdBodies[ccdBodyBIndex];

            // if we collide earlier than bodyB, we may invalidate bodyB's collision
            if (ccdBodyB.hitBodyIndex !== -1 && ccdBodyB.fraction > ccdBody.fraction) {
                // we hit first - invalidate bodyB's later collision and shorten its travel
                ccdBodyB.hitBodyIndex = -1;
                ccdBodyB.fractionPlusSlop = ccdBody.fraction;
            }
        }

        // apply collision impulse (only affects dynamic bodies)
        applyCCD(world, ccdBody, bodyA, bodyB);

        // move bodyA to collision point (with penetration allowance)
        // kinematic bodies already moved in velocity integration, only update dynamic
        if (bodyA.motionType === MotionType.DYNAMIC) {
            vec3.scale(_resolveCCDContacts_movement, ccdBody.deltaPosition, ccdBody.fractionPlusSlop);
            vec3.add(bodyA.position, bodyA.position, _resolveCCDContacts_movement);

            // update body transform and broadphase
            rigidBody.setTransform(world, bodyA, bodyA.position, bodyA.quaternion, false);
        }

        // wake up bodyB if it was sleeping
        if (bodyB.sleeping) {
            rigidBody.wake(world, bodyB);
        }
    }
}
