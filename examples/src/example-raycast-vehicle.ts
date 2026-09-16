import type { RigidBody, World } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    CastRayStatus,
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
    MotionType,
    registerAll,
    rigidBody,
    triangleMesh,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import type { Mat4, Quat, Vec3 } from 'math';
import { mat4, quat, vec3 } from 'math';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import * as debugUI from './debug/debug-ui';
import { loadGLTF } from './utils/gltf';
import { color, mix } from 'three/tsl';

const trackGLTF = await loadGLTF('./models/track.glb');

/* vehicle types */

type VehicleState = {
    sliding: boolean;
    currentVehicleSpeedKmHour: number;
};

type WheelState = {
    suspensionLength: number;
    suspensionRelativeVelocity: number;
    suspensionForce: number;
    clippedInvContactDotSuspension: number;

    inContactWithGround: boolean;
    hitPointWorld: Vec3;
    hitNormalWorld: Vec3;

    directionWorld: Vec3;
    axleWorld: Vec3;

    chassisConnectionPointWorld: Vec3;

    sideImpulse: number;
    forwardImpulse: number;

    forwardWS: Vec3;
    axle: Vec3;

    worldTransformPosition: Vec3;
    worldTransformQuaternion: Quat;

    engineForce: number;
    brakeForce: number;
    steering: number;

    rotation: number;
    deltaRotation: number;

    groundBody: RigidBody | null;

    slipInfo: number;
    skidInfo: number;

    sliding: boolean;
};

type WheelOptions = {
    radius: number;

    directionLocal: Vec3;
    axleLocal: Vec3;

    // suspension spring settings
    // frequency: natural frequency in Hz (1.0-2.0 typical for cars)
    // damping: damping ratio, 0 = undamped, 1 = critically damped (0.3-0.7 typical)
    suspensionFrequency: number;
    suspensionDamping: number;

    suspensionRestLength: number;
    maxSuspensionForce: number;
    maxSuspensionTravel: number;

    // optional: custom point to apply suspension force (local space)
    // if not provided, force is applied at mid-suspension point
    suspensionForcePoint: Vec3 | null;

    sideFrictionStiffness: number;
    frictionSlip: number;

    rollInfluence: number;

    customSlidingRotationalSpeed: number;
    useCustomSlidingRotationalSpeed: boolean;

    forwardAcceleration: number;
    sideAcceleration: number;

    chassisConnectionPointLocal: Vec3;
};

type Wheel = {
    state: WheelState;
    options: WheelOptions;
};

type Vehicle = {
    wheels: Wheel[];
    state: VehicleState;
    chassisBody: RigidBody;

    // cached transforms, updated each frame
    chassisWorldMatrix: Mat4;
    chassisInvRotationMatrix: Mat4;

    // raycast filter
    queryFilter: ReturnType<typeof filter.create>;

    // anti-roll bars
    antiRollBars: AntiRollBar[];
};

type AntiRollBar = {
    leftWheel: number;
    rightWheel: number;
    stiffness: number;
};

/* vehicle creation */

type CreateVehicleSettings = {
    chassisBody: RigidBody;
    queryFilter: ReturnType<typeof filter.create>;
};

function createVehicle(settings: CreateVehicleSettings): Vehicle {
    const vehicle: Vehicle = {
        wheels: [],
        state: {
            sliding: false,
            currentVehicleSpeedKmHour: 0,
        },
        chassisBody: settings.chassisBody,
        chassisWorldMatrix: mat4.create(),
        chassisInvRotationMatrix: mat4.create(),
        queryFilter: settings.queryFilter,
        antiRollBars: [],
    };
    return vehicle;
}

const vehicleRayCollector = createClosestCastRayCollector();
const vehicleRaySettings = createDefaultCastRaySettings();

function addAntiRollBar(vehicle: Vehicle, arb: AntiRollBar): void {
    vehicle.antiRollBars.push(arb);
}

function createWheelState(): WheelState {
    return {
        suspensionLength: 0,
        suspensionForce: 0,
        suspensionRelativeVelocity: 0,
        clippedInvContactDotSuspension: 1,
        directionWorld: vec3.create(),
        inContactWithGround: false,
        hitNormalWorld: vec3.create(),
        hitPointWorld: vec3.create(),
        chassisConnectionPointWorld: vec3.create(),
        axleWorld: vec3.create(),
        sideImpulse: 0,
        forwardImpulse: 0,
        forwardWS: vec3.create(),
        axle: vec3.create(),
        worldTransformPosition: vec3.create(),
        worldTransformQuaternion: quat.create(),
        steering: 0,
        brakeForce: 0,
        engineForce: 0,
        rotation: 0,
        deltaRotation: 0,
        groundBody: null,
        slipInfo: 0,
        skidInfo: 0,
        sliding: false,
    };
}

function addWheel(vehicle: Vehicle, options: WheelOptions): number {
    const wheel: Wheel = {
        options,
        state: createWheelState(),
    };
    vehicle.wheels.push(wheel);
    return vehicle.wheels.length - 1;
}

function setEngineForce(vehicle: Vehicle, force: number, wheelIndex: number): void {
    vehicle.wheels[wheelIndex].state.engineForce = force;
}

function setSteeringValue(vehicle: Vehicle, steering: number, wheelIndex: number): void {
    vehicle.wheels[wheelIndex].state.steering = steering;
}

function setBrakeValue(vehicle: Vehicle, brake: number, wheelIndex: number): void {
    vehicle.wheels[wheelIndex].state.brakeForce = brake;
}

/* helpers */
function updateChassisMatrices(vehicle: Vehicle): void {
    mat4.fromRotationTranslation(vehicle.chassisWorldMatrix, vehicle.chassisBody.quaternion, vehicle.chassisBody.position);
    mat4.fromQuat(vehicle.chassisInvRotationMatrix, vehicle.chassisBody.quaternion);
    mat4.transpose(vehicle.chassisInvRotationMatrix, vehicle.chassisInvRotationMatrix);
}

// mat4 is column-major: column 0 is indices 0,1,2; column 1 is 4,5,6; column 2 is 8,9,10
function getVehicleAxisWorld(out: Vec3, vehicle: Vehicle, axisIndex: number): Vec3 {
    const col = axisIndex * 4;
    out[0] = vehicle.chassisWorldMatrix[col];
    out[1] = vehicle.chassisWorldMatrix[col + 1];
    out[2] = vehicle.chassisWorldMatrix[col + 2];
    return out;
}

const _resolveBilateral_vel1 = vec3.create();
const _resolveBilateral_vel2 = vec3.create();
const _resolveBilateral_vel = vec3.create();

// bilateral constraint between two dynamic objects
function resolveSingleBilateralConstraint(body1: RigidBody, pos1: Vec3, body2: RigidBody, pos2: Vec3, normal: Vec3): number {
    const normalLenSqr = vec3.squaredLength(normal);
    if (normalLenSqr > 1.1) {
        return 0;
    }

    rigidBody.getVelocityAtPoint(_resolveBilateral_vel1, body1, pos1);
    rigidBody.getVelocityAtPoint(_resolveBilateral_vel2, body2, pos2);

    vec3.subtract(_resolveBilateral_vel, _resolveBilateral_vel1, _resolveBilateral_vel2);

    const relVel = vec3.dot(normal, _resolveBilateral_vel);

    const contactDamping = 0.2;

    const body1InvMass = body1.motionProperties.invMass;
    const body2InvMass = body2.motionProperties.invMass;

    const massTerm = 1 / (body1InvMass + body2InvMass);
    const impulse = -contactDamping * relVel * massTerm;

    return impulse;
}

const _computeImpulseDenominator_r0 = vec3.create();
const _computeImpulseDenominator_c0 = vec3.create();
const _computeImpulseDenominator_vec = vec3.create();
const _computeImpulseDenominator_m = vec3.create();
const _computeImpulseDenominator_invInertia = mat4.create();

// compute impulse denominator for rolling friction
function computeImpulseDenominator(body: RigidBody, pos: Vec3, normal: Vec3): number {
    const r0 = _computeImpulseDenominator_r0;
    const c0 = _computeImpulseDenominator_c0;
    const vec = _computeImpulseDenominator_vec;
    const m = _computeImpulseDenominator_m;

    // get world-space inverse inertia
    rigidBody.getInverseInertia(_computeImpulseDenominator_invInertia, body);

    vec3.subtract(r0, pos, body.position);
    vec3.cross(c0, r0, normal);

    // multiply c0 by inverse inertia matrix
    vec3.transformMat4(m, c0, _computeImpulseDenominator_invInertia);

    vec3.cross(vec, m, r0);

    return body.motionProperties.invMass + vec3.dot(normal, vec);
}

const _calcRollingFriction_vel1 = vec3.create();
const _calcRollingFriction_vel2 = vec3.create();
const _calcRollingFriction_vel = vec3.create();

// calculate rolling friction
function calcRollingFriction(
    body0: RigidBody,
    body1: RigidBody,
    frictionPosWorld: Vec3,
    frictionDirectionWorld: Vec3,
    maxImpulse: number,
): number {
    const contactPosWorld = frictionPosWorld;

    rigidBody.getVelocityAtPoint(_calcRollingFriction_vel1, body0, contactPosWorld);
    rigidBody.getVelocityAtPoint(_calcRollingFriction_vel2, body1, contactPosWorld);
    vec3.subtract(_calcRollingFriction_vel, _calcRollingFriction_vel1, _calcRollingFriction_vel2);

    const vrel = vec3.dot(frictionDirectionWorld, _calcRollingFriction_vel);

    const denom0 = computeImpulseDenominator(body0, frictionPosWorld, frictionDirectionWorld);
    const denom1 = computeImpulseDenominator(body1, frictionPosWorld, frictionDirectionWorld);

    const relaxation = 1;
    const jacDiagABInv = relaxation / (denom0 + denom1);

    let j1 = -vrel * jacDiagABInv;

    if (maxImpulse < j1) {
        j1 = maxImpulse;
    }
    if (j1 < -maxImpulse) {
        j1 = -maxImpulse;
    }

    return j1;
}

/* vehicle */

function resetStates(vehicle: Vehicle): void {
    vehicle.state.sliding = false;

    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];
        wheel.state.inContactWithGround = false;
        wheel.state.groundBody = null;
    }
}

function updateWheelTransformWorld(vehicle: Vehicle, wheel: Wheel): void {
    vec3.transformMat4(
        wheel.state.chassisConnectionPointWorld,
        wheel.options.chassisConnectionPointLocal,
        vehicle.chassisWorldMatrix,
    );
    mat4.multiply3x3Vec(wheel.state.directionWorld, vehicle.chassisWorldMatrix, wheel.options.directionLocal);
    mat4.multiply3x3Vec(wheel.state.axleWorld, vehicle.chassisWorldMatrix, wheel.options.axleLocal);
}

const _updateWheelTransform_up = vec3.create();
const _updateWheelTransform_right = vec3.create();
const _updateWheelTransform_fwd = vec3.create();
const _updateWheelTransform_steeringQuat = quat.create();
const _updateWheelTransform_rotatingQuat = quat.create();
const _updateWheelTransform_chassisQuat = quat.create();
const _updateWheelTransform_tmpQuat = quat.create();

function updateWheelTransform(vehicle: Vehicle): void {
    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        const up = _updateWheelTransform_up;
        const right = _updateWheelTransform_right;
        const fwd = _updateWheelTransform_fwd;

        updateWheelTransformWorld(vehicle, wheel);

        // up = -directionLocal
        vec3.copy(up, wheel.options.directionLocal);
        vec3.scale(up, up, -1);

        // right = axleLocal
        vec3.copy(right, wheel.options.axleLocal);

        // fwd = up x right
        vec3.cross(fwd, up, right);
        vec3.normalize(fwd, fwd);
        vec3.normalize(right, right);

        // rotate around steering over the wheel axle
        const steering = wheel.state.steering;
        const steeringQuat = _updateWheelTransform_steeringQuat;
        quat.setAxisAngle(steeringQuat, up, steering);

        const rotatingQuat = _updateWheelTransform_rotatingQuat;
        quat.setAxisAngle(rotatingQuat, right, wheel.state.rotation);

        // world rotation of the wheel
        quat.copy(_updateWheelTransform_chassisQuat, vehicle.chassisBody.quaternion);
        quat.multiply(_updateWheelTransform_tmpQuat, _updateWheelTransform_chassisQuat, steeringQuat);
        quat.multiply(wheel.state.worldTransformQuaternion, _updateWheelTransform_tmpQuat, rotatingQuat);
        quat.normalize(wheel.state.worldTransformQuaternion, wheel.state.worldTransformQuaternion);

        // world position of the wheel
        vec3.copy(wheel.state.worldTransformPosition, wheel.state.directionWorld);
        vec3.scale(wheel.state.worldTransformPosition, wheel.state.worldTransformPosition, wheel.state.suspensionLength);
        vec3.add(wheel.state.worldTransformPosition, wheel.state.worldTransformPosition, wheel.state.chassisConnectionPointWorld);
    }
}

const _updateCurrentSpeed_chassisVelocity = vec3.create();
const _updateCurrentSpeed_forwardWorld = vec3.create();

function updateCurrentSpeed(vehicle: Vehicle): void {
    const chassis = vehicle.chassisBody;
    vec3.copy(_updateCurrentSpeed_chassisVelocity, chassis.motionProperties.linearVelocity);

    vehicle.state.currentVehicleSpeedKmHour = 3.6 * vec3.length(_updateCurrentSpeed_chassisVelocity);

    getVehicleAxisWorld(_updateCurrentSpeed_forwardWorld, vehicle, 2);

    if (vec3.dot(_updateCurrentSpeed_forwardWorld, _updateCurrentSpeed_chassisVelocity) > 0) {
        vehicle.state.currentVehicleSpeedKmHour *= -1;
    }
}

const _updateWheelSuspension_hitPoint = vec3.create();
const _updateWheelSuspension_hitNormal = vec3.create();
const _updateWheelSuspension_direction = vec3.create();
const _updateWheelSuspension_chassisVelAtContact = vec3.create();

const _computeEffectiveMass_forcePoint = vec3.create();
const _computeEffectiveMass_forcePointCrossUp = vec3.create();
const _computeEffectiveMass_negUp = vec3.create();
const _computeEffectiveMass_temp = vec3.create();

function updateWheelSuspension(world: World, vehicle: Vehicle): void {
    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        updateWheelTransformWorld(vehicle, wheel);

        // ray length should cover full suspension travel + wheel radius
        // this matches jolt: ray_length = mSuspensionMaxLength + wheel_radius
        // where our max length = suspensionRestLength + maxSuspensionTravel
        const maxSuspensionLength = wheel.options.suspensionRestLength + wheel.options.maxSuspensionTravel;
        const rayLength = wheel.options.radius + maxSuspensionLength;

        const origin = wheel.state.chassisConnectionPointWorld;

        // use the transformed suspension direction (rotates with chassis)
        vec3.copy(_updateWheelSuspension_direction, wheel.state.directionWorld);
        vec3.normalize(_updateWheelSuspension_direction, _updateWheelSuspension_direction);

        // exclude chassis body from raycast using body filter
        const chassisId = vehicle.chassisBody.id;
        vehicle.queryFilter.bodyFilter = (body: RigidBody) => body.id !== chassisId;

        // perform raycast
        vehicleRayCollector.reset();
        castRay(
            world,
            vehicleRayCollector,
            vehicleRaySettings,
            origin,
            _updateWheelSuspension_direction,
            rayLength,
            vehicle.queryFilter,
        );

        const hit = vehicleRayCollector.hit;

        // get the hit body from the body id
        const hitBody = hit.status === CastRayStatus.COLLIDING ? rigidBody.get(world, hit.bodyIdB) : undefined;

        if (hit.status === CastRayStatus.COLLIDING && hitBody) {
            // store ground body
            wheel.state.groundBody = hitBody;

            // update wheel state
            wheel.state.inContactWithGround = true;

            // compute hit point: origin + direction * (fraction * rayLength)
            const hitDistance = hit.fraction * rayLength;
            vec3.scaleAndAdd(_updateWheelSuspension_hitPoint, origin, _updateWheelSuspension_direction, hitDistance);
            vec3.copy(wheel.state.hitPointWorld, _updateWheelSuspension_hitPoint);

            // get hit normal using surface normal query
            rigidBody.getSurfaceNormal(_updateWheelSuspension_hitNormal, hitBody, wheel.state.hitPointWorld, hit.subShapeId);
            vec3.copy(wheel.state.hitNormalWorld, _updateWheelSuspension_hitNormal);

            // compute suspension length
            wheel.state.suspensionLength = hitDistance - wheel.options.radius;

            // clamp on max suspension travel
            const minSuspensionLength = wheel.options.suspensionRestLength - wheel.options.maxSuspensionTravel;
            const maxSuspensionLength = wheel.options.suspensionRestLength + wheel.options.maxSuspensionTravel;

            if (wheel.state.suspensionLength < minSuspensionLength) {
                wheel.state.suspensionLength = minSuspensionLength;
            }
            if (wheel.state.suspensionLength > maxSuspensionLength) {
                wheel.state.suspensionLength = maxSuspensionLength;

                wheel.state.groundBody = null;
                wheel.state.inContactWithGround = false;
                vec3.zero(wheel.state.hitNormalWorld);
                vec3.zero(wheel.state.hitPointWorld);
            }

            const denominator = vec3.dot(wheel.state.hitNormalWorld, wheel.state.directionWorld);

            rigidBody.getVelocityAtPoint(
                _updateWheelSuspension_chassisVelAtContact,
                vehicle.chassisBody,
                wheel.state.hitPointWorld,
            );

            const projVel = vec3.dot(wheel.state.hitNormalWorld, _updateWheelSuspension_chassisVelAtContact);

            if (denominator >= -0.1) {
                wheel.state.suspensionRelativeVelocity = 0;
                wheel.state.clippedInvContactDotSuspension = 1 / 0.1;
            } else {
                const inv = -1 / denominator;
                wheel.state.suspensionRelativeVelocity = projVel * inv;
                wheel.state.clippedInvContactDotSuspension = inv;
            }
        } else {
            // put wheel info as in rest position
            wheel.state.suspensionLength = wheel.options.suspensionRestLength;
            wheel.state.suspensionRelativeVelocity = 0;
            vec3.copy(wheel.state.hitNormalWorld, wheel.state.directionWorld);
            vec3.scale(wheel.state.hitNormalWorld, wheel.state.hitNormalWorld, -1);
            wheel.state.clippedInvContactDotSuspension = 1.0;
        }

        // calculate suspension force
        wheel.state.suspensionForce = 0;

        if (wheel.state.inContactWithGround) {
            const suspensionRestLength = wheel.options.suspensionRestLength;
            const currentLength = wheel.state.suspensionLength;
            const lengthDifference = suspensionRestLength - currentLength;

            // compute stiffness/damping from frequency and effective mass
            // this makes suspension behavior independent of vehicle mass distribution

            // calculate the force application point in local space
            // if suspensionForcePoint is provided, use it; otherwise use mid-suspension point
            let forcePointLocal: Vec3;
            if (wheel.options.suspensionForcePoint !== null) {
                forcePointLocal = wheel.options.suspensionForcePoint;
            } else {
                // use wheel connection point + half suspension length in suspension direction
                vec3.copy(_computeEffectiveMass_forcePoint, wheel.options.chassisConnectionPointLocal);
                vec3.scaleAndAdd(
                    _computeEffectiveMass_forcePoint,
                    _computeEffectiveMass_forcePoint,
                    wheel.options.directionLocal,
                    0.5 * (wheel.options.suspensionRestLength + wheel.options.maxSuspensionTravel),
                );
                forcePointLocal = _computeEffectiveMass_forcePoint;
            }

            // compute effective mass: K = 1 / (J M^-1 J^T)
            // where J is the jacobian for the constraint
            // for a point constraint along the up direction:
            // effective_mass = 1 / (invMass + (r × up) · (invInertia · (r × up)))
            // where r is the force point relative to center of mass

            const mp = vehicle.chassisBody.motionProperties;
            const invMass = mp.invMass;

            // get up direction: suspension points down, so up = -directionLocal
            vec3.negate(_computeEffectiveMass_negUp, wheel.options.directionLocal);
            const negUp = _computeEffectiveMass_negUp;

            // r × (-up) = forcePoint × (-up)
            vec3.cross(_computeEffectiveMass_forcePointCrossUp, forcePointLocal, negUp);

            // compute invInertia · (r × up) using the diagonal inverse inertia
            // in local space, invInertia is diagonal so this is element-wise multiply
            const invInertiaDiag = mp.invInertiaDiagonal;
            const rCrossUp = _computeEffectiveMass_forcePointCrossUp;
            const invInertiaTimesRCrossUp = _computeEffectiveMass_temp;
            vec3.set(
                invInertiaTimesRCrossUp,
                rCrossUp[0] * invInertiaDiag[0],
                rCrossUp[1] * invInertiaDiag[1],
                rCrossUp[2] * invInertiaDiag[2],
            );

            // effective_mass = 1 / (invMass + (r × up) · (invInertia · (r × up)))
            const angularContribution = vec3.dot(rCrossUp, invInertiaTimesRCrossUp);
            const effectiveMass = 1.0 / (invMass + angularContribution);

            // convert frequency and damping ratio to stiffness and damping
            // stiffness = effectiveMass * (2π * frequency)²
            // damping = 2 * effectiveMass * dampingRatio * 2π * frequency
            const omega = 2.0 * Math.PI * wheel.options.suspensionFrequency;
            let stiffness = effectiveMass * omega * omega;
            let damping = 2.0 * effectiveMass * wheel.options.suspensionDamping * omega;

            // apply cos(angle) correction for angled surfaces
            // this adjusts stiffness/damping based on angle between suspension and contact normal
            // clamp to 1/0.1 = 10x max increase
            const cosAngleCorrection = wheel.state.clippedInvContactDotSuspension;
            stiffness *= cosAngleCorrection;
            damping *= cosAngleCorrection;

            // spring force: F = k * x
            let force = stiffness * lengthDifference;

            // damper force: F = -c * v
            const projectedRelativeVelocity = wheel.state.suspensionRelativeVelocity;
            force -= damping * projectedRelativeVelocity;

            wheel.state.suspensionForce = force;

            if (wheel.state.suspensionForce < 0) {
                wheel.state.suspensionForce = 0;
            }
        }
    }
}

const _applyWheelSuspensionForce_impulse = vec3.create();

function applyWheelSuspensionForce(world: World, vehicle: Vehicle, delta: number): void {
    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        let suspensionForce = wheel.state.suspensionForce;
        if (suspensionForce > wheel.options.maxSuspensionForce) {
            suspensionForce = wheel.options.maxSuspensionForce;
        }

        vec3.copy(_applyWheelSuspensionForce_impulse, wheel.state.hitNormalWorld);
        vec3.scale(_applyWheelSuspensionForce_impulse, _applyWheelSuspensionForce_impulse, suspensionForce * delta);

        rigidBody.addImpulseAtPosition(world, vehicle.chassisBody, _applyWheelSuspensionForce_impulse, wheel.state.hitPointWorld);
    }

    // apply anti-roll bar forces
    for (const arb of vehicle.antiRollBars) {
        if (arb.stiffness <= 0) continue;

        const lw = vehicle.wheels[arb.leftWheel];
        const rw = vehicle.wheels[arb.rightWheel];

        if (lw.state.inContactWithGround && rw.state.inContactWithGround) {
            const diff = rw.state.suspensionLength - lw.state.suspensionLength;
            const impulse = diff * arb.stiffness * delta;

            // apply to left wheel (push up if diff is positive)
            vec3.copy(_applyWheelSuspensionForce_impulse, lw.state.hitNormalWorld);
            vec3.scale(_applyWheelSuspensionForce_impulse, _applyWheelSuspensionForce_impulse, -impulse);
            rigidBody.addImpulseAtPosition(
                world,
                vehicle.chassisBody,
                _applyWheelSuspensionForce_impulse,
                lw.state.hitPointWorld,
            );

            // apply to right wheel
            vec3.copy(_applyWheelSuspensionForce_impulse, rw.state.hitNormalWorld);
            vec3.scale(_applyWheelSuspensionForce_impulse, _applyWheelSuspensionForce_impulse, impulse);
            rigidBody.addImpulseAtPosition(
                world,
                vehicle.chassisBody,
                _applyWheelSuspensionForce_impulse,
                rw.state.hitPointWorld,
            );
        }
    }
}

const _updateFriction_surfNormalScaledProj = vec3.create();
const _updateFriction_impulse = vec3.create();
const _updateFriction_sideImp = vec3.create();
const _updateFriction_worldPos = vec3.create();
const _updateFriction_relPos = vec3.create();
const _updateFriction_rollInfluenceAdjustedWorldPos = vec3.create();

const directions: Vec3[] = [vec3.fromValues(1, 0, 0), vec3.fromValues(0, 1, 0), vec3.fromValues(0, 0, 1)];

function updateFriction(world: World, vehicle: Vehicle, delta: number): void {
    const surfNormalScaledProj = _updateFriction_surfNormalScaledProj;

    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        wheel.state.sideImpulse = 0;
        wheel.state.forwardImpulse = 0;

        if (wheel.state.inContactWithGround && wheel.state.groundBody) {
            const axle = wheel.state.axle;
            const forwardWS = wheel.state.forwardWS;

            // get world axle from wheel world transform quaternion
            vec3.transformQuat(axle, directions[0], wheel.state.worldTransformQuaternion);

            const surfNormalWS = wheel.state.hitNormalWorld;
            const proj = vec3.dot(axle, surfNormalWS);

            vec3.copy(surfNormalScaledProj, surfNormalWS);
            vec3.scale(surfNormalScaledProj, surfNormalScaledProj, proj);
            vec3.subtract(axle, axle, surfNormalScaledProj);
            vec3.normalize(axle, axle);

            vec3.cross(forwardWS, surfNormalWS, axle);
            vec3.normalize(forwardWS, forwardWS);

            wheel.state.sideImpulse = resolveSingleBilateralConstraint(
                vehicle.chassisBody,
                wheel.state.hitPointWorld,
                wheel.state.groundBody,
                wheel.state.hitPointWorld,
                axle,
            );

            wheel.state.sideImpulse *= wheel.options.sideFrictionStiffness;
        }
    }

    const sideFactor = 1;
    const fwdFactor = 0.5;

    vehicle.state.sliding = false;

    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        let rollingFriction = 0;

        wheel.state.slipInfo = 1;

        if (wheel.state.groundBody) {
            const defaultRollingFrictionImpulse = 0;

            const maxImpulse = wheel.state.brakeForce ? wheel.state.brakeForce : defaultRollingFrictionImpulse;

            // brake
            rollingFriction = calcRollingFriction(
                vehicle.chassisBody,
                wheel.state.groundBody,
                wheel.state.hitPointWorld,
                wheel.state.forwardWS,
                maxImpulse,
            );

            // acceleration
            rollingFriction += wheel.state.engineForce * delta;

            const factor = maxImpulse / rollingFriction;
            wheel.state.slipInfo *= factor;
        }

        // switch between active rolling (throttle), braking and non-active rolling friction
        wheel.state.forwardImpulse = 0;
        wheel.state.skidInfo = 1;

        if (wheel.state.groundBody) {
            const maxImp = wheel.state.suspensionForce * delta * wheel.options.frictionSlip;
            const maxImpSide = maxImp;

            const maxImpSquared = maxImp * maxImpSide;

            wheel.state.forwardImpulse = rollingFriction;

            const x = (wheel.state.forwardImpulse * fwdFactor) / wheel.options.forwardAcceleration;
            const y = (wheel.state.sideImpulse * sideFactor) / wheel.options.sideAcceleration;

            const impulseSquared = x * x + y * y;

            wheel.state.sliding = false;
            if (impulseSquared > maxImpSquared) {
                vehicle.state.sliding = true;
                wheel.state.sliding = true;

                const factor = maxImp / Math.sqrt(impulseSquared);

                wheel.state.skidInfo *= factor;
            }
        }
    }

    if (vehicle.state.sliding) {
        for (let i = 0; i < vehicle.wheels.length; i++) {
            const wheel = vehicle.wheels[i];

            if (wheel.state.sideImpulse !== 0) {
                if (wheel.state.skidInfo < 1) {
                    wheel.state.forwardImpulse *= wheel.state.skidInfo;
                    wheel.state.sideImpulse *= wheel.state.skidInfo;
                }
            }
        }
    }

    // apply the impulses
    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        vec3.copy(_updateFriction_worldPos, wheel.state.hitPointWorld);

        vec3.copy(_updateFriction_relPos, _updateFriction_worldPos);
        vec3.subtract(_updateFriction_relPos, _updateFriction_relPos, vehicle.chassisBody.position);

        if (wheel.state.forwardImpulse !== 0) {
            vec3.copy(_updateFriction_impulse, wheel.state.forwardWS);
            vec3.scale(_updateFriction_impulse, _updateFriction_impulse, wheel.state.forwardImpulse);

            rigidBody.addImpulseAtPosition(world, vehicle.chassisBody, _updateFriction_impulse, _updateFriction_worldPos);
        }

        if (wheel.state.sideImpulse !== 0) {
            const chassisBody = vehicle.chassisBody;
            const groundBody = wheel.state.groundBody!;

            const worldPos2 = wheel.state.hitPointWorld;

            vec3.copy(_updateFriction_sideImp, wheel.state.axle);
            vec3.scale(_updateFriction_sideImp, _updateFriction_sideImp, wheel.state.sideImpulse);

            const rollInfluenceAdjustedWorldPos = _updateFriction_rollInfluenceAdjustedWorldPos;

            // scale the relative position in the up direction with rollInfluence
            mat4.multiply3x3Vec(rollInfluenceAdjustedWorldPos, vehicle.chassisInvRotationMatrix, _updateFriction_relPos);

            rollInfluenceAdjustedWorldPos[1] *= wheel.options.rollInfluence;

            mat4.multiply3x3Vec(rollInfluenceAdjustedWorldPos, vehicle.chassisWorldMatrix, rollInfluenceAdjustedWorldPos);

            // back to world pos
            vec3.add(rollInfluenceAdjustedWorldPos, rollInfluenceAdjustedWorldPos, vehicle.chassisBody.position);

            rigidBody.addImpulseAtPosition(world, chassisBody, _updateFriction_sideImp, rollInfluenceAdjustedWorldPos);

            // apply friction impulse on the ground
            vec3.scale(_updateFriction_sideImp, _updateFriction_sideImp, -1);

            rigidBody.addImpulseAtPosition(world, groundBody, _updateFriction_sideImp, worldPos2);
        }
    }
}

const _updateWheelRotation_hitNormalScaledWithProj = vec3.create();
const _updateWheelRotation_fwd = vec3.create();
const _updateWheelRotation_vel = vec3.create();

function updateWheelRotation(vehicle: Vehicle, delta: number): void {
    const hitNormalScaledWithProj = _updateWheelRotation_hitNormalScaledWithProj;
    const fwd = _updateWheelRotation_fwd;
    const vel = _updateWheelRotation_vel;

    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        rigidBody.getVelocityAtPoint(vel, vehicle.chassisBody, wheel.state.chassisConnectionPointWorld);

        // wheel rotation based on velocity at contact point
        // positive rotation = wheel rolls forward (car moves in +Z direction)
        if (wheel.state.inContactWithGround) {
            getVehicleAxisWorld(fwd, vehicle, 2);

            const proj = vec3.dot(fwd, wheel.state.hitNormalWorld);

            vec3.copy(hitNormalScaledWithProj, wheel.state.hitNormalWorld);
            vec3.scale(hitNormalScaledWithProj, hitNormalScaledWithProj, proj);

            vec3.subtract(fwd, fwd, hitNormalScaledWithProj);

            const proj2 = vec3.dot(fwd, vel);

            wheel.state.deltaRotation = (proj2 * delta) / wheel.options.radius;
        }

        if (
            (wheel.state.sliding || !wheel.state.inContactWithGround) &&
            wheel.state.engineForce !== 0 &&
            wheel.options.useCustomSlidingRotationalSpeed
        ) {
            // apply custom rotation when accelerating and sliding
            wheel.state.deltaRotation =
                (wheel.state.engineForce > 0 ? 1 : -1) * wheel.options.customSlidingRotationalSpeed * delta;
        }

        // lock wheels
        if (Math.abs(wheel.state.brakeForce) > Math.abs(wheel.state.engineForce)) {
            wheel.state.deltaRotation = 0;
        }

        wheel.state.rotation += wheel.state.deltaRotation;
        wheel.state.deltaRotation *= 0.99; // damping of rotation when not in contact
    }
}

function updateVehicle(world: World, vehicle: Vehicle, delta: number): void {
    updateChassisMatrices(vehicle);
    resetStates(vehicle);

    updateWheelTransform(vehicle);
    updateCurrentSpeed(vehicle);
    updateWheelSuspension(world, vehicle);
    applyWheelSuspensionForce(world, vehicle, delta);
    updateFriction(world, vehicle, delta);
    updateWheelRotation(vehicle, delta);
}

/* debug helpers */

type VehicleDebug = {
    wheelHelpers: THREE.Group[];
    suspensionLines: THREE.Line[];
    rayHitLines: THREE.Line[];
    rayMissLines: THREE.Line[];
};

function createVehicleDebug(vehicle: Vehicle): VehicleDebug {
    const wheelHelpers: THREE.Group[] = [];
    const suspensionLines: THREE.Line[] = [];
    const rayHitLines: THREE.Line[] = [];
    const rayMissLines: THREE.Line[] = [];

    for (let i = 0; i < vehicle.wheels.length; i++) {
        // wheel helper group
        const wheelGroup = new THREE.Group();
        wheelHelpers.push(wheelGroup);

        // suspension line (cyan - shows actual suspension compression)
        const suspensionGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
        const suspensionMat = new THREE.LineBasicMaterial({ color: 0x00ffff, depthTest: false, depthWrite: false });
        const suspensionLine = new THREE.Line(suspensionGeom, suspensionMat);
        suspensionLine.renderOrder = 999;
        suspensionLine.frustumCulled = false;
        suspensionLines.push(suspensionLine);

        // ray hit line (green - origin to hit point)
        const rayHitGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
        const rayHitMat = new THREE.LineBasicMaterial({
            color: 0x00ff00,
            depthTest: false,
            depthWrite: false,
        });
        const rayHitLine = new THREE.Line(rayHitGeom, rayHitMat);
        rayHitLine.renderOrder = 999;
        rayHitLine.frustumCulled = false;
        rayHitLines.push(rayHitLine);

        // ray miss line (red - hit point to max ray length, or full ray if no hit)
        const rayMissGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
        const rayMissMat = new THREE.LineBasicMaterial({
            color: 0xff0000,
            transparent: true,
            opacity: 0.5,
            depthTest: false,
            depthWrite: false,
        });
        const rayMissLine = new THREE.Line(rayMissGeom, rayMissMat);
        rayMissLine.renderOrder = 999;
        rayMissLine.frustumCulled = false;
        rayMissLines.push(rayMissLine);
    }

    return {
        wheelHelpers,
        suspensionLines,
        rayHitLines,
        rayMissLines,
    };
}

const _updateVehicleDebug_suspOffset = vec3.create();

function updateVehicleDebug(vehicle: Vehicle, debug: VehicleDebug): void {
    const offsetX = 0.1; // offset to separate the lines visually

    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];

        // update wheel helper transform
        const wheelHelper = debug.wheelHelpers[i];
        wheelHelper.position.set(
            wheel.state.worldTransformPosition[0],
            wheel.state.worldTransformPosition[1],
            wheel.state.worldTransformPosition[2],
        );
        wheelHelper.quaternion.set(
            wheel.state.worldTransformQuaternion[0],
            wheel.state.worldTransformQuaternion[1],
            wheel.state.worldTransformQuaternion[2],
            wheel.state.worldTransformQuaternion[3],
        );

        const origin = wheel.state.chassisConnectionPointWorld;
        const dirX = wheel.state.directionWorld[0];
        const dirY = wheel.state.directionWorld[1];
        const dirZ = wheel.state.directionWorld[2];
        const maxSuspensionLength = wheel.options.suspensionRestLength + wheel.options.maxSuspensionTravel;
        const rayLength = wheel.options.radius + maxSuspensionLength;

        // suspension offset: left wheels (-X), right wheels (+X), rotated by chassis orientation
        const localOffset = i % 2 === 0 ? -offsetX : offsetX;
        vec3.set(_updateVehicleDebug_suspOffset, localOffset, 0, 0);
        vec3.transformQuat(_updateVehicleDebug_suspOffset, _updateVehicleDebug_suspOffset, vehicle.chassisBody.quaternion);

        // compute hit distance (or full ray length if no hit)
        const hitDistance = wheel.state.inContactWithGround ? wheel.state.suspensionLength + wheel.options.radius : rayLength;

        // ray hit line (green) - on the wheel spot
        const rayHitLine = debug.rayHitLines[i];
        const rayHitPos = rayHitLine.geometry.attributes.position.array as Float32Array;
        rayHitPos[0] = origin[0];
        rayHitPos[1] = origin[1];
        rayHitPos[2] = origin[2];
        rayHitPos[3] = origin[0] + dirX * hitDistance;
        rayHitPos[4] = origin[1] + dirY * hitDistance;
        rayHitPos[5] = origin[2] + dirZ * hitDistance;
        rayHitLine.geometry.attributes.position.needsUpdate = true;
        rayHitLine.visible = wheel.state.inContactWithGround;

        // ray miss line (red) - on the wheel spot
        const rayMissLine = debug.rayMissLines[i];
        const rayMissPos = rayMissLine.geometry.attributes.position.array as Float32Array;
        if (wheel.state.inContactWithGround) {
            // from hit point to end of ray
            rayMissPos[0] = origin[0] + dirX * hitDistance;
            rayMissPos[1] = origin[1] + dirY * hitDistance;
            rayMissPos[2] = origin[2] + dirZ * hitDistance;
        } else {
            // no hit - start from origin
            rayMissPos[0] = origin[0];
            rayMissPos[1] = origin[1];
            rayMissPos[2] = origin[2];
        }
        rayMissPos[3] = origin[0] + dirX * rayLength;
        rayMissPos[4] = origin[1] + dirY * rayLength;
        rayMissPos[5] = origin[2] + dirZ * rayLength;
        rayMissLine.geometry.attributes.position.needsUpdate = true;

        // update suspension line (cyan) - offset to outside of vehicle
        const suspensionLine = debug.suspensionLines[i];
        const suspensionPos = suspensionLine.geometry.attributes.position.array as Float32Array;
        const suspLen = Math.max(0, wheel.state.suspensionLength);
        suspensionPos[0] = origin[0] + _updateVehicleDebug_suspOffset[0];
        suspensionPos[1] = origin[1] + _updateVehicleDebug_suspOffset[1];
        suspensionPos[2] = origin[2] + _updateVehicleDebug_suspOffset[2];
        suspensionPos[3] = origin[0] + _updateVehicleDebug_suspOffset[0] + dirX * suspLen;
        suspensionPos[4] = origin[1] + _updateVehicleDebug_suspOffset[1] + dirY * suspLen;
        suspensionPos[5] = origin[2] + _updateVehicleDebug_suspOffset[2] + dirZ * suspLen;
        suspensionLine.geometry.attributes.position.needsUpdate = true;
    }
}

/* camera rig */

type CameraRig = {
    camera: THREE.PerspectiveCamera;
    currentPosition: Vec3;
    currentLookAt: Vec3;
    idealOffset: Vec3;
    idealLookAt: Vec3;
};

function createCameraRig(camera: THREE.PerspectiveCamera): CameraRig {
    return {
        camera,
        currentPosition: vec3.fromValues(15, 15, 0),
        currentLookAt: vec3.create(),
        idealOffset: vec3.fromValues(0, 3, 10), // behind the vehicle in -Z direction
        idealLookAt: vec3.fromValues(0, 1, 0),
    };
}

const _updateCameraRig_offset = vec3.create();
const _updateCameraRig_lookAt = vec3.create();
const _updateCameraRig_tmpPosition = new THREE.Vector3();
const _updateCameraRig_tmpLookAt = new THREE.Vector3();

function initCameraRig(state: CameraRig, chassisBody: RigidBody): void {
    // compute initial camera position and look-at based on chassis transform
    vec3.copy(_updateCameraRig_offset, state.idealOffset);
    vec3.transformQuat(_updateCameraRig_offset, _updateCameraRig_offset, chassisBody.quaternion);
    vec3.add(state.currentPosition, _updateCameraRig_offset, chassisBody.position);

    vec3.copy(_updateCameraRig_lookAt, state.idealLookAt);
    vec3.transformQuat(_updateCameraRig_lookAt, _updateCameraRig_lookAt, chassisBody.quaternion);
    vec3.add(state.currentLookAt, _updateCameraRig_lookAt, chassisBody.position);

    // set three.js camera
    state.camera.position.set(state.currentPosition[0], state.currentPosition[1], state.currentPosition[2]);
    state.camera.lookAt(state.currentLookAt[0], state.currentLookAt[1], state.currentLookAt[2]);
}

function updateCameraRig(state: CameraRig, chassisBody: RigidBody, delta: number): void {
    const t = 1.0 - 0.01 ** delta;

    // compute ideal offset in world space
    vec3.copy(_updateCameraRig_offset, state.idealOffset);
    vec3.transformQuat(_updateCameraRig_offset, _updateCameraRig_offset, chassisBody.quaternion);
    vec3.add(_updateCameraRig_offset, _updateCameraRig_offset, chassisBody.position);

    // prevent camera from going below ground
    if (_updateCameraRig_offset[1] < 0.5) {
        _updateCameraRig_offset[1] = 0.5;
    }

    // compute ideal look-at in world space
    vec3.copy(_updateCameraRig_lookAt, state.idealLookAt);
    vec3.transformQuat(_updateCameraRig_lookAt, _updateCameraRig_lookAt, chassisBody.quaternion);
    vec3.add(_updateCameraRig_lookAt, _updateCameraRig_lookAt, chassisBody.position);

    // lerp towards ideal positions
    vec3.lerp(state.currentPosition, state.currentPosition, _updateCameraRig_offset, t);
    vec3.lerp(state.currentLookAt, state.currentLookAt, _updateCameraRig_lookAt, t);

    // update three.js camera
    _updateCameraRig_tmpPosition.set(state.currentPosition[0], state.currentPosition[1], state.currentPosition[2]);
    _updateCameraRig_tmpLookAt.set(state.currentLookAt[0], state.currentLookAt[1], state.currentLookAt[2]);

    state.camera.position.copy(_updateCameraRig_tmpPosition);
    state.camera.lookAt(_updateCameraRig_tmpLookAt);
}

/* input */

type Controls = {
    forward: boolean;
    backward: boolean;
    left: boolean;
    right: boolean;
    brake: boolean;
};

function createControls(): Controls {
    const controls: Controls = {
        forward: false,
        backward: false,
        left: false,
        right: false,
        brake: false,
    };

    const onKeyDown = (event: KeyboardEvent) => {
        switch (event.code) {
            case 'KeyW':
            case 'ArrowUp':
                controls.forward = true;
                break;
            case 'KeyS':
            case 'ArrowDown':
                controls.backward = true;
                break;
            case 'KeyA':
            case 'ArrowLeft':
                controls.left = true;
                break;
            case 'KeyD':
            case 'ArrowRight':
                controls.right = true;
                break;
            case 'Space':
                controls.brake = true;
                break;
        }
    };

    const onKeyUp = (event: KeyboardEvent) => {
        switch (event.code) {
            case 'KeyW':
            case 'ArrowUp':
                controls.forward = false;
                break;
            case 'KeyS':
            case 'ArrowDown':
                controls.backward = false;
                break;
            case 'KeyA':
            case 'ArrowLeft':
                controls.left = false;
                break;
            case 'KeyD':
            case 'ArrowRight':
                controls.right = false;
                break;
            case 'Space':
                controls.brake = false;
                break;
        }
    };

    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);

    return controls;
}

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color('#c4eeff');

scene.add(trackGLTF.scene);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

const onResize = () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
};

window.addEventListener('resize', onResize);
onResize();

const ambientLight = new THREE.AmbientLight(0xffffff, 1);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
directionalLight.position.set(20, 30, 10);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 2048;
directionalLight.shadow.mapSize.height = 2048;
directionalLight.shadow.camera.near = 0.5;
directionalLight.shadow.camera.far = 100;
directionalLight.shadow.camera.left = -50;
directionalLight.shadow.camera.right = 50;
directionalLight.shadow.camera.top = 50;
directionalLight.shadow.camera.bottom = -50;
scene.add(directionalLight);

/* physics world */

registerAll();

const worldSettings = createWorldSettings();

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

// create query filter for vehicle raycasts
const queryFilter = filter.create(worldSettings.layers);
filter.enableAllLayers(queryFilter, worldSettings.layers);

// create triangle mesh collider from trackGLTF
const trackPositions: number[] = [];
const trackIndices: number[] = [];

trackGLTF.scene.traverse((object) => {
    if (object instanceof THREE.Mesh) {
        const geometry = object.geometry as THREE.BufferGeometry;
        const positions = geometry.getAttribute('position');
        const index = geometry.getIndex();

        // get world transform for this mesh
        object.updateWorldMatrix(true, false);
        const worldMatrix = object.matrixWorld;

        // offset for indices
        const indexOffset = trackPositions.length / 3;

        // add positions transformed to world space
        const tempVec = new THREE.Vector3();
        for (let i = 0; i < positions.count; i++) {
            tempVec.fromBufferAttribute(positions, i);
            tempVec.applyMatrix4(worldMatrix);
            trackPositions.push(tempVec.x, tempVec.y, tempVec.z);
        }

        // add indices
        if (index) {
            for (let i = 0; i < index.count; i++) {
                trackIndices.push(index.getX(i) + indexOffset);
            }
        } else {
            // if no index, create triangles from non-indexed geometry
            for (let i = 0; i < positions.count; i += 3) {
                trackIndices.push(i + indexOffset, i + 1 + indexOffset, i + 2 + indexOffset);
            }
        }
    }
});

if (trackPositions.length > 0) {
    const trackShape = triangleMesh.create({
        positions: trackPositions,
        indices: trackIndices,
    });
    rigidBody.create(world, {
        shape: trackShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 0, 0),
        restitution: 0,
        friction: 0.5,
    });
}

/* setup vehicle */

const chassisHalfWidth = 1;
const chassisHalfHeight = 0.55;
const chassisHalfLength = 2.35;

// create chassis body
const chassisShape = box.create({
    halfExtents: [chassisHalfWidth, chassisHalfHeight, chassisHalfLength],
    convexRadius: 0.05,
});
const chassisBody = rigidBody.create(world, {
    shape: chassisShape,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-16, 10, 90),
    quaternion: quat.fromDegrees(quat.create(), 0, -60, 0, 'yxz'),
    mass: 150,
    restitution: 0,
    friction: 0.5,
});

// create vehicle
const vehicle = createVehicle({
    chassisBody,
    queryFilter,
});

const wheelRadius = 0.38;
const wheelWidth = 0.3;
const suspensionRestLength = 0.3;
const vehicleWidth = 2.2;
const vehicleHeight = -0.3;
const vehicleFront = -1.35;
const vehicleBack = 1.3;

const commonWheelOptions: Omit<WheelOptions, 'chassisConnectionPointLocal'> = {
    radius: wheelRadius,
    directionLocal: vec3.fromValues(0, -1, 0),
    axleLocal: vec3.fromValues(1, 0, 0),

    // suspension spring settings
    // frequency: natural frequency in Hz - higher = stiffer (typical car: 1.0-2.0 Hz)
    // damping: damping ratio - 0 = undamped, 1 = critically damped (typical car: 0.3-0.7)
    suspensionFrequency: 1.5,
    suspensionDamping: 0.5,

    suspensionRestLength,
    maxSuspensionForce: 100000,
    maxSuspensionTravel: 0.3,

    // no custom force point - apply at mid-suspension point
    suspensionForcePoint: null,

    sideFrictionStiffness: 1.5,
    frictionSlip: 1.6,

    rollInfluence: 0.2,

    customSlidingRotationalSpeed: -30,
    useCustomSlidingRotationalSpeed: true,

    forwardAcceleration: 1,
    sideAcceleration: 1,
};

// front left
addWheel(vehicle, {
    ...commonWheelOptions,
    chassisConnectionPointLocal: vec3.fromValues(-vehicleWidth * 0.5, vehicleHeight, vehicleFront),
});

// front right
addWheel(vehicle, {
    ...commonWheelOptions,
    chassisConnectionPointLocal: vec3.fromValues(vehicleWidth * 0.5, vehicleHeight, vehicleFront),
});

// back left
addWheel(vehicle, {
    ...commonWheelOptions,
    chassisConnectionPointLocal: vec3.fromValues(-vehicleWidth * 0.5, vehicleHeight, vehicleBack),
});

// back right
addWheel(vehicle, {
    ...commonWheelOptions,
    chassisConnectionPointLocal: vec3.fromValues(vehicleWidth * 0.5, vehicleHeight, vehicleBack),
});

// add anti-roll bars
addAntiRollBar(vehicle, { leftWheel: 0, rightWheel: 1, stiffness: 500 }); // front axle
addAntiRollBar(vehicle, { leftWheel: 2, rightWheel: 3, stiffness: 500 }); // rear axle

// create chassis mesh
const chassisMesh = new THREE.Mesh(
    new THREE.BoxGeometry(chassisHalfWidth * 2, chassisHalfHeight * 2, chassisHalfLength * 2),
    new THREE.MeshStandardMaterial({ color: 0x2244aa }),
);
chassisMesh.castShadow = true;
scene.add(chassisMesh);

// create wheel meshes
const wheelMeshes: THREE.Group[] = [];
const wheelGeometry = new THREE.CylinderGeometry(wheelRadius, wheelRadius, wheelWidth, 24);
const wheelMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });
const wheelEdgesGeometry = new THREE.EdgesGeometry(wheelGeometry);
const wheelEdgesMaterial = new THREE.LineBasicMaterial({ color: 0xffffff });

for (let i = 0; i < vehicle.wheels.length; i++) {
    const wheelMesh = new THREE.Mesh(wheelGeometry, wheelMaterial);
    // rotate to align cylinder with X axis (wheel rotation axis)
    wheelMesh.rotation.z = Math.PI / 2;
    const wheelEdges = new THREE.LineSegments(wheelEdgesGeometry, wheelEdgesMaterial);
    wheelEdges.rotation.z = Math.PI / 2;
    const wheelGroup = new THREE.Group();
    wheelGroup.add(wheelMesh);
    wheelGroup.add(wheelEdges);
    wheelGroup.castShadow = true;
    scene.add(wheelGroup);
    wheelMeshes.push(wheelGroup);
}

// create vehicle debug visuals
const vehicleDebug = createVehicleDebug(vehicle);
for (let i = 0; i < vehicle.wheels.length; i++) {
    scene.add(vehicleDebug.suspensionLines[i]);
    scene.add(vehicleDebug.rayHitLines[i]);
    scene.add(vehicleDebug.rayMissLines[i]);
}

/* controls and camera */

const controls = createControls();
const cameraRig = createCameraRig(camera);
initCameraRig(cameraRig, vehicle.chassisBody);
const orbitControls = new OrbitControls(camera, renderer.domElement);

// vehicle tuning settings
const vehicleSettings = {
    maxForce: 500,
    maxSteer: 0.5,
    maxBrake: 4,

    // spring settings (frequency/damping mode)
    suspensionFrequency: commonWheelOptions.suspensionFrequency,
    suspensionDamping: commonWheelOptions.suspensionDamping,
    maxSuspensionTravel: commonWheelOptions.maxSuspensionTravel,
    maxSuspensionForce: commonWheelOptions.maxSuspensionForce,

    frictionSlip: commonWheelOptions.frictionSlip,
    sideFrictionStiffness: commonWheelOptions.sideFrictionStiffness,
    rollInfluence: commonWheelOptions.rollInfluence,
};

// debug visibility (we read these each frame to set line visibility)
const vehicleDebugVisibility = {
    suspensionLines: true,
    rayLines: true,
};

// camera settings
const cameraSettings = {
    enableCameraRig: true,
};

/* debug ui */

const options = debugRenderer.createDefaultOptions();
options.bodies.enabled = false;
options.bodies.wireframe = true;

const debugRendererState = debugRenderer.init(options);
debugRendererState.object3d.position.y += 0.01;
scene.add(debugRendererState.object3d);

const ui = debugUI.init(options);

// vehicle debug visibility folder
const vehicleDebugFolder = ui.gui.addFolder('Vehicle Debug');
vehicleDebugFolder.add(vehicleDebugVisibility, 'suspensionLines').name('Suspension Lines');
vehicleDebugFolder.add(vehicleDebugVisibility, 'rayLines').name('Ray Lines (green=hit, red=miss)');
vehicleDebugFolder.add(cameraSettings, 'enableCameraRig').name('Enable Camera Rig');
vehicleDebugFolder.open();

// vehicle tuning folder
const vehicleTuningFolder = ui.gui.addFolder('Vehicle Tuning');
vehicleTuningFolder.add(vehicleSettings, 'maxForce', 0, 1000).name('Max Force');
vehicleTuningFolder.add(vehicleSettings, 'maxSteer', 0, 1).name('Max Steer');
vehicleTuningFolder.add(vehicleSettings, 'maxBrake', 0, 10).name('Max Brake');
vehicleTuningFolder.add(vehicleSettings, 'suspensionFrequency', 0.5, 5).name('Suspension Freq (Hz)');
vehicleTuningFolder.add(vehicleSettings, 'suspensionDamping', 0, 1).name('Suspension Damping');
vehicleTuningFolder.add(vehicleSettings, 'maxSuspensionTravel', 0.1, 1).name('Max Suspension Travel');
vehicleTuningFolder.add(vehicleSettings, 'maxSuspensionForce', 1000, 200000).name('Max Suspension Force');
vehicleTuningFolder.add(vehicleSettings, 'frictionSlip', 0.1, 5).name('Friction Slip');
vehicleTuningFolder.add(vehicleSettings, 'sideFrictionStiffness', 0.1, 3).name('Side Friction Stiffness');
vehicleTuningFolder.add(vehicleSettings, 'rollInfluence', 0, 1).name('Roll Influence');
vehicleTuningFolder.add(vehicle.antiRollBars[0], 'stiffness', 0, 2000).name('Front ARB Stiffness');
vehicleTuningFolder.add(vehicle.antiRollBars[1], 'stiffness', 0, 2000).name('Rear ARB Stiffness');
vehicleTuningFolder.open();

/* loop */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // update vehicle from controls
    let engineForce = 0;
    let steering = 0;

    if (controls.forward) {
        engineForce += vehicleSettings.maxForce;
    }
    if (controls.backward) {
        engineForce -= vehicleSettings.maxForce;
    }
    if (controls.left) {
        steering += vehicleSettings.maxSteer;
    }
    if (controls.right) {
        steering -= vehicleSettings.maxSteer;
    }

    const brakeForce = controls.brake ? vehicleSettings.maxBrake : 0;

    // apply tuning settings to all wheels
    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];
        wheel.options.suspensionFrequency = vehicleSettings.suspensionFrequency;
        wheel.options.suspensionDamping = vehicleSettings.suspensionDamping;
        wheel.options.maxSuspensionTravel = vehicleSettings.maxSuspensionTravel;
        wheel.options.maxSuspensionForce = vehicleSettings.maxSuspensionForce;
        wheel.options.frictionSlip = vehicleSettings.frictionSlip;
        wheel.options.sideFrictionStiffness = vehicleSettings.sideFrictionStiffness;
        wheel.options.rollInfluence = vehicleSettings.rollInfluence;

        setBrakeValue(vehicle, brakeForce, i);
    }

    // steer front wheels (indices 0 and 1)
    setSteeringValue(vehicle, steering, 0);
    setSteeringValue(vehicle, steering, 1);

    // apply engine force to back wheels (indices 2 and 3)
    setEngineForce(vehicle, engineForce, 2);
    setEngineForce(vehicle, engineForce, 3);

    // update vehicle physics (raycast, compute forces, apply impulses)
    updateVehicle(world, vehicle, delta);

    // step physics world (integrates forces)
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // update wheel transforms after physics step so they match the new chassis position
    updateChassisMatrices(vehicle);
    updateWheelTransform(vehicle);

    // update debug renderer
    debugRenderer.update(debugRendererState, world);

    // update chassis mesh
    chassisMesh.position.set(chassisBody.position[0], chassisBody.position[1], chassisBody.position[2]);
    chassisMesh.quaternion.set(
        chassisBody.quaternion[0],
        chassisBody.quaternion[1],
        chassisBody.quaternion[2],
        chassisBody.quaternion[3],
    );

    // update wheel meshes
    for (let i = 0; i < vehicle.wheels.length; i++) {
        const wheel = vehicle.wheels[i];
        const wheelMesh = wheelMeshes[i];

        wheelMesh.position.set(
            wheel.state.worldTransformPosition[0],
            wheel.state.worldTransformPosition[1],
            wheel.state.worldTransformPosition[2],
        );
        wheelMesh.quaternion.set(
            wheel.state.worldTransformQuaternion[0],
            wheel.state.worldTransformQuaternion[1],
            wheel.state.worldTransformQuaternion[2],
            wheel.state.worldTransformQuaternion[3],
        );
    }

    // update vehicle debug visuals
    updateVehicleDebug(vehicle, vehicleDebug);

    // update debug visibility
    for (let i = 0; i < vehicle.wheels.length; i++) {
        vehicleDebug.suspensionLines[i].visible = vehicleDebugVisibility.suspensionLines;
        vehicleDebug.rayHitLines[i].visible = vehicleDebugVisibility.rayLines;
        vehicleDebug.rayMissLines[i].visible = vehicleDebugVisibility.rayLines;
    }

    // update camera
    orbitControls.enabled = !cameraSettings.enableCameraRig;
    if (!cameraSettings.enableCameraRig) {
        orbitControls.target.set(chassisBody.position[0], chassisBody.position[1], chassisBody.position[2]);
        orbitControls.update();
    } else {
        updateCameraRig(cameraRig, chassisBody, delta);
    }

    // render
    renderer.render(scene, camera);
}

animate();
