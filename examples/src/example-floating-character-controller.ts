import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    CastRayStatus,
    ConstraintSpace,
    capsule,
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    distanceConstraint,
    emptyShape,
    enableCollision,
    type Filter,
    filter,
    MotionType,
    type RigidBody,
    registerAll,
    rigidBody,
    type Shape,
    SpringMode,
    sphere,
    triangleMesh,
    updateWorld,
    type World,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import type { Vec3 } from 'mathcat';
import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import * as debugUI from './debug/debug-ui';

// reference: https://github.com/pmndrs/ecctrl

type FloatingCharacterState = {
    // core components
    body: RigidBody;
    shape: Shape;

    // configuration
    capsuleRadius: number;
    capsuleHalfHeight: number;
    floatHeight: number;

    // floating spring-damper parameters
    floatSpringK: number;
    floatDampingC: number;

    // movement parameters
    maxWalkSpeed: number;
    maxRunSpeed: number;
    accelerationTime: number;
    turnSpeed: number;
    turnVelMultiplier: number;
    airControlFactor: number;
    rejectVelMult: number;
    dragDampingC: number;
    moveImpulsePointY: number;

    // slope parameters
    maxSlopeAngle: number;
    slopeRayOffset: number;
    slopeUpExtraForce: number;
    slopeDownExtraForce: number;

    // auto-balance parameters
    enableAutoBalance: boolean;
    balanceSpringK: number;
    balanceDampingC: number;
    balanceSpringOnY: number;
    balanceDampingOnY: number;

    // jump parameters
    jumpVelocity: number;
    jumpForceToGroundMult: number;
    slopeJumpMult: number;
    sprintJumpMult: number;

    // gravity parameters
    normalGravityScale: number;
    fallingGravityScale: number;
    maxFallSpeed: number;

    // ray parameters
    rayHitForgiveness: number;

    // runtime state - ground detection
    isOnGround: boolean;
    isFalling: boolean;
    canJump: boolean;
    slopeAngle: number;
    actualSlopeNormal: Vec3;
    actualSlopeAngle: number;
    groundBodyId: number | null;
    groundSubShapeId: number;
    groundPosition: Vec3;
    groundDistance: number;

    // runtime state - moving platforms
    isOnMovingObject: boolean;
    massRatio: number;
    movingObjectVelocity: Vec3;

    // runtime state - rotation
    characterModelIndicator: { y: number };
    characterRotated: boolean;

    // runtime state - contact forces
    bodyContactForce: Vec3;
    characterMassForce: Vec3;

    // input state
    input: {
        moveDirection: Vec3;
        wantToRun: boolean;
        wantToJump: boolean;
    };
};

type FloatingCharacterControllerOptions = {
    capsuleRadius: number;
    capsuleHalfHeight: number;
    floatHeight: number;
    floatSpringK: number;
    floatDampingC: number;
    maxWalkSpeed: number;
    maxRunSpeed: number;
    accelerationTime: number;
    turnSpeed: number;
    turnVelMultiplier: number;
    airControlFactor: number;
    rejectVelMult: number;
    dragDampingC: number;
    moveImpulsePointY: number;
    maxSlopeAngle: number;
    slopeUpExtraForce: number;
    slopeDownExtraForce: number;
    enableAutoBalance: boolean;
    balanceSpringK: number;
    balanceDampingC: number;
    balanceSpringOnY: number;
    balanceDampingOnY: number;
    jumpVelocity: number;
    jumpForceToGroundMult: number;
    slopeJumpMult: number;
    sprintJumpMult: number;
    normalGravityScale: number;
    fallingGravityScale: number;
    maxFallSpeed: number;
    rayHitForgiveness: number;
};

function createFloatingCharacterControllerOptions(): FloatingCharacterControllerOptions {
    return {
        capsuleRadius: 0.5,
        capsuleHalfHeight: 0.5,
        floatHeight: 0.3,
        floatSpringK: 1.2,
        floatDampingC: 0.08,
        maxWalkSpeed: 2.5,
        maxRunSpeed: 5.0,
        accelerationTime: 8,
        turnSpeed: 40,
        turnVelMultiplier: 0.2,
        airControlFactor: 0.2,
        rejectVelMult: 4.0,
        dragDampingC: 0.15,
        moveImpulsePointY: 0.5,
        maxSlopeAngle: 1.0,
        slopeUpExtraForce: 0.1,
        slopeDownExtraForce: 0.2,
        enableAutoBalance: true,
        balanceSpringK: 0.2,
        balanceDampingC: 0.1,
        balanceSpringOnY: 0.3,
        balanceDampingOnY: 0.05,
        jumpVelocity: 4.0,
        jumpForceToGroundMult: 5.0,
        slopeJumpMult: 0.25,
        sprintJumpMult: 1.2,
        normalGravityScale: 1.0,
        fallingGravityScale: 2.5,
        maxFallSpeed: 20.0,
        rayHitForgiveness: 0.1,
    };
}

// initialization
function initFloatingCharacterController(
    world: World,
    position: Vec3,
    objectLayer: number,
    options: FloatingCharacterControllerOptions,
): FloatingCharacterState {
    const slopeRayOffset = options.capsuleRadius - 0.03;

    // create capsule shape
    const shape = capsule.create({
        halfHeightOfCylinder: options.capsuleHalfHeight,
        radius: options.capsuleRadius,
    });

    // create dynamic rigidbody
    const body = rigidBody.create(world, {
        shape,
        motionType: MotionType.DYNAMIC,
        position,
        objectLayer,
        friction: 0.5,
        restitution: 0.0,
        linearDamping: 0.0,
        angularDamping: 0.0,
        mass: 1.0,
        allowedDegreesOfFreedom: options.enableAutoBalance ? 0b111111 : 0b111000, // all if balance enabled, else translation only
    });

    // set gravity factor
    body.motionProperties.gravityFactor = options.normalGravityScale;

    return {
        body,
        shape,
        ...options,
        slopeRayOffset,
        isOnGround: false,
        isFalling: false,
        canJump: false,
        slopeAngle: 0,
        actualSlopeNormal: vec3.fromValues(0, 1, 0),
        actualSlopeAngle: 0,
        groundBodyId: null,
        groundSubShapeId: 0,
        groundPosition: vec3.create(),
        groundDistance: 0,
        isOnMovingObject: false,
        massRatio: 1,
        movingObjectVelocity: vec3.create(),
        characterModelIndicator: { y: 0 },
        characterRotated: true,
        bodyContactForce: vec3.create(),
        characterMassForce: vec3.create(),
        input: {
            moveDirection: vec3.create(),
            wantToRun: false,
            wantToJump: false,
        },
    };
}

const _updateGroundDetection_rayOrigin: Vec3 = vec3.create();
const _updateSlopeDetection_forward: Vec3 = vec3.create();
const _updateSlopeDetection_slopeRayOrigin: Vec3 = vec3.create();
const _applyMovementForce_indicatorQuat = quat.create();
const _applyMovementForce_movingObjectVelocityInCharacterDir: Vec3 = vec3.create();
const _applyAutoBalanceTorque_desiredForward: Vec3 = vec3.create();
const _applyAutoBalanceTorque_crossX: Vec3 = vec3.create();
const _applyAutoBalanceTorque_crossY: Vec3 = vec3.create();
const _applyAutoBalanceTorque_crossZ: Vec3 = vec3.create();
const _updateMovingPlatform_distanceFromCharacterToObject: Vec3 = vec3.create();
const _updateMovingPlatform_objectAngvelToLinvel: Vec3 = vec3.create();
const _updateMovingPlatform_velocityDiff: Vec3 = vec3.create();
const _handleJump_jumpDirection: Vec3 = vec3.create();
const _handleJump_projection: Vec3 = vec3.create();
const _updateCharacterRotation_moveDir: Vec3 = vec3.create();
const _animate_oldPosition: Vec3 = vec3.create();
const _animate_deltaPos: Vec3 = vec3.create();
const _animate_targetRotation = quat.create();
const _animate_deltaRotation = quat.create();

const rayCollector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();

const ignoreSingleBodyChainedBodyFilterState = {
    bodyId: -1,
    innerBodyFilter: undefined as ((body: RigidBody) => boolean) | undefined,
};

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

function setIgnoreSingleBodyFilter(filter: Filter, ignoreBodyId: number): void {
    ignoreSingleBodyChainedBodyFilterState.bodyId = ignoreBodyId;
    ignoreSingleBodyChainedBodyFilterState.innerBodyFilter = filter.bodyFilter;
    filter.bodyFilter = ignoreSingleBodyChainedBodyFilter;
}

function resetIgnoreSingleBodyFilter(filter: Filter): void {
    filter.bodyFilter = ignoreSingleBodyChainedBodyFilterState.innerBodyFilter;
    ignoreSingleBodyChainedBodyFilterState.bodyId = -1;
    ignoreSingleBodyChainedBodyFilterState.innerBodyFilter = undefined;
}

function updateGroundDetection(world: World, character: FloatingCharacterState, filter: Filter): void {
    const bodyPos = character.body.position;
    _updateGroundDetection_rayOrigin[0] = bodyPos[0];
    _updateGroundDetection_rayOrigin[1] = bodyPos[1] - character.capsuleHalfHeight;
    _updateGroundDetection_rayOrigin[2] = bodyPos[2];

    const rayLength = character.capsuleRadius + 2.0;

    setIgnoreSingleBodyFilter(filter, character.body.id);

    // cast ray
    rayCollector.reset();
    castRay(world, rayCollector, raySettings, _updateGroundDetection_rayOrigin, [0, -1, 0], rayLength, filter);

    resetIgnoreSingleBodyFilter(filter);

    const floatingDis = character.capsuleRadius + character.floatHeight;

    if (rayCollector.hit.status === CastRayStatus.COLLIDING) {
        const hitDistance = rayCollector.hit.fraction * rayLength;

        // check if close enough to ground (without forgiveness for ground detection)
        if (hitDistance < floatingDis + character.rayHitForgiveness) {
            character.isOnGround = true;
            character.groundDistance = hitDistance;

            // get ground position
            character.groundPosition = [
                _updateGroundDetection_rayOrigin[0],
                _updateGroundDetection_rayOrigin[1] - hitDistance,
                _updateGroundDetection_rayOrigin[2],
            ];

            // get ground body id and sub-shape id (for platform interaction)
            character.groundBodyId = rayCollector.hit.bodyIdB;
            character.groundSubShapeId = rayCollector.hit.subShapeId;

            // canJump will be set by slope detection (requires valid slope check)
        } else {
            character.isOnGround = false;
            character.canJump = false;
            character.groundBodyId = null;
            character.groundDistance = 0;
            character.actualSlopeAngle = 0;
        }
    } else {
        character.isOnGround = false;
        character.canJump = false;
        character.groundBodyId = null;
        character.groundSubShapeId = 0;
        character.groundDistance = 0;
        character.actualSlopeAngle = 0;
    }
}

function updateSlopeDetection(world: World, character: FloatingCharacterState, filter: Filter): void {
    if (!character.isOnGround) {
        character.slopeAngle = 0;
        character.actualSlopeNormal = [0, 1, 0];
        character.canJump = false;
        return;
    }

    // get character forward direction
    const bodyRot = character.body.quaternion;
    _updateSlopeDetection_forward[0] = 0;
    _updateSlopeDetection_forward[1] = 0;
    _updateSlopeDetection_forward[2] = 1;
    vec3.transformQuat(_updateSlopeDetection_forward, _updateSlopeDetection_forward, bodyRot);

    // slope ray origin: offset forward from character center
    const bodyPos = character.body.position;
    _updateSlopeDetection_slopeRayOrigin[0] = bodyPos[0] + _updateSlopeDetection_forward[0] * character.slopeRayOffset;
    _updateSlopeDetection_slopeRayOrigin[1] = bodyPos[1] - character.capsuleHalfHeight;
    _updateSlopeDetection_slopeRayOrigin[2] = bodyPos[2] + _updateSlopeDetection_forward[2] * character.slopeRayOffset;

    const rayLength = character.capsuleRadius + 3.0;

    setIgnoreSingleBodyFilter(filter, character.body.id);

    rayCollector.reset();
    castRay(world, rayCollector, raySettings, _updateSlopeDetection_slopeRayOrigin, [0, -1, 0], rayLength, filter);

    resetIgnoreSingleBodyFilter(filter);

    const floatingDis = character.capsuleRadius + character.floatHeight;

    // require both main ray and slope ray
    if (rayCollector.hit.status === CastRayStatus.COLLIDING && rayCollector.hit.fraction * rayLength < floatingDis + 0.5) {
        const slopeRayDistance = rayCollector.hit.fraction * rayLength;
        const mainRayDistance = character.groundDistance;

        // calculate slope angle from height difference
        const slopeAngle = Math.atan((mainRayDistance - slopeRayDistance) / character.slopeRayOffset);

        character.slopeAngle = Number(slopeAngle.toFixed(2));

        // get actual surface normal and validate slope
        if (character.groundBodyId !== null) {
            const groundBody = rigidBody.get(world, character.groundBodyId);
            if (groundBody) {
                rigidBody.getSurfaceNormal(
                    character.actualSlopeNormal,
                    groundBody,
                    character.groundPosition,
                    character.groundSubShapeId,
                );

                // calculate actual slope angle from normal
                character.actualSlopeAngle = Math.acos(
                    Math.max(-1, Math.min(1, vec3.dot(character.actualSlopeNormal, [0, 1, 0] as Vec3))),
                );

                // canJump only if slope is walkable AND slope ray hit
                if (character.actualSlopeAngle < character.maxSlopeAngle) {
                    character.canJump = true;
                } else {
                    character.canJump = false;
                }
            } else {
                character.canJump = false;
            }
        } else {
            character.canJump = false;
        }
    } else {
        character.slopeAngle = 0;
        character.actualSlopeNormal = [0, 1, 0];
        character.actualSlopeAngle = 0;
        character.canJump = false;
    }
}

function applyFloatingForce(world: World, character: FloatingCharacterState): void {
    if (!character.isOnGround || character.groundBodyId === null) {
        return;
    }

    // get ground body from ID
    const groundBody = rigidBody.get(world, character.groundBodyId);
    if (!groundBody) {
        return;
    }

    const floatingDis = character.capsuleRadius + character.floatHeight;
    const displacement = floatingDis - character.groundDistance;

    const velocity = character.body.motionProperties.linearVelocity;
    const verticalVelocity = velocity[1];

    // spring-damper formula: F = k * x - c * v
    const floatingForce = character.floatSpringK * displacement - character.floatDampingC * verticalVelocity;

    // apply directly as per-frame impulse
    const impulse: Vec3 = [0, floatingForce, 0];
    rigidBody.addImpulse(world, character.body, impulse);

    // track character mass force for jump application
    character.characterMassForce[0] = 0;
    character.characterMassForce[1] = floatingForce > 0 ? -floatingForce : 0;
    character.characterMassForce[2] = 0;

    // newton's third law: apply opposite force to ground (dynamic or kinematic)
    if (groundBody.motionType === MotionType.DYNAMIC || groundBody.motionType === MotionType.KINEMATIC) {
        rigidBody.addImpulseAtPosition(world, groundBody, character.characterMassForce, character.groundPosition);
    }
}

function applyMovementForce(world: World, character: FloatingCharacterState): void {
    const run = character.input.wantToRun;
    const maxVelLimit = run ? character.maxRunSpeed : character.maxWalkSpeed;

    // setup moving direction based on slope
    let movingDirection: Vec3 = [0, 0, 1];

    // only apply slope angle to moving direction when slope is walkable
    if (
        character.actualSlopeAngle < character.maxSlopeAngle &&
        Math.abs(character.slopeAngle) > 0.2 &&
        Math.abs(character.slopeAngle) < character.maxSlopeAngle
    ) {
        movingDirection = [0, Math.sin(character.slopeAngle), Math.cos(character.slopeAngle)];
    }
    // if on a maxSlopeAngle slope, only apply small amount of forward direction
    else if (character.actualSlopeAngle >= character.maxSlopeAngle) {
        movingDirection = [
            0,
            Math.sin(character.slopeAngle) > 0 ? 0 : Math.sin(character.slopeAngle),
            Math.sin(character.slopeAngle) > 0 ? 0.1 : 1,
        ];
    }

    // apply character model indicator rotation to moving direction
    quat.setAxisAngle(_applyMovementForce_indicatorQuat, [0, 1, 0], character.characterModelIndicator.y);
    vec3.transformQuat(movingDirection, movingDirection, _applyMovementForce_indicatorQuat);

    // current velocity
    const currentVel = character.body.motionProperties.linearVelocity;

    // calculate moving object velocity direction according to character moving direction
    const movingObjectDot = vec3.dot(character.movingObjectVelocity, movingDirection);
    vec3.scale(_applyMovementForce_movingObjectVelocityInCharacterDir, movingDirection, movingObjectDot);

    // calculate angle between moving object velocity direction and character moving direction
    const angleBetweenCharacterDirAndObjectDir = Math.acos(
        Math.max(
            -1,
            Math.min(
                1,
                vec3.dot(character.movingObjectVelocity, movingDirection) /
                    (vec3.length(character.movingObjectVelocity) * vec3.length(movingDirection) + 0.0001),
            ),
        ),
    );

    // setup rejection velocity (currently only work on ground)
    const wantToMoveMag = currentVel[0] * movingDirection[0] + currentVel[2] * movingDirection[2];
    const wantToMoveVel: Vec3 = [movingDirection[0] * wantToMoveMag, 0, movingDirection[2] * wantToMoveMag];
    const rejectVel: Vec3 = [currentVel[0] - wantToMoveVel[0], 0, currentVel[2] - wantToMoveVel[2]];

    // calculate required acceleration and force: a = Δv/Δt
    // if on moving/rotating platform, apply platform velocity to Δv accordingly
    // also apply reject velocity when character is moving opposite of movement direction
    const moveAccNeeded: Vec3 = [
        (movingDirection[0] * (maxVelLimit + _applyMovementForce_movingObjectVelocityInCharacterDir[0]) -
            (currentVel[0] -
                character.movingObjectVelocity[0] * Math.sin(angleBetweenCharacterDirAndObjectDir) +
                rejectVel[0] * (character.isOnMovingObject ? 0 : character.rejectVelMult))) /
            character.accelerationTime,
        0,
        (movingDirection[2] * (maxVelLimit + _applyMovementForce_movingObjectVelocityInCharacterDir[2]) -
            (currentVel[2] -
                character.movingObjectVelocity[2] * Math.sin(angleBetweenCharacterDirAndObjectDir) +
                rejectVel[2] * (character.isOnMovingObject ? 0 : character.rejectVelMult))) /
            character.accelerationTime,
    ];

    // wanted move force: F = ma (mass = 1.0)
    const moveForceNeeded: Vec3 = vec3.clone(moveAccNeeded);

    // apply turnVelMultiplier if character hasn't completed turning
    const controlMult = character.characterRotated ? 1 : character.turnVelMultiplier;
    const airMult = character.canJump ? 1 : character.airControlFactor;

    // calculate slope impulse Y component
    let slopeImpulseY = 0;
    if (character.slopeAngle !== 0) {
        slopeImpulseY =
            movingDirection[1] *
            (movingDirection[1] > 0 ? character.slopeUpExtraForce : character.slopeDownExtraForce) *
            (run ? character.maxRunSpeed / character.maxWalkSpeed : 1);
    }

    const moveImpulse: Vec3 = [
        moveForceNeeded[0] * controlMult * airMult,
        slopeImpulseY,
        moveForceNeeded[2] * controlMult * airMult,
    ];

    // apply impulse at offset point
    const currentPos = character.body.position;
    const impulsePoint: Vec3 = [currentPos[0], currentPos[1] + character.moveImpulsePointY, currentPos[2]];

    rigidBody.addImpulseAtPosition(world, character.body, moveImpulse, impulsePoint);

    // apply opposite drag force to standing platform
    if (character.isOnMovingObject && character.groundBodyId !== null) {
        const groundBody = rigidBody.get(world, character.groundBodyId);
        if (groundBody && groundBody.motionType === MotionType.DYNAMIC) {
            const movingObjectDragForce: Vec3 = [
                -moveImpulse[0] * Math.min(1, 1 / character.massRatio),
                0,
                -moveImpulse[2] * Math.min(1, 1 / character.massRatio),
            ];
            rigidBody.addImpulseAtPosition(world, groundBody, movingObjectDragForce, character.groundPosition);
        }
    }
}

function applyAutoBalanceTorque(world: World, character: FloatingCharacterState): void {
    if (!character.enableAutoBalance) {
        return;
    }

    // get body orientation
    const bodyRot = character.body.quaternion;

    // body's current up vector
    const bodyUp: Vec3 = [0, 1, 0];
    vec3.transformQuat(bodyUp, bodyUp, bodyRot);

    // body's current forward vector
    const bodyForward: Vec3 = [0, 0, 1];
    vec3.transformQuat(bodyForward, bodyForward, bodyRot);

    // desired up (world up)
    const desiredUp: Vec3 = [0, 1, 0];

    // desired forward (movement direction)
    if (vec3.length(character.input.moveDirection) > 0.001) {
        vec3.normalize(_applyAutoBalanceTorque_desiredForward, character.input.moveDirection);
    } else {
        vec3.copy(_applyAutoBalanceTorque_desiredForward, bodyForward);
    }

    // decompose balance vector into components
    const bodyBalanceOnX: Vec3 = [0, bodyUp[1], bodyUp[2]];
    const bodyBalanceOnZ: Vec3 = [bodyUp[0], bodyUp[1], 0];
    const bodyFacingOnY: Vec3 = [bodyForward[0], 0, bodyForward[2]];

    // compute cross products for torque direction
    vec3.cross(_applyAutoBalanceTorque_crossX, desiredUp, bodyBalanceOnX);
    vec3.cross(_applyAutoBalanceTorque_crossY, _applyAutoBalanceTorque_desiredForward, bodyFacingOnY);
    vec3.cross(_applyAutoBalanceTorque_crossZ, desiredUp, bodyBalanceOnZ);

    // get angular velocity
    const angVel = character.body.motionProperties.angularVelocity;

    // compute spring-damper torque on each axis
    const angleX = Math.acos(Math.max(-1, Math.min(1, vec3.dot(bodyBalanceOnX, desiredUp))));
    const angleY = Math.acos(Math.max(-1, Math.min(1, vec3.dot(bodyFacingOnY, _applyAutoBalanceTorque_desiredForward))));
    const angleZ = Math.acos(Math.max(-1, Math.min(1, vec3.dot(bodyBalanceOnZ, desiredUp))));

    const torque: Vec3 = [
        (_applyAutoBalanceTorque_crossX[0] < 0 ? 1 : -1) * character.balanceSpringK * angleX -
            angVel[0] * character.balanceDampingC,
        (_applyAutoBalanceTorque_crossY[1] < 0 ? 1 : -1) * character.balanceSpringOnY * angleY -
            angVel[1] * character.balanceDampingOnY,
        (_applyAutoBalanceTorque_crossZ[2] < 0 ? 1 : -1) * character.balanceSpringK * angleZ -
            angVel[2] * character.balanceDampingC,
    ];

    rigidBody.addAngularImpulse(world, character.body, torque);
}

function updateMovingPlatform(world: World, character: FloatingCharacterState, isMoving: boolean): void {
    // reset moving platform state if not on ground
    if (!character.canJump || character.groundBodyId === null) {
        character.massRatio = 1;
        character.isOnMovingObject = false;
        vec3.set(character.movingObjectVelocity, 0, 0, 0);
        vec3.set(character.bodyContactForce, 0, 0, 0);
        return;
    }

    const groundBody = rigidBody.get(world, character.groundBodyId);
    if (!groundBody) {
        character.massRatio = 1;
        character.isOnMovingObject = false;
        vec3.set(character.movingObjectVelocity, 0, 0, 0);
        return;
    }

    const groundBodyType = groundBody.motionType;

    // check if on dynamic or kinematic body
    if (groundBodyType === MotionType.DYNAMIC || groundBodyType === MotionType.KINEMATIC) {
        character.isOnMovingObject = true;

        // calculate mass ratio
        const groundMass = groundBody.massProperties.mass;
        const characterMass = character.body.massProperties.mass;
        character.massRatio = characterMass / groundMass;

        // calculate distance from character to object
        const currentPos = character.body.position;
        const groundPos = groundBody.position;
        _updateMovingPlatform_distanceFromCharacterToObject[0] = currentPos[0] - groundPos[0];
        _updateMovingPlatform_distanceFromCharacterToObject[1] = currentPos[1] - groundPos[1];
        _updateMovingPlatform_distanceFromCharacterToObject[2] = currentPos[2] - groundPos[2];

        // get platform velocities
        const groundLinvel = groundBody.motionProperties?.linearVelocity || vec3.create();
        const groundAngvel = groundBody.motionProperties?.angularVelocity || vec3.create();

        // combine linear and angular velocity
        vec3.cross(_updateMovingPlatform_objectAngvelToLinvel, groundAngvel, _updateMovingPlatform_distanceFromCharacterToObject);
        vec3.set(
            character.movingObjectVelocity,
            groundLinvel[0] + _updateMovingPlatform_objectAngvelToLinvel[0],
            groundLinvel[1],
            groundLinvel[2] + _updateMovingPlatform_objectAngvelToLinvel[2],
        );
        vec3.scale(character.movingObjectVelocity, character.movingObjectVelocity, Math.min(1, 1 / character.massRatio));

        // velocity difference clamping
        const currentVel = character.body.motionProperties.linearVelocity;
        _updateMovingPlatform_velocityDiff[0] = character.movingObjectVelocity[0] - currentVel[0];
        _updateMovingPlatform_velocityDiff[1] = character.movingObjectVelocity[1] - currentVel[1];
        _updateMovingPlatform_velocityDiff[2] = character.movingObjectVelocity[2] - currentVel[2];
        if (vec3.length(_updateMovingPlatform_velocityDiff) > 30) {
            vec3.scale(
                character.movingObjectVelocity,
                character.movingObjectVelocity,
                1 / vec3.length(_updateMovingPlatform_velocityDiff),
            );
        }

        // apply drag force to platform
        if (groundBodyType === MotionType.DYNAMIC) {
            if (!isMoving && vec3.squaredLength(character.input.moveDirection) < 0.001) {
                // when idle, use body contact force
                const movingObjectDragForce: Vec3 = [
                    -character.bodyContactForce[0] * Math.min(1, 1 / character.massRatio),
                    -character.bodyContactForce[1] * Math.min(1, 1 / character.massRatio),
                    -character.bodyContactForce[2] * Math.min(1, 1 / character.massRatio),
                ];
                rigidBody.addImpulseAtPosition(world, groundBody, movingObjectDragForce, character.groundPosition);
                vec3.set(character.bodyContactForce, 0, 0, 0);
            }
            // when moving, drag force is applied in applyMovementForce
        }
    } else {
        // on fixed body
        character.massRatio = 1;
        character.isOnMovingObject = false;
        vec3.set(character.movingObjectVelocity, 0, 0, 0);
        vec3.set(character.bodyContactForce, 0, 0, 0);
    }
}

function updateGravityScale(character: FloatingCharacterState): void {
    const velocity = character.body.motionProperties.linearVelocity;
    const verticalVel = velocity[1];

    // detect falling
    character.isFalling = verticalVel < 0 && !character.canJump;

    // adjust gravity scale
    if (verticalVel < -character.maxFallSpeed) {
        // terminal velocity - disable gravity
        character.body.motionProperties.gravityFactor = 0.0;
    } else if (character.isFalling) {
        // falling - higher gravity
        character.body.motionProperties.gravityFactor = character.fallingGravityScale;
    } else {
        // normal gravity
        character.body.motionProperties.gravityFactor = character.normalGravityScale;
    }
}

function handleJump(world: World, character: FloatingCharacterState): void {
    if (!character.input.wantToJump || !character.canJump) {
        return;
    }

    const currentVel = character.body.motionProperties.linearVelocity;
    const run = character.input.wantToRun;
    const jumpVel = run ? character.sprintJumpMult * character.jumpVelocity : character.jumpVelocity;

    // base jump velocity
    const jumpVelocityVec: Vec3 = [currentVel[0], jumpVel, currentVel[2]];

    // project jump direction along slope normal
    _handleJump_jumpDirection[0] = 0;
    _handleJump_jumpDirection[1] = jumpVel * character.slopeJumpMult;
    _handleJump_jumpDirection[2] = 0;

    // project onto slope normal
    const normalLength = vec3.length(character.actualSlopeNormal);
    if (normalLength > 0.0001) {
        const dot = vec3.dot(_handleJump_jumpDirection, character.actualSlopeNormal);
        vec3.scale(_handleJump_projection, character.actualSlopeNormal, dot / (normalLength * normalLength));
        vec3.copy(_handleJump_jumpDirection, _handleJump_projection);
    }

    // combine base velocity and slope-projected jump
    vec3.add(jumpVelocityVec, jumpVelocityVec, _handleJump_jumpDirection);

    rigidBody.setLinearVelocity(world, character.body, jumpVelocityVec);

    // apply jump force downward to standing platform
    if (character.groundBodyId !== null) {
        const groundBody = rigidBody.get(world, character.groundBodyId);
        if (groundBody && (groundBody.motionType === MotionType.DYNAMIC || groundBody.motionType === MotionType.KINEMATIC)) {
            const jumpForceToGround: Vec3 = [
                character.characterMassForce[0],
                character.characterMassForce[1] * character.jumpForceToGroundMult,
                character.characterMassForce[2],
            ];
            rigidBody.addImpulseAtPosition(world, groundBody, jumpForceToGround, character.groundPosition);
        }
    }
}

function updateCharacterRotation(character: FloatingCharacterState, moveDirection: Vec3, delta: number): void {
    if (vec3.length(moveDirection) > 0.001) {
        // calculate target rotation from movement direction (already in world space from camera-relative input)
        const targetY = Math.atan2(moveDirection[0], moveDirection[2]);

        // smoothly rotate towards target
        const deltaRot = targetY - character.characterModelIndicator.y;
        let normalizedDelta = deltaRot;

        // normalize to [-PI, PI]
        while (normalizedDelta > Math.PI) normalizedDelta -= 2 * Math.PI;
        while (normalizedDelta < -Math.PI) normalizedDelta += 2 * Math.PI;

        // apply turn speed
        const maxRotationStep = character.turnSpeed * delta;
        const rotationStep = Math.max(-maxRotationStep, Math.min(maxRotationStep, normalizedDelta));
        character.characterModelIndicator.y += rotationStep;

        // check if rotation complete
        character.characterRotated = Math.abs(normalizedDelta) < 0.01;
    } else {
        character.characterRotated = true;
    }
}

function applyDragForce(world: World, character: FloatingCharacterState): void {
    if (!character.canJump) {
        return;
    }

    const currentVel = character.body.motionProperties.linearVelocity;

    // different drag behavior on moving object vs static ground
    if (!character.isOnMovingObject) {
        const dragForce: Vec3 = [-currentVel[0] * character.dragDampingC, 0, -currentVel[2] * character.dragDampingC];
        rigidBody.addImpulse(world, character.body, dragForce);
    } else {
        // on moving object, drag toward object velocity
        const dragForce: Vec3 = [
            (character.movingObjectVelocity[0] - currentVel[0]) * character.dragDampingC,
            0,
            (character.movingObjectVelocity[2] - currentVel[2]) * character.dragDampingC,
        ];
        rigidBody.addImpulse(world, character.body, dragForce);
    }
}

// main update function
function updateFloatingCharacterController(
    world: World,
    character: FloatingCharacterState,
    input: {
        forward: boolean;
        backward: boolean;
        left: boolean;
        right: boolean;
        jump: boolean;
        run: boolean;
    },
    cameraDirection: Vec3,
    _cameraRight: Vec3, // unused with new direction calculation
    delta: number,
    filter: Filter,
): void {
    // clamp delta
    if (delta > 1) {
        delta = delta % 1;
    }

    // update input state
    character.input.wantToRun = input.run;
    character.input.wantToJump = input.jump;

    // camera direction is already computed from forward/backward/left/right input
    vec3.copy(_updateCharacterRotation_moveDir, cameraDirection);

    // project to horizontal plane and normalize
    _updateCharacterRotation_moveDir[1] = 0;
    if (vec3.length(_updateCharacterRotation_moveDir) > 0.001) {
        vec3.normalize(_updateCharacterRotation_moveDir, _updateCharacterRotation_moveDir);
    }
    character.input.moveDirection = _updateCharacterRotation_moveDir;

    const isMoving = vec3.length(_updateCharacterRotation_moveDir) > 0.001;

    // update character model indicator rotation (separate from physics body)
    updateCharacterRotation(character, _updateCharacterRotation_moveDir, delta);

    // ray casting for ground detection
    updateGroundDetection(world, character, filter);
    updateSlopeDetection(world, character, filter);

    // moving platform tracking (must be after ground/slope detection)
    updateMovingPlatform(world, character, isMoving);

    // apply movement force (if moving)
    if (isMoving) {
        applyMovementForce(world, character);
    } else {
        // apply drag when not moving
        applyDragForce(world, character);
    }

    // floating force (after ground detection)
    applyFloatingForce(world, character);

    // auto-balance torque (always, to keep upright)
    applyAutoBalanceTorque(world, character);

    // handle jump
    handleJump(world, character);

    // update gravity scaling
    updateGravityScale(character);
}

/* physics world */

registerAll();

const worldSettings = createWorldSettings();

worldSettings.gravity = [0, -9.81, 0];

const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);

const LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
const LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

enableCollision(worldSettings, LAYER_NON_MOVING, LAYER_MOVING);
enableCollision(worldSettings, LAYER_MOVING, LAYER_MOVING);

const world = createWorld(worldSettings);

/* scene */
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x333333);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 5, 10);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(10, 20, 10);
scene.add(directionalLight);

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugState.object3d);

/* input */

const keyboard = {
    forward: false,
    backward: false,
    left: false,
    right: false,
    jump: false,
    run: false,
};

window.addEventListener('keydown', (e) => {
    const key = e.key.toLowerCase();
    if (key === 'w') keyboard.forward = true;
    if (key === 's') keyboard.backward = true;
    if (key === 'a') keyboard.left = true;
    if (key === 'd') keyboard.right = true;
    if (key === ' ') keyboard.jump = true;
    if (e.key === 'Shift') keyboard.run = true;
});

window.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (key === 'w') keyboard.forward = false;
    if (key === 's') keyboard.backward = false;
    if (key === 'a') keyboard.left = false;
    if (key === 'd') keyboard.right = false;
    if (key === ' ') keyboard.jump = false;
    if (e.key === 'Shift') keyboard.run = false;
});

/* environment */

// floor
rigidBody.create(world, {
    shape: box.create({ halfExtents: [50, 0.5, 50] }),
    position: [0, -0.5, 0],
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// left wall
rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 2, 45] }),
    position: [-45, 1, 0],
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// right wall
rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.5, 2, 45] }),
    position: [45, 1, 0],
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// conveyor belt
rigidBody.create(world, {
    shape: box.create({ halfExtents: [10, 0.25, 2] }),
    position: [0, 0, -10],
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// stairs - 5 sets of varying heights
for (let j = 0; j < 5; j++) {
    const stepHeight = 0.3 + 0.1 * j;
    for (let i = 1; i < 10; i++) {
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [2, stepHeight / 2, 2] }),
            position: [15 + 5 * j, i * stepHeight - 0.5 + stepHeight / 2, -20 - i * 3],
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });
    }
}

// slopes - 10 boxes at varying angles (70° down to 25° in 5° increments)
for (let i = 0; i < 10; i++) {
    const angle = ((70 - i * 5.0) * Math.PI) / 180;
    const rotation = quat.setAxisAngle(quat.create(), [1, 0, 0], angle);
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [2.5, 0.6, 8] }),
        position: [-40 + 5 * i, 2, -25],
        quaternion: rotation,
        motionType: MotionType.STATIC,
        objectLayer: LAYER_NON_MOVING,
    });
}

// pushable block
rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.75, 0.75, 0.75] }),
    position: [-10, 5, 10],
    motionType: MotionType.DYNAMIC,
    objectLayer: LAYER_MOVING,
    friction: 0.1,
    mass: 1,
});

// pushable sphere
rigidBody.create(world, {
    shape: sphere.create({ radius: 0.75 }),
    position: [-8, 2, 10],
    motionType: MotionType.DYNAMIC,
    objectLayer: LAYER_MOVING,
    friction: 0.1,
    mass: 1,
});

// spinning platform - kinematic thin box that rotates
const spinningPlatform = rigidBody.create(world, {
    shape: box.create({ halfExtents: [4, 0.2, 4] }),
    position: [0, 2, 15],
    motionType: MotionType.KINEMATIC,
    objectLayer: LAYER_MOVING,
});

// sliding platform - kinematic box that moves side to side
const slidingPlatform = rigidBody.create(world, {
    shape: box.create({ halfExtents: [3, 0.2, 3] }),
    position: [20, 2, -5],
    motionType: MotionType.KINEMATIC,
    objectLayer: LAYER_MOVING,
});
const slidingPlatformCenter: Vec3 = [20, 2, -5];
const slidingPlatformAmplitude = 8;
const slidingPlatformSpeed = 0.15;

// diagonal platform - kinematic box that moves sideways and up/down
const diagonalPlatform = rigidBody.create(world, {
    shape: box.create({ halfExtents: [3, 0.2, 3] }),
    position: [20, 3, 5],
    motionType: MotionType.KINEMATIC,
    objectLayer: LAYER_MOVING,
});
const diagonalPlatformCenter: Vec3 = [20, 3, 5];
const diagonalPlatformAmplitudeX = 6;
const diagonalPlatformAmplitudeY = 2;
const diagonalPlatformSpeed = 0.12;

// triangle mesh terrain - floor section with hills and valleys
{
    const gridSize = 16;
    const cellSize = 2.0;
    const heightScale = 2;

    const positions: number[] = [];
    const heightAt = (x: number, z: number): number => {
        const freq1 = 0.3;
        const freq2 = 0.7;
        const freq3 = 0.15;
        return (
            Math.sin(x * freq1) * Math.cos(z * freq1) * heightScale * 0.5 +
            Math.sin(x * freq2 + 1.0) * Math.sin(z * freq2) * heightScale * 0.3 +
            Math.cos((x + z) * freq3) * heightScale * 0.2
        );
    };

    for (let iz = 0; iz <= gridSize; iz++) {
        for (let ix = 0; ix <= gridSize; ix++) {
            const x = (ix - gridSize / 2) * cellSize;
            const z = (iz - gridSize / 2) * cellSize;
            const y = heightAt(x, z);
            positions.push(x, y, z);
        }
    }

    const indices: number[] = [];
    for (let iz = 0; iz < gridSize; iz++) {
        for (let ix = 0; ix < gridSize; ix++) {
            const row = gridSize + 1;
            const bl = iz * row + ix;
            const br = bl + 1;
            const tl = bl + row;
            const tr = tl + 1;

            indices.push(bl, tl, br);
            indices.push(br, tl, tr);
        }
    }

    const terrainShape = triangleMesh.create({
        positions,
        indices,
    });

    rigidBody.create(world, {
        shape: terrainShape,
        position: [-30, 2, 30],
        motionType: MotionType.STATIC,
        objectLayer: LAYER_NON_MOVING,
    });
}

// bumpy floor - individual spheres of varying sizes
{
    const spheres: Array<{ x: number; z: number; radius: number }> = [
        { x: 25, z: 28, radius: 0.8 },
        { x: 29, z: 32, radius: 0.9 },
        { x: 23, z: 34, radius: 0.7 },
        { x: 31, z: 28, radius: 0.85 },
        { x: 27, z: 30, radius: 1.0 },
        { x: 22, z: 30, radius: 0.5 },
        { x: 32, z: 30, radius: 0.55 },
        { x: 25, z: 26, radius: 0.45 },
        { x: 29, z: 34, radius: 0.6 },
        { x: 21, z: 32, radius: 0.5 },
        { x: 33, z: 32, radius: 0.48 },
        { x: 27, z: 35, radius: 0.52 },
        { x: 27, z: 25, radius: 0.55 },
        { x: 24, z: 31, radius: 0.35 },
        { x: 30, z: 29, radius: 0.3 },
        { x: 26, z: 33, radius: 0.38 },
        { x: 28, z: 27, radius: 0.32 },
        { x: 22, z: 28, radius: 0.28 },
        { x: 32, z: 34, radius: 0.34 },
        { x: 20, z: 30, radius: 0.4 },
        { x: 34, z: 30, radius: 0.36 },
        { x: 23, z: 36, radius: 0.33 },
        { x: 31, z: 26, radius: 0.31 },
    ];

    for (const s of spheres) {
        rigidBody.create(world, {
            shape: sphere.create({ radius: s.radius }),
            position: [s.x, s.radius * -0.5, s.z],
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });
    }
}

// stepping boxes - static boxes of doubling sizes
{
    const baseSize = 0.25;
    const startX = 5;
    const startZ = 30;
    let currentX = startX;

    for (let i = 0; i < 5; i++) {
        const halfExtent = baseSize * 2 ** i;
        rigidBody.create(world, {
            shape: box.create({ halfExtents: [halfExtent, halfExtent, halfExtent] }),
            position: [currentX, halfExtent, startZ],
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });
        currentX += halfExtent * 3;
    }
}

// spring-suspended platform - dynamic platform held up by 4 spring distance constraints
{
    const platformCenter: Vec3 = [-10, 5, 35];
    const platformHalfExtents: Vec3 = [3, 0.3, 3];
    const anchorHeight = 12;
    const springLength = 4;

    const suspendedPlatform = rigidBody.create(world, {
        shape: box.create({ halfExtents: platformHalfExtents }),
        position: platformCenter,
        motionType: MotionType.DYNAMIC,
        objectLayer: LAYER_MOVING,
        linearDamping: 0.5,
        angularDamping: 0.5,
    });

    const anchorOffsets: Vec3[] = [
        [-platformHalfExtents[0], 0, -platformHalfExtents[2]],
        [platformHalfExtents[0], 0, -platformHalfExtents[2]],
        [-platformHalfExtents[0], 0, platformHalfExtents[2]],
        [platformHalfExtents[0], 0, platformHalfExtents[2]],
    ];

    for (const offset of anchorOffsets) {
        const anchorPosition: Vec3 = [platformCenter[0] + offset[0], anchorHeight, platformCenter[2] + offset[2]];

        const anchor = rigidBody.create(world, {
            shape: emptyShape.create(),
            position: anchorPosition,
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });

        const platformAttachPoint: Vec3 = [
            platformCenter[0] + offset[0],
            platformCenter[1] + platformHalfExtents[1],
            platformCenter[2] + offset[2],
        ];

        distanceConstraint.create(world, {
            bodyIdA: anchor.id,
            bodyIdB: suspendedPlatform.id,
            space: ConstraintSpace.WORLD,
            pointA: anchorPosition,
            pointB: platformAttachPoint,
            springSettings: {
                mode: SpringMode.FREQUENCY_AND_DAMPING,
                frequencyOrStiffness: 0.8,
                damping: 0.1,
            },
            minDistance: springLength * 0.5,
            maxDistance: springLength * 2,
        });
    }
}

/* character */

// initialize character
const characterOptions = createFloatingCharacterControllerOptions();

const character = initFloatingCharacterController(world, [0, 5, 0], LAYER_MOVING, characterOptions);

// create query filter for raycasts
const queryFilter = filter.create(world.settings.layers);

/* debug ui controls */

// movement folder
const movementFolder = ui.gui.addFolder('Movement');
movementFolder.add(character, 'maxWalkSpeed', 0, 10, 0.1).name('Walk Speed');
movementFolder.add(character, 'maxRunSpeed', 0, 20, 0.1).name('Run Speed');
movementFolder.add(character, 'accelerationTime', 1, 20, 0.5).name('Acceleration Time');
movementFolder.add(character, 'turnSpeed', 5, 50, 1).name('Turn Speed');
movementFolder.add(character, 'turnVelMultiplier', 0, 1, 0.05).name('Turn Penalty');
movementFolder.add(character, 'airControlFactor', 0, 1, 0.05).name('Air Control');

// floating force folder
const floatingFolder = ui.gui.addFolder('Floating');
floatingFolder.add(character, 'floatHeight', 0, 2, 0.05).name('Float Height');
floatingFolder.add(character, 'floatSpringK', 0, 5, 0.1).name('Spring K');
floatingFolder.add(character, 'floatDampingC', 0, 0.5, 0.01).name('Damping C');
floatingFolder.add(character, 'dragDampingC', 0, 1, 0.01).name('Drag Damping');

// slope folder
const slopeFolder = ui.gui.addFolder('Slope');
slopeFolder.add(character, 'maxSlopeAngle', 0, Math.PI / 2, 0.05).name('Max Slope Angle');
slopeFolder.add(character, 'slopeUpExtraForce', 0, 1, 0.05).name('Uphill Force');
slopeFolder.add(character, 'slopeDownExtraForce', 0, 1, 0.05).name('Downhill Force');
slopeFolder.add(character, 'rejectVelMult', 0, 10, 0.5).name('Reject Vel Mult');

// auto balance folder
const balanceFolder = ui.gui.addFolder('Auto Balance');
balanceFolder
    .add(character, 'enableAutoBalance')
    .name('Enable')
    .onChange((enabled: boolean) => {
        // update allowed degrees of freedom based on balance setting
        if (enabled) {
            character.body.motionProperties.allowedDegreesOfFreedom = 0b111111; // all
        } else {
            character.body.motionProperties.allowedDegreesOfFreedom = 0b111000; // translation only
            // reset angular velocity and rotation when disabling
            character.body.motionProperties.angularVelocity = [0, 0, 0];
            // reset to upright rotation (identity quaternion)
            character.body.quaternion = [0, 0, 0, 1];
        }
    });
balanceFolder.add(character, 'balanceSpringK', 0, 2, 0.05).name('Spring K');
balanceFolder.add(character, 'balanceDampingC', 0, 0.2, 0.005).name('Damping C');
balanceFolder.add(character, 'balanceSpringOnY', 0, 2, 0.05).name('Spring K Y');
balanceFolder.add(character, 'balanceDampingOnY', 0, 0.2, 0.005).name('Damping C Y');

// jump folder
const jumpFolder = ui.gui.addFolder('Jump');
jumpFolder.add(character, 'jumpVelocity', 0, 10, 0.5).name('Jump Velocity');
jumpFolder.add(character, 'jumpForceToGroundMult', 0, 20, 0.5).name('Force to Ground');
jumpFolder.add(character, 'slopeJumpMult', 0, 1, 0.05).name('Slope Jump Mult');
jumpFolder.add(character, 'sprintJumpMult', 1, 2, 0.1).name('Sprint Jump Mult');

// gravity folder
const gravityFolder = ui.gui.addFolder('Gravity');
gravityFolder
    .add(character, 'normalGravityScale', 0, 3, 0.1)
    .name('Normal Scale')
    .onChange((scale: number) => {
        if (!character.isFalling) {
            character.body.motionProperties.gravityFactor = scale;
        }
    });
gravityFolder
    .add(character, 'fallingGravityScale', 0, 5, 0.1)
    .name('Falling Scale')
    .onChange((scale: number) => {
        if (character.isFalling) {
            character.body.motionProperties.gravityFactor = scale;
        }
    });
gravityFolder.add(character, 'maxFallSpeed', 0, 50, 1).name('Max Fall Speed');

// create character mesh (external to physics)
const characterMesh = new THREE.Mesh(
    new THREE.CapsuleGeometry(character.capsuleRadius, character.capsuleHalfHeight * 2, 8, 16),
    new THREE.MeshStandardMaterial({ color: 0x4488ff }),
);
scene.add(characterMesh);

/* debug helpers */

// ground ray visualization
const groundRayHelper = new THREE.ArrowHelper(new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 0), 1, 0xff0000);
scene.add(groundRayHelper);

// slope ray visualization
const slopeRayHelper = new THREE.ArrowHelper(new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 0), 1, 0x00ff00);
scene.add(slopeRayHelper);

// ground normal visualization
const groundNormalHelper = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1, 0x0000ff);
scene.add(groundNormalHelper);

function updateDebugHelpers() {
    const bodyPos = character.body.position;

    // update ground ray
    groundRayHelper.position.set(bodyPos[0], bodyPos[1] - character.capsuleHalfHeight, bodyPos[2]);
    groundRayHelper.setLength(character.groundDistance > 0 ? character.groundDistance : 1);
    groundRayHelper.setColor(character.isOnGround ? 0x00ff00 : 0xff0000);

    // update slope ray
    const bodyRot = character.body.quaternion;
    const forward: Vec3 = [0, 0, 1];
    vec3.transformQuat(forward, forward, bodyRot);

    slopeRayHelper.position.set(
        bodyPos[0] + forward[0] * character.slopeRayOffset,
        bodyPos[1] - character.capsuleHalfHeight,
        bodyPos[2] + forward[2] * character.slopeRayOffset,
    );

    // update ground normal
    if (character.isOnGround) {
        groundNormalHelper.position.set(character.groundPosition[0], character.groundPosition[1], character.groundPosition[2]);
        groundNormalHelper.setDirection(
            new THREE.Vector3(character.actualSlopeNormal[0], character.actualSlopeNormal[1], character.actualSlopeNormal[2]),
        );
    }
}

/* animation loop */

const clock = new THREE.Clock();

function animate() {
    requestAnimationFrame(animate);

    const delta = Math.min(clock.getDelta(), 1 / 45); // cap at 45 FPS

    // get camera rotation and calculate movement direction
    const cameraRotation = new THREE.Quaternion();
    camera.getWorldQuaternion(cameraRotation);
    const forward = keyboard.forward ? 1.0 : keyboard.backward ? -1.0 : 0.0;
    const right = keyboard.right ? 1.0 : keyboard.left ? -1.0 : 0.0;
    const cameraDirection = new THREE.Vector3(right, 0, -forward).applyQuaternion(cameraRotation);
    cameraDirection.y = 0;
    cameraDirection.normalize();

    const cameraForwardVec: Vec3 = [cameraDirection.x, cameraDirection.y, cameraDirection.z];
    const cameraRightVec: Vec3 = [0, 0, 0]; // not used with new direction calculation

    debugUI.beginPerf(ui);

    // update character physics
    vec3.copy(_animate_oldPosition, character.body.position);
    updateFloatingCharacterController(world, character, keyboard, cameraForwardVec, cameraRightVec, delta, queryFilter);

    // update spinning platform - use moveKinematic to properly set angular velocity
    const spinSpeed = 1.0;
    quat.copy(_animate_targetRotation, spinningPlatform.quaternion);
    quat.setAxisAngle(_animate_deltaRotation, [0, 1, 0], spinSpeed * delta);
    quat.multiply(_animate_targetRotation, _animate_targetRotation, _animate_deltaRotation);
    rigidBody.moveKinematic(spinningPlatform, spinningPlatform.position, _animate_targetRotation, delta);

    // update sliding platform - use moveKinematic to properly set linear velocity
    const time = clock.getElapsedTime();
    const slideOffset = Math.sin(time * slidingPlatformSpeed * Math.PI * 2) * slidingPlatformAmplitude;
    const targetPosition: Vec3 = [slidingPlatformCenter[0] + slideOffset, slidingPlatformCenter[1], slidingPlatformCenter[2]];
    rigidBody.moveKinematic(slidingPlatform, targetPosition, slidingPlatform.quaternion, delta);

    // update diagonal platform - moves sideways and up/down
    const diagonalPhase = time * diagonalPlatformSpeed * Math.PI * 2;
    const diagonalOffsetX = Math.sin(diagonalPhase) * diagonalPlatformAmplitudeX;
    const diagonalOffsetY = Math.sin(diagonalPhase * 2) * diagonalPlatformAmplitudeY;
    const diagonalTargetPosition: Vec3 = [
        diagonalPlatformCenter[0] + diagonalOffsetX,
        diagonalPlatformCenter[1] + diagonalOffsetY,
        diagonalPlatformCenter[2],
    ];
    rigidBody.moveKinematic(diagonalPlatform, diagonalTargetPosition, diagonalPlatform.quaternion, delta);

    // update world physics
    updateWorld(world, undefined, delta);

    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    const newPosition = vec3.clone(character.body.position);

    // sync visual character mesh to physics body
    characterMesh.position.set(newPosition[0], newPosition[1], newPosition[2]);
    characterMesh.quaternion.set(
        character.body.quaternion[0],
        character.body.quaternion[1],
        character.body.quaternion[2],
        character.body.quaternion[3],
    );

    // move camera with character
    vec3.sub(_animate_deltaPos, newPosition, _animate_oldPosition);
    camera.position.add(new THREE.Vector3(_animate_deltaPos[0], _animate_deltaPos[1], _animate_deltaPos[2]));
    controls.target.set(newPosition[0], newPosition[1], newPosition[2]);

    // update debug helpers
    updateDebugHelpers();

    // update debug renderer
    debugRenderer.update(debugState, world);

    // render
    controls.update();
    renderer.render(scene, camera);
}

animate();

/* window resize handler */

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});
