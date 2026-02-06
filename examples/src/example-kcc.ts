import type { Vec3 } from 'mathcat';
import { quat, vec3, vec4 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    ConstraintSpace,
    createWorld,
    createWorldSettings,
    distanceConstraint,
    emptyShape,
    enableCollision,
    filter,
    type KCC,
    kcc,
    MotionType,
    rigidBody,
    type Shape,
    sphere,
    SpringMode,
    transformed,
    triangleMesh,
    updateWorld,
    registerAllShapes,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

enum CharacterShapeType {
    CAPSULE = 'CAPSULE',
    BOX = 'BOX',
}

/* physics world */

registerAllShapes();

const worldSettings = createWorldSettings();

worldSettings.gravity = vec3.fromValues(0, -25, 0);

const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);

const LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);
const LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);

enableCollision(worldSettings, LAYER_NON_MOVING, LAYER_MOVING);
enableCollision(worldSettings, LAYER_MOVING, LAYER_MOVING);


const world = createWorld(worldSettings);

/* character settings */
const characterFilter = filter.create(world.settings.layers);

const characterHeightStanding = 2;
const characterRadiusStanding = 1;
const characterHeightCrouching = 1;
const characterRadiusCrouching = 0.8;

const characterMass = 1000;

const controlMovementDuringJump = true;
const characterSpeed = 6.0;
const jumpSpeed = 15.0;
const enableCharacterInertia = true;

const maxSlopeAngle = (45.0 * Math.PI) / 180;
const maxStrength = 100.0;
const characterPadding = 0.02;
const penetrationRecoverySpeed = 1.0;
const predictiveContactDistance = 0.1;
const enableWalkStairs = true;
const enableStickToFloor = true;

/* scene */
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x333333);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 10, 20);

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

/* debugging */
const options = debugRenderer.createDefaultOptions();
const debugState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugState.object3d);

const kccDebugHelpers = {
    contactMarkers: [] as THREE.Mesh[],
    contactNormals: [] as THREE.ArrowHelper[],
    surfaceNormals: [] as THREE.ArrowHelper[],
    groundNormal: null as THREE.ArrowHelper | null,
    groundVelocity: null as THREE.ArrowHelper | null,
    groundPosition: null as THREE.Mesh | null,
    characterVelocity: null as THREE.ArrowHelper | null,
    supportingPlane: null as THREE.Mesh | null,
};

function clearKccDebugHelpers() {
    // Remove all contact markers
    for (const marker of kccDebugHelpers.contactMarkers) {
        scene.remove(marker);
        marker.geometry.dispose();
        (marker.material as THREE.Material).dispose();
    }
    kccDebugHelpers.contactMarkers = [];

    // Remove all arrows
    const allArrows = [...kccDebugHelpers.contactNormals, ...kccDebugHelpers.surfaceNormals];
    if (kccDebugHelpers.groundNormal) allArrows.push(kccDebugHelpers.groundNormal);
    if (kccDebugHelpers.groundVelocity) allArrows.push(kccDebugHelpers.groundVelocity);
    if (kccDebugHelpers.characterVelocity) allArrows.push(kccDebugHelpers.characterVelocity);

    // Remove ground position marker
    if (kccDebugHelpers.groundPosition) {
        scene.remove(kccDebugHelpers.groundPosition);
        kccDebugHelpers.groundPosition.geometry.dispose();
        (kccDebugHelpers.groundPosition.material as THREE.Material).dispose();
        kccDebugHelpers.groundPosition = null;
    }

    for (const arrow of allArrows) {
        scene.remove(arrow);
        arrow.dispose();
    }
    kccDebugHelpers.contactNormals = [];
    kccDebugHelpers.surfaceNormals = [];
    kccDebugHelpers.groundNormal = null;
    kccDebugHelpers.groundVelocity = null;
    kccDebugHelpers.characterVelocity = null;

    // Remove supporting plane
    if (kccDebugHelpers.supportingPlane) {
        scene.remove(kccDebugHelpers.supportingPlane);
        kccDebugHelpers.supportingPlane.geometry.dispose();
        (kccDebugHelpers.supportingPlane.material as THREE.Material).dispose();
        kccDebugHelpers.supportingPlane = null;
    }
}

/* Character State */
type CharacterState = {
    character: KCC;
    standingShape: Shape;
    crouchingShape: Shape;
    isCrouched: boolean;
    desiredVelocity: Vec3;
    allowSliding: boolean;
    isInLava: boolean;
};

const createCharacterShapes = (shapeType: CharacterShapeType) => {
    let standingShape: Shape;
    let crouchingShape: Shape;

    // offset positions to place shape center above character position (character position = bottom of shape)
    const positionStanding = vec3.fromValues(0, 0.5 * characterHeightStanding + characterRadiusStanding, 0);
    const positionCrouching = vec3.fromValues(0, 0.5 * characterHeightCrouching + characterRadiusCrouching, 0);
    const quaternion = quat.create();

    switch (shapeType) {
        case CharacterShapeType.CAPSULE: {
            standingShape = transformed.create({
                shape: capsule.create({
                    halfHeightOfCylinder: characterHeightStanding / 2,
                    radius: characterRadiusStanding,
                }),
                position: positionStanding,
                quaternion,
            });
            crouchingShape = transformed.create({
                shape: capsule.create({
                    halfHeightOfCylinder: characterHeightCrouching / 2,
                    radius: characterRadiusCrouching,
                }),
                position: positionCrouching,
                quaternion,
            });
            break;
        }

        case CharacterShapeType.BOX: {
            standingShape = transformed.create({
                shape: box.create({
                    halfExtents: vec3.fromValues(
                        characterRadiusStanding,
                        characterHeightStanding / 2 + characterRadiusStanding,
                        characterRadiusStanding,
                    ),
                }),
                position: positionStanding,
                quaternion,
            });
            crouchingShape = transformed.create({
                shape: box.create({
                    halfExtents: vec3.fromValues(
                        characterRadiusCrouching,
                        characterHeightCrouching / 2 + characterRadiusCrouching,
                        characterRadiusCrouching,
                    ),
                }),
                position: positionCrouching,
                quaternion,
            });
            break;
        }
    }

    return { standingShape, crouchingShape, positionStanding, positionCrouching };
};

const initCharacter = (
    shapeType: CharacterShapeType,
    lavaBody: ReturnType<typeof rigidBody.create>,
    conveyorBody: ReturnType<typeof rigidBody.create>,
) => {
    const { standingShape, crouchingShape } = createCharacterShapes(shapeType);

    const state: CharacterState = {
        character: null!,
        standingShape,
        crouchingShape,
        isCrouched: false,
        desiredVelocity: vec3.create(),
        allowSliding: false,
        isInLava: false,
    };

    // Character contact listener - handles special contacts (lava, conveyor belt)
    const characterContactListener: kcc.CharacterListener = {
        onAdjustBodyVelocity: (_character, body, linearVelocity, _angularVelocity) => {
            // apply conveyor belt velocity to the body
            // this is called before collision detection, making the body appear to be moving
            if (body.id === conveyorBody.id) {
                linearVelocity[0] += 5.0; // add 5 m/s in X direction
            }
        },
        onContactValidate: (_character, body, _subShapeId, _position, _normal) => {
            // check if touching lava
            if (body.id === lavaBody.id) {
                state.isInLava = true;
            }
            return true;
        },
        onContactSolve: (
            _character,
            _body,
            _subShapeId,
            _contactPosition,
            contactNormal,
            contactVelocity,
            _characterVelocity,
            newCharacterVelocity,
        ) => {
            // don't allow sliding down static not-too-steep surfaces when not moving
            if (!state.allowSliding) {
                const contactVelLenSq = vec3.squaredLength(contactVelocity);
                if (contactVelLenSq < 1e-6 && !kcc.isSlopeTooSteep(state.character, contactNormal)) {
                    // zero out the new character velocity
                    vec3.zero(newCharacterVelocity);
                }
            }
        },
    };

    state.character = kcc.create(
        {
            shape: standingShape,
            innerRigidBody: {
                shape: standingShape,
                objectLayer: LAYER_MOVING,
            },
            mass: characterMass,
            maxSlopeAngle: maxSlopeAngle,
            maxStrength: maxStrength,
            characterPadding: characterPadding,
            penetrationRecoverySpeed: penetrationRecoverySpeed,
            predictiveContactDistance: predictiveContactDistance,
            up: vec3.fromValues(0, 1, 0),
            // supporting volume plane: defined in character local space
            // for a transformed shape with translation [0, 2, 0]:
            //   - capsule center is at [0, 2, 0] in character local space
            //   - bottom hemisphere center is at [0, 1, 0]
            // plane [0, 1, 0, -1] passes through y=1 (bottom hemisphere center)
            supportingVolumePlane: vec4.fromValues(0, 1, 0, -characterRadiusStanding),
        },
        vec3.fromValues(0, 10, 0),
        quat.create(),
    );

    // Add character to world (creates inner body)
    kcc.add(world, state.character);

    return { state, listener: characterContactListener };
};

const disposeCharacter = (state: CharacterState) => {
    // remove character from world
    kcc.remove(world, state.character);
};

/* level */

// lava (kill zone)
const lavaBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(1000, 2, 1000) }),
    position: vec3.fromValues(0, -50, 0),
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// conveyor belt - created second like in original
const conveyorBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(10, 0.25, 2) }),
    position: vec3.fromValues(0, 0, -10),
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// floor - createFloor() equivalent
rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(50, 0.5, 50) }),
    position: vec3.fromValues(0, -0.5, 0),
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// left wall
rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(0.5, 2, 45) }),
    position: vec3.fromValues(-45, 1, 0),
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// right wall
rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(0.5, 2, 45) }),
    position: vec3.fromValues(45, 1, 0),
    motionType: MotionType.STATIC,
    objectLayer: LAYER_NON_MOVING,
});

// stairs - 5 sets of varying heights
for (let j = 0; j < 5; j++) {
    const stepHeight = 0.3 + 0.1 * j;
    for (let i = 1; i < 10; i++) {
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(2, stepHeight / 2, 2) }),
            position: vec3.fromValues(15 + 5 * j, i * stepHeight - 0.5 + stepHeight / 2, -20 - i * 3),
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });
    }
}

// slopes - 10 boxes at varying angles (70° down to 25° in 5° increments)
for (let i = 0; i < 10; i++) {
    const angle = ((70 - i * 5.0) * Math.PI) / 180;
    const rotation = quat.setAxisAngle(quat.create(), vec3.fromValues(1, 0, 0), angle);
    rigidBody.create(world, {
        shape: box.create({ halfExtents: vec3.fromValues(2.5, 0.6, 8) }),
        position: vec3.fromValues(-40 + 5 * i, 2, -25),
        quaternion: rotation,
        motionType: MotionType.STATIC,
        objectLayer: LAYER_NON_MOVING,
    });
}

// pushable block
rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(0.75, 0.75, 0.75) }),
    position: vec3.fromValues(-10, 5, 10),
    motionType: MotionType.DYNAMIC,
    objectLayer: LAYER_MOVING,
    friction: 0.1,
    mass: 1,
});

// pushable sphere
rigidBody.create(world, {
    shape: sphere.create({ radius: 0.75 }),
    position: vec3.fromValues(-8, 2, 10),
    motionType: MotionType.DYNAMIC,
    objectLayer: LAYER_MOVING,
    friction: 0.1,
    mass: 1,
});

// spinning platform - kinematic thin box that rotates
const spinningPlatform = rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(4, 0.2, 4) }),
    position: vec3.fromValues(0, 2, 15),
    motionType: MotionType.KINEMATIC,
    objectLayer: LAYER_MOVING,
});

// sliding platform - kinematic box that moves side to side
const slidingPlatform = rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(3, 0.2, 3) }),
    position: vec3.fromValues(20, 2, -5),
    motionType: MotionType.KINEMATIC,
    objectLayer: LAYER_MOVING,
});
const slidingPlatformCenter = vec3.fromValues(20, 2, -5);
const slidingPlatformAmplitude = 8; // how far it moves from center
const slidingPlatformSpeed = 0.15; // cycles per second (Hz) - slow and smooth

// diagonal platform - kinematic box that moves sideways and up/down
const diagonalPlatform = rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(3, 0.2, 3) }),
    position: vec3.fromValues(20, 3, 5),
    motionType: MotionType.KINEMATIC,
    objectLayer: LAYER_MOVING,
});
const diagonalPlatformCenter = vec3.fromValues(20, 3, 5);
const diagonalPlatformAmplitudeX = 6; // How far it moves sideways
const diagonalPlatformAmplitudeY = 2; // How far it moves up/down
const diagonalPlatformSpeed = 0.12; // Cycles per second (Hz) - slow and smooth

// triangle mesh terrain - floor section with hills and valleys
{
    const gridSize = 16; // 16x16 grid
    const cellSize = 2.0; // 2 meters per cell
    const heightScale = 2; // Maximum height variation

    // Generate heightfield positions (centered around origin in shape local space)
    const positions: number[] = [];
    const heightAt = (x: number, z: number): number => {
        // Create some interesting terrain with multiple frequencies
        const freq1 = 0.3;
        const freq2 = 0.7;
        const freq3 = 0.15;
        return (
            Math.sin(x * freq1) * Math.cos(z * freq1) * heightScale * 0.5 +
            Math.sin(x * freq2 + 1.0) * Math.sin(z * freq2) * heightScale * 0.3 +
            Math.cos((x + z) * freq3) * heightScale * 0.2
        );
    };

    // Generate vertices centered around (0, 0, 0) in local space
    for (let iz = 0; iz <= gridSize; iz++) {
        for (let ix = 0; ix <= gridSize; ix++) {
            const x = (ix - gridSize / 2) * cellSize;
            const z = (iz - gridSize / 2) * cellSize;
            const y = heightAt(x, z);
            positions.push(x, y, z);
        }
    }

    // Generate triangle indices (two triangles per cell)
    const indices: number[] = [];
    for (let iz = 0; iz < gridSize; iz++) {
        for (let ix = 0; ix < gridSize; ix++) {
            const row = gridSize + 1; // vertices per row
            const bl = iz * row + ix; // bottom-left
            const br = bl + 1; // bottom-right
            const tl = bl + row; // top-left
            const tr = tl + 1; // top-right

            // Two triangles per quad (CCW winding for upward-facing normals)
            indices.push(bl, tl, br);
            indices.push(br, tl, tr);
        }
    }

    // Create the triangle mesh shape and rigid body
    const terrainShape = triangleMesh.create({
        positions,
        indices,
    });

    // Position in the corner opposing the slopes (slopes are at negative Z)
    rigidBody.create(world, {
        shape: terrainShape,
        position: vec3.fromValues(-30, 2, 30),
        motionType: MotionType.STATIC,
        objectLayer: LAYER_NON_MOVING,
    });
}

// Bumpy floor - individual spheres of varying sizes creating an uneven surface
{
    // Hardcoded sphere positions and radii for bumpy surface
    // Positioned at positive X, away from triangle mesh terrain
    const spheres: Array<{ x: number; z: number; radius: number }> = [
        // Larger bumps
        { x: 25, z: 28, radius: 0.8 },
        { x: 29, z: 32, radius: 0.9 },
        { x: 23, z: 34, radius: 0.7 },
        { x: 31, z: 28, radius: 0.85 },
        { x: 27, z: 30, radius: 1.0 },
        // Medium bumps
        { x: 22, z: 30, radius: 0.5 },
        { x: 32, z: 30, radius: 0.55 },
        { x: 25, z: 26, radius: 0.45 },
        { x: 29, z: 34, radius: 0.6 },
        { x: 21, z: 32, radius: 0.5 },
        { x: 33, z: 32, radius: 0.48 },
        { x: 27, z: 35, radius: 0.52 },
        { x: 27, z: 25, radius: 0.55 },
        // Smaller bumps scattered around
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
            position: vec3.fromValues(s.x, s.radius * -0.5, s.z), // Deeply embedded, just tops peeking out
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });
    }
}

// stepping boxes - static boxes of doubling sizes for testing step-up behavior
{
    const baseSize = 0.25; // Starting half-extent
    const startX = 5;
    const startZ = 30;
    let currentX = startX;

    for (let i = 0; i < 5; i++) {
        const halfExtent = baseSize * 2 ** i; // 0.25, 0.5, 1.0, 2.0, 4.0
        rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(halfExtent, halfExtent, halfExtent) }),
            position: vec3.fromValues(currentX, halfExtent, startZ),
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });
        // Move to next position: current box edge + next box half-width
        // Next box half-extent is double, so: halfExtent + (halfExtent * 2) = halfExtent * 3
        currentX += halfExtent * 3;
    }
}

// spring-suspended platform - dynamic platform held up by 4 spring distance constraints
{
    const platformCenter = vec3.fromValues(-10, 5, 35);
    const platformHalfExtents = vec3.fromValues(3, 0.3, 3);
    const anchorHeight = 12; // Height of anchor points above ground
    const springLength = 4; // Rest length of springs

    // Create the dynamic platform
    const suspendedPlatform = rigidBody.create(world, {
        shape: box.create({ halfExtents: platformHalfExtents }),
        position: platformCenter,
        motionType: MotionType.DYNAMIC,
        objectLayer: LAYER_MOVING,
        linearDamping: 0.5,
        angularDamping: 0.5,
    });

    // Create 4 static anchor points using emptyShape bodies at the corners
    const anchorOffsets = [
        vec3.fromValues(-platformHalfExtents[0], 0, -platformHalfExtents[2]),
        vec3.fromValues(platformHalfExtents[0], 0, -platformHalfExtents[2]),
        vec3.fromValues(-platformHalfExtents[0], 0, platformHalfExtents[2]),
        vec3.fromValues(platformHalfExtents[0], 0, platformHalfExtents[2]),
    ];

    for (const offset of anchorOffsets) {
        const anchorPosition = vec3.fromValues(
            platformCenter[0] + offset[0],
            anchorHeight,
            platformCenter[2] + offset[2],
        );

        const anchor = rigidBody.create(world, {
            shape: emptyShape.create(),
            position: anchorPosition,
            motionType: MotionType.STATIC,
            objectLayer: LAYER_NON_MOVING,
        });

        const platformAttachPoint = vec3.fromValues(
            platformCenter[0] + offset[0],
            platformCenter[1] + platformHalfExtents[1],
            platformCenter[2] + offset[2],
        );

        // Create spring distance constraint
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

/* Character Initialization */

// update settings (for stair walking and stick to floor)
const updateSettings = kcc.createDefaultUpdateSettings();

let { state: characterState, listener: characterListener } = initCharacter(CharacterShapeType.CAPSULE, lavaBody, conveyorBody);

/* Input Handling */
const input = {
    forwardPressed: false,
    backwardPressed: false,
    leftPressed: false,
    rightPressed: false,
    jump: false,
    crouched: false,
};

document.addEventListener('keydown', (event) => {
    switch (event.code) {
        case 'KeyW':
            input.forwardPressed = true;
            break;
        case 'KeyS':
            input.backwardPressed = true;
            break;
        case 'KeyA':
            input.leftPressed = true;
            break;
        case 'KeyD':
            input.rightPressed = true;
            break;
        case 'Space':
            input.jump = true;
            event.preventDefault();
            break;
        case 'ShiftLeft':
        case 'ShiftRight':
            input.crouched = true;
            break;
    }
});

document.addEventListener('keyup', (event) => {
    switch (event.code) {
        case 'KeyW':
            input.forwardPressed = false;
            break;
        case 'KeyS':
            input.backwardPressed = false;
            break;
        case 'KeyA':
            input.leftPressed = false;
            break;
        case 'KeyD':
            input.rightPressed = false;
            break;
        case 'Space':
            input.jump = false;
            break;
        case 'ShiftLeft':
        case 'ShiftRight':
            input.crouched = false;
            break;
    }
});

/* Character Controller Logic */
const setCrouched = (state: CharacterState, crouched: boolean, forceUpdate = false) => {
    if (crouched !== state.isCrouched || forceUpdate) {
        const newShape = crouched ? state.crouchingShape : state.standingShape;
        const newRadius = crouched ? characterRadiusCrouching : characterRadiusStanding;

        // Try to change the shape - only succeeds if there's room
        if (kcc.setShape(world, state.character, newShape, characterFilter, characterListener, 0.1)) {
            // Accept the new shape
            state.isCrouched = crouched;

            // Update supporting volume plane to match new shape
            // Plane passes through bottom hemisphere center in character local space
            state.character.supportingVolumePlane[0] = 0;
            state.character.supportingVolumePlane[1] = 1;
            state.character.supportingVolumePlane[2] = 0;
            state.character.supportingVolumePlane[3] = -newRadius;
        }
    }
};

const handleCharacterInput = (state: CharacterState, deltaTime: number, movementDirection: Vec3) => {
    const playerControlsHorizontalVelocity = controlMovementDuringJump || kcc.isSupported(state.character);

    // Normalize and calculate movement length
    const movementLength = vec3.length(movementDirection);
    if (movementLength > 1e-6) {
        vec3.scale(movementDirection, movementDirection, 1 / movementLength);
    }

    if (playerControlsHorizontalVelocity) {
        // True if the player intended to move
        state.allowSliding = movementLength > 1e-6;

        // Smooth the player input
        if (enableCharacterInertia) {
            vec3.scale(state.desiredVelocity, state.desiredVelocity, 0.75);
            const inputVel = vec3.scale(vec3.create(), movementDirection, 0.25 * characterSpeed);
            vec3.add(state.desiredVelocity, state.desiredVelocity, inputVel);
        } else {
            vec3.scale(state.desiredVelocity, movementDirection, characterSpeed);
        }
    } else {
        // While in air we allow sliding
        state.allowSliding = true;
    }

    // Update ground velocity (for moving platforms)
    kcc.updateGroundVelocity(world, state.character, characterListener);

    // Get character orientation
    const characterUp = vec3.clone(state.character.up);
    const linearVelocity = vec3.clone(state.character.linearVelocity);
    const currentVerticalVelocity = vec3.scale(vec3.create(), characterUp, vec3.dot(linearVelocity, characterUp));
    const groundVelocity = vec3.clone(state.character.ground.velocity);
    const gravity = vec3.clone(worldSettings.gravity);

    const newVelocity = vec3.create();
    // Check if moving towards ground: (verticalVel - groundVel) dot up < 0.1
    // This means: not moving away from ground faster than 0.1 m/s
    const verticalRelativeVel = vec3.dot(vec3.sub(vec3.create(), currentVerticalVelocity, groundVelocity), characterUp);
    const movingTowardsGround = verticalRelativeVel < 0.1;

    if (state.character.ground.state === kcc.GroundState.ON_GROUND) {
        // Check if we should stick to ground
        const shouldStickToGround = enableCharacterInertia
            ? movingTowardsGround
            : !kcc.isSlopeTooSteep(state.character, state.character.ground.normal);

        if (shouldStickToGround) {
            // Assume velocity of ground when on ground
            vec3.copy(newVelocity, groundVelocity);

            // Jump
            if (input.jump && movingTowardsGround) {
                vec3.scaleAndAdd(newVelocity, newVelocity, characterUp, jumpSpeed);
            }
        } else {
            vec3.copy(newVelocity, currentVerticalVelocity);
        }
    } else {
        vec3.copy(newVelocity, currentVerticalVelocity);
    }

    // Apply gravity
    vec3.scaleAndAdd(newVelocity, newVelocity, gravity, deltaTime);

    // Apply player input or preserve horizontal velocity
    if (playerControlsHorizontalVelocity) {
        vec3.add(newVelocity, newVelocity, state.desiredVelocity);
    } else {
        const currentHorizontalVelocity = vec3.sub(vec3.create(), linearVelocity, currentVerticalVelocity);
        vec3.add(newVelocity, newVelocity, currentHorizontalVelocity);
    }

    // Set the new velocity
    vec3.copy(state.character.linearVelocity, newVelocity);
};

/* GUI */
const settings = {
    shapeType: CharacterShapeType.CAPSULE,
    showDebug: true,
    enableWalkStairs: enableWalkStairs,
    enableStickToFloor: enableStickToFloor,
};

const kccDebugSettings = {
    showContacts: false,
    showGroundNormal: false,
    showGroundVelocity: false,
    showGroundPosition: false,
    showVelocity: false,
    showSupportingPlane: false,
};

ui.gui
    .add(settings, 'shapeType', [CharacterShapeType.CAPSULE, CharacterShapeType.BOX])
    .onChange((value: CharacterShapeType) => {
        // Dispose old character
        disposeCharacter(characterState);

        // Create new character with new shape type
        const result = initCharacter(value, lavaBody, conveyorBody);
        characterState = result.state;
        characterListener = result.listener;
    });

ui.gui.add(settings, 'showDebug').name('Show Debug');
ui.gui.add(settings, 'enableWalkStairs').name('Walk Stairs');
ui.gui.add(settings, 'enableStickToFloor').name('Stick to Floor');
ui.gui.add(characterState.character, 'enhancedInternalEdgeRemoval').name('Enhanced Internal Edge Removal');

// Ground State display (read-only)
const groundStateDisplay = { state: 'IN_AIR' };
ui.gui.add(groundStateDisplay, 'state').name('Ground State').listen().disable();

const kccDebugFolder = ui.gui.addFolder('KCC Debug');
kccDebugFolder.add(kccDebugSettings, 'showContacts').name('Show Contacts');
kccDebugFolder.add(kccDebugSettings, 'showGroundNormal').name('Show Ground Normal');
kccDebugFolder.add(kccDebugSettings, 'showGroundVelocity').name('Show Ground Velocity');
kccDebugFolder.add(kccDebugSettings, 'showGroundPosition').name('Show Ground Position');
kccDebugFolder.add(kccDebugSettings, 'showVelocity').name('Show Velocity');
kccDebugFolder.add(kccDebugSettings, 'showSupportingPlane').name('Show Supporting Plane');

/* KCC Debug Visualization Update */
function updateKccDebugVisualization(characterState: CharacterState) {
    clearKccDebugHelpers();

    if (!settings.showDebug) {
        return;
    }

    const character = characterState.character;

    // Show contacts
    if (kccDebugSettings.showContacts) {
        const activeContacts = kcc.getActiveContacts(character.contacts);

        for (const contact of activeContacts) {
            // Contact marker (yellow sphere)
            const markerGeometry = new THREE.SphereGeometry(0.1, 8, 8);
            const markerMaterial = new THREE.MeshBasicMaterial({
                color: 0xffff00,
                depthTest: false,
                depthWrite: false,
            });
            const marker = new THREE.Mesh(markerGeometry, markerMaterial);
            marker.position.set(contact.position[0], contact.position[1], contact.position[2]);
            marker.renderOrder = 999;
            scene.add(marker);
            kccDebugHelpers.contactMarkers.push(marker);

            // Contact normal (cyan arrow)
            if (contact.contactNormal) {
                const origin = new THREE.Vector3(contact.position[0], contact.position[1], contact.position[2]);
                const direction = new THREE.Vector3(contact.contactNormal[0], contact.contactNormal[1], contact.contactNormal[2]);
                const length = 1.0;
                const arrow = new THREE.ArrowHelper(direction, origin, length, 0x00ffff);
                // Make arrow render on top
                (arrow.line.material as THREE.Material).depthTest = false;
                (arrow.line.material as THREE.Material).depthWrite = false;
                arrow.line.renderOrder = 999;
                (arrow.cone.material as THREE.Material).depthTest = false;
                (arrow.cone.material as THREE.Material).depthWrite = false;
                arrow.cone.renderOrder = 999;
                scene.add(arrow);
                kccDebugHelpers.contactNormals.push(arrow);
            }

            // Surface normal (red arrow) if different from contact normal
            if (contact.surfaceNormal && contact.contactNormal) {
                const dot = vec3.dot(contact.contactNormal, contact.surfaceNormal);
                if (Math.abs(dot - 1.0) > 0.01) {
                    const origin = new THREE.Vector3(contact.position[0], contact.position[1], contact.position[2]);
                    const direction = new THREE.Vector3(
                        contact.surfaceNormal[0],
                        contact.surfaceNormal[1],
                        contact.surfaceNormal[2],
                    );
                    const length = 1.0;
                    const arrow = new THREE.ArrowHelper(direction, origin, length, 0xff0000);
                    // Make arrow render on top
                    (arrow.line.material as THREE.Material).depthTest = false;
                    (arrow.line.material as THREE.Material).depthWrite = false;
                    arrow.line.renderOrder = 999;
                    (arrow.cone.material as THREE.Material).depthTest = false;
                    (arrow.cone.material as THREE.Material).depthWrite = false;
                    arrow.cone.renderOrder = 999;
                    scene.add(arrow);
                    kccDebugHelpers.surfaceNormals.push(arrow);
                }
            }
        }
    }

    // Show ground normal
    if (kccDebugSettings.showGroundNormal && character.ground.state !== kcc.GroundState.IN_AIR) {
        const origin = new THREE.Vector3(character.position[0], character.position[1], character.position[2]);
        const direction = new THREE.Vector3(character.ground.normal[0], character.ground.normal[1], character.ground.normal[2]);
        const length = 2.0;
        const arrow = new THREE.ArrowHelper(direction, origin, length, 0x00ff00);
        // Make arrow render on top
        (arrow.line.material as THREE.Material).depthTest = false;
        (arrow.line.material as THREE.Material).depthWrite = false;
        arrow.line.renderOrder = 999;
        (arrow.cone.material as THREE.Material).depthTest = false;
        (arrow.cone.material as THREE.Material).depthWrite = false;
        arrow.cone.renderOrder = 999;
        scene.add(arrow);
        kccDebugHelpers.groundNormal = arrow;
    }

    // Show ground velocity
    if (kccDebugSettings.showGroundVelocity && character.ground.state !== kcc.GroundState.IN_AIR) {
        const groundVelocityLen = vec3.length(character.ground.velocity);
        if (groundVelocityLen > 0.01) {
            const origin = new THREE.Vector3(character.position[0], character.position[1], character.position[2]);
            const velDirection = new THREE.Vector3(
                character.ground.velocity[0] / groundVelocityLen,
                character.ground.velocity[1] / groundVelocityLen,
                character.ground.velocity[2] / groundVelocityLen,
            );
            const velArrow = new THREE.ArrowHelper(velDirection, origin, groundVelocityLen, 0x0000ff);
            // Make arrow render on top
            (velArrow.line.material as THREE.Material).depthTest = false;
            (velArrow.line.material as THREE.Material).depthWrite = false;
            velArrow.line.renderOrder = 999;
            (velArrow.cone.material as THREE.Material).depthTest = false;
            (velArrow.cone.material as THREE.Material).depthWrite = false;
            velArrow.cone.renderOrder = 999;
            scene.add(velArrow);
            kccDebugHelpers.groundVelocity = velArrow;
        }
    }

    // Show ground position
    if (kccDebugSettings.showGroundPosition && character.ground.state !== kcc.GroundState.IN_AIR) {
        const markerGeometry = new THREE.SphereGeometry(0.15, 12, 12);
        const markerMaterial = new THREE.MeshBasicMaterial({
            color: 0xff8800,
            depthTest: false,
            depthWrite: false,
        });
        const marker = new THREE.Mesh(markerGeometry, markerMaterial);
        marker.position.set(character.ground.position[0], character.ground.position[1], character.ground.position[2]);
        marker.renderOrder = 999;
        scene.add(marker);
        kccDebugHelpers.groundPosition = marker;
    }

    // Show character velocity
    if (kccDebugSettings.showVelocity) {
        const velocityLen = vec3.length(character.linearVelocity);
        if (velocityLen > 0.01) {
            const origin = new THREE.Vector3(character.position[0], character.position[1], character.position[2]);
            const direction = new THREE.Vector3(
                character.linearVelocity[0] / velocityLen,
                character.linearVelocity[1] / velocityLen,
                character.linearVelocity[2] / velocityLen,
            );
            const arrow = new THREE.ArrowHelper(direction, origin, velocityLen, 0xff00ff);
            // Make arrow render on top
            (arrow.line.material as THREE.Material).depthTest = false;
            (arrow.line.material as THREE.Material).depthWrite = false;
            arrow.line.renderOrder = 999;
            (arrow.cone.material as THREE.Material).depthTest = false;
            (arrow.cone.material as THREE.Material).depthWrite = false;
            arrow.cone.renderOrder = 999;
            scene.add(arrow);
            kccDebugHelpers.characterVelocity = arrow;
        }
    }

    // Show supporting volume plane
    if (kccDebugSettings.showSupportingPlane && character.supportingVolumePlane) {
        const planeNormal = vec3.fromValues(
            character.supportingVolumePlane[0],
            character.supportingVolumePlane[1],
            character.supportingVolumePlane[2],
        );
        const planeDistance = character.supportingVolumePlane[3];

        const planePointLocal = vec3.create();
        vec3.scale(planePointLocal, planeNormal, -planeDistance);

        const planePointRotated = vec3.create();
        vec3.transformQuat(planePointRotated, planePointLocal, character.quaternion);

        const planeCenter = vec3.create();
        vec3.add(planeCenter, character.position, planePointRotated);

        const planeNormalWorld = vec3.create();
        vec3.transformQuat(planeNormalWorld, planeNormal, character.quaternion);

        const planeGeometry = new THREE.PlaneGeometry(4, 4);
        const planeMaterial = new THREE.MeshBasicMaterial({
            color: 0xffaa00,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.3,
        });
        const planeMesh = new THREE.Mesh(planeGeometry, planeMaterial);

        // Orient the plane to match the world-space normal
        const quaternion = new THREE.Quaternion();
        const upVector = new THREE.Vector3(0, 0, 1);
        const normalVector = new THREE.Vector3(planeNormalWorld[0], planeNormalWorld[1], planeNormalWorld[2]);
        quaternion.setFromUnitVectors(upVector, normalVector);
        planeMesh.quaternion.copy(quaternion);

        planeMesh.position.set(planeCenter[0], planeCenter[1], planeCenter[2]);
        scene.add(planeMesh);
        kccDebugHelpers.supportingPlane = planeMesh;
    }
}

/* Animation Loop */
const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const deltaTime = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // Handle lava teleport
    if (characterState.isInLava) {
        kcc.setPosition(world, characterState.character, vec3.fromValues(0, 10, 0));
        vec3.zero(characterState.character.linearVelocity);
        characterState.isInLava = false;
    }

    // Update extended update settings based on GUI
    // Scale by character up direction (not hardcoded Y-axis) to support arbitrary gravity
    const stepDownLength = 0.5;
    const stepUpLength = 0.4;

    if (settings.enableStickToFloor) {
        vec3.scale(updateSettings.stickToFloorStepDown, characterState.character.up, -stepDownLength);
    } else {
        vec3.zero(updateSettings.stickToFloorStepDown);
    }

    if (settings.enableWalkStairs) {
        vec3.scale(updateSettings.walkStairsStepUp, characterState.character.up, stepUpLength);
    } else {
        vec3.zero(updateSettings.walkStairsStepUp);
    }

    // Handle input and update character stance
    setCrouched(characterState, input.crouched);

    // Get camera rotation and calculate movement direction
    const cameraRotation = new THREE.Quaternion();
    camera.getWorldQuaternion(cameraRotation);
    const forward = input.forwardPressed ? 1.0 : input.backwardPressed ? -1.0 : 0.0;
    const right = input.rightPressed ? 1.0 : input.leftPressed ? -1.0 : 0.0;
    const cameraDirection = new THREE.Vector3(right, 0, -forward).applyQuaternion(cameraRotation);
    cameraDirection.y = 0;
    cameraDirection.normalize();

    handleCharacterInput(characterState, deltaTime, [cameraDirection.x, cameraDirection.y, cameraDirection.z]);

    // Update spinning platform - use moveKinematic to properly set angular velocity
    // This is required for the character to follow the platform's rotation
    const spinSpeed = 1.0; // radians per second
    const targetRotation = quat.clone(spinningPlatform.quaternion);
    const deltaRotation = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), spinSpeed * deltaTime);
    quat.multiply(targetRotation, targetRotation, deltaRotation);
    rigidBody.moveKinematic(spinningPlatform, spinningPlatform.position, targetRotation, deltaTime);

    // Update sliding platform - use moveKinematic to properly set linear velocity
    // This is required for the character to follow the platform's movement
    const time = currentTime / 1000;
    const slideOffset = Math.sin(time * slidingPlatformSpeed * Math.PI * 2) * slidingPlatformAmplitude;
    const targetPosition = vec3.fromValues(
        slidingPlatformCenter[0] + slideOffset,
        slidingPlatformCenter[1],
        slidingPlatformCenter[2],
    );
    rigidBody.moveKinematic(slidingPlatform, targetPosition, slidingPlatform.quaternion, deltaTime);

    // Update diagonal platform - moves sideways and up/down in a figure-8 like pattern
    const diagonalPhase = time * diagonalPlatformSpeed * Math.PI * 2;
    const diagonalOffsetX = Math.sin(diagonalPhase) * diagonalPlatformAmplitudeX;
    const diagonalOffsetY = Math.sin(diagonalPhase * 2) * diagonalPlatformAmplitudeY; // 2x frequency for up/down
    const diagonalTargetPosition = vec3.fromValues(
        diagonalPlatformCenter[0] + diagonalOffsetX,
        diagonalPlatformCenter[1] + diagonalOffsetY,
        diagonalPlatformCenter[2],
    );
    rigidBody.moveKinematic(diagonalPlatform, diagonalTargetPosition, diagonalPlatform.quaternion, deltaTime);

    debugUI.beginPerf(ui);

    // Update character (with extended features)
    const oldPosition = vec3.clone(characterState.character.position);
    kcc.update(
        world,
        characterState.character,
        deltaTime,
        worldSettings.gravity,
        updateSettings,
        characterListener,
        characterFilter,
    );
    const newPosition = vec3.clone(characterState.character.position);

    // Update physics
    updateWorld(world, undefined, deltaTime);

    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Update ground state display
    const groundState = characterState.character.ground.state;
    groundStateDisplay.state =
        groundState === 0 ? 'ON_GROUND' : groundState === 1 ? 'ON_STEEP_GROUND' : groundState === 2 ? 'NOT_SUPPORTED' : 'IN_AIR';

    // Update KCC debug visualization
    updateKccDebugVisualization(characterState);

    // Move camera with character
    const deltaPos = vec3.sub(vec3.create(), newPosition, oldPosition);
    camera.position.add(new THREE.Vector3(deltaPos[0], deltaPos[1], deltaPos[2]));
    controls.target.set(newPosition[0], newPosition[1], newPosition[2]);

    // Update debug renderer
    if (settings.showDebug) {
        debugRenderer.update(debugState, world);
    } else {
        debugRenderer.clear(debugState);
    }

    controls.update();
    renderer.render(scene, camera);
}

animate();

/* Window Resize */
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});
