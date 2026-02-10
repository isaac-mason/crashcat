import type { Vec3 } from 'mathcat';
import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { HingeConstraint, Listener, RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    ConstraintSpace,
    createWorld,
    createWorldSettings,
    enableCollision,
    hingeConstraint,
    motorSettings,
    MotionType,
    MotorState,
    registerAll,
    rigidBody,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(-40, 10, 20);

const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
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

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;
orbitControls.target.set(0, 0, 0);

const hemisphereLight = new THREE.HemisphereLight(0xffffff, 0xf6d186, 0.35 * Math.PI);
scene.add(hemisphereLight);

const spotLight = new THREE.SpotLight(0xffffff, Math.PI);
spotLight.angle = Math.PI / 5;
spotLight.penumbra = 1;
spotLight.position.set(20, 30, 10);
spotLight.castShadow = true;
spotLight.shadow.mapSize.width = 256;
spotLight.shadow.mapSize.height = 256;
scene.add(spotLight);

/* physics world */

registerAll();

const worldSettings = createWorldSettings();
worldSettings.gravity = vec3.fromValues(0, -40, 0);

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

// collision groups
const GROUP_GROUND = 1 << 0;
const GROUP_BODY = 1 << 1;

/** filter out collisions between constraint-connected bodies */
const constraintFilterListener: Listener = {
    onBodyPairValidate: (bodyA: RigidBody, bodyB: RigidBody): boolean => {
        return !rigidBody.bodiesShareConstraint(bodyA, bodyB);
    },
};

/* ground plane */

rigidBody.create(world, {
    shape: box.create({ halfExtents: [5000, 0.5, 5000] }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -5.5, 0),
    restitution: 0,
    friction: 0.5,
    collisionGroup: GROUP_GROUND,
    collisionMask: GROUP_BODY | GROUP_GROUND,
});

/* walking robot - Theo Jansen linkage mechanism
 *
 * Ported from the use-cannon HingeMotor demo.
 *
 * The original demo uses two leg sets offset in Z with a lock constraint.
 * Here we use a symmetrical design: one central chassis body with a leg set
 * on each side (positive and negative Z). Each leg set is a planar Theo Jansen
 * linkage driven by a hinge motor on the crank.
 *
 * All hinge axes are along Z. All constraint pivots use LOCAL space.
 */

// Dimensions (matching cannon demo)
const partDepth = 0.3;
const bodyWidth = 10;
const bodyHeight = 2;
const legLength = 6;
const chassisDepth = 6; // Z thickness of the central body

let motorSpeed = 7;

// collect crank hinge handles so we can update their target velocity when
// `motorSpeed` changes (the gui only updates the variable currently)
const crankHinges: HingeConstraint[] = [];

function updateMotorTargets() {
    for (const h of crankHinges) {
        hingeConstraint.setTargetAngularVelocity(h, -motorSpeed);
    }
}
// The body is rotated PI/2 around Z so that:
//   local X (bodyHeight=2) -> world Y (height)
//   local Y (bodyWidth=10) -> world -X (length)
//   local Z (chassisDepth) -> world Z (width)
const bodyRotation = (() => {
    const q = quat.create();
    quat.setAxisAngle(q, vec3.fromValues(0, 0, 1), Math.PI / 2);
    return q;
})();

// Create the central chassis body
const chassisBody = rigidBody.create(world, {
    shape: box.create({ halfExtents: [bodyHeight / 2, bodyWidth / 2, chassisDepth / 2], convexRadius: 0.05, density: 1 }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(0, 0, 0),
    quaternion: bodyRotation,
    linearDamping: 0.4,
    friction: 0.5,
    collisionGroup: GROUP_BODY,
    collisionMask: GROUP_GROUND,
    allowSleeping: false,
});

/**
 * Create one set of Theo Jansen legs on one side of the chassis.
 *
 * @param side +1 for right side (positive Z), -1 for left side (negative Z)
 * @param crankPhase initial rotation of the crank (radians) for gait phasing
 */
function createLegSet(side: number, crankPhase: number): void {
    // Z position for this side's parts (just outside the chassis face)
    const legZ = side * (chassisDepth / 2 + partDepth / 2);

    // Body pivot Z: the face of the chassis on this side (in body local space)
    const bodyPivotZ = side * (chassisDepth / 2);

    // Part full extents (matching cannon demo args)
    const upperFrontLegFull: Vec3 = [1, 3, partDepth];
    const crankFull: Vec3 = [0.5, 1, partDepth];
    const frontLegFull: Vec3 = [1, legLength, partDepth];
    const horizontalBarFull: Vec3 = [1, bodyWidth, partDepth];
    const backLegFull: Vec3 = [1, legLength, partDepth];

    // Helper: initial rotation around Z
    function quatZ(angle: number) {
        const q = quat.create();
        quat.setAxisAngle(q, vec3.fromValues(0, 0, 1), angle);
        return q;
    }

    // --- Upper front leg ---
    const upperFrontLeg = rigidBody.create(world, {
        shape: box.create({ halfExtents: [upperFrontLegFull[0] / 2, upperFrontLegFull[1] / 2, upperFrontLegFull[2] / 2], convexRadius: 0.02 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(-2, 0.5, legZ),
        quaternion: quatZ(Math.PI / 3),
        linearDamping: 0.4,
        friction: 0.5,
        collisionGroup: GROUP_BODY,
        collisionMask: GROUP_GROUND,
        allowSleeping: false,
    });

    // Constraint 1: upperFrontLeg → chassis
    // cannon: pivotA = normUpperFrontLeg([0, -0.5, -0.5]), pivotB = normBody([0, 0.2, 0.5])
    hingeConstraint.create(world, {
        bodyIdA: upperFrontLeg.id,
        bodyIdB: chassisBody.id,
        pointA: [0, -upperFrontLegFull[1] / 2, 0],
        pointB: [0, bodyWidth * 0.2, bodyPivotZ],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.LOCAL,
    });

    // --- Crank (motor-driven) ---
    const crank = rigidBody.create(world, {
        shape: box.create({ halfExtents: [crankFull[0] / 2, crankFull[1] / 2, crankFull[2] / 2], convexRadius: 0.02 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(bodyWidth * -0.5, -1.5 / 2, legZ),
        quaternion: quatZ(crankPhase),
        linearDamping: 0.4,
        friction: 0.5,
        collisionGroup: GROUP_BODY,
        collisionMask: GROUP_GROUND,
        allowSleeping: false,
    });

    // Constraint 2: crank → chassis (MOTOR)
    // cannon: pivotA = normCrank([0, 0.5, -0.5]), pivotB = normBody([0, 0.5, 0.5])
    const crankHinge = hingeConstraint.create(world, {
        bodyIdA: crank.id,
        bodyIdB: chassisBody.id,
        pointA: [0, crankFull[1] / 2, 0],
        pointB: [0, bodyWidth * 0.5, bodyPivotZ],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.LOCAL,
    });

    motorSettings.setTorqueLimit(crankHinge.motorSettings, 1e6);
    hingeConstraint.setMotorState(crankHinge, MotorState.VELOCITY);
    crankHinges.push(crankHinge);
    updateMotorTargets();

    // --- Front leg ---
    const frontLeg = rigidBody.create(world, {
        shape: box.create({ halfExtents: [frontLegFull[0] / 2, frontLegFull[1] / 2, frontLegFull[2] / 2], convexRadius: 0.02 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(bodyWidth * -0.5, -1, legZ),
        quaternion: quatZ(Math.PI / -6),
        linearDamping: 0.4,
        friction: 0.5,
        collisionGroup: GROUP_BODY,
        collisionMask: GROUP_GROUND,
        allowSleeping: false,
    });

    // Constraint 3: frontLeg → crank
    // Both parts are at the same Z (legZ), so Z pivot = 0 on both sides
    hingeConstraint.create(world, {
        bodyIdA: frontLeg.id,
        bodyIdB: crank.id,
        pointA: [0, 0, 0],
        pointB: [0, -crankFull[1] / 2, 0],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.LOCAL,
    });

    // --- Horizontal bar ---
    const horizontalBar = rigidBody.create(world, {
        shape: box.create({ halfExtents: [horizontalBarFull[0] / 2, horizontalBarFull[1] / 2, horizontalBarFull[2] / 2], convexRadius: 0.02 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(0, 0, legZ),
        quaternion: quatZ(Math.PI / -2.5),
        linearDamping: 0.4,
        friction: 0.5,
        collisionGroup: GROUP_BODY,
        collisionMask: GROUP_GROUND,
        allowSleeping: false,
    });

    // Constraint 4: horizontalBar → frontLeg
    // Both parts at same Z, so Z pivot = 0
    hingeConstraint.create(world, {
        bodyIdA: horizontalBar.id,
        bodyIdB: frontLeg.id,
        pointA: [0, -horizontalBarFull[1] / 2, 0],
        pointB: [0, 0, 0],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.LOCAL,
    });

    // --- Back leg ---
    const backLeg = rigidBody.create(world, {
        shape: box.create({ halfExtents: [backLegFull[0] / 2, backLegFull[1] / 2, backLegFull[2] / 2], convexRadius: 0.02 }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(bodyWidth * 0.5, 0, legZ),
        quaternion: quatZ(Math.PI / 4),
        linearDamping: 0.4,
        friction: 0.5,
        collisionGroup: GROUP_BODY,
        collisionMask: GROUP_GROUND,
        allowSleeping: false,
    });

    // Constraint 5: backLeg → chassis
    // cannon: pivotA = backLeg([0, 0, -1]) -> [0, 0, -legLength*1], pivotB = normBody([0, -0.5, 0.5])
    // The cannon pivot [0,0,-1] with normalizeSize([1, legLength, partDepth]) = [0, 0, -partDepth]
    // That's just the inner face of the part
    hingeConstraint.create(world, {
        bodyIdA: backLeg.id,
        bodyIdB: chassisBody.id,
        pointA: [0, 0, 0],
        pointB: [0, -bodyWidth * 0.5, bodyPivotZ],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.LOCAL,
    });

    // --- Triangulation constraints ---
    // These connect parts on the same side to form the closed-loop linkage.
    // All parts at same Z, so Z pivot = 0.

    // Constraint 6: upperFrontLeg → frontLeg
    hingeConstraint.create(world, {
        bodyIdA: upperFrontLeg.id,
        bodyIdB: frontLeg.id,
        pointA: [0, 1.5, 0],
        pointB: [0, 2.5, 0],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.LOCAL,
    });

    // Constraint 7: backLeg → horizontalBar
    hingeConstraint.create(world, {
        bodyIdA: backLeg.id,
        bodyIdB: horizontalBar.id,
        pointA: [0, 2.5, 0],
        pointB: [0, 5, 0],
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.LOCAL,
    });
}

// Create legs on both sides of the chassis
createLegSet(+1, 0);           // right side, crank at 0
createLegSet(-1, Math.PI);    // left side, crank offset by half revolution

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

ui.gui.add({ motorSpeed }, 'motorSpeed', 1, 15, 0.5).name('Motor Speed').onChange((v: number) => {
    motorSpeed = v;
    updateMotorTargets();
});

const maxDelta = 1 / 30;
let lastTime = performance.now();

// Track the robot body for the camera
const robotTarget = new THREE.Vector3();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // Update physics
    debugUI.beginPerf(ui);
    updateWorld(world, constraintFilterListener, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Follow the robot with the camera
    const bodyPos = chassisBody.centerOfMassPosition;
    robotTarget.set(bodyPos[0], bodyPos[1], bodyPos[2]);
    camera.lookAt(robotTarget);

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    // Update orbit controls
    orbitControls.target.copy(robotTarget);
    orbitControls.update();

    // Render
    renderer.render(scene, camera);
}

animate();
