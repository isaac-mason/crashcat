import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { HingeConstraint, Listener, RigidBody, SliderConstraint } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    ConstraintSpace,
    compound,
    createWorld,
    createWorldSettings,
    enableCollision,
    hingeConstraint,
    MotionType,
    MotorState,
    rigidBody,
    sliderConstraint,
    sphere,
    updateWorld,
    registerAllShapes,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 15, 35);
camera.lookAt(0, 5, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
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
orbitControls.target.set(0, 5, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* physics world */

registerAllShapes();

const worldSettings = createWorldSettings();

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

// Create static ground
const groundShape = box.create({ halfExtents: [200, 0.5, 50] });
rigidBody.create(world, {
    shape: groundShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0,
    friction: 0.5,
});

/* motor demos */

/** create a listener that filters out collisions between constraint-connected bodies */
const constraintFilterListener: Listener = {
    onBodyPairValidate: (bodyA: RigidBody, bodyB: RigidBody): boolean => {
        // skip collision if bodies are connected by a constraint
        return !rigidBody.bodiesShareConstraint(bodyA, bodyB);
    },
};

// helper to create a label
function createLabel(text: string, x: number, y: number, z: number): void {
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 512;
    canvas.height = 128;
    context.fillStyle = '#ffffff';
    context.font = 'bold 48px monospace';
    context.textAlign = 'center';
    context.fillText(text, 256, 80);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(8, 2, 1);
    sprite.position.set(x, y, z);
    scene.add(sprite);
}

// Windmill with powered hinge constraint
let windmillHinge: HingeConstraint;
{
    const centerX = -20;
    const centerY = 10;
    const centerZ = 0;

    createLabel('Powered Hinge', centerX, centerY + 12, centerZ);

    // Create a small static box to rotate the wings around
    const centerBoxShape = box.create({ halfExtents: [0.25, 0.25, 0.25] });
    const centerBox = rigidBody.create(world, {
        shape: centerBoxShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(centerX, centerY, centerZ),
        restitution: 0,
        friction: 0.5,
    });

    // Create the wings as a compound shape
    // Wing dimensions match Jolt: half-extent of 4 units, thickness of 0.5
    const wingHalfExtent = 4;
    const wingThickness = 0.5;
    const convexRadius = 0.05;

    // Position wings at distance 5 from center (half-extent 4 + convex radius 0.05 + margin)
    const wingOffset = 5;

    // Create 4 wings positioned at right, left, top, bottom
    const wingsShape = compound.create({
        children: [
            // Right wing
            {
                position: vec3.fromValues(wingOffset, 0, 0),
                quaternion: quat.create(),
                shape: box.create({
                    halfExtents: [wingHalfExtent, wingThickness, wingThickness],
                    convexRadius,
                }),
            },
            // Left wing
            {
                position: vec3.fromValues(-wingOffset, 0, 0),
                quaternion: quat.create(),
                shape: box.create({
                    halfExtents: [wingHalfExtent, wingThickness, wingThickness],
                    convexRadius,
                }),
            },
            // Top wing
            {
                position: vec3.fromValues(0, wingOffset, 0),
                quaternion: quat.create(),
                shape: box.create({
                    halfExtents: [wingThickness, wingHalfExtent, wingThickness],
                    convexRadius,
                }),
            },
            // Bottom wing
            {
                position: vec3.fromValues(0, -wingOffset, 0),
                quaternion: quat.create(),
                shape: box.create({
                    halfExtents: [wingThickness, wingHalfExtent, wingThickness],
                    convexRadius,
                }),
            },
        ],
    });

    // Create the wings body
    const wings = rigidBody.create(world, {
        shape: wingsShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(centerX, centerY, centerZ),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });

    // Constrain the wings to the center box using a powered hinge
    windmillHinge = hingeConstraint.create(world, {
        bodyIdA: centerBox.id,
        bodyIdB: wings.id,
        pointA: [centerX, centerY, centerZ],
        pointB: [centerX, centerY, centerZ],
        hingeAxisA: [0, 0, 1], // rotate around Z axis
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
        space: ConstraintSpace.WORLD,
    });

    // Set motor to velocity mode and target angular velocity
    hingeConstraint.setMotorState(windmillHinge, MotorState.VELOCITY);
    hingeConstraint.setTargetAngularVelocity(windmillHinge, -5);

    // Create a sphere that will move towards the windmill
    const sphereShape = sphere.create({ radius: 2 });
    const ballBody = rigidBody.create(world, {
        shape: sphereShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: [centerX - 20, 2, 0],
        quaternion: quat.create(),
        restitution: 0.45,
        friction: 0.5,
    });
    // Give the sphere initial velocity towards the windmill
    ballBody.motionProperties.linearVelocity[0] = 25;
    // Create a wall for the ball to bounce against
    const backWallShape = box.create({ halfExtents: [0.2, 15, 5] });
    rigidBody.create(world, {
        shape: backWallShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(centerX - 25, 15, 0),
        restitution: 0,
        friction: 0.5,
    });

    // Invisible right wall
    const rightWallShape = box.create({ halfExtents: [0.2, 150, 50] });
    rigidBody.create(world, {
        shape: rightWallShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(50, 0, 0),
        restitution: 0,
        friction: 0.5,
    });

    // Invisible top wall
    const topWallShape = box.create({ halfExtents: [150, 0.2, 50] });
    rigidBody.create(world, {
        shape: topWallShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(0, 75, 0),
        restitution: 0,
        friction: 0.5,
    });
}

// Slider with powered position motor
let poweredSliderConstraint: SliderConstraint;
let sliderBox: RigidBody;
{
    const platformX = 18;
    const platformY = 0.25;
    const platformZ = 0;

    createLabel('Powered Slider', platformX, 10, platformZ);

    // Create the static platform
    rigidBody.create(world, {
        shape: box.create({ halfExtents: [12.5, 0.25, 2] }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(platformX, platformY, platformZ),
        restitution: 0,
        friction: 0.5,
    });

    // Create the target end wall
    const targetShape = box.create({ halfExtents: [0.5, 2, 2] });
    const target = rigidBody.create(world, {
        shape: targetShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(30, 2, platformZ),
        restitution: 0,
        friction: 0.5,
    });

    // Create the moving box on the slider
    const boxShape = box.create({ halfExtents: [0.75, 0.75, 0.75] });
    sliderBox = rigidBody.create(world, {
        shape: boxShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(15, 1.25, platformZ),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
        allowSleeping: false,
    });

    // Place reference markers along the slider axis
    const colors = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00];
    const offsets = [0, 4, 13, 21];
    for (let i = 0; i < colors.length; i++) {
        const markerShape = box.create({ halfExtents: [1, 4, 0.75] });
        rigidBody.create(world, {
            shape: markerShape,
            objectLayer: OBJECT_LAYER_NOT_MOVING,
            motionType: MotionType.STATIC,
            position: vec3.fromValues(28 - offsets[i], 2, -2.75),
            restitution: 0,
            friction: 0.5,
        });
    }

    // Create the slider constraint
    poweredSliderConstraint = sliderConstraint.create(world, {
        bodyIdA: sliderBox.id,
        bodyIdB: target.id,
        pointA: [15, 1.25, platformZ],
        pointB: [28, 1.25, platformZ],
        sliderAxisA: [1, 0, 0],
        sliderAxisB: [1, 0, 0],
        normalAxisA: [0, 1, 0],
        normalAxisB: [0, 1, 0],
        space: ConstraintSpace.WORLD,
    });

    // poweredSliderConstraint.motorSettings.springSettings.frequencyOrStiffness = 10; // 8 Hz

    // set motor to position mode
    sliderConstraint.setMotorState(poweredSliderConstraint, MotorState.POSITION);
    sliderConstraint.setTargetPosition(poweredSliderConstraint, 0);
}

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

const maxDelta = 1 / 30;
let lastTime = performance.now();
const startTime = performance.now();
let pTime = 0; // Previous time when slider was updated
let counter = 4; // Start closer to initial spawn point

// Offsets for slider position sequence
const sliderOffsets = [0, 4, 13, 21];

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // Total elapsed time since start (in seconds)
    const time = (currentTime - startTime) / 1000;

    // Update slider position every second
    if (time - pTime > 1.0) {
        pTime = time;
        // Creates a sequence going from -n to n, and set to absolute to wrap the sequence to up/down stepping
        const index = Math.abs((counter++ % (sliderOffsets.length * 2 - 2)) - (sliderOffsets.length - 1));
        const targetPos = sliderOffsets[index];

        const currentPos = sliderConstraint.getCurrentPosition(poweredSliderConstraint, world.bodies);

        console.log(
            `[${time.toFixed(1)}s] Setting targetPos=${targetPos} (currentPos=${currentPos.toFixed(2)}, box.x=${sliderBox.centerOfMassPosition[0].toFixed(2)})`
        );

        sliderConstraint.setTargetPosition(poweredSliderConstraint, targetPos);
        rigidBody.wake(world, sliderBox); // activate body after changing target (matches Jolt)
    }

    // Update physics with delta time
    debugUI.beginPerf(ui);
    updateWorld(world, constraintFilterListener, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    // Update orbit controls
    orbitControls.update();

    // Render
    renderer.render(scene, camera);
}

animate();
