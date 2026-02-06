import type { Vec3 } from 'mathcat';
import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { Listener, RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    ConstraintSpace,
    coneConstraint,
    createWorld,
    createWorldSettings,
    distanceConstraint,
    enableCollision,
    fixedConstraint,
    hingeConstraint,
    MotionType,
    pointConstraint,
    rigidBody,
    sixDOFConstraint,
    sliderConstraint,
    swingTwistConstraint,
    updateWorld,
    registerAllShapes,
    SpringMode,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(10, 30, 30);
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
const groundShape = box.create({ halfExtents: [50, 0.5, 10] });
rigidBody.create(world, {
    shape: groundShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0,
    friction: 0.5,
});

/* constraint demos */

/** creates a listener that filters out collisions between constraint-connected bodies */
function createConstraintFilterListener(): Listener {
    return {
        onBodyPairValidate: (bodyA: RigidBody, bodyB: RigidBody): boolean => {
            // skip collision if bodies are connected by a constraint
            return !rigidBody.bodiesShareConstraint(bodyA, bodyB);
        },
    };
}

const constraintFilterListener = createConstraintFilterListener();

// Helper to create a label above a constraint chain
function createLabel(text: string, x: number, y: number, z: number): void {
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 256;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 24px monospace';
    context.textAlign = 'center';
    context.fillText(text, 128, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(4, 1, 1);
    sprite.position.set(x, y, z);
    scene.add(sprite);
}

// Helper to create a box body
function createBody(position: Vec3, motionType: MotionType, objectLayer: number): RigidBody {
    const shape = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 });

    const body = rigidBody.create(world, {
        shape,
        objectLayer,
        motionType,
        position,
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });

    return body;
}

const CHAIN_LENGTH = 10;

function degreesToRadians(degrees: number): number {
    return degrees * (Math.PI / 180);
}

// Fixed constraint
{
    const startX = -15;
    const startY = 20;
    const startZ = 0;

    createLabel('Fixed', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            fixedConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                point1: vec3.fromValues(startX, startY, startZ + z - 0.5),
                point2: vec3.fromValues(startX, startY, startZ + z - 0.5),
                axisX1: vec3.fromValues(1, 0, 0),
                axisX2: vec3.fromValues(1, 0, 0),
                axisY1: vec3.fromValues(0, 1, 0),
                axisY2: vec3.fromValues(0, 1, 0),
                space: ConstraintSpace.WORLD,
            });
        }

        // Add impulse to last body
        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

// Point constraint
{
    const startX = -10;
    const startY = 20;
    const startZ = 0;

    createLabel('Point', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            pointConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                pointA: vec3.fromValues(startX, startY, startZ + z - 0.5),
                pointB: vec3.fromValues(startX, startY, startZ + z - 0.5),
                space: ConstraintSpace.WORLD,
            });
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

// Distance constraint
{
    const startX = -5;
    const startY = 20;
    const startZ = 0;

    createLabel('Distance', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            distanceConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                pointA: vec3.fromValues(startX, startY, startZ + z - 0.5),
                pointB: vec3.fromValues(startX, startY, startZ + z - 0.5),
                minDistance: 0,
                maxDistance: 1,
                space: ConstraintSpace.WORLD,
            });
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

// Hinge constraint
{
    const startX = 0;
    const startY = 20;
    const startZ = 0;

    createLabel('Hinge', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            hingeConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                pointA: vec3.fromValues(startX, startY - 0.5, startZ + z - 0.5),
                pointB: vec3.fromValues(startX, startY - 0.5, startZ + z - 0.5),
                hingeAxisA: vec3.fromValues(1, 0, 0),
                hingeAxisB: vec3.fromValues(1, 0, 0),
                normalAxisA: vec3.fromValues(0, 1, 0),
                normalAxisB: vec3.fromValues(0, 1, 0),
                space: ConstraintSpace.WORLD,
            });
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

// Slider constraint
{
    const startX = 5;
    const startY = 20;
    const startZ = 0;

    createLabel('Slider', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    const sliderAxis = vec3.fromValues(0, -1, 1);
    vec3.normalize(sliderAxis, sliderAxis);

    const normalAxis = vec3.create();
    vec3.perpendicular(normalAxis, sliderAxis);

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            const slider = sliderConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                pointA: vec3.fromValues(startX, startY, startZ + z - 0.5),
                pointB: vec3.fromValues(startX, startY, startZ + z - 0.5),
                sliderAxisA: sliderAxis,
                sliderAxisB: sliderAxis,
                normalAxisA: normalAxis,
                normalAxisB: normalAxis,
                limitsMin: 0,
                limitsMax: 1,
                space: ConstraintSpace.WORLD,
            });
            slider.limitsSpringSettings.mode = SpringMode.STIFFNESS_AND_DAMPING;
            slider.limitsSpringSettings.frequencyOrStiffness = 0;
            
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

// Cone constraint
{
    const startX = 10;
    const startY = 20;
    const startZ = 0;

    createLabel('Cone', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            coneConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                pointA: vec3.fromValues(startX, startY, startZ + z),
                pointB: vec3.fromValues(startX, startY, startZ + z),
                twistAxisA: vec3.fromValues(0, 0, 1),
                twistAxisB: vec3.fromValues(0, 0, 1),
                halfConeAngle: 0.1 * Math.PI,
                space: ConstraintSpace.WORLD,
            });
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

// Swing-twist constraint
{
    const startX = 15;
    const startY = 20;
    const startZ = 0;

    createLabel('Swing-Twist', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            swingTwistConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                position1: vec3.fromValues(startX, startY, startZ + z),
                position2: vec3.fromValues(startX, startY, startZ + z),
                twistAxis1: vec3.fromValues(0, 0, 1),
                twistAxis2: vec3.fromValues(0, 0, 1),
                planeAxis1: vec3.fromValues(1, 0, 0),
                planeAxis2: vec3.fromValues(1, 0, 0),
                normalHalfConeAngle: 0.2 * Math.PI,
                planeHalfConeAngle: 0.1 * Math.PI,
                twistMinAngle: -0.3 * Math.PI,
                twistMaxAngle: 0.4 * Math.PI,
                space: ConstraintSpace.WORLD,
            });
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

// Six DOF constraint
{
    const startX = 20;
    const startY = 20;
    const startZ = 0;

    createLabel('6DOF', startX, startY + 2, startZ + CHAIN_LENGTH / 2);

    let prevBody: RigidBody | null = null;

    const maxAngle = degreesToRadians(10);

    for (let z = 0; z < CHAIN_LENGTH; z++) {
        const position = vec3.fromValues(startX, startY, startZ + z);
        const motionType = z === 0 ? MotionType.STATIC : MotionType.DYNAMIC;
        const objectLayer = z === 0 ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING;

        const body = createBody(position, motionType, objectLayer);

        if (prevBody !== null) {
            sixDOFConstraint.create(world, {
                bodyIdA: prevBody.id,
                bodyIdB: body.id,
                position1: vec3.fromValues(startX, startY, startZ + z),
                position2: vec3.fromValues(startX, startY, startZ + z),
                axisX1: vec3.fromValues(0, 0, 1),
                axisX2: vec3.fromValues(0, 0, 1),
                axisY1: vec3.fromValues(1, 0, 0),
                axisY2: vec3.fromValues(1, 0, 0),
                space: ConstraintSpace.WORLD,
                limitMin: [-0.1, -0.1, -0.1, -maxAngle, -maxAngle, -maxAngle],
                limitMax: [0.1, 0.1, 0.1, maxAngle, maxAngle, maxAngle],
            });
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(1000, 0, 0));
        }

        prevBody = body;
    }
}

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

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
