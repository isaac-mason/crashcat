import type { Listener, RigidBody } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    ConstraintSpace,
    capsule,
    compound,
    coneConstraint,
    convexHull,
    createWorld,
    createWorldSettings,
    cylinder,
    debug,
    distanceConstraint,
    enableCollision,
    fixedConstraint,
    hingeConstraint,
    MotionType,
    plane,
    pointConstraint,
    registerAll,
    rigidBody,
    sixDOFConstraint,
    sliderConstraint,
    sphere,
    staticCompound,
    swingTwistConstraint,
    updateWorld,
} from 'crashcat';
import { quat, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 15, 30);
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

registerAll();

const worldSettings = createWorldSettings();

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

function createConstraintFilterListener(): Listener {
    return {
        onBodyPairValidate: (bodyA: RigidBody, bodyB: RigidBody): boolean => {
            return !rigidBody.bodiesShareConstraint(bodyA, bodyB);
        },
    };
}

const constraintFilterListener = createConstraintFilterListener();

const world = createWorld(worldSettings);

const groundShape = plane.create({
    plane: { normal: [0, 1, 0], constant: 0 },
    halfExtent: 50,
});
rigidBody.create(world, {
    shape: groundShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    restitution: 0,
    friction: 0.5,
});

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

function createBox(x: number, y: number, z: number, sx: number, sy: number, sz: number, motionType: MotionType): RigidBody {
    const shape = box.create({ halfExtents: [sx, sy, sz], convexRadius: 0.05 });
    return rigidBody.create(world, {
        shape,
        objectLayer: motionType === MotionType.STATIC ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING,
        motionType,
        position: vec3.fromValues(x, y, z),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
}

function createSphere(x: number, y: number, z: number, radius: number, motionType: MotionType): RigidBody {
    const shape = sphere.create({ radius });
    return rigidBody.create(world, {
        shape,
        objectLayer: motionType === MotionType.STATIC ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING,
        motionType,
        position: vec3.fromValues(x, y, z),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
}

function createCapsule(x: number, y: number, z: number, halfHeight: number, radius: number, motionType: MotionType): RigidBody {
    const shape = capsule.create({ halfHeightOfCylinder: halfHeight, radius });
    return rigidBody.create(world, {
        shape,
        objectLayer: motionType === MotionType.STATIC ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING,
        motionType,
        position: vec3.fromValues(x, y, z),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
}

function createCylinder(x: number, y: number, z: number, halfHeight: number, radius: number, motionType: MotionType): RigidBody {
    const shape = cylinder.create({ halfHeight, radius });
    return rigidBody.create(world, {
        shape,
        objectLayer: motionType === MotionType.STATIC ? OBJECT_LAYER_NOT_MOVING : OBJECT_LAYER_MOVING,
        motionType,
        position: vec3.fromValues(x, y, z),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
}

const shapes: RigidBody[] = [];

{
    createLabel('Shapes', 0, 12, -15);
    const startX = -12;
    const startY = 1;
    const spacing = 4;

    shapes.push(createSphere(startX, startY, 0, 1, MotionType.DYNAMIC));
    createLabel('Sphere', startX, startY + 2.5, 0);

    shapes.push(createBox(startX + spacing, startY, 0, 1, 1, 1, MotionType.DYNAMIC));
    createLabel('Box', startX + spacing, startY + 2.5, 0);

    shapes.push(createCapsule(startX + spacing * 2, startY + 1, 0, 1, 0.8, MotionType.DYNAMIC));
    createLabel('Capsule', startX + spacing * 2, startY + 3.5, 0);

    shapes.push(createCylinder(startX + spacing * 3, startY, 0, 1, 0.8, MotionType.DYNAMIC));
    createLabel('Cylinder', startX + spacing * 3, startY + 2.5, 0);
}

{
    createLabel('Static Shapes', -10, 6, -8);
    const startX = -14;
    const startY = 0.5;
    const spacing = 4;

    shapes.push(createBox(startX, startY, -10, 1.5, 0.5, 1.5, MotionType.STATIC));
    createLabel('Static Box', startX, startY + 2, -10);

    shapes.push(createSphere(startX + spacing, startY + 1, -10, 1, MotionType.STATIC));
    createLabel('Static Sphere', startX + spacing, startY + 3, -10);
}

{
    const compoundShape = compound.create({
        children: [
            {
                position: vec3.fromValues(-0.6, 0, 0),
                quaternion: quat.create(),
                shape: sphere.create({ radius: 0.6 }),
            },
            {
                position: vec3.fromValues(0.6, 0, 0),
                quaternion: quat.create(),
                shape: sphere.create({ radius: 0.6 }),
            },
            {
                position: vec3.fromValues(0, 0, 0),
                quaternion: quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI * 0.5),
                shape: cylinder.create({ halfHeight: 0.5, radius: 0.3 }),
            },
        ],
    });
    const compoundBody = rigidBody.create(world, {
        shape: compoundShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(10, 1, -10),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
    shapes.push(compoundBody);
    createLabel('Compound', 10, 4, -10);
}

{
    const staticCompoundShape = staticCompound.create({
        children: [
            {
                position: vec3.fromValues(-1, 0, 0),
                quaternion: quat.create(),
                shape: box.create({ halfExtents: [0.3, 1, 0.3], convexRadius: 0.05 }),
            },
            {
                position: vec3.fromValues(1, 0, 0),
                quaternion: quat.create(),
                shape: box.create({ halfExtents: [0.3, 1, 0.3], convexRadius: 0.05 }),
            },
            ...[-0.5, 0.5].map((z) => ({
                position: vec3.fromValues(0, -0.5, z),
                quaternion: quat.create(),
                shape: box.create({ halfExtents: [1.2, 0.1, 0.1], convexRadius: 0.05 }),
            })),
        ],
    });
    const staticCompoundBody = rigidBody.create(world, {
        shape: staticCompoundShape,
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position: vec3.fromValues(10, 1, -4),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
    shapes.push(staticCompoundBody);
    createLabel('Static Compound', 10, 4, -4);
}

{
    const pyramidPositions = [-0.8, 0, -0.8, 0.8, 0, -0.8, 0.8, 0, 0.8, -0.8, 0, 0.8, 0, 1.2, 0];
    const convexHullShape = convexHull.create({
        positions: pyramidPositions,
        convexRadius: 0.05,
    });
    const convexHullBody = rigidBody.create(world, {
        shape: convexHullShape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(10, 1, 2),
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
    shapes.push(convexHullBody);
    createLabel('Convex Hull', 10, 4, 2);
}

const CHAIN_LENGTH = 5;

function createBody(position: Vec3, motionType: MotionType, objectLayer: number): RigidBody {
    const shape = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 });
    return rigidBody.create(world, {
        shape,
        objectLayer,
        motionType,
        position,
        quaternion: quat.create(),
        restitution: 0,
        friction: 0.5,
    });
}

{
    createLabel('Constraints', -10, 15, 0);

    const startX = -15;
    const startY = 20;
    const startZ = 0;

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

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('Fixed', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

{
    const startX = -10;
    const startY = 20;
    const startZ = 0;

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
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('Point', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

{
    const startX = -5;
    const startY = 20;
    const startZ = 0;

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
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('Distance', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

{
    const startX = 0;
    const startY = 20;
    const startZ = 0;

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
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('Hinge', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

{
    const startX = 5;
    const startY = 20;
    const startZ = 0;

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
            sliderConstraint.create(world, {
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
        }

        if (z === CHAIN_LENGTH - 1) {
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('Slider', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

{
    const startX = 10;
    const startY = 20;
    const startZ = 0;

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
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('Cone', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

{
    const startX = 15;
    const startY = 20;
    const startZ = 0;

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
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('Swing-Twist', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

{
    const startX = 20;
    const startY = 20;
    const startZ = 0;

    let prevBody: RigidBody | null = null;

    const maxAngle = Math.PI / 18;

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
            rigidBody.addImpulse(world, body, vec3.fromValues(500, 0, 0));
        }

        prevBody = body;
    }
    createLabel('6DOF', startX, startY + 2, startZ + CHAIN_LENGTH / 2);
}

const debugOptions = {
    showBodies: true,
    showContacts: true,
    showContactConstraints: true,
    showConstraints: true,
    colorMode: debug.BodyColorMode.MOTION_TYPE,
};

const MAX_LINES = 50000;
const MAX_VERTICES = MAX_LINES * 2;

function createDebugLineObject(): THREE.LineSegments {
    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(MAX_VERTICES * 3);
    const colors = new Float32Array(MAX_VERTICES * 3);
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    const material = new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 });
    const segments = new THREE.LineSegments(geometry, material);
    segments.frustumCulled = false;
    return segments;
}

const bodiesLineSegments = createDebugLineObject();
scene.add(bodiesLineSegments);

const contactsLineSegments = createDebugLineObject();
scene.add(contactsLineSegments);

const contactConstraintsLineSegments = createDebugLineObject();
scene.add(contactConstraintsLineSegments);

const constraintsLineSegments = createDebugLineObject();
scene.add(constraintsLineSegments);

function updateDebugLines() {
    if (debugOptions.showBodies) {
        const result = debug.bodies(world, {
            colorMode: debugOptions.colorMode,
            showLinearVelocity: false,
            showAngularVelocity: false,
        });
        const count = Math.min(result.numLines, MAX_LINES);
        if (count > 0) {
            const positions = bodiesLineSegments.geometry.attributes.position.array as Float32Array;
            const colors = bodiesLineSegments.geometry.attributes.color.array as Float32Array;
            positions.set(result.vertices.subarray(0, count * 6));
            colors.set(result.colors.subarray(0, count * 6));
            bodiesLineSegments.geometry.attributes.position.needsUpdate = true;
            bodiesLineSegments.geometry.attributes.color.needsUpdate = true;
            bodiesLineSegments.geometry.setDrawRange(0, count * 2);
            bodiesLineSegments.visible = true;
        } else {
            bodiesLineSegments.visible = false;
        }
    } else {
        bodiesLineSegments.visible = false;
    }

    if (debugOptions.showContacts) {
        const result = debug.contacts(world, {});
        const count = Math.min(result.numLines, MAX_LINES);
        if (count > 0) {
            const positions = contactsLineSegments.geometry.attributes.position.array as Float32Array;
            const colors = contactsLineSegments.geometry.attributes.color.array as Float32Array;
            positions.set(result.vertices.subarray(0, count * 6));
            colors.set(result.colors.subarray(0, count * 6));
            contactsLineSegments.geometry.attributes.position.needsUpdate = true;
            contactsLineSegments.geometry.attributes.color.needsUpdate = true;
            contactsLineSegments.geometry.setDrawRange(0, count * 2);
            contactsLineSegments.visible = true;
        } else {
            contactsLineSegments.visible = false;
        }
    } else {
        contactsLineSegments.visible = false;
    }

    if (debugOptions.showContactConstraints) {
        const result = debug.contactConstraints(world, {});
        const count = Math.min(result.numLines, MAX_LINES);
        if (count > 0) {
            const positions = contactConstraintsLineSegments.geometry.attributes.position.array as Float32Array;
            const colors = contactConstraintsLineSegments.geometry.attributes.color.array as Float32Array;
            positions.set(result.vertices.subarray(0, count * 6));
            colors.set(result.colors.subarray(0, count * 6));
            contactConstraintsLineSegments.geometry.attributes.position.needsUpdate = true;
            contactConstraintsLineSegments.geometry.attributes.color.needsUpdate = true;
            contactConstraintsLineSegments.geometry.setDrawRange(0, count * 2);
            contactConstraintsLineSegments.visible = true;
        } else {
            contactConstraintsLineSegments.visible = false;
        }
    } else {
        contactConstraintsLineSegments.visible = false;
    }

    if (debugOptions.showConstraints) {
        const result = debug.joints(world, { size: 0.5, drawLimits: true });
        const count = Math.min(result.numLines, MAX_LINES);
        if (count > 0) {
            const positions = constraintsLineSegments.geometry.attributes.position.array as Float32Array;
            const colors = constraintsLineSegments.geometry.attributes.color.array as Float32Array;
            positions.set(result.vertices.subarray(0, count * 6));
            colors.set(result.colors.subarray(0, count * 6));
            constraintsLineSegments.geometry.attributes.position.needsUpdate = true;
            constraintsLineSegments.geometry.attributes.color.needsUpdate = true;
            constraintsLineSegments.geometry.setDrawRange(0, count * 2);
            constraintsLineSegments.visible = true;
        } else {
            constraintsLineSegments.visible = false;
        }
    } else {
        constraintsLineSegments.visible = false;
    }
}

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    updateWorld(world, constraintFilterListener, delta);

    updateDebugLines();

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
