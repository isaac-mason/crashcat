import { euler, quat, vec3, type Vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    cylinder,
    createWorld,
    createWorldSettings,
    enableCollision,
    type Listener,
    MotionType,
    rigidBody,
    updateWorld,
    registerAll,
    type ContactSettings,
    type RigidBody,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(60, 20, 50);
camera.lookAt(0, 0, 0);

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
orbitControls.target.set(0, 0, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
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

/* conveyor belt configuration */

type BeltConfig = {
    bodyId: number;
    type: 'linear' | 'angular';
    localLinearVelocity?: Vec3;
    localAngularVelocity?: Vec3;
};

const beltConfigs: Map<number, BeltConfig> = new Map();

// temp vectors for contact listener
const _tempWorldVel = vec3.create();
const _tempR = vec3.create();
const _tempAngularVel = vec3.create();
const _tempBody1LinearVel = vec3.create();
const _tempBody2LinearVel = vec3.create();
const _tempBody1AngularVel = vec3.create();
const _tempBody2AngularVel = vec3.create();
const _tempBody2LinearFromAngular = vec3.create();

function handleConveyorContact(bodyA: RigidBody, bodyB: RigidBody, settings: ContactSettings): void {
    const configA = beltConfigs.get(bodyA.id);
    const configB = beltConfigs.get(bodyB.id);

    if (!configA && !configB) return;

    // handle linear belt
    if (configA?.type === 'linear' || configB?.type === 'linear') {
        // determine the world space surface velocity of both bodies
        vec3.set(_tempBody1LinearVel, 0, 0, 0);
        vec3.set(_tempBody2LinearVel, 0, 0, 0);

        if (configA?.type === 'linear') {
            vec3.transformQuat(_tempBody1LinearVel, configA.localLinearVelocity!, bodyA.quaternion);
        }
        if (configB?.type === 'linear') {
            vec3.transformQuat(_tempBody2LinearVel, configB.localLinearVelocity!, bodyB.quaternion);
        }

        // calculate the relative surface velocity (body2 - body1)
        vec3.subtract(_tempWorldVel, _tempBody2LinearVel, _tempBody1LinearVel);
        vec3.copy(settings.relativeLinearSurfaceVelocity, _tempWorldVel);
    }

    // handle angular belt
    if (configA?.type === 'angular' || configB?.type === 'angular') {
        const body1Angular = configA?.type === 'angular';
        const body2Angular = configB?.type === 'angular';

        // determine the world space angular surface velocity of both bodies
        vec3.set(_tempBody1AngularVel, 0, 0, 0);
        vec3.set(_tempBody2AngularVel, 0, 0, 0);

        if (body1Angular) {
            vec3.transformQuat(_tempBody1AngularVel, configA!.localAngularVelocity!, bodyA.quaternion);
        }
        if (body2Angular) {
            vec3.transformQuat(_tempBody2AngularVel, configB!.localAngularVelocity!, bodyB.quaternion);
        }

        // note that the angular velocity is the angular velocity around body 1's center of mass,
        // so we need to add the linear velocity of body 2's center of mass
        const com1 = bodyA.centerOfMassPosition;
        const com2 = bodyB.centerOfMassPosition;
        vec3.set(_tempBody2LinearFromAngular, 0, 0, 0);

        if (body2Angular) {
            // compute: body2AngularSurfaceVelocity × (COM1 - COM2)
            vec3.subtract(_tempR, com1, com2);
            vec3.cross(_tempBody2LinearFromAngular, _tempBody2AngularVel, _tempR);
        }

        // calculate the relative angular surface velocity
        vec3.copy(settings.relativeLinearSurfaceVelocity, _tempBody2LinearFromAngular);
        vec3.subtract(_tempAngularVel, _tempBody2AngularVel, _tempBody1AngularVel);
        vec3.copy(settings.relativeAngularSurfaceVelocity, _tempAngularVel);
    }
}

const listener: Listener = {
    onContactAdded: (bodyA, bodyB, _manifold, settings) => {
        handleConveyorContact(bodyA, bodyB, settings);
    },
    onContactPersisted: (bodyA, bodyB, _manifold, settings) => {
        handleConveyorContact(bodyA, bodyB, settings);
    },
};

const world = createWorld(worldSettings);

(globalThis as any).world = world;

/* create scene */

// floor
const floorShape = box.create({ halfExtents: [100, 0.5, 100] });
rigidBody.create(world, {
    shape: floorShape,
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -0.5, 0),
    restitution: 0,
    friction: 1.0,
});

// create 4 linear conveyor belts in a cross pattern
const beltWidth = 10.0;
const beltLength = 50.0;
const linearBelts: RigidBody[] = [];

for (let i = 0; i < 4; i++) {
    const friction = 0.25 * (i + 1);

    // rotate belt around Y axis (90 degrees each iteration)
    const rot1 = quat.fromEuler(quat.create(), euler.fromValues(0, (0.5 * Math.PI * i), 0, 'xyz'));
    // slight upward tilt (1 degree)
    const rot2 = quat.fromEuler(quat.create(), euler.fromValues(Math.PI / 180, 0, 0, 'xyz'));
    const rotation = quat.multiply(quat.create(), rot1, rot2);

    // position belt along its rotated axis
    // in jolt: new THREE.Vector3(cBeltLength, 6.0, cBeltWidth).applyQuaternion(rotation)
    const localOffset = vec3.fromValues(beltLength, 6.0, beltWidth);
    const position = vec3.transformQuat(vec3.create(), localOffset, rotation);

    const belt = rigidBody.create(world, {
        shape: box.create({ halfExtents: [beltWidth, 0.1, beltLength] }),
        objectLayer: OBJECT_LAYER_NOT_MOVING,
        motionType: MotionType.STATIC,
        position,
        quaternion: rotation,
        restitution: 0,
        friction,
    });

    // configure belt with local space velocity
    beltConfigs.set(belt.id, {
        bodyId: belt.id,
        type: 'linear',
        localLinearVelocity: vec3.fromValues(0, 0, -10.0), // move along local -Z
    });

    linearBelts.push(belt);
}

// create cargo boxes with decreasing friction on the linear belts
for (let i = 0; i <= 10; i++) {
    const friction = 1.0 - 0.1 * i;
    const xPos = -beltLength + i * 10.0;

    rigidBody.create(world, {
        shape: box.create({ halfExtents: [2, 2, 2] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(xPos, 10.0, -beltLength),
        restitution: 0,
        friction,
    });

    // add label
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 28px monospace';
    context.textAlign = 'center';
    context.fillText(`μ=${friction.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(xPos, 14, -beltLength);
    scene.add(sprite);
}

// create 2 cylinders (axles for dynamic belt)
const cylinderRotation = quat.fromEuler(quat.create(), euler.fromValues(0, 0, Math.PI / 2, 'xyz'));
rigidBody.create(world, {
    shape: cylinder.create({ halfHeight: 6.0, radius: 1.0 }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-25.0, 1.0, -20.0),
    quaternion: cylinderRotation,
    restitution: 0,
    friction: 0.1,
});

rigidBody.create(world, {
    shape: cylinder.create({ halfHeight: 6.0, radius: 1.0 }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-25.0, 1.0, 20.0),
    quaternion: cylinderRotation,
    restitution: 0,
    friction: 0.1,
});

// dynamic belt resting on cylinders
const dynamicBelt = rigidBody.create(world, {
    shape: box.create({ halfExtents: [5.0, 0.1, 25.0] }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-25.0, 3.0, 0),
    restitution: 0,
    friction: 1.0,
});

beltConfigs.set(dynamicBelt.id, {
    bodyId: dynamicBelt.id,
    type: 'linear',
    localLinearVelocity: vec3.fromValues(0, 0, -10.0),
});

linearBelts.push(dynamicBelt);

// cargo on dynamic belt
rigidBody.create(world, {
    shape: box.create({ halfExtents: [2, 2, 2] }),
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.DYNAMIC,
    position: vec3.fromValues(-25.0, 6.0, 15.0),
    restitution: 0,
    friction: 1.0,
});

// angular/rotating belt (lazy susan)
const angularBelt = rigidBody.create(world, {
    shape: box.create({ halfExtents: [20.0, 0.1, 20.0] }),
    objectLayer: OBJECT_LAYER_NOT_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(10.0, 3.0, 0),
    restitution: 0,
    friction: 1.0,
});

beltConfigs.set(angularBelt.id, {
    bodyId: angularBelt.id,
    type: 'angular',
    localAngularVelocity: vec3.fromValues(0, (10.0 * Math.PI) / 180, 0), // 10 degrees/sec around Y
});

// cargo with decreasing friction on angular belt
for (let i = 0; i <= 6; i++) {
    const friction = 1.0 - 0.1 * i;
    const zPos = -15.0 + 5.0 * i;

    rigidBody.create(world, {
        shape: box.create({ halfExtents: [2, 2, 2] }),
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position: vec3.fromValues(10.0, 10.0, zPos),
        restitution: 0,
        friction,
    });

    // add label
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 128;
    canvas.height = 64;
    context.fillStyle = '#ffffff';
    context.font = 'bold 28px monospace';
    context.textAlign = 'center';
    context.fillText(`μ=${friction.toFixed(1)}`, 64, 40);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2, 1, 1);
    sprite.position.set(10.0, 14, zPos);
    scene.add(sprite);
}

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

/* simulation */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    debugUI.beginPerf(ui);
    updateWorld(world, listener, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    debugRenderer.update(debugRendererState, world);

    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
