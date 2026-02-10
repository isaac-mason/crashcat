import GUI from 'lil-gui';
import type { Quat, Vec3 } from 'mathcat';
import { euler, quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type RigidBody,
    box,
    CastRayStatus,
    capsule,
    castRay,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
    MotionType,
    rigidBody,
    type Shape,
    sphere,
    triangleMesh,
    type World,
    registerAll,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';
import { createShapeHelper } from 'crashcat/three';

type MeshObject = {
    visualMesh: THREE.Object3D;
    body: RigidBody;
    position: Vec3;
    quaternion: Quat;
    rotationSpeed: Vec3;
};

type State = {
    renderer: THREE.WebGLRenderer;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    containerObj: THREE.Object3D;
    world: World;
    queryFilter: ReturnType<typeof filter.create>;
    meshObjects: MeshObject[];
    rayCasterObjects: RaycasterObject[];
    rayCollector: ReturnType<typeof createClosestCastRayCollector>;
    raySettings: ReturnType<typeof createDefaultCastRaySettings>;
    rayNear: number;
    rayFar: number;
    lastFrameTime: number | null;
    deltaTime: number;
    debugRendererState: ReturnType<typeof debugRenderer.init>;
    ui: ReturnType<typeof debugUI.init>;
};

let state: State;

const pointDist = 5;

const SHAPE_OPTIONS = {
    'Torus Knot Triangle Mesh': 'torus-knot',
    Sphere: 'sphere',
    Box: 'box',
    Capsule: 'capsule',
} as const;

type ShapeType = (typeof SHAPE_OPTIONS)[keyof typeof SHAPE_OPTIONS];

const params = {
    raycasters: {
        count: 150,
        speed: 1,
        near: 0,
        far: pointDist,
    },
    mesh: {
        shapeType: 'torus-knot' as ShapeType,
        count: 1,
        speed: 1,
    },
};

type RaycasterObject = {
    rootObject: THREE.Object3D;
    originMesh: THREE.Mesh;
    hitMesh: THREE.Mesh;
    cylinderMesh: THREE.Mesh;
    quaternion: Quat;
    rotationSpeed: Vec3;
    origin: Vec3;
    direction: Vec3;
};

const _vector3 = new THREE.Vector3();
const _deltaQuat = quat.create();

function createRaycasterObject(): RaycasterObject {
    const sphereGeometry = new THREE.SphereGeometry(0.25, 20, 20);
    const cylinderGeometry = new THREE.CylinderGeometry(0.01, 0.01);
    const material = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const lineMaterial = new THREE.MeshBasicMaterial({
        color: 0xffffff,
        transparent: true,
        opacity: 0.25,
    });

    const rootObject = new THREE.Object3D();
    const originMesh = new THREE.Mesh(sphereGeometry, material);
    const hitMesh = new THREE.Mesh(sphereGeometry, material);
    const cylinderMesh = new THREE.Mesh(cylinderGeometry, lineMaterial);

    originMesh.scale.multiplyScalar(0.1);
    hitMesh.scale.multiplyScalar(0.05);

    rootObject.add(cylinderMesh);
    rootObject.add(originMesh);
    rootObject.add(hitMesh);

    // position origin at pointDist from center
    originMesh.position.set(pointDist, 0, 0);

    // random initial rotation as quaternion (source of truth)
    const quaternion = quat.create();
    quat.fromEuler(quaternion, euler.fromValues(Math.random() * 10, Math.random() * 10, Math.random() * 10, 'xyz'));

    // random rotation speeds
    const rotationSpeed = vec3.fromValues(
        (Math.random() - 0.5) * 0.0001,
        (Math.random() - 0.5) * 0.0001,
        (Math.random() - 0.5) * 0.0001,
    );

    state.scene.add(rootObject);

    return {
        rootObject,
        originMesh,
        hitMesh,
        cylinderMesh,
        quaternion,
        rotationSpeed,
        origin: vec3.create(),
        direction: vec3.create(),
    };
}

function updateRaycasterObject(rc: RaycasterObject) {
    // Update quaternion based on rotation speed
    quat.fromEuler(
        _deltaQuat,
        euler.fromValues(
            rc.rotationSpeed[0] * params.raycasters.speed * state.deltaTime,
            rc.rotationSpeed[1] * params.raycasters.speed * state.deltaTime,
            rc.rotationSpeed[2] * params.raycasters.speed * state.deltaTime,
            'xyz',
        ),
    );
    quat.multiply(rc.quaternion, rc.quaternion, _deltaQuat);

    // Set Three.js rootObject from mathcat quaternion
    rc.rootObject.quaternion.set(rc.quaternion[0], rc.quaternion[1], rc.quaternion[2], rc.quaternion[3]);

    // Get origin position in world space
    rc.originMesh.updateMatrixWorld();
    _vector3.setFromMatrixPosition(rc.originMesh.matrixWorld);

    // Convert to vec3 for crashcat
    vec3.set(rc.origin, _vector3.x, _vector3.y, _vector3.z);

    // Direction points toward center (negative of origin position)
    vec3.negate(rc.direction, rc.origin);

    // Normalize direction
    vec3.normalize(rc.direction, rc.direction);

    // Perform raycast with crashcat
    state.rayCollector.reset();
    castRay(state.world, state.rayCollector, state.raySettings, rc.origin, rc.direction, pointDist, state.queryFilter);

    let hitDistance = pointDist;
    const hadHit = state.rayCollector.hit.status === CastRayStatus.COLLIDING;
    if (hadHit) {
        hitDistance = state.rayCollector.hit.fraction * pointDist;
    }

    // Update hit mesh position
    rc.hitMesh.position.set(pointDist - hitDistance, 0, 0);

    // Update cylinder (ray line) between near and hit point
    const lineLength = hadHit ? hitDistance - state.rayNear : state.rayFar - state.rayNear;

    rc.cylinderMesh.position.set(pointDist - state.rayNear - lineLength / 2, 0, 0);
    rc.cylinderMesh.scale.set(1, lineLength, 1);
    rc.cylinderMesh.rotation.z = Math.PI / 2;
}

function removeRaycasterObject(rc: RaycasterObject) {
    state.scene.remove(rc.rootObject);
}

registerAll();

function initWorld() {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    state.world = createWorld(worldSettings);
    state.queryFilter = filter.create(worldSettings.layers);
}

function init() {
    // Renderer setup
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    // Scene setup
    const scene = new THREE.Scene();
    scene.fog = new THREE.Fog(0x263238 / 2, 40, 80);

    const light = new THREE.DirectionalLight(0xffffff, 0.5);
    light.position.set(1, 1, 1);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0xffffff, 0.4));

    // Container for meshes
    const containerObj = new THREE.Object3D();
    containerObj.scale.multiplyScalar(1);
    containerObj.rotation.x = 10.989999999999943;
    containerObj.rotation.y = 10.989999999999943;
    scene.add(containerObj);

    // Camera setup
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 50);
    camera.position.z = 10;
    camera.far = 100;
    camera.updateProjectionMatrix();

    // Initialize debug renderer
    const options = debugRenderer.createDefaultOptions();
    const debugRendererState = debugRenderer.init(options);
    const ui = debugUI.init(options);
    debugRendererState.options.bodies.enabled = false;
    scene.add(debugRendererState.object3d);

    // Initialize state
    state = {
        renderer,
        scene,
        camera,
        containerObj,
        world: null as any, // Will be set by initWorld
        queryFilter: null as any, // Will be set by initWorld
        meshObjects: [],
        rayCasterObjects: [],
        rayCollector: createClosestCastRayCollector(),
        raySettings: createDefaultCastRaySettings(),
        rayNear: 0,
        rayFar: pointDist,
        lastFrameTime: null,
        deltaTime: 0,
        debugRendererState,
        ui,
    };

    // Initialize physics world
    initWorld();

    // GUI
    const gui = new GUI();
    const rcFolder = gui.addFolder('Raycasters');
    rcFolder
        .add(params.raycasters, 'count')
        .min(1)
        .max(1000)
        .step(1)
        .onChange(() => updateFromOptions());
    rcFolder.add(params.raycasters, 'speed').min(0).max(20);
    rcFolder
        .add(params.raycasters, 'near')
        .min(0)
        .max(pointDist)
        .onChange(() => updateFromOptions());
    rcFolder
        .add(params.raycasters, 'far')
        .min(0)
        .max(pointDist)
        .onChange(() => updateFromOptions());
    rcFolder.open();

    const meshFolder = gui.addFolder('Mesh');
    meshFolder.add(params.mesh, 'shapeType', SHAPE_OPTIONS).onChange(() => updateFromOptions());
    meshFolder
        .add(params.mesh, 'count')
        .min(1)
        .max(300)
        .step(1)
        .onChange(() => updateFromOptions());
    meshFolder.add(params.mesh, 'speed').min(0).max(20);
    meshFolder.open();

    window.addEventListener('resize', () => {
        state.camera.aspect = window.innerWidth / window.innerHeight;
        state.camera.updateProjectionMatrix();
        state.renderer.setSize(window.innerWidth, window.innerHeight);
    });
}

function createShape(shapeType: ShapeType): Shape {
    switch (shapeType) {
        case 'torus-knot': {
            const geometry = new THREE.TorusKnotGeometry(1, 0.4, 40, 10);
            return threeGeometryToTriangleMesh(geometry);
        }
        case 'sphere':
            return sphere.create({ radius: 1 });
        case 'box':
            return box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        case 'capsule':
            return capsule.create({ halfHeightOfCylinder: 1.0, radius: 0.5 });
        default:
            return sphere.create({ radius: 1 });
    }
}

function threeGeometryToTriangleMesh(geometry: THREE.BufferGeometry): Shape {
    const positionAttribute = geometry.getAttribute('position');
    const positions = positionAttribute.array;
    const indexAttribute = geometry.index;

    const indices = indexAttribute ? Array.from(indexAttribute.array) : [];

    return triangleMesh.create({
        positions: Array.from(positions),
        indices,
    });
}

function createVisualMesh(shape: Shape): THREE.Object3D {
    const material = new THREE.MeshPhongMaterial({ color: 0xe91e63 });
    const helper = createShapeHelper(shape, { material });
    return helper.object;
}

function addMesh() {
    const shape = createShape(params.mesh.shapeType);
    const visualMesh = createVisualMesh(shape);

    // Random rotation speeds
    const rotationSpeed = vec3.fromValues(
        (Math.random() - 0.5) * 0.0001,
        (Math.random() - 0.5) * 0.0001,
        (Math.random() - 0.5) * 0.0001,
    );

    // Initial rotation as quaternion (source of truth)
    const quaternion = quat.create();
    quat.fromEuler(quaternion, euler.fromValues(Math.random() * 10, Math.random() * 10, 0, 'xyz'));

    // Position (will be set by updateFromOptions)
    const position = vec3.create();

    // Create body in physics world
    const body = rigidBody.create(state.world, {
        shape,
        motionType: MotionType.STATIC,
        position,
        quaternion,
        objectLayer: 0,
    });

    const meshObject: MeshObject = {
        visualMesh,
        body,
        position,
        quaternion,
        rotationSpeed,
    };

    state.meshObjects.push(meshObject);
    state.containerObj.add(visualMesh);
}

function addRaycaster() {
    state.rayCasterObjects.push(createRaycasterObject());
}

function updateFromOptions() {
    // update raycaster near/far
    state.rayNear = params.raycasters.near;
    state.rayFar = params.raycasters.far;

    // update raycaster count
    while (state.rayCasterObjects.length > params.raycasters.count) {
        const rc = state.rayCasterObjects.pop();
        if (rc) removeRaycasterObject(rc);
    }

    while (state.rayCasterObjects.length < params.raycasters.count) {
        addRaycaster();
    }

    // clear existing bodies and meshes
    while (state.meshObjects.length > 0) {
        const meshObj = state.meshObjects.pop()!;
        state.containerObj.remove(meshObj.visualMesh);
    }

    // recreate world with new shape
    initWorld();

    // update mesh count
    const oldLen = state.meshObjects.length;
    while (state.meshObjects.length < params.mesh.count) {
        addMesh();
    }

    // reposition meshes if count changed
    if (oldLen !== state.meshObjects.length) {
        const lerp = (a: number, b: number, t: number) => a + (b - a) * t;
        const lerpAmt = (state.meshObjects.length - 1) / (300 - 1);
        const dist = lerp(0, 2, lerpAmt);
        const scale = lerp(1, 0.2, lerpAmt);

        state.meshObjects.forEach((meshObj) => {
            meshObj.visualMesh.scale.set(1, 1, 1).multiplyScalar(scale);

            const vec = new THREE.Vector3(0, 1, 0);
            vec.applyAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * Math.random());
            vec.applyAxisAngle(new THREE.Vector3(0, 1, 0), 2 * Math.PI * Math.random());
            vec.multiplyScalar(dist);

            vec3.set(meshObj.position, vec.x, vec.y, vec.z);
        });
    }
}

init();

function render() {
    const now = window.performance.now();
    state.lastFrameTime = state.lastFrameTime || now;
    state.deltaTime = now - state.lastFrameTime;

    debugUI.beginPerf(state.ui);

    // rotate container
    state.containerObj.rotation.x += 0.0001 * params.mesh.speed * state.deltaTime;
    state.containerObj.rotation.y += 0.0001 * params.mesh.speed * state.deltaTime;
    state.containerObj.updateMatrixWorld();

    // Scratch vectors for world-space calculations
    const worldPos = new THREE.Vector3();
    const worldQuat = new THREE.Quaternion();
    const worldScale = new THREE.Vector3();
    const worldPosVec3 = vec3.create();
    const worldQuatVec4 = quat.create();

    // Update mesh objects (source of truth: mathcat position/quaternion)
    for (const meshObj of state.meshObjects) {
        // Update quaternion based on rotation speed
        quat.fromEuler(
            _deltaQuat,
            euler.fromValues(
                meshObj.rotationSpeed[0] * params.mesh.speed * state.deltaTime,
                meshObj.rotationSpeed[1] * params.mesh.speed * state.deltaTime,
                meshObj.rotationSpeed[2] * params.mesh.speed * state.deltaTime,
                'xyz',
            ),
        );
        quat.multiply(meshObj.quaternion, meshObj.quaternion, _deltaQuat);

        // Set Three.js mesh from mathcat quaternion (local space)
        meshObj.visualMesh.quaternion.set(
            meshObj.quaternion[0],
            meshObj.quaternion[1],
            meshObj.quaternion[2],
            meshObj.quaternion[3],
        );
        meshObj.visualMesh.position.set(meshObj.position[0], meshObj.position[1], meshObj.position[2]);

        // Calculate world-space transform for physics body
        meshObj.visualMesh.updateMatrixWorld();
        meshObj.visualMesh.matrixWorld.decompose(worldPos, worldQuat, worldScale);

        // Convert to mathcat types
        vec3.set(worldPosVec3, worldPos.x, worldPos.y, worldPos.z);
        quat.set(worldQuatVec4, worldQuat.x, worldQuat.y, worldQuat.z, worldQuat.w);

        // Update physics body transform
        rigidBody.setTransform(state.world, meshObj.body, worldPosVec3, worldQuatVec4, false);
    }

    // update raycasters
    for (const rc of state.rayCasterObjects) {
        updateRaycasterObject(rc);
    }

    debugUI.endPerf(state.ui);
    debugUI.updateStats(state.ui, state.world);

    // Update debug renderer
    debugRenderer.update(state.debugRendererState, state.world);

    state.renderer.render(state.scene, state.camera);

    state.lastFrameTime = now;

    requestAnimationFrame(render);
}

updateFromOptions();
render();
