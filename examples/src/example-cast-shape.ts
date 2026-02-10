import GUI from 'lil-gui';
import type { Quat, Vec3 } from 'mathcat';
import { euler, quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    castShape,
    CastShapeStatus,
    createClosestCastShapeCollector,
    createDefaultCastShapeSettings,
    createWorld,
    createWorldSettings,
    enableCollision,
    filter,
    MotionType,
    registerAll,
    type RigidBody,
    rigidBody,
    type Shape,
    sphere,
    triangleMesh,
    type World,
} from 'crashcat';
import { createShapeHelper, debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

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
    shapeCasterObjects: ShapeCasterObject[];
    shapeCollector: ReturnType<typeof createClosestCastShapeCollector>;
    shapeSettings: ReturnType<typeof createDefaultCastShapeSettings>;
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

const CAST_SHAPE_OPTIONS = {
    Sphere: 'sphere',
    Box: 'box',
    Capsule: 'capsule',
} as const;

type CastShapeType = (typeof CAST_SHAPE_OPTIONS)[keyof typeof CAST_SHAPE_OPTIONS];

const params = {
    shapecasters: {
        count: 10,
        speed: 1,
        shapeType: 'box' as CastShapeType,
    },
    mesh: {
        shapeType: 'torus-knot' as ShapeType,
        count: 1,
        speed: 1,
    },
};

type ShapeCasterObject = {
    rootObject: THREE.Object3D;
    originMesh: THREE.Mesh;
    hitMesh: THREE.Mesh;
    lineMesh: THREE.Mesh;
    quaternion: Quat;
    rotationSpeed: Vec3;
    position: Vec3;
    direction: Vec3;
    displacement: Vec3;
    castShape: Shape;
    castShapeScale: Vec3;
};

const _vector3 = new THREE.Vector3();
const _deltaQuat = quat.create();

function createCastShapeForType(shapeType: CastShapeType): Shape {
    switch (shapeType) {
        case 'sphere':
            return sphere.create({ radius: 0.1 });
        case 'box':
            return box.create({ halfExtents: vec3.fromValues(0.1, 0.1, 0.1) });
        case 'capsule':
            return capsule.create({ radius: 0.075, halfHeightOfCylinder: 0.1 });
    }
}

function createShapeVisualization(shapeType: CastShapeType): THREE.Mesh {
    let geometry: THREE.BufferGeometry;
    switch (shapeType) {
        case 'sphere':
            geometry = new THREE.SphereGeometry(0.1, 16, 16);
            break;
        case 'box':
            geometry = new THREE.BoxGeometry(0.2, 0.2, 0.2);
            break;
        case 'capsule':
            geometry = new THREE.CapsuleGeometry(0.075, 0.2, 8, 16);
            break;
    }
    const material = new THREE.MeshBasicMaterial({
        color: 0xffffff,
        transparent: true,
        opacity: 0.6,
    });
    return new THREE.Mesh(geometry, material);
}

function createShapeCasterObject(shapeType: CastShapeType): ShapeCasterObject {
    const cylinderGeometry = new THREE.CylinderGeometry(0.01, 0.01);
    const lineMaterial = new THREE.MeshBasicMaterial({
        color: 0xffffff,
        transparent: true,
        opacity: 0.25,
    });

    const rootObject = new THREE.Object3D();
    const originMesh = createShapeVisualization(shapeType);
    const hitMesh = createShapeVisualization(shapeType);
    hitMesh.scale.multiplyScalar(0.5);
    const lineMesh = new THREE.Mesh(cylinderGeometry, lineMaterial);

    rootObject.add(lineMesh);
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
        lineMesh,
        quaternion,
        rotationSpeed,
        position: vec3.create(),
        direction: vec3.create(),
        displacement: vec3.create(),
        castShape: createCastShapeForType(shapeType),
        castShapeScale: vec3.fromValues(1, 1, 1),
    };
}

function updateShapeCasterObject(sc: ShapeCasterObject) {
    // Update quaternion based on rotation speed
    quat.fromEuler(
        _deltaQuat,
        euler.fromValues(
            sc.rotationSpeed[0] * params.shapecasters.speed * state.deltaTime,
            sc.rotationSpeed[1] * params.shapecasters.speed * state.deltaTime,
            sc.rotationSpeed[2] * params.shapecasters.speed * state.deltaTime,
            'xyz',
        ),
    );
    quat.multiply(sc.quaternion, sc.quaternion, _deltaQuat);

    // Set Three.js rootObject from mathcat quaternion
    sc.rootObject.quaternion.set(sc.quaternion[0], sc.quaternion[1], sc.quaternion[2], sc.quaternion[3]);

    // Get origin position in world space
    sc.originMesh.updateMatrixWorld();
    _vector3.setFromMatrixPosition(sc.originMesh.matrixWorld);

    // Convert to vec3 for crashcat
    vec3.set(sc.position, _vector3.x, _vector3.y, _vector3.z);

    // Direction points toward center (negative of position)
    vec3.negate(sc.direction, sc.position);
    vec3.normalize(sc.direction, sc.direction);

    // Displacement is direction * pointDist
    vec3.scale(sc.displacement, sc.direction, pointDist);

    // Perform shape cast with crashcat
    state.shapeCollector.reset();
    castShape(
        state.world,
        state.shapeCollector,
        state.shapeSettings,
        sc.castShape,
        sc.position,
        sc.quaternion,
        sc.castShapeScale,
        sc.displacement,
        state.queryFilter,
    );

    let hitDistance = pointDist;
    const hadHit = state.shapeCollector.hit.status === CastShapeStatus.COLLIDING;
    if (hadHit) {
        hitDistance = state.shapeCollector.hit.fraction * pointDist;
    }

    // Update hit mesh position
    sc.hitMesh.position.set(pointDist - hitDistance, 0, 0);

    // Update line between origin and hit point
    const lineLength = hadHit ? hitDistance : pointDist;

    sc.lineMesh.position.set(pointDist - lineLength / 2, 0, 0);
    sc.lineMesh.scale.set(1, lineLength, 1);
    sc.lineMesh.rotation.z = Math.PI / 2;
}

function removeShapeCasterObject(sc: ShapeCasterObject) {
    state.scene.remove(sc.rootObject);
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
        shapeCasterObjects: [],
        shapeCollector: createClosestCastShapeCollector(),
        shapeSettings: createDefaultCastShapeSettings(),
        lastFrameTime: null,
        deltaTime: 0,
        debugRendererState,
        ui,
    };

    // Initialize physics world
    initWorld();

    // GUI
    const gui = new GUI();
    const scFolder = gui.addFolder('Shape Casters');
    scFolder
        .add(params.shapecasters, 'count')
        .min(1)
        .max(1000)
        .step(1)
        .onChange(() => updateFromOptions());
    scFolder.add(params.shapecasters, 'speed').min(0).max(20);
    scFolder.add(params.shapecasters, 'shapeType', CAST_SHAPE_OPTIONS).onChange(() => updateFromOptions());
    scFolder.open();

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

function addShapeCaster() {
    state.shapeCasterObjects.push(createShapeCasterObject(params.shapecasters.shapeType));
}

function updateFromOptions() {
    // update shapecaster count
    while (state.shapeCasterObjects.length > params.shapecasters.count) {
        const sc = state.shapeCasterObjects.pop();
        if (sc) removeShapeCasterObject(sc);
    }

    while (state.shapeCasterObjects.length < params.shapecasters.count) {
        addShapeCaster();
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

    // update shapecasters
    for (const sc of state.shapeCasterObjects) {
        updateShapeCasterObject(sc);
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
