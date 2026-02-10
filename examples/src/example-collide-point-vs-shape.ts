import GUI from 'lil-gui';
import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import {
    box,
    capsule,
    collidePointVsShape,
    compound,
    convexHull,
    createAllCollidePointCollector,
    createAnyCollidePointCollector,
    createDefaultCollidePointSettings,
    EMPTY_SUB_SHAPE_ID,
    registerAll,
    sphere,
    transformed,
    triangleMesh,
    type CollidePointCollector,
    type CollidePointHit,
} from 'crashcat';
import { createShapeHelper } from 'crashcat/three';
import { loadGLTFPoints } from './utils/gltf';

registerAll();

const GUI_SHAPE_OPTIONS = {
    Sphere: 'sphere',
    Box: 'box',
    Capsule: 'capsule',
    'Triangle Mesh': 'triangle-mesh',
    'Compound (Two Boxes)': 'compound-two-boxes',
    'Convex Hull (Suzanne)': 'convexhull',
    'Transformed (Rotated Box)': 'transformed-box',
} as const;

const GUI_COLLECTOR_OPTIONS = {
    Any: 'any',
    All: 'all',
} as const;

type ShapeOption = (typeof GUI_SHAPE_OPTIONS)[keyof typeof GUI_SHAPE_OPTIONS];
type CollectorOption = (typeof GUI_COLLECTOR_OPTIONS)[keyof typeof GUI_COLLECTOR_OPTIONS];

const config: {
    shape: ShapeOption;
    collectorMode: CollectorOption;
} = { shape: 'box', collectorMode: 'any' };

const gui = new GUI();
gui.add(config, 'shape', GUI_SHAPE_OPTIONS).name('Shape Type').onChange(setup);
gui.add(config, 'collectorMode', GUI_COLLECTOR_OPTIONS).name('Collector Mode');

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(5, 5, 5);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* Setup and state management */

const TRANSFORM_MODES: ('translate' | 'rotate')[] = ['translate', 'rotate'];
let modeIndexShape = 0;
let modeIndexPoint = 0;

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

const controlsShape = new TransformControls(camera, renderer.domElement);
controlsShape.setMode('translate');
scene.add(controlsShape.getHelper());

const controlsPoint = new TransformControls(camera, renderer.domElement);
controlsPoint.setMode('translate');
scene.add(controlsPoint.getHelper());

let state: ReturnType<typeof init>;

const infoPanel = document.createElement('div');
infoPanel.style.position = 'absolute';
infoPanel.style.top = '10px';
infoPanel.style.right = '10px';
infoPanel.style.color = 'white';
infoPanel.style.fontFamily = 'monospace';
infoPanel.style.fontSize = '14px';
infoPanel.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
infoPanel.style.padding = '10px';
infoPanel.style.borderRadius = '5px';
infoPanel.style.minWidth = '200px';
document.body.appendChild(infoPanel);

const gltfLoader = new GLTFLoader();
const gltf = await gltfLoader.loadAsync('./models/bunny.glb');
const gltfMesh = gltf.scene.children[0].children[0] as THREE.Mesh;
const triangleMeshGeometry = gltfMesh.geometry as THREE.BufferGeometry;

const suzannePoints = await loadGLTFPoints('./models/suzi.glb');

function createTestShape(type: ShapeOption) {
    switch (type) {
        case 'sphere':
            return sphere.create({ radius: 1.0 });
        case 'box':
            return box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
        case 'capsule':
            return capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
        case 'triangle-mesh': {
            const positions = triangleMeshGeometry.getAttribute('position').array!;
            const indices = triangleMeshGeometry.getIndex()!.array;
            return triangleMesh.create({
                positions: Array.from(positions),
                indices: Array.from(indices),
            });
        }
        case 'compound-two-boxes': {
            return compound.create({
                children: [
                    {
                        shape: box.create({ halfExtents: vec3.fromValues(0.8, 0.5, 0.8) }),
                        position: vec3.fromValues(-1.2, 0.8, 0),
                        quaternion: quat.fromDegrees(quat.create(), 0, 45, 0, 'zyx'),
                    },
                    {
                        shape: box.create({ halfExtents: vec3.fromValues(0.6, 0.7, 0.6) }),
                        position: vec3.fromValues(1.0, -0.6, 0),
                        quaternion: quat.fromDegrees(quat.create(), 30, -30, 15, 'zyx'),
                    }
                ],
            });
        }
        case 'convexhull':
            return convexHull.create({ positions: suzannePoints, convexRadius: 0.0 });
        case 'transformed-box':
            return transformed.create({
                shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
                position: vec3.fromValues(0, 0, 0),
                quaternion: quat.fromDegrees(quat.create(), 25, 30, 45, 'zyx'),
            });
        default:
            return sphere.create({ radius: 1.0 });
    }
}

function init() {
    const shape = createTestShape(config.shape);
    const helper = createShapeHelper(shape);
    const meshShape = helper.object as THREE.Mesh;
    meshShape.position.set(2, 0, 0);
    scene.add(meshShape);
    controlsShape.attach(meshShape);

    const materialShape = new THREE.MeshStandardMaterial({ color: 0x00ff00, transparent: true, opacity: 0.5 });
    (meshShape as THREE.Mesh).material = materialShape;

    // Create point gizmo
    const pointGeometry = new THREE.SphereGeometry(0.2, 16, 16);
    const pointMaterial = new THREE.MeshBasicMaterial({ color: 0xffaa00 });
    const pointGizmo = new THREE.Mesh(pointGeometry, pointMaterial);
    pointGizmo.position.set(-2, 0, 0);
    scene.add(pointGizmo);
    controlsPoint.attach(pointGizmo);

    return {
        shape,
        meshShape,
        materialShape,
        helper,
        pointGizmo,
        lastLoggedState: 'unknown',
    };
}

type ExampleState = ReturnType<typeof init>;

function dispose(state: ExampleState) {
    scene.remove(state.meshShape);
    scene.remove(state.pointGizmo);
    controlsShape.detach();
    controlsPoint.detach();

    state.helper.dispose();
    state.materialShape.dispose();
    (state.pointGizmo.geometry as THREE.BufferGeometry).dispose();
    (state.pointGizmo.material as THREE.MeshBasicMaterial).dispose();
}

const collidePointSettings = createDefaultCollidePointSettings();

function update(state: ExampleState) {
    // Get current transforms from meshes
    const point = vec3.fromValues(state.pointGizmo.position.x, state.pointGizmo.position.y, state.pointGizmo.position.z);

    const posShape = vec3.fromValues(state.meshShape.position.x, state.meshShape.position.y, state.meshShape.position.z);
    const quatShape = quat.fromValues(
        state.meshShape.quaternion.x,
        state.meshShape.quaternion.y,
        state.meshShape.quaternion.z,
        state.meshShape.quaternion.w,
    );
    const scaleShape = vec3.fromValues(1, 1, 1);

    // Distance between point and shape center
    const dx = posShape[0] - point[0];
    const dy = posShape[1] - point[1];
    const dz = posShape[2] - point[2];
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // Create collector based on mode
    let collector: CollidePointCollector;
    let hits: CollidePointHit[] = [];

    if (config.collectorMode === 'any') {
        const anyCollector = createAnyCollidePointCollector();
        collector = anyCollector;

        collidePointVsShape(
            collector,
            collidePointSettings,
            point[0],
            point[1],
            point[2],
            state.shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            posShape[0],
            posShape[1],
            posShape[2],
            quatShape[0],
            quatShape[1],
            quatShape[2],
            quatShape[3],
            scaleShape[0],
            scaleShape[1],
            scaleShape[2],
        );

        if (anyCollector.hit !== null) {
            hits = [anyCollector.hit];
        }
    } else {
        const allCollector = createAllCollidePointCollector();
        collector = allCollector;

        collidePointVsShape(
            collector,
            collidePointSettings,
            point[0],
            point[1],
            point[2],
            state.shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            posShape[0],
            posShape[1],
            posShape[2],
            quatShape[0],
            quatShape[1],
            quatShape[2],
            quatShape[3],
            scaleShape[0],
            scaleShape[1],
            scaleShape[2],
        );

        hits = allCollector.hits;
        allCollector.reset();
    }

    // Update materials based on collision state
    const isColliding = hits.length > 0;
    const currentState = isColliding ? 'colliding' : 'separated';

    if (currentState !== state.lastLoggedState) {
        console.log(`[${dist.toFixed(2)}] State changed: ${state.lastLoggedState} -> ${currentState} (hits: ${hits.length})`);
        state.lastLoggedState = currentState;
    }

    const color = isColliding ? 0x00ff00 : 0xff0000;
    state.materialShape.color.setHex(color);
    (state.pointGizmo.material as THREE.MeshBasicMaterial).color.setHex(isColliding ? 0x00ff00 : 0xffaa00);

    // Update info panel
    let infoHTML = `<b>collidePointVsShape</b><br><br>`;
    infoHTML += `Mode: ${config.collectorMode}<br>`;
    infoHTML += `Colliding: <span style="color: ${isColliding ? '#00ff00' : '#ff0000'}">${isColliding ? 'YES' : 'NO'}</span><br>`;
    infoHTML += `Hits: ${hits.length}<br>`;
    infoHTML += `Distance: ${dist.toFixed(3)}<br><br>`;

    infoHTML += `<b>Point:</b><br>`;
    infoHTML += `(${point[0].toFixed(2)}, ${point[1].toFixed(2)}, ${point[2].toFixed(2)})<br><br>`;

    for (let i = 0; i < Math.min(hits.length, 5); i++) {
        const hit = hits[i];
        infoHTML += `<b>Hit ${i}:</b><br>`;
        infoHTML += `SubShape ID: ${hit.subShapeIdB}<br>`;
        infoHTML += `Body ID: ${hit.bodyIdB}<br>`;
        if (i < hits.length - 1) infoHTML += `<br>`;
    }

    if (hits.length > 5) {
        infoHTML += `<br>... and ${hits.length - 5} more hits`;
    }

    infoPanel.innerHTML = infoHTML;
}

function setup() {
    if (state) dispose(state);
    state = init();

    modeIndexShape = 0;
    modeIndexPoint = 0;

    update(state);
}

/* Event handlers */

renderer.domElement.addEventListener('contextmenu', (event) => {
    event.preventDefault();

    if (!state) return;

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects([state.meshShape, state.pointGizmo]);

    if (intersects.length > 0) {
        const clickedMesh = intersects[0].object;

        if (clickedMesh === state.meshShape) {
            modeIndexShape = (modeIndexShape + 1) % TRANSFORM_MODES.length;
            controlsShape.setMode(TRANSFORM_MODES[modeIndexShape]);
        } else if (clickedMesh === state.pointGizmo) {
            modeIndexPoint = (modeIndexPoint + 1) % TRANSFORM_MODES.length;
            controlsPoint.setMode(TRANSFORM_MODES[modeIndexPoint]);
        }
    }
});

/* Instructions */

const instructions = document.createElement('div');
instructions.style.position = 'absolute';
instructions.style.top = '10px';
instructions.style.left = '10px';
instructions.style.color = 'white';
instructions.style.fontFamily = 'monospace';
instructions.style.fontSize = '14px';
instructions.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
instructions.style.padding = '10px';
instructions.style.borderRadius = '5px';
instructions.innerHTML = `
<b>collidePointVsShape Example</b><br>
<br>
<b>Controls:</b><br>
Right-click gizmos to toggle translate/rotate<br>
Drag gizmos to move them around<br>
<br>
<b>Visualization:</b><br>
<span style="color: #00ff00;">Green</span>: Colliding<br>
<span style="color: #ff0000;">Red</span>: Not colliding<br>
<span style="color: #ffaa00;">Orange sphere</span>: Point (not colliding)<br>
<span style="color: #00ff00;">Green sphere</span>: Point (colliding)
`;
document.body.appendChild(instructions);

setup();

controlsShape.addEventListener('change', () => update(state));
controlsPoint.addEventListener('change', () => update(state));

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate() {
    requestAnimationFrame(animate);

    update(state);

    orbitControls.enabled = !controlsShape.dragging && !controlsPoint.dragging;
    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
