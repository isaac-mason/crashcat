import GUI from 'lil-gui';
import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import {
    box,
    capsule,
    collideShapeVsShape,
    compound,
    convexHull,
    createAllCollideShapeCollector,
    createAnyCollideShapeCollector,
    createClosestCollideShapeCollector,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    sphere,
    transformed,
    triangleMesh,
    type CollideShapeCollector,
    type CollideShapeHit,
} from 'crashcat';
import { createShapeHelper } from 'crashcat/three';
import { createFaceGeometry } from './debug/face.js';
import { loadGLTFPoints } from './utils/gltf';

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
    Closest: 'closest',
    Any: 'any',
    All: 'all',
} as const;

type ShapeOption = (typeof GUI_SHAPE_OPTIONS)[keyof typeof GUI_SHAPE_OPTIONS];
type CollectorOption = (typeof GUI_COLLECTOR_OPTIONS)[keyof typeof GUI_COLLECTOR_OPTIONS];

const config: {
    shapeA: ShapeOption;
    shapeB: ShapeOption;
    collectorMode: CollectorOption;
    collectFaces: boolean;
} = { shapeA: 'sphere', shapeB: 'box', collectorMode: 'closest', collectFaces: true };

const gui = new GUI();
gui.add(config, 'shapeA', GUI_SHAPE_OPTIONS).name('Shape A Type').onChange(setup);
gui.add(config, 'shapeB', GUI_SHAPE_OPTIONS).name('Shape B Type').onChange(setup);
gui.add(config, 'collectorMode', GUI_COLLECTOR_OPTIONS).name('Collector Mode');
gui.add(config, 'collectFaces').name('Collect Faces');

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
let modeIndexA = 0;
let modeIndexB = 0;

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

const controlsA = new TransformControls(camera, renderer.domElement);
controlsA.setMode('translate');
scene.add(controlsA.getHelper());

const controlsB = new TransformControls(camera, renderer.domElement);
controlsB.setMode('translate');
scene.add(controlsB.getHelper());

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
                        quaternion: quat.fromDegrees(quat.create(), 0, 45, 0, 'zyx'),
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
    const shapeA = createTestShape(config.shapeA);
    const helperA = createShapeHelper(shapeA);
    const meshA = helperA.object as THREE.Mesh;
    meshA.position.set(-3, 0, 0);
    scene.add(meshA);
    controlsA.attach(meshA);

    const shapeB = createTestShape(config.shapeB);
    const helperB = createShapeHelper(shapeB);
    const meshB = helperB.object as THREE.Mesh;
    meshB.position.set(3, 0, 0);
    scene.add(meshB);
    controlsB.attach(meshB);

    const materialA = new THREE.MeshStandardMaterial({ color: 0x00ff00, transparent: true, opacity: 0.5 });
    const materialB = new THREE.MeshStandardMaterial({ color: 0x00ff00, transparent: true, opacity: 0.5 });
    (meshA as THREE.Mesh).material = materialA;
    (meshB as THREE.Mesh).material = materialB;

    const contactPointsA: THREE.Mesh[] = [];
    const contactPointsB: THREE.Mesh[] = [];
    const contactNormals: THREE.Line[] = [];

    for (let i = 0; i < 10; i++) {
        const pointGeometryA = new THREE.SphereGeometry(0.15, 16, 16);
        const pointMaterialA = new THREE.MeshBasicMaterial({ color: 0xffaa00 });
        const pointA = new THREE.Mesh(pointGeometryA, pointMaterialA);
        pointA.visible = false;
        scene.add(pointA);
        contactPointsA.push(pointA);

        const pointGeometryB = new THREE.SphereGeometry(0.15, 16, 16);
        const pointMaterialB = new THREE.MeshBasicMaterial({ color: 0xff00ff });
        const pointB = new THREE.Mesh(pointGeometryB, pointMaterialB);
        pointB.visible = false;
        scene.add(pointB);
        contactPointsB.push(pointB);
    }

    // Create face visualization line segments
    const faceAMaterial = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 2 });
    const faceBMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 2 });
    const faceALines = new THREE.LineSegments();
    faceALines.material = faceAMaterial;
    faceALines.visible = false;
    scene.add(faceALines);

    const faceBLines = new THREE.LineSegments();
    faceBLines.material = faceBMaterial;
    faceBLines.visible = false;
    scene.add(faceBLines);

    return {
        shapeA,
        shapeB,
        meshA,
        meshB,
        materialA,
        materialB,
        helperA,
        helperB,
        contactPointsA,
        contactPointsB,
        contactNormals,
        faceALines,
        faceBLines,
        lastLoggedState: 'unknown',
        collisionFlashCount: 0,
    };
}

type ExampleState = ReturnType<typeof init>;

function dispose(state: ExampleState) {
    scene.remove(state.meshA);
    scene.remove(state.meshB);
    controlsA.detach();
    controlsB.detach();

    state.helperA.dispose();
    state.helperB.dispose();
    state.materialA.dispose();
    state.materialB.dispose();

    for (const point of state.contactPointsA) {
        scene.remove(point);
        (point.geometry as THREE.BufferGeometry).dispose();
        (point.material as THREE.MeshBasicMaterial).dispose();
    }

    for (const point of state.contactPointsB) {
        scene.remove(point);
        (point.geometry as THREE.BufferGeometry).dispose();
        (point.material as THREE.MeshBasicMaterial).dispose();
    }

    for (const line of state.contactNormals) {
        scene.remove(line);
        (line.geometry as THREE.BufferGeometry).dispose();
        (line.material as THREE.LineBasicMaterial).dispose();
    }

    state.contactNormals.length = 0;

    scene.remove(state.faceALines);
    scene.remove(state.faceBLines);
    state.faceALines.geometry?.dispose();
    state.faceBLines.geometry?.dispose();
}

const collideShapeSettings = createDefaultCollideShapeSettings();

function update(state: ExampleState) {
    // Get current transforms from meshes
    const posA = vec3.fromValues(state.meshA.position.x, state.meshA.position.y, state.meshA.position.z);
    const quatA = quat.fromValues(
        state.meshA.quaternion.x,
        state.meshA.quaternion.y,
        state.meshA.quaternion.z,
        state.meshA.quaternion.w,
    );
    const scaleA = vec3.fromValues(1, 1, 1);

    const posB = vec3.fromValues(state.meshB.position.x, state.meshB.position.y, state.meshB.position.z);
    const quatB = quat.fromValues(
        state.meshB.quaternion.x,
        state.meshB.quaternion.y,
        state.meshB.quaternion.z,
        state.meshB.quaternion.w,
    );
    const scaleB = vec3.fromValues(1, 1, 1);

    // Update settings
    collideShapeSettings.collectFaces = config.collectFaces;

    // Set movement direction for active edge detection
    // For static collision, use relative position as heuristic
    vec3.subtract(collideShapeSettings.activeEdgeMovementDirection, posA, posB);

    // Distance between centers
    const dx = posB[0] - posA[0];
    const dy = posB[1] - posA[1];
    const dz = posB[2] - posA[2];
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

    // Create collector based on mode
    let collector: CollideShapeCollector;
    let hits: CollideShapeHit[] = [];

    if (config.collectorMode === 'closest') {
        const closestCollector = createClosestCollideShapeCollector();
        collector = closestCollector;

        collideShapeVsShape(
            collector,
            collideShapeSettings,
            state.shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0],
            posA[1],
            posA[2],
            quatA[0],
            quatA[1],
            quatA[2],
            quatA[3],
            scaleA[0],
            scaleA[1],
            scaleA[2],
            state.shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0],
            posB[1],
            posB[2],
            quatB[0],
            quatB[1],
            quatB[2],
            quatB[3],
            scaleB[0],
            scaleB[1],
            scaleB[2],
        );

        if (closestCollector.hit !== null) {
            hits = [closestCollector.hit];
        }
    } else if (config.collectorMode === 'any') {
        const anyCollector = createAnyCollideShapeCollector();
        collector = anyCollector;

        collideShapeVsShape(
            collector,
            collideShapeSettings,
            state.shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0],
            posA[1],
            posA[2],
            quatA[0],
            quatA[1],
            quatA[2],
            quatA[3],
            scaleA[0],
            scaleA[1],
            scaleA[2],
            state.shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0],
            posB[1],
            posB[2],
            quatB[0],
            quatB[1],
            quatB[2],
            quatB[3],
            scaleB[0],
            scaleB[1],
            scaleB[2],
        );

        if (anyCollector.hit !== null) {
            hits = [anyCollector.hit];
        }
    } else {
        const allCollector = createAllCollideShapeCollector();
        collector = allCollector;

        collideShapeVsShape(
            collector,
            collideShapeSettings,
            state.shapeA,
            EMPTY_SUB_SHAPE_ID,
            0,
            posA[0],
            posA[1],
            posA[2],
            quatA[0],
            quatA[1],
            quatA[2],
            quatA[3],
            scaleA[0],
            scaleA[1],
            scaleA[2],
            state.shapeB,
            EMPTY_SUB_SHAPE_ID,
            0,
            posB[0],
            posB[1],
            posB[2],
            quatB[0],
            quatB[1],
            quatB[2],
            quatB[3],
            scaleB[0],
            scaleB[1],
            scaleB[2],
        );

        hits = allCollector.hits;
        allCollector.reset();
    }

    // Visualize contacts
    for (const point of state.contactPointsA) {
        point.visible = false;
    }
    for (const point of state.contactPointsB) {
        point.visible = false;
    }
    for (const line of state.contactNormals) {
        scene.remove(line);
    }
    state.contactNormals.length = 0;

    for (let i = 0; i < hits.length; i++) {
        const hit = hits[i];

        if (i < state.contactPointsA.length) {
            const pointA = state.contactPointsA[i];
            pointA.position.set(hit.pointA[0], hit.pointA[1], hit.pointA[2]);
            pointA.visible = true;
        }

        if (i < state.contactPointsB.length) {
            const pointB = state.contactPointsB[i];
            pointB.position.set(hit.pointB[0], hit.pointB[1], hit.pointB[2]);
            pointB.visible = true;
        }

        const arrowLength = 0.8;

        // Visualize penetration axis (cyan - direction to move B out)
        const penetrationEnd = vec3.create();
        vec3.scaleAndAdd(penetrationEnd, hit.pointA, hit.penetrationAxis, arrowLength);
        const penetrationGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(hit.pointA[0], hit.pointA[1], hit.pointA[2]),
            new THREE.Vector3(penetrationEnd[0], penetrationEnd[1], penetrationEnd[2]),
        ]);
        const penetrationMaterial = new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 2 });
        const penetrationLine = new THREE.Line(penetrationGeometry, penetrationMaterial);
        scene.add(penetrationLine);
        state.contactNormals.push(penetrationLine);

        // Visualize contact normal (green - opposite of penetration axis)
        const normalEnd = vec3.create();
        vec3.scaleAndAdd(normalEnd, hit.pointA, hit.penetrationAxis, arrowLength);
        const normalGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(hit.pointA[0], hit.pointA[1], hit.pointA[2]),
            new THREE.Vector3(normalEnd[0], normalEnd[1], normalEnd[2]),
        ]);
        const normalMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 2 });
        const normalLine = new THREE.Line(normalGeometry, normalMaterial);
        scene.add(normalLine);
        state.contactNormals.push(normalLine);
    }

    // Visualize supporting faces (if collected and colliding)
    if (hits.length > 0 && config.collectFaces) {
        const firstHit = hits[0];

        if (firstHit.faceA && firstHit.faceA.numVertices > 0) {
            state.faceALines.geometry = createFaceGeometry(firstHit.faceA);
            state.faceALines.visible = true;
        } else {
            state.faceALines.visible = false;
        }

        if (firstHit.faceB && firstHit.faceB.numVertices > 0) {
            state.faceBLines.geometry = createFaceGeometry(firstHit.faceB);
            state.faceBLines.visible = true;
        } else {
            state.faceBLines.visible = false;
        }
    } else {
        state.faceALines.visible = false;
        state.faceBLines.visible = false;
    }

    // Update materials based on collision state
    const isColliding = hits.length > 0;
    const currentState = isColliding ? 'colliding' : 'separated';

    if (currentState !== state.lastLoggedState) {
        console.log(`[${dist.toFixed(2)}] State changed: ${state.lastLoggedState} -> ${currentState} (contacts: ${hits.length})`);
        state.lastLoggedState = currentState;
        if (isColliding && dist > 4) {
            state.collisionFlashCount++;
            console.log(
                `⚠️ FALSE POSITIVE #${state.collisionFlashCount}: Reporting collision at distance ${dist.toFixed(2)} with ${hits.length} contacts`,
            );
        }
    }

    const color = isColliding ? 0x00ff00 : 0xff0000;
    state.materialA.color.setHex(color);
    state.materialB.color.setHex(color);

    // Update info panel
    let infoHTML = `<b>collideShapeVsShape</b><br><br>`;
    infoHTML += `Mode: ${config.collectorMode}<br>`;
    infoHTML += `Colliding: <span style="color: ${isColliding ? '#00ff00' : '#ff0000'}">${isColliding ? 'YES' : 'NO'}</span><br>`;
    infoHTML += `Contacts: ${hits.length}<br>`;
    infoHTML += `Distance: ${dist.toFixed(3)}<br><br>`;

    for (let i = 0; i < Math.min(hits.length, 5); i++) {
        const hit = hits[i];
        infoHTML += `<b>Contact ${i}:</b><br>`;
        infoHTML += `Penetration: ${hit.penetration.toFixed(6)}<br>`;
        infoHTML += `<span style="color: #00ffff">Penetration Axis:</span> (${hit.penetrationAxis[0].toFixed(2)}, ${hit.penetrationAxis[1].toFixed(2)}, ${hit.penetrationAxis[2].toFixed(2)})<br>`;
        if (i < hits.length - 1) infoHTML += `<br>`;

        if (isColliding && dist > 4 && i === 0) {
            console.log(
                `  Contact points: A=(${hit.pointA[0].toFixed(2)}, ${hit.pointA[1].toFixed(2)}, ${hit.pointA[2].toFixed(2)})`,
            );
            console.log(
                `                  B=(${hit.pointB[0].toFixed(2)}, ${hit.pointB[1].toFixed(2)}, ${hit.pointB[2].toFixed(2)})`,
            );
            console.log(`  Distance between contact points: ${vec3.distance(hit.pointA, hit.pointB).toFixed(3)}`);
        }
    }

    if (hits.length > 5) {
        infoHTML += `<br>... and ${hits.length - 5} more contacts`;
    }

    // Add face extraction info
    if (config.collectFaces && hits.length > 0) {
        const firstHit = hits[0];
        infoHTML += `<br><br><b>Face Extraction:</b><br>`;
        infoHTML += `Face A vertices: ${firstHit.faceA?.numVertices ?? 0}<br>`;
        infoHTML += `Face B vertices: ${firstHit.faceB?.numVertices ?? 0}<br>`;
        infoHTML += `<span style="color: #ff0000">Red lines = Face A</span><br>`;
        infoHTML += `<span style="color: #00ff00">Green lines = Face B</span>`;
    }

    infoPanel.innerHTML = infoHTML;
}

function setup() {
    if (state) dispose(state);
    state = init();

    modeIndexA = 0;
    modeIndexB = 0;

    update(state);
}

/* Event handlers */

renderer.domElement.addEventListener('contextmenu', (event) => {
    event.preventDefault();

    if (!state) return;

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects([state.meshA, state.meshB]);

    if (intersects.length > 0) {
        const clickedMesh = intersects[0].object;

        if (clickedMesh === state.meshA) {
            modeIndexA = (modeIndexA + 1) % TRANSFORM_MODES.length;
            controlsA.setMode(TRANSFORM_MODES[modeIndexA]);
        } else if (clickedMesh === state.meshB) {
            modeIndexB = (modeIndexB + 1) % TRANSFORM_MODES.length;
            controlsB.setMode(TRANSFORM_MODES[modeIndexB]);
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
<b>collideShapeVsShape Example</b><br>
<br>
<b>Controls:</b><br>
Right-click meshes to toggle translate/rotate<br>
Drag meshes to move them around<br>
<br>
<b>Visualization:</b><br>
<span style="color: #00ff00;">Green</span>: Colliding<br>
<span style="color: #ff0000;">Red</span>: Not colliding<br>
<span style="color: #ffaa00;">Orange sphere</span>: Contact point on A<br>
<span style="color: #ff00ff;">Magenta sphere</span>: Contact point on B<br>
<span style="color: #00ffff;">Cyan line</span>: Penetration axis<br>
<span style="color: #00ff00;">Green line</span>: Contact normal<br>
<span style="color: #ff0000;">Red wireframe</span>: Supporting face A<br>
<span style="color: #00ff00;">Green wireframe</span>: Supporting face B
`;
document.body.appendChild(instructions);

setup();

controlsA.addEventListener('change', () => update(state));
controlsB.addEventListener('change', () => update(state));

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate() {
    requestAnimationFrame(animate);

    update(state);

    orbitControls.enabled = !controlsA.dragging && !controlsB.dragging;
    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
