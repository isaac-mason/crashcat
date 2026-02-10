import GUI from 'lil-gui';
import { quat, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import {
    box,
    capsule,
    CastShapeStatus,
    castShapeVsShape,
    compound,
    convexHull,
    createAllCastShapeCollector,
    createAnyCastShapeCollector,
    createClosestCastShapeCollector,
    createDefaultCastShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    type Face,
    registerAll,
    sphere,
    transformed,
    triangleMesh,
} from 'crashcat';
import { createShapeHelper } from 'crashcat/three';
import { createFaceGeometry } from './debug/face';
import { loadGLTFPoints } from './utils/gltf';

registerAll();

let state: ReturnType<typeof init>;

const GUI_SHAPE_OPTIONS = {
    Sphere: 'sphere',
    Box: 'box',
    Capsule: 'capsule',
    'Triangle Mesh': 'triangle-mesh',
    'Compound (Two Boxes)': 'compound-two-boxes',
    'Convex Hull (Suzanne)': 'convexhull',
    'Transformed (Rotated Box)': 'transformed-box',
} as const;

type ShapeOption = (typeof GUI_SHAPE_OPTIONS)[keyof typeof GUI_SHAPE_OPTIONS];

const COLLECTOR_MODE_OPTIONS = {
    Closest: 'closest',
    Any: 'any',
    All: 'all',
} as const;

type CollectorMode = (typeof COLLECTOR_MODE_OPTIONS)[keyof typeof COLLECTOR_MODE_OPTIONS];

const config: {
    shapeA: ShapeOption;
    shapeB: ShapeOption;
    collectorMode: CollectorMode;
    collideWithBackfaces: boolean;
} = { shapeA: 'sphere', shapeB: 'sphere', collectorMode: 'closest', collideWithBackfaces: false };

const gui = new GUI();
gui.add(config, 'shapeA', GUI_SHAPE_OPTIONS).name('Shape A Type').onChange(setup);
gui.add(config, 'shapeB', GUI_SHAPE_OPTIONS).name('Shape B Type').onChange(setup);
gui.add(config, 'collectorMode', COLLECTOR_MODE_OPTIONS).name('Collector Mode');
gui.add(config, 'collideWithBackfaces').name('Collide With Backfaces');

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(5, 5, 8);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const resize = () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
};

window.addEventListener('resize', resize);
resize();

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* setup example */

const TRANSFORM_CONTROLS_MODES: ('translate' | 'rotate')[] = ['translate', 'rotate'];
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

const controlsDisplacement = new TransformControls(camera, renderer.domElement);
controlsDisplacement.setMode('translate');
scene.add(controlsDisplacement.getHelper());

renderer.domElement.addEventListener('contextmenu', (event: MouseEvent) => {
    event.preventDefault();

    if (!state) return;

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects([state.meshShapeA, state.meshShapeB]);

    if (intersects.length > 0) {
        const clickedMesh = intersects[0].object;
        if (clickedMesh === state.meshShapeA) {
            modeIndexA = (modeIndexA + 1) % TRANSFORM_CONTROLS_MODES.length;
            controlsA.setMode(TRANSFORM_CONTROLS_MODES[modeIndexA]);
        } else if (clickedMesh === state.meshShapeB) {
            modeIndexB = (modeIndexB + 1) % TRANSFORM_CONTROLS_MODES.length;
            controlsB.setMode(TRANSFORM_CONTROLS_MODES[modeIndexB]);
        }
    }
});

const materialShapeA = new THREE.MeshStandardMaterial({ color: 0x4488ff });
const materialShapeB = new THREE.MeshStandardMaterial({ color: 0xff8844 });

const displacementEndpointGeometry = new THREE.SphereGeometry(0.2, 16, 16);
const displacementEndpointMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
const displacementEndpointMesh = new THREE.Mesh(displacementEndpointGeometry, displacementEndpointMaterial);
scene.add(displacementEndpointMesh);

const displacementLineGeometry = new THREE.BufferGeometry();
const displacementLineMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 2 });
const displacementLineMesh = new THREE.Line(displacementLineGeometry, displacementLineMaterial);
scene.add(displacementLineMesh);
controlsDisplacement.attach(displacementEndpointMesh);

const infoPanel = document.createElement('div');
infoPanel.style.position = 'absolute';
infoPanel.style.top = '10px';
infoPanel.style.left = '10px';
infoPanel.style.color = 'white';
infoPanel.style.fontFamily = 'monospace';
infoPanel.style.fontSize = '12px';
infoPanel.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
infoPanel.style.padding = '10px';
infoPanel.style.borderRadius = '5px';
infoPanel.style.minWidth = '250px';
document.body.appendChild(infoPanel);

const gltfLoader = new GLTFLoader();
// const gltf = await gltfLoader.loadAsync('./models/bunny.glb');
// const gltfMesh = gltf.scene.children[0].children[0] as THREE.Mesh;
const gltf = await gltfLoader.loadAsync('./models/suzi.glb');
const gltfMesh = gltf.scene.children[0] as THREE.Mesh;
const triangleMeshGeometry = gltfMesh.geometry as THREE.BufferGeometry;

const suzannePoints = await loadGLTFPoints('./models/suzi.glb');

function setup() {
    if (state) dispose(state);
    state = init(config.shapeA, config.shapeB);
    update(state);
}

setup();

controlsDisplacement.addEventListener('change', () => update(state));
controlsA.addEventListener('change', () => update(state));
controlsB.addEventListener('change', () => update(state));

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
                ]
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

function init(shapeTypeA: ShapeOption, shapeTypeB: ShapeOption) {
    const collisionShapeA = createTestShape(shapeTypeA);
    const collisionShapeB = createTestShape(shapeTypeB);

    const shapeAHelper = createShapeHelper(collisionShapeA);
    const meshShapeA = shapeAHelper.object;
    meshShapeA.position.set(-3, 0, 0);
    scene.add(meshShapeA);

    const shapeBHelper = createShapeHelper(collisionShapeB);
    const meshShapeB = shapeBHelper.object;
    meshShapeB.position.set(3, 0, 0);
    scene.add(meshShapeB);

    controlsA.attach(meshShapeA);
    controlsB.attach(meshShapeB);

    const hitMaterial = new THREE.MeshStandardMaterial({
        color: 0xffff00,
        transparent: true,
        opacity: 0.3,
        wireframe: false,
    });
    const hitShapeHelper = createShapeHelper(collisionShapeA, { material: hitMaterial });
    const hitMesh = hitShapeHelper.object;
    hitMesh.visible = false;
    scene.add(hitMesh);

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

    displacementEndpointMesh.position.copy(meshShapeA.position);
    displacementEndpointMesh.position.x += 10;

    return {
        collisionShapeA,
        collisionShapeB,
        meshShapeA,
        meshShapeB,
        shapeAHelper,
        shapeBHelper,
        hitShapeHelper,
        faceALines,
        faceBLines,
        penetrationLines: [] as THREE.Line[],
        lastHitState: false,
    };
}

function dispose(state: ExampleState) {
    scene.remove(state.meshShapeA);
    scene.remove(state.meshShapeB);
    scene.remove(state.hitShapeHelper.object);
    scene.remove(state.faceALines);
    scene.remove(state.faceBLines);
    for (const line of state.penetrationLines) {
        scene.remove(line);
        line.geometry?.dispose();
    }
    controlsA.detach();
    controlsB.detach();
    state.shapeAHelper.dispose();
    state.shapeBHelper.dispose();
    state.hitShapeHelper.dispose();
    state.faceALines.geometry?.dispose();
    state.faceBLines.geometry?.dispose();
}

type ExampleState = ReturnType<typeof init>;

function update(state: ExampleState) {
    // update displacement visualization
    const startPos = state.meshShapeA.position;
    const endPos = displacementEndpointMesh.position;
    const displacement = vec3.create();
    vec3.subtract(displacement, [endPos.x, endPos.y, endPos.z], [startPos.x, startPos.y, startPos.z]);

    const points = [new THREE.Vector3(startPos.x, startPos.y, startPos.z), new THREE.Vector3(endPos.x, endPos.y, endPos.z)];
    displacementLineGeometry.dispose();
    displacementLineGeometry.copy(new THREE.BufferGeometry().setFromPoints(points));

    // shape cast
    const posA = vec3.fromValues(state.meshShapeA.position.x, state.meshShapeA.position.y, state.meshShapeA.position.z);
    const quatA = quat.fromValues(
        state.meshShapeA.quaternion.x,
        state.meshShapeA.quaternion.y,
        state.meshShapeA.quaternion.z,
        state.meshShapeA.quaternion.w,
    );
    const scaleA = vec3.fromValues(1, 1, 1);

    const posB = vec3.fromValues(state.meshShapeB.position.x, state.meshShapeB.position.y, state.meshShapeB.position.z);
    const quatB = quat.fromValues(
        state.meshShapeB.quaternion.x,
        state.meshShapeB.quaternion.y,
        state.meshShapeB.quaternion.z,
        state.meshShapeB.quaternion.w,
    );
    const scaleB = vec3.fromValues(1, 1, 1);

    type CastShapeResult = {
        hit: boolean;
        fraction: number;
        hitPosition: Vec3;
        penetrationAxis: Vec3;
        normal: Vec3;
        faceA?: Face;
        faceB?: Face;
    };

    let castResult: CastShapeResult | undefined;

    const settings = createDefaultCastShapeSettings();
    settings.collectFaces = true;
    settings.collideWithBackfaces = config.collideWithBackfaces;

    // Set movement direction for active edge detection
    // Use the cast displacement as the movement direction
    vec3.copy(settings.activeEdgeMovementDirection, displacement);

    if (config.collectorMode === 'closest') {
        const closestCollector = createClosestCastShapeCollector();
        castShapeVsShape(
            closestCollector,
            settings,
            state.collisionShapeA,
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
            displacement[0],
            displacement[1],
            displacement[2],
            state.collisionShapeB,
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

        const hit = closestCollector.hit;
        if (hit.status === CastShapeStatus.COLLIDING) {
            const hitPos = vec3.create();
            vec3.scaleAndAdd(hitPos, posA, displacement, hit.fraction);
            castResult = {
                hit: true,
                fraction: hit.fraction,
                hitPosition: [hitPos[0], hitPos[1], hitPos[2]],
                penetrationAxis: vec3.clone(hit.penetrationAxis),
                normal: vec3.clone(hit.normal),
                faceA: hit.faceA,
                faceB: hit.faceB,
            };
        }
    } else if (config.collectorMode === 'any') {
        const anyCollector = createAnyCastShapeCollector();
        castShapeVsShape(
            anyCollector,
            settings,
            state.collisionShapeA,
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
            displacement[0],
            displacement[1],
            displacement[2],
            state.collisionShapeB,
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

        const hit = anyCollector.hit;
        if (hit.status === CastShapeStatus.COLLIDING) {
            const hitPos = vec3.create();
            vec3.scaleAndAdd(hitPos, posA, displacement, hit.fraction);
            castResult = {
                hit: true,
                fraction: hit.fraction,
                hitPosition: [hitPos[0], hitPos[1], hitPos[2]],
                penetrationAxis: vec3.clone(hit.penetrationAxis),
                normal: vec3.clone(hit.normal),
                faceA: hit.faceA,
                faceB: hit.faceB,
            };
        }
    } else {
        const allCollector = createAllCastShapeCollector();
        castShapeVsShape(
            allCollector,
            settings,
            state.collisionShapeA,
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
            displacement[0],
            displacement[1],
            displacement[2],
            state.collisionShapeB,
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

        // For 'all' mode, use the first hit (closest) for visualization
        // In a real application, you might want to visualize all hits
        const hits = allCollector.hits;
        if (hits.length > 0) {
            const hit = hits[0];
            if (hit.status === CastShapeStatus.COLLIDING) {
                const hitPos = vec3.create();
                vec3.scaleAndAdd(hitPos, posA, displacement, hit.fraction);
                castResult = {
                    hit: true,
                    fraction: hit.fraction,
                    hitPosition: [hitPos[0], hitPos[1], hitPos[2]],
                    penetrationAxis: vec3.clone(hit.penetrationAxis),
                    normal: vec3.clone(hit.normal),
                    faceA: hit.faceA,
                    faceB: hit.faceB,
                };
            }
        }
    }

    // Clean up previous penetration lines
    for (const line of state.penetrationLines) {
        scene.remove(line);
        line.geometry?.dispose();
    }
    state.penetrationLines = [];

    // visualize
    const hitObj = state.hitShapeHelper.object;

    if (castResult?.hit) {
        if (!state.lastHitState) {
            materialShapeA.color.setHex(0x00ff00);
            materialShapeB.color.setHex(0x00ff00);
            state.lastHitState = true;
        }

        hitObj.position.set(castResult.hitPosition[0], castResult.hitPosition[1], castResult.hitPosition[2]);
        hitObj.quaternion.copy(state.meshShapeA.quaternion);
        hitObj.visible = true;

        if (castResult.faceA && castResult.faceA.numVertices > 0) {
            state.faceALines.geometry = createFaceGeometry(castResult.faceA);
            state.faceALines.visible = true;
        } else {
            state.faceALines.visible = false;
        }

        if (castResult.faceB && castResult.faceB.numVertices > 0) {
            state.faceBLines.geometry = createFaceGeometry(castResult.faceB);
            state.faceBLines.visible = true;
        } else {
            state.faceBLines.visible = false;
        }

        // Visualize penetration axis (cyan - direction to move B out)
        const arrowLength = 1.5;
        const penetrationEnd = vec3.create();
        vec3.scaleAndAdd(penetrationEnd, castResult.hitPosition, castResult.penetrationAxis, arrowLength);
        const penetrationGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(castResult.hitPosition[0], castResult.hitPosition[1], castResult.hitPosition[2]),
            new THREE.Vector3(penetrationEnd[0], penetrationEnd[1], penetrationEnd[2]),
        ]);
        const penetrationMaterial = new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 2 });
        const penetrationLine = new THREE.Line(penetrationGeometry, penetrationMaterial);
        scene.add(penetrationLine);
        state.penetrationLines.push(penetrationLine);

        // Visualize contact normal (green - opposite of penetration axis)
        const normalEnd = vec3.create();
        vec3.scaleAndAdd(normalEnd, castResult.hitPosition, castResult.normal, arrowLength);
        const normalGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(castResult.hitPosition[0], castResult.hitPosition[1], castResult.hitPosition[2]),
            new THREE.Vector3(normalEnd[0], normalEnd[1], normalEnd[2]),
        ]);
        const normalMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 2 });
        const normalLine = new THREE.Line(normalGeometry, normalMaterial);
        scene.add(normalLine);
        state.penetrationLines.push(normalLine);
    } else {
        if (state.lastHitState) {
            materialShapeA.color.setHex(0x4488ff);
            materialShapeB.color.setHex(0xff8844);
            state.lastHitState = false;
        }

        hitObj.visible = false;
        state.faceALines.visible = false;
        state.faceBLines.visible = false;
    }

    // update info panel
    let infoHTML = `<b>Continuous Collision Detection</b><br><br>`;
    infoHTML += `<b>Shape A:</b> ${config.shapeA}<br>`;
    infoHTML += `Position: (${state.meshShapeA.position.x.toFixed(3)}, ${state.meshShapeA.position.y.toFixed(3)}, ${state.meshShapeA.position.z.toFixed(3)})<br><br>`;

    infoHTML += `<b>Shape B:</b> ${config.shapeB}<br>`;
    infoHTML += `Position: (${state.meshShapeB.position.x.toFixed(3)}, ${state.meshShapeB.position.y.toFixed(3)}, ${state.meshShapeB.position.z.toFixed(3)})<br><br>`;

    infoHTML += `<b>Displacement:</b><br>`;
    infoHTML += `X: ${displacement[0].toFixed(3)}<br>`;
    infoHTML += `Y: ${displacement[1].toFixed(3)}<br>`;
    infoHTML += `Z: ${displacement[2].toFixed(3)}<br>`;
    infoHTML += `Length: ${vec3.length(displacement).toFixed(3)}<br><br>`;

    if (castResult?.hit) {
        infoHTML += `<b>🎯 COLLISION DETECTED</b><br>`;
        infoHTML += `Collision Fraction: ${castResult.fraction.toFixed(3)}<br>`;
        infoHTML += `(0 = overlapping now, 1 = at end of displacement)<br><br>`;
        infoHTML += `<b>Hit Position:</b><br>`;
        infoHTML += `(${castResult.hitPosition[0].toFixed(3)}, ${castResult.hitPosition[1].toFixed(3)}, ${castResult.hitPosition[2].toFixed(3)})<br><br>`;
        infoHTML += `<span style="color: #00ffff">Penetration Axis:</span> (${castResult.penetrationAxis[0].toFixed(2)}, ${castResult.penetrationAxis[1].toFixed(2)}, ${castResult.penetrationAxis[2].toFixed(2)})<br>`;
        infoHTML += `<span style="color: #00ff00">Normal:</span> (${castResult.normal[0].toFixed(2)}, ${castResult.normal[1].toFixed(2)}, ${castResult.normal[2].toFixed(2)})<br><br>`;
        infoHTML += `<b>Face Extraction:</b><br>`;
        infoHTML += `Face A vertices: ${castResult.faceA?.numVertices ?? 0}<br>`;
        infoHTML += `Face B vertices: ${castResult.faceB?.numVertices ?? 0}<br>`;
        infoHTML += `<span style="color: #ff0000">Red lines = Face A</span><br>`;
        infoHTML += `<span style="color: #00ff00">Green lines = Face B</span>`;
    } else {
        infoHTML += `<b>❌ No collision</b><br>`;
    }

    infoPanel.innerHTML = infoHTML;
}

function loop() {
    requestAnimationFrame(loop);

    const anyControlsDragging = controlsA.dragging || controlsB.dragging || controlsDisplacement.dragging;
    orbitControls.enabled = !anyControlsDragging;

    orbitControls.update();
    renderer.render(scene, camera);
}

loop();
