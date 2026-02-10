import GUI from 'lil-gui';
import { quat, vec3 } from 'mathcat';
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import {
    box,
    CastRayStatus,
    capsule,
    castRayVsShape,
    compound,
    convexHull,
    createAllCastRayCollector,
    createAnyCastRayCollector,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    EMPTY_SUB_SHAPE_ID,
    getShapeSurfaceNormal,
    sphere,
    transformed,
    triangleMesh,
    registerAll,
} from 'crashcat';
import { createShapeHelper } from 'crashcat/three';
import { loadGLTFPoints } from './utils/gltf.js';

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
    shapeType: ShapeOption;
    collectorMode: CollectorMode;
} = { shapeType: 'sphere', collectorMode: 'all' };

const gui = new GUI();
gui.add(config, 'shapeType', GUI_SHAPE_OPTIONS).name('Shape Type').onChange(setup);
gui.add(config, 'collectorMode', COLLECTOR_MODE_OPTIONS).name('Collector Mode');

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
let modeIndexShape = 0;
let modeIndexRayOrigin = 0;
let modeIndexRayEndpoint = 0;

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

const materialShape = new THREE.MeshStandardMaterial({ color: 0x4488ff });
const controlsShape = new TransformControls(camera, renderer.domElement);
controlsShape.setMode('translate');
scene.add(controlsShape.getHelper());

const controlsRayOrigin = new TransformControls(camera, renderer.domElement);
controlsRayOrigin.setMode('translate');
scene.add(controlsRayOrigin.getHelper());

const controlsRayEndpoint = new TransformControls(camera, renderer.domElement);
controlsRayEndpoint.setMode('translate');
scene.add(controlsRayEndpoint.getHelper());

const rayLineGeometry = new THREE.BufferGeometry();
const rayLineMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 2 });
const rayLineMesh = new THREE.Line(rayLineGeometry, rayLineMaterial);
scene.add(rayLineMesh);

const hitPointGeometry = new THREE.SphereGeometry(0.15, 16, 16);
const hitPointMeshes: THREE.Mesh[] = [];
const normalArrows: THREE.ArrowHelper[] = [];

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
infoPanel.style.minWidth = '200px';
document.body.appendChild(infoPanel);

renderer.domElement.addEventListener('contextmenu', (event) => {
    event.preventDefault();

    if (!state) return;

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects([state.meshShape, state.rayOriginMesh, state.rayEndpointMesh]);

    if (intersects.length > 0) {
        const clickedMesh = intersects[0].object;

        if (clickedMesh === state.meshShape) {
            modeIndexShape = (modeIndexShape + 1) % TRANSFORM_CONTROLS_MODES.length;
            controlsShape.setMode(TRANSFORM_CONTROLS_MODES[modeIndexShape]);
        } else if (clickedMesh === state.rayOriginMesh) {
            modeIndexRayOrigin = (modeIndexRayOrigin + 1) % TRANSFORM_CONTROLS_MODES.length;
            controlsRayOrigin.setMode(TRANSFORM_CONTROLS_MODES[modeIndexRayOrigin]);
        } else if (clickedMesh === state.rayEndpointMesh) {
            modeIndexRayEndpoint = (modeIndexRayEndpoint + 1) % TRANSFORM_CONTROLS_MODES.length;
            controlsRayEndpoint.setMode(TRANSFORM_CONTROLS_MODES[modeIndexRayEndpoint]);
        }
    }
});

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
                        quaternion: quat.fromDegrees(quat.create(), 15, 45, -10, 'zyx'),
                    },
                    {
                        shape: box.create({ halfExtents: vec3.fromValues(0.6, 0.7, 0.6) }),
                        position: vec3.fromValues(1.0, -0.6, 0),
                        quaternion: quat.fromDegrees(quat.create(), -20, 30, 25, 'zyx'),
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
    const collisionShape = createTestShape(config.shapeType);
    const shapeHelper = createShapeHelper(collisionShape);
    const meshShape = shapeHelper.object as THREE.Mesh;
    meshShape.position.set(2, 0, 0);
    scene.add(meshShape);
    controlsShape.attach(meshShape);

    const rayOriginGeometry = new THREE.SphereGeometry(0.2, 16, 16);
    const rayOriginMaterial = new THREE.MeshBasicMaterial({ color: 0xff00ff });
    const rayOriginMesh = new THREE.Mesh(rayOriginGeometry, rayOriginMaterial);
    rayOriginMesh.position.set(-3, 0, 0);
    scene.add(rayOriginMesh);
    controlsRayOrigin.attach(rayOriginMesh);

    const rayEndpointGeometry = new THREE.SphereGeometry(0.2, 16, 16);
    const rayEndpointMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const rayEndpointMesh = new THREE.Mesh(rayEndpointGeometry, rayEndpointMaterial);
    rayEndpointMesh.position.set(7, 0, 0);
    scene.add(rayEndpointMesh);
    controlsRayEndpoint.attach(rayEndpointMesh);

    return {
        collisionShape,
        shapeHelper,
        meshShape,
        rayOriginMesh,
        rayEndpointMesh,
        lastHadHits: false,
    };
}

type ExampleState = ReturnType<typeof init>;

function dispose(state: ExampleState) {
    scene.remove(state.meshShape);
    scene.remove(state.rayOriginMesh);
    scene.remove(state.rayEndpointMesh);
    controlsShape.detach();
    controlsRayOrigin.detach();
    controlsRayEndpoint.detach();
    state.shapeHelper.dispose();
    for (const mesh of hitPointMeshes) {
        scene.remove(mesh);
    }
    for (const arrow of normalArrows) {
        scene.remove(arrow);
    }
    hitPointMeshes.length = 0;
    normalArrows.length = 0;
}

function setup() {
    if (state) dispose(state);
    state = init();
    update(state);
}

function getOrCreateHitPointMesh(index: number): THREE.Mesh {
    if (!hitPointMeshes[index]) {
        const material = new THREE.MeshBasicMaterial({ color: 0xffaa00 });
        const mesh = new THREE.Mesh(hitPointGeometry, material);
        hitPointMeshes[index] = mesh;
        scene.add(mesh);
    }
    return hitPointMeshes[index];
}

function getOrCreateNormalArrow(index: number): THREE.ArrowHelper {
    if (!normalArrows[index]) {
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1.0, 0x00ff00, 0.3, 0.2);
        normalArrows[index] = arrow;
        scene.add(arrow);
    }
    return normalArrows[index];
}

function updateRayVisualization(state: ExampleState) {
    const origin = state.rayOriginMesh.position;
    const endpoint = state.rayEndpointMesh.position;

    const points = [new THREE.Vector3(origin.x, origin.y, origin.z), new THREE.Vector3(endpoint.x, endpoint.y, endpoint.z)];
    rayLineGeometry.setFromPoints(points);
}

type CastRayHitResult = {
    fraction: number;
    normal: [number, number, number];
    subShapeId: number;
};

function performRaycast(state: ExampleState): CastRayHitResult[] {
    const origin = state.rayOriginMesh.position;
    const endpoint = state.rayEndpointMesh.position;

    const direction = vec3.create();
    vec3.subtract(direction, [endpoint.x, endpoint.y, endpoint.z], [origin.x, origin.y, origin.z]);
    const rayLength = vec3.length(direction);
    vec3.normalize(direction, direction);

    const shapePos = vec3.fromValues(state.meshShape.position.x, state.meshShape.position.y, state.meshShape.position.z);
    const shapeQuat = quat.fromValues(
        state.meshShape.quaternion.x,
        state.meshShape.quaternion.y,
        state.meshShape.quaternion.z,
        state.meshShape.quaternion.w,
    );
    const shapeScale = vec3.fromValues(1, 1, 1);

    const ray = {
        origin: vec3.fromValues(origin.x, origin.y, origin.z),
        direction,
        length: rayLength,
    };

    const hits: CastRayHitResult[] = [];

    if (config.collectorMode === 'closest') {
        const closestCollector = createClosestCastRayCollector();
        const options = createDefaultCastRaySettings();
        options.collideWithBackfaces = true;
        castRayVsShape(
            closestCollector,
            options,
            ray,
            state.collisionShape,
            EMPTY_SUB_SHAPE_ID,
            0,
            shapePos[0],
            shapePos[1],
            shapePos[2],
            shapeQuat[0],
            shapeQuat[1],
            shapeQuat[2],
            shapeQuat[3],
            shapeScale[0],
            shapeScale[1],
            shapeScale[2],
        );

        const hit = closestCollector.hit;
        if (hit.status === CastRayStatus.COLLIDING) {
            const hitPoint = vec3.create();
            const normalizedDistance = hit.fraction * rayLength;
            vec3.scaleAndAdd(hitPoint, ray.origin, direction, normalizedDistance);

            const localHitPoint = vec3.create();
            vec3.subtract(localHitPoint, hitPoint, shapePos);
            const invQuat = quat.conjugate(quat.create(), shapeQuat);
            vec3.transformQuat(localHitPoint, localHitPoint, invQuat);

            const localNormal = vec3.create();
            getShapeSurfaceNormal(localNormal, state.collisionShape, localHitPoint, hit.subShapeId);

            vec3.transformQuat(localNormal, localNormal, shapeQuat);

            hits.push({
                fraction: hit.fraction,
                normal: [localNormal[0], localNormal[1], localNormal[2]],
                subShapeId: hit.subShapeId,
            });
        }
    } else if (config.collectorMode === 'any') {
        const anyCollector = createAnyCastRayCollector();
        castRayVsShape(
            anyCollector,
            { collideWithBackfaces: true, treatConvexAsSolid: true },
            ray,
            state.collisionShape,
            EMPTY_SUB_SHAPE_ID,
            0,
            shapePos[0],
            shapePos[1],
            shapePos[2],
            shapeQuat[0],
            shapeQuat[1],
            shapeQuat[2],
            shapeQuat[3],
            shapeScale[0],
            shapeScale[1],
            shapeScale[2],
        );

        const hit = anyCollector.hit;
        if (hit.status === CastRayStatus.COLLIDING) {
            const hitPoint = vec3.create();
            const normalizedDistance = hit.fraction * rayLength;
            vec3.scaleAndAdd(hitPoint, ray.origin, direction, normalizedDistance);

            const localHitPoint = vec3.create();
            vec3.subtract(localHitPoint, hitPoint, shapePos);
            const invQuat = quat.conjugate(quat.create(), shapeQuat);
            vec3.transformQuat(localHitPoint, localHitPoint, invQuat);

            const localNormal = vec3.create();
            getShapeSurfaceNormal(localNormal, state.collisionShape, localHitPoint, hit.subShapeId);

            vec3.transformQuat(localNormal, localNormal, shapeQuat);

            hits.push({
                fraction: hit.fraction,
                normal: [localNormal[0], localNormal[1], localNormal[2]],
                subShapeId: hit.subShapeId,
            });
        }
    } else {
        const allCollector = createAllCastRayCollector();
        castRayVsShape(
            allCollector,
            { collideWithBackfaces: true, treatConvexAsSolid: true },
            ray,
            state.collisionShape,
            EMPTY_SUB_SHAPE_ID,
            0,
            shapePos[0],
            shapePos[1],
            shapePos[2],
            shapeQuat[0],
            shapeQuat[1],
            shapeQuat[2],
            shapeQuat[3],
            shapeScale[0],
            shapeScale[1],
            shapeScale[2],
        );

        for (const hit of allCollector.hits) {
            if (hit.status === CastRayStatus.COLLIDING) {
                const hitPoint = vec3.create();
                const normalizedDistance = hit.fraction * rayLength;
                vec3.scaleAndAdd(hitPoint, ray.origin, direction, normalizedDistance);

                const localHitPoint = vec3.create();
                vec3.subtract(localHitPoint, hitPoint, shapePos);
                const invQuat = quat.conjugate(quat.create(), shapeQuat);
                vec3.transformQuat(localHitPoint, localHitPoint, invQuat);

                const localNormal = vec3.create();
                getShapeSurfaceNormal(localNormal, state.collisionShape, localHitPoint, hit.subShapeId);

                vec3.transformQuat(localNormal, localNormal, shapeQuat);

                hits.push({
                    fraction: hit.fraction,
                    normal: [localNormal[0], localNormal[1], localNormal[2]],
                    subShapeId: hit.subShapeId,
                });
            }
        }
        hits.sort((a, b) => a.fraction - b.fraction);
    }

    return hits;
}

function update(state: ExampleState) {
    updateRayVisualization(state);

    const hits = performRaycast(state);

    const origin = state.rayOriginMesh.position;
    const endpoint = state.rayEndpointMesh.position;

    const direction = vec3.create();
    vec3.subtract(direction, [endpoint.x, endpoint.y, endpoint.z], [origin.x, origin.y, origin.z]);
    const rayLength = vec3.length(direction);
    vec3.normalize(direction, direction);

    for (let i = 0; i < hitPointMeshes.length; i++) {
        hitPointMeshes[i].visible = false;
    }
    for (let i = 0; i < normalArrows.length; i++) {
        normalArrows[i].visible = false;
    }

    for (let i = 0; i < hits.length; i++) {
        const hitMesh = getOrCreateHitPointMesh(i);
        const hitPos = vec3.create();
        const normalizedDistance = hits[i].fraction * rayLength;
        vec3.scaleAndAdd(hitPos, [origin.x, origin.y, origin.z], direction, normalizedDistance);
        hitMesh.position.set(hitPos[0], hitPos[1], hitPos[2]);
        hitMesh.visible = true;

        const arrow = getOrCreateNormalArrow(i);
        arrow.position.set(hitPos[0], hitPos[1], hitPos[2]);
        const normalDir = new THREE.Vector3(hits[i].normal[0], hits[i].normal[1], hits[i].normal[2]);
        arrow.setDirection(normalDir);
        arrow.visible = true;
    }

    if (hits.length > 0) {
        if (!state.lastHadHits) {
            materialShape.color.setHex(0x00ff00);
            state.lastHadHits = true;
        }
    } else {
        if (state.lastHadHits) {
            materialShape.color.setHex(0x4488ff);
            state.lastHadHits = false;
        }
    }

    let infoHTML = `<b>Ray Info</b><br><br>`;
    infoHTML += `<b>Origin:</b><br>`;
    infoHTML += `X: ${origin.x.toFixed(3)}<br>`;
    infoHTML += `Y: ${origin.y.toFixed(3)}<br>`;
    infoHTML += `Z: ${origin.z.toFixed(3)}<br><br>`;

    infoHTML += `<b>Direction:</b><br>`;
    infoHTML += `X: ${direction[0].toFixed(3)}<br>`;
    infoHTML += `Y: ${direction[1].toFixed(3)}<br>`;
    infoHTML += `Z: ${direction[2].toFixed(3)}<br><br>`;

    infoHTML += `<b>Length:</b> ${rayLength.toFixed(3)}<br><br>`;

    if (hits.length > 0) {
        infoHTML += `<b>Hits: ${hits.length}</b><br>`;
        for (let i = 0; i < Math.min(hits.length, 5); i++) {
            const hitPos = vec3.create();
            const normalizedDistance = hits[i].fraction * rayLength;
            vec3.scaleAndAdd(hitPos, [origin.x, origin.y, origin.z], direction, normalizedDistance);
            infoHTML += `<b>Hit ${i + 1}:</b><br>`;
            infoHTML += `  Pos: (${hitPos[0].toFixed(2)}, ${hitPos[1].toFixed(2)}, ${hitPos[2].toFixed(2)})<br>`;
            infoHTML += `  Normal: (${hits[i].normal[0].toFixed(3)}, ${hits[i].normal[1].toFixed(3)}, ${hits[i].normal[2].toFixed(3)})<br>`;
            infoHTML += `  SubShapeId: ${hits[i].subShapeId}<br>`;
        }
        if (hits.length > 5) {
            infoHTML += `... and ${hits.length - 5} more<br>`;
        }
    } else {
        infoHTML += `<b>No Hits</b><br>`;
    }

    infoPanel.innerHTML = infoHTML;
}

setup();

controlsShape.addEventListener('change', () => update(state));
controlsRayOrigin.addEventListener('change', () => update(state));
controlsRayEndpoint.addEventListener('change', () => update(state));

function animate() {
    requestAnimationFrame(animate);

    const anyControlsDragging = controlsShape.dragging || controlsRayOrigin.dragging || controlsRayEndpoint.dragging;
    orbitControls.enabled = !anyControlsDragging;
    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
