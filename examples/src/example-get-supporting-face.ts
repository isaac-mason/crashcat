import GUI from 'lil-gui';
import { quat, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { box, getShapeSupportingFace, convexHull, registerAllShapes, type Shape, scaled, transformed } from 'crashcat';
import { createShapeHelper } from 'crashcat/three';

registerAllShapes();

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(4, 4, 4);
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

// Create mesh to visualize the shape
const meshMaterial = new THREE.MeshStandardMaterial({ color: 0x00ff00, wireframe: false });

let currentObject: THREE.Object3D;
let currentShape: Shape;
let shapeIndex = 0;

const shapes: { name: string; shape: Shape; object: THREE.Object3D }[] = [];

// Box
const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
const boxHelper = createShapeHelper(boxShape, { material: meshMaterial });
const boxMesh = boxHelper.object;
shapes.push({ name: 'Box', shape: boxShape, object: boxMesh });

// Scaled Box
const scaledBoxShape = scaled.create({
    shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
    scale: vec3.fromValues(2, 1, 1),
});
const scaledBoxHelper = createShapeHelper(scaledBoxShape, { material: meshMaterial });
const scaledBoxMesh = scaledBoxHelper.object;
shapes.push({
    name: 'Scaled Box',
    shape: scaledBoxShape,
    object: scaledBoxMesh,
});

// Rotated Box
const rotatedBoxShape = transformed.create({
    shape: box.create({ halfExtents: vec3.fromValues(1, 1, 1) }),
    position: vec3.fromValues(0, 0, 0),
    quaternion: quat.fromValues(0, 0, Math.sin(Math.PI / 8), Math.cos(Math.PI / 8)),
});
const rotatedBoxHelper = createShapeHelper(rotatedBoxShape, { material: meshMaterial });
const rotatedBoxMesh = rotatedBoxHelper.object;
shapes.push({
    name: 'Rotated Box',
    shape: rotatedBoxShape,
    object: rotatedBoxMesh,
});

// Convex Hull - octahedron shape
const octahedronPoints = [
    0,
    1.5,
    0, // top
    0,
    -1.5,
    0, // bottom
    1,
    0,
    0, // +X
    -1,
    0,
    0, // -X
    0,
    0,
    1, // +Z
    0,
    0,
    -1, // -Z
];
const convexHullShape = convexHull.create({ positions: octahedronPoints, convexRadius: 0.0 });
const convexHullHelper = createShapeHelper(convexHullShape, { material: meshMaterial });
const convexHullMesh = convexHullHelper.object;
shapes.push({
    name: 'Convex Hull (Octahedron)',
    shape: convexHullShape,
    object: convexHullMesh,
});

currentObject = shapes[0].object;
currentShape = shapes[0].shape;
scene.add(currentObject);

// Direction control point - draggable sphere that controls the direction
const directionPointGeometry = new THREE.SphereGeometry(0.2, 16, 16);
const directionPointMaterial = new THREE.MeshBasicMaterial({ color: 0xffff00 });
const directionPoint = new THREE.Mesh(directionPointGeometry, directionPointMaterial);
directionPoint.position.set(1, 0, 0);
scene.add(directionPoint);

// Transform controls for direction point
const transformControls = new TransformControls(camera, renderer.domElement);
transformControls.attach(directionPoint);
transformControls.setMode('translate');
scene.add(transformControls.getHelper());

// Direction visualization - arrow from origin to gizmo showing the query direction
const directionArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 2, 0x00ffff);
scene.add(directionArrow);

// Supporting face visualization - line segments
const faceLines: THREE.Line[] = [];
const faceVertices: THREE.Mesh[] = [];

// Create reusable vertex indicators (up to 8 for convex hulls)
for (let i = 0; i < 8; i++) {
    const vertexGeometry = new THREE.SphereGeometry(0.15, 16, 16);
    const vertexMaterial = new THREE.MeshBasicMaterial({ color: 0xff00ff });
    const vertex = new THREE.Mesh(vertexGeometry, vertexMaterial);
    vertex.visible = false;
    scene.add(vertex);
    faceVertices.push(vertex);
}

// Cycle transform modes on right click
const modes: ('translate' | 'rotate')[] = ['translate', 'rotate'];
let modeIndex = 0;

const raycaster = new THREE.Raycaster();
const mousePos = new THREE.Vector2();

renderer.domElement.addEventListener('contextmenu', (event) => {
    event.preventDefault();

    // Update mouse position
    mousePos.x = (event.clientX / window.innerWidth) * 2 - 1;
    mousePos.y = -(event.clientY / window.innerHeight) * 2 + 1;

    // Raycast
    raycaster.setFromCamera(mousePos, camera);
    const intersects = raycaster.intersectObjects([directionPoint]);

    if (intersects.length > 0) {
        modeIndex = (modeIndex + 1) % modes.length;
        transformControls.setMode(modes[modeIndex]);
    }
});

function updateDirection() {
    // get direction from origin to direction point
    const direction = vec3.create();
    vec3.set(direction, directionPoint.position.x, directionPoint.position.y, directionPoint.position.z);

    // normalize direction
    const length = vec3.length(direction);
    const normalizedDir = vec3.create();
    if (length > 0) {
        vec3.scale(normalizedDir, direction, 1 / length);
    }

    // update direction arrow to show the query direction
    directionArrow.setDirection(new THREE.Vector3(normalizedDir[0], normalizedDir[1], normalizedDir[2]));
    if (length > 0) {
        directionArrow.setLength(Math.min(length, 3));
    }

    // get supporting face with this direction
    // shapes are at origin with identity transforms
    const face = { vertices: [], numVertices: 0 };
    const position = vec3.fromValues(0, 0, 0);
    const quaternion = quat.fromValues(0, 0, 0, 1);
    const scale = vec3.fromValues(1, 1, 1);
    getShapeSupportingFace(face, currentShape, 0, normalizedDir, position, quaternion, scale);

    // visualize face vertices
    for (let i = 0; i < 8; i++) {
        faceVertices[i].visible = false;
    }

    for (let i = 0; i < face.numVertices; i++) {
        const vx = face.vertices[i * 3];
        const vy = face.vertices[i * 3 + 1];
        const vz = face.vertices[i * 3 + 2];

        faceVertices[i].position.set(vx, vy, vz);
        faceVertices[i].visible = true;
    }

    // Draw face outline
    for (const line of faceLines) {
        scene.remove(line);
    }
    faceLines.length = 0;

    if (face.numVertices >= 2) {
        for (let i = 0; i < face.numVertices; i++) {
            const nextI = (i + 1) % face.numVertices;
            const v0 = vec3.fromValues(face.vertices[i * 3], face.vertices[i * 3 + 1], face.vertices[i * 3 + 2]);
            const v1 = vec3.fromValues(face.vertices[nextI * 3], face.vertices[nextI * 3 + 1], face.vertices[nextI * 3 + 2]);

            const lineGeometry = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(v0[0], v0[1], v0[2]),
                new THREE.Vector3(v1[0], v1[1], v1[2]),
            ]);
            const lineMaterial = new THREE.LineBasicMaterial({ color: 0xff00ff, linewidth: 3 });
            const line = new THREE.Line(lineGeometry, lineMaterial);
            scene.add(line);
            faceLines.push(line);
        }
    }

    updateInfoPanel(face, normalizedDir);
}

// Info panel
const infoPanel = document.createElement('div');
infoPanel.style.position = 'absolute';
infoPanel.style.top = '10px';
infoPanel.style.left = '10px';
infoPanel.style.color = 'white';
infoPanel.style.fontFamily = 'monospace';
infoPanel.style.fontSize = '14px';
infoPanel.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
infoPanel.style.padding = '15px';
infoPanel.style.borderRadius = '5px';
infoPanel.style.minWidth = '300px';
document.body.appendChild(infoPanel);

function updateInfoPanel(face: { vertices: number[]; numVertices: number }, direction: Vec3) {
    let html = `<b>computeSupportingFace</b><br><br>`;
    html += `<b>Shape:</b> ${shapes[shapeIndex].name}<br>`;
    html += `<b>Query Direction:</b> (${direction[0].toFixed(3)}, ${direction[1].toFixed(3)}, ${direction[2].toFixed(3)})<br>`;
    html += `<b>Gizmo Position:</b> (${directionPoint.position.x.toFixed(3)}, ${directionPoint.position.y.toFixed(3)}, ${directionPoint.position.z.toFixed(3)})<br>`;
    html += `<b>Face Vertices:</b> ${face.numVertices}<br><br>`;
    html += `<b>Note:</b> Returns the face perpendicular to the query direction<br><br>`;

    if (face.numVertices > 0) {
        html += `<b>Vertex Positions:</b><br>`;
        for (let i = 0; i < face.numVertices; i++) {
            const x = face.vertices[i * 3];
            const y = face.vertices[i * 3 + 1];
            const z = face.vertices[i * 3 + 2];
            html += `[${i}]: (${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)})<br>`;
        }
    }

    infoPanel.innerHTML = html;
}

// lil-gui controls
const gui = new GUI();
gui.add({ shape: 0 }, 'shape', { Box: 0, 'Scaled Box': 1, 'Rotated Box': 2, 'Convex Hull': 3 }).onChange((index: number) => {
    shapeIndex = index;
    scene.remove(currentObject);
    currentObject = shapes[index].object;
    currentShape = shapes[index].shape;
    scene.add(currentObject);
    updateDirection();
});

// Animation loop
function animate() {
    requestAnimationFrame(animate);

    updateDirection();

    orbitControls.enabled = !transformControls.dragging;
    orbitControls.update();
    renderer.render(scene, camera);
}

// Handle window resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

animate();
