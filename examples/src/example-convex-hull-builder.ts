import GUI from 'lil-gui';
import type { Vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { convexHullBuilder } from 'crashcat';

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

/* info panel */

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
infoPanel.style.minWidth = '300px';
document.body.appendChild(infoPanel);

/* visualization groups */

const inputPointsGroup = new THREE.Group();
scene.add(inputPointsGroup);

const hullWireframeGroup = new THREE.Group();
scene.add(hullWireframeGroup);

const hullSolidGroup = new THREE.Group();
scene.add(hullSolidGroup);

const centerOfMassGroup = new THREE.Group();
scene.add(centerOfMassGroup);

/* config and GUI */

const config = {
    shape: 'Cube',
    showInputPoints: true,
    showHullWireframe: true,
    showHullSolid: true,
    showCenterOfMass: true,
    solidOpacity: 0.3,
    maxVertices: 1000,
    tolerance: 1e-4,
};

const gui = new GUI();
const shapeController = gui.add(config, 'shape', [
    'Cube',
    'Icosphere',
    'Random Points (Sphere)',
    'Random Points (Cube)',
    'Bunny (GLTF)',
    'Cat (GLTF)',
    'Suzi (GLTF)',
]);
shapeController.onChange(() => updateVisualization());

gui.add(config, 'showInputPoints').onChange(() => updateVisualization());
gui.add(config, 'showHullWireframe').onChange(() => updateVisualization());
gui.add(config, 'showHullSolid').onChange(() => updateVisualization());
gui.add(config, 'showCenterOfMass').onChange(() => updateVisualization());
gui.add(config, 'solidOpacity', 0, 1, 0.05).onChange(() => updateVisualization());
gui.add(config, 'maxVertices', 10, 5000, 1).onChange(() => updateVisualization());
gui.add(config, 'tolerance', 1e-6, 1e-2).onChange(() => updateVisualization());

/* shape generators */

function generateCube(): Vec3[] {
    const points: Vec3[] = [];
    for (let x = -1; x <= 1; x += 0.5) {
        for (let y = -1; y <= 1; y += 0.5) {
            for (let z = -1; z <= 1; z += 0.5) {
                points.push([x, y, z]);
            }
        }
    }
    return points;
}

function generateIcosphere(subdivisions = 2): Vec3[] {
    const geometry = new THREE.IcosahedronGeometry(2, subdivisions);
    const positions = geometry.getAttribute('position');
    const points: Vec3[] = [];
    for (let i = 0; i < positions.count; i++) {
        points.push([positions.getX(i), positions.getY(i), positions.getZ(i)]);
    }
    return points;
}

function generateRandomPointsInSphere(count = 100, radius = 2): Vec3[] {
    const points: Vec3[] = [];
    for (let i = 0; i < count; i++) {
        // Random point in sphere using rejection sampling
        let x: number, y: number, z: number, distSq: number;
        do {
            x = (Math.random() - 0.5) * 2 * radius;
            y = (Math.random() - 0.5) * 2 * radius;
            z = (Math.random() - 0.5) * 2 * radius;
            distSq = x * x + y * y + z * z;
        } while (distSq > radius * radius);
        points.push([x, y, z]);
    }
    return points;
}

function generateRandomPointsInCube(count = 100, size = 2): Vec3[] {
    const points: Vec3[] = [];
    for (let i = 0; i < count; i++) {
        points.push([(Math.random() - 0.5) * 2 * size, (Math.random() - 0.5) * 2 * size, (Math.random() - 0.5) * 2 * size]);
    }
    return points;
}

async function loadGLTFPoints(path: string): Promise<Vec3[]> {
    const gltfLoader = new GLTFLoader();
    const gltf = await gltfLoader.loadAsync(path);

    // Find the first mesh in the scene hierarchy
    let mesh: THREE.Mesh | undefined;

    gltf.scene.traverse((object) => {
        if (!mesh && object instanceof THREE.Mesh) {
            mesh = object;
        }
    });

    if (!mesh) {
        throw new Error('No mesh found in GLTF file');
    }

    const geometry = mesh.geometry as THREE.BufferGeometry;
    const positions = geometry.getAttribute('position');

    const points: Vec3[] = [];

    for (let i = 0; i < positions.count; i++) {
        points.push([positions.getX(i), positions.getY(i), positions.getZ(i)]);
    }

    return points;
}

/* visualization update */

async function updateVisualization() {
    // Clear previous visualization
    inputPointsGroup.clear();
    hullWireframeGroup.clear();
    hullSolidGroup.clear();
    centerOfMassGroup.clear();

    // Get input points based on selected shape
    let inputPoints: Vec3[];

    try {
        switch (config.shape) {
            case 'Cube':
                inputPoints = generateCube();
                break;
            case 'Icosphere':
                inputPoints = generateIcosphere();
                break;
            case 'Random Points (Sphere)':
                inputPoints = generateRandomPointsInSphere();
                break;
            case 'Random Points (Cube)':
                inputPoints = generateRandomPointsInCube();
                break;
            case 'Bunny (GLTF)':
                inputPoints = await loadGLTFPoints('./models/bunny.glb');
                break;
            case 'Cat (GLTF)':
                inputPoints = await loadGLTFPoints('./models/cat.gltf');
                break;
            case 'Suzi (GLTF)':
                inputPoints = await loadGLTFPoints('./models/suzi.glb');
                break;
            default:
                inputPoints = generateCube();
        }
    } catch (error) {
        console.error('Error loading shape:', error);
        inputPoints = generateCube();
        infoPanel.innerHTML = `<span style="color: #ff4444;">Error loading shape</span>`;
        return;
    }

    // Show input points if enabled
    if (config.showInputPoints) {
        const pointsMaterial = new THREE.PointsMaterial({
            color: 0x00ffff,
            size: 0.05,
            sizeAttenuation: true,
        });
        const pointsGeometry = new THREE.BufferGeometry();
        const positions = new Float32Array(inputPoints.length * 3);

        for (let i = 0; i < inputPoints.length; i++) {
            positions[i * 3] = inputPoints[i][0];
            positions[i * 3 + 1] = inputPoints[i][1];
            positions[i * 3 + 2] = inputPoints[i][2];
        }

        pointsGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        const pointsMesh = new THREE.Points(pointsGeometry, pointsMaterial);
        inputPointsGroup.add(pointsMesh);
    }

    // Build convex hull
    const builder = convexHullBuilder.create(inputPoints);
    const result = convexHullBuilder.initialize(builder, config.maxVertices, config.tolerance);

    // Update info panel
    let infoHTML = `<strong>Input Shape:</strong> ${config.shape}<br>`;
    infoHTML += `<strong>Input Points:</strong> ${inputPoints.length}<br>`;
    infoHTML += `<strong>Max Vertices:</strong> ${config.maxVertices}<br>`;
    infoHTML += `<strong>Tolerance:</strong> ${config.tolerance.toExponential(2)}<br><br>`;

    if (result.result !== convexHullBuilder.Result.Success) {
        infoHTML += `<span style="color: #ff4444;"><strong>Build Result:</strong> ${convexHullBuilder.Result[result.result]}</span><br>`;
        infoHTML += `<strong>Error:</strong> ${result.error}`;
        infoPanel.innerHTML = infoHTML;
        return;
    }

    infoHTML += `<span style="color: #44ff44;"><strong>Build Result:</strong> Success</span><br>`;
    const numVertices = convexHullBuilder.getNumVerticesUsed(builder);
    const faces = builder.faces;
    const numFaces = faces.filter((f) => !f.removed).length;
    
    // Get center of mass and volume
    const { centerOfMass, volume } = convexHullBuilder.getCenterOfMassAndVolume(builder);
    
    infoHTML += `<strong>Hull Vertices:</strong> ${numVertices}<br>`;
    infoHTML += `<strong>Hull Faces:</strong> ${numFaces}<br>`;
    infoHTML += `<strong>Volume:</strong> ${volume.toFixed(4)}<br>`;
    infoHTML += `<strong>Center of Mass:</strong> (${centerOfMass[0].toFixed(2)}, ${centerOfMass[1].toFixed(2)}, ${centerOfMass[2].toFixed(2)})<br>`;

    infoPanel.innerHTML = infoHTML;

    // Create hull mesh geometry
    const hullGeometry = new THREE.BufferGeometry();
    const hullPositions: number[] = [];
    const hullIndices: number[] = [];

    // Collect all face triangles - reuse faces variable from above
    for (let faceIdx = 0; faceIdx < faces.length; faceIdx++) {
        const face = faces[faceIdx];
        if (!face || face.removed) continue;

        // Get face vertices by following edges
        const faceVertices: number[] = [];
        let edge = face.firstEdge;
        const startEdge = edge;

        if (!edge) continue;

        do {
            faceVertices.push(edge.mStartIdx);
            edge = edge.mNextEdge!;
        } while (edge !== startEdge && faceVertices.length < 100); // Safety limit

        // Triangulate face (simple fan triangulation)
        for (let i = 1; i < faceVertices.length - 1; i++) {
            hullIndices.push(faceVertices[0], faceVertices[i], faceVertices[i + 1]);
        }
    }

    // Add vertex positions
    for (let i = 0; i < builder.positions.length; i++) {
        const v = builder.positions[i];
        hullPositions.push(v[0], v[1], v[2]);
    }

    hullGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(hullPositions), 3));
    hullGeometry.setIndex(hullIndices);
    hullGeometry.computeVertexNormals();

    // Show wireframe if enabled
    if (config.showHullWireframe) {
        const wireframeMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00 });
        const wireframe = new THREE.LineSegments(new THREE.EdgesGeometry(hullGeometry), wireframeMaterial);
        hullWireframeGroup.add(wireframe);
    }

    // Show solid hull if enabled
    if (config.showHullSolid) {
        const solidMaterial = new THREE.MeshStandardMaterial({
            color: 0x44ff44,
            transparent: true,
            opacity: config.solidOpacity,
            side: THREE.DoubleSide,
            depthWrite: false, // Prevent z-fighting with wireframe
        });
        const solidMesh = new THREE.Mesh(hullGeometry, solidMaterial);
        hullSolidGroup.add(solidMesh);
    }

    // Show center of mass if enabled
    if (config.showCenterOfMass) {
        // Create a sphere at the center of mass
        const comGeometry = new THREE.SphereGeometry(0.1, 16, 16);
        const comMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const comSphere = new THREE.Mesh(comGeometry, comMaterial);
        comSphere.position.set(centerOfMass[0], centerOfMass[1], centerOfMass[2]);
        centerOfMassGroup.add(comSphere);

        // Add coordinate axes at the center of mass
        const axesHelper = new THREE.AxesHelper(0.5);
        axesHelper.position.set(centerOfMass[0], centerOfMass[1], centerOfMass[2]);
        centerOfMassGroup.add(axesHelper);
    }
}

/* animation loop */

function animate() {
    requestAnimationFrame(animate);
    orbitControls.update();
    renderer.render(scene, camera);
}

/* initialize */

updateVisualization();
animate();
